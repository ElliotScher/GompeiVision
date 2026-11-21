#include "io/DatasetRecorder.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>

#ifndef GOMPEI_APRILTAG_VERSION
#define GOMPEI_APRILTAG_VERSION "unknown"
#endif

namespace {

using nlohmann::json;

json pose_json(const frc::Pose3d& p) {
  const auto q = p.Rotation().GetQuaternion();
  return {{"x", p.X().value()}, {"y", p.Y().value()}, {"z", p.Z().value()},
          {"qw", q.W()},        {"qx", q.X()},        {"qy", q.Y()},
          {"qz", q.Z()}};
}

json mat_json(const cv::Mat& m) {
  json rows = json::array();
  cv::Mat d;
  m.convertTo(d, CV_64F);
  for (int r = 0; r < d.rows; ++r) {
    json row = json::array();
    for (int c = 0; c < d.cols; ++c) row.push_back(d.at<double>(r, c));
    rows.push_back(row);
  }
  return rows;
}

std::string utc_stamp() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof buf, "%Y%m%dT%H%M%SZ", &tm);
  return buf;
}

bool write_pgm(const std::filesystem::path& path, const cv::Mat& gray) {
  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  std::fprintf(f, "P5\n%d %d\n255\n", gray.cols, gray.rows);
  bool ok = true;
  for (int y = 0; y < gray.rows && ok; ++y)
    ok = std::fwrite(gray.ptr(y), 1, static_cast<size_t>(gray.cols), f) ==
         static_cast<size_t>(gray.cols);
  return std::fclose(f) == 0 && ok;
}

}  // namespace

std::unique_ptr<DatasetRecorder> DatasetRecorder::fromEnvironment(
    const std::string& hardware_id, const std::string& role,
    const apriltag_detector_t* detector, double tag_size_m,
    const std::map<int, frc::Pose3d>& field) {
  const char* root = std::getenv("GOMPEI_RECORD_DIR");
  if (root == nullptr || *root == '\0') return nullptr;
  int every = 1;
  if (const char* e = std::getenv("GOMPEI_RECORD_EVERY")) every = std::max(1, std::atoi(e));

  const std::filesystem::path dir =
      std::filesystem::path(root) / (hardware_id + "_" + role + "_" + utc_stamp());
  std::error_code ec;
  std::filesystem::create_directories(dir / "frames", ec);
  if (ec) {
    std::cerr << "[" << role << "] DatasetRecorder: cannot create " << dir
              << ": " << ec.message() << std::endl;
    return nullptr;
  }

  json session = {
      {"format", "gompeivision-dataset"},
      {"version", 1},
      {"hardware_id", hardware_id},
      {"role", role},
      {"apriltag_version", GOMPEI_APRILTAG_VERSION},
      {"tag_family", "tag36h11"},
      {"tag_size_m", tag_size_m},
      {"record_every", every},
      {"timestamp_units", "microseconds, nt::Now()"},
      {"image", "grayscale PGM, as passed to apriltag_detector_detect"},
  };
  if (detector != nullptr) {
    session["detector"] = {
        {"quad_decimate", detector->quad_decimate},
        {"quad_sigma", detector->quad_sigma},
        {"nthreads", detector->nthreads},
        {"refine_edges", detector->refine_edges},
        {"decode_sharpening", detector->decode_sharpening},
        {"min_cluster_pixels", detector->qtp.min_cluster_pixels},
        {"max_nmaxima", detector->qtp.max_nmaxima},
        {"critical_rad", detector->qtp.critical_rad},
        {"max_line_fit_mse", detector->qtp.max_line_fit_mse},
        {"min_white_black_diff", detector->qtp.min_white_black_diff},
        {"deglitch", detector->qtp.deglitch},
    };
  }
  json tags = json::object();
  for (const auto& [id, pose] : field) tags[std::to_string(id)] = pose_json(pose);
  session["field"] = tags;

  std::ofstream(dir / "session.json") << session.dump(2) << "\n";
  std::cout << "[" << role << "] DatasetRecorder: recording 1 in " << every
            << " frames to " << dir << std::endl;
  return std::unique_ptr<DatasetRecorder>(new DatasetRecorder(dir, every));
}

DatasetRecorder::DatasetRecorder(std::filesystem::path dir, int every)
    : m_dir(std::move(dir)), m_every(every), m_frames(m_dir / "frames.jsonl") {
  m_writer = std::thread(&DatasetRecorder::writer_loop, this);
}

DatasetRecorder::~DatasetRecorder() {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stop = true;
  }
  m_cv.notify_all();
  if (m_writer.joinable()) m_writer.join();
  std::cout << "DatasetRecorder: " << m_queued << " frames recorded to " << m_dir
            << ", " << m_dropped << " dropped (writer behind)"
            << (m_disk_full ? ", stopped early: disk nearly full" : "")
            << std::endl;
}

void DatasetRecorder::record(const QueuedFrame& frame,
                             const FiducialImageObservation& observation,
                             const AprilTagResult* result,
                             const cv::Mat& camera_matrix,
                             const cv::Mat& dist_coeffs) {
  if (m_seen++ % static_cast<uint64_t>(m_every) != 0 || frame.frame.empty()) return;
  Item item;
  item.frame = frame.frame;  // reference, not a copy
  item.timestamp = frame.timestamp;
  item.processed_index = m_seen - 1;
  item.observation = observation;
  if (result != nullptr) item.result = *result;
  if (!camera_matrix.empty()) {
    item.camera_matrix = camera_matrix.clone();
    item.dist_coeffs = dist_coeffs.clone();
  }
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_disk_full) return;
    if (m_queue.size() >= kQueueCapacity) {
      ++m_dropped;
      return;
    }
    m_queue.push_back(std::move(item));
    ++m_queued;
  }
  m_cv.notify_one();
}

namespace {

// The frames.jsonl entry for one frame, with its image file name.
json frame_json(const std::string& file, int64_t timestamp, uint64_t processed_index,
                const cv::Mat& gray, const FiducialImageObservation& observation,
                const std::optional<AprilTagResult>& result) {
  json j = {{"file", file},
            {"timestamp", timestamp},
            {"processed_index", processed_index},
            {"width", gray.cols},
            {"height", gray.rows}};
  json dets = json::array();
  for (size_t i = 0; i < observation.tag_ids.size(); ++i)
    dets.push_back({{"id", observation.tag_ids[i]},
                    {"corners", i < observation.corners_pixels.size()
                                    ? json(observation.corners_pixels[i])
                                    : json::array()}});
  j["detections"] = dets;
  if (result) {
    json singles = json::array();
    for (const auto& s : result->single_tag_poses) {
      json o = {{"id", s.tag_id}, {"pose_0", pose_json(s.pose_0)},
                {"error_0", s.error_0}, {"distance_m", s.distance.value()}};
      if (s.pose_1) o["pose_1"] = pose_json(*s.pose_1);
      if (s.error_1) o["error_1"] = *s.error_1;
      singles.push_back(o);
    }
    j["single_tag_poses"] = singles;
    const auto& m = result->multi_tag_pose;
    if (!m.tag_ids.empty()) {
      json o = {{"tags", m.tag_ids}, {"pose_0", pose_json(m.pose_0)},
                {"error_0", m.error_0}};
      if (m.pose_1) o["pose_1"] = pose_json(*m.pose_1);
      if (m.error_1) o["error_1"] = *m.error_1;
      j["camera_pose"] = o;
    }
  }
  return j;
}

}  // namespace

void DatasetRecorder::writer_loop() {
  uint64_t index = 0;
  while (true) {
    Item item;
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_cv.wait(lock, [&] { return m_stop || !m_queue.empty(); });
      if (m_queue.empty()) return;  // stopping, and drained
      item = std::move(m_queue.front());
      m_queue.pop_front();
    }
    if (index % 100 == 0) {
      std::error_code ec;
      const auto space = std::filesystem::space(m_dir, ec);
      if (!ec && space.available < kMinFreeBytes) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_disk_full = true;
        m_queue.clear();
        std::cerr << "DatasetRecorder: under 2 GiB free, stopping" << std::endl;
        return;
      }
    }
    if (!item.camera_matrix.empty()) {
      const std::string cal =
          json{{"camera_matrix", mat_json(item.camera_matrix)},
               {"distortion_coefficients", mat_json(item.dist_coeffs)}}
              .dump();
      if (cal != m_last_calibration) {
        m_last_calibration = cal;
        m_frames << "{\"calibration\":" << cal << "}\n";
      }
    }
    // The same conversion FiducialDetector::detect applies.
    cv::Mat gray;
    if (item.frame.channels() == 3) {
      cv::cvtColor(item.frame, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = item.frame;
    }
    char name[32];
    std::snprintf(name, sizeof name, "%06llu.pgm", static_cast<unsigned long long>(index));
    if (write_pgm(m_dir / "frames" / name, gray)) {
      m_frames << frame_json(std::string("frames/") + name, item.timestamp,
                             item.processed_index, gray, item.observation, item.result)
                      .dump()
               << "\n";
      if (++index % 30 == 0) m_frames.flush();
    }
  }
}
