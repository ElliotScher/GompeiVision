#pragma once

#include <apriltag/apriltag.h>
#include <frc/geometry/Pose3d.h>

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>

#include "util/QueuedFiducialData.h"
#include "util/QueuedFrame.h"

/**
 * @class DatasetRecorder
 * @brief Records frames with the CPU pipeline's detections and poses, as
 * golden-model datasets for the FPGA project (Stage 1).
 *
 * Off unless GOMPEI_RECORD_DIR is set. GOMPEI_RECORD_EVERY=N keeps one frame
 * in N (default 1). Each pipeline writes its own session directory:
 *
 *   <GOMPEI_RECORD_DIR>/<hardware_id>_<role>_<UTC time>/
 *     session.json   camera, detector settings, AprilTag version, tag size, field map
 *     frames.jsonl   one JSON object per recorded frame (and calibration changes)
 *     frames/NNNNNN.pgm  the grayscale image the detector saw
 *
 * The processing thread only takes a reference to the frame (cv::Mat is
 * reference-counted, and each captured frame is a fresh buffer) and copies the
 * small detection structs into a bounded queue. A writer thread does the
 * grayscale conversion, JSON and disk work; when the queue is full, frames are dropped and
 * counted, never waited on. Recording stops once free space falls below
 * kMinFreeBytes. See model/datasets/README.md for the format.
 */
class DatasetRecorder {
 public:
  // nullptr unless GOMPEI_RECORD_DIR is set (or the session can't be created).
  static std::unique_ptr<DatasetRecorder> fromEnvironment(
      const std::string& hardware_id, const std::string& role,
      const apriltag_detector_t* detector, double tag_size_m,
      const std::map<int, frc::Pose3d>& field);

  ~DatasetRecorder();
  DatasetRecorder(const DatasetRecorder&) = delete;
  DatasetRecorder& operator=(const DatasetRecorder&) = delete;

  // Called once per processed frame. `result` is null when no pose was
  // computed (no tags, or no calibration). Cheap: no pixel copies.
  void record(const QueuedFrame& frame,
              const FiducialImageObservation& observation,
              const AprilTagResult* result, const cv::Mat& camera_matrix,
              const cv::Mat& dist_coeffs);

  static constexpr size_t kQueueCapacity = 64;
  static constexpr std::uintmax_t kMinFreeBytes = 2ull << 30;  // 2 GiB

 private:
  DatasetRecorder(std::filesystem::path dir, int every);

  struct Item {
    cv::Mat frame;  // shares the captured buffer
    int64_t timestamp = 0;
    uint64_t processed_index = 0;
    FiducialImageObservation observation;
    std::optional<AprilTagResult> result;
    cv::Mat camera_matrix, dist_coeffs;  // small copies; empty without calibration
  };

  void writer_loop();

  std::filesystem::path m_dir;
  int m_every;
  uint64_t m_seen = 0;      // processed frames offered (processing thread)
  uint64_t m_queued = 0;    // frames accepted into the queue
  uint64_t m_dropped = 0;   // frames dropped because the queue was full
  std::string m_last_calibration;  // writer thread only

  std::mutex m_mutex;
  std::condition_variable m_cv;
  std::deque<Item> m_queue;
  bool m_stop = false;
  bool m_disk_full = false;
  std::ofstream m_frames;
  std::thread m_writer;
};
