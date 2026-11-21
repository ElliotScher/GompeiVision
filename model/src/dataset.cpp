#include "gv/model/dataset.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace gv::model {

namespace {

using nlohmann::json;

json read_json(const std::filesystem::path& p) {
  std::ifstream in(p);
  if (!in) throw std::runtime_error("cannot open " + p.string());
  return json::parse(in);
}

double worst_diff(const Detection& a, const Detection& b) {
  double w = 0;
  for (size_t k = 0; k < 4; ++k)
    w = std::max(w, std::hypot(a.corners[k][0] - b.corners[k][0], a.corners[k][1] - b.corners[k][1]));
  return w;
}

}  // namespace

Dataset load_dataset(const std::filesystem::path& dir) {
  Dataset d;
  d.dir = dir;
  const json s = read_json(dir / "session.json");
  if (s.value("format", "") != "gompeivision-dataset")
    throw std::runtime_error(dir.string() + ": not a gompeivision dataset");
  if (s.value("version", 0) != 1)
    throw std::runtime_error(dir.string() + ": unsupported dataset version");
  d.hardware_id = s.value("hardware_id", "");
  d.role = s.value("role", "");
  d.apriltag_version = s.value("apriltag_version", "");
  if (s.contains("detector")) d.quad_decimate = static_cast<int>(s["detector"].value("quad_decimate", 1.0));

  std::ifstream in(dir / "frames.jsonl");
  if (!in) throw std::runtime_error("cannot open " + (dir / "frames.jsonl").string());
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    ++n;
    if (line.empty()) continue;
    const json j = json::parse(line);
    if (j.contains("calibration")) continue;
    DatasetFrame f;
    f.file = j.at("file").get<std::string>();
    f.timestamp = j.value("timestamp", 0LL);
    for (const auto& det : j.value("detections", json::array())) {
      const auto c = det.at("corners").get<std::vector<double>>();
      if (c.size() != 8) throw std::runtime_error("frames.jsonl line " + std::to_string(n) + ": need 8 corner values");
      Detection x;
      x.id = det.at("id").get<int>();
      for (size_t k = 0; k < 4; ++k) x.corners[k] = {c[2 * k], c[2 * k + 1]};
      f.cpu.push_back(x);
    }
    d.frames.push_back(std::move(f));
  }
  return d;
}

double ReplayStats::quantile(double q) const {
  if (corner_diffs.empty()) return 0;
  auto e = corner_diffs;
  std::sort(e.begin(), e.end());
  return e[std::min(e.size() - 1, static_cast<size_t>(q * static_cast<double>(e.size())))];
}

ReplayStats replay(const Dataset& d, const DetectorParams& params, double match_px,
                   const std::function<void(const ReplayFrame&)>& on_frame) {
  static const Tag36h11Lookup family;
  ReplayStats st;
  for (const DatasetFrame& f : d.frames) {
    ReplayFrame rf{&f, detect(read_pgm(d.dir / f.file), family, params), 0, 0, 0, 0};
    std::vector<bool> used(rf.golden.size(), false);
    for (const Detection& c : f.cpu) {
      bool hit = false;
      for (size_t i = 0; i < rf.golden.size(); ++i) {
        if (used[i] || rf.golden[i].id != c.id) continue;
        const double diff = worst_diff(c, rf.golden[i]);
        if (diff <= match_px) {
          used[i] = hit = true;
          st.corner_diffs.push_back(diff);
          rf.worst_diff = std::max(rf.worst_diff, diff);
          break;
        }
      }
      hit ? ++rf.matched : ++rf.only_cpu;
    }
    for (bool u : used) rf.only_golden += !u;
    ++st.frames;
    st.cpu_tags += static_cast<int>(f.cpu.size());
    st.golden_tags += static_cast<int>(rf.golden.size());
    st.matched += rf.matched, st.only_cpu += rf.only_cpu, st.only_golden += rf.only_golden;
    if (on_frame) on_frame(rf);
  }
  return st;
}

}  // namespace gv::model
