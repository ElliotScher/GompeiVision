// Recorded datasets (format: model/datasets/README.md) and the replay that
// runs the golden detector over one and diffs it against the CPU pipeline's
// recorded detections (Stage 1: "a replay tool: run any dataset through the
// models and diff the result against the CPU output").
#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "gv/model/detector.h"

namespace gv::model {

struct DatasetFrame {
  std::string file;  // relative to the session directory
  long long timestamp = 0;
  std::vector<Detection> cpu;  // the CPU pipeline's detections (corners TL, TR, BR, BL)
};

struct Dataset {
  std::filesystem::path dir;
  std::string hardware_id, role, apriltag_version;
  int quad_decimate = 1;  // the CPU detector's setting
  std::vector<DatasetFrame> frames;
};

// Throws std::runtime_error on a malformed session.
Dataset load_dataset(const std::filesystem::path& session_dir);

struct ReplayStats {
  int frames = 0;
  int cpu_tags = 0, golden_tags = 0;
  int matched = 0;      // same ID in both, corners within match_px in order
  int only_cpu = 0;     // the CPU saw it, the golden detector didn't
  int only_golden = 0;  // the golden detector saw it, the CPU didn't
  std::vector<double> corner_diffs;  // per matched tag: worst corner difference, px
  double quantile(double q) const;
};

struct ReplayFrame {
  const DatasetFrame* frame;
  std::vector<Detection> golden;
  int matched = 0, only_cpu = 0, only_golden = 0;
  double worst_diff = 0;
};

// Runs the golden detector on every frame. `on_frame` (optional) sees each
// frame's result, e.g. for a per-frame CSV.
ReplayStats replay(const Dataset& d, const DetectorParams& params, double match_px = 2.0,
                   const std::function<void(const ReplayFrame&)>& on_frame = {});

}  // namespace gv::model
