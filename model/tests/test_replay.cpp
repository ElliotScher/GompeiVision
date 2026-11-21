// The replay end to end: write a session in the recorder's exact format
// (src/cpp/io/DatasetRecorder.cpp), with AprilTag standing in for the CPU
// pipeline, then load and replay it.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "apriltag_ref.h"
#include "gv/model/dataset.h"
#include "synth.h"

using namespace gv::model;
namespace fs = std::filesystem;

namespace {

fs::path write_session(const fs::path& dir, int frames, int decimate) {
  fs::remove_all(dir);
  fs::create_directories(dir / "frames");
  std::ofstream(dir / "session.json") << R"({"format": "gompeivision-dataset", "version": 1, "hardware_id": "test",
    "role": "front", "apriltag_version": "3.4.5", "tag_family": "tag36h11", "tag_size_m": 0.1651,
    "record_every": 1, "detector": {"quad_decimate": )" << decimate << R"(.0, "nthreads": 1}, "field": {}})";
  std::ofstream jl(dir / "frames.jsonl");
  jl << R"({"calibration":{"camera_matrix":[[900,0,640],[0,900,400],[0,0,1]],"distortion_coefficients":[[0,0,0,0,0]]}})" << "\n";
  gv::test::SceneParams p;
  p.width = 1280, p.height = 800, p.num_tags = 4, p.min_tag_px = 60, p.max_tag_px = 300;
  for (int i = 0; i < frames; ++i) {
    const auto scene = gv::test::make_scene(static_cast<uint32_t>(500 + i), p);
    char name[32];
    std::snprintf(name, sizeof name, "frames/%06d.pgm", i);
    write_pgm(dir / name, scene.image);
    jl << R"({"file":")" << name << R"(","timestamp":)" << 1000000 + i * 8333 << R"(,"processed_index":)" << i
       << R"(,"width":1280,"height":800,"detections":[)";
    const auto dets = gv::test::apriltag_detect(scene.image, static_cast<float>(decimate));
    for (size_t k = 0; k < dets.size(); ++k) {
      jl << (k ? "," : "") << R"({"id":)" << dets[k].id << R"(,"corners":[)";
      for (size_t c = 0; c < 4; ++c) jl << (c ? "," : "") << dets[k].corners[c][0] << "," << dets[k].corners[c][1];
      jl << "]}";
    }
    jl << "]}\n";
  }
  return dir;
}

}  // namespace

TEST(Replay, SyntheticSessionMatchesCpuDetections) {
  const fs::path dir = write_session(fs::temp_directory_path() / "gv_model_replay_test", 12, 4);
  const Dataset d = load_dataset(dir);
  ASSERT_EQ(d.frames.size(), 12u);
  EXPECT_EQ(d.quad_decimate, 4);
  EXPECT_EQ(d.apriltag_version, "3.4.5");
  DetectorParams dp;
  dp.decimate = d.quad_decimate;
  int callbacks = 0;
  const ReplayStats st = replay(d, dp, 2.0, [&](const ReplayFrame&) { ++callbacks; });
  std::cout << "[ replay ] " << st.frames << " frames: CPU " << st.cpu_tags << " tags, golden " << st.golden_tags
            << ", matched " << st.matched << ", only CPU " << st.only_cpu << ", only golden " << st.only_golden
            << ", corner diff median " << st.quantile(0.5) << " p95 " << st.quantile(0.95) << " px\n";
  EXPECT_EQ(callbacks, 12);
  EXPECT_GT(st.cpu_tags, 30);
  EXPECT_GE(st.matched * 100, st.cpu_tags * 90);
  EXPECT_LT(st.quantile(0.5), 0.1);
  fs::remove_all(dir);
}

TEST(Replay, RejectsWrongFormat) {
  const fs::path dir = fs::temp_directory_path() / "gv_model_replay_bad";
  fs::remove_all(dir);
  fs::create_directories(dir);
  std::ofstream(dir / "session.json") << R"({"format": "something-else", "version": 1})";
  EXPECT_THROW(load_dataset(dir), std::runtime_error);
  fs::remove_all(dir);
}
