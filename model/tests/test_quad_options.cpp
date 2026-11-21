// Stage 1's quad-finding decision, on synthetic scenes: 4B alone, 4B with
// the 4C tracer, and 4A (AprilTag's own boundary clustering and quad fit,
// apriltag_quad_thresh), each followed by the same golden quick decode,
// refinement and decode. AprilTag's whole detector is the reference.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>

#include "apriltag.h"
#include "apriltag_ref.h"
#include "common/zarray.h"
#include "gv/model/decimate.h"
#include "gv/model/detector.h"
#include "synth.h"
#include "tag36h11.h"

extern "C" zarray_t* apriltag_quad_thresh(apriltag_detector_t* td, image_u8_t* im);

using namespace gv::model;

namespace {

const Tag36h11Lookup& family() {
  static const Tag36h11Lookup f;
  return f;
}

// AprilTag's 4A quads on the decimated image, in decimated coordinates.
std::vector<Quad> apriltag_quads(const Image8& small) {
  auto td = gv::test::make_detector(1);
  // fit_quads reads the families to choose border polarity and minimum size.
  static apriltag_family_t* fam = tag36h11_create();
  apriltag_detector_add_family(td.get(), fam);
  image_u8_t v = gv::test::view(small);
  zarray_t* quads = apriltag_quad_thresh(td.get(), &v);
  std::vector<Quad> out;
  for (int i = 0; i < zarray_size(quads); ++i) {
    struct quad* q;
    zarray_get_volatile(quads, i, &q);
    if (q->reversed_border) continue;  // tag36h11 has a normal border
    Quad m;
    for (size_t k = 0; k < 4; ++k) m.corners[k] = {q->p[k][0], q->p[k][1]};
    out.push_back(m);
  }
  zarray_destroy(quads);
  return out;
}

struct Row {
  int drawn = 0, found = 0, false_ids = 0, cut = 0, candidates = 0, refinements = 0, frames = 0;
};

void count(const std::vector<Detection>& dets, const gv::test::Scene& s, Row& r) {
  r.drawn += static_cast<int>(s.tags.size());
  for (const Detection& d : dets) {
    bool ok = false;
    for (const auto& t : s.tags) {
      double e = 0;
      for (size_t k = 0; k < 4; ++k) e = std::max(e, std::hypot(d.corners[k][0] - t.corners[k][0], d.corners[k][1] - t.corners[k][1]));
      ok |= t.id == d.id && e < 2.0;
    }
    if (!ok && std::getenv("GV_DEBUG")) {
      double best = 1e9;
      int bid = -1;
      for (const auto& t : s.tags) {
        double e = 0;
        for (size_t k = 0; k < 4; ++k) e = std::max(e, std::hypot(d.corners[k][0] - t.corners[k][0], d.corners[k][1] - t.corners[k][1]));
        if (e < best) best = e, bid = t.id;
      }
      std::cout << "    false: id " << d.id << " hamming " << d.hamming << " margin " << d.decision_margin
                << ", nearest drawn " << bid << " at " << best << " px\n";
    }
    if (!ok && d.cut_by_frame) {
      ++r.cut;  // extrapolated corner, flagged: not a false detection
      continue;
    }
    ok ? ++r.found : ++r.false_ids;
  }
}

struct Scenario {
  const char* name;
  gv::test::SceneParams p;
  int decimate;
  uint32_t seed;
};

}  // namespace

TEST(QuadOptions, Compare) {
  gv::test::SceneParams cpu;
  cpu.width = 1280, cpu.height = 800, cpu.num_tags = 6, cpu.min_tag_px = 60, cpu.max_tag_px = 400;
  gv::test::SceneParams full;
  gv::test::SceneParams small;
  small.num_tags = 6, small.min_tag_px = 20, small.max_tag_px = 40;
  gv::test::SceneParams dark;
  dark.num_tags = 4, dark.background_min = 20, dark.background_max = 90;
  gv::test::SceneParams dark_small = small;
  dark_small.background_min = 20, dark_small.background_max = 90;
  const Scenario scenarios[] = {{"1280x800, decimate 4", cpu, 4, 1000},
                                {"640x400 full res", full, 1, 2000},
                                {"small tags 20-40 px", small, 1, 3000},
                                {"dark background", dark, 1, 5000},
                                {"dark background, small tags", dark_small, 1, 6000}};
  int total_4a = 0, total_4b = 0, total_4bc = 0;
  for (const Scenario& sc : scenarios) {
    Row r4b, r4bc, r4a, ref;
    for (uint32_t s = sc.seed; s < sc.seed + 30; ++s) {
      const auto scene = gv::test::make_scene(s, sc.p);
      DetectorParams dp;
      dp.decimate = sc.decimate;
      DetectorStats st;
      dp.use_4c = false;
      count(detect(scene.image, family(), dp, &st), scene, r4b);
      r4b.candidates += st.quads_ok, r4b.refinements += st.quads_ok - st.rough_decode_fail;
      dp.use_4c = true;
      count(detect(scene.image, family(), dp, &st), scene, r4bc);
      r4bc.candidates += st.quads_ok + st.quads_4c, r4bc.refinements += st.quads_ok + st.quads_4c - st.rough_decode_fail;
      const auto quads = apriltag_quads(decimate(scene.image, sc.decimate));
      count(detect_from_rough_quads(scene.image, family(), quads, dp, &st), scene, r4a);
      r4a.candidates += st.quads_ok, r4a.refinements += st.quads_ok - st.rough_decode_fail;
      count(gv::test::apriltag_detect(scene.image, static_cast<float>(sc.decimate)), scene, ref);
      ++r4b.frames, ++r4bc.frames, ++r4a.frames;
    }
    auto line = [](const char* who, const Row& r) {
      std::cout << "[ quads ]   " << std::left << std::setw(26) << who << std::right << " found " << std::setw(3) << r.found
                << "/" << r.drawn << "  false " << r.false_ids << "  cut " << r.cut;
      if (r.frames) std::cout << "  quads/frame " << r.candidates / r.frames << "  refinements/frame " << r.refinements / r.frames;
      std::cout << "\n";
    };
    std::cout << "[ quads ] " << sc.name << "\n";
    line("4B", r4b);
    line("4B + 4C", r4bc);
    line("4A (AprilTag quads)", r4a);
    line("AprilTag (reference)", ref);
    total_4a += r4a.found, total_4b += r4b.found, total_4bc += r4bc.found;
    EXPECT_EQ(r4b.false_ids + r4bc.false_ids + r4a.false_ids, 0) << sc.name;
  }
  RecordProperty("found_4a", total_4a);
  RecordProperty("found_4b", total_4b);
  RecordProperty("found_4b_4c", total_4bc);
}
