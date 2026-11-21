// The whole golden detector against ground truth and against AprilTag 3
// configured as the CPU pipeline runs it (PLAN: "the whole detector is tested
// by tolerance against AprilTag 3 ... same IDs, corners within ~0.1 px, and a
// detection rate at least as good").
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "apriltag.h"
#include "apriltag_ref.h"
#include "gv/model/decode.h"
#include "gv/model/detector.h"
#include "synth.h"
#include "tag36h11.h"

using namespace gv::model;

namespace {

const Tag36h11Lookup& family() {
  static const Tag36h11Lookup f;
  return f;
}

using Corners = std::array<std::array<double, 2>, 4>;

double ordered_error(const Corners& a, const Corners& b) {
  double worst = 0;
  for (size_t k = 0; k < 4; ++k) worst = std::max(worst, std::hypot(a[k][0] - b[k][0], a[k][1] - b[k][1]));
  return worst;
}

Corners truth_of(const gv::test::PlacedTag& t) {
  Corners c;
  for (size_t k = 0; k < 4; ++k) c[k] = {t.corners[k][0], t.corners[k][1]};
  return c;
}

using gv::test::apriltag_detect;

struct Tally {
  int drawn = 0, found = 0, false_ids = 0, cut = 0;  // cut: flagged cut_by_frame, not false
  std::vector<double> errors;  // per found tag, worst corner vs truth (ordered)
  double q(double f) const {
    if (errors.empty()) return 0;
    auto e = errors;
    std::sort(e.begin(), e.end());
    return e[std::min(e.size() - 1, static_cast<size_t>(f * static_cast<double>(e.size())))];
  }
};

// A detection counts as the drawn tag if its id matches and every corner (in
// order) is within 2 px; anything else is a false detection.
void score(const std::vector<Detection>& dets, const gv::test::Scene& s, Tally& t, std::vector<const Detection*>* matched) {
  t.drawn += static_cast<int>(s.tags.size());
  std::vector<bool> used(dets.size(), false);
  for (const auto& tag : s.tags) {
    const Detection* hit = nullptr;
    for (size_t i = 0; i < dets.size(); ++i)
      if (!used[i] && dets[i].id == tag.id && ordered_error(dets[i].corners, truth_of(tag)) < 2.0) {
        used[i] = true, hit = &dets[i];
        break;
      }
    if (matched) matched->push_back(hit);
    if (hit) ++t.found, t.errors.push_back(ordered_error(hit->corners, truth_of(tag)));
  }
  for (size_t i = 0; i < dets.size(); ++i)
    if (!used[i]) dets[i].cut_by_frame ? ++t.cut : ++t.false_ids;
}

struct Comparison {
  Tally ours, april;
  std::vector<double> agreement;  // per tag both found: worst corner difference
  DetectorStats stats;
};

Comparison compare(const gv::test::SceneParams& sp, int decimate, uint32_t seed0, int scenes) {
  Comparison c;
  DetectorParams dp;
  dp.decimate = decimate;
  for (uint32_t s = seed0; s < seed0 + static_cast<uint32_t>(scenes); ++s) {
    const auto scene = gv::test::make_scene(s, sp);
    DetectorStats st;
    const auto ours = detect(scene.image, family(), dp, &st);
    c.stats.components += st.components, c.stats.quads_ok += st.quads_ok, c.stats.sent_to_4c += st.sent_to_4c;
    c.stats.rough_decode_fail += st.rough_decode_fail, c.stats.refine_fail += st.refine_fail;
    c.stats.decode_fail += st.decode_fail, c.stats.duplicates += st.duplicates;
    c.stats.quads_4c += st.quads_4c, c.stats.found_4c += st.found_4c;
    const auto april = apriltag_detect(scene.image, static_cast<float>(decimate));
    std::vector<const Detection*> mo, ma;
    score(ours, scene, c.ours, &mo);
    score(april, scene, c.april, &ma);
    for (size_t i = 0; i < mo.size(); ++i)
      if (mo[i] && ma[i]) c.agreement.push_back(ordered_error(mo[i]->corners, ma[i]->corners));
  }
  return c;
}

void print(const char* name, const Comparison& c) {
  auto line = [](const char* who, const Tally& t) {
    std::cout << "[ detector ]   " << std::left << std::setw(10) << who << std::right << std::fixed << std::setprecision(3)
              << " found " << t.found << "/" << t.drawn << ", false " << t.false_ids << ", cut " << t.cut << ", corner error vs truth median "
              << t.q(0.5) << " p95 " << t.q(0.95) << " px\n";
  };
  std::cout << "[ detector ] " << name << "\n";
  line("golden", c.ours);
  line("AprilTag", c.april);
  auto a = c.agreement;
  std::sort(a.begin(), a.end());
  if (!a.empty())
    std::cout << "[ detector ]   golden vs AprilTag on " << a.size() << " tags both found: median " << a[a.size() / 2]
              << " p95 " << a[std::min(a.size() - 1, a.size() * 95 / 100)] << " px\n";
  std::cout << "[ detector ]   pipeline: " << c.stats.components << " components, " << c.stats.quads_ok << " 4B quads, "
            << c.stats.sent_to_4c << " to 4C (" << c.stats.quads_4c << " quads, " << c.stats.found_4c << " tags), " << c.stats.rough_decode_fail << " failed rough decode, "
            << c.stats.refine_fail << " failed refinement, " << c.stats.decode_fail << " failed final decode, "
            << c.stats.duplicates << " duplicates\n";
}

}  // namespace

TEST(Decode, TrueCornersDecodeExactly) {
  for (uint32_t s = 1; s <= 20; ++s) {
    const auto scene = gv::test::make_scene(s);
    for (const auto& tag : scene.tags) {
      const auto read = read_code(scene.image, truth_of(tag), family());
      ASSERT_TRUE(read) << "seed " << s;
      const auto m = family().decode(read->code);
      ASSERT_TRUE(m) << "seed " << s << " tag " << tag.id;
      EXPECT_EQ(m->id, tag.id);
      EXPECT_EQ(m->rotation, 0);  // truth corners are in canonical order
      EXPECT_EQ(m->hamming, 0);
      EXPECT_GT(read->decision_margin, 20);
    }
  }
}

TEST(Decode, RotatedCornerOrderGivesRotation) {
  const auto scene = gv::test::make_scene(5);
  const auto& tag = scene.tags[0];
  const Corners t = truth_of(tag);
  for (int r = 0; r < 4; ++r) {
    Corners q;  // start the quad r corners later: the read is the code rotated
    for (size_t k = 0; k < 4; ++k) q[k] = t[(k + static_cast<size_t>(r)) % 4];
    const auto m = family().decode(read_code(scene.image, q, family())->code);
    ASSERT_TRUE(m);
    EXPECT_EQ(m->id, tag.id);
    // canonical corner k = quad corner (k - rotation) mod 4, i.e. t[k] = q[(k - rot) mod 4] = t[(k - rot + r) mod 4]
    EXPECT_EQ(m->rotation, r) << "start offset " << r;
  }
}

TEST(Detector, CpuConfigAgainstAprilTag) {
  gv::test::SceneParams p;
  p.width = 1280, p.height = 800, p.num_tags = 6, p.min_tag_px = 60, p.max_tag_px = 400;
  const Comparison c = compare(p, 4, 1000, 30);
  print("1280x800, decimate 4 (CPU config)", c);
  EXPECT_EQ(c.ours.false_ids, 0);
  EXPECT_GE(c.ours.found * 100, c.ours.drawn * 90);
}

TEST(Detector, FullResolutionAgainstAprilTag) {
  gv::test::SceneParams p;
  const Comparison c = compare(p, 1, 2000, 40);
  print("640x400, full resolution", c);
  EXPECT_EQ(c.ours.false_ids, 0);
  EXPECT_GE(c.ours.found, c.april.found - c.april.drawn / 50);  // "at least as good", within 2%
}

TEST(Detector, SmallTagsAgainstAprilTag) {
  gv::test::SceneParams p;
  p.num_tags = 6, p.min_tag_px = 20, p.max_tag_px = 40;
  const Comparison c = compare(p, 1, 3000, 40);
  print("640x400, small tags (20-40 px), full resolution", c);
  EXPECT_EQ(c.ours.false_ids, 0);
  EXPECT_GE(c.ours.found, c.april.found - c.april.drawn / 50);
}
