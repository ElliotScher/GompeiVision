// Edge refinement (step 5) on 4B's rough quads, measured against the synthetic
// scenes' drawn corners, for the window policies the Stage 1 doc compares.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "gv/model/components.h"
#include "gv/model/decimate.h"
#include "gv/model/quad.h"
#include "gv/model/refine.h"
#include "gv/model/threshold.h"
#include "synth.h"

using namespace gv::model;

namespace {

struct Result {
  int tags = 0, refined = 0, failed = 0, bad = 0;  // bad: refined but a corner > 1 px off
  std::vector<double> errors;                      // worst corner per refined tag
  double quantile(double q) const {
    if (errors.empty()) return 0;
    std::vector<double> e = errors;
    std::sort(e.begin(), e.end());
    return e[std::min(e.size() - 1, static_cast<size_t>(q * static_cast<double>(e.size())))];
  }
};

double worst_corner_error(const std::array<std::array<double, 2>, 4>& q, const double truth[4][2]) {
  double worst = 0;
  for (const auto& c : q) {
    double best = 1e9;
    for (int k = 0; k < 4; ++k) best = std::min(best, std::hypot(c[0] - truth[k][0], c[1] - truth[k][1]));
    worst = std::max(worst, best);
  }
  return worst;
}

// One tag to refine: the scene image, 4B's accepted rough quad at full
// resolution, and the drawn corners.
struct Case {
  const Image8* image;
  Quad rough;
  double truth[4][2];
};

struct Scenario {
  std::vector<gv::test::Scene> scenes;
  std::vector<Case> cases;
  int tags = 0;
};

// Rough quads that 4B got right (every corner within max(2 px, 5% of the
// side) on the image it ran on, as test_quad counts them). 4B's own misses,
// including its rare wrong-but-accepted quads, are measured in test_quad;
// here only refinement is measured.
Scenario build(const gv::test::SceneParams& sp, int decimation, uint32_t seed0, int scenes) {
  Scenario sc;
  sc.scenes.reserve(static_cast<size_t>(scenes));
  for (uint32_t s = seed0; s < seed0 + static_cast<uint32_t>(scenes); ++s) sc.scenes.push_back(gv::test::make_scene(s, sp));
  for (const auto& scene : sc.scenes) {
    const Labeling l = label_components(threshold(decimate(scene.image, decimation)).image);
    for (const auto& tag : scene.tags) {
      ++sc.tags;
      Case c{&scene.image, {}, {}};
      for (int k = 0; k < 4; ++k) c.truth[k][0] = tag.corners[k][0], c.truth[k][1] = tag.corners[k][1];
      const double side = std::hypot(c.truth[1][0] - c.truth[0][0], c.truth[1][1] - c.truth[0][1]);
      const double tol = std::max(2.0, 0.05 * side / decimation) * decimation;
      double best = tol;
      bool found = false;
      for (const Component& comp : l.closed) {
        if (comp.color != 0 || comp.area < 16) continue;
        const QuadResult q = scorecard_quad(comp);
        if (q.verdict != QuadVerdict::kOk) continue;
        const Quad full = to_full_resolution(q.quad, decimation);
        const double e = worst_corner_error(full.corners, c.truth);
        if (e <= best) best = e, c.rough = full, found = true;
      }
      if (found) sc.cases.push_back(c);
    }
  }
  return sc;
}

Result run(const Scenario& sc, const RefineParams& rp) {
  Result r;
  for (const Case& c : sc.cases) {
    ++r.tags;
    const auto refined = refine_quad(*c.image, c.rough, rp);
    if (!refined) {
      ++r.failed;
      continue;
    }
    ++r.refined;
    const double e = worst_corner_error(refined->corners, c.truth);
    r.errors.push_back(e);
    r.bad += e > 1.0;
  }
  return r;
}

struct Policy {
  std::string name;
  RefineParams params;
};

std::vector<Policy> policies() {
  RefineParams scaled;  // the default: scaled window, direction check
  RefineParams scaled_any = scaled;
  scaled_any.check_direction = false;
  RefineParams fixed2 = scaled, fixed5 = scaled;
  fixed2.window = fixed5.window = WindowPolicy::kFixed;
  fixed2.fixed_window = 2, fixed5.fixed_window = 5;
  RefineParams fixed5_any = fixed5;
  fixed5_any.check_direction = false;
  RefineParams scaled2 = scaled;
  scaled2.passes = 2;
  return {{"scaled, direction check (default)", scaled},
          {"  ... with a second pass", scaled2},
          {"scaled, any direction", scaled_any},
          {"fixed +/-2, direction check", fixed2},
          {"fixed +/-5, direction check", fixed5},
          {"fixed +/-5, any direction", fixed5_any}};
}

std::vector<Result> report(const std::string& scenario, const gv::test::SceneParams& sp, int decimation, uint32_t seed0,
              int scenes) {
  const Scenario sc = build(sp, decimation, seed0, scenes);
  std::vector<Result> all;
  std::cout << "[ refine ] " << scenario << ": " << sc.cases.size() << " of " << sc.tags
            << " drawn tags have a correct 4B quad\n";
  for (const auto& [name, rp] : policies()) {
    const Result r = run(sc, rp);
    std::cout << "[ refine ]   " << std::left << std::setw(34) << name << std::right << std::fixed
              << std::setprecision(3) << " median " << r.quantile(0.5) << "  p95 " << r.quantile(0.95)
              << "  max " << (r.errors.empty() ? 0 : *std::max_element(r.errors.begin(), r.errors.end()))
              << " px;  >1 px: " << r.bad << ", failed: " << r.failed << " of " << r.tags << "\n";
    all.push_back(r);
  }
  return all;
}

}  // namespace

TEST(Refine, CpuConfigDecimatedQuads) {
  gv::test::SceneParams p;
  p.width = 1280, p.height = 800, p.num_tags = 6, p.min_tag_px = 60, p.max_tag_px = 400;
  // Rough quads from a decimated image need the second pass (policies()[1]).
  const Result r = report("1280x800, 4B on decimate 4, refined at full res", p, 4, 1000, 40)[1];
  EXPECT_GE(r.tags, 140);
  EXPECT_LT(r.quantile(0.5), 0.1);
  EXPECT_LT(r.quantile(0.95), 0.25);
  EXPECT_EQ(r.bad + r.failed, 0);
}

TEST(Refine, FullResolutionQuads) {
  gv::test::SceneParams p;
  const Result r = report("640x400, 4B and refinement at full res", p, 1, 2000, 60)[0];
  EXPECT_LT(r.quantile(0.5), 0.1);
  EXPECT_LT(r.quantile(0.95), 0.25);
  EXPECT_EQ(r.bad + r.failed, 0);
}

TEST(Refine, SmallTags) {
  // 20-40 px tags: cells of 2.5-5 px, so the wrong edges are close.
  gv::test::SceneParams p;
  p.num_tags = 6, p.min_tag_px = 20, p.max_tag_px = 40;
  const Result r = report("640x400, small tags (20-40 px)", p, 1, 3000, 60)[0];
  EXPECT_LT(r.quantile(0.95), 0.6);
  EXPECT_EQ(r.bad + r.failed, 0);
}

TEST(Refine, HeavyBlur) {
  gv::test::SceneParams p;
  p.extra_blur = 3;  // four 3x3 box passes: edges spread over ~8 px
  const Result r = report("640x400, heavy blur", p, 1, 4000, 60)[0];
  EXPECT_LT(r.quantile(0.95), 0.5);
  EXPECT_EQ(r.bad + r.failed, 0);
}
