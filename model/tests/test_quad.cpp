#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <map>

#include "gv/model/components.h"
#include "gv/model/decimate.h"
#include "gv/model/quad.h"
#include "gv/model/threshold.h"
#include "synth.h"

using namespace gv::model;

namespace {

// White image with black pixels wherever inside(x, y); returns the black
// component covering (sx, sy).
Component black_shape(int w, int h, const std::function<bool(double, double)>& inside, int sx, int sy) {
  Image8 t(w, h, 255);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      if (inside(x + 0.5, y + 0.5)) t.at(x, y) = 0;
  const Labeling l = label_components(t);
  const uint32_t id = l.label[t.index(sx, sy)];
  for (const Component& c : l.closed)
    if (c.id == id) return c;
  ADD_FAILURE() << "no component at " << sx << "," << sy;
  return {};
}

double worst_corner_error(const Quad& q, const double truth[4][2]) {
  double worst = 0;
  for (const auto& c : q.corners) {
    double best = 1e9;
    for (int k = 0; k < 4; ++k) best = std::min(best, std::hypot(c[0] - truth[k][0], c[1] - truth[k][1]));
    worst = std::max(worst, best);
  }
  return worst;
}

}  // namespace

// A tag-like square ring: black border 1/8 of the side (one tag cell), white inside.
bool ring(double u, double v, double half) {
  return std::abs(u) < half && std::abs(v) < half && !(std::abs(u) < half * 0.75 && std::abs(v) < half * 0.75);
}

TEST(Quad, AxisAlignedSquare) {
  const Component c = black_shape(40, 40, [](double x, double y) { return ring(x - 20, y - 20, 10); }, 11, 11);
  const QuadResult r = scorecard_quad(c);
  EXPECT_EQ(r.verdict, QuadVerdict::kOk) << to_string(r.verdict);
  const double truth[4][2] = {{10.5, 10.5}, {29.5, 10.5}, {29.5, 29.5}, {10.5, 29.5}};
  EXPECT_LT(worst_corner_error(r.quad, truth), 0.01);
}

TEST(Quad, RotatedSquareCornersWithinAPixel) {
  for (double deg : {10.0, 30.0, 45.0, 60.0, 80.0}) {
    const double a = deg * M_PI / 180, cx = 50, cy = 50, half = 20;
    const double ca = std::cos(a), sa = std::sin(a);
    auto inside = [&](double x, double y) {
      const double u = (x - cx) * ca + (y - cy) * sa, v = -(x - cx) * sa + (y - cy) * ca;
      return ring(u, v, half);
    };
    double truth[4][2];
    const double s[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    for (int k = 0; k < 4; ++k) {
      truth[k][0] = cx + half * (s[k][0] * ca - s[k][1] * sa);
      truth[k][1] = cy + half * (s[k][0] * sa + s[k][1] * ca);
    }
    // Seed the lookup on the ring, halfway along the top border.
    const double su = 0, sv = -half * 0.875;
    const int sx = static_cast<int>(cx + su * ca - sv * sa), sy = static_cast<int>(cy + su * sa + sv * ca);
    const QuadResult r = scorecard_quad(black_shape(100, 100, inside, sx, sy));
    EXPECT_EQ(r.verdict, QuadVerdict::kOk) << deg << ": " << to_string(r.verdict);
    // Extremes are pixel centers, so up to ~0.7 px inside the true corner.
    EXPECT_LT(worst_corner_error(r.quad, truth), 1.0) << deg;
  }
}

TEST(Quad, LShapeIsSentToTheTracer) {
  const Component c = black_shape(60, 60, [](double x, double y) {
    return (x > 10 && x < 50 && y > 10 && y < 25) || (x > 10 && x < 25 && y > 10 && y < 50);
  }, 15, 15);
  const QuadResult r = scorecard_quad(c);
  EXPECT_NE(r.verdict, QuadVerdict::kOk);
}

TEST(Quad, SolidSquareIsTooFullForATag) {
  // Every tag36h11 code has white cells inside, so a solid square isn't a tag.
  const Component c = black_shape(40, 40, [](double x, double y) { return x > 10 && x < 30 && y > 10 && y < 30; }, 20, 20);
  EXPECT_EQ(scorecard_quad(c).verdict, QuadVerdict::kFillOutOfRange);
}

TEST(Quad, TinyBlobIsRejected) {
  const Component c = black_shape(10, 10, [](double x, double y) { return x > 4 && x < 6 && y > 4 && y < 6; }, 5, 5);
  EXPECT_NE(scorecard_quad(c).verdict, QuadVerdict::kOk);
}

namespace {

struct Stats {
  int tags = 0, tight = 0, found = 0, ok_quads = 0, stray_ok_quads = 0;
  std::map<std::string, int> misses;
};

// "Refinable": every rough corner within max(2 px, 5% of the side) of the
// truth. Step 5 refines whole edges along their normals, so it only needs the
// rough quad's edges inside its search window, which is assumed to be about
// +/-3 px (a Stage 4 design parameter). "Tight" is within 1.5 px.
double refinable_tolerance(const double truth[4][2]) {
  const double side = std::hypot(truth[1][0] - truth[0][0], truth[1][1] - truth[0][1]);
  return std::max(2.0, 0.05 * side);
}

// For each drawn tag: is there a closed black component whose 4B quad passes
// and has every corner within tol of the tag's black-border corners?
void evaluate(const gv::test::SceneParams& sp, int decimation, uint32_t seed0, int scenes, Stats& st) {
  for (uint32_t s = seed0; s < seed0 + static_cast<uint32_t>(scenes); ++s) {
    const auto scene = gv::test::make_scene(s, sp);
    const Image8 img = decimate(scene.image, decimation);
    const Labeling l = label_components(threshold(img).image);
    // Accepted quads that match no tag: decode rejects them, at some cost.
    for (const Component& c : l.closed) {
      if (c.color != 0 || c.area < 16) continue;
      const QuadResult r = scorecard_quad(c);
      if (r.verdict != QuadVerdict::kOk) continue;
      ++st.ok_quads;
      bool matches = false;
      for (const auto& tag : scene.tags) {
        double truth[4][2];
        for (int k = 0; k < 4; ++k)
          for (int j = 0; j < 2; ++j) truth[k][j] = (tag.corners[k][j] - 0.5) / decimation + 0.5;
        matches |= worst_corner_error(r.quad, truth) <= refinable_tolerance(truth);
      }
      st.stray_ok_quads += !matches;
    }
    for (const auto& tag : scene.tags) {
      double truth[4][2];
      for (int k = 0; k < 4; ++k)  // full-res pixel-span coordinates to decimated ones
        for (int j = 0; j < 2; ++j) truth[k][j] = (tag.corners[k][j] - 0.5) / decimation + 0.5;
      ++st.tags;
      double best_err = 1e9;
      QuadVerdict best_verdict = QuadVerdict::kTooFewCorners;
      for (const Component& c : l.closed) {
        if (c.color != 0 || c.area < 16) continue;
        const QuadResult r = scorecard_quad(c);
        if (r.verdict == QuadVerdict::kTooFewCorners) continue;
        const double err = worst_corner_error(r.quad, truth);
        if (err < best_err || (std::abs(err - best_err) < 1e-9 && r.verdict == QuadVerdict::kOk))
          best_err = err, best_verdict = r.verdict;
      }
      const double tol = refinable_tolerance(truth);
      if (best_err <= 1.5 && best_verdict == QuadVerdict::kOk) ++st.tight;
      if (best_err <= tol && best_verdict == QuadVerdict::kOk) ++st.found;
      else if (best_err <= tol) st.misses[std::string("rejected, goes to 4C: ") + to_string(best_verdict)]++;
      else if (best_verdict == QuadVerdict::kOk) st.misses["wrong quad accepted (silent miss unless 4C also runs)"]++;
      else st.misses[std::string("no matching quad, rejected, goes to 4C: ") + to_string(best_verdict)]++;
    }
  }
}

void report(const char* name, const Stats& st) {
  std::cout << "[ 4B ] " << name << ": " << st.found << "/" << st.tags << " tags with a refinable quad ("
            << 100.0 * st.found / st.tags << "%), " << st.tight << " within 1.5 px\n";
  for (const auto& [why, n] : st.misses) std::cout << "[ 4B ]   miss: " << why << ": " << n << "\n";
  std::cout << "[ 4B ]   accepted quads: " << st.ok_quads << ", of which " << st.stray_ok_quads
            << " match no tag (sent to decode, which rejects them)\n";
}

}  // namespace

TEST(Quad, ScorecardOnSyntheticScenesCpuConfig) {
  // The CPU pipeline's setup: 1280x800, quad_decimate = 4.
  gv::test::SceneParams p;
  p.width = 1280, p.height = 800, p.num_tags = 6, p.min_tag_px = 60, p.max_tag_px = 400;
  Stats st;
  evaluate(p, 4, 1000, 40, st);
  report("1280x800, decimate 4", st);
  RecordProperty("found_percent", static_cast<int>(100.0 * st.found / st.tags));
  EXPECT_GE(st.found * 100, st.tags * 90);
}

TEST(Quad, ScorecardOnSyntheticScenesFullRes) {
  gv::test::SceneParams p;  // 640x400, tags 40-160 px
  Stats st;
  evaluate(p, 1, 2000, 60, st);
  report("640x400, full resolution", st);
  EXPECT_GE(st.found * 100, st.tags * 90);
}
