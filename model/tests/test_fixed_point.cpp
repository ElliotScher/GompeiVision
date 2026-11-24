// Stage 4's fixed-point error budget: how few bits refinement and decode can
// carry before the answers change. Each configuration runs the golden
// detector with Quantization on and compares it with double precision on the
// same frames: the corner error it adds, and tags gained or lost.
//
// The sweep is DISABLED_ (slow); run it with
//   gv_model_tests --gtest_filter='FixedPoint.*' --gtest_also_run_disabled_tests
// FixedPoint.ChosenWidthsMeetBudget checks the widths picked from it.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "gv/model/detector.h"
#include "synth.h"

using namespace gv::model;

namespace {

const Tag36h11Lookup& family() {
  static const Tag36h11Lookup f;
  return f;
}

struct Budget {
  int compared = 0, lost = 0, gained = 0;
  std::vector<double> added;  // per tag: worst corner difference from double precision
  double q(double f) const {
    if (added.empty()) return 0;
    auto e = added;
    std::sort(e.begin(), e.end());
    return e[std::min(e.size() - 1, static_cast<size_t>(f * static_cast<double>(e.size())))];
  }
  double max() const { return added.empty() ? 0 : *std::max_element(added.begin(), added.end()); }
};

struct Scenario {
  const char* name;
  gv::test::SceneParams p;
  uint32_t seed;
  int scenes;
};

std::vector<Scenario> scenarios() {
  gv::test::SceneParams full;
  gv::test::SceneParams small;
  small.num_tags = 6, small.min_tag_px = 20, small.max_tag_px = 40;
  gv::test::SceneParams blur;
  blur.extra_blur = 3;
  gv::test::SceneParams big;
  big.width = 1280, big.height = 800, big.num_tags = 4, big.min_tag_px = 150, big.max_tag_px = 500;
  return {{"full res", full, 2000, 20}, {"small tags", small, 3000, 20}, {"heavy blur", blur, 4000, 20},
          {"1280x800 large tags", big, 7000, 8}};
}

// The FPGA path: full resolution, 4B, refinement and decode.
Budget measure(const Quantization& q) {
  Budget b;
  DetectorParams base;
  DetectorParams fx = base;
  fx.quant = q;
  for (const Scenario& sc : scenarios())
    for (uint32_t s = sc.seed; s < sc.seed + static_cast<uint32_t>(sc.scenes); ++s) {
      const auto scene = gv::test::make_scene(s, sc.p);
      const auto ref = detect(scene.image, family(), base);
      const auto got = detect(scene.image, family(), fx);
      std::vector<bool> used(got.size(), false);
      for (const Detection& r : ref) {
        bool hit = false;
        for (size_t i = 0; i < got.size() && !hit; ++i) {
          if (used[i] || got[i].id != r.id) continue;
          double e = 0;
          for (size_t k = 0; k < 4; ++k)
            e = std::max(e, std::hypot(got[i].corners[k][0] - r.corners[k][0], got[i].corners[k][1] - r.corners[k][1]));
          if (e < 2.0) used[i] = hit = true, b.added.push_back(e);
        }
        hit ? ++b.compared : ++b.lost;
      }
      for (bool u : used) b.gained += !u;
    }
  return b;
}

std::string label(const Quantization& q) {
  return "coord " + std::to_string(q.coord_frac) + ", weight " + std::to_string(q.weight_frac) + ", dir " +
         std::to_string(q.direction_frac) + ", value " +
         std::to_string(q.value_frac) + ", homography " + std::to_string(q.homography_bits) +
         ", plane " + std::to_string(q.plane_frac) + ", projector " + std::to_string(q.proj_scale_frac) + "/" +
         std::to_string(q.proj_persp_frac);
}

void print(const std::string& what, const Budget& b) {
  std::cout << "[ fixed ] " << std::left << std::setw(48) << what << std::right << std::fixed << std::setprecision(4)
            << " added error median " << b.q(0.5) << "  p99 " << b.q(0.99) << "  max " << b.max() << " px;  lost "
            << b.lost << ", gained " << b.gained << " of " << b.compared + b.lost << "\n";
}

// Generous widths, each quantity then reduced alone.
const Quantization kGenerous{12, 12, 16, 8, 28};

}  // namespace

TEST(FixedPoint, DISABLED_Sweep) {
  print("generous: " + label(kGenerous), measure(kGenerous));
  for (int f : {10, 8, 6}) {
    Quantization q = kGenerous;
    q.coord_frac = f;
    print("coord_frac " + std::to_string(f), measure(q));
  }
  for (int f : {10, 8, 6, 5, 4, 3}) {
    Quantization q = kGenerous;
    q.weight_frac = f;
    print("weight_frac " + std::to_string(f), measure(q));
  }
  for (int f : {14, 12, 10, 8}) {
    Quantization q = kGenerous;
    q.direction_frac = f;
    print("direction_frac " + std::to_string(f), measure(q));
  }
  for (int f : {6, 4, 2, 1}) {
    Quantization q = kGenerous;
    q.value_frac = f;
    print("value_frac " + std::to_string(f), measure(q));
  }
  for (int f : {16, 14, 12}) {
    Quantization q = kGenerous;
    q.homography_bits = f;
    print("homography_bits " + std::to_string(f), measure(q));
  }
}

TEST(FixedPoint, DISABLED_Combined) {
  // All quantities at once: errors add.
  print("lean: " + label({8, 8, 12, 4, 14}), measure({8, 8, 12, 4, 14}));
  print("lean + coord 10: " + label({10, 8, 12, 4, 14}), measure({10, 8, 12, 4, 14}));
  print("margin: " + label({10, 8, 14, 6, 16}), measure({10, 8, 14, 6, 16}));
}

TEST(FixedPoint, ChosenWidthsMeetBudget) {
  // The Stage 4 widths (decided 2026-11-22 from the sweep): coordinates 8
  // fractional bits, bilinear weights 8, edge direction 12, pixel values 4,
  // brightness planes 8, PL projector 14/24 fractional bits. Budget: no tag gained or lost, and the
  // corner error added to double precision well under the detector's own
  // 0.04-0.08 px accuracy.
  const Budget b = measure(kStage4Widths);
  print("Stage 4 widths: " + label(kStage4Widths), b);
  EXPECT_EQ(b.lost, 0);
  EXPECT_EQ(b.gained, 0);
  EXPECT_LT(b.q(0.5), 0.005);
  EXPECT_LT(b.q(0.99), 0.03);
  EXPECT_LT(b.max(), 0.06);
}

TEST(FixedPoint, DISABLED_PlaneSweep) {
  // Decode's brightness-plane coefficients, with everything else at the
  // Stage 4 widths.
  for (int f : {12, 10, 8, 6, 4, 2}) {
    Quantization q = kStage4Widths;
    q.plane_frac = f;
    print("Stage 4 + plane_frac " + std::to_string(f), measure(q));
  }
}

namespace {

// Decode's projection: every code read the detector makes, projected exactly
// (double homography, points rounded to coord_frac) and with `q`'s
// projection, everything else at the Stage 4 widths.
struct ProjectionBudget {
  long reads = 0, points = 0, points_off = 0, codes_changed = 0, ok_changed = 0, lost = 0, gained = 0;
  long frames = 0, max_reads_per_frame = 0, unpaired = 0, invalid = 0;
  int max_lsb = 0;
  double max_margin_change = 0;
  int64_t max_a = 0, max_g = 0;
  int128 max_num = 0, max_den = 0;
};

ProjectionBudget measure_projection(const Quantization& q) {
  ProjectionBudget b;
  Quantization exact = kStage4Widths;
  exact.homography_bits = 0, exact.proj_scale_frac = 0, exact.proj_persp_frac = 0;
  for (const Scenario& sc : scenarios())
    for (uint32_t s = sc.seed; s < sc.seed + static_cast<uint32_t>(sc.scenes); ++s) {
      const auto scene = gv::test::make_scene(s, sc.p);
      std::vector<DecodeSample> ref, got;
      const DecodeTrace tr_ref = [&](const DecodeSample& d) { ref.push_back(d); };
      const DecodeTrace tr_got = [&](const DecodeSample& d) { got.push_back(d); };
      DetectorParams pr, pg;
      pr.quant = exact, pr.decode_trace = &tr_ref;
      pg.quant = q, pg.decode_trace = &tr_got;
      const auto dr = detect(scene.image, family(), pr);
      const auto dg = detect(scene.image, family(), pg);
      ++b.frames;
      b.max_reads_per_frame = std::max(b.max_reads_per_frame, static_cast<long>(ref.size()));
      for (const Detection& r : dr) {
        bool hit = false;
        for (const Detection& g : dg) hit = hit || g.id == r.id;
        b.lost += !hit;
      }
      for (const Detection& g : dg) {
        bool hit = false;
        for (const Detection& r : dr) hit = hit || g.id == r.id;
        b.gained += !hit;
      }
      // Pair reads by quad (projection doesn't move corners, but a changed
      // decode can change which reads follow).
      std::vector<bool> used(got.size(), false);
      for (const DecodeSample& a : ref) {
        size_t j = 0;
        while (j < got.size() && (used[j] || got[j].quad != a.quad)) ++j;
        if (j == got.size()) {
          ++b.unpaired;
          continue;
        }
        used[j] = true;
        const DecodeSample& g = got[j];
        b.invalid += !g.projected;
        ++b.reads;
        auto cmp = [&](const std::array<double, 2>& x, const std::array<double, 2>& y) {
          const int lsb = static_cast<int>(std::lround(
              std::ldexp(std::max(std::abs(x[0] - y[0]), std::abs(x[1] - y[1])), q.coord_frac)));
          ++b.points;
          b.points_off += lsb != 0;
          b.max_lsb = std::max(b.max_lsb, lsb);
        };
        for (size_t k = 0; k < 64; ++k) cmp(a.border[k], g.border[k]);
        for (size_t k = 0; k < 36; ++k) cmp(a.cells[k], g.cells[k]);
        b.ok_changed += a.ok != g.ok;
        if (a.ok && g.ok) {
          b.codes_changed += a.code != g.code;
          b.max_margin_change = std::max(b.max_margin_change, std::abs(a.decision_margin - g.decision_margin));
        }
        if (q.fixed_projector()) {
          const FixedProjector fp(square_to_quad(g.quad), q);
          for (int64_t v : fp.a) b.max_a = std::max(b.max_a, v < 0 ? -v : v);
          for (int64_t v : fp.g) b.max_g = std::max(b.max_g, v < 0 ? -v : v);
          for (int U = -1; U <= 17; U += 2)
            for (int V = -1; V <= 17; V += 2) {
              const int128 den = fp.den(U, V);
              b.max_den = std::max(b.max_den, den < 0 ? -den : den);
              for (int c = 0; c < 2; ++c) {
                const int128 num = fp.num(c, U, V);
                b.max_num = std::max(b.max_num, num < 0 ? -num : num);
              }
            }
        }
      }
      for (bool u : used) b.unpaired += !u;
    }
  return b;
}

int bits(int128 v) {
  int n = 0;
  while (v > 0) v >>= 1, ++n;
  return n + 1;  // signed
}

void print_projection(const std::string& what, const ProjectionBudget& b) {
  std::cout << "[ proj  ] " << std::left << std::setw(30) << what << std::right << " points off "
            << b.points_off << "/" << b.points << " (max " << b.max_lsb << " lsb), codes changed "
            << b.codes_changed << ", ok changed " << b.ok_changed << ", margin max change " << b.max_margin_change
            << ", tags lost " << b.lost << " gained " << b.gained << ", unpaired reads " << b.unpaired << ", invalid " << b.invalid;
  if (b.max_a) std::cout << "; widths a " << bits(b.max_a) << " g " << bits(b.max_g) << " num " << bits(b.max_num)
                         << " den " << bits(b.max_den);
  std::cout << "\n";
}

}  // namespace

TEST(FixedPoint, DISABLED_ProjectionSweep) {
  {
    Quantization rpu = kStage4Widths;
    rpu.homography_bits = 14, rpu.proj_scale_frac = 0, rpu.proj_persp_frac = 0;
    const ProjectionBudget b = measure_projection(rpu);
    std::cout << "[ proj  ] " << b.reads << " reads in " << b.frames << " frames, at most " << b.max_reads_per_frame
              << " per frame\n";
    print_projection("RPU, 14 significant bits", b);
  }
  for (int sf : {8, 10, 12, 14})
    for (int pf : {20, 24, 28}) {
      Quantization q = kStage4Widths;
      q.proj_scale_frac = sf, q.proj_persp_frac = pf;
      print_projection("PL, scale " + std::to_string(sf) + " persp " + std::to_string(pf), measure_projection(q));
    }
}
