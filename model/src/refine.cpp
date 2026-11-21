#include "gv/model/refine.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace gv::model {

namespace {

using Pt = std::array<double, 2>;

// Bilinear sample at continuous coordinates (pixel centers at +0.5), clamped
// to the image.
double sample(const Image8& im, double x, double y) {
  const double fx = std::clamp(x - 0.5, 0.0, im.width - 1.0);
  const double fy = std::clamp(y - 0.5, 0.0, im.height - 1.0);
  const int x0 = std::min(static_cast<int>(fx), im.width - 2 < 0 ? 0 : im.width - 2);
  const int y0 = std::min(static_cast<int>(fy), im.height - 2 < 0 ? 0 : im.height - 2);
  const int x1 = std::min(x0 + 1, im.width - 1), y1 = std::min(y0 + 1, im.height - 1);
  const double ax = fx - x0, ay = fy - y0;
  const double top = im.at(x0, y0) * (1 - ax) + im.at(x1, y0) * ax;
  const double bot = im.at(x0, y1) * (1 - ax) + im.at(x1, y1) * ax;
  return top * (1 - ay) + bot * ay;
}

// Line through points as (point, unit direction), by total least squares.
struct Line {
  Pt p{}, d{};
};

std::optional<Line> fit_line(const std::vector<Pt>& pts) {
  if (pts.size() < 2) return std::nullopt;
  double mx = 0, my = 0;
  for (const Pt& q : pts) mx += q[0], my += q[1];
  mx /= static_cast<double>(pts.size()), my /= static_cast<double>(pts.size());
  double sxx = 0, sxy = 0, syy = 0;
  for (const Pt& q : pts) {
    const double dx = q[0] - mx, dy = q[1] - my;
    sxx += dx * dx, sxy += dx * dy, syy += dy * dy;
  }
  const double theta = 0.5 * std::atan2(2 * sxy, sxx - syy);
  return Line{{mx, my}, {std::cos(theta), std::sin(theta)}};
}

double distance(const Line& l, const Pt& q) {
  return std::abs((q[0] - l.p[0]) * l.d[1] - (q[1] - l.p[1]) * l.d[0]);
}

std::optional<Pt> intersect(const Line& a, const Line& b) {
  const double den = a.d[0] * b.d[1] - a.d[1] * b.d[0];
  if (std::abs(den) < 1e-9) return std::nullopt;
  const double t = ((b.p[0] - a.p[0]) * b.d[1] - (b.p[1] - a.p[1]) * b.d[0]) / den;
  return Pt{a.p[0] + t * a.d[0], a.p[1] + t * a.d[1]};
}

// Offset along the outward normal of the edge in [-w, w], or nullopt.
// The profile is sampled every 0.5 px; the gradient at j is the central
// difference. The strongest lobe of the wanted sign wins, and its centroid
// (down to 5% of the peak, as NAPA's lobe_centroid) is the edge.
std::optional<double> find_edge(const Image8& im, const Pt& at, const Pt& n, double w, bool check_direction) {
  constexpr double kStep = 0.5;
  const int half = static_cast<int>(std::ceil(w / kStep)) + 1;
  // A search line that leaves the image gives no sample: clamped pixels would
  // fake an edge along the frame border (a tag cut by the frame edge then
  // gets dragged corners).
  for (const int j : {-half, half}) {
    const double x = at[0] + n[0] * j * kStep, y = at[1] + n[1] * j * kStep;
    if (x < 0.5 || y < 0.5 || x > im.width - 0.5 || y > im.height - 0.5) return std::nullopt;
  }
  std::vector<double> prof(static_cast<size_t>(2 * half + 1));
  for (int j = -half; j <= half; ++j)
    prof[static_cast<size_t>(j + half)] = sample(im, at[0] + n[0] * j * kStep, at[1] + n[1] * j * kStep);
  // Outward: dark (inside the border) to bright (outside) is a positive gradient.
  std::vector<double> g(prof.size(), 0);
  for (size_t j = 1; j + 1 < prof.size(); ++j) g[j] = (prof[j + 1] - prof[j - 1]) / 2;
  auto score = [&](size_t j) { return check_direction ? g[j] : std::abs(g[j]); };
  size_t best = 0;
  double peak = 0;
  for (size_t j = 1; j + 1 < g.size(); ++j)
    if (std::abs((static_cast<double>(j) - half) * kStep) <= w && score(j) > peak) peak = score(j), best = j;
  if (peak <= 0) return std::nullopt;
  size_t lo = best, hi = best;
  while (lo > 1 && score(lo - 1) > 0.05 * peak) --lo;
  while (hi + 2 < g.size() && score(hi + 1) > 0.05 * peak) ++hi;
  double sum = 0, wsum = 0;
  for (size_t j = lo; j <= hi; ++j) sum += score(j) * (static_cast<double>(j) - half) * kStep, wsum += score(j);
  return sum / wsum;
}

}  // namespace

Quad to_full_resolution(const Quad& q, int factor) {
  Quad out = q;
  for (auto& c : out.corners)
    for (double& v : c) v = (v - 0.5) * factor + 0.5;
  out.area = q.area * factor * factor;
  return out;
}

namespace {

std::optional<RefinedQuad> refine_once(const Image8& im, const Quad& rough, const RefineParams& p) {
  RefinedQuad out;
  const auto& c = rough.corners;
  Pt center{0, 0};
  double sides = 0;
  for (size_t k = 0; k < 4; ++k) {
    center[0] += c[k][0] / 4, center[1] += c[k][1] / 4;
    sides += std::hypot(c[(k + 1) % 4][0] - c[k][0], c[(k + 1) % 4][1] - c[k][1]);
  }
  const double cell = sides / 4 / 8;  // tag36h11's black border is 8 cells across
  out.window = p.window == WindowPolicy::kFixed ? p.fixed_window
                                                : std::clamp(0.5 * cell + 1, p.window_min, p.window_max);

  std::array<Line, 4> lines;
  for (size_t k = 0; k < 4; ++k) {
    const Pt& a = c[k];
    const Pt& b = c[(k + 1) % 4];
    const double len = std::hypot(b[0] - a[0], b[1] - a[1]);
    const Pt d{(b[0] - a[0]) / len, (b[1] - a[1]) / len};
    Pt n{d[1], -d[0]};  // a normal; flip it to point away from the center
    const Pt mid{(a[0] + b[0]) / 2, (a[1] + b[1]) / 2};
    if ((mid[0] - center[0]) * n[0] + (mid[1] - center[1]) * n[1] < 0) n = {-n[0], -n[1]};

    std::vector<Pt> pts;
    const double skip = std::min(0.5 * cell, len / 4);
    for (double t = skip; t <= len - skip + 1e-9; t += p.sample_spacing) {
      const Pt at{a[0] + d[0] * t, a[1] + d[1] * t};
      if (auto off = find_edge(im, at, n, out.window, p.check_direction))
        pts.push_back({at[0] + n[0] * *off, at[1] + n[1] * *off});
    }
    auto line = fit_line(pts);
    if (!line) return std::nullopt;
    std::vector<Pt> kept;
    for (const Pt& q : pts)
      if (distance(*line, q) <= p.outlier_px) kept.push_back(q);
    if (static_cast<int>(kept.size()) < p.min_points) return std::nullopt;
    line = fit_line(kept);
    lines[k] = *line;
    out.points_used[k] = static_cast<int>(kept.size());
  }
  // Corner k is where edge k-1 (into it) meets edge k (out of it).
  for (size_t k = 0; k < 4; ++k) {
    auto q = intersect(lines[(k + 3) % 4], lines[k]);
    if (!q) return std::nullopt;
    out.corners[k] = *q;
  }
  return out;
}

}  // namespace

std::optional<RefinedQuad> refine_quad(const Image8& im, const Quad& rough, const RefineParams& p) {
  Quad q = rough;
  std::optional<RefinedQuad> out;
  for (int pass = 0; pass < std::max(1, p.passes); ++pass) {
    RefineParams pp = p;
    if (pass == 0 && p.first_pass_window_max > 0) pp.window_max = p.first_pass_window_max;
    out = refine_once(im, q, pp);
    if (!out) return std::nullopt;
    q.corners = out->corners;
  }
  return out;
}

}  // namespace gv::model
