#include "gv/model/refine.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace gv::model {

namespace {

using Pt = std::array<double, 2>;

// Bilinear sample at continuous coordinates (pixel centers at +0.5), clamped
// to the image.
double sample(const Image8& im, double x, double y, const Quantization& q) {
  const double fx = std::clamp(x - 0.5, 0.0, im.width - 1.0);
  const double fy = std::clamp(y - 0.5, 0.0, im.height - 1.0);
  const int x0 = std::min(static_cast<int>(fx), im.width - 2 < 0 ? 0 : im.width - 2);
  const int y0 = std::min(static_cast<int>(fy), im.height - 2 < 0 ? 0 : im.height - 2);
  const int x1 = std::min(x0 + 1, im.width - 1), y1 = std::min(y0 + 1, im.height - 1);
  const double ax = quantize(fx - x0, q.weight_frac), ay = quantize(fy - y0, q.weight_frac);
  const double top = im.at(x0, y0) * (1 - ax) + im.at(x1, y0) * ax;
  const double bot = im.at(x0, y1) * (1 - ax) + im.at(x1, y1) * ax;
  return quantize(top * (1 - ay) + bot * ay, q.value_frac);
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

// One sample point's edge offset along the outward normal, or nullopt: the
// PL edge sampler's inner loop (see EdgeJob). The profile is sampled every
// 0.5 px; the gradient at j is the central difference. The strongest lobe of
// the wanted sign within the window wins, and its centroid (down to 5% of the
// peak, as NAPA's lobe_centroid) is the edge. Comparisons are written so they
// are exact in fixed point (20 * score > peak, not score > 0.05 * peak).
std::optional<double> find_edge(const Image8& im, const Pt& at, const Pt& n, const EdgeJob& job,
                                const Quantization& q) {
  constexpr double kStep = 0.5;
  const int half = job.half_steps;
  auto pos = [&](int j) {
    return Pt{quantize(at[0] + n[0] * j * kStep, q.coord_frac), quantize(at[1] + n[1] * j * kStep, q.coord_frac)};
  };
  // A search line that leaves the image gives no sample: clamped pixels would
  // fake an edge along the frame border (a tag cut by the frame edge then
  // gets dragged corners).
  for (const int j : {-half, half}) {
    const Pt e = pos(j);
    if (e[0] < 0.5 || e[1] < 0.5 || e[0] > im.width - 0.5 || e[1] > im.height - 0.5) return std::nullopt;
  }
  std::vector<double> prof(static_cast<size_t>(2 * half + 1));
  for (int j = -half; j <= half; ++j) {
    const Pt e = pos(j);
    prof[static_cast<size_t>(j + half)] = sample(im, e[0], e[1], q);
  }
  // Outward: dark (inside the border) to bright (outside) is a positive gradient.
  std::vector<double> g(prof.size(), 0);
  for (size_t j = 1; j + 1 < prof.size(); ++j) g[j] = (prof[j + 1] - prof[j - 1]) / 2;
  auto score = [&](size_t j) { return job.check_direction ? g[j] : std::abs(g[j]); };
  size_t best = 0;
  double peak = 0;
  for (size_t j = 1; j + 1 < g.size(); ++j)
    if (std::abs(static_cast<int>(j) - half) <= job.window_steps && score(j) > peak) peak = score(j), best = j;
  if (peak <= 0) return std::nullopt;
  size_t lo = best, hi = best;
  while (lo > 1 && 20 * score(lo - 1) > peak) --lo;
  while (hi + 2 < g.size() && 20 * score(hi + 1) > peak) ++hi;
  double sum = 0, wsum = 0;
  for (size_t j = lo; j <= hi; ++j) sum += score(j) * (static_cast<double>(j) - half) * kStep, wsum += score(j);
  return quantize(sum / wsum, q.coord_frac);
}

}  // namespace

std::vector<std::array<double, 2>> sample_edge(const Image8& im, const EdgeJob& job, const Quantization& q) {
  std::vector<Pt> pts;
  for (int i = 0; i < job.count; ++i) {
    const double t = job.t0 + i * job.spacing;
    const Pt at{quantize(job.a[0] + job.d[0] * t, q.coord_frac), quantize(job.a[1] + job.d[1] * t, q.coord_frac)};
    if (auto off = find_edge(im, at, job.n, job, q))
      pts.push_back({quantize(at[0] + job.n[0] * *off, q.coord_frac), quantize(at[1] + job.n[1] * *off, q.coord_frac)});
  }
  return pts;
}

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
  const Quantization& qz = p.quant;
  auto c = rough.corners;  // as the PL receives them
  for (auto& corner : c)
    for (double& v : corner) v = quantize(v, qz.coord_frac);
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
    // Setup (RPU): the edge job.
    const Pt& a = c[k];
    const Pt& b = c[(k + 1) % 4];
    const double len = std::hypot(b[0] - a[0], b[1] - a[1]);
    EdgeJob job;
    job.a = a;
    job.d = {quantize((b[0] - a[0]) / len, qz.direction_frac), quantize((b[1] - a[1]) / len, qz.direction_frac)};
    job.n = {job.d[1], -job.d[0]};  // a normal; flip it to point away from the center
    const Pt mid{(a[0] + b[0]) / 2, (a[1] + b[1]) / 2};
    if ((mid[0] - center[0]) * job.n[0] + (mid[1] - center[1]) * job.n[1] < 0) job.n = {-job.n[0], -job.n[1]};
    const double skip = std::min(0.5 * cell, len / 4);
    job.t0 = quantize(skip, qz.coord_frac);
    job.spacing = p.sample_spacing;
    job.count = 0;
    for (double t = job.t0; t <= len - skip + 1e-9; t += job.spacing) ++job.count;
    job.half_steps = static_cast<int>(std::ceil(out.window / 0.5)) + 1;
    job.window_steps = static_cast<int>(std::floor(2 * out.window + 1e-9));
    job.check_direction = p.check_direction;

    // Sampler (PL).
    const std::vector<Pt> pts = sample_edge(im, job, qz);
    if (p.trace) (*p.trace)(job, pts);

    // Fit (RPU).
    auto line = fit_line(pts);
    if (!line) return std::nullopt;
    std::vector<Pt> kept;
    for (const Pt& pt : pts)
      if (distance(*line, pt) <= p.outlier_px) kept.push_back(pt);
    if (static_cast<int>(kept.size()) < p.min_points) return std::nullopt;
    line = fit_line(kept);
    lines[k] = *line;
    out.points_used[k] = static_cast<int>(kept.size());
  }
  // Corner k is where edge k-1 (into it) meets edge k (out of it).
  for (size_t k = 0; k < 4; ++k) {
    auto q = intersect(lines[(k + 3) % 4], lines[k]);
    if (!q) return std::nullopt;
    out.corners[k] = {quantize((*q)[0], qz.coord_frac), quantize((*q)[1], qz.coord_frac)};
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
