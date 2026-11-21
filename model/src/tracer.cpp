#include "gv/model/tracer.h"

#include <algorithm>
#include <cmath>

namespace gv::model {

namespace {

using Pt = std::array<double, 2>;

double cross(const Pt& o, const Pt& a, const Pt& b) {
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

// Andrew's monotone chain; returns the hull counter-clockwise in math axes,
// which is clockwise on screen (y down), like 4B's quads.
std::vector<Pt> convex_hull(std::vector<Pt> pts) {
  std::sort(pts.begin(), pts.end());
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  if (pts.size() < 3) return pts;
  std::vector<Pt> h(2 * pts.size());
  size_t k = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
    h[k++] = pts[i];
  }
  for (size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(h[k - 2], h[k - 1], pts[i - 1]) <= 0) --k;
    h[k++] = pts[i - 1];
  }
  h.resize(k - 1);
  return h;
}

double quad_area(const Pt& a, const Pt& b, const Pt& c, const Pt& d) {
  return std::abs(cross(a, b, c) + cross(a, c, d)) / 2;
}

double line_distance(const Pt& u, const Pt& v, const Pt& q) {
  return std::abs(cross(u, v, q)) / std::hypot(v[0] - u[0], v[1] - u[1]);
}

}  // namespace

std::vector<Pt> trace_boundary(const Labeling& l, uint32_t id, size_t max_points) {
  const int w = l.width, h = l.height;
  auto in = [&](int x, int y) {
    return x >= 0 && y >= 0 && x < w && y < h && l.label[static_cast<size_t>(y * w + x)] == id;
  };
  const int sx = static_cast<int>(id % static_cast<uint32_t>(w)), sy = static_cast<int>(id / static_cast<uint32_t>(w));
  // Moore neighborhood, clockwise on screen starting west.
  static constexpr int dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
  static constexpr int dy[8] = {0, -1, -1, -1, 0, 1, 1, 1};
  // Radial sweep: from each boundary pixel, scan clockwise starting just
  // after the pixel we came from. Stop on returning to the start pixel about
  // to repeat the first move (Jacob's criterion), so thin parts that pass
  // through the start pixel twice are still traced.
  std::vector<Pt> out{{sx + 0.5, sy + 0.5}};
  auto next = [&](int x, int y, int from) {
    for (int i = 1; i <= 8; ++i) {
      const int d = (from + i) % 8;
      if (in(x + dx[d], y + dy[d])) return d;
    }
    return -1;
  };
  const int first = next(sx, sy, 0);  // entered from the west (raster order)
  if (first < 0) return out;          // a single pixel
  int x = sx + dx[first], y = sy + dy[first], from = (first + 4) % 8;
  for (size_t steps = 0; steps < 4 * max_points; ++steps) {
    if (x == sx && y == sy && next(x, y, from) == first) break;
    out.push_back({x + 0.5, y + 0.5});
    if (out.size() >= max_points) break;
    const int d = next(x, y, from);
    x += dx[d], y += dy[d];
    from = (d + 4) % 8;
  }
  return out;
}

std::optional<Quad> trace_quad(const Labeling& l, const Component& c, const TracerParams& p) {
  const auto boundary = trace_boundary(l, c.id, p.max_boundary);
  if (boundary.size() < 8 || boundary.size() >= p.max_boundary) return std::nullopt;
  const auto hull = convex_hull(boundary);
  const size_t n = hull.size();
  if (n < 4) return std::nullopt;
  double best = -1;
  std::array<size_t, 4> pick{};
  for (size_t a = 0; a < n; ++a)
    for (size_t b = a + 1; b < n; ++b)
      for (size_t d = b + 1; d < n; ++d)
        for (size_t e = d + 1; e < n; ++e) {
          const double area = quad_area(hull[a], hull[b], hull[d], hull[e]);
          if (area > best) best = area, pick = {a, b, d, e};
        }
  Quad q;
  for (size_t k = 0; k < 4; ++k) q.corners[k] = hull[pick[k]];
  q.area = best;
  for (size_t k = 0; k < 4; ++k) {
    const Pt& u = q.corners[k];
    const Pt& v = q.corners[(k + 1) % 4];
    if (std::hypot(v[0] - u[0], v[1] - u[1]) < p.min_side) return std::nullopt;
  }
  // Each boundary point belongs to its nearest edge; every edge must fit.
  std::array<double, 4> sum{};
  std::array<int, 4> count{};
  for (const Pt& b : boundary) {
    size_t e = 0;
    double dmin = 1e18;
    for (size_t k = 0; k < 4; ++k) {
      const double dist = line_distance(q.corners[k], q.corners[(k + 1) % 4], b);
      if (dist < dmin) dmin = dist, e = k;
    }
    sum[e] += dmin * dmin, count[e]++;
  }
  for (size_t k = 0; k < 4; ++k) {
    const Pt& u = q.corners[k];
    const Pt& v = q.corners[(k + 1) % 4];
    const double len = std::hypot(v[0] - u[0], v[1] - u[1]);
    if (count[k] < 3) return std::nullopt;
    const double rms = std::sqrt(sum[k] / count[k]);
    if (rms > std::max(p.max_rms, p.max_rms_frac * len)) return std::nullopt;
  }
  return q;
}

}  // namespace gv::model
