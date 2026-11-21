#include "gv/model/quad.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace gv::model {

namespace {

using Pt = std::array<double, 2>;

double cross(const Pt& o, const Pt& a, const Pt& b) {
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

double polygon_area(const std::array<Pt, 4>& q) {
  double s = 0;
  for (size_t i = 0; i < 4; ++i) s += q[i][0] * q[(i + 1) % 4][1] - q[(i + 1) % 4][0] * q[i][1];
  return std::abs(s) / 2;
}

}  // namespace

QuadResult scorecard_quad(const Component& c, const QuadParams& p) {
  QuadResult r;
  // 1. Extremes in direction order, merged with their neighbors (cyclically).
  // A cluster keeps the point farthest from the blob's bounding-box center:
  // on a pixel staircase near a corner, that's the one nearest the corner.
  const Pt center{(c.x_min + c.x_max + 1) / 2.0, (c.y_min + c.y_max + 1) / 2.0};
  auto reach = [&](const Pt& q) { return std::hypot(q[0] - center[0], q[1] - center[1]); };
  auto near = [&](const Pt& a, const Pt& b) { return std::hypot(a[0] - b[0], a[1] - b[1]) <= p.merge_distance; };
  std::vector<Pt> raw;
  for (const Extreme& e : c.extremes) raw.push_back({e.x + 0.5, e.y + 0.5});
  // Start the cyclic scan at a point that doesn't merge with its predecessor.
  size_t start = 0;
  while (start < raw.size() && near(raw[start], raw[(start + raw.size() - 1) % raw.size()])) ++start;
  if (start == raw.size()) return r;  // everything within merge distance: a dot
  std::vector<Pt> pts;
  Pt anchor{}, keep{};
  for (size_t i = 0; i < raw.size(); ++i) {
    const Pt& q = raw[(start + i) % raw.size()];
    if (i > 0 && near(q, anchor)) {
      if (reach(q) > reach(keep)) keep = q;
      continue;
    }
    if (i > 0) pts.push_back(keep);
    anchor = keep = q;
  }
  pts.push_back(keep);
  if (pts.size() < 4) return r;

  // 2. The 4 (in cyclic order) enclosing the largest area. At most C(16, 4).
  const size_t n = pts.size();
  double best = -1;
  std::array<size_t, 4> pick{};
  for (size_t a = 0; a < n; ++a)
    for (size_t b = a + 1; b < n; ++b)
      for (size_t d = b + 1; d < n; ++d)
        for (size_t e = d + 1; e < n; ++e) {
          const double area = polygon_area({pts[a], pts[b], pts[d], pts[e]});
          if (area > best) best = area, pick = {a, b, d, e};
        }
  for (size_t k = 0; k < 4; ++k) r.quad.corners[k] = pts[pick[k]];
  r.quad.area = best;

  // 3. Sanity checks.
  for (size_t k = 0; k < 4; ++k) {
    const Pt& u = r.quad.corners[k];
    const Pt& v = r.quad.corners[(k + 1) % 4];
    if (std::hypot(v[0] - u[0], v[1] - u[1]) < p.min_side) {
      r.verdict = QuadVerdict::kTooSmall;
      return r;
    }
  }
  // In image coordinates (y down) the direction order puts the quad's inside
  // at cross > 0 for each edge u -> v.
  double sides = 0;
  for (size_t k = 0; k < 4; ++k)
    sides += std::hypot(r.quad.corners[(k + 1) % 4][0] - r.quad.corners[k][0],
                        r.quad.corners[(k + 1) % 4][1] - r.quad.corners[k][1]);
  const double outside_limit = std::max(p.max_outside, p.max_outside_frac * sides / 4);
  for (const Pt& q : pts) {
    for (size_t k = 0; k < 4; ++k) {
      const Pt& u = r.quad.corners[k];
      const Pt& v = r.quad.corners[(k + 1) % 4];
      const double len = std::hypot(v[0] - u[0], v[1] - u[1]);
      if (cross(u, v, q) / len < -outside_limit) {
        r.verdict = QuadVerdict::kExtremesOffQuad;
        return r;
      }
    }
  }
  // The extremes are pixel centers, half a pixel inside the blob's outline:
  // compare the blob with the quad grown by that half pixel.
  double perimeter = 0;
  for (size_t k = 0; k < 4; ++k) {
    const Pt& u = r.quad.corners[k];
    const Pt& v = r.quad.corners[(k + 1) % 4];
    perimeter += std::hypot(v[0] - u[0], v[1] - u[1]);
  }
  const double fill = c.area / (r.quad.area + 0.5 * perimeter + 1.0);
  r.verdict = fill < p.min_fill || fill > p.max_fill ? QuadVerdict::kFillOutOfRange : QuadVerdict::kOk;
  return r;
}

const char* to_string(QuadVerdict v) {
  switch (v) {
    case QuadVerdict::kOk: return "ok";
    case QuadVerdict::kTooFewCorners: return "too few corners";
    case QuadVerdict::kTooSmall: return "too small";
    case QuadVerdict::kExtremesOffQuad: return "extremes off quad";
    case QuadVerdict::kFillOutOfRange: return "fill out of range";
  }
  return "?";
}

}  // namespace gv::model
