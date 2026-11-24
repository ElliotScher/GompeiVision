#include "gv/model/quad.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace gv::model {

namespace {

// Integer pixel coordinates. Every quantity below is a difference or a
// translation-invariant sum, so it's the same for pixel centers.
struct Pt {
  int64_t x = 0, y = 0;
};

int64_t dist_sq(const Pt& a, const Pt& b) { return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y); }

int64_t cross(const Pt& o, const Pt& a, const Pt& b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

// Twice the area (the shoelace sum's magnitude).
int64_t twice_area(const std::array<Pt, 4>& q) {
  int64_t s = 0;
  for (size_t i = 0; i < 4; ++i) s += q[i].x * q[(i + 1) % 4].y - q[(i + 1) % 4].x * q[i].y;
  return std::llabs(s);
}

}  // namespace

uint64_t isqrt(uint64_t v) {
  uint64_t root = 0, rem = 0;
  for (int i = 31; i >= 0; --i) {
    rem = (rem << 2) | ((v >> (2 * i)) & 3);
    const uint64_t trial = (root << 2) | 1;
    root <<= 1;
    if (rem >= trial) rem -= trial, root |= 1;
  }
  return root;
}

QuadResult scorecard_quad(const Component& c, const QuadParams& p) {
  QuadResult r;
  // 1. Extremes in direction order, merged with their neighbors (cyclically).
  // A cluster keeps the point farthest from the blob's bounding-box center:
  // on a pixel staircase near a corner, that's the one nearest the corner.
  // Distances from the center are compared in doubled coordinates, where the
  // center (x_min + x_max + 1) / 2 and pixel centers are integers.
  const int64_t cx2 = c.x_min + c.x_max + 1, cy2 = c.y_min + c.y_max + 1;
  auto reach = [&](const Pt& q) {
    const int64_t dx = 2 * q.x + 1 - cx2, dy = 2 * q.y + 1 - cy2;
    return dx * dx + dy * dy;
  };
  auto near = [&](const Pt& a, const Pt& b) { return dist_sq(a, b) <= p.merge_distance_sq; };
  std::vector<Pt> raw;
  for (const Extreme& e : c.extremes) raw.push_back({e.x, e.y});
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

  // 2. The 4 (in cyclic order) enclosing the largest area. At most C(16, 4);
  // the first in this loop order wins a tie.
  const size_t n = pts.size();
  int64_t best = -1;
  std::array<size_t, 4> pick{};
  for (size_t a = 0; a < n; ++a)
    for (size_t b = a + 1; b < n; ++b)
      for (size_t d = b + 1; d < n; ++d)
        for (size_t e = d + 1; e < n; ++e) {
          const int64_t area2 = twice_area({pts[a], pts[b], pts[d], pts[e]});
          if (area2 > best) best = area2, pick = {a, b, d, e};
        }
  std::array<Pt, 4> q;
  for (size_t k = 0; k < 4; ++k) {
    q[k] = pts[pick[k]];
    r.quad.corners[k] = {static_cast<double>(q[k].x) + 0.5, static_cast<double>(q[k].y) + 0.5};
  }
  r.quad.area = static_cast<double>(best) / 2.0;

  // 3. Sanity checks.
  std::array<int64_t, 4> len_q8{};  // floor(256 * side length)
  int64_t perimeter_q8 = 0;
  for (size_t k = 0; k < 4; ++k) {
    const int64_t d2 = dist_sq(q[k], q[(k + 1) % 4]);
    if (d2 < int64_t{p.min_side} * p.min_side) {
      r.verdict = QuadVerdict::kTooSmall;
      return r;
    }
    len_q8[k] = static_cast<int64_t>(isqrt(static_cast<uint64_t>(d2) << 16));
    perimeter_q8 += len_q8[k];
  }
  // Off-quad: an extreme q fails if cross(u, v, q) / |uv| < -limit, where
  // limit = max(max_outside, mean side / div). With M = 4 * div * limit in
  // 1/256 px, that's -cross * 4 * div * 2^16 > M * len_q8.
  // In image coordinates (y down) the direction order puts the quad's inside
  // at cross > 0 for each edge u -> v.
  const int64_t div4 = 4 * int64_t{p.max_outside_div};
  const int64_t m = std::max(int64_t{p.max_outside_q8} * div4, perimeter_q8);
  for (const Pt& e : pts) {
    for (size_t k = 0; k < 4; ++k) {
      if (-cross(q[k], q[(k + 1) % 4], e) * div4 * 65536 > m * len_q8[k]) {
        r.verdict = QuadVerdict::kExtremesOffQuad;
        return r;
      }
    }
  }
  // Fill: blob area / (quad area + perimeter / 2 + 1). The extremes are pixel
  // centers, half a pixel inside the blob's outline, so the blob is compared
  // with the quad grown by that half pixel. Times 512 the denominator is
  // 256 * twice_area + perimeter_q8 + 512, and the bounds are percentages.
  const int64_t den = 256 * best + perimeter_q8 + 512;
  const int64_t blob = int64_t{c.area} * 512 * 100;
  r.verdict = blob < p.min_fill_pct * den || blob > p.max_fill_pct * den ? QuadVerdict::kFillOutOfRange
                                                                          : QuadVerdict::kOk;
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
