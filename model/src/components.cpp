#include "gv/model/components.h"

#include <algorithm>
#include <limits>
#include <numeric>

#include "gv/model/threshold.h"

namespace gv::model {

JoinMask reference_joins(const Image8& t, int x, int y) {
  JoinMask m;
  const uint8_t v = t.at(x, y);
  if (v == kSkip || x < 1 || x > t.width - 2) return m;
  m.left = true;
  if (y == 0) return m;
  // Same tests as do_unionfind_line2, written with named neighbors.
  const uint8_t left = t.at(x - 1, y);
  const uint8_t up_left = t.at(x - 1, y - 1);
  const uint8_t up = t.at(x, y - 1);
  const uint8_t up_right = t.at(x + 1, y - 1);
  m.up = x == 1 || !(left == up_left && up_left == up);
  if (v == 255) {
    m.up_left = x == 1 || !(left == up_left || up == up_left);
    m.up_right = up != up_right;
  }
  return m;
}

Component new_component(uint32_t id, uint8_t color, int x, int y) {
  Component c{id, color, 1, x, x, y, y, 0, {}};
  for (int k = 0; k < kDirections; ++k) {
    const auto& d = kDirectionVectors[static_cast<size_t>(k)];
    c.extremes[static_cast<size_t>(k)] = {x, y, d[0] * x + d[1] * y};
  }
  return c;
}

void add_pixel(Component& c, int x, int y) {
  c.area += 1;
  c.x_min = std::min(c.x_min, x);
  c.x_max = std::max(c.x_max, x);
  c.y_min = std::min(c.y_min, y);
  c.y_max = std::max(c.y_max, y);
  for (int k = 0; k < kDirections; ++k) {
    const auto& d = kDirectionVectors[static_cast<size_t>(k)];
    Extreme& e = c.extremes[static_cast<size_t>(k)];
    const int32_t s = d[0] * x + d[1] * y;
    if (s > e.score) e = {x, y, s};  // pixels arrive in raster order: ties keep the earlier one
  }
}

void merge(Component& a, const Component& b, int width) {
  a.id = std::min(a.id, b.id);
  a.area += b.area;
  a.x_min = std::min(a.x_min, b.x_min);
  a.x_max = std::max(a.x_max, b.x_max);
  a.y_min = std::min(a.y_min, b.y_min);
  a.y_max = std::max(a.y_max, b.y_max);
  for (size_t k = 0; k < kDirections; ++k) {
    Extreme& e = a.extremes[k];
    const Extreme& f = b.extremes[k];
    if (f.score > e.score || (f.score == e.score && f.y * width + f.x < e.y * width + e.x)) e = f;
  }
}

namespace {

constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();

// Equivalence table over provisional labels, each root holding its
// component's accumulator. The RTL version recycles labels; the model doesn't
// need to.
class Equivalences {
 public:
  explicit Equivalences(int width) : width_(width) {}
  uint32_t create(uint32_t raster, uint8_t color, int x, int y) {
    const auto id = static_cast<uint32_t>(parent_.size());
    parent_.push_back(id);
    acc_.push_back(new_component(raster, color, x, y));
    emitted_.push_back(false);
    return id;
  }
  uint32_t find(uint32_t a) {
    while (parent_[a] != a) a = parent_[a] = parent_[parent_[a]];
    return a;
  }
  void join(uint32_t a, uint32_t b) {
    a = find(a);
    b = find(b);
    if (a == b) return;
    if (acc_[b].id < acc_[a].id) std::swap(a, b);  // keep the root with the smaller id
    parent_[b] = a;
    merge(acc_[a], acc_[b], width_);
  }
  void grow(uint32_t a, int x, int y) { add_pixel(acc_[find(a)], x, y); }
  Component& acc(uint32_t root) { return acc_[root]; }
  bool take_emit(uint32_t root) {
    if (emitted_[root]) return false;
    emitted_[root] = true;
    return true;
  }

 private:
  int width_;
  std::vector<uint32_t> parent_;
  std::vector<Component> acc_;
  std::vector<bool> emitted_;
};

}  // namespace

Labeling label_components(const Image8& t) {
  const int w = t.width, h = t.height;
  Labeling out;
  out.width = w;
  out.height = h;
  Equivalences eq(w);
  std::vector<uint32_t> prev(static_cast<size_t>(w), kNone), cur(static_cast<size_t>(w), kNone);
  std::vector<uint32_t> provisional(t.pixels.size(), kNone);  // for the final per-pixel labels

  auto close_from_row = [&](const std::vector<uint32_t>& row, int y_closed, bool frame_end) {
    for (uint32_t p : row) {
      if (p == kNone) continue;
      const uint32_t r = eq.find(p);
      Component& c = eq.acc(r);
      if ((frame_end || c.y_max < y_closed) && eq.take_emit(r)) {
        c.closed_after_row = y_closed;
        out.closed.push_back(c);
      }
    }
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const uint8_t v = t.at(x, y);
      const auto ux = static_cast<size_t>(x);
      if (v == kSkip) {
        cur[ux] = kNone;
        continue;
      }
      // Each pixel starts a provisional label, then joins whatever the
      // reference would join. All neighbors are earlier in raster order.
      uint32_t p = kNone;
      const JoinMask m = reference_joins(t, x, y);
      // Reads the neighbor only when the reference would join it: (nx, ny) is
      // out of the image exactly when the mask bit is false.
      auto try_join = [&](bool want, int nx, int ny) {
        if (!want) return;
        const std::vector<uint32_t>& row = ny == y ? cur : prev;
        const uint32_t neighbor = row[static_cast<size_t>(nx)];
        if (neighbor == kNone || t.at(nx, ny) != v) return;
        if (p == kNone) {
          p = neighbor;
          eq.grow(p, x, y);
        } else {
          eq.join(p, neighbor);
        }
      };
      try_join(m.left, x - 1, y);
      try_join(m.up, x, y - 1);
      try_join(m.up_left, x - 1, y - 1);
      try_join(m.up_right, x + 1, y - 1);
      if (p == kNone) p = eq.create(static_cast<uint32_t>(y * w + x), v, x, y);
      cur[ux] = p;
      provisional[t.index(x, y)] = p;
    }
    // Components with pixels in row y-1 and none in row y can't grow again.
    if (y > 0) close_from_row(prev, y, false);
    std::swap(prev, cur);
  }
  close_from_row(prev, h - 1, true);

  out.label.resize(t.pixels.size());
  for (size_t i = 0; i < t.pixels.size(); ++i)
    out.label[i] = provisional[i] == kNone ? static_cast<uint32_t>(i) : eq.acc(eq.find(provisional[i])).id;
  return out;
}

}  // namespace gv::model
