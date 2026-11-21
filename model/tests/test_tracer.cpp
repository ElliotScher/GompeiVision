#include <gtest/gtest.h>

#include <cmath>
#include <functional>

#include "gv/model/components.h"
#include "gv/model/tracer.h"

using namespace gv::model;

namespace {

struct Shape {
  Labeling l;
  Component c;
};

Shape black_shape(int w, int h, const std::function<bool(int, int)>& inside, int sx, int sy) {
  Image8 t(w, h, 255);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      if (inside(x, y)) t.at(x, y) = 0;
  Shape s{label_components(t), {}};
  const uint32_t id = s.l.label[t.index(sx, sy)];
  for (const Component& c : s.l.closed)
    if (c.id == id) s.c = c;
  return s;
}

}  // namespace

TEST(Tracer, BoundaryOfASquareVisitsEveryEdgePixelOnce) {
  const Shape s = black_shape(20, 20, [](int x, int y) { return x >= 5 && x <= 14 && y >= 5 && y <= 14; }, 5, 5);
  const auto b = trace_boundary(s.l, s.c.id);
  EXPECT_EQ(b.size(), 36u);  // a 10x10 square has 36 boundary pixels
  for (const auto& p : b) {
    const int x = static_cast<int>(p[0]), y = static_cast<int>(p[1]);
    EXPECT_TRUE(x == 5 || x == 14 || y == 5 || y == 14) << x << "," << y;
  }
}

TEST(Tracer, ThinShapeThroughTheStartIsTracedFully) {
  // A plus sign whose first pixel joins two arms: stopping at the first return
  // to the start would miss half of it.
  const Shape s = black_shape(21, 21, [](int x, int y) { return (x == 10 && y >= 2 && y <= 18) || (y == 10 && x >= 2 && x <= 18); }, 10, 2);
  const auto b = trace_boundary(s.l, s.c.id);
  int left = 0, right = 0;
  for (const auto& p : b) left += p[0] < 5, right += p[0] > 15;
  EXPECT_GT(left, 0);
  EXPECT_GT(right, 0);
}

TEST(Tracer, SquareRingWithBumpsGivesItsQuad) {
  // A tag-like ring with a 2-pixel bump on one side: 4B's off-quad check
  // could reject it, the tracer accepts the quad.
  auto inside = [](int x, int y) {
    const bool outer = x >= 10 && x <= 49 && y >= 10 && y <= 49;
    const bool hole = x >= 15 && x <= 44 && y >= 15 && y <= 44;
    const bool bump = y >= 8 && y <= 9 && x >= 28 && x <= 31;
    return (outer && !hole) || bump;
  };
  const Shape s = black_shape(60, 60, inside, 12, 12);
  const auto q = trace_quad(s.l, s.c);
  ASSERT_TRUE(q);
  for (const auto& c : q->corners) {
    const double ex = std::min(std::abs(c[0] - 10.5), std::abs(c[0] - 49.5));
    const double ey = std::min(std::abs(c[1] - 10.5), std::abs(c[1] - 49.5));
    EXPECT_LT(std::hypot(ex, ey), 1.5) << c[0] << "," << c[1];
  }
}

TEST(Tracer, LShapeIsRejected) {
  const Shape s = black_shape(60, 60, [](int x, int y) {
    return (x >= 10 && x < 50 && y >= 10 && y < 25) || (x >= 10 && x < 25 && y >= 10 && y < 50);
  }, 15, 15);
  EXPECT_FALSE(trace_quad(s.l, s.c));
}
