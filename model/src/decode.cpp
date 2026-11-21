#include "gv/model/decode.h"

#include <cmath>

namespace gv::model {

namespace {

constexpr int kBorderWidth = 8;   // tag36h11 width_at_border
constexpr int kTotalWidth = 10;   // tag36h11 total_width (with the white ring)

// Least-squares plane v = c0*x + c1*y + c2 (AprilTag's graymodel).
struct Plane {
  double a[3][3] = {}, b[3] = {}, c[3] = {};
  int n = 0;
  void add(double x, double y, double v) {
    const double j[3] = {x, y, 1};
    for (int r = 0; r < 3; ++r) {
      for (int k = 0; k < 3; ++k) a[r][k] += j[r] * j[k];
      b[r] += j[r] * v;
    }
    ++n;
  }
  bool solve() {
    // Cramer's rule on the 3x3 normal equations.
    auto det3 = [](const double m[3][3]) {
      return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
             m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
             m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    const double d = det3(a);
    if (n < 3 || std::abs(d) < 1e-12) return false;
    for (int col = 0; col < 3; ++col) {
      double m[3][3];
      for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 3; ++k) m[r][k] = k == col ? b[r] : a[r][k];
      c[col] = det3(m) / d;
    }
    return true;
  }
  double at(double x, double y) const { return c[0] * x + c[1] * y + c[2]; }
};

// Bilinear value with pixel centers at +0.5, or nullopt outside the image
// (AprilTag's value_for_pixel).
std::optional<double> bilinear(const Image8& im, double px, double py) {
  const int x1 = static_cast<int>(std::floor(px - 0.5)), x2 = static_cast<int>(std::ceil(px - 0.5));
  const int y1 = static_cast<int>(std::floor(py - 0.5)), y2 = static_cast<int>(std::ceil(py - 0.5));
  if (x1 < 0 || x2 >= im.width || y1 < 0 || y2 >= im.height) return std::nullopt;
  const double x = px - 0.5 - x1, y = py - 0.5 - y1;
  return im.at(x1, y1) * (1 - x) * (1 - y) + im.at(x2, y1) * x * (1 - y) + im.at(x1, y2) * (1 - x) * y +
         im.at(x2, y2) * x * y;
}

}  // namespace

Homography square_to_quad(const std::array<std::array<double, 2>, 4>& q) {
  const double x0 = q[0][0], y0 = q[0][1], x1 = q[1][0], y1 = q[1][1];
  const double x2 = q[2][0], y2 = q[2][1], x3 = q[3][0], y3 = q[3][1];
  const double sx = x0 - x1 + x2 - x3, sy = y0 - y1 + y2 - y3;
  const double dx1 = x1 - x2, dx2 = x3 - x2, dy1 = y1 - y2, dy2 = y3 - y2;
  const double den = dx1 * dy2 - dx2 * dy1;
  const double g = (sx * dy2 - dx2 * sy) / den;
  const double h = (dx1 * sy - sx * dy1) / den;
  return {x1 - x0 + g * x1, x3 - x0 + h * x3, x0, y1 - y0 + g * y1, y3 - y0 + h * y3, y0, g, h, 1.0};
}

std::array<double, 2> project(const Homography& h, double u, double v) {
  const double w = h[6] * u + h[7] * v + h[8];
  return {(h[0] * u + h[1] * v + h[2]) / w, (h[3] * u + h[4] * v + h[5]) / w};
}

std::array<double, 2> project_tag(const Homography& h, double tag_x, double tag_y) {
  return project(h, (tag_x + 1) / 2, (tag_y + 1) / 2);
}

std::optional<CodeRead> read_code(const Image8& im, const std::array<std::array<double, 2>, 4>& quad,
                                  const Tag36h11Lookup& family, double sharpening) {
  const Homography H = square_to_quad(quad);
  // Border samples, in cell coordinates (0..8 across the black border):
  // {start x, start y, step x, step y, white?}, as AprilTag's patterns[].
  const double w = kBorderWidth;
  const double patterns[8][5] = {{-0.5, 0.5, 0, 1, 1},    {0.5, 0.5, 0, 1, 0},     {w + 0.5, 0.5, 0, 1, 1},
                                 {w - 0.5, 0.5, 0, 1, 0}, {0.5, -0.5, 1, 0, 1},    {0.5, 0.5, 1, 0, 0},
                                 {0.5, w + 0.5, 1, 0, 1}, {0.5, w - 0.5, 1, 0, 0}};
  Plane white, black;
  for (const auto& p : patterns) {
    for (int i = 0; i < kBorderWidth; ++i) {
      const double tx = 2 * ((p[0] + i * p[2]) / w - 0.5), ty = 2 * ((p[1] + i * p[3]) / w - 0.5);
      const auto px = project_tag(H, tx, ty);
      // The pixel containing the point (AprilTag truncates, it doesn't round).
      const int ix = static_cast<int>(px[0]), iy = static_cast<int>(px[1]);
      if (px[0] < 0 || px[1] < 0 || ix >= im.width || iy >= im.height) continue;
      (p[4] != 0 ? white : black).add(tx, ty, im.at(ix, iy));
    }
  }
  if (!white.solve() || !black.solve()) return std::nullopt;
  if (white.at(0, 0) - black.at(0, 0) < 0) return std::nullopt;  // reversed border: not this family

  // Cell values on the 10x10 grid (data cells only; the rest stay 0).
  constexpr int kMin = (kBorderWidth - kTotalWidth) / 2;  // -1
  std::array<double, kTotalWidth * kTotalWidth> values{};
  for (int k = 0; k < Tag36h11Lookup::kBits; ++k) {
    const int bx = family.cell_x(k), by = family.cell_y(k);
    const double tx = 2 * ((bx + 0.5) / w - 0.5), ty = 2 * ((by + 0.5) / w - 0.5);
    const auto px = project_tag(H, tx, ty);
    const auto v = bilinear(im, px[0], px[1]);
    if (!v) continue;
    values[static_cast<size_t>((by - kMin) * kTotalWidth + (bx - kMin))] = *v - (black.at(tx, ty) + white.at(tx, ty)) / 2;
  }
  // Laplacian sharpening, clipped at the grid edge.
  std::array<double, kTotalWidth * kTotalWidth> sharp{};
  for (int y = 0; y < kTotalWidth; ++y)
    for (int x = 0; x < kTotalWidth; ++x) {
      double s = 4 * values[static_cast<size_t>(y * kTotalWidth + x)];
      if (y > 0) s -= values[static_cast<size_t>((y - 1) * kTotalWidth + x)];
      if (y + 1 < kTotalWidth) s -= values[static_cast<size_t>((y + 1) * kTotalWidth + x)];
      if (x > 0) s -= values[static_cast<size_t>(y * kTotalWidth + x - 1)];
      if (x + 1 < kTotalWidth) s -= values[static_cast<size_t>(y * kTotalWidth + x + 1)];
      sharp[static_cast<size_t>(y * kTotalWidth + x)] = s;
    }
  CodeRead r;
  double white_score = 0, black_score = 0, white_n = 1, black_n = 1;
  for (int k = 0; k < Tag36h11Lookup::kBits; ++k) {
    const auto i = static_cast<size_t>((family.cell_y(k) - kMin) * kTotalWidth + (family.cell_x(k) - kMin));
    const double v = values[i] + sharpening * sharp[i];
    r.code <<= 1;
    if (v > 0) {
      r.code |= 1;
      white_score += v, white_n += 1;
    } else {
      black_score -= v, black_n += 1;
    }
  }
  r.decision_margin = std::min(white_score / white_n, black_score / black_n);
  return r;
}

}  // namespace gv::model
