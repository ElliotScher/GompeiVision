#include "synth.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "apriltag.h"
#include "tag36h11.h"

namespace gv::test {

namespace {

// Uniform helpers over raw mt19937 output, so scenes are identical on every
// standard library (std::uniform_*_distribution is implementation-defined).
struct Rng {
  std::mt19937 g;
  explicit Rng(uint32_t seed) : g(seed) {}
  double uniform() { return static_cast<double>(g()) / 4294967296.0; }  // [0, 1)
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }
  int integer(int lo, int hi) { return lo + static_cast<int>(g() % static_cast<uint32_t>(hi - lo + 1)); }
};

// Homography mapping the unit square (0,0),(1,0),(1,1),(0,1) to quad q
// (Heckbert's closed form).
std::array<double, 9> square_to_quad(const double q[4][2]) {
  const double x0 = q[0][0], y0 = q[0][1], x1 = q[1][0], y1 = q[1][1];
  const double x2 = q[2][0], y2 = q[2][1], x3 = q[3][0], y3 = q[3][1];
  const double sx = x0 - x1 + x2 - x3, sy = y0 - y1 + y2 - y3;
  const double dx1 = x1 - x2, dx2 = x3 - x2, dy1 = y1 - y2, dy2 = y3 - y2;
  const double den = dx1 * dy2 - dx2 * dy1;
  const double g = (sx * dy2 - dx2 * sy) / den;
  const double h = (dx1 * sy - sx * dy1) / den;
  return {x1 - x0 + g * x1, x3 - x0 + h * x3, x0, y1 - y0 + g * y1, y3 - y0 + h * y3, y0, g, h, 1.0};
}

std::array<double, 9> invert3(const std::array<double, 9>& m) {
  const double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
  const double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const double det = a * A + b * B + c * C;
  return {A / det, -(b * i - c * h) / det, (b * f - c * e) / det,
          B / det, (a * i - c * g) / det,  -(a * f - c * d) / det,
          C / det, -(a * h - b * g) / det, (a * e - b * d) / det};
}

uint8_t clamp8(double v) { return static_cast<uint8_t>(std::clamp(std::lround(v), 0L, 255L)); }

}  // namespace

Scene make_scene(uint32_t seed, const SceneParams& p) {
  Rng rng(seed);
  Scene scene;
  scene.image = model::Image8(p.width, p.height);

  // Background: a lighting gradient plus a little texture.
  std::vector<double> img(static_cast<size_t>(p.width * p.height));
  const double base = rng.uniform(60, 190), gx = rng.uniform(-0.15, 0.15), gy = rng.uniform(-0.15, 0.15);
  for (int y = 0; y < p.height; ++y)
    for (int x = 0; x < p.width; ++x)
      img[static_cast<size_t>(y * p.width + x)] =
          base + gx * (x - p.width / 2.0) + gy * (y - p.height / 2.0) + 8 * std::sin(x * 0.05) * std::cos(y * 0.07);

  apriltag_family_t* fam = tag36h11_create();
  // The rendered tag is 10x10 cells: a white quiet-zone ring around the
  // 8x8 black-bordered tag.
  std::vector<std::array<double, 3>> placed_circles;  // center x, y, radius
  for (int n = 0; n < p.num_tags; ++n) {
    PlacedTag t;
    t.id = rng.integer(0, static_cast<int>(fam->ncodes) - 1);
    image_u8_t* tag = apriltag_to_image(fam, t.id);
    const int cells = tag->width;                      // 10 for 36h11
    const double border_cells = cells - 2;             // black border width in cells
    const double size = rng.uniform(p.min_tag_px, p.max_tag_px);
    const double full = size * cells / border_cells;   // including the white ring
    // Keep tags apart, so the drawn corners stay ground truth.
    double cx = 0, cy = 0;
    bool placed = false;
    for (int attempt = 0; attempt < 50 && !placed; ++attempt) {
      cx = rng.uniform(full / 2, p.width - full / 2);
      cy = rng.uniform(full / 2, p.height - full / 2);
      placed = true;
      for (const auto& [ox, oy, orad] : placed_circles)
        if (std::hypot(cx - ox, cy - oy) < orad + full * 0.75) placed = false;
    }
    if (!placed) {
      image_u8_destroy(tag);
      continue;
    }
    placed_circles.push_back({cx, cy, full * 0.75});
    const double ang = rng.uniform(0, 2 * M_PI);
    double quad[4][2];
    const double unit[4][2] = {{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}};
    for (int k = 0; k < 4; ++k) {
      const double ux = unit[k][0] + rng.uniform(-p.max_skew, p.max_skew) / 2;
      const double uy = unit[k][1] + rng.uniform(-p.max_skew, p.max_skew) / 2;
      quad[k][0] = cx + full * (ux * std::cos(ang) - uy * std::sin(ang));
      quad[k][1] = cy + full * (ux * std::sin(ang) + uy * std::cos(ang));
    }
    const auto H = square_to_quad(quad);
    const auto Hi = invert3(H);
    // Black-border corners: 1 cell in from the full rendered square.
    for (int k = 0; k < 4; ++k) {
      const double u = k == 1 || k == 2 ? 1 - 1.0 / cells : 1.0 / cells;
      const double v = k >= 2 ? 1 - 1.0 / cells : 1.0 / cells;
      const double wd = H[6] * u + H[7] * v + H[8];
      t.corners[k][0] = (H[0] * u + H[1] * v + H[2]) / wd;
      t.corners[k][1] = (H[3] * u + H[4] * v + H[5]) / wd;
    }
    const double black = rng.uniform(10, 70), white = rng.uniform(170, 250);
    int x0 = p.width, x1 = 0, y0 = p.height, y1 = 0;
    for (auto& c : quad) {
      x0 = std::min(x0, static_cast<int>(std::floor(c[0])));
      x1 = std::max(x1, static_cast<int>(std::ceil(c[0])));
      y0 = std::min(y0, static_cast<int>(std::floor(c[1])));
      y1 = std::max(y1, static_cast<int>(std::ceil(c[1])));
    }
    x0 = std::max(x0, 0), y0 = std::max(y0, 0), x1 = std::min(x1, p.width - 1), y1 = std::min(y1, p.height - 1);
    // 4x4 supersampling for anti-aliased edges.
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) {
        double sum = 0;
        int hits = 0;
        for (int sy = 0; sy < 4; ++sy)
          for (int sx = 0; sx < 4; ++sx) {
            const double px = x + (sx + 0.5) / 4, py = y + (sy + 0.5) / 4;
            const double wd = Hi[6] * px + Hi[7] * py + Hi[8];
            const double u = (Hi[0] * px + Hi[1] * py + Hi[2]) / wd;
            const double v = (Hi[3] * px + Hi[4] * py + Hi[5]) / wd;
            if (u < 0 || u >= 1 || v < 0 || v >= 1) continue;
            const int cxi = static_cast<int>(u * cells), cyi = static_cast<int>(v * cells);
            sum += tag->buf[cyi * tag->stride + cxi] ? white : black;
            ++hits;
          }
        if (hits) {
          double& d = img[static_cast<size_t>(y * p.width + x)];
          d = (d * (16 - hits) + sum) / 16;
        }
      }
    image_u8_destroy(tag);
    scene.tags.push_back(t);
  }
  tag36h11_destroy(fam);

  std::vector<double> out = img;
  for (int pass = 0; pass < (p.blur ? 1 + p.extra_blur : 0); ++pass) {
    const std::vector<double> in = out;
    for (int y = 1; y + 1 < p.height; ++y)
      for (int x = 1; x + 1 < p.width; ++x) {
        double s = 0;
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx) s += in[static_cast<size_t>((y + dy) * p.width + x + dx)];
        out[static_cast<size_t>(y * p.width + x)] = s / 9;
      }
  }
  for (size_t i = 0; i < out.size(); ++i)
    scene.image.pixels[i] = clamp8(out[i] + (p.noise ? rng.integer(-p.noise, p.noise) : 0));
  return scene;
}

model::Image8 noise_image(uint32_t seed, int w, int h, int lo, int hi) {
  Rng rng(seed);
  model::Image8 img(w, h);
  for (auto& v : img.pixels) v = static_cast<uint8_t>(rng.integer(lo, hi));
  return img;
}

model::Image8 blocks_image(uint32_t seed, int w, int h, int num_blocks) {
  Rng rng(seed);
  model::Image8 img(w, h, static_cast<uint8_t>(rng.integer(0, 255)));
  for (int n = 0; n < num_blocks; ++n) {
    const int x0 = rng.integer(0, w - 1), y0 = rng.integer(0, h - 1);
    const int x1 = std::min(w - 1, x0 + rng.integer(0, w / 3)), y1 = std::min(h - 1, y0 + rng.integer(0, h / 3));
    const auto v = static_cast<uint8_t>(rng.integer(0, 255));
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) img.at(x, y) = v;
  }
  return img;
}

}  // namespace gv::test
