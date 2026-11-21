#include "gv/model/pnp.h"

#include <algorithm>
#include <cmath>

namespace gv::model {

namespace {

Vec3 apply(const Mat3& R, const Vec3& v) {
  return {R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2], R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
          R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]};
}

Vec2 distort_normalized(const Intrinsics& k, double x, double y) {
  const double r2 = x * x + y * y;
  const double radial = 1 + k.dist[0] * r2 + k.dist[1] * r2 * r2 + k.dist[4] * r2 * r2 * r2;
  const double xd = x * radial + 2 * k.dist[2] * x * y + k.dist[3] * (r2 + 2 * x * x);
  const double yd = y * radial + k.dist[2] * (r2 + 2 * y * y) + 2 * k.dist[3] * x * y;
  return {xd, yd};
}

// Least-squares t for a fixed R: for each point, u * z - x = 0 and v * z - y = 0
// with (x, y, z) = R p + t (OpenCV's IPPE computeTranslation).
std::optional<Vec3> solve_translation(const Mat3& R, const std::vector<Vec3>& obj, const std::vector<Vec2>& uv) {
  double A[3][3] = {}, b[3] = {};
  for (size_t i = 0; i < obj.size(); ++i) {
    const Vec3 rp = apply(R, obj[i]);
    const double u = uv[i][0], v = uv[i][1];
    // Rows: [-1, 0, u] t = rp.x - u rp.z ; [0, -1, v] t = rp.y - v rp.z
    const double rows[2][3] = {{-1, 0, u}, {0, -1, v}};
    const double rhs[2] = {rp[0] - u * rp[2], rp[1] - v * rp[2]};
    for (int r = 0; r < 2; ++r)
      for (int c = 0; c < 3; ++c) {
        for (int d = 0; d < 3; ++d) A[c][d] += rows[r][c] * rows[r][d];
        b[c] += rows[r][c] * rhs[r];
      }
  }
  const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                     A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (std::abs(det) < 1e-18) return std::nullopt;
  Vec3 t{};
  for (int col = 0; col < 3; ++col) {
    double m[3][3];
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) m[r][c] = c == col ? b[r] : A[r][c];
    t[static_cast<size_t>(col)] = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                                   m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                                   m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det;
  }
  return t;
}

double rms_px(const Intrinsics& k, const Pose& pose, const std::vector<Vec3>& obj, const std::vector<Vec2>& px) {
  double s = 0;
  for (size_t i = 0; i < obj.size(); ++i) {
    const Vec2 p = project(k, pose, obj[i]);
    s += (p[0] - px[i][0]) * (p[0] - px[i][0]) + (p[1] - px[i][1]) * (p[1] - px[i][1]);
  }
  return std::sqrt(s / (2.0 * static_cast<double>(obj.size())));
}

// Sum of squared reprojection residuals in normalized (undistorted) image
// coordinates: the order OpenCV's IPPE returns its two solutions in.
double normalized_error(const Pose& pose, const std::vector<Vec3>& obj, const std::vector<Vec2>& uv) {
  double s = 0;
  for (size_t i = 0; i < obj.size(); ++i) {
    const Vec3 c = apply(pose.R, obj[i]);
    const double z = c[2] + pose.t[2];
    const double du = (c[0] + pose.t[0]) / z - uv[i][0], dv = (c[1] + pose.t[1]) / z - uv[i][1];
    s += du * du + dv * dv;
  }
  return s;
}

}  // namespace

Mat3 multiply(const Mat3& a, const Mat3& b) {
  Mat3 c{};
  for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 3; ++j)
      for (size_t k = 0; k < 3; ++k) c[i][j] += a[i][k] * b[k][j];
  return c;
}

Mat3 transpose(const Mat3& a) {
  Mat3 t{};
  for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 3; ++j) t[i][j] = a[j][i];
  return t;
}

Mat3 rodrigues(const Vec3& r) {
  const double th = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  if (th < 1e-12) return {{{1, -r[2], r[1]}, {r[2], 1, -r[0]}, {-r[1], r[0], 1}}};
  const double kx = r[0] / th, ky = r[1] / th, kz = r[2] / th, c = std::cos(th), s = std::sin(th), v = 1 - c;
  return {{{c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s},
           {ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s},
           {kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v}}};
}

Vec3 rotation_vector(const Mat3& R) {
  const double c = std::clamp((R[0][0] + R[1][1] + R[2][2] - 1) / 2, -1.0, 1.0);
  const double th = std::acos(c);
  if (th < 1e-9) return {(R[2][1] - R[1][2]) / 2, (R[0][2] - R[2][0]) / 2, (R[1][0] - R[0][1]) / 2};
  if (M_PI - th < 1e-6) {  // near 180 degrees: axis from the diagonal
    Vec3 a{std::sqrt(std::max(0.0, (R[0][0] + 1) / 2)), std::sqrt(std::max(0.0, (R[1][1] + 1) / 2)),
           std::sqrt(std::max(0.0, (R[2][2] + 1) / 2))};
    if (R[0][1] + R[1][0] < 0) a[1] = -a[1];
    if (R[0][2] + R[2][0] < 0) a[2] = -a[2];
    return {a[0] * th, a[1] * th, a[2] * th};
  }
  const double s = th / (2 * std::sin(th));
  return {(R[2][1] - R[1][2]) * s, (R[0][2] - R[2][0]) * s, (R[1][0] - R[0][1]) * s};
}

Vec2 undistort(const Intrinsics& k, const Vec2& px, int iterations) {
  const double xd = (px[0] - k.cx) / k.fx, yd = (px[1] - k.cy) / k.fy;
  double x = xd, y = yd;
  for (int i = 0; i < iterations; ++i) {  // fixed-point iteration, as cv::undistortPoints
    const double r2 = x * x + y * y;
    const double radial = 1 + k.dist[0] * r2 + k.dist[1] * r2 * r2 + k.dist[4] * r2 * r2 * r2;
    const double dx = 2 * k.dist[2] * x * y + k.dist[3] * (r2 + 2 * x * x);
    const double dy = k.dist[2] * (r2 + 2 * y * y) + 2 * k.dist[3] * x * y;
    x = (xd - dx) / radial, y = (yd - dy) / radial;
  }
  return {x, y};
}

Vec2 project(const Intrinsics& k, const Pose& pose, const Vec3& p) {
  const Vec3 c = apply(pose.R, p);
  const Vec2 d = distort_normalized(k, (c[0] + pose.t[0]) / (c[2] + pose.t[2]), (c[1] + pose.t[1]) / (c[2] + pose.t[2]));
  return {k.fx * d[0] + k.cx, k.fy * d[1] + k.cy};
}

std::vector<Pose> ippe_square(const Intrinsics& k, const std::array<Vec2, 4>& corners_px, double s) {
  const std::vector<Vec3> obj = {{-s / 2, s / 2, 0}, {s / 2, s / 2, 0}, {s / 2, -s / 2, 0}, {-s / 2, -s / 2, 0}};
  std::vector<Vec2> uv, px(corners_px.begin(), corners_px.end());
  for (const Vec2& c : corners_px) uv.push_back(undistort(k, c));

  // Homography from the model plane (x, y) to normalized image points:
  // Heckbert's square-to-quad on the unit square, composed with
  // (x, y) -> (x / s + 1/2, 1/2 - y / s), which sends the corners to (0,0), (1,0), (1,1), (0,1).
  const double x0 = uv[0][0], y0 = uv[0][1], x1 = uv[1][0], y1 = uv[1][1];
  const double x2 = uv[2][0], y2 = uv[2][1], x3 = uv[3][0], y3 = uv[3][1];
  const double sx = x0 - x1 + x2 - x3, sy = y0 - y1 + y2 - y3;
  const double dx1 = x1 - x2, dx2 = x3 - x2, dy1 = y1 - y2, dy2 = y3 - y2;
  const double den = dx1 * dy2 - dx2 * dy1;
  if (std::abs(den) < 1e-15) return {};
  const double g = (sx * dy2 - dx2 * sy) / den, h = (dx1 * sy - sx * dy1) / den;
  const Mat3 Hs = {{{x1 - x0 + g * x1, x3 - x0 + h * x3, x0}, {y1 - y0 + g * y1, y3 - y0 + h * y3, y0}, {g, h, 1}}};
  const Mat3 M = {{{1 / s, 0, 0.5}, {0, -1 / s, 0.5}, {0, 0, 1}}};
  Mat3 H = multiply(Hs, M);
  const double n = H[2][2];
  for (auto& row : H)
    for (double& v : row) v /= n;

  // IPPE at the model origin: v = H(0), J = dH/d(x, y) at 0.
  const double p = H[0][2], q = H[1][2];
  const double j00 = H[0][0] - H[2][0] * p, j01 = H[0][1] - H[2][1] * p;
  const double j10 = H[1][0] - H[2][0] * q, j11 = H[1][1] - H[2][1] * q;

  // Rv rotates (0, 0, 1) onto the ray through v.
  const double nrm = std::sqrt(p * p + q * q + 1);
  const double ax = p / nrm, ay = q / nrm, az = 1 / nrm;
  Mat3 Rv{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  if (std::abs(ax) > 1e-15 || std::abs(ay) > 1e-15) {
    const double kn = std::sqrt(ax * ax + ay * ay);  // axis = z x a = (-ay, ax, 0)
    const double angle = std::acos(std::clamp(az, -1.0, 1.0));
    Rv = rodrigues({-ay / kn * angle, ax / kn * angle, 0});
  }
  // B = [I2 | -v] * Rv(:, 0:2), A = B^-1 J.
  const double b00 = Rv[0][0] - p * Rv[2][0], b01 = Rv[0][1] - p * Rv[2][1];
  const double b10 = Rv[1][0] - q * Rv[2][0], b11 = Rv[1][1] - q * Rv[2][1];
  const double bdet = b00 * b11 - b01 * b10;
  if (std::abs(bdet) < 1e-15) return {};
  const double i00 = b11 / bdet, i01 = -b01 / bdet, i10 = -b10 / bdet, i11 = b00 / bdet;
  const double a00 = i00 * j00 + i01 * j10, a01 = i00 * j01 + i01 * j11;
  const double a10 = i10 * j00 + i11 * j10, a11 = i10 * j01 + i11 * j11;
  // Largest singular value of A; the top-left 2x2 of R is A / gamma.
  const double t00 = a00 * a00 + a01 * a01, t01 = a00 * a10 + a01 * a11, t11 = a10 * a10 + a11 * a11;
  const double gamma = std::sqrt(0.5 * (t00 + t11 + std::sqrt((t00 - t11) * (t00 - t11) + 4 * t01 * t01)));
  if (gamma < 1e-15) return {};
  const double r00 = a00 / gamma, r01 = a01 / gamma, r10 = a10 / gamma, r11 = a11 / gamma;
  const double c0 = std::sqrt(std::max(0.0, 1 - r00 * r00 - r10 * r10));
  double c1 = std::sqrt(std::max(0.0, 1 - r01 * r01 - r11 * r11));
  if (-r00 * r01 - r10 * r11 < 0) c1 = -c1;  // columns must be orthogonal
  // The two solutions differ in the sign of the third row of the first two
  // columns; the third column is their cross product.
  std::vector<Pose> out;
  std::vector<double> order_key;
  for (const double sgn : {1.0, -1.0}) {
    const Vec3 a{r00, r10, sgn * c0}, b{r01, r11, sgn * c1};
    const Vec3 c{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    const Mat3 Rl = {{{a[0], b[0], c[0]}, {a[1], b[1], c[1]}, {a[2], b[2], c[2]}}};
    Pose pose;
    pose.R = multiply(Rv, Rl);
    const auto t = solve_translation(pose.R, obj, uv);
    if (!t) continue;
    pose.t = *t;
    pose.reprojection_rms = rms_px(k, pose, obj, px);
    out.push_back(pose);
    order_key.push_back(normalized_error(pose, obj, uv));
  }
  if (out.size() == 2 && order_key[1] < order_key[0]) std::swap(out[0], out[1]);
  return out;
}

Pose refine_pose(const Intrinsics& k, const std::vector<Vec3>& obj, const std::vector<Vec2>& px, const Pose& seed,
                 int max_iterations) {
  // Parameters: a rotation perturbation w (R = rodrigues(w) * R0) and t.
  Pose cur = seed;
  cur.reprojection_rms = rms_px(k, cur, obj, px);
  double lambda = 1e-3;
  const size_t n = obj.size();
  auto residuals = [&](const Pose& ps, std::vector<double>& r) {
    r.resize(2 * n);
    for (size_t i = 0; i < n; ++i) {
      const Vec2 pp = project(k, ps, obj[i]);
      r[2 * i] = pp[0] - px[i][0], r[2 * i + 1] = pp[1] - px[i][1];
    }
  };
  auto perturbed = [&](const Pose& base, const std::array<double, 6>& d) {
    Pose p = base;
    p.R = multiply(rodrigues({d[0], d[1], d[2]}), base.R);
    p.t = {base.t[0] + d[3], base.t[1] + d[4], base.t[2] + d[5]};
    return p;
  };
  std::vector<double> r0, r1;
  for (int it = 0; it < max_iterations; ++it) {
    residuals(cur, r0);
    // Numerical Jacobian (central differences).
    std::vector<std::array<double, 6>> J(2 * n);
    for (int j = 0; j < 6; ++j) {
      const double eps = j < 3 ? 1e-7 : 1e-7 * std::max(1.0, std::abs(cur.t[static_cast<size_t>(j - 3)]));
      std::array<double, 6> d{};
      d[static_cast<size_t>(j)] = eps;
      std::vector<double> rp, rm;
      residuals(perturbed(cur, d), rp);
      d[static_cast<size_t>(j)] = -eps;
      residuals(perturbed(cur, d), rm);
      for (size_t i = 0; i < 2 * n; ++i) J[i][static_cast<size_t>(j)] = (rp[i] - rm[i]) / (2 * eps);
    }
    double A[6][6] = {}, g[6] = {};
    for (size_t i = 0; i < 2 * n; ++i)
      for (size_t a = 0; a < 6; ++a) {
        g[a] += J[i][a] * r0[i];
        for (size_t b = 0; b < 6; ++b) A[a][b] += J[i][a] * J[i][b];
      }
    bool improved = false;
    for (int tries = 0; tries < 10 && !improved; ++tries) {
      // Solve (A + lambda diag(A)) d = -g by Gaussian elimination.
      double M[6][7];
      for (int a = 0; a < 6; ++a) {
        for (int b = 0; b < 6; ++b) M[a][b] = A[a][b] + (a == b ? lambda * std::max(A[a][a], 1e-12) : 0);
        M[a][6] = -g[a];
      }
      for (int c = 0; c < 6; ++c) {
        int piv = c;
        for (int r = c + 1; r < 6; ++r)
          if (std::abs(M[r][c]) > std::abs(M[piv][c])) piv = r;
        std::swap(M[c], M[piv]);
        if (std::abs(M[c][c]) < 1e-30) break;
        for (int r = 0; r < 6; ++r)
          if (r != c) {
            const double f = M[r][c] / M[c][c];
            for (int cc = c; cc < 7; ++cc) M[r][cc] -= f * M[c][cc];
          }
      }
      std::array<double, 6> d{};
      for (int a = 0; a < 6; ++a) d[static_cast<size_t>(a)] = M[a][6] / M[a][a];
      const Pose cand = perturbed(cur, d);
      residuals(cand, r1);
      double e0 = 0, e1 = 0;
      for (size_t i = 0; i < 2 * n; ++i) e0 += r0[i] * r0[i], e1 += r1[i] * r1[i];
      if (std::isfinite(e1) && e1 < e0) {
        cur = cand, lambda = std::max(lambda / 10, 1e-12), improved = true;
        if (e0 - e1 < 1e-14 * std::max(1.0, e0)) it = max_iterations;  // converged
      } else {
        lambda *= 10;
      }
    }
    if (!improved) break;
  }
  cur.reprojection_rms = rms_px(k, cur, obj, px);
  return cur;
}

Pose refine_from_seeds(const Intrinsics& k, const std::vector<Vec3>& obj, const std::vector<Vec2>& px,
                       const std::vector<Pose>& seeds, int max_iterations) {
  Pose best;
  best.reprojection_rms = INFINITY;
  for (const Pose& s : seeds) {
    const Pose r = refine_pose(k, obj, px, s, max_iterations);
    if (r.reprojection_rms < best.reprojection_rms) best = r;
  }
  return best;
}

Pose to_wpilib(const Pose& o) {
  // C: OpenCV camera axes -> WPILib camera axes (X = z, Y = -x, Z = -y).
  // D: OpenCV tag-object axes (x right, y up, z toward the camera) -> WPILib
  //    tag axes (X = z, Y = x, Z = y).
  const Mat3 C = {{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}};
  const Mat3 D = {{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}}};
  Pose w;
  w.R = multiply(multiply(C, o.R), transpose(D));
  w.t = apply(C, o.t);
  w.reprojection_rms = o.reprojection_rms;
  return w;
}

}  // namespace gv::model
