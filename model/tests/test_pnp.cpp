#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "gv/model/pnp.h"

using namespace gv::model;

namespace {

constexpr double kTag = 0.1651;

Intrinsics camera(bool distorted) {
  Intrinsics k{910, 905, 640, 400, {}};
  if (distorted) k.dist = {-0.12, 0.05, 0.001, -0.0008, -0.01};
  return k;
}

const std::vector<Vec3> kObj = {{-kTag / 2, kTag / 2, 0}, {kTag / 2, kTag / 2, 0}, {kTag / 2, -kTag / 2, 0}, {-kTag / 2, -kTag / 2, 0}};

double rot_err_deg(const Mat3& a, const Mat3& b) {
  const Vec3 r = rotation_vector(multiply(a, transpose(b)));
  return std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) * 180 / M_PI;
}

double dist(const Vec3& a, const Vec3& b) { return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]); }

// A random tag pose in front of the camera, facing it within ~60 degrees.
Pose random_pose(std::mt19937& g) {
  std::uniform_real_distribution<double> u(-1, 1);
  Pose p;
  // Tag z axis points toward the camera: start from a tag facing the camera
  // (R = rotate 180 about x: tag y up -> camera y down), then tilt.
  const Mat3 facing = rodrigues({M_PI, 0, 0});
  p.R = multiply(rodrigues({0.8 * u(g), 0.8 * u(g), 0.5 * u(g)}), facing);
  const double z = 1.0 + 4.0 * (u(g) + 1) / 2;
  p.t = {0.4 * z * u(g), 0.25 * z * u(g), z};
  return p;
}

std::array<Vec2, 4> corners(const Intrinsics& k, const Pose& p, std::mt19937* g = nullptr, double noise = 0) {
  std::normal_distribution<double> n(0, noise);
  std::array<Vec2, 4> c;
  for (size_t i = 0; i < 4; ++i) {
    c[i] = project(k, p, kObj[i]);
    if (g && noise > 0) c[i][0] += n(*g), c[i][1] += n(*g);
  }
  return c;
}

}  // namespace

TEST(Pnp, UndistortInvertsProjection) {
  const Intrinsics k = camera(true);
  for (double x = -0.6; x <= 0.6; x += 0.2)
    for (double y = -0.4; y <= 0.4; y += 0.2) {
      const Pose id{{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, {0, 0, 0}, 0};
      const Vec2 px = project(k, id, {x, y, 1});
      const Vec2 back = undistort(k, px);
      EXPECT_NEAR(back[0], x, 1e-9);
      EXPECT_NEAR(back[1], y, 1e-9);
    }
}

TEST(Pnp, RotationVectorRoundTrip) {
  std::mt19937 g(3);
  std::uniform_real_distribution<double> u(-1, 1);
  for (int i = 0; i < 200; ++i) {
    Vec3 r{u(g), u(g), u(g)};
    const double n = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    const double angle = (i % 4 == 0) ? M_PI - 1e-9 : 3.0 * (u(g) + 1) / 2;  // include ~180 degrees
    for (double& v : r) v *= angle / n;
    EXPECT_LT(rot_err_deg(rodrigues(rotation_vector(rodrigues(r))), rodrigues(r)), 1e-5) << i;
  }
}

TEST(Pnp, IppeRecoversExactPose) {
  std::mt19937 g(7);
  for (bool distorted : {false, true}) {
    const Intrinsics k = camera(distorted);
    for (int i = 0; i < 500; ++i) {
      const Pose truth = random_pose(g);
      const auto sols = ippe_square(k, corners(k, truth), kTag);
      ASSERT_EQ(sols.size(), 2u);
      EXPECT_LT(rot_err_deg(sols[0].R, truth.R), 1e-6) << i;
      EXPECT_LT(dist(sols[0].t, truth.t), 1e-9) << i;
      EXPECT_LT(sols[0].reprojection_rms, 1e-6);
      EXPECT_LE(sols[0].reprojection_rms, sols[1].reprojection_rms);
    }
  }
}

TEST(Pnp, NoisyCornersGiveSensibleErrors) {
  // 0.1 px corner noise: the best solution is near the truth at a few meters.
  std::mt19937 g(11);
  const Intrinsics k = camera(false);
  int ambiguous_wrong = 0;
  double worst_t = 0;
  for (int i = 0; i < 500; ++i) {
    const Pose truth = random_pose(g);
    const auto sols = ippe_square(k, corners(k, truth, &g, 0.1), kTag);
    ASSERT_EQ(sols.size(), 2u);
    const bool best_is_right = rot_err_deg(sols[0].R, truth.R) < rot_err_deg(sols[1].R, truth.R);
    ambiguous_wrong += !best_is_right;
    const Pose& right = best_is_right ? sols[0] : sols[1];
    worst_t = std::max(worst_t, dist(right.t, truth.t) / truth.t[2]);
  }
  std::cout << "[ pnp ] 0.1 px noise: the lower-error IPPE solution was the wrong one in " << ambiguous_wrong
            << "/500; worst position error of the right one " << 100 * worst_t << "% of range\n";
  EXPECT_LT(worst_t, 0.02);
}

TEST(Pnp, MultiTagRefinementBeatsSingleTag) {
  // Three tags on a plane-ish wall, camera ~4 m away, 0.3 px noise. Seed LM
  // from tag 0's best IPPE solution, solve on all 12 corners.
  std::mt19937 g(5);
  const Intrinsics k = camera(true);
  double single_sum = 0, multi_sum = 0;
  for (int trial = 0; trial < 100; ++trial) {
    // Field (object) frame: tags at x offsets, camera pose = truth.
    Pose truth = random_pose(g);
    truth.t[2] += 2;
    std::vector<Vec3> obj;
    std::vector<Vec2> px;
    std::normal_distribution<double> n(0, 0.3);
    for (double off : {-0.8, 0.0, 0.7})
      for (const Vec3& c : kObj) {
        const Vec3 p{c[0] + off, c[1] + 0.1 * off, c[2]};
        obj.push_back(p);
        Vec2 q = project(k, truth, p);
        q[0] += n(g), q[1] += n(g);
        px.push_back(q);
      }
    // Single tag (the middle one, whose object points are kObj).
    std::array<Vec2, 4> mid{px[4], px[5], px[6], px[7]};
    const auto sols = ippe_square(k, mid, kTag);
    ASSERT_FALSE(sols.empty());
    const Pose refined = refine_from_seeds(k, obj, px, sols);
    single_sum += dist(sols[0].t, truth.t);
    multi_sum += dist(refined.t, truth.t);
    EXPECT_LE(refined.reprojection_rms, 0.6) << trial;
  }
  std::cout << "[ pnp ] mean position error: single tag " << single_sum / 100 << " m, three-tag LM " << multi_sum / 100
            << " m\n";
  EXPECT_LT(multi_sum, single_sum);
}

TEST(Pnp, WpilibConventionForAFrontalTag) {
  // A tag straight ahead, facing the camera, upright: in WPILib terms its X
  // axis (out of its face) points back at the camera (-X), its Y axis (the
  // viewer's right) is the camera's -Y, and its Z axis is up (+Z).
  Pose o;
  o.R = rodrigues({M_PI, 0, 0});  // object y up -> camera y down, object z toward the camera
  o.t = {0, 0, 3};
  const Pose w = to_wpilib(o);
  const Mat3 want = {{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}};
  for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 3; ++j) EXPECT_NEAR(w.R[i][j], want[i][j], 1e-12) << i << "," << j;
  EXPECT_NEAR(w.t[0], 3, 1e-12);
  EXPECT_NEAR(w.t[1], 0, 1e-12);
  EXPECT_NEAR(w.t[2], 0, 1e-12);
}
