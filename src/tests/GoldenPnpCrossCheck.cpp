// The golden PnP model (model/src/pnp.cpp) against OpenCV's
// solvePnPGeneric(SOLVEPNP_IPPE_SQUARE), as the CPU pipeline calls it: both
// solutions, their order and reprojection errors, with 0.3 px corner noise
// and lens distortion; and model::to_wpilib() against
// PoseUtils::openCvTagPoseToWpilib. Exits 1 on any difference over 1e-5.
//
//   cmake -DGOMPEI_TESTS=ON .. && make GoldenPnpCrossCheck && ./src/GoldenPnpCrossCheck
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <opencv2/calib3d.hpp>
#include "gv/model/pnp.h"
#include "util/PoseUtils.h"
using namespace gv::model;
int main() {
  const double s = 0.1651;
  Intrinsics k{910, 905, 640, 400, {-0.12, 0.05, 0.001, -0.0008, -0.01}};
  cv::Mat K = (cv::Mat_<double>(3, 3) << k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1);
  cv::Mat D = (cv::Mat_<double>(1, 5) << k.dist[0], k.dist[1], k.dist[2], k.dist[3], k.dist[4]);
  std::vector<cv::Point3d> obj = {{-s / 2, s / 2, 0}, {s / 2, s / 2, 0}, {s / 2, -s / 2, 0}, {-s / 2, -s / 2, 0}};
  std::mt19937 g(1);
  std::uniform_real_distribution<double> u(-1, 1);
  std::normal_distribution<double> n(0, 0.3);
  double worst_R = 0, worst_t = 0, worst_e = 0, worst_w = 0;
  int cases = 0, order_mismatch = 0;
  for (int i = 0; i < 2000; ++i) {
    Pose truth;
    truth.R = multiply(rodrigues({0.8 * u(g), 0.8 * u(g), 0.5 * u(g)}), rodrigues({M_PI, 0, 0}));
    const double z = 0.8 + 5 * (u(g) + 1) / 2;
    truth.t = {0.4 * z * u(g), 0.25 * z * u(g), z};
    std::array<Vec2, 4> c;
    std::vector<cv::Point2d> img;
    for (size_t j = 0; j < 4; ++j) {
      c[j] = project(k, truth, {obj[j].x, obj[j].y, 0});
      c[j][0] += n(g), c[j][1] += n(g);
      img.emplace_back(c[j][0], c[j][1]);
    }
    std::vector<cv::Mat> rv, tv; std::vector<double> err;
    cv::solvePnPGeneric(obj, img, K, D, rv, tv, false, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(), err);
    const auto ours = ippe_square(k, c, s);
    if (rv.size() != 2 || ours.size() != 2) continue;
    ++cases;
    for (int sol = 0; sol < 2; ++sol) {
      // Match each OpenCV solution with ours by rotation.
      cv::Mat Rc; cv::Rodrigues(rv[sol], Rc);
      double best = 1e9; int bi = 0;
      for (int o = 0; o < 2; ++o) {
        double d = 0;
        for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) d += std::abs(Rc.at<double>(a, b) - ours[o].R[a][b]);
        if (d < best) best = d, bi = o;
      }
      order_mismatch += (bi != sol);
      worst_R = std::max(worst_R, best);
      for (int a = 0; a < 3; ++a) worst_t = std::max(worst_t, std::abs(tv[sol].at<double>(a) - ours[bi].t[a]));
      worst_e = std::max(worst_e, std::abs(err[sol] - ours[bi].reprojection_rms));
      // Convention check against the CPU pipeline's (fixed) PoseUtils.
      const frc::Pose3d wp = PoseUtils::openCvTagPoseToWpilib(rv[sol], tv[sol]);
      const Pose w = to_wpilib(ours[bi]);
      const auto M = wp.Rotation().ToMatrix();
      for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) worst_w = std::max(worst_w, std::abs(M(a, b) - w.R[a][b]));
      worst_w = std::max({worst_w, std::abs(wp.X().value() - w.t[0]), std::abs(wp.Y().value() - w.t[1]), std::abs(wp.Z().value() - w.t[2])});
    }
  }
  std::printf("%d cases (0.3 px noise, distortion): max |R diff| %.2e, max |t diff| %.2e m, max |error diff| %.2e px, solution order differs %d, max WPILib diff %.2e\n",
              cases, worst_R, worst_t, worst_e, order_mismatch, worst_w);
  const bool ok = cases > 1900 && worst_R < 1e-5 && worst_t < 1e-5 && worst_e < 1e-5 && order_mismatch == 0 && worst_w < 1e-5;
  std::printf(ok ? "MATCH\n" : "MISMATCH\n");
  return ok ? 0 : 1;
}
