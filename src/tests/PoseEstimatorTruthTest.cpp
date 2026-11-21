// Truth test for the CPU pose estimators: a camera at a known field pose, tag
// corners projected (no distortion) with the estimators' own corner convention,
// then each estimator's output compared with the truth. Exits 1 on any error
// over 1 mm / 0.05 deg.
//
//   cmake -DGOMPEI_TESTS=ON .. && make PoseEstimatorTruthTest && ./src/PoseEstimatorTruthTest
//
// Written after finding (2026-11-20) that single-tag poses were 1.2 m and 180
// degrees off: the tag's own axes weren't converted to WPILib's convention,
// SingleTagPoseEstimator passed (tvec, rvec) swapped, and the single-tag
// branch looked up the first tag seen rather than the one in the layout.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <opencv2/core.hpp>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include "estimator/CameraPoseEstimator.h"
#include "estimator/SingleTagPoseEstimator.h"
#include "util/PoseUtils.h"

static const double kTag = 0.1651;
static cv::Mat K = (cv::Mat_<double>(3, 3) << 900, 0, 640, 0, 900, 400, 0, 0, 1);
static cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);

// Field point -> pixel, for a camera at field_to_camera (WPILib camera frame: X forward, Y left, Z up).
static cv::Point2d project(const frc::Pose3d& field_to_camera, const frc::Translation3d& p_field) {
  const frc::Pose3d p{p_field, frc::Rotation3d{}};
  const frc::Translation3d c = p.RelativeTo(field_to_camera).Translation();
  const cv::Point3f cv = PoseUtils::wpilibTranslationToOpenCV(c);
  return {K.at<double>(0, 0) * cv.x / cv.z + K.at<double>(0, 2), K.at<double>(1, 1) * cv.y / cv.z + K.at<double>(1, 2)};
}

static std::vector<double> tag_corners(const frc::Pose3d& tag, const frc::Pose3d& cam) {
  const auto h = units::meter_t(kTag / 2);
  const frc::Translation3d local[4] = {{0_m, -h, h}, {0_m, h, h}, {0_m, h, -h}, {0_m, -h, -h}};  // TL TR BR BL, as the estimator
  std::vector<double> out;
  for (const auto& l : local) {
    const auto px = project(cam, (tag + frc::Transform3d{l, frc::Rotation3d{}}).Translation());
    out.push_back(px.x), out.push_back(px.y);
  }
  return out;
}

static int failures = 0;

static void report(const char* what, const frc::Pose3d& got, const frc::Pose3d& want) {
  const double dt = got.Translation().Distance(want.Translation()).value();
  const double dr = (got.Rotation() - want.Rotation()).Angle().value() * 180 / M_PI;
  const bool ok = dt < 1e-3 && dr < 0.05;
  failures += !ok;
  std::printf("  %-44s position error %8.4f m, rotation error %7.3f deg  %s\n", what, dt, dr, ok ? "ok" : "FAIL");
}

int main() {
  // Tag 7 on a wall facing -X (into the field), tag 8 a meter to its side.
  std::map<int, frc::Pose3d> field{
      {7, frc::Pose3d{frc::Translation3d{8_m, 4_m, 1_m}, frc::Rotation3d{0_rad, 0_rad, units::radian_t{M_PI}}}},
      {8, frc::Pose3d{frc::Translation3d{8_m, 5_m, 1.2_m}, frc::Rotation3d{0_rad, 0_rad, units::radian_t{M_PI}}}}};
  // Camera 3 m in front of the tags, a bit to the side, yawed toward them, pitched up.
  const frc::Pose3d cam{frc::Translation3d{5_m, 4.3_m, 0.5_m},
                        frc::Rotation3d{0_rad, units::radian_t{-0.15}, units::radian_t{0.1}}};

  {
    std::printf("single tag (tag 7):\n");
    FiducialImageObservation obs{{7}, {tag_corners(field[7], cam)}, 0};
    AprilTagResult r;
    CameraPoseEstimator::estimatePose(obs, r, K, D, kTag, field);
    const auto& m = r.multi_tag_pose;
    const bool use1 = m.error_1 && m.pose_1 && *m.error_1 < m.error_0;
    report("CameraPoseEstimator field-to-camera (best)", use1 ? *m.pose_1 : m.pose_0, cam);
    SingleTagPoseEstimator::estimatePose(obs, r, K, D, kTag);
    const frc::Pose3d cam_to_tag_truth = field[7].RelativeTo(cam);
    const auto& s = r.single_tag_poses.at(0);
    const bool s1 = s.error_1 && s.pose_1 && *s.error_1 < s.error_0;
    report("SingleTagPoseEstimator camera-to-tag (best)", s1 ? *s.pose_1 : s.pose_0, cam_to_tag_truth);
    std::printf("  SingleTagPoseEstimator distance %.4f m (truth %.4f m)\n", s.distance.value(), cam_to_tag_truth.Translation().Norm().value());
    failures += std::abs(s.distance.value() - cam_to_tag_truth.Translation().Norm().value()) > 1e-3;
  }
  {
    std::printf("two tags (7, 8):\n");
    FiducialImageObservation obs{{7, 8}, {tag_corners(field[7], cam), tag_corners(field[8], cam)}, 0};
    AprilTagResult r;
    CameraPoseEstimator::estimatePose(obs, r, K, D, kTag, field);
    report("CameraPoseEstimator field-to-camera (SQPnP)", r.multi_tag_pose.pose_0, cam);
  }
  {
    std::printf("tag 99 (not in the layout) seen first, then tag 7:\n");
    FiducialImageObservation obs{{99, 7}, {tag_corners(field[7], cam), tag_corners(field[7], cam)}, 0};
    AprilTagResult r;
    CameraPoseEstimator::estimatePose(obs, r, K, D, kTag, field);
    const auto& m = r.multi_tag_pose;
    const bool use1 = m.error_1 && m.pose_1 && *m.error_1 < m.error_0;
    report("CameraPoseEstimator field-to-camera (best)", use1 ? *m.pose_1 : m.pose_0, cam);
    std::printf("  tag_ids reported: %zu (want 1, the tag actually used)\n", m.tag_ids.size());
    failures += m.tag_ids.size() != 1;
  }
  // A sweep of camera poses: both single-tag outputs, worst case.
  double worst_field = 0, worst_tag = 0;
  int n = 0;
  for (double x = 3; x <= 7; x += 1)
    for (double y = 2.5; y <= 5.5; y += 0.75)
      for (double yaw = -0.4; yaw <= 0.4; yaw += 0.2) {
        // Aim roughly at tag 7.
        const double aim = std::atan2(4 - y, 8 - x) + yaw * 0.5;
        const frc::Pose3d c{frc::Translation3d{units::meter_t{x}, units::meter_t{y}, 0.6_m},
                            frc::Rotation3d{0_rad, units::radian_t{-0.1}, units::radian_t{aim}}};
        FiducialImageObservation obs{{7}, {tag_corners(field[7], c)}, 0};
        AprilTagResult r;
        CameraPoseEstimator::estimatePose(obs, r, K, D, kTag, field);
        SingleTagPoseEstimator::estimatePose(obs, r, K, D, kTag);
        const auto& m = r.multi_tag_pose;
        const auto& best_field = (m.error_1 && *m.error_1 < m.error_0) ? *m.pose_1 : m.pose_0;
        const auto& s = r.single_tag_poses.at(0);
        const auto& best_tag = (s.error_1 && *s.error_1 < s.error_0) ? *s.pose_1 : s.pose_0;
        worst_field = std::max(worst_field, best_field.Translation().Distance(c.Translation()).value());
        worst_tag = std::max(worst_tag, best_tag.Translation().Distance(field[7].RelativeTo(c).Translation()).value());
        ++n;
      }
  std::printf("sweep of %d camera poses, single tag: worst field-to-camera position error %.5f m, worst camera-to-tag %.5f m\n", n, worst_field, worst_tag);
  failures += worst_field > 1e-3 || worst_tag > 1e-3;
  std::printf(failures ? "FAILED (%d)\n" : "all ok\n", failures);
  return failures ? 1 : 0;
}
