#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>

#include <cassert>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace PoseUtils {
/**
 * @brief Converts an OpenCV rotation vector and translation vector to an
 * frc::Pose3d, transforming from the OpenCV camera frame to the WPILib robot
 * frame.
 *
 * OpenCV Camera Frame: +Z forward, +X right, +Y down
 * WPILib Robot Frame:  +X forward, +Y left,  +Z up
 *
 * This results in the following mapping:
 *  - WPILib's X-axis becomes OpenCV's Z-axis.
 *  - WPILib's Y-axis becomes OpenCV's -X-axis.
 *  - WPILib's Z-axis becomes OpenCV's -Y-axis.
 *
 * @param rvec The rotation vector from solvePnP (in OpenCV's camera frame).
 * @param tvec The translation vector from solvePnP (in OpenCV's camera frame).
 * @return The calculated frc::Pose3d in WPILib's coordinate system.
 */
inline frc::Pose3d openCvPoseToWpilib(const cv::Mat& rvec,
                                      const cv::Mat& tvec) {
  assert(rvec.rows == 3 && rvec.cols == 1 && rvec.type() == CV_64F);
  assert(tvec.rows == 3 && tvec.cols == 1 && tvec.type() == CV_64F);

  // 1. Translation conversion (OpenCV to WPILib)
  const frc::Translation3d frc_translation(
      units::meter_t{tvec.at<double>(2, 0)},    // Z -> X
      units::meter_t{-tvec.at<double>(0, 0)},   // -X -> Y
      units::meter_t{-tvec.at<double>(1, 0)});  // -Y -> Z

  // 2. Rotation conversion (Axis-angle)
  const Eigen::Vector3d axis(rvec.at<double>(2, 0),    // Z -> X
                             -rvec.at<double>(0, 0),   // -X -> Y
                             -rvec.at<double>(1, 0));  // -Y -> Z

  const double angle = axis.norm();

  const frc::Rotation3d frc_rotation(axis, units::radian_t(angle));

  return frc::Pose3d(frc_translation, frc_rotation);
}

/**
 * @brief Converts a solvePnP tag pose (the tag in the camera frame, both in
 * OpenCV conventions) to the tag's pose in the WPILib camera frame, with the
 * tag in WPILib's tag convention.
 *
 * openCvPoseToWpilib converts only the camera's axes. A tag pose also has the
 * tag's own axes, which differ: the IPPE object points use x right, y up,
 * z toward the camera, while a WPILib tag has X out of its face, Y to the
 * viewer's right and Z up (the corners CameraPoseEstimator uses: top-left at
 * (0, -s/2, +s/2)). The two differ by 180 degrees about the tag's X axis,
 * applied in the tag's own frame.
 */
inline frc::Pose3d openCvTagPoseToWpilib(const cv::Mat& rvec,
                                         const cv::Mat& tvec) {
  return openCvPoseToWpilib(rvec, tvec)
      .TransformBy(frc::Transform3d{
          frc::Translation3d{},
          frc::Rotation3d{units::radian_t{M_PI}, 0_rad, 0_rad}});
}

inline cv::Point3f wpilibTranslationToOpenCV(
    const frc::Translation3d& translation) {
  return cv::Point3f(-translation.Y().value(), -translation.Z().value(),
                     translation.X().value());
}

inline bool isPoseZero(const frc::Pose3d& pose) {
  return pose.X() == 0_m && pose.Y() == 0_m && pose.Z() == 0_m;
}
}  // namespace PoseUtils