// PnP golden model: the reference for module-side PnP on the RPU and the
// robot library's solver (PLAN "Robot-side PnP"). Dependency-free C++.
//
//   - undistort(): OpenCV's 5-coefficient model (k1, k2, p1, p2, k3), inverted
//     iteratively, to normalized image coordinates.
//   - ippe_square(): both IPPE solutions for one square tag (Collins & Bartoli,
//     "Infinitesimal Plane-Based Pose Estimation", IJCV 2014), as OpenCV's
//     SOLVEPNP_IPPE_SQUARE computes them, ranked by reprojection error. Corner
//     order and object points match the CPU pipeline: top-left, top-right,
//     bottom-right, bottom-left at (-s/2, s/2), (s/2, s/2), (s/2, -s/2),
//     (-s/2, -s/2), z = 0.
//   - refine_pose(): Levenberg-Marquardt on the reprojection error of any set
//     of 3D-2D points: the multi-tag solve, seeded from the best single tag.
//   - to_wpilib(): camera and tag axes converted as the CPU pipeline's
//     PoseUtils::openCvTagPoseToWpilib does (fixed 2026-11-20).
//
// Poses are OpenCV-style: x_cam = R * x_obj + t, camera x right, y down, z
// forward.
#pragma once

#include <array>
#include <optional>
#include <vector>

namespace gv::model {

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

struct Intrinsics {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  std::array<double, 5> dist{};  // k1, k2, p1, p2, k3
};

struct Pose {
  Mat3 R{};
  Vec3 t{};
  // Pixels, as OpenCV's solvePnPGeneric reports it: the L2 norm of all
  // reprojection residuals over sqrt(2 * points), i.e. RMS per coordinate.
  double reprojection_rms = 0;
};

// Pixel -> normalized, undistorted image coordinates; and the reverse.
Vec2 undistort(const Intrinsics& k, const Vec2& pixel, int iterations = 20);
Vec2 project(const Intrinsics& k, const Pose& pose, const Vec3& object_point);  // with distortion

// Both IPPE solutions, ordered as OpenCV orders them: by reprojection error
// in undistorted normalized coordinates (not by the reported pixel error,
// which can disagree in near-ties). Empty if the corners are degenerate.
std::vector<Pose> ippe_square(const Intrinsics& k, const std::array<Vec2, 4>& corners_px, double tag_size_m);

// LM refinement of `seed` on all point pairs.
Pose refine_pose(const Intrinsics& k, const std::vector<Vec3>& object_points, const std::vector<Vec2>& image_px,
                 const Pose& seed, int max_iterations = 50);

// The multi-tag solve: refine from each seed (e.g. both IPPE solutions of
// the best tag; at range the lower-error one is often the wrong one) and keep
// the lowest reprojection error.
Pose refine_from_seeds(const Intrinsics& k, const std::vector<Vec3>& object_points, const std::vector<Vec2>& image_px,
                       const std::vector<Pose>& seeds, int max_iterations = 50);

// Rotation helpers.
Mat3 rodrigues(const Vec3& rvec);
Vec3 rotation_vector(const Mat3& R);
Mat3 multiply(const Mat3& a, const Mat3& b);
Mat3 transpose(const Mat3& a);

// The tag's pose in the WPILib camera frame (X forward, Y left, Z up), with
// the tag in WPILib's tag convention (X out of its face, Y to the viewer's
// right, Z up).
Pose to_wpilib(const Pose& opencv_tag_in_camera);

}  // namespace gv::model
