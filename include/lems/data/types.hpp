/**
 * @file types.hpp
 * @brief Dataset, camera, pose, frame, and metadata value types.
 * @ingroup datasets
 */
#pragma once

// Dataset/camera/frame types consolidated from the Brown-LEMS stereo and
// multinocular visual-odometry repositories.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "lems/data/edge.hpp"

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace lems::data {

/**
 * @brief Authoritative dataset time unit.
 * @ingroup datasets
 *
 * Reader timestamps are signed integer nanoseconds. Floating-point seconds
 * fields exist only as convenience/legacy views.
 */
using Timestamp = std::int64_t;

/**
 * @brief Legacy file-level metadata retained for Brown-LEMS consumers.
 * @ingroup datasets
 *
 * `has_gt` is a file/pose availability flag in this modern type. It is not the
 * compatibility adapter's dense-reference `Dataset::has_gt()` method.
 */
struct FileInfo {
  std::string dataset_type; ///< Configured reader type.
  std::string dataset_path; ///< Dataset root as a string.
  std::string output_path; ///< Optional output directory.
  std::string sequence_name; ///< Configured sequence.
  std::string GT_file_name; ///< Ground-truth file selected by the reader.
  bool has_gt{false}; ///< Pose-file availability in the reader.
  std::vector<double> GT_time_stamps; ///< Ground-truth seconds (legacy view).
  std::vector<double> Img_time_stamps; ///< Image seconds (legacy view).
};

/**
 * @brief Legacy camera mirror used by the original VO repositories.
 * @ingroup cameras
 *
 * Modern consumers should prefer @ref CameraCalibration and
 * @ref StereoCalibration; this type remains populated for compatibility.
 */
struct Camera {
  std::vector<int> resolution; ///< `[width, height]`.
  std::vector<double> intrinsics; ///< `[fx, fy, cx, cy]`.
  std::vector<double> distortion; ///< Native distortion coefficients.
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()}; ///< Legacy relative rotation.
  Eigen::Vector3d T{Eigen::Vector3d::Zero()}; ///< Legacy relative translation.
  Eigen::Matrix3d F{Eigen::Matrix3d::Zero()}; ///< Legacy fundamental matrix.
  Eigen::Matrix3d K{Eigen::Matrix3d::Identity()}; ///< Pinhole matrix.
};

/** @brief Legacy left/right camera and frame-to-body mirror. @ingroup cameras */
struct CameraInfo {
  Camera left; ///< Reference/left camera mirror.
  Camera right; ///< Target/right camera mirror.
  Eigen::Matrix3d rot_frame2body_left{Eigen::Matrix3d::Identity()}; ///< Frame-to-body rotation.
  Eigen::Vector3d transl_frame2body_left{Eigen::Vector3d::Zero()}; ///< Frame-to-body translation.
};

/**
 * @brief Timestamped world-to-camera rigid pose.
 * @ingroup cameras
 *
 * The convention is `p_camera = R * p_world + t`. `center()` returns
 * `-R.transpose() * t`; `transform()` and `detransform()` apply the forward
 * and inverse mappings respectively.
 */
struct CameraPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_ns{}; ///< Pose time in authoritative nanoseconds.
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()}; ///< World-to-camera rotation.
  Eigen::Vector3d t{Eigen::Vector3d::Zero()}; ///< World-to-camera translation.
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()}; ///< Normalized rotation view.

  CameraPose() = default;
  /// Construct an untimestamped world-to-camera pose from a rotation matrix.
  CameraPose(const Eigen::Matrix3d& rotation,
             const Eigen::Vector3d& translation)
      : R(rotation), t(translation), q(rotation) {
    q.normalize();
  }
  CameraPose(const Eigen::Quaterniond& quaternion,
             const Eigen::Vector3d& translation)
      : R(quaternion.normalized().toRotationMatrix()),
        t(translation), q(quaternion.normalized()) {}
  CameraPose(Timestamp timestamp, const Eigen::Matrix3d& rotation,
             const Eigen::Vector3d& translation)
      : timestamp_ns(timestamp), R(rotation), t(translation), q(rotation) {
    q.normalize();
  }
  CameraPose(Timestamp timestamp, const Eigen::Quaterniond& quaternion,
             const Eigen::Vector3d& translation)
      : timestamp_ns(timestamp), R(quaternion.normalized().toRotationMatrix()),
        t(translation), q(quaternion.normalized()) {}

  /// Return the `[R | t]` world-to-camera matrix.
  Eigen::Matrix<double, 3, 4> matrix3x4() const {
    Eigen::Matrix<double, 3, 4> result;
    result << R, t;
    return result;
  }
  /// Return the homogeneous world-to-camera matrix.
  Eigen::Matrix4d matrix() const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.topLeftCorner<3, 3>() = R;
    result.topRightCorner<3, 1>() = t;
    return result;
  }
  /// Return the homogeneous camera-to-world inverse.
  Eigen::Matrix4d inverse_matrix() const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.topLeftCorner<3, 3>() = R.transpose();
    result.topRightCorner<3, 1>() = -R.transpose() * t;
    return result;
  }

  /// Return the normalized quaternion rotation as a matrix.
  Eigen::Matrix3d quat_to_R() const { return q.normalized().toRotationMatrix(); }
  /// Legacy alias for @ref matrix3x4.
  Eigen::Matrix<double, 3, 4> make_Rt_in_3x4() const { return matrix3x4(); }
  /// Legacy alias for @ref matrix.
  Eigen::Matrix4d make_Rt_in_4x4() const { return matrix(); }
  /// Legacy alias for @ref inverse_matrix.
  Eigen::Matrix4d inverse_in_4x4() const { return inverse_matrix(); }
  /// Rotate a point by the pose's world-to-camera rotation only.
  Eigen::Vector3d rotate(const Eigen::Vector3d& point) const { return R * point; }
  /// Transform a world point into camera coordinates.
  Eigen::Vector3d transform(const Eigen::Vector3d& point) const {
    return rotate(point) + t;
  }
  /// Transform a camera point back into world coordinates.
  Eigen::Vector3d detransform(const Eigen::Vector3d& point) const {
    return R.transpose() * (point - t);
  }
  /// Return the camera center expressed in world coordinates.
  Eigen::Vector3d center() const { return -R.transpose() * t; }
  /// Print this pose using the legacy Brown-LEMS format.
  void print_Camera_Pose(const std::string& pose_name) const {
    std::cout << pose_name << ":\n"
              << "- Rotation:\n" << R << "\n"
              << "- Translation:\n" << t.transpose() << "\n\n";
  }
};

/**
 * @brief Native camera calibration retained in Eigen/OpenCV-compatible form.
 * @ingroup cameras
 */
struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name; ///< Reader-assigned camera name, such as `cam0`.
  std::string model{"pinhole"}; ///< Native camera model label.
  cv::Size resolution; ///< Image dimensions as width and height.
  Eigen::Vector4d intrinsics{Eigen::Vector4d::Zero()}; ///< `[fx, fy, cx, cy]`.
  Eigen::VectorXd distortion; ///< Native distortion coefficients.
  Eigen::Matrix3d K{Eigen::Matrix3d::Identity()}; ///< Pinhole matrix.
  Eigen::Matrix3d R_rect{Eigen::Matrix3d::Identity()}; ///< Rectification rotation.
  Eigen::Matrix<double, 3, 4> P{
      Eigen::Matrix<double, 3, 4>::Zero()}; ///< Projection matrix.
  Eigen::Isometry3d T_body_camera{Eigen::Isometry3d::Identity()}; ///< Body-from-camera extrinsic.

  /// Return @c K as a newly allocated OpenCV matrix.
  cv::Mat camera_matrix_cv() const;
  /// Return distortion coefficients as a row OpenCV matrix.
  cv::Mat distortion_cv() const;
};

/**
 * @brief Relative stereo transform and fundamental matrix.
 * @ingroup cameras
 *
 * `R_target_reference` and `t_target_reference` map reference-camera points
 * into target-camera coordinates. `baseline` is their translation norm.
 */
struct StereoCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string reference_camera{"cam0"}; ///< Reference camera name.
  std::string target_camera{"cam1"}; ///< Target camera name.
  Eigen::Matrix3d R_target_reference{Eigen::Matrix3d::Identity()}; ///< Reference-to-target rotation.
  Eigen::Vector3d t_target_reference{Eigen::Vector3d::Zero()}; ///< Reference-to-target translation.
  Eigen::Matrix3d F_target_reference{Eigen::Matrix3d::Zero()}; ///< Fundamental matrix in target/reference direction.
  double baseline{}; ///< Norm of @c t_target_reference, in native distance units (readers use metres).
};

/** @brief Optional dense paths and decoded matrices attached to a frame. @ingroup datasets */
struct FrameMetadata {
  std::optional<std::filesystem::path> disparity_path; ///< Optional disparity file.
  std::optional<std::filesystem::path> occlusion_mask_path; ///< Optional non-occlusion mask file.
  std::optional<std::filesystem::path> depth_path; ///< Optional depth file.
  cv::Mat disparity; ///< Decoded disparity; may shallow-share legacy alias.
  cv::Mat occlusion_mask; ///< Decoded mask; may shallow-share legacy alias.
  cv::Mat depth; ///< Decoded depth matrix.
};

/**
 * @brief One camera observation in a synchronized frame set.
 * @ingroup datasets
 *
 * The value type keeps canonical metadata alongside legacy fields. OpenCV
 * matrices are reference-counted shallow copies. `synchronize_legacy_fields`
 * updates timestamp, dense aliases, pose flag, and legacy pose from canonical
 * fields.
 */
struct Frame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string camera; ///< Camera identity, such as `cam0`.
  Timestamp timestamp_ns{}; ///< Authoritative timestamp in nanoseconds.
  double timestamp_seconds{}; ///< Floating-point seconds convenience value.
  double timestamp{}; ///< Legacy seconds spelling, synchronized by readers.
  std::filesystem::path image_path; ///< Source image path.
  cv::Mat image; ///< Decoded grayscale image after iterator materialization.
  cv::Mat image_undistorted; ///< Pipeline-owned undistorted image.
  cv::Mat image_gradients_x; ///< Pipeline-owned x gradient.
  cv::Mat image_gradients_y; ///< Pipeline-owned y gradient.
  std::vector<Edge> edges; ///< Detector output; this library does not populate it.
  cv::Mat disparity_map; ///< Legacy alias of @c metadata.disparity.
  cv::Mat occlusion_mask; ///< Legacy alias of @c metadata.occlusion_mask.
  CameraPose gt_camera_pose; ///< Legacy pose view; identity when absent.
  bool has_ground_truth{false}; ///< Whether @c ground_truth contains a pose.
  std::optional<CameraPose> ground_truth; ///< Optional world-to-camera pose.
  FrameMetadata metadata; ///< Canonical dense paths and matrices.
  Eigen::Matrix3d K{Eigen::Matrix3d::Identity()}; ///< Camera intrinsics for this frame.

  /// Synchronize legacy fields from canonical timestamps, metadata, and pose.
  void synchronize_legacy_fields() {
    timestamp = timestamp_seconds;
    disparity_map = metadata.disparity;
    occlusion_mask = metadata.occlusion_mask;
    has_ground_truth = ground_truth.has_value();
    gt_camera_pose = ground_truth.value_or(CameraPose{});
  }
};

/**
 * @brief Synchronized observations for one time or complete temporal window.
 * @ingroup datasets
 *
 * The frame vector is not required to contain exactly two cameras; inspect
 * each frame's camera name and timestamp.
 */
struct FrameSet {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::size_t index{}; ///< Reader/window index.
  Timestamp timestamp_ns{}; ///< Window start timestamp in nanoseconds.
  std::vector<Frame> frames; ///< Synchronized camera observations.
};

} // namespace lems::data
