#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lems::data {

using Timestamp = std::int64_t; // nanoseconds

struct CameraPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_ns{};
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};

  CameraPose() = default;
  CameraPose(Timestamp timestamp, const Eigen::Matrix3d& rotation,
             const Eigen::Vector3d& translation)
      : timestamp_ns(timestamp), R(rotation), t(translation), q(rotation) {
    q.normalize();
  }
  CameraPose(Timestamp timestamp, const Eigen::Quaterniond& quaternion,
             const Eigen::Vector3d& translation)
      : timestamp_ns(timestamp), R(quaternion.normalized().toRotationMatrix()),
        t(translation), q(quaternion.normalized()) {}

  Eigen::Matrix<double, 3, 4> matrix3x4() const {
    Eigen::Matrix<double, 3, 4> result;
    result << R, t;
    return result;
  }
  Eigen::Matrix4d matrix() const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.topLeftCorner<3, 3>() = R;
    result.topRightCorner<3, 1>() = t;
    return result;
  }
  Eigen::Matrix4d inverse_matrix() const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.topLeftCorner<3, 3>() = R.transpose();
    result.topRightCorner<3, 1>() = -R.transpose() * t;
    return result;
  }
};

struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name;
  std::string model{"pinhole"};
  cv::Size resolution;
  Eigen::Vector4d intrinsics{Eigen::Vector4d::Zero()}; // fx, fy, cx, cy
  Eigen::VectorXd distortion;
  Eigen::Matrix3d K{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d R_rect{Eigen::Matrix3d::Identity()};
  Eigen::Matrix<double, 3, 4> P{
      Eigen::Matrix<double, 3, 4>::Zero()};
  Eigen::Isometry3d T_body_camera{Eigen::Isometry3d::Identity()};

  cv::Mat camera_matrix_cv() const;
  cv::Mat distortion_cv() const;
};

struct StereoCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string reference_camera{"cam0"};
  std::string target_camera{"cam1"};
  Eigen::Matrix3d R_target_reference{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_target_reference{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d F_target_reference{Eigen::Matrix3d::Zero()};
  double baseline{};
};

struct FrameMetadata {
  std::optional<std::filesystem::path> disparity_path;
  std::optional<std::filesystem::path> occlusion_mask_path;
  std::optional<std::filesystem::path> depth_path;
  cv::Mat disparity;
  cv::Mat occlusion_mask;
  cv::Mat depth;
};

struct Frame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string camera;
  Timestamp timestamp_ns{};
  double timestamp_seconds{};
  std::filesystem::path image_path;
  cv::Mat image;
  cv::Mat image_undistorted;
  std::optional<CameraPose> ground_truth;
  FrameMetadata metadata;
  Eigen::Matrix3d K{Eigen::Matrix3d::Identity()};
};

struct FrameSet {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::size_t index{};
  Timestamp timestamp_ns{};
  std::vector<Frame> frames;
};

} // namespace lems::data
