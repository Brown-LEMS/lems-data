#pragma once

// Drop-in compatibility surface for the Brown-LEMS stereo and multinocular
// VO repositories. Put this directory before the project's include directory,
// or replace the repository's Dataset.h with this forwarding header.

#include <lems/data/dataset.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include "toed/cpu_toed.hpp"

#include <memory>
#include <optional>
#include <fstream>
#include <vector>

struct alignas(32) Camera_Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};

  Camera_Pose() = default;
  Camera_Pose(const Eigen::Quaterniond& quaternion,
              const Eigen::Vector3d& translation)
      : R(quaternion.normalized().toRotationMatrix()), t(translation),
        q(quaternion.normalized()) {}
  Camera_Pose(const Eigen::Matrix3d& rotation,
              const Eigen::Vector3d& translation)
      : R(rotation), t(translation), q(rotation) { q.normalize(); }

  Eigen::Matrix3d quat_to_R() const { return q.normalized().toRotationMatrix(); }
  Eigen::Matrix<double, 3, 4> make_Rt_in_3x4() const {
    Eigen::Matrix<double, 3, 4> result;
    result << R, t;
    return result;
  }
  Eigen::Matrix4d make_Rt_in_4x4() const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.topLeftCorner<3, 3>() = R;
    result.topRightCorner<3, 1>() = t;
    return result;
  }
  Eigen::Matrix4d inverse_in_4x4() const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.topLeftCorner<3, 3>() = R.transpose();
    result.topRightCorner<3, 1>() = -R.transpose() * t;
    return result;
  }
  Eigen::Vector3d rotate(const Eigen::Vector3d& point) const { return R * point; }
  Eigen::Vector3d detransform(const Eigen::Vector3d& point) const {
    return R.transpose() * (point - t);
  }
  Eigen::Vector3d transform(const Eigen::Vector3d& point) const {
    return rotate(point) + t;
  }
  Eigen::Vector3d center() const { return -R.transpose() * t; }
};

struct StereoFrame {
  cv::Mat left_image, right_image;
  cv::Mat left_image_undistorted, right_image_undistorted;
  double timestamp{};
  cv::Mat left_image_gradients_x, right_image_gradients_x;
  cv::Mat left_image_gradients_y, right_image_gradients_y;
  std::vector<Edge> left_edges, right_edges;
  cv::Mat left_disparity_map, right_disparity_map;
  cv::Mat left_occlusion_mask, right_occlusion_mask;
  Camera_Pose gt_camera_pose;
};

struct Frame {
  cv::Mat image, image_undistorted;
  double timestamp{};
  cv::Mat image_gradients_x, image_gradients_y;
  std::vector<Edge> edges;
  cv::Mat disparity_map, occlusion_mask;
  Camera_Pose gt_camera_pose;
  Eigen::Matrix3d K{Eigen::Matrix3d::Identity()};
};

inline Camera_Pose lems_compat_pose(
    const std::optional<lems::data::CameraPose>& pose) {
  return pose ? Camera_Pose(pose->R, pose->t) : Camera_Pose();
}

class StereoIterator {
 public:
  explicit StereoIterator(std::unique_ptr<lems::data::DatasetIterator> iterator)
      : iterator_(std::move(iterator)) { advance(); }
  bool hasNext() { return next_.has_value(); }
  bool getNext(StereoFrame& output) {
    if (!next_ || next_->frames.size() < 2) return false;
    const auto& left = next_->frames[0];
    const auto& right = next_->frames[1];
    output.left_image = left.image;
    output.right_image = right.image;
    output.timestamp = left.timestamp_seconds;
    output.left_disparity_map = left.metadata.disparity;
    output.right_disparity_map = right.metadata.disparity;
    output.left_occlusion_mask = left.metadata.occlusion_mask;
    output.right_occlusion_mask = right.metadata.occlusion_mask;
    output.gt_camera_pose = lems_compat_pose(left.ground_truth);
    advance();
    return true;
  }
  void reset() { iterator_->reset(); advance(); }

 private:
  void advance() { next_ = iterator_->next(); }
  std::unique_ptr<lems::data::DatasetIterator> iterator_;
  std::optional<lems::data::FrameSet> next_;
};

class MultinocularIterator {
 public:
  explicit MultinocularIterator(
      std::unique_ptr<lems::data::DatasetIterator> iterator)
      : iterator_(std::move(iterator)) { advance(); }
  bool hasNext() { return next_.has_value(); }
  bool getNext(std::vector<Frame>& output) {
    if (!next_) return false;
    output.clear();
    output.reserve(next_->frames.size());
    for (const auto& input : next_->frames) {
      Frame frame;
      frame.image = input.image;
      frame.timestamp = input.timestamp_seconds;
      frame.disparity_map = input.metadata.disparity;
      frame.occlusion_mask = input.metadata.occlusion_mask;
      frame.gt_camera_pose = lems_compat_pose(input.ground_truth);
      frame.K = input.K;
      output.push_back(std::move(frame));
    }
    advance();
    return true;
  }
  void reset() { iterator_->reset(); advance(); }

 private:
  void advance() { next_ = iterator_->next(); }
  std::unique_ptr<lems::data::DatasetIterator> iterator_;
  std::optional<lems::data::FrameSet> next_;
};

class Dataset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Dataset>;

  explicit Dataset(const YAML::Node& node)
      : core_(lems::data::open_dataset(lems::data::load_config(node))) {}

  void load_dataset(const std::string&, std::vector<cv::Mat>& left_disparities,
                    std::vector<cv::Mat>& right_disparities,
                    std::vector<cv::Mat>& left_masks,
                    std::vector<cv::Mat>& right_masks) {
    stereo_iterator = std::make_unique<StereoIterator>(core_->iterate());
    collect_dense(left_disparities, right_disparities, left_masks, right_masks);
  }

  void load_dataset(const std::string&,
                    std::vector<std::vector<cv::Mat>>& disparities,
                    std::vector<std::vector<cv::Mat>>& masks) {
    multinocular_iterator =
        std::make_unique<MultinocularIterator>(core_->iterate());
    auto iterator = core_->iterate();
    while (auto set = iterator->next()) {
      std::vector<cv::Mat> frame_disparities, frame_masks;
      for (const auto& frame : set->frames) {
        frame_disparities.push_back(frame.metadata.disparity);
        frame_masks.push_back(frame.metadata.occlusion_mask);
        has_ground_truth_ |= frame.ground_truth.has_value() ||
                             !frame.metadata.disparity.empty();
      }
      disparities.push_back(std::move(frame_disparities));
      masks.push_back(std::move(frame_masks));
    }
  }

  bool has_gt() const { return has_ground_truth_; }
  std::string get_dataset_type() const { return core_->config().type; }
  std::string get_output_path() const {
    auto it = core_->config().options.find("output_dir");
    return it == core_->config().options.end() ? "./outputs" : it->second;
  }
  std::string get_stereo_pairs_path() const {
    return (core_->config().root / core_->config().sequence / "stereo_pairs").string();
  }
  int get_omp_threads() const { return 1; }
  unsigned get_num_imgs() const { return total_num_images_; }
  void increment_num_imgs() { ++total_num_images_; }

  Eigen::Matrix3d get_left_calib_matrix() const { return camera(0).K; }
  Eigen::Matrix3d get_right_calib_matrix() const { return camera(1).K; }
  cv::Mat get_left_calib_matrix_cvMat() const { return camera(0).camera_matrix_cv(); }
  cv::Mat get_right_calib_matrix_cvMat() const { return camera(1).camera_matrix_cv(); }
  cv::Mat get_left_dist_coeff_mat() const { return camera(0).distortion_cv(); }
  cv::Mat get_right_dist_coeff_mat() const { return camera(1).distortion_cv(); }
  Eigen::Matrix3d get_fund_mat_21() const { return stereo().F_target_reference; }
  Eigen::Matrix3d get_fund_mat_12() const {
    const auto& calibration = stereo();
    Eigen::Vector3d inverse_t =
        -calibration.R_target_reference.transpose() *
        calibration.t_target_reference;
    Eigen::Matrix3d tx;
    tx << 0.0, -inverse_t.z(), inverse_t.y(), inverse_t.z(), 0.0,
        -inverse_t.x(), -inverse_t.y(), inverse_t.x(), 0.0;
    return camera(0).K.inverse().transpose() * tx *
           calibration.R_target_reference.transpose() * camera(1).K.inverse();
  }
  Eigen::Matrix3d get_relative_rot_left_to_right() const {
    return stereo().R_target_reference;
  }
  Eigen::Vector3d get_relative_transl_left_to_right() const {
    return stereo().t_target_reference;
  }
  Eigen::Matrix3d get_relative_rot_right_to_left() const {
    return stereo().R_target_reference.transpose();
  }
  Eigen::Vector3d get_relative_transl_right_to_left() const {
    return -stereo().R_target_reference.transpose() * stereo().t_target_reference;
  }
  std::vector<double> left_intr() const {
    const auto& value = camera(0).intrinsics;
    return {value[0], value[1], value[2], value[3]};
  }
  std::vector<double> right_intr() const {
    const auto& value = camera(1).intrinsics;
    return {value[0], value[1], value[2], value[3]};
  }
  std::vector<double> left_dist_coeffs() const {
    const auto& value = camera(0).distortion;
    return std::vector<double>(value.data(), value.data() + value.size());
  }
  std::vector<double> right_dist_coeffs() const {
    const auto& value = camera(1).distortion;
    return std::vector<double>(value.data(), value.data() + value.size());
  }

  int get_left_height() const { return left_height_; }
  int get_left_width() const { return left_width_; }
  int get_right_height() const { return right_height_; }
  int get_right_width() const { return right_width_; }
  int get_height() const { return left_height_; }
  int get_width() const { return left_width_; }
  void set_left_height(int value) { left_height_ = value; }
  void set_left_width(int value) { left_width_ = value; }
  void set_right_height(int value) { right_height_ = value; }
  void set_right_width(int value) { right_width_ = value; }
  void set_height(int value) { left_height_ = right_height_ = value; }
  void set_width(int value) { left_width_ = right_width_ = value; }

  cv::Mat readPFM(const std::string& path) const {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open PFM: " + path);
    std::string type;
    int width, height;
    float scale;
    input >> type >> width >> height >> scale;
    input.get();
    if ((type != "Pf" && type != "PF") || width <= 0 || height <= 0)
      throw std::runtime_error("invalid PFM: " + path);
    cv::Mat result(height, width, type == "PF" ? CV_32FC3 : CV_32FC1);
    input.read(reinterpret_cast<char*>(result.data),
               static_cast<std::streamsize>(result.total() * result.elemSize()));
    if (!input) throw std::runtime_error("truncated PFM: " + path);
    cv::flip(result, result, 0);
    return result;
  }
  bool readDispETH3D(const std::string& path, cv::Mat& disparity,
                     cv::Mat& valid_mask) const {
    try {
      disparity = readPFM(path);
      auto mask_path = std::filesystem::path(path).parent_path() /
          (std::filesystem::path(path).filename().string().find("disp1") == 0
               ? "mask1nocc.png" : "mask0nocc.png");
      valid_mask = cv::imread(mask_path.string(), cv::IMREAD_GRAYSCALE);
      return !disparity.empty();
    } catch (...) { return false; }
  }
  bool readDispMiddlebury(const std::string& path, cv::Mat& disparity,
                          cv::Mat& valid_mask) const {
    return readDispETH3D(path, disparity, valid_mask);
  }

  std::unique_ptr<StereoIterator> stereo_iterator;
  std::unique_ptr<MultinocularIterator> multinocular_iterator;
  std::vector<Edge> left_edges, right_edges;

 private:
  const lems::data::CameraCalibration& camera(std::size_t index) const {
    return core_->cameras().at(index);
  }
  const lems::data::StereoCalibration& stereo() const {
    return core_->stereo_calibration().value();
  }
  void collect_dense(std::vector<cv::Mat>& left_disparities,
                     std::vector<cv::Mat>& right_disparities,
                     std::vector<cv::Mat>& left_masks,
                     std::vector<cv::Mat>& right_masks) {
    auto iterator = core_->iterate();
    while (auto set = iterator->next()) {
      if (set->frames.size() < 2) continue;
      left_disparities.push_back(set->frames[0].metadata.disparity);
      right_disparities.push_back(set->frames[1].metadata.disparity);
      left_masks.push_back(set->frames[0].metadata.occlusion_mask);
      right_masks.push_back(set->frames[1].metadata.occlusion_mask);
      has_ground_truth_ |= set->frames[0].ground_truth.has_value() ||
                           !set->frames[0].metadata.disparity.empty();
    }
  }

  std::unique_ptr<lems::data::Dataset> core_;
  bool has_ground_truth_{};
  unsigned total_num_images_{};
  int left_height_{}, left_width_{}, right_height_{}, right_width_{};
};
