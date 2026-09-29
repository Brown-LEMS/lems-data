#pragma once

// Drop-in compatibility surface for the Brown-LEMS stereo and multinocular
// VO repositories. Put this directory before the project's include directory,
// or replace the repository's Dataset.h with this forwarding header.

#include <lems/data/dataset.hpp>
#include <lems/data/pipeline_types.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <optional>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>

// These aliases are deliberate: old VO translation units must store and pass
// the same objects as the canonical library, rather than silently creating a
// second Edge/pose/frame ABI.
using Edge = lems::data::Edge;
using Edge_3D = lems::data::Edge_3D;
using FileInfo = lems::data::FileInfo;
using Camera = lems::data::Camera;
using CameraInfo = lems::data::CameraInfo;
using Camera_Pose = lems::data::CameraPose;
using Frame = lems::data::Frame;
#ifndef LEMS_DATA_STEREO_COMPAT
using SpatialGrid = lems::data::SpatialGrid;
using scores = lems::data::scores;
using EdgeCluster = lems::data::EdgeCluster;
using Evaluation_Statistics = lems::data::Evaluation_Statistics;
using Observer = lems::data::Observer;
using Main_Observer = lems::data::Main_Observer;
using Sub_Observer = lems::data::Sub_Observer;
using Internal_Matching_Edge_Clusters =
    lems::data::Internal_Matching_Edge_Clusters;
using EdgeMatch = lems::data::EdgeMatch;
using Edge_Loop = lems::data::Edge_Loop;
using Camera_Set = lems::data::Camera_Set;
using final_stereo_edge_pair = lems::data::final_stereo_edge_pair;
using Temporal_CF_Edge_Cluster = lems::data::Temporal_CF_Edge_Cluster;
using temporal_edge_pair = lems::data::temporal_edge_pair;
using Veridical_Quad_Entry = lems::data::Veridical_Quad_Entry;
using Candidate_Quad_Entry = lems::data::Candidate_Quad_Entry;
using KF_Temporal_Edge_Quads = lems::data::KF_Temporal_Edge_Quads;
using Temporal_View_Match = lems::data::Temporal_View_Match;
using Temporal_Candidate = lems::data::Temporal_Candidate;
using KF_Temporal_Match = lems::data::KF_Temporal_Match;
#endif

#ifndef LEMS_DATA_STEREO_COMPAT
struct StereoFrame {
  // Canonical observations retain camera identity, nanosecond timestamps,
  // source paths, and deferred metadata alongside the legacy split fields.
  lems::data::Frame left_frame;
  lems::data::Frame right_frame;
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
#endif

inline Camera_Pose lems_compat_pose(
    const std::optional<Camera_Pose>& pose) {
  return pose.value_or(Camera_Pose{});
}

inline cv::Mat eigen_camera_matrix(const Camera& camera) {
  cv::Mat result;
  cv::eigen2cv(camera.K, result);
  return result;
}

inline cv::Mat vector_camera_matrix(const std::vector<double>& values) {
  if (values.empty()) return {};
  cv::Mat result(1, static_cast<int>(values.size()), CV_64F);
  for (int i = 0; i < result.cols; ++i) result.at<double>(0, i) = values[i];
  return result;
}

inline std::string lems_compat_pfm_token(std::istream& input) {
  std::string token;
  char ch = 0;
  while (input.get(ch)) {
    if (ch == '#') {
      std::string ignored;
      std::getline(input, ignored);
      continue;
    }
    if (!std::isspace(static_cast<unsigned char>(ch))) {
      token.push_back(ch);
      break;
    }
  }
  while (input.get(ch) && !std::isspace(static_cast<unsigned char>(ch)))
    token.push_back(ch);
  return token;
}

class StereoIterator {
 public:
  explicit StereoIterator(std::unique_ptr<lems::data::DatasetIterator> iterator)
      : iterator_(std::move(iterator)) {}
  bool hasNext() const { return iterator_ && iterator_->has_next(); }
  bool getNext(StereoFrame& output) {
    if (!iterator_ || !iterator_->has_next()) return false;
    auto next = iterator_->next();
    if (!next || next->frames.size() < 2) return false;
    const auto& left = next->frames[0];
    const auto& right = next->frames[1];
    output.left_frame = left;
    output.right_frame = right;
    output.left_image = left.image;
    output.left_image_undistorted = left.image_undistorted;
    output.right_image = right.image;
    output.right_image_undistorted = right.image_undistorted;
    output.timestamp = left.timestamp_seconds;
    output.left_image_gradients_x = left.image_gradients_x;
    output.right_image_gradients_x = right.image_gradients_x;
    output.left_image_gradients_y = left.image_gradients_y;
    output.right_image_gradients_y = right.image_gradients_y;
    output.left_edges = left.edges;
    output.right_edges = right.edges;
    output.left_disparity_map = left.metadata.disparity;
    output.right_disparity_map = right.metadata.disparity;
    output.left_occlusion_mask = left.metadata.occlusion_mask;
    output.right_occlusion_mask = right.metadata.occlusion_mask;
    output.gt_camera_pose = lems_compat_pose(left.ground_truth);
    return true;
  }
  void reset() { if (iterator_) iterator_->reset(); }
  std::size_t size() const { return iterator_ ? iterator_->size() : 0; }

 private:
  std::unique_ptr<lems::data::DatasetIterator> iterator_;
};

class MultinocularIterator {
 public:
  explicit MultinocularIterator(
      std::unique_ptr<lems::data::DatasetIterator> iterator)
      : iterator_(std::move(iterator)) {}
  bool hasNext() const { return iterator_ && iterator_->has_next(); }
  bool getNext(std::vector<Frame>& output) {
    if (!iterator_ || !iterator_->has_next()) return false;
    auto next = iterator_->next();
    if (!next) return false;
    output = std::move(next->frames);
    return true;
  }
  void reset() { if (iterator_) iterator_->reset(); }
  std::size_t size() const { return iterator_ ? iterator_->size() : 0; }

 private:
  std::unique_ptr<lems::data::DatasetIterator> iterator_;
};

class Dataset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Dataset>;

  explicit Dataset(const YAML::Node& node)
      : core_(lems::data::open_dataset(lems::data::load_config(node))),
        file_info(core_->file_info()), camera_info(core_->camera_info()),
        left_height_(core_->height()), left_width_(core_->width()),
        right_height_(core_->height()), right_width_(core_->width()) {}

  void load_dataset(const std::string&, std::vector<cv::Mat>& left_disparities,
                    std::vector<cv::Mat>& right_disparities,
                    std::vector<cv::Mat>& left_masks,
                    std::vector<cv::Mat>& right_masks) {
    left_disparities.clear();
    right_disparities.clear();
    left_masks.clear();
    right_masks.clear();
    has_dense_reference_ = false;
    stereo_iterator = std::make_unique<StereoIterator>(core_->iterate());
    collect_dense(left_disparities, right_disparities, left_masks, right_masks);
  }

  void load_dataset(const std::string&,
                    std::vector<std::vector<cv::Mat>>& disparities,
                    std::vector<std::vector<cv::Mat>>& masks) {
    disparities.clear();
    masks.clear();
    has_dense_reference_ = false;
    multinocular_iterator =
        std::make_unique<MultinocularIterator>(core_->iterate());
    auto iterator = core_->iterate();
    while (auto set = iterator->next()) {
      std::vector<cv::Mat> frame_disparities, frame_masks;
      for (const auto& frame : set->frames) {
        frame_disparities.push_back(frame.metadata.disparity);
        frame_masks.push_back(frame.metadata.occlusion_mask);
        has_dense_reference_ |= !frame.metadata.disparity.empty();
      }
      disparities.push_back(std::move(frame_disparities));
      masks.push_back(std::move(frame_masks));
    }
  }

  // The old VO name means dense disparity/reference products. Pose
  // availability is intentionally separate in has_ground_truth().
  bool has_gt() const { return has_dense_reference_; }
  bool has_ground_truth() const { return core_->has_ground_truth(); }
  std::size_t size() const { return core_->size(); }
  int width() const { return core_->width(); }
  int height() const { return core_->height(); }
  const FileInfo& get_file_info() const { return file_info; }
  const CameraInfo& get_camera_info() const { return camera_info; }
  std::string get_dataset_type() const { return file_info.dataset_type; }
  std::string get_output_path() const { return file_info.output_path; }
  std::string get_stereo_pairs_path() const {
    return (std::filesystem::path(file_info.dataset_path) /
            file_info.sequence_name / "stereo_pairs").string();
  }
  int get_omp_threads() const {
    auto it = core_->config().options.find("omp_threads");
    return it == core_->config().options.end() ? 1 : std::stoi(it->second);
  }
  // This is the legacy processed-image counter used by Pipeline.cpp to
  // lazily initialize detector state. Dataset::size() is the stream count.
  unsigned get_num_imgs() const { return total_num_images_; }
  void increment_num_imgs() { ++total_num_images_; }

  Eigen::Matrix3d get_left_calib_matrix() const { return camera_info.left.K; }
  Eigen::Matrix3d get_right_calib_matrix() const { return camera_info.right.K; }
  cv::Mat get_left_calib_matrix_cvMat() const { return eigen_camera_matrix(camera_info.left); }
  cv::Mat get_right_calib_matrix_cvMat() const { return eigen_camera_matrix(camera_info.right); }
  cv::Mat get_left_dist_coeff_mat() const { return vector_camera_matrix(camera_info.left.distortion); }
  cv::Mat get_right_dist_coeff_mat() const { return vector_camera_matrix(camera_info.right.distortion); }
  Eigen::Matrix3d get_fund_mat_21() const { return camera_info.left.F; }
  Eigen::Matrix3d get_fund_mat_12() const {
    return camera_info.right.F;
  }
  Eigen::Matrix3d get_relative_rot_left_to_right() const {
    return camera_info.left.R;
  }
  Eigen::Vector3d get_relative_transl_left_to_right() const {
    return camera_info.left.T;
  }
  Eigen::Matrix3d get_relative_rot_right_to_left() const {
    return camera_info.right.R;
  }
  Eigen::Vector3d get_relative_transl_right_to_left() const {
    return camera_info.right.T;
  }
  std::vector<double> left_intr() const {
    return camera_info.left.intrinsics;
  }
  std::vector<double> right_intr() const {
    return camera_info.right.intrinsics;
  }
  std::vector<double> left_dist_coeffs() const {
    return camera_info.left.distortion;
  }
  std::vector<double> right_dist_coeffs() const {
    return camera_info.right.distortion;
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
    const auto type = lems_compat_pfm_token(input);
    const auto width_text = lems_compat_pfm_token(input);
    const auto height_text = lems_compat_pfm_token(input);
    const auto scale_text = lems_compat_pfm_token(input);
    if ((type != "Pf" && type != "PF") || width_text.empty() ||
        height_text.empty() || scale_text.empty())
      throw std::runtime_error("invalid PFM header: " + path);
    int width = 0;
    int height = 0;
    float scale = 0.0f;
    try {
      width = std::stoi(width_text);
      height = std::stoi(height_text);
      scale = std::stof(scale_text);
    } catch (const std::exception&) {
      throw std::runtime_error("invalid PFM dimensions/scale: " + path);
    }
    if (width <= 0 || height <= 0 || !std::isfinite(scale) || scale == 0.0f)
      throw std::runtime_error("invalid PFM dimensions/scale: " + path);
    const int channels = type == "PF" ? 3 : 1;
    const auto sample_count = static_cast<std::uint64_t>(width) *
                              static_cast<std::uint64_t>(height) *
                              static_cast<std::uint64_t>(channels);
    if (sample_count >
        static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max() /
                                   sizeof(float)))
      throw std::runtime_error("PFM payload is too large: " + path);
    cv::Mat result(height, width, channels == 3 ? CV_32FC3 : CV_32FC1);
    const std::uint16_t marker = 1;
    const bool host_little_endian =
        *reinterpret_cast<const unsigned char*>(&marker) == 1;
    const bool swap_bytes = (scale < 0.0f) != host_little_endian;
    const float magnitude = std::abs(scale);
    std::array<std::uint8_t, sizeof(float)> bytes{};
    for (int source_row = 0; source_row < height; ++source_row) {
      const int destination_row = height - source_row - 1;
      for (int x = 0; x < width; ++x) {
        for (int channel = 0; channel < channels; ++channel) {
          input.read(reinterpret_cast<char*>(bytes.data()), sizeof(float));
          if (!input) throw std::runtime_error("truncated PFM payload: " + path);
          if (swap_bytes) std::reverse(bytes.begin(), bytes.end());
          float value = 0.0f;
          std::memcpy(&value, bytes.data(), sizeof(value));
          value *= magnitude;
          if (channels == 1)
            result.at<float>(destination_row, x) = value;
          else
            result.at<cv::Vec3f>(destination_row, x)[channel] = value;
        }
      }
    }
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
      has_dense_reference_ |= !set->frames[0].metadata.disparity.empty() ||
                             !set->frames[1].metadata.disparity.empty();
    }
  }

  std::unique_ptr<lems::data::Dataset> core_;
 public:
  // References keep the legacy field names bound to the canonical records
  // owned by the core dataset for this adapter's entire lifetime.
  const FileInfo& file_info;
  const CameraInfo& camera_info;

 private:
  bool has_dense_reference_{};
  unsigned total_num_images_{};
  int left_height_{}, left_width_{}, right_height_{}, right_width_{};
};
