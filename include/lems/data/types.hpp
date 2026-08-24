#pragma once
#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lems::data {

using Timestamp = std::int64_t; // nanoseconds

struct Pose {
  Timestamp timestamp_ns{};
  std::array<double, 3> translation{};
  std::array<double, 4> quaternion{1., 0., 0., 0.}; // w, x, y, z
};

struct Camera {
  std::string name;
  std::string model{"pinhole"};
  std::array<int, 2> resolution{}; // width, height
  std::array<double, 4> intrinsics{}; // fx, fy, cx, cy
  std::vector<double> distortion;
  std::array<double, 9> K{1,0,0, 0,1,0, 0,0,1};
  std::array<double, 9> R_rect{1,0,0, 0,1,0, 0,0,1};
  std::array<double, 12> projection{1,0,0,0, 0,1,0,0, 0,0,1,0};
  std::array<double, 16> T_body_camera{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
};

struct StereoCalibration {
  std::string reference_camera{"cam0"};
  std::string target_camera{"cam1"};
  std::array<double, 9> R_target_reference{1,0,0, 0,1,0, 0,0,1};
  std::array<double, 3> t_target_reference{};
  std::array<double, 9> fundamental{};
  double baseline{};
};

struct FrameMetadata {
  std::optional<std::filesystem::path> disparity_path;
  std::optional<std::filesystem::path> occlusion_mask_path;
  std::optional<std::filesystem::path> depth_path;
};

struct Frame {
  std::string camera;
  Timestamp timestamp_ns{};
  std::filesystem::path image_path; // deliberately lazy: algorithms choose their image backend
  std::optional<Pose> ground_truth;
  FrameMetadata metadata;
};

struct FrameSet {
  std::size_t index{};
  Timestamp timestamp_ns{};
  std::vector<Frame> frames;
};

} // namespace lems::data
