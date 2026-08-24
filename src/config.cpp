#include "lems/data/config.hpp"

#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <stdexcept>

namespace lems::data {
namespace {

template <int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> matrix_from_yaml(const YAML::Node& node) {
  Eigen::Matrix<double, Rows, Cols> result;
  if (!node) throw std::runtime_error("missing calibration matrix");

  if (node.IsSequence() && node.size() == Rows && node[0].IsSequence()) {
    for (int row = 0; row < Rows; ++row) {
      if (node[row].size() != Cols)
        throw std::runtime_error("invalid calibration matrix dimensions");
      for (int col = 0; col < Cols; ++col)
        result(row, col) = node[row][col].as<double>();
    }
    return result;
  }

  const YAML::Node values = node["data"] ? node["data"] : node;
  if (!values.IsSequence() || values.size() != Rows * Cols)
    throw std::runtime_error("invalid flattened calibration matrix");
  for (int i = 0; i < Rows * Cols; ++i)
    result(i / Cols, i % Cols) = values[i].as<double>();
  return result;
}

Eigen::VectorXd vector_from_yaml(const YAML::Node& node) {
  if (!node || !node.IsSequence()) return {};
  Eigen::VectorXd result(node.size());
  for (std::size_t i = 0; i < node.size(); ++i)
    result[static_cast<Eigen::Index>(i)] = node[i].as<double>();
  return result;
}

CameraCalibration camera_from_yaml(const YAML::Node& node,
                                   const std::string& name) {
  if (!node) throw std::runtime_error("missing " + name + " calibration");
  CameraCalibration camera;
  camera.name = name;
  camera.model = node["camera_model"].as<std::string>(
      node["model"].as<std::string>("pinhole"));

  auto resolution = node["resolution"].as<std::vector<int>>();
  if (resolution.size() != 2)
    throw std::runtime_error(name + " resolution must be [width, height]");
  camera.resolution = {resolution[0], resolution[1]};

  auto intrinsics = node["intrinsics"].as<std::vector<double>>();
  if (intrinsics.size() != 4)
    throw std::runtime_error(name + " intrinsics must be [fx, fy, cx, cy]");
  camera.intrinsics = Eigen::Map<Eigen::Vector4d>(intrinsics.data());
  camera.K << intrinsics[0], 0.0, intrinsics[2],
              0.0, intrinsics[1], intrinsics[3],
              0.0, 0.0, 1.0;
  camera.P << camera.K, Eigen::Vector3d::Zero();

  camera.distortion = vector_from_yaml(node["distortion_coefficients"]);
  if (node["rectification_matrix"])
    camera.R_rect = matrix_from_yaml<3, 3>(node["rectification_matrix"]);
  if (node["projection_matrix"])
    camera.P = matrix_from_yaml<3, 4>(node["projection_matrix"]);
  if (node["T_body_camera"])
    camera.T_body_camera.matrix() =
        matrix_from_yaml<4, 4>(node["T_body_camera"]);
  return camera;
}

} // namespace

cv::Mat CameraCalibration::camera_matrix_cv() const {
  cv::Mat result;
  cv::eigen2cv(K, result);
  return result;
}

cv::Mat CameraCalibration::distortion_cv() const {
  cv::Mat result(1, static_cast<int>(distortion.size()), CV_64F);
  for (Eigen::Index i = 0; i < distortion.size(); ++i)
    result.at<double>(0, static_cast<int>(i)) = distortion[i];
  return result;
}

DatasetConfig load_config(const YAML::Node& node,
                          const std::filesystem::path& base) {
  DatasetConfig config;
  config.type = node["dataset_type"].as<std::string>(
      node["type"].as<std::string>(""));
  config.root = node["dataset_dir"].as<std::string>(
      node["root"].as<std::string>(""));
  config.sequence = node["sequence_name"].as<std::string>(
      node["sequence"].as<std::string>(""));
  config.skip_frames = node["skip_frames"].as<std::size_t>(0);
  config.sync_tolerance_ns =
      node["sync_tolerance_ns"].as<std::int64_t>(20'000'000);

  if (config.type.empty() || config.root.empty())
    throw std::runtime_error("config requires dataset_type and dataset_dir");
  if (config.root.is_relative() && !base.empty()) config.root = base / config.root;
  config.root = std::filesystem::weakly_canonical(config.root);

  if (node["gt_file_path"]) {
    config.ground_truth_path = node["gt_file_path"].as<std::string>();
    if (config.ground_truth_path->is_relative())
      *config.ground_truth_path = config.root / *config.ground_truth_path;
  }

  if (node["left_camera"])
    config.cameras.push_back(camera_from_yaml(node["left_camera"], "cam0"));
  if (node["right_camera"])
    config.cameras.push_back(camera_from_yaml(node["right_camera"], "cam1"));

  if (node["stereo"]) {
    const auto stereo_node = node["stereo"];
    StereoCalibration stereo;
    stereo.R_target_reference =
        matrix_from_yaml<3, 3>(stereo_node["R21"]);
    const auto translation =
        stereo_node["T21"].as<std::vector<double>>();
    if (translation.size() != 3)
      throw std::runtime_error("stereo.T21 must have three values");
    stereo.t_target_reference = Eigen::Map<const Eigen::Vector3d>(
        translation.data());
    stereo.baseline = stereo.t_target_reference.norm();
    if (stereo_node["F21"]) {
      stereo.F_target_reference =
          matrix_from_yaml<3, 3>(stereo_node["F21"]);
    } else if (config.cameras.size() >= 2) {
      Eigen::Matrix3d tx;
      const auto& t = stereo.t_target_reference;
      tx << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
      stereo.F_target_reference =
          config.cameras[1].K.inverse().transpose() * tx *
          stereo.R_target_reference * config.cameras[0].K.inverse();
    }
    config.stereo = stereo;
  }

  if (node["frame_to_body"] && !config.cameras.empty()) {
    const auto body = node["frame_to_body"];
    config.cameras[0].T_body_camera.linear() =
        matrix_from_yaml<3, 3>(body["rotation"]);
    const auto translation = body["translation"].as<std::vector<double>>();
    if (translation.size() != 3)
      throw std::runtime_error("frame_to_body.translation needs 3 values");
    config.cameras[0].T_body_camera.translation() =
        Eigen::Map<const Eigen::Vector3d>(translation.data());
  }
  if (config.stereo && config.cameras.size() >= 2) {
    Eigen::Isometry3d target_from_reference = Eigen::Isometry3d::Identity();
    target_from_reference.linear() = config.stereo->R_target_reference;
    target_from_reference.translation() = config.stereo->t_target_reference;
    config.cameras[1].T_body_camera =
        config.cameras[0].T_body_camera * target_from_reference.inverse();
  }

  if (node["output_dir"])
    config.options["output_dir"] = node["output_dir"].as<std::string>();
  if (node["num_cameras"])
    config.options["num_cameras"] =
        std::to_string(node["num_cameras"].as<int>());
  return config;
}

DatasetConfig load_config(const std::filesystem::path& path) {
  return load_config(YAML::LoadFile(path.string()), path.parent_path());
}

} // namespace lems::data
