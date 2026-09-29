#include "lems/data/config.hpp"

#include <opencv2/core/eigen.hpp>

#include <initializer_list>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lems::data {
namespace {

bool present(const YAML::Node& node);

template <int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> matrix_from_yaml(const YAML::Node& node,
                                                   const std::string& label) {
  if (!node || node.IsNull())
    throw std::runtime_error("missing matrix " + label);

  YAML::Node values = node;
  if (node.IsMap() && present(node["rows"]) && present(node["cols"])) {
    if (node["rows"].as<int>() != Rows || node["cols"].as<int>() != Cols)
      throw std::runtime_error("matrix " + label + " has invalid rows/cols");
  }
  if (node.IsMap() && present(node["data"]))
    values = node["data"];

  Eigen::Matrix<double, Rows, Cols> result;
  if (values.IsSequence() && values.size() == static_cast<std::size_t>(Rows) &&
      values[0].IsSequence()) {
    for (int row = 0; row < Rows; ++row) {
      if (!values[row].IsSequence() ||
          values[row].size() != static_cast<std::size_t>(Cols)) {
        throw std::runtime_error("matrix " + label + " must be " +
                                 std::to_string(Rows) + "x" +
                                 std::to_string(Cols));
      }
      for (int col = 0; col < Cols; ++col)
        result(row, col) = values[row][col].as<double>();
    }
    if (!result.allFinite())
      throw std::runtime_error("matrix " + label + " contains non-finite values");
    return result;
  }

  if (!values.IsSequence() ||
      values.size() != static_cast<std::size_t>(Rows * Cols)) {
    throw std::runtime_error("matrix " + label + " must contain " +
                             std::to_string(Rows * Cols) + " values");
  }
  for (int i = 0; i < Rows * Cols; ++i)
    result(i / Cols, i % Cols) = values[i].as<double>();
  if (!result.allFinite())
    throw std::runtime_error("matrix " + label + " contains non-finite values");
  return result;
}

std::vector<double> vector_from_yaml(const YAML::Node& node,
                                     const std::string& label,
                                     bool required = false) {
  if (!node || node.IsNull()) {
    if (required) throw std::runtime_error("missing vector " + label);
    return {};
  }
  YAML::Node values = node;
  if (node.IsMap() && present(node["data"]))
    values = node["data"];
  if (!values.IsSequence())
    throw std::runtime_error("vector " + label + " must be a sequence");
  std::vector<double> result;
  result.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i)
    result.push_back(values[i].as<double>());
  for (const auto value : result)
    if (!std::isfinite(value))
      throw std::runtime_error("vector " + label + " contains non-finite values");
  return result;
}

std::string scalar_string(const YAML::Node& map,
                          std::initializer_list<const char*> keys,
                          const std::string& fallback = {}) {
  for (const char* key : keys)
    if (present(map[key])) return map[key].as<std::string>();
  return fallback;
}

bool present(const YAML::Node& node) { return node && !node.IsNull(); }

void validate_rotation(const Eigen::Matrix3d& rotation,
                       const std::string& label) {
  if (!rotation.allFinite() ||
      !(rotation.transpose() * rotation).isApprox(
          Eigen::Matrix3d::Identity(), 1e-5) ||
      std::abs(rotation.determinant() - 1.0) > 1e-5)
    throw std::runtime_error(label + " must be a finite rigid rotation");
}

void validate_camera_matrix(const Eigen::Matrix3d& matrix,
                            const std::string& label) {
  if (!matrix.allFinite())
    throw std::runtime_error(label + " contains non-finite values");
  if (!(matrix(0, 0) > 0.0) || !(matrix(1, 1) > 0.0))
    throw std::runtime_error(label + " must have positive focal lengths");
  const double determinant = matrix.determinant();
  if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-12)
    throw std::runtime_error(label + " must be invertible");
}

void validate_resolution(const std::vector<double>& resolution,
                         const std::string& label) {
  if (resolution.size() != 2)
    throw std::runtime_error(label + " must be [width, height]");
  for (const double value : resolution) {
    if (!std::isfinite(value) || value <= 0.0 ||
        value > static_cast<double>(std::numeric_limits<int>::max()) ||
        std::floor(value) != value)
      throw std::runtime_error(label + " must contain positive dimensions");
  }
}

std::filesystem::path resolve_path(const std::filesystem::path& value,
                                   const std::filesystem::path& base) {
  if (value.empty()) return value;
  if (value.is_relative() && !base.empty()) return base / value;
  return value;
}

Eigen::Isometry3d transform_from_yaml(const YAML::Node& node,
                                      const std::string& label) {
  if (!present(node)) return Eigen::Isometry3d::Identity();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  if (node.IsMap() && (present(node["data"]) || present(node["rows"]) ||
                       present(node["cols"]))) {
    result.matrix() = matrix_from_yaml<4, 4>(node, label);
    if (!result.matrix().row(3).transpose().isApprox(
            Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)))
      throw std::runtime_error(label + " must have homogeneous bottom row");
    validate_rotation(result.linear(), label + ".rotation");
    return result;
  }
  if (node.IsSequence()) {
    result.matrix() = matrix_from_yaml<4, 4>(node, label);
    if (!result.matrix().row(3).transpose().isApprox(
            Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)))
      throw std::runtime_error(label + " must have homogeneous bottom row");
    validate_rotation(result.linear(), label + ".rotation");
    return result;
  }
  if (node.IsMap() && (present(node["rotation"]) ||
                       present(node["translation"]))) {
    if (present(node["rotation"]))
      result.linear() =
          matrix_from_yaml<3, 3>(node["rotation"], label + ".rotation");
    const auto translation =
        vector_from_yaml(node["translation"], label + ".translation", true);
    if (translation.size() != 3)
      throw std::runtime_error(label + ".translation must contain 3 values");
    result.translation() =
        Eigen::Vector3d(translation[0], translation[1], translation[2]);
    validate_rotation(result.linear(), label + ".rotation");
    return result;
  }
  throw std::runtime_error("unsupported transform format for " + label);
}

CameraCalibration camera_from_yaml(const YAML::Node& node,
                                   const std::string& name) {
  if (!node || !node.IsMap())
    throw std::runtime_error("missing or invalid " + name + " calibration");

  CameraCalibration camera;
  camera.name = name;
  camera.model = scalar_string(node, {"camera_model", "model"}, "pinhole");

  if (present(node["resolution"])) {
    const auto resolution =
        vector_from_yaml(node["resolution"], name + ".resolution", true);
    validate_resolution(resolution, name + ".resolution");
    camera.resolution = {static_cast<int>(resolution[0]),
                         static_cast<int>(resolution[1])};
  } else {
    const bool has_width = present(node["image_width"]);
    const bool has_height = present(node["image_height"]);
    if (has_width != has_height)
      throw std::runtime_error(name +
                               " requires both image_width and image_height");
    if (has_width) {
      const int width = node["image_width"].as<int>();
      const int height = node["image_height"].as<int>();
      if (width <= 0 || height <= 0)
        throw std::runtime_error(name +
                                 " must have positive image dimensions");
      camera.resolution = {width, height};
    }
  }

  const bool has_projection_matrix = present(node["projection_matrix"]);
  const bool has_projection_alias =
      !has_projection_matrix && present(node["P"]);
  const bool has_projection = has_projection_matrix || has_projection_alias;
  const YAML::Node projection_node =
      has_projection_matrix ? node["projection_matrix"] : node["P"];
  if (has_projection)
    camera.P = matrix_from_yaml<3, 4>(projection_node, name + ".P");

  YAML::Node k_node;
  bool has_k = false;
  if (present(node["camera_matrix"])) {
    k_node = node["camera_matrix"];
    has_k = true;
  } else if (present(node["K"])) {
    k_node = node["K"];
    has_k = true;
  }
  bool has_intrinsics = false;
  bool has_intrinsics_vector = false;
  if (has_k) {
    camera.K = matrix_from_yaml<3, 3>(k_node, name + ".K");
    camera.intrinsics << camera.K(0, 0), camera.K(1, 1), camera.K(0, 2),
        camera.K(1, 2);
    has_intrinsics = true;
  } else if (present(node["intrinsics"])) {
    const auto intrinsics =
        vector_from_yaml(node["intrinsics"], name + ".intrinsics", true);
    if (intrinsics.size() != 4)
      throw std::runtime_error(name +
                               ".intrinsics must be [fx, fy, cx, cy]");
    camera.intrinsics = Eigen::Vector4d(intrinsics[0], intrinsics[1],
                                        intrinsics[2], intrinsics[3]);
    has_intrinsics = true;
    has_intrinsics_vector = true;
  } else if (has_projection) {
    camera.K = camera.P.leftCols<3>();
    camera.intrinsics << camera.K(0, 0), camera.K(1, 1), camera.K(0, 2),
        camera.K(1, 2);
    has_intrinsics = true;
  }
  if (has_intrinsics_vector && !has_k) {
    camera.K << camera.intrinsics[0], 0.0, camera.intrinsics[2], 0.0,
               camera.intrinsics[1], camera.intrinsics[3], 0.0, 0.0, 1.0;
    if (!has_projection) camera.P << camera.K, Eigen::Vector3d::Zero();
  } else if (has_k && !has_projection) {
    camera.P << camera.K, Eigen::Vector3d::Zero();
  }
  if (!has_intrinsics && !has_k && !has_projection)
    throw std::runtime_error(name + " calibration requires K, P, or intrinsics");
  validate_camera_matrix(camera.K, name + ".K");

  const YAML::Node distortion_node =
      present(node["distortion_coefficients"])
          ? node["distortion_coefficients"]
          : (present(node["distortion"]) ? node["distortion"] : node["D"]);
  const auto distortion = vector_from_yaml(distortion_node, name + ".D");
  camera.distortion.resize(static_cast<Eigen::Index>(distortion.size()));
  for (Eigen::Index i = 0; i < camera.distortion.size(); ++i)
    camera.distortion[i] = distortion[static_cast<std::size_t>(i)];

  const YAML::Node rectification_node = present(node["rectification_matrix"])
                                             ? node["rectification_matrix"]
                                             : node["R_rect"];
  if (present(rectification_node))
    camera.R_rect =
        matrix_from_yaml<3, 3>(rectification_node, name + ".R_rect");

  const YAML::Node transform_node = present(node["T_body_camera"])
                                        ? node["T_body_camera"]
                                        : node["T_BS"];
  if (present(transform_node))
    camera.T_body_camera =
        transform_from_yaml(transform_node, name + ".T_body_camera");
  return camera;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& t) {
  Eigen::Matrix3d result;
  result << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  return result;
}

void fill_stereo(DatasetConfig& config, const YAML::Node& stereo_node) {
  if (!present(stereo_node)) return;
  StereoCalibration stereo;
  stereo.reference_camera = scalar_string(
      stereo_node, {"reference_camera", "reference"}, "cam0");
  stereo.target_camera =
      scalar_string(stereo_node, {"target_camera", "target"}, "cam1");
  const bool has_r21 = present(stereo_node["R21"]);
  const bool has_rotation = !has_r21 && present(stereo_node["rotation"]);
  const bool has_r = has_r21 || has_rotation || present(stereo_node["R"]);
  const YAML::Node r_node =
      has_r21 ? stereo_node["R21"]
              : (has_rotation ? stereo_node["rotation"] : stereo_node["R"]);
  if (has_r)
    stereo.R_target_reference =
        matrix_from_yaml<3, 3>(r_node, "stereo.R21");
  validate_rotation(stereo.R_target_reference, "stereo.R21");
  const bool has_t21 = present(stereo_node["T21"]);
  const bool has_translation =
      !has_t21 && present(stereo_node["translation"]);
  const bool has_t = has_t21 || has_translation || present(stereo_node["T"]);
  const YAML::Node t_node =
      has_t21 ? stereo_node["T21"]
              : (has_translation ? stereo_node["translation"] : stereo_node["T"]);
  if (has_t) {
    const auto values = vector_from_yaml(t_node, "stereo.T21", true);
    if (values.size() != 3)
      throw std::runtime_error("stereo.T21 must contain 3 values");
    stereo.t_target_reference =
        Eigen::Vector3d(values[0], values[1], values[2]);
  }
  if (present(stereo_node["F21"]))
    stereo.F_target_reference =
        matrix_from_yaml<3, 3>(stereo_node["F21"], "stereo.F21");
  stereo.baseline = stereo.t_target_reference.norm();
  if (stereo.F_target_reference.isZero(0.0) && config.cameras.size() >= 2) {
    stereo.F_target_reference =
        config.cameras[1].K.inverse().transpose() *
        skew(stereo.t_target_reference) * stereo.R_target_reference *
        config.cameras[0].K.inverse();
  }
  if (!std::isfinite(stereo.baseline))
    throw std::runtime_error("stereo.T21 must be finite");
  config.stereo = stereo;
}

}  // namespace

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
  if (!node || !node.IsMap())
    throw std::runtime_error("dataset configuration must be a YAML mapping");

  DatasetConfig config;
  config.type = scalar_string(node, {"dataset_type", "type"});
  config.root = resolve_path(
      scalar_string(node, {"dataset_dir", "root"}), base);
  config.sequence = scalar_string(node, {"sequence_name", "sequence"});
  const auto skip = node["skip_frames"].as<long long>(0);
  if (skip < 0) throw std::runtime_error("skip_frames cannot be negative");
  config.skip_frames = static_cast<std::size_t>(skip);
  config.sync_tolerance_ns = present(node["sync_tolerance_ns"])
                                 ? node["sync_tolerance_ns"].as<std::int64_t>()
                                 : 20'000'000;
  if (config.type.empty() || config.root.empty())
    throw std::runtime_error(
        "config requires dataset_type and dataset_dir/root");
  if (config.sync_tolerance_ns < 0)
    throw std::runtime_error("sync_tolerance_ns cannot be negative");

  if (present(node["gt_file_path"]) || present(node["ground_truth"]) ||
      present(node["ground_truth_path"])) {
    const auto gt =
        present(node["gt_file_path"])
            ? node["gt_file_path"].as<std::string>()
            : (present(node["ground_truth_path"])
                   ? node["ground_truth_path"].as<std::string>()
                   : node["ground_truth"].as<std::string>());
    config.ground_truth_path = resolve_path(gt, config.root);
  }

  if (present(node["left_camera"]))
    config.cameras.push_back(camera_from_yaml(node["left_camera"], "cam0"));
  if (present(node["right_camera"]))
    config.cameras.push_back(camera_from_yaml(node["right_camera"], "cam1"));
  if (present(node["cameras"]) && node["cameras"].IsSequence()) {
    config.cameras.clear();
    for (std::size_t i = 0; i < node["cameras"].size(); ++i)
      config.cameras.push_back(camera_from_yaml(
          node["cameras"][i], "cam" + std::to_string(i)));
  }

  if (present(node["stereo"]))
    fill_stereo(config, node["stereo"]);
  else if (present(node["R21"]) || present(node["T21"]) ||
           present(node["F21"]))
    fill_stereo(config, node);

  const YAML::Node frame_to_body = present(node["frame_to_body"])
                                       ? node["frame_to_body"]
                                       : node["T_frame_body"];
  if (present(frame_to_body) && !config.cameras.empty())
    config.cameras[0].T_body_camera =
        transform_from_yaml(frame_to_body, "frame_to_body");

  const bool right_has_transform =
      present(node["right_camera"]) &&
      (present(node["right_camera"]["T_body_camera"]) ||
       present(node["right_camera"]["T_BS"]));
  if (config.stereo && config.cameras.size() >= 2 && !right_has_transform) {
    Eigen::Isometry3d target_from_reference = Eigen::Isometry3d::Identity();
    target_from_reference.linear() = config.stereo->R_target_reference;
    target_from_reference.translation() = config.stereo->t_target_reference;
    config.cameras[1].T_body_camera =
        config.cameras[0].T_body_camera * target_from_reference.inverse();
  }

  for (const auto& item : node) {
    const auto key = item.first.as<std::string>();
    if (!item.second.IsScalar()) continue;
    if (key == "output_dir") {
      config.options[key] = resolve_path(item.second.as<std::string>(), base)
                                .string();
    } else if (key != "dataset_type" && key != "type" &&
               key != "dataset_dir" && key != "root" &&
               key != "sequence_name" && key != "sequence" &&
               key != "skip_frames" && key != "sync_tolerance_ns") {
      config.options[key] = item.second.as<std::string>();
    }
  }
  if (present(node["num_cameras"]))
    config.options["num_cameras"] =
        std::to_string(node["num_cameras"].as<int>());
  return config;
}

DatasetConfig load_config(const std::filesystem::path& path) {
  try {
    return load_config(YAML::LoadFile(path.string()), path.parent_path());
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("failed to parse dataset config " +
                             path.string() + ": " + error.what());
  }
}

}  // namespace lems::data
