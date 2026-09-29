#include "lems/data/dataset.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>

namespace fs = std::filesystem;
namespace lems::data {
namespace {

Timestamp seconds_to_ns(double seconds) {
  if (!std::isfinite(seconds))
    throw std::runtime_error("non-finite dataset timestamp");
  return static_cast<Timestamp>(std::llround(seconds * 1e9));
}

double ns_to_seconds(Timestamp timestamp) {
  return static_cast<double>(timestamp) / 1e9;
}

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool is_image(const fs::path& path) {
  const auto extension = lower(path.extension().string());
  return extension == ".png" || extension == ".jpg" ||
         extension == ".jpeg" || extension == ".pgm" ||
         extension == ".tif" || extension == ".tiff";
}

std::vector<fs::path> images_in(const fs::path& directory) {
  if (!fs::exists(directory))
    throw std::runtime_error("missing image directory: " + directory.string());
  std::vector<fs::path> result;
  for (const auto& entry : fs::directory_iterator(directory))
    if (entry.is_regular_file() && is_image(entry.path())) result.push_back(entry.path());
  std::sort(result.begin(), result.end());
  return result;
}

struct Sample {
  Timestamp timestamp{};
  fs::path path;
  std::string name;
};

std::vector<Sample> image_csv(const fs::path& csv, const fs::path& data_directory,
                              const std::string& label,
                              bool timestamps_are_nanoseconds = false) {
  std::ifstream input(csv);
  if (!input)
    throw std::runtime_error("missing " + label + " index: " + csv.string());
  std::vector<Sample> result;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    std::string timestamp_text;
    std::string filename;
    if (!(values >> timestamp_text >> filename))
      throw std::runtime_error("malformed " + label + " index at " +
                               csv.string() + ":" + std::to_string(line_number));
    Timestamp timestamp = 0;
    try {
      if (timestamps_are_nanoseconds) {
        std::size_t parsed = 0;
        timestamp = static_cast<Timestamp>(std::stoll(timestamp_text, &parsed));
        if (parsed != timestamp_text.size()) throw std::invalid_argument("trailing");
      } else {
        timestamp = seconds_to_ns(std::stod(timestamp_text));
      }
    } catch (const std::exception&) {
      throw std::runtime_error("invalid timestamp in " + label + " index at " +
                               csv.string() + ":" + std::to_string(line_number));
    }
    fs::path path(filename);
    if (path.is_relative()) path = data_directory / path;
    if (!fs::is_regular_file(path))
      throw std::runtime_error("missing " + label + " image: " + path.string());
    result.push_back({timestamp, path, path.filename().string()});
  }
  return result;
}

std::vector<Sample> timed_paths(const fs::path& list, const fs::path& root,
                                const std::string& label) {
  std::ifstream input(list);
  if (!input)
    throw std::runtime_error("missing " + label + " index: " + list.string());
  std::vector<Sample> result;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::istringstream values(line);
    long double timestamp = 0.0;
    std::string filename;
    if (!(values >> timestamp >> filename))
      throw std::runtime_error("malformed " + label + " index at " +
                               list.string() + ":" + std::to_string(line_number));
    fs::path path(filename);
    if (path.is_relative()) path = root / path;
    if (!fs::is_regular_file(path))
      throw std::runtime_error("missing " + label + " image: " + path.string());
    result.push_back({seconds_to_ns(static_cast<double>(timestamp)), path,
                      path.filename().string()});
  }
  return result;
}

std::vector<Timestamp> scalar_timestamps(const fs::path& list,
                                          const std::string& label) {
  std::ifstream input(list);
  if (!input)
    throw std::runtime_error("missing " + label + " index: " + list.string());
  std::vector<Timestamp> result;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::istringstream values(line);
    std::string token;
    if (!(values >> token)) continue;
    std::string extra;
    if (values >> extra)
      throw std::runtime_error("malformed " + label + " at " + list.string() +
                               ":" + std::to_string(line_number));
    try {
      result.push_back(seconds_to_ns(std::stod(token)));
    } catch (const std::exception&) {
      throw std::runtime_error("invalid timestamp in " + label + " at " +
                               list.string() + ":" + std::to_string(line_number));
    }
  }
  return result;
}

std::vector<Sample> exact_pair(const std::vector<fs::path>& left,
                               const std::vector<fs::path>& right,
                               const std::string& label) {
  std::map<std::string, fs::path> right_by_name;
  for (const auto& path : right) right_by_name.emplace(path.filename().string(), path);
  std::vector<Sample> result;
  result.reserve(left.size());
  for (std::size_t i = 0; i < left.size(); ++i) {
    const auto name = left[i].filename().string();
    const auto found = right_by_name.find(name);
    if (found == right_by_name.end())
      throw std::runtime_error(label + " image basename mismatch at " + name);
    result.push_back({seconds_to_ns(static_cast<double>(i)), left[i], name});
  }
  if (right_by_name.size() != left.size())
    throw std::runtime_error(label + " image streams have different file counts");
  return result;
}

std::vector<double> line_values(const std::string& line) {
  const auto colon = line.find(':');
  std::istringstream input(colon == std::string::npos ? line
                                                     : line.substr(colon + 1));
  std::vector<double> values;
  double value = 0.0;
  while (input >> value) values.push_back(value);
  return values;
}

bool host_little_endian() {
  const std::uint16_t value = 1;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 1;
}

std::string pfm_token(std::istream& input) {
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

cv::Mat read_pfm(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("failed to open PFM: " + path.string());
  const auto type = pfm_token(input);
  const auto width_text = pfm_token(input);
  const auto height_text = pfm_token(input);
  const auto scale_text = pfm_token(input);
  if ((type != "Pf" && type != "PF") || width_text.empty() ||
      height_text.empty() || scale_text.empty())
    throw std::runtime_error("invalid PFM header: " + path.string());
  int width = 0;
  int height = 0;
  float scale = 0.0F;
  try {
    width = std::stoi(width_text);
    height = std::stoi(height_text);
    scale = std::stof(scale_text);
  } catch (const std::exception&) {
    throw std::runtime_error("invalid PFM dimensions/scale: " + path.string());
  }
  if (width <= 0 || height <= 0 || scale == 0.0F || !std::isfinite(scale))
    throw std::runtime_error("invalid PFM dimensions/scale: " + path.string());
  const int channels = type == "PF" ? 3 : 1;
  cv::Mat result(height, width, channels == 1 ? CV_32FC1 : CV_32FC3);
  const bool file_little = scale < 0.0F;
  const bool swap_bytes = file_little != host_little_endian();
  const float multiplier = std::abs(scale);
  std::array<std::uint8_t, sizeof(float)> bytes{};
  for (int source_row = 0; source_row < height; ++source_row) {
    const int destination_row = height - source_row - 1;  // PFM is bottom-up.
    for (int x = 0; x < width; ++x) {
      for (int channel = 0; channel < channels; ++channel) {
        input.read(reinterpret_cast<char*>(bytes.data()), sizeof(float));
        if (!input)
          throw std::runtime_error("truncated PFM payload: " + path.string());
        if (swap_bytes) std::reverse(bytes.begin(), bytes.end());
        float value = 0.0F;
        std::memcpy(&value, bytes.data(), sizeof(value));
        value *= multiplier;
        if (channels == 1)
          result.at<float>(destination_row, x) = value;
        else
          result.at<cv::Vec3f>(destination_row, x)[channel] = value;
      }
    }
  }
  return result;
}

cv::Mat read_metadata(const fs::path& path, bool disparity) {
  if (lower(path.extension().string()) == ".pfm") return read_pfm(path);
  const int flags = disparity ? cv::IMREAD_UNCHANGED : cv::IMREAD_GRAYSCALE;
  cv::Mat result = cv::imread(path.string(), flags);
  if (result.empty())
    throw std::runtime_error("failed to read metadata: " + path.string());
  return result;
}

template <typename T, typename = void>
struct has_image_gradients : std::false_type {};
template <typename T>
struct has_image_gradients<T, std::void_t<decltype(std::declval<T&>().image_gradients_x),
                                          decltype(std::declval<T&>().image_gradients_y)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_edges : std::false_type {};
template <typename T>
struct has_edges<T, std::void_t<decltype(std::declval<T&>().edges)>> : std::true_type {};

template <typename T, typename = void>
struct has_disparity_map : std::false_type {};
template <typename T>
struct has_disparity_map<T, std::void_t<decltype(std::declval<T&>().disparity_map)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_occlusion_mask : std::false_type {};
template <typename T>
struct has_occlusion_mask<T, std::void_t<decltype(std::declval<T&>().occlusion_mask)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_gt_camera_pose : std::false_type {};
template <typename T>
struct has_gt_camera_pose<T, std::void_t<decltype(std::declval<T&>().gt_camera_pose)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_has_ground_truth : std::false_type {};
template <typename T>
struct has_has_ground_truth<T, std::void_t<decltype(std::declval<T&>().has_ground_truth)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_timestamp : std::false_type {};
template <typename T>
struct has_timestamp<T, std::void_t<decltype(std::declval<T&>().timestamp)>>
    : std::true_type {};

void set_ground_truth(Frame& frame, const CameraPose& pose) {
  frame.ground_truth = pose;
  if constexpr (has_gt_camera_pose<Frame>::value) frame.gt_camera_pose = pose;
  if constexpr (has_has_ground_truth<Frame>::value) frame.has_ground_truth = true;
}

void materialize(Frame& frame) {
  frame.image = cv::imread(frame.image_path.string(), cv::IMREAD_GRAYSCALE);
  if (frame.image.empty())
    throw std::runtime_error("failed to read image: " + frame.image_path.string());
  if (frame.metadata.disparity_path) {
    frame.metadata.disparity = read_metadata(*frame.metadata.disparity_path, true);
    if (frame.metadata.disparity.rows != frame.image.rows ||
        frame.metadata.disparity.cols != frame.image.cols)
      throw std::runtime_error("disparity dimensions do not match image: " +
                               frame.metadata.disparity_path->string());
    if (frame.metadata.disparity.channels() != 1)
      throw std::runtime_error("disparity must be single-channel: " +
                               frame.metadata.disparity_path->string());
    if constexpr (has_disparity_map<Frame>::value)
      frame.disparity_map = frame.metadata.disparity;
  }
  if (frame.metadata.occlusion_mask_path) {
    frame.metadata.occlusion_mask =
        read_metadata(*frame.metadata.occlusion_mask_path, false);
    if (frame.metadata.occlusion_mask.rows != frame.image.rows ||
        frame.metadata.occlusion_mask.cols != frame.image.cols)
      throw std::runtime_error("occlusion mask dimensions do not match image: " +
                               frame.metadata.occlusion_mask_path->string());
    if constexpr (has_occlusion_mask<Frame>::value)
      frame.occlusion_mask = frame.metadata.occlusion_mask;
  }
  if (frame.metadata.depth_path)
    frame.metadata.depth = read_metadata(*frame.metadata.depth_path, true);
  if constexpr (has_image_gradients<Frame>::value) {
    // Gradients are deliberately left empty: the edge pipeline chooses its
    // precision/kernel and computes them with its utility layer.
    frame.image_gradients_x.release();
    frame.image_gradients_y.release();
  }
  frame.synchronize_legacy_fields();
}

Frame make_frame(const std::string& camera, Timestamp timestamp, const fs::path& image,
                 const CameraCalibration* calibration) {
  Frame frame;
  frame.camera = camera;
  frame.timestamp_ns = timestamp;
  frame.timestamp_seconds = ns_to_seconds(timestamp);
  if constexpr (has_timestamp<Frame>::value) frame.timestamp = frame.timestamp_seconds;
  frame.image_path = image;
  if (calibration) frame.K = calibration->K;
  return frame;
}

Eigen::Isometry3d pose_transform(const CameraPose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.R;
  result.translation() = pose.t;
  return result;
}

CameraPose make_pose(Timestamp timestamp, const Eigen::Isometry3d& transform) {
  return CameraPose(timestamp, transform.rotation(), transform.translation());
}

std::optional<CameraPose> nearest_pose(const std::vector<CameraPose>& poses,
                                       Timestamp timestamp, Timestamp tolerance) {
  if (poses.empty()) return std::nullopt;
  auto it = std::lower_bound(
      poses.begin(), poses.end(), timestamp,
      [](const CameraPose& pose, Timestamp value) { return pose.timestamp_ns < value; });
  if (it == poses.end()) --it;
  else if (it != poses.begin() &&
           std::llabs((it - 1)->timestamp_ns - timestamp) <=
               std::llabs(it->timestamp_ns - timestamp))
    --it;
  if (std::llabs(it->timestamp_ns - timestamp) > tolerance) return std::nullopt;
  return *it;
}

std::optional<std::size_t> nearest_unused_pose(
    const std::vector<CameraPose>& poses, Timestamp timestamp,
    Timestamp tolerance, std::vector<bool>& used) {
  std::size_t best = poses.size();
  Timestamp best_delta = std::numeric_limits<Timestamp>::max();
  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (used[i]) continue;
    const Timestamp delta = std::llabs(poses[i].timestamp_ns - timestamp);
    if (delta < best_delta) {
      best = i;
      best_delta = delta;
    }
  }
  if (best == poses.size() || best_delta > tolerance) return std::nullopt;
  used[best] = true;
  return best;
}

std::vector<Eigen::Matrix<double, 3, 4>> read_kitti_poses(const fs::path& path) {
  std::ifstream input(path);
  if (!input) return {};
  std::vector<Eigen::Matrix<double, 3, 4>> result;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::istringstream values(line);
    Eigen::Matrix<double, 3, 4> pose;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 4; ++col)
        if (!(values >> pose(row, col)))
          throw std::runtime_error("malformed KITTI pose at " + path.string() +
                                   ":" + std::to_string(line_number));
    result.push_back(pose);
  }
  return result;
}

std::vector<CameraPose> read_euroc_ground_truth(
    const fs::path& path, const CameraCalibration* camera,
    std::vector<Timestamp>& timestamps) {
  std::ifstream input(path);
  if (!input) return {};
  std::vector<CameraPose> result;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    Timestamp timestamp = 0;
    Eigen::Vector3d position;
    double qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;
    if (!(values >> timestamp >> position.x() >> position.y() >> position.z() >>
          qw >> qx >> qy >> qz))
      throw std::runtime_error("malformed EuRoC ground truth at " + path.string() +
                               ":" + std::to_string(line_number));
    Eigen::Isometry3d world_from_body = Eigen::Isometry3d::Identity();
    world_from_body.linear() =
        Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    world_from_body.translation() = position;
    // EuRoC stores T_world_body (body-to-world).  The public pose contract
    // is world-to-camera, so compose T_cam_body * T_body_world explicitly.
    const Eigen::Isometry3d body_from_world = world_from_body.inverse();
    const Eigen::Isometry3d camera_from_body =
        camera ? camera->T_body_camera.inverse()
               : Eigen::Isometry3d::Identity();
    result.push_back(make_pose(timestamp, camera_from_body * body_from_world));
    timestamps.push_back(timestamp);
  }
  return result;
}

std::vector<CameraPose> read_eth3d_poses(const fs::path& path,
                                         const std::string& image_name,
                                         Timestamp timestamp) {
  std::ifstream input(path);
  if (!input) return {};
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream values(line);
    int image_id = 0;
    int camera_id = 0;
    double qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;
    double tx = 0.0, ty = 0.0, tz = 0.0;
    std::string filename;
    if (!(values >> image_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >>
          camera_id >> filename))
      continue;
    if (filename != image_name) continue;
    const Eigen::Quaterniond quaternion(qw, qx, qy, qz);
    return {CameraPose(timestamp, quaternion, Eigen::Vector3d(tx, ty, tz))};
  }
  return {};
}

std::vector<CameraPose> read_tum_ground_truth(const fs::path& path) {
  std::ifstream input(path);
  if (!input) return {};
  std::vector<CameraPose> result;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream values(line);
    double timestamp = 0.0, tx = 0.0, ty = 0.0, tz = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
    if (!(values >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
      throw std::runtime_error("malformed TUM ground truth: " + path.string());
    Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
    world_from_camera.linear() =
        Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    world_from_camera.translation() = Eigen::Vector3d(tx, ty, tz);
    result.push_back(make_pose(seconds_to_ns(timestamp),
                               world_from_camera.inverse()));
  }
  std::sort(result.begin(), result.end(),
            [](const CameraPose& a, const CameraPose& b) {
              return a.timestamp_ns < b.timestamp_ns;
            });
  return result;
}

std::vector<std::pair<std::size_t, std::size_t>> match_samples(
    const std::vector<Sample>& left, const std::vector<Sample>& right,
    Timestamp tolerance) {
  std::vector<std::pair<std::size_t, std::size_t>> result;
  std::vector<bool> used(right.size(), false);
  for (std::size_t i = 0; i < left.size(); ++i) {
    std::size_t best = right.size();
    Timestamp best_delta = std::numeric_limits<Timestamp>::max();
    for (std::size_t j = 0; j < right.size(); ++j) {
      if (used[j]) continue;
      const auto delta = std::llabs(right[j].timestamp - left[i].timestamp);
      if (delta < best_delta) {
        best_delta = delta;
        best = j;
      }
    }
    if (best == right.size() || best_delta > tolerance) continue;
    used[best] = true;
    result.emplace_back(i, best);
  }
  return result;
}

void set_resolution_from_image(CameraCalibration& camera, const fs::path& path) {
  if (camera.resolution.width > 0 && camera.resolution.height > 0) return;
  const cv::Mat image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
  if (image.empty())
    throw std::runtime_error("failed to read image for calibration dimensions: " +
                             path.string());
  camera.resolution = {image.cols, image.rows};
}

void build_camera_info(const std::vector<CameraCalibration>& cameras,
                       const std::optional<StereoCalibration>& stereo,
                       CameraInfo& info) {
  auto copy_camera = [](const CameraCalibration& source, Camera& target) {
    target.resolution = {source.resolution.width, source.resolution.height};
    target.intrinsics = {source.intrinsics[0], source.intrinsics[1],
                         source.intrinsics[2], source.intrinsics[3]};
    target.distortion.resize(static_cast<std::size_t>(source.distortion.size()));
    for (Eigen::Index i = 0; i < source.distortion.size(); ++i)
      target.distortion[static_cast<std::size_t>(i)] = source.distortion[i];
    target.K = source.K;
  };
  if (!cameras.empty()) copy_camera(cameras[0], info.left);
  if (cameras.size() > 1) copy_camera(cameras[1], info.right);
  if (!cameras.empty()) {
    info.rot_frame2body_left = cameras[0].T_body_camera.rotation();
    info.transl_frame2body_left = cameras[0].T_body_camera.translation();
  }
  if (stereo && cameras.size() >= 2) {
    info.left.R = stereo->R_target_reference;
    info.left.T = stereo->t_target_reference;
    info.left.F = stereo->F_target_reference;
    info.right.R = stereo->R_target_reference.transpose();
    info.right.T = -info.right.R * stereo->t_target_reference;
    info.right.F = stereo->F_target_reference.transpose();
  }
}

void load_kitti_calibration(const fs::path& path, DatasetConfig& config,
                            Eigen::Vector3d* left_reference_offset = nullptr) {
  std::ifstream input(path);
  if (!input) return;
  std::map<int, Eigen::Matrix<double, 3, 4>> projections;
  Eigen::Matrix3d rectification = Eigen::Matrix3d::Identity();
  std::string line;
  while (std::getline(input, line)) {
    if (line.rfind("P", 0) == 0 && line.size() > 2 && line[1] >= '0' &&
        line[1] <= '3' && line[2] == ':') {
      const auto values = line_values(line);
      if (values.size() != 12)
        throw std::runtime_error("invalid KITTI projection in " + path.string());
      Eigen::Matrix<double, 3, 4> projection;
      for (int i = 0; i < 12; ++i) projection(i / 4, i % 4) = values[i];
      projections[line[1] - '0'] = projection;
    } else if (line.rfind("R0_rect:", 0) == 0 ||
               line.rfind("R_rect_00:", 0) == 0) {
      const auto values = line_values(line);
      if (values.size() == 9)
        for (int i = 0; i < 9; ++i) rectification(i / 3, i % 3) = values[i];
    }
  }
  std::string pair = "gray";
  auto pair_it = config.options.find("kitti_camera_pair");
  if (pair_it != config.options.end()) pair = lower(pair_it->second);
  int left_index = pair == "color" ? 2 : 0;
  int right_index = pair == "color" ? 3 : 1;
  if (config.options.count("kitti_left_camera"))
    left_index = std::stoi(config.options.at("kitti_left_camera"));
  if (config.options.count("kitti_right_camera"))
    right_index = std::stoi(config.options.at("kitti_right_camera"));
  if (!projections.count(left_index) || !projections.count(right_index)) return;

  CameraCalibration left = config.cameras.size() > 0 ? config.cameras[0]
                                                       : CameraCalibration{};
  CameraCalibration right = config.cameras.size() > 1 ? config.cameras[1]
                                                        : CameraCalibration{};
  left.name = "cam0";
  right.name = "cam1";
  left.P = projections[left_index];
  right.P = projections[right_index];
  left.K = left.P.leftCols<3>();
  right.K = right.P.leftCols<3>();
  left.intrinsics << left.K(0, 0), left.K(1, 1), left.K(0, 2), left.K(1, 2);
  right.intrinsics << right.K(0, 0), right.K(1, 1), right.K(0, 2), right.K(1, 2);
  left.R_rect = right.R_rect = rectification;
  if (left_reference_offset) {
    if (projections.count(0)) {
      const Eigen::Matrix3d k0 = projections.at(0).leftCols<3>();
      const Eigen::Vector3d offset0 =
          k0.inverse() * projections.at(0).col(3);
      const Eigen::Vector3d offset_left =
          left.K.inverse() * left.P.col(3);
      *left_reference_offset = offset_left - offset0;
    } else {
      *left_reference_offset = Eigen::Vector3d::Zero();
    }
  }
  config.cameras = {left, right};

  StereoCalibration stereo;
  stereo.R_target_reference = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d left_offset = left.K.inverse() * left.P.col(3);
  const Eigen::Vector3d right_offset = right.K.inverse() * right.P.col(3);
  stereo.t_target_reference = right_offset - left_offset;
  stereo.baseline = stereo.t_target_reference.norm();
  Eigen::Matrix3d tx;
  const auto& t = stereo.t_target_reference;
  tx << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  stereo.F_target_reference = right.K.inverse().transpose() * tx *
                             left.K.inverse();
  config.stereo = stereo;
}

CameraCalibration read_euroc_camera(const fs::path& path,
                                    const std::string& name) {
  const YAML::Node node = YAML::LoadFile(path.string());
  CameraCalibration camera;
  camera.name = name;
  camera.model = node["camera_model"].as<std::string>("pinhole");
  const auto resolution = node["resolution"].as<std::vector<int>>();
  const auto intrinsics = node["intrinsics"].as<std::vector<double>>();
  if (resolution.size() != 2 || intrinsics.size() != 4)
    throw std::runtime_error("invalid EuRoC calibration: " + path.string());
  camera.resolution = {resolution[0], resolution[1]};
  camera.intrinsics = Eigen::Vector4d(intrinsics[0], intrinsics[1],
                                      intrinsics[2], intrinsics[3]);
  camera.K << intrinsics[0], 0.0, intrinsics[2], 0.0, intrinsics[1],
      intrinsics[3], 0.0, 0.0, 1.0;
  camera.P << camera.K, Eigen::Vector3d::Zero();
  const auto distortion = node["distortion_coefficients"]
                              ? node["distortion_coefficients"].as<std::vector<double>>()
                              : std::vector<double>{};
  camera.distortion.resize(static_cast<Eigen::Index>(distortion.size()));
  for (Eigen::Index i = 0; i < camera.distortion.size(); ++i)
    camera.distortion[i] = distortion[static_cast<std::size_t>(i)];
  if (node["T_BS"] && node["T_BS"]["data"]) {
    const auto values = node["T_BS"]["data"].as<std::vector<double>>();
    if (values.size() != 16)
      throw std::runtime_error("EuRoC T_BS must contain 16 values: " + path.string());
    Eigen::Matrix4d matrix;
    for (int i = 0; i < 16; ++i) matrix(i / 4, i % 4) = values[i];
    camera.T_body_camera.matrix() = matrix;
  }
  return camera;
}

std::vector<double> read_eth3d_scalars(const fs::path& path) {
  std::ifstream input(path);
  if (!input) return {};
  std::vector<double> result;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    const char* cursor = line.c_str();
    while (*cursor != '\0' && *cursor != '#') {
      char* end = nullptr;
      const double value = std::strtod(cursor, &end);
      if (end == cursor) {
        ++cursor;
      } else {
        result.push_back(value);
        cursor = end;
      }
    }
  }
  return result;
}

CameraCalibration eth3d_camera_from_file(const fs::path& path,
                                         const std::string& name) {
  const auto values = read_eth3d_scalars(path);
  if (values.size() < 4)
    throw std::runtime_error("ETH3D calibration must contain fx fy cx cy: " +
                             path.string());
  CameraCalibration camera;
  camera.name = name;
  camera.intrinsics = Eigen::Vector4d(values[0], values[1], values[2], values[3]);
  camera.K << values[0], 0.0, values[2], 0.0, values[1], values[3], 0.0, 0.0, 1.0;
  camera.P << camera.K, Eigen::Vector3d::Zero();
  return camera;
}

CameraCalibration eth3d_camera_from_values(const std::vector<double>& values,
                                           std::size_t offset,
                                           const std::string& name) {
  if (offset + 4 > values.size())
    throw std::runtime_error("ETH3D calibration must contain fx fy cx cy");
  CameraCalibration camera;
  camera.name = name;
  camera.intrinsics = Eigen::Vector4d(values[offset], values[offset + 1],
                                      values[offset + 2], values[offset + 3]);
  camera.K << values[offset], 0.0, values[offset + 2], 0.0,
      values[offset + 1], values[offset + 3], 0.0, 0.0, 1.0;
  camera.P << camera.K, Eigen::Vector3d::Zero();
  return camera;
}

std::vector<double> eth3d_numbers(const std::string& text) {
  std::vector<double> values;
  const char* cursor = text.c_str();
  while (*cursor != '\0' && *cursor != '#') {
    char* end = nullptr;
    const double value = std::strtod(cursor, &end);
    if (end == cursor) {
      ++cursor;
    } else {
      values.push_back(value);
      cursor = end;
    }
  }
  return values;
}

std::vector<double> eth3d_values_after(const std::string& line,
                                       const std::string& key) {
  const auto lowered = lower(line);
  const auto key_position = lowered.find(key);
  if (key_position == std::string::npos) return {};
  const auto equal_position = line.find('=', key_position + key.size());
  const auto value_position = equal_position == std::string::npos
                                  ? key_position + key.size()
                                  : equal_position + 1;
  return eth3d_numbers(line.substr(value_position));
}

struct Eth3dPairCalibration {
  CameraCalibration left;
  CameraCalibration right;
  StereoCalibration stereo;
};

CameraCalibration eth3d_camera_from_matrix(const std::vector<double>& values,
                                           const std::string& name, int width,
                                           int height) {
  if (values.size() < 9)
    throw std::runtime_error("ETH3D pair calibration " + name +
                             " matrix must contain 9 values");
  CameraCalibration camera;
  camera.name = name;
  camera.K << values[0], values[1], values[2], values[3], values[4], values[5],
      values[6], values[7], values[8];
  if (!camera.K.allFinite() || std::abs(camera.K.determinant()) < 1e-12 ||
      camera.K(0, 0) <= 0.0 || camera.K(1, 1) <= 0.0)
    throw std::runtime_error("invalid ETH3D pair " + name + " camera matrix");
  camera.intrinsics << camera.K(0, 0), camera.K(1, 1), camera.K(0, 2),
      camera.K(1, 2);
  camera.P << camera.K, Eigen::Vector3d::Zero();
  if (width > 0 && height > 0) camera.resolution = {width, height};
  return camera;
}

Eth3dPairCalibration read_eth3d_pair_calibration(const fs::path& path) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("missing ETH3D pair calibration: " + path.string());
  std::vector<double> cam0_values;
  std::vector<double> cam1_values;
  double baseline_mm = std::numeric_limits<double>::quiet_NaN();
  int width = 0;
  int height = 0;
  std::string line;
  while (std::getline(input, line)) {
    const auto lowered = lower(line);
    if (lowered.empty() || lowered.front() == '#') continue;
    if (lowered.find("cam0") != std::string::npos) {
      const auto values = eth3d_values_after(line, "cam0");
      cam0_values.insert(cam0_values.end(), values.begin(), values.end());
    }
    if (lowered.find("cam1") != std::string::npos) {
      const auto values = eth3d_values_after(line, "cam1");
      cam1_values.insert(cam1_values.end(), values.begin(), values.end());
    }
    if (lowered.find("baseline") != std::string::npos) {
      const auto values = eth3d_values_after(line, "baseline");
      if (!values.empty()) baseline_mm = values.front();
    }
    if (lowered.find("width") != std::string::npos) {
      const auto values = eth3d_values_after(line, "width");
      if (!values.empty()) width = static_cast<int>(std::llround(values.front()));
    }
    if (lowered.find("height") != std::string::npos) {
      const auto values = eth3d_values_after(line, "height");
      if (!values.empty()) height = static_cast<int>(std::llround(values.front()));
    }
    if (lowered.find("resolution") != std::string::npos) {
      const auto values = eth3d_values_after(line, "resolution");
      if (values.size() >= 2) {
        width = static_cast<int>(std::llround(values[0]));
        height = static_cast<int>(std::llround(values[1]));
      }
    }
  }
  if (!std::isfinite(baseline_mm) || baseline_mm <= 0.0)
    throw std::runtime_error("ETH3D pair calibration has invalid baseline: " +
                             path.string());
  Eth3dPairCalibration result;
  result.left = eth3d_camera_from_matrix(cam0_values, "cam0", width, height);
  result.right = eth3d_camera_from_matrix(cam1_values, "cam1", width, height);
  result.stereo.R_target_reference = Eigen::Matrix3d::Identity();
  // The right camera is displaced +baseline in the left frame, so a point's
  // right-camera coordinates are p_right = p_left - [baseline, 0, 0].
  result.stereo.t_target_reference =
      Eigen::Vector3d(-baseline_mm * 1e-3, 0.0, 0.0);
  result.stereo.baseline = baseline_mm * 1e-3;
  Eigen::Matrix3d tx;
  const auto& t = result.stereo.t_target_reference;
  tx << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  result.stereo.F_target_reference = result.right.K.inverse().transpose() * tx *
                                     result.left.K.inverse();
  return result;
}

void load_eth3d_calibration(const fs::path& base, DatasetConfig& config) {
  // The official ETH3D stereo layout stores one four-scalar intrinsics file
  // per camera and a 3x4 CAMERA2->CAMERA1 transform.  Lab YAML values remain
  // authoritative when supplied, so native files only fill missing pieces.
  const fs::path left_file = base / "calibration.txt";
  const fs::path right_file = base / "calibration2.txt";
  if (config.cameras.size() < 2 && fs::exists(left_file) && fs::exists(right_file)) {
    CameraCalibration left = eth3d_camera_from_file(left_file, "cam0");
    CameraCalibration right = eth3d_camera_from_file(right_file, "cam1");
    if (!config.cameras.empty()) left = config.cameras.front();
    if (config.cameras.size() > 1) right = config.cameras[1];
    config.cameras = {left, right};
  }
  if (config.cameras.size() < 2 && fs::exists(base / "calib.txt")) {
    const auto values = read_eth3d_scalars(base / "calib.txt");
    if (values.size() >= 8) {
      config.cameras = {eth3d_camera_from_values(values, 0, "cam0"),
                        eth3d_camera_from_values(values, 4, "cam1")};
    }
  }
  if (!config.stereo && fs::exists(base / "extrinsics_1_2.txt")) {
    const auto values = read_eth3d_scalars(base / "extrinsics_1_2.txt");
    if (values.size() < 12)
      throw std::runtime_error("ETH3D extrinsics_1_2.txt must contain 12 values");
    Eigen::Matrix<double, 3, 4> camera1_from_camera2;
    for (int i = 0; i < 12; ++i) camera1_from_camera2(i / 4, i % 4) = values[i];
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = camera1_from_camera2.leftCols<3>();
    transform.translation() = camera1_from_camera2.col(3);
    // File name means CAMERA2 -> CAMERA1 (p1 = T_1_2 p2).  Our API uses
    // target-reference = CAMERA2 <- CAMERA1, hence the inverse.
    const auto right_from_left = transform.inverse();
    StereoCalibration stereo;
    stereo.R_target_reference = right_from_left.rotation();
    stereo.t_target_reference = right_from_left.translation();
    stereo.baseline = stereo.t_target_reference.norm();
    if (config.cameras.size() >= 2) {
      Eigen::Matrix3d tx;
      const auto& t = stereo.t_target_reference;
      tx << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
      stereo.F_target_reference = config.cameras[1].K.inverse().transpose() * tx *
                                 stereo.R_target_reference * config.cameras[0].K.inverse();
    }
    config.stereo = stereo;
  }
  if (config.cameras.size() < 2)
    throw std::runtime_error("ETH3D requires left/right calibration YAML or native calibration.txt/calibration2.txt");
  if (!config.stereo)
    throw std::runtime_error("ETH3D requires stereo calibration or extrinsics_1_2.txt");
}

std::size_t configured_num_cameras(const DatasetConfig& config) {
  if (!config.options.count("num_cameras")) return 2;
  const auto value = std::stoll(config.options.at("num_cameras"));
  if (value < 2)
    throw std::runtime_error("num_cameras must be >= 2");
  return static_cast<std::size_t>(value);
}

void make_temporal_windows(std::vector<FrameSet>& sets,
                           const DatasetConfig& config) {
  const std::size_t num_cameras = configured_num_cameras(config);
  const std::size_t pairs_per_window = (num_cameras + 1) / 2;
  if (pairs_per_window <= 1) return;
  if (sets.size() < pairs_per_window) {
    sets.clear();
    return;
  }
  std::vector<FrameSet> windows;
  windows.reserve(sets.size() - pairs_per_window + 1);
  for (std::size_t start = 0;
       start + pairs_per_window <= sets.size(); ++start) {
    FrameSet window;
    window.index = start;
    window.timestamp_ns = sets[start].timestamp_ns;
    window.frames.reserve(num_cameras);
    for (std::size_t offset = 0; offset < pairs_per_window; ++offset) {
      if (sets[start + offset].frames.empty())
        throw std::runtime_error("temporal window contains an empty frame set");
      window.frames.push_back(sets[start + offset].frames[0]);
      if (window.frames.size() < num_cameras) {
        if (sets[start + offset].frames.size() < 2)
          throw std::runtime_error("temporal window contains an incomplete stereo set");
        window.frames.push_back(sets[start + offset].frames[1]);
      }
    }
    windows.push_back(std::move(window));
  }
  sets.swap(windows);
}

fs::path euroc_base(const DatasetConfig& config) {
  const fs::path direct = config.root / "mav0";
  if (fs::exists(direct)) return direct;
  if (!config.sequence.empty() && fs::exists(config.root / config.sequence / "mav0"))
    return config.root / config.sequence / "mav0";
  return direct;
}

class VectorIterator final : public DatasetIterator {
 public:
  VectorIterator(std::vector<FrameSet> sets, std::size_t skip)
      : sets_(std::move(sets)), step_(skip + 1) {}
  std::optional<FrameSet> next() override {
    if (!has_next()) return std::nullopt;
    FrameSet result = sets_[index_];
    for (auto& frame : result.frames) materialize(frame);
    index_ += step_;
    return result;
  }
  void reset() override { index_ = 0; }
  bool has_next() const override { return index_ < sets_.size(); }
  std::size_t size() const override {
    return index_ >= sets_.size() ? 0 : (sets_.size() - index_ + step_ - 1) / step_;
  }

 private:
  std::vector<FrameSet> sets_;
  std::size_t index_{0};
  std::size_t step_{1};
};

class BuiltinDataset final : public Dataset {
 public:
  explicit BuiltinDataset(DatasetConfig config) : config_(std::move(config)) {
    load();
    if (sets_.empty())
      throw std::runtime_error("no synchronized frames found under " +
                               config_.root.string());
    for (const auto& frame : sets_.front().frames) {
      if (width_ == 0 || height_ == 0) {
        const cv::Mat image = cv::imread(frame.image_path.string(), cv::IMREAD_GRAYSCALE);
        if (image.empty())
          throw std::runtime_error("failed to read image: " + frame.image_path.string());
        width_ = image.cols;
        height_ = image.rows;
      }
      break;
    }
    build_camera_info(config_.cameras, config_.stereo, camera_info_);
  }

  const DatasetConfig& config() const override { return config_; }
  const std::vector<CameraCalibration>& cameras() const override { return config_.cameras; }
  const std::optional<StereoCalibration>& stereo_calibration() const override {
    return config_.stereo;
  }
  const FileInfo& file_info() const override { return file_info_; }
  const CameraInfo& camera_info() const override { return camera_info_; }
  std::size_t size() const override { return sets_.size(); }
  int width() const override { return width_; }
  int height() const override { return height_; }
  bool has_ground_truth() const override { return file_info_.has_gt; }
  std::unique_ptr<DatasetIterator> iterate() const override {
    return std::make_unique<VectorIterator>(sets_, config_.skip_frames);
  }

 private:
  void set_file_identity() {
    file_info_.dataset_type = config_.type;
    file_info_.dataset_path = config_.root.string();
    file_info_.sequence_name = config_.sequence;
    auto output = config_.options.find("output_dir");
    if (output != config_.options.end()) file_info_.output_path = output->second;
  }

  void append_image_timestamp(Timestamp timestamp) {
    file_info_.Img_time_stamps.push_back(ns_to_seconds(timestamp));
  }

  void append_gt_timestamp(Timestamp timestamp) {
    file_info_.GT_time_stamps.push_back(ns_to_seconds(timestamp));
  }

  void load_kitti() {
    fs::path base = config_.root / config_.sequence;
    const auto has_kitti_stream = [](const fs::path& path) {
      return fs::exists(path / "image_0") || fs::exists(path / "image_2");
    };
    if (!has_kitti_stream(base)) base = config_.root / "sequences" / config_.sequence;
    const bool has_gray = fs::exists(base / "image_0") &&
                          fs::exists(base / "image_1");
    const bool has_color = fs::exists(base / "image_2") &&
                           fs::exists(base / "image_3");
    if (!has_gray && !has_color)
      throw std::runtime_error("KITTI sequence has no complete stereo image pair: " +
                               base.string());
    std::string pair = config_.options.count("kitti_camera_pair")
                           ? lower(config_.options.at("kitti_camera_pair"))
                           : (has_gray ? "gray" : "color");
    if ((pair == "gray" && !has_gray) || (pair == "color" && !has_color))
      throw std::runtime_error("requested KITTI camera pair is missing: " + pair);
    config_.options["kitti_camera_pair"] = pair;
    Eigen::Vector3d left_reference_offset = Eigen::Vector3d::Zero();
    load_kitti_calibration(base / "calib.txt", config_, &left_reference_offset);
    if (config_.cameras.size() < 2)
      throw std::runtime_error("KITTI requires two camera calibrations: " +
                               (base / "calib.txt").string());
    const fs::path left_dir = base / (pair == "color" ? "image_2" : "image_0");
    const fs::path right_dir = base / (pair == "color" ? "image_3" : "image_1");
    const auto left_images = images_in(left_dir);
    const auto right_images = images_in(right_dir);
    auto samples = exact_pair(left_images, right_images, "KITTI");
    const auto times = fs::exists(base / "times.txt")
                           ? scalar_timestamps(base / "times.txt", "KITTI times")
                           : std::vector<Timestamp>{};
    if (!times.empty() && times.size() != samples.size())
      throw std::runtime_error("KITTI times.txt count does not match image count");
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const Timestamp timestamp = times.empty() ? samples[i].timestamp : times[i];
      samples[i].timestamp = timestamp;
      append_image_timestamp(timestamp);
    }
    fs::path gt;
    const auto sequence = fs::path(config_.sequence).filename().string();
    if (config_.ground_truth_path) {
      gt = *config_.ground_truth_path;
      if (fs::is_directory(gt)) gt /= sequence + ".txt";
    } else {
      gt = config_.root / "poses" / (sequence + ".txt");
    }
    const auto poses = read_kitti_poses(gt);
    if (!poses.empty() && poses.size() != samples.size())
      throw std::runtime_error("KITTI ground-truth count does not match images: " +
                               gt.string());
    if (!poses.empty()) {
      file_info_.has_gt = true;
      file_info_.GT_file_name = gt.string();
    }
    for (std::size_t i = 0; i < samples.size(); ++i) {
      FrameSet set;
      set.index = i;
      set.timestamp_ns = samples[i].timestamp;
      set.frames.push_back(make_frame("cam0", samples[i].timestamp, samples[i].path,
                                      config_.cameras.size() > 0 ? &config_.cameras[0]
                                                                 : nullptr));
      const auto right_path = right_images[i];
      set.frames.push_back(make_frame("cam1", samples[i].timestamp, right_path,
                                      config_.cameras.size() > 1 ? &config_.cameras[1]
                                                                 : nullptr));
      if (!poses.empty()) {
        Eigen::Isometry3d world_from_left = Eigen::Isometry3d::Identity();
        world_from_left.linear() = poses[i].leftCols<3>();
        world_from_left.translation() = poses[i].col(3);
        // KITTI poses are expressed in the camera-0 reference frame.  Color
        // streams use P2/P3, so move that world-to-camera-0 transform into
        // the selected P2/P3 frame using the full K^-1 p3 offsets.
        Eigen::Isometry3d left_from_camera0 = Eigen::Isometry3d::Identity();
        left_from_camera0.translation() = left_reference_offset;
        const Eigen::Isometry3d world_to_left =
            left_from_camera0 * world_from_left.inverse();
        set_ground_truth(set.frames[0], make_pose(samples[i].timestamp, world_to_left));
        Eigen::Isometry3d right_from_left = Eigen::Isometry3d::Identity();
        if (config_.stereo) {
          right_from_left.linear() = config_.stereo->R_target_reference;
          right_from_left.translation() = config_.stereo->t_target_reference;
        }
        set_ground_truth(set.frames[1],
                         make_pose(samples[i].timestamp, right_from_left * world_to_left));
        append_gt_timestamp(samples[i].timestamp);
      }
      sets_.push_back(std::move(set));
    }
    make_temporal_windows(sets_, config_);
  }

  void load_euroc() {
    const fs::path base = euroc_base(config_);
    if (!fs::exists(base / "cam0/data.csv") || !fs::exists(base / "cam1/data.csv"))
      throw std::runtime_error("EuRoC sequence missing cam0/cam1 data.csv: " +
                               base.string());
    if (config_.cameras.size() < 2) {
      config_.cameras = {read_euroc_camera(base / "cam0/sensor.yaml", "cam0"),
                         read_euroc_camera(base / "cam1/sensor.yaml", "cam1")};
    }
    if (!config_.stereo && config_.cameras.size() >= 2) {
      const auto right_from_left = config_.cameras[1].T_body_camera.inverse() *
                                   config_.cameras[0].T_body_camera;
      StereoCalibration stereo;
      stereo.R_target_reference = right_from_left.rotation();
      stereo.t_target_reference = right_from_left.translation();
      stereo.baseline = stereo.t_target_reference.norm();
      Eigen::Matrix3d tx;
      const auto& t = stereo.t_target_reference;
      tx << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
      stereo.F_target_reference = config_.cameras[1].K.inverse().transpose() * tx *
                                 stereo.R_target_reference * config_.cameras[0].K.inverse();
      config_.stereo = stereo;
    }
    const auto left = image_csv(base / "cam0/data.csv", base / "cam0/data", "EuRoC cam0", true);
    const auto right = image_csv(base / "cam1/data.csv", base / "cam1/data", "EuRoC cam1", true);
    for (const auto& sample : left) append_image_timestamp(sample.timestamp);
    fs::path gt = config_.ground_truth_path
                      ? *config_.ground_truth_path
                      : base / "state_groundtruth_estimate0/data.csv";
    std::vector<Timestamp> gt_timestamps;
    const auto gt_poses = read_euroc_ground_truth(
        gt, config_.cameras.empty() ? nullptr : &config_.cameras[0], gt_timestamps);
    if (!gt_poses.empty()) {
      file_info_.has_gt = true;
      file_info_.GT_file_name = gt.string();
      for (const auto timestamp : gt_timestamps) append_gt_timestamp(timestamp);
    }
    const auto pairs = match_samples(left, right, config_.sync_tolerance_ns);
    for (const auto& pair : pairs) {
      const auto& l = left[pair.first];
      const auto& r = right[pair.second];
      FrameSet set;
      set.index = sets_.size();
      set.timestamp_ns = l.timestamp;
      set.frames.push_back(make_frame("cam0", l.timestamp, l.path,
                                      config_.cameras.size() > 0 ? &config_.cameras[0]
                                                                 : nullptr));
      set.frames.push_back(make_frame("cam1", r.timestamp, r.path,
                                      config_.cameras.size() > 1 ? &config_.cameras[1]
                                                                 : nullptr));
      if (!gt_poses.empty()) {
        const auto left_pose = nearest_pose(
            gt_poses, l.timestamp, config_.sync_tolerance_ns);
        if (left_pose) {
          set_ground_truth(set.frames[0], *left_pose);
          const Eigen::Isometry3d camera1_from_body =
              config_.cameras[1].T_body_camera.inverse();
          const Eigen::Isometry3d body_from_camera0 =
              config_.cameras[0].T_body_camera;
          const auto right_pose = camera1_from_body * body_from_camera0 *
                                  pose_transform(*left_pose);
          set_ground_truth(set.frames[1], make_pose(r.timestamp, right_pose));
        }
      }
      sets_.push_back(std::move(set));
    }
    make_temporal_windows(sets_, config_);
  }

  void attach_eth3d_metadata(Frame& frame, const fs::path& folder,
                             bool right) {
    const std::string suffix = right ? "1" : "0";
    const fs::path disparity = folder / ("disp" + suffix + "GT.pfm");
    const fs::path mask = folder / ("mask" + suffix + "nocc.png");
    if (fs::exists(disparity)) frame.metadata.disparity_path = disparity;
    if (fs::exists(mask)) frame.metadata.occlusion_mask_path = mask;
  }

  void load_eth3d_pair_folders(const fs::path& base) {
    const fs::path pairs_path = base / "stereo_pairs";
    if (!fs::exists(pairs_path)) return;
    std::vector<fs::path> folders;
    for (const auto& entry : fs::directory_iterator(pairs_path))
      if (entry.is_directory()) folders.push_back(entry.path());
    std::sort(folders.begin(), folders.end());
    const std::size_t num_cameras = configured_num_cameras(config_);
    const std::size_t folder_count = (num_cameras + 1) / 2;
    if (folders.size() < folder_count) return;
    const bool allow_calibration_fallback =
        config_.cameras.size() >= 2 && config_.stereo.has_value();
    for (std::size_t start = 0; start + folder_count <= folders.size(); ++start) {
      FrameSet set;
      set.index = sets_.size();
      set.timestamp_ns = seconds_to_ns(static_cast<double>(start));
      for (std::size_t camera_index = 0; camera_index < num_cameras; ++camera_index) {
        const fs::path folder = folders[start + static_cast<std::size_t>(camera_index / 2)];
        const bool right = camera_index % 2 == 1;
        const fs::path image = folder / (right ? "im1.png" : "im0.png");
        if (!fs::is_regular_file(image))
          throw std::runtime_error("missing ETH3D stereo image: " + image.string());
        std::optional<Eth3dPairCalibration> pair_calibration;
        const fs::path calibration_path = folder / "calib.txt";
        if (fs::exists(calibration_path)) {
          pair_calibration = read_eth3d_pair_calibration(calibration_path);
          if (config_.cameras.size() < 2) {
            config_.cameras = {pair_calibration->left, pair_calibration->right};
            config_.stereo = pair_calibration->stereo;
          } else if (!config_.stereo) {
            config_.stereo = pair_calibration->stereo;
          }
        } else if (!allow_calibration_fallback) {
          throw std::runtime_error("ETH3D pair has no calibration: " +
                                   calibration_path.string());
        }
        const auto* camera = pair_calibration
                                 ? (right ? &pair_calibration->right
                                          : &pair_calibration->left)
                                 : &config_.cameras[static_cast<std::size_t>(camera_index) %
                                                    config_.cameras.size()];
        const Timestamp frame_timestamp =
            seconds_to_ns(static_cast<double>(start + static_cast<std::size_t>(camera_index / 2)));
        auto frame = make_frame("cam" + std::to_string(camera_index), frame_timestamp,
                                image, camera);
        attach_eth3d_metadata(frame, folder, right);
        const auto poses = read_eth3d_poses(
            folder / "images.txt", right ? "im1.png" : "im0.png", frame_timestamp);
        if (!poses.empty()) {
          set_ground_truth(frame, poses.front());
          file_info_.has_gt = true;
          append_gt_timestamp(frame_timestamp);
        }
        set.frames.push_back(std::move(frame));
      }
      append_image_timestamp(set.timestamp_ns);
      sets_.push_back(std::move(set));
    }
  }

  void load_eth3d_slam(const fs::path& base) {
    const auto left = timed_paths(base / "rgb.txt", base, "ETH3D rgb");
    const fs::path right_list = base / "rgb2.txt";
    std::vector<Sample> right;
    if (fs::exists(right_list)) right = timed_paths(right_list, base, "ETH3D rgb2");
    else {
      right.reserve(left.size());
      for (const auto& sample : left) {
        fs::path path = sample.path;
        auto text = path.generic_string();
        const auto position = text.find("rgb/");
        if (position != std::string::npos) text.replace(position, 4, "rgb2/");
        else text = (base / "rgb2" / path.filename()).string();
        const fs::path right_path(text);
        if (!fs::is_regular_file(right_path))
          throw std::runtime_error("missing ETH3D right image: " + right_path.string());
        right.push_back({sample.timestamp, right_path,
                         right_path.filename().string()});
      }
    }
    const auto pairs = match_samples(left, right, config_.sync_tolerance_ns);
    if (pairs.size() != left.size() || pairs.size() != right.size())
      throw std::runtime_error(
          "ETH3D SLAM image streams contain unmatched timestamps");
    fs::path gt = config_.ground_truth_path ? *config_.ground_truth_path
                                            : base / "groundtruth.txt";
    const auto gt_poses = read_tum_ground_truth(gt);
    if (!gt_poses.empty()) {
      file_info_.has_gt = true;
      file_info_.GT_file_name = gt.string();
      for (const auto& pose : gt_poses) append_gt_timestamp(pose.timestamp_ns);
    }
    const std::size_t num_cameras = configured_num_cameras(config_);
    const std::size_t window = (num_cameras + 1) / 2;
    if (pairs.size() < window) return;
    for (std::size_t start = 0; start + window <= pairs.size(); ++start) {
      FrameSet set;
      set.index = sets_.size();
      set.timestamp_ns = left[pairs[start].first].timestamp;
      for (std::size_t offset = 0; offset < window; ++offset) {
        const auto& l = left[pairs[start + offset].first];
        const auto& r = right[pairs[start + offset].second];
        if (!fs::is_regular_file(l.path) || !fs::is_regular_file(r.path))
          throw std::runtime_error("missing ETH3D SLAM image in synchronized pair");
        const auto* left_camera = config_.cameras.empty() ? nullptr : &config_.cameras[0];
        const auto* right_camera = config_.cameras.size() < 2 ? nullptr : &config_.cameras[1];
        auto left_frame = make_frame("cam0", l.timestamp, l.path, left_camera);
        auto right_frame = make_frame("cam1", r.timestamp, r.path, right_camera);
        if (!gt_poses.empty()) {
          const auto pose = nearest_pose(gt_poses, l.timestamp, config_.sync_tolerance_ns);
          if (pose) {
            set_ground_truth(left_frame, *pose);
            Eigen::Isometry3d right_from_left = Eigen::Isometry3d::Identity();
            right_from_left.linear() = config_.stereo->R_target_reference;
            right_from_left.translation() = config_.stereo->t_target_reference;
            set_ground_truth(
                right_frame,
                make_pose(r.timestamp, right_from_left * pose_transform(*pose)));
          }
        }
        // Keep camera order [L(k), R(k), L(k+1), R(k+1), ...].  An odd
        // num_cameras requests the final left frame without its right mate.
        set.frames.push_back(std::move(left_frame));
        if (set.frames.size() < num_cameras)
          set.frames.push_back(std::move(right_frame));
      }
      append_image_timestamp(set.timestamp_ns);
      sets_.push_back(std::move(set));
    }
  }

  void load_eth3d() {
    fs::path base = config_.root;
    if (!config_.sequence.empty() && fs::exists(config_.root / config_.sequence))
      base = config_.root / config_.sequence;
    const bool pair_layout = fs::exists(base / "stereo_pairs");
    const bool has_base_calibration =
        (fs::exists(base / "calibration.txt") &&
         fs::exists(base / "calibration2.txt")) ||
        fs::exists(base / "calib.txt") ||
        fs::exists(base / "extrinsics_1_2.txt");
    // Native stereo-pair scenes carry calibration inside each pair folder.
    // Defer the global calibration requirement until those files are read.
    if (!pair_layout || has_base_calibration ||
        (config_.cameras.size() >= 2 && config_.stereo))
      load_eth3d_calibration(base, config_);
    if (pair_layout) load_eth3d_pair_folders(base);
    else if (fs::exists(base / "rgb.txt")) load_eth3d_slam(base);
    else throw std::runtime_error("ETH3D layout not recognized under " + base.string());
  }

  void load() {
    set_file_identity();
    const auto type = lower(config_.type);
    if (type.find("kitti") != std::string::npos) load_kitti();
    else if (type.find("euroc") != std::string::npos) load_euroc();
    else if (type.find("eth3d") != std::string::npos) load_eth3d();
    else throw std::invalid_argument("unsupported dataset type: " + config_.type);
    if (config_.cameras.size() >= 2) {
      for (const auto& set : sets_) {
        if (!set.frames.empty()) set_resolution_from_image(config_.cameras[0], set.frames[0].image_path);
        if (set.frames.size() > 1)
          set_resolution_from_image(config_.cameras[1], set.frames[1].image_path);
        break;
      }
    }
  }

  DatasetConfig config_;
  FileInfo file_info_;
  CameraInfo camera_info_;
  std::vector<FrameSet> sets_;
  int width_{0};
  int height_{0};
};

}  // namespace

std::unique_ptr<Dataset> open_dataset(DatasetConfig config) {
  return std::make_unique<BuiltinDataset>(std::move(config));
}

}  // namespace lems::data
