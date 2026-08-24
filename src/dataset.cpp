#include "lems/data/dataset.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;
namespace lems::data {
namespace {

Timestamp seconds_to_ns(double seconds) {
  return static_cast<Timestamp>(std::llround(seconds * 1e9));
}

double ns_to_seconds(Timestamp timestamp) {
  return static_cast<double>(timestamp) / 1e9;
}

std::vector<fs::path> images_in(const fs::path& directory) {
  std::vector<fs::path> result;
  if (!fs::exists(directory)) return result;
  for (const auto& entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file()) continue;
    auto extension = entry.path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (extension == ".png" || extension == ".jpg" ||
        extension == ".jpeg" || extension == ".pgm")
      result.push_back(entry.path());
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<Timestamp> timestamps_file(const fs::path& path) {
  std::vector<Timestamp> result;
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream values(line);
    double timestamp;
    if (values >> timestamp) result.push_back(seconds_to_ns(timestamp));
  }
  return result;
}

std::vector<double> values_after_label(const std::string& line) {
  std::vector<double> result;
  const auto separator = line.find(':');
  std::istringstream input(
      separator == std::string::npos ? line : line.substr(separator + 1));
  double value;
  while (input >> value) result.push_back(value);
  return result;
}

std::vector<Eigen::Matrix<double, 3, 4>> kitti_poses(const fs::path& path) {
  std::vector<Eigen::Matrix<double, 3, 4>> result;
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream values(line);
    Eigen::Matrix<double, 3, 4> pose;
    bool valid = true;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 4; ++col)
        if (!(values >> pose(row, col))) valid = false;
    if (valid) result.push_back(pose);
  }
  return result;
}

void load_kitti_calibration(const fs::path& path, DatasetConfig& config) {
  std::ifstream input(path);
  if (!input) return;
  Eigen::Matrix<double, 3, 4> left_p, right_p;
  bool has_left = false, has_right = false;
  std::string line;
  while (std::getline(input, line)) {
    const bool left = line.rfind("P0:", 0) == 0 || line.rfind("P2:", 0) == 0;
    const bool right = line.rfind("P1:", 0) == 0 || line.rfind("P3:", 0) == 0;
    if (!left && !right) continue;
    const auto values = values_after_label(line);
    if (values.size() != 12) continue;
    auto& projection = left ? left_p : right_p;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 4; ++col)
        projection(row, col) = values[row * 4 + col];
    (left ? has_left : has_right) = true;
  }
  if (!has_left || !has_right) return;

  CameraCalibration left, right;
  left.name = "cam0";
  right.name = "cam1";
  left.P = left_p;
  right.P = right_p;
  left.K = left_p.leftCols<3>();
  right.K = right_p.leftCols<3>();
  left.intrinsics << left.K(0, 0), left.K(1, 1), left.K(0, 2), left.K(1, 2);
  right.intrinsics << right.K(0, 0), right.K(1, 1), right.K(0, 2), right.K(1, 2);
  left.distortion = Eigen::Vector4d::Zero();
  right.distortion = Eigen::Vector4d::Zero();
  config.cameras = {left, right};

  StereoCalibration stereo;
  const double left_tx = left_p(0, 3) / left_p(0, 0);
  const double right_tx = right_p(0, 3) / right_p(0, 0);
  stereo.t_target_reference.x() = right_tx - left_tx;
  stereo.baseline = std::abs(stereo.t_target_reference.x());
  Eigen::Matrix3d tx;
  const auto& t = stereo.t_target_reference;
  tx << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  stereo.F_target_reference =
      right.K.inverse().transpose() * tx * left.K.inverse();
  config.stereo = stereo;
}

struct Sample {
  Timestamp timestamp;
  fs::path path;
};

std::vector<CameraPose> euroc_ground_truth(
    const fs::path& csv, const Eigen::Isometry3d& body_from_camera) {
  std::vector<CameraPose> result;
  std::ifstream input(csv);
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    Timestamp timestamp;
    Eigen::Vector3d position;
    double qw, qx, qy, qz;
    if (!(values >> timestamp >> position.x() >> position.y() >> position.z()
                 >> qw >> qx >> qy >> qz)) continue;
    Eigen::Isometry3d world_from_body = Eigen::Isometry3d::Identity();
    world_from_body.linear() =
        Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    world_from_body.translation() = position;
    const Eigen::Isometry3d world_from_camera =
        world_from_body * body_from_camera;
    result.emplace_back(timestamp, world_from_camera.rotation(),
                        world_from_camera.translation());
  }
  return result;
}

std::optional<CameraPose> eth3d_pose(const fs::path& images_file,
                                     const std::string& image_name,
                                     Timestamp timestamp) {
  std::ifstream input(images_file);
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream values(line);
    int image_id, camera_id;
    double qw, qx, qy, qz, tx, ty, tz;
    std::string filename;
    if (values >> image_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz
               >> camera_id >> filename && filename == image_name) {
      return CameraPose(timestamp, Eigen::Quaterniond(qw, qx, qy, qz),
                        Eigen::Vector3d(tx, ty, tz));
    }
  }
  return std::nullopt;
}

std::optional<CameraPose> closest_pose(const std::vector<CameraPose>& poses,
                                       Timestamp timestamp) {
  if (poses.empty()) return std::nullopt;
  auto it = std::lower_bound(poses.begin(), poses.end(), timestamp,
      [](const CameraPose& pose, Timestamp value) {
        return pose.timestamp_ns < value;
      });
  if (it == poses.end()) return poses.back();
  if (it == poses.begin()) return *it;
  const auto previous = std::prev(it);
  return std::llabs(previous->timestamp_ns - timestamp) <
                 std::llabs(it->timestamp_ns - timestamp)
             ? *previous : *it;
}

std::vector<Sample> euroc_csv(const fs::path& csv,
                              const fs::path& data_directory) {
  std::vector<Sample> result;
  std::ifstream input(csv);
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    Timestamp timestamp;
    std::string filename;
    if (values >> timestamp >> filename)
      result.push_back({timestamp, data_directory / filename});
  }
  return result;
}

CameraCalibration euroc_camera(const fs::path& sensor_yaml,
                               const std::string& name) {
  const auto node = YAML::LoadFile(sensor_yaml.string());
  CameraCalibration camera;
  camera.name = name;
  camera.model = node["camera_model"].as<std::string>("pinhole");
  const auto resolution = node["resolution"].as<std::vector<int>>();
  const auto intrinsics = node["intrinsics"].as<std::vector<double>>();
  const auto distortion =
      node["distortion_coefficients"].as<std::vector<double>>();
  if (resolution.size() != 2 || intrinsics.size() != 4)
    throw std::runtime_error("invalid EuRoC sensor calibration: " +
                             sensor_yaml.string());
  camera.resolution = {resolution[0], resolution[1]};
  camera.intrinsics = Eigen::Map<const Eigen::Vector4d>(intrinsics.data());
  camera.K << intrinsics[0], 0.0, intrinsics[2],
              0.0, intrinsics[1], intrinsics[3],
              0.0, 0.0, 1.0;
  camera.P << camera.K, Eigen::Vector3d::Zero();
  camera.distortion = Eigen::Map<const Eigen::VectorXd>(distortion.data(),
      static_cast<Eigen::Index>(distortion.size()));
  const auto transform = node["T_BS"]["data"].as<std::vector<double>>();
  if (transform.size() == 16) {
    Eigen::Matrix4d matrix;
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        matrix(row, col) = transform[row * 4 + col];
    camera.T_body_camera.matrix() = matrix;
  }
  return camera;
}

Frame make_frame(std::string camera, Timestamp timestamp, fs::path image,
                 const CameraCalibration* calibration = nullptr) {
  Frame frame;
  frame.camera = std::move(camera);
  frame.timestamp_ns = timestamp;
  frame.timestamp_seconds = ns_to_seconds(timestamp);
  frame.image_path = std::move(image);
  if (calibration) frame.K = calibration->K;
  return frame;
}

cv::Mat read_pfm(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return {};
  std::string type;
  int width, height;
  float scale;
  input >> type >> width >> height >> scale;
  input.get();
  if ((type != "Pf" && type != "PF") || width <= 0 || height <= 0) return {};
  const int channels = type == "PF" ? 3 : 1;
  cv::Mat result(height, width, channels == 1 ? CV_32FC1 : CV_32FC3);
  input.read(reinterpret_cast<char*>(result.data),
             static_cast<std::streamsize>(result.total() * result.elemSize()));
  if (!input) return {};
  cv::flip(result, result, 0);
  return result;
}

void materialize(Frame& frame) {
  frame.image = cv::imread(frame.image_path.string(), cv::IMREAD_GRAYSCALE);
  if (frame.image.empty())
    throw std::runtime_error("failed to read image: " + frame.image_path.string());
  if (frame.metadata.disparity_path)
    frame.metadata.disparity = read_pfm(*frame.metadata.disparity_path);
  if (frame.metadata.occlusion_mask_path)
    frame.metadata.occlusion_mask = cv::imread(
        frame.metadata.occlusion_mask_path->string(), cv::IMREAD_GRAYSCALE);
  if (frame.metadata.depth_path)
    frame.metadata.depth = cv::imread(frame.metadata.depth_path->string(),
                                      cv::IMREAD_UNCHANGED);
}

class VectorIterator final : public DatasetIterator {
 public:
  VectorIterator(std::vector<FrameSet> sets, std::size_t skip)
      : sets_(std::move(sets)), step_(skip + 1) {}

  std::optional<FrameSet> next() override {
    if (index_ >= sets_.size()) return std::nullopt;
    FrameSet result = sets_[index_];
    index_ += step_;
    for (auto& frame : result.frames) materialize(frame);
    return result;
  }
  void reset() override { index_ = 0; }

 private:
  std::vector<FrameSet> sets_;
  std::size_t index_{};
  std::size_t step_{1};
};

class BuiltinDataset final : public Dataset {
 public:
  explicit BuiltinDataset(DatasetConfig config) : config_(std::move(config)) {
    load();
  }
  const DatasetConfig& config() const override { return config_; }
  const std::vector<CameraCalibration>& cameras() const override {
    return config_.cameras;
  }
  const std::optional<StereoCalibration>& stereo_calibration() const override {
    return config_.stereo;
  }
  std::unique_ptr<DatasetIterator> iterate() const override {
    return std::make_unique<VectorIterator>(sets_, config_.skip_frames);
  }

 private:
  void load_kitti() {
    fs::path base = config_.root / config_.sequence;
    if (!fs::exists(base / "image_0"))
      base = config_.root / "sequences" / config_.sequence;
    load_kitti_calibration(base / "calib.txt", config_);
    auto left = images_in(base / "image_0");
    auto right = images_in(base / "image_1");
    auto timestamps = timestamps_file(base / "times.txt");

    fs::path gt;
    const auto sequence = fs::path(config_.sequence).filename().string();
    if (config_.ground_truth_path) {
      gt = *config_.ground_truth_path;
      if (fs::is_directory(gt)) gt /= sequence + ".txt";
    } else {
      gt = config_.root / "poses" / (sequence + ".txt");
    }
    const auto poses = kitti_poses(gt);
    const auto count = std::min(left.size(), right.size());
    for (std::size_t i = 0; i < count; ++i) {
      const Timestamp timestamp =
          i < timestamps.size() ? timestamps[i]
                                : seconds_to_ns(static_cast<double>(i));
      FrameSet set;
      set.index = i;
      set.timestamp_ns = timestamp;
      set.frames.push_back(make_frame("cam0", timestamp, left[i],
          config_.cameras.size() > 0 ? &config_.cameras[0] : nullptr));
      set.frames.push_back(make_frame("cam1", timestamp, right[i],
          config_.cameras.size() > 1 ? &config_.cameras[1] : nullptr));
      if (i < poses.size()) {
        CameraPose pose(timestamp, poses[i].leftCols<3>(), poses[i].col(3));
        set.frames[0].ground_truth = pose;
        set.frames[1].ground_truth = pose;
      }
      sets_.push_back(std::move(set));
    }
  }

  void load_euroc() {
    const fs::path base = config_.root / config_.sequence / "mav0";
    if (config_.cameras.size() < 2 &&
        fs::exists(base / "cam0/sensor.yaml") &&
        fs::exists(base / "cam1/sensor.yaml")) {
      auto left_camera = euroc_camera(base / "cam0/sensor.yaml", "cam0");
      auto right_camera = euroc_camera(base / "cam1/sensor.yaml", "cam1");
      StereoCalibration stereo;
      const Eigen::Isometry3d target_from_reference =
          right_camera.T_body_camera.inverse() * left_camera.T_body_camera;
      stereo.R_target_reference = target_from_reference.rotation();
      stereo.t_target_reference = target_from_reference.translation();
      stereo.baseline = stereo.t_target_reference.norm();
      Eigen::Matrix3d tx;
      const auto& t = stereo.t_target_reference;
      tx << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
      stereo.F_target_reference = right_camera.K.inverse().transpose() * tx *
          stereo.R_target_reference * left_camera.K.inverse();
      config_.cameras = {std::move(left_camera), std::move(right_camera)};
      config_.stereo = stereo;
    }
    const auto left = euroc_csv(base / "cam0/data.csv", base / "cam0/data");
    const auto right = euroc_csv(base / "cam1/data.csv", base / "cam1/data");
    const Eigen::Isometry3d body_from_camera = config_.cameras.empty()
        ? Eigen::Isometry3d(Eigen::Isometry3d::Identity())
        : config_.cameras[0].T_body_camera;
    const auto ground_truth = euroc_ground_truth(
        base / "state_groundtruth_estimate0/data.csv", body_from_camera);
    std::size_t right_index = 0;
    for (const auto& left_sample : left) {
      while (right_index + 1 < right.size() &&
             std::llabs(right[right_index + 1].timestamp - left_sample.timestamp) <
                 std::llabs(right[right_index].timestamp - left_sample.timestamp))
        ++right_index;
      if (right_index >= right.size() ||
          std::llabs(right[right_index].timestamp - left_sample.timestamp) >
              config_.sync_tolerance_ns)
        continue;
      FrameSet set;
      set.index = sets_.size();
      set.timestamp_ns = left_sample.timestamp;
      set.frames.push_back(make_frame("cam0", left_sample.timestamp,
          left_sample.path, config_.cameras.size() > 0 ? &config_.cameras[0] : nullptr));
      set.frames.push_back(make_frame("cam1", right[right_index].timestamp,
          right[right_index].path,
          config_.cameras.size() > 1 ? &config_.cameras[1] : nullptr));
      const auto pose = closest_pose(ground_truth, left_sample.timestamp);
      set.frames[0].ground_truth = pose;
      set.frames[1].ground_truth = pose;
      sets_.push_back(std::move(set));
    }
  }

  void load_eth3d() {
    const fs::path base = config_.root / config_.sequence;
    const fs::path stereo_pairs = base / "stereo_pairs";
    if (fs::exists(stereo_pairs)) {
      std::vector<fs::path> folders;
      for (const auto& entry : fs::directory_iterator(stereo_pairs))
        if (entry.is_directory()) folders.push_back(entry.path());
      std::sort(folders.begin(), folders.end());
      int num_cameras = 2;
      auto option = config_.options.find("num_cameras");
      if (option != config_.options.end()) num_cameras = std::stoi(option->second);
      const std::size_t folders_per_set =
          static_cast<std::size_t>((num_cameras + 1) / 2);
      for (std::size_t start = 0; start + folders_per_set <= folders.size(); ++start) {
        const Timestamp timestamp =
            seconds_to_ns(static_cast<double>(sets_.size()));
        FrameSet set;
        set.index = sets_.size();
        set.timestamp_ns = timestamp;
        bool valid = true;
        for (int camera_index = 0; camera_index < num_cameras; ++camera_index) {
          const auto& folder = folders[start + camera_index / 2];
          const bool right_camera = camera_index % 2 == 1;
          const auto image = folder / (right_camera ? "im1.png" : "im0.png");
          if (!fs::exists(image)) { valid = false; break; }
          const auto* calibration = config_.cameras.empty()
              ? nullptr
              : &config_.cameras[std::min<std::size_t>(camera_index % 2,
                                                       config_.cameras.size() - 1)];
          auto frame = make_frame("cam" + std::to_string(camera_index),
                                  timestamp, image, calibration);
          frame.ground_truth = eth3d_pose(
              folder / "images.txt", right_camera ? "im1.png" : "im0.png",
              timestamp);
          const auto disparity = folder /
              (right_camera ? "disp1GT.pfm" : "disp0GT.pfm");
          const auto mask = folder /
              (right_camera ? "mask1nocc.png" : "mask0nocc.png");
          if (fs::exists(disparity)) frame.metadata.disparity_path = disparity;
          if (fs::exists(mask)) frame.metadata.occlusion_mask_path = mask;
          set.frames.push_back(std::move(frame));
        }
        if (valid) sets_.push_back(std::move(set));
      }
      return;
    }

    const auto left = images_in(base / "rgb");
    const auto right = images_in(base / "rgb2");
    for (std::size_t i = 0; i < std::min(left.size(), right.size()); ++i) {
      FrameSet set;
      set.index = i;
      set.timestamp_ns = seconds_to_ns(static_cast<double>(i));
      set.frames = {make_frame("cam0", set.timestamp_ns, left[i]),
                    make_frame("cam1", set.timestamp_ns, right[i])};
      sets_.push_back(std::move(set));
    }
  }

  void load() {
    auto type = config_.type;
    std::transform(type.begin(), type.end(), type.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (type.find("kitti") != std::string::npos) load_kitti();
    else if (type.find("euroc") != std::string::npos) load_euroc();
    else if (type.find("eth3d") != std::string::npos) load_eth3d();
    else throw std::invalid_argument("unsupported dataset type: " + config_.type);
    if (sets_.empty())
      throw std::runtime_error("no synchronized frames found under " +
                               config_.root.string());
  }

  DatasetConfig config_;
  std::vector<FrameSet> sets_;
};

} // namespace

std::unique_ptr<Dataset> open_dataset(DatasetConfig config) {
  return std::make_unique<BuiltinDataset>(std::move(config));
}

} // namespace lems::data
