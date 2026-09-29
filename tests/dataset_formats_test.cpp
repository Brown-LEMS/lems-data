#include "lems/data/dataset.hpp"

#include <opencv2/imgcodecs.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using lems::data::CameraPose;
using lems::data::DatasetConfig;

namespace {

struct TestState {
  int failures{};
  void check(bool condition, const std::string& message) {
    if (!condition) {
      ++failures;
      std::cerr << "FAIL: " << message << "\n";
    }
  }
};

void make_directory(const fs::path& path) { fs::create_directories(path); }

void write_image(const fs::path& path, int value = 127, int width = 4,
                 int height = 3) {
  make_directory(path.parent_path());
  cv::Mat image(height, width, CV_8UC1, cv::Scalar(value));
  if (!cv::imwrite(path.string(), image))
    throw std::runtime_error("failed to write test image: " + path.string());
}

void write_matrix(std::ofstream& output, const Eigen::Matrix4d& matrix) {
  output << "  data: [";
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      output << matrix(row, col) << ((row == 3 && col == 3) ? "" : ", ");
  output << "]\n";
}

bool little_endian_host() {
  const std::uint16_t value = 1;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 1;
}

void write_float(std::ofstream& output, float value, bool little_endian_file) {
  std::array<std::uint8_t, sizeof(float)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(value));
  if (little_endian_file != little_endian_host())
    std::reverse(bytes.begin(), bytes.end());
  output.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
}

void write_pfm(const fs::path& path, bool little_endian_file, float scale,
               const std::vector<std::vector<float>>& top_down) {
  if (top_down.empty() || top_down.front().empty())
    throw std::runtime_error("empty PFM test data");
  const int height = static_cast<int>(top_down.size());
  const int width = static_cast<int>(top_down.front().size());
  std::ofstream output(path, std::ios::binary);
  output << "Pf\n" << width << " " << height << "\n"
         << (little_endian_file ? -scale : scale) << "\n";
  for (int row = height - 1; row >= 0; --row)
    for (int col = 0; col < width; ++col)
      write_float(output, top_down[static_cast<std::size_t>(row)]
                              [static_cast<std::size_t>(col)] /
                          scale,
                  little_endian_file);
}

void write_mask(const fs::path& path) {
  make_directory(path.parent_path());
  cv::Mat mask = (cv::Mat_<std::uint8_t>(2, 2) << 0, 255, 255, 0);
  if (!cv::imwrite(path.string(), mask))
    throw std::runtime_error("failed to write test mask");
}

Eigen::Matrix3d rx(double angle) {
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d ry(double angle) {
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

Eigen::Matrix3d rz(double angle) {
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix4d rigid(const Eigen::Matrix3d& rotation,
                     const Eigen::Vector3d& translation) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.topLeftCorner<3, 3>() = rotation;
  result.topRightCorner<3, 1>() = translation;
  return result;
}

void write_kitti(TestState& state, const fs::path& root) {
  const fs::path sequence = root / "sequences/00";
  make_directory(sequence / "image_2");
  make_directory(sequence / "image_3");
  for (int index = 0; index < 3; ++index) {
    std::ostringstream name;
    name << std::setfill('0') << std::setw(6) << index << ".png";
    write_image(sequence / "image_2" / name.str(), 40 + index);
    write_image(sequence / "image_3" / name.str(), 80 + index);
  }
  {
    std::ofstream times(sequence / "times.txt");
    times << "1.25\n2.50\n3.75\n";
  }
  const Eigen::Matrix<double, 3, 4> p2 =
      (Eigen::Matrix<double, 3, 4>() << 100.0, 2.0, 50.0, 8.0,
       0.0, 110.0, 40.0, -4.0, 0.0, 0.0, 1.0, 0.5)
          .finished();
  const Eigen::Matrix<double, 3, 4> p3 =
      (Eigen::Matrix<double, 3, 4>() << 102.0, 3.0, 51.0, 15.0,
       0.0, 111.0, 41.0, -2.0, 0.0, 0.0, 1.0, 0.25)
          .finished();
  std::ofstream calibration(sequence / "calib.txt");
  calibration << "P0: 100 0 50 0 0 110 40 0 0 0 1 0\n"
              << "P1: 100 0 50 -5 0 110 40 0 0 0 1 0\n";
  for (int camera = 0; camera < 2; ++camera) {
    const auto& projection = camera == 0 ? p2 : p3;
    calibration << (camera == 0 ? "P2: " : "P3: ");
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 4; ++col)
        calibration << projection(row, col)
                    << ((row == 2 && col == 3) ? '\n' : ' ');
  }
  calibration.close();
  make_directory(root / "poses");
  std::ofstream poses(root / "poses/00.txt");
  poses << "1 0 0 0 0 1 0 0 0 0 1 0\n"
        << "1 0 0 1 0 1 0 2 0 0 1 3\n"
        << "1 0 0 2 0 1 0 4 0 0 1 6\n";
  poses.close();
  std::ofstream config_file(root / "config.yaml");
  config_file << "dataset_type: KITTI\n"
              << "dataset_dir: .\nsequence_name: 00\n"
              << "num_cameras: 3\nskip_frames: 1\n";
  config_file.close();

  auto config = lems::data::load_config(root / "config.yaml");
  auto dataset = lems::data::open_dataset(config);
  state.check(dataset->cameras().size() == 2, "KITTI native cameras parsed");
  state.check(std::abs(dataset->cameras()[0].K(0, 1) - 2.0) < 1e-12,
              "KITTI projection preserves skew");
  const Eigen::Vector3d expected_offset =
      dataset->cameras()[1].K.inverse() * dataset->cameras()[1].P.col(3) -
      dataset->cameras()[0].K.inverse() * dataset->cameras()[0].P.col(3);
  state.check(dataset->stereo_calibration()->t_target_reference.isApprox(
                  expected_offset, 1e-12),
              "KITTI color baseline uses full K inverse projection offset");
  state.check(dataset->size() == 2, "KITTI complete temporal windows only");
  auto iterator = dataset->iterate();
  state.check(iterator->has_next() && iterator->size() == 1,
              "KITTI skip_frames advances window starts");
  const auto first = iterator->next();
  state.check(first && first->frames.size() == 3,
              "KITTI odd temporal window is left/right/left");
  state.check(first && first->timestamp_ns == 1'250'000'000,
              "KITTI times.txt scalar timestamps are seconds");
  state.check(first && first->frames[0].ground_truth &&
                  first->frames[0].ground_truth->t.isApprox(
                      dataset->cameras()[0].K.inverse() *
                          dataset->cameras()[0].P.col(3),
                      1e-12),
              "KITTI color camera applies P2/P0 world-frame offset");
  state.check(first && first->frames[0].image.rows == 3,
              "KITTI iterator materializes images");
  state.check(!iterator->has_next(), "KITTI skip omits incomplete/next start");
  iterator->reset();
  state.check(iterator->has_next(), "KITTI iterator reset");
  fs::remove_all(root);
}

void write_euroc(TestState& state, const fs::path& root) {
  const fs::path base = root / "mav0";
  const fs::path cam0 = base / "cam0";
  const fs::path cam1 = base / "cam1";
  for (const auto& camera : {cam0, cam1}) make_directory(camera / "data");
  write_image(cam0 / "data/0.png", 30);
  write_image(cam0 / "data/1.png", 31);
  write_image(cam1 / "data/0.png", 60);
  write_image(cam1 / "data/1.png", 61);
  const Eigen::Matrix4d body_from_cam0 = rigid(rz(0.5), {1.0, 0.0, 0.0});
  const Eigen::Matrix4d body_from_cam1 = rigid(ry(-0.4), {0.0, 2.0, 0.0});
  for (const auto& item : std::array<std::pair<fs::path, Eigen::Matrix4d>, 2>{
           std::make_pair(cam0 / "sensor.yaml", body_from_cam0),
           std::make_pair(cam1 / "sensor.yaml", body_from_cam1)}) {
    std::ofstream sensor(item.first);
    sensor << std::setprecision(17);
    sensor << "camera_model: pinhole\nresolution: [4, 3]\n"
           << "intrinsics: [100, 101, 2, 1]\n"
           << "distortion_coefficients: [0, 0, 0, 0]\nT_BS:\n";
    write_matrix(sensor, item.second);
  }
  std::ofstream(cam0 / "data.csv") << "1000000000,0.png\n2000000000,1.png\n";
  std::ofstream(cam1 / "data.csv") << "1000000001,0.png\n2000000001,1.png\n";
  const Eigen::Matrix4d world_from_body =
      rigid(rx(0.6), {1.0, 2.0, 3.0});
  const Eigen::Quaterniond q(world_from_body.topLeftCorner<3, 3>());
  make_directory((base / "state_groundtruth_estimate0"));
  std::ofstream ground_truth(base / "state_groundtruth_estimate0/data.csv");
  ground_truth << std::setprecision(17);
  ground_truth << "1000000000 " << world_from_body(0, 3) << " "
               << world_from_body(1, 3) << " " << world_from_body(2, 3)
               << " " << q.w() << " " << q.x() << " " << q.y() << " "
               << q.z() << "\n";
  ground_truth << "2000000000 " << world_from_body(0, 3) << " "
               << world_from_body(1, 3) << " " << world_from_body(2, 3)
               << " " << q.w() << " " << q.x() << " " << q.y() << " "
               << q.z() << "\n";
  ground_truth.close();
  std::ofstream config_file(root / "config.yaml");
  config_file << "dataset_type: EuRoC\ndataset_dir: .\nsequence_name: ''\n"
              << "sync_tolerance_ns: 10\nnum_cameras: 3\n";
  config_file.close();
  auto config = lems::data::load_config(root / "config.yaml");
  auto dataset = lems::data::open_dataset(config);
  state.check(dataset->size() == 1, "EuRoC complete temporal window count");
  state.check(dataset->file_info().Img_time_stamps.size() == 2,
              "EuRoC integer nanosecond image timestamps");
  auto iterator = dataset->iterate();
  const auto set = iterator->next();
  state.check(set && set->frames.size() == 3,
              "EuRoC odd temporal window ordering");
  Eigen::Isometry3d body_from_world = Eigen::Isometry3d::Identity();
  body_from_world.matrix() = world_from_body.inverse();
  Eigen::Isometry3d expected_left = Eigen::Isometry3d::Identity();
  expected_left.matrix() = body_from_cam0.inverse() * body_from_world.matrix();
  state.check(set && set->frames[0].ground_truth &&
                  set->frames[0].ground_truth->matrix().isApprox(
                      expected_left.matrix(), 1e-9),
              "EuRoC noncommuting T_cam_body*T_body_world pose");
  Eigen::Isometry3d expected_right = Eigen::Isometry3d::Identity();
  expected_right.matrix() = body_from_cam1.inverse() * body_from_cam0 *
                           expected_left.matrix();
  state.check(set && set->frames[1].ground_truth &&
                  set->frames[1].ground_truth->matrix().isApprox(
                      expected_right.matrix(), 1e-9),
              "EuRoC right pose left-multiplies camera/body transforms");
  state.check(set && set->frames[1].timestamp_ns == 1'000'000'001,
              "EuRoC right frame keeps its own nanosecond timestamp");
  fs::remove_all(root);
}

void write_eth_calibration(const fs::path& root) {
  make_directory(root);
  std::ofstream(root / "calibration.txt") << "100 101 2 1\n";
  std::ofstream(root / "calibration2.txt") << "102 103 3 1.5\n";
  std::ofstream extrinsics(root / "extrinsics_1_2.txt");
  extrinsics << "0 -1 0 1\n1 0 0 2\n0 0 1 0\n";
  extrinsics.close();
}

void write_eth_pair(TestState& state, const fs::path& root) {
  const fs::path scene = root / "scene";
  write_eth_calibration(scene);
  for (int index = 0; index < 3; ++index) {
    std::ostringstream folder_name;
    folder_name << std::setw(3) << std::setfill('0') << index;
    const fs::path folder = scene / "stereo_pairs" / folder_name.str();
    write_image(folder / "im0.png", 20 + index, 2, 2);
    write_image(folder / "im1.png", 50 + index, 2, 2);
    write_pfm(folder / "disp0GT.pfm", true, 2.0F, {{2.0F, 4.0F}, {6.0F, 8.0F}});
    write_pfm(folder / "disp1GT.pfm", false, 1.0F, {{1.0F, 3.0F}, {5.0F, 7.0F}});
    write_mask(folder / "mask0nocc.png");
    write_mask(folder / "mask1nocc.png");
    std::ofstream poses(folder / "images.txt");
    poses << "1 1 0 0 0 0 0 0 1 im0.png\n"
          << "2 1 0 0 0 1 0 0 1 im1.png\n";
  }
  std::ofstream config_file(root / "config.yaml");
  config_file << "dataset_type: ETH3D_stereo\ndataset_dir: .\n"
              << "sequence_name: scene\nnum_cameras: 3\nskip_frames: 1\n";
  config_file.close();
  auto config = lems::data::load_config(root / "config.yaml");
  auto dataset = lems::data::open_dataset(config);
  state.check(dataset->cameras().size() == 2,
              "ETH3D native calibration cameras");
  state.check(dataset->stereo_calibration() &&
                  std::abs(dataset->stereo_calibration()->baseline -
                           std::sqrt(5.0)) < 1e-9,
              "ETH3D extrinsics_1_2 inverted for R21/T21");
  state.check(dataset->size() == 2, "ETH3D pair temporal windows");
  auto iterator = dataset->iterate();
  const auto set = iterator->next();
  state.check(set && set->frames.size() == 3,
              "ETH3D pair odd window and skip stride");
  state.check(set && set->frames[2].timestamp_ns == 1'000'000'000,
              "ETH3D pair frame timestamp includes temporal offset");
  state.check(set && set->frames[0].ground_truth &&
                  set->frames[0].ground_truth->t.isApprox(
                      Eigen::Vector3d::Zero()),
              "ETH3D COLMAP left world-to-camera pose");
  state.check(set && set->frames[0].image_gradients_x.empty() &&
                  set->frames[0].image_gradients_y.empty(),
              "materialization leaves gradients pipeline-owned");
  state.check(set && set->frames[0].metadata.disparity.type() == CV_32FC1 &&
                  std::abs(set->frames[0].metadata.disparity.at<float>(0, 0) -
                           2.0F) < 1e-6F,
              "ETH3D little-endian scaled PFM and vertical flip");
  state.check(set && set->frames[1].metadata.disparity.type() == CV_32FC1 &&
                  std::abs(set->frames[1].metadata.disparity.at<float>(0, 0) -
                           1.0F) < 1e-6F,
              "ETH3D big-endian PFM");
  state.check(set && set->frames[0].occlusion_mask.at<std::uint8_t>(0, 1) == 255,
              "ETH3D non-occlusion mask retains raw 255 visibility");
  write_pfm(scene / "stereo_pairs/000/disp0GT.pfm", true, 1.0F, {{9.0F}});
  bool rejected_bad_metadata = false;
  try {
    auto bad_iterator = dataset->iterate();
    (void)bad_iterator->next();
  } catch (const std::exception&) {
    rejected_bad_metadata = true;
  }
  state.check(rejected_bad_metadata,
              "ETH3D rejects metadata with image dimension mismatch");
  iterator->reset();
  state.check(iterator->has_next(), "ETH3D iterator reset");
  fs::remove_all(root);
}

void write_eth_native_pair(TestState& state, const fs::path& root) {
  const fs::path scene = root / "scene";
  for (int index = 0; index < 2; ++index) {
    std::ostringstream folder_name;
    folder_name << std::setw(3) << std::setfill('0') << index;
    const fs::path folder = scene / "stereo_pairs" / folder_name.str();
    write_image(folder / "im0.png", 30 + index, 2, 2);
    write_image(folder / "im1.png", 60 + index, 2, 2);
    std::ofstream calibration(folder / "calib.txt");
    const double focal_left = 100.0 + 20.0 * index;
    const double focal_right = 102.0 + 20.0 * index;
    calibration << "cam0 = [" << focal_left << " 0 1; 0 " << focal_left + 1
                << " 1; 0 0 1]\n"
                << "cam1 = [" << focal_right << " 0 1.2; 0 "
                << focal_right + 1 << " 1.2; 0 0 1]\n"
                << "baseline = 120\nwidth = 2\nheight = 2\n";
  }
  std::ofstream config_file(root / "config.yaml");
  config_file << "dataset_type: ETH3D_stereo\ndataset_dir: .\n"
              << "sequence_name: scene\n";
  config_file.close();
  auto config = lems::data::load_config(root / "config.yaml");
  auto dataset = lems::data::open_dataset(config);
  state.check(dataset->cameras().size() == 2,
              "ETH3D pair calib.txt supplies both camera matrices");
  state.check(dataset->stereo_calibration() &&
                  std::abs(dataset->stereo_calibration()->baseline - 0.12) < 1e-12,
              "ETH3D pair calib.txt converts baseline millimeters to meters");
  state.check(dataset->stereo_calibration() &&
                  std::abs(dataset->stereo_calibration()->t_target_reference.x() +
                           0.12) < 1e-12,
              "ETH3D pair baseline points from left to right camera");
  if (dataset->stereo_calibration()) {
    const Eigen::Vector3d point_left(0.0, 0.0, 4.0);
    const auto& left_camera = dataset->cameras()[0];
    const auto& right_camera = dataset->cameras()[1];
    const Eigen::Vector3d point_right =
        dataset->stereo_calibration()->R_target_reference * point_left +
        dataset->stereo_calibration()->t_target_reference;
    const double left_u =
        (left_camera.K * point_left).x() / point_left.z();
    const double right_u =
        (right_camera.K * point_right).x() / point_right.z();
    state.check(left_u > right_u,
                "ETH3D pair calibration gives positive stereo disparity");
  }
  auto iterator = dataset->iterate();
  const auto first = iterator->next();
  const auto second = iterator->next();
  state.check(first && first->frames.size() == 2 &&
                  std::abs(first->frames[0].K(0, 0) - 100.0) < 1e-12,
              "ETH3D pair calib applies first folder K");
  state.check(second && second->frames.size() == 2 &&
                  std::abs(second->frames[0].K(0, 0) - 120.0) < 1e-12,
              "ETH3D pair calib updates K for each folder");
  fs::remove(scene / "stereo_pairs/001/calib.txt");
  bool rejected_missing_calibration = false;
  try {
    (void)lems::data::open_dataset(config);
  } catch (const std::exception&) {
    rejected_missing_calibration = true;
  }
  state.check(rejected_missing_calibration,
              "ETH3D requires calibration in every native pair folder");
  fs::remove_all(root);
}

void write_eth_slam(TestState& state, const fs::path& root) {
  const fs::path scene = root / "scene";
  write_eth_calibration(scene);
  write_image(scene / "rgb/a.png", 70, 2, 2);
  write_image(scene / "rgb/b.png", 71, 2, 2);
  write_image(scene / "rgb2/a.png", 90, 2, 2);
  write_image(scene / "rgb2/b.png", 91, 2, 2);
  std::ofstream(scene / "rgb.txt") << "10.0 rgb/a.png\n11.0 rgb/b.png\n";
  std::ofstream ground_truth(scene / "groundtruth.txt");
  ground_truth << "10.0 1 0 0 0 0 0 1\n11.0 1 0 0 0 0 0 1\n";
  ground_truth.close();
  std::ofstream config_file(root / "config.yaml");
  config_file << "dataset_type: ETH3D_slam\ndataset_dir: .\n"
              << "sequence_name: scene\n";
  config_file.close();
  auto config = lems::data::load_config(root / "config.yaml");
  auto dataset = lems::data::open_dataset(config);
  auto iterator = dataset->iterate();
  const auto set = iterator->next();
  state.check(set && set->frames.size() == 2,
              "ETH3D SLAM rgb/rgb2 fallback pairing");
  state.check(set && set->frames[0].timestamp_ns == 10'000'000'000,
              "ETH3D SLAM seconds timestamp conversion");
  const Eigen::Vector3d left_translation =
      set && set->frames[0].ground_truth
          ? set->frames[0].ground_truth->t
          : Eigen::Vector3d::Constant(999.0);
  const Eigen::Vector3d right_translation =
      set && set->frames[1].ground_truth
          ? set->frames[1].ground_truth->t
          : Eigen::Vector3d::Constant(999.0);
  state.check(left_translation.isApprox(Eigen::Vector3d(-1, 0, 0), 1e-9),
              "ETH3D SLAM TUM camera-to-world inversion");
  state.check(!right_translation.isApprox(left_translation, 1e-9),
              "ETH3D SLAM right pose applies stereo R21/T21");
  fs::remove_all(root);
}

void test_config_validation(TestState& state) {
  try {
    lems::data::load_config(YAML::Load(
        "dataset_type: KITTI\ndataset_dir: /tmp\nskip_frames: -1\n"));
    state.check(false, "negative skip_frames rejected before unsigned conversion");
  } catch (const std::exception&) {
    state.check(true, "negative skip_frames rejected before unsigned conversion");
  }
}

}  // namespace

int main() {
  TestState state;
  const auto token = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path root = fs::temp_directory_path() /
                        ("lems_data_formats_test_" + std::to_string(token));
  try {
    write_kitti(state, root / "kitti");
    write_euroc(state, root / "euroc");
    write_eth_pair(state, root / "eth_pair");
    write_eth_native_pair(state, root / "eth_native_pair");
    write_eth_slam(state, root / "eth_slam");
    test_config_validation(state);
  } catch (const std::exception& error) {
    ++state.failures;
    std::cerr << "UNEXPECTED: " << error.what() << "\n";
  }
  fs::remove_all(root);
  if (state.failures != 0)
    std::cerr << state.failures << " dataset format checks failed\n";
  return state.failures == 0 ? 0 : 1;
}
