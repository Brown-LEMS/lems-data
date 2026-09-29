#include "lems/data/dataset.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <system_error>

namespace {

bool require(bool condition, const char* message) {
  if (condition) return true;
  std::cerr << "dataset_test: " << message << '\n';
  return false;
}

}  // namespace

int main() {
  namespace fs = std::filesystem;
  std::error_code create_error;
  const auto token = std::chrono::steady_clock::now().time_since_epoch().count();
  fs::path root;
  for (unsigned attempt = 0; attempt < 100 && root.empty(); ++attempt) {
    const auto candidate = fs::temp_directory_path() /
                          ("lems_data_test_" + std::to_string(token) + "_" +
                           std::to_string(attempt));
    if (fs::create_directory(candidate, create_error)) {
      root = candidate;
      break;
    }
    if (create_error && create_error != std::errc::file_exists) break;
    create_error.clear();
  }
  if (root.empty()) {
    std::cerr << "dataset_test: could not create an owned temporary directory\n";
    return 1;
  }
  const auto cleanup = [&]() {
    std::error_code error;
    fs::remove_all(root, error);
  };

  bool ok = true;
  try {
    fs::create_directories(root / "sequences/00/image_0");
    fs::create_directories(root / "sequences/00/image_1");
    cv::Mat image(2, 3, CV_8U, cv::Scalar(127));
    ok &= require(
        cv::imwrite((root / "sequences/00/image_0/000000.png").string(),
                    image),
        "could not write left fixture image");
    ok &= require(
        cv::imwrite((root / "sequences/00/image_1/000000.png").string(),
                    image),
        "could not write right fixture image");
    std::ofstream times(root / "sequences/00/times.txt");
    times << "0.5\n";
    times.flush();
    ok &= require(static_cast<bool>(times), "could not write timestamp fixture");
    times.close();

    const auto yaml = root / "test.yaml";
    std::ofstream config_file(yaml);
    config_file
        << "dataset_type: KITTI\n"
        << "dataset_dir: .\n"
        << "sequence_name: 00\n"
        << "left_camera:\n"
        << "  resolution: [3, 2]\n"
        << "  intrinsics: [718.856, 718.856, 607.1928, 185.2157]\n"
        << "  distortion_coefficients: [0, 0, 0, 0]\n"
        << "right_camera:\n"
        << "  resolution: [3, 2]\n"
        << "  intrinsics: [718.856, 718.856, 607.1928, 185.2157]\n"
        << "  distortion_coefficients: [0, 0, 0, 0]\n"
        << "stereo:\n"
        << "  R21:\n"
        << "  - [1, 0, 0]\n"
        << "  - [0, 1, 0]\n"
        << "  - [0, 0, 1]\n"
        << "  T21: [-0.54, 0, 0]\n";
    config_file.flush();
    ok &= require(static_cast<bool>(config_file), "could not write config fixture");
    config_file.close();

    auto config = lems::data::load_config(yaml);
    ok &= require(config.cameras.size() == 2,
                  "KITTI fixture should produce two camera calibrations");
    if (config.cameras.size() >= 2) {
      ok &= require(std::abs(config.cameras[0].K(0, 0) - 718.856) < 1e-9,
                    "left focal length was not loaded");
    }
    ok &= require(config.stereo.has_value(),
                  "KITTI fixture should produce stereo calibration");
    if (config.stereo) {
      ok &= require(std::abs(config.stereo->baseline - 0.54) < 1e-9,
                    "stereo baseline was not loaded");
    }

    auto dataset = lems::data::open_dataset(config);
    auto frame_set = dataset->iterate()->next();
    ok &= require(frame_set.has_value(), "dataset should yield one frame set");
    if (frame_set) {
      ok &= require(frame_set->frames.size() == 2,
                    "frame set should contain the stereo pair");
      ok &= require(frame_set->timestamp_ns == 500000000,
                    "frame-set timestamp should be 0.5 seconds in ns");
      if (!frame_set->frames.empty()) {
        ok &= require(frame_set->frames[0].image.rows == 2,
                      "left image should retain fixture height");
        if (!config.cameras.empty()) {
          ok &= require(frame_set->frames[0].K.isApprox(config.cameras[0].K),
                        "frame K should match left camera calibration");
        }
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "dataset_test: unexpected exception: " << error.what() << '\n';
    ok = false;
  }
  cleanup();
  return ok ? 0 : 1;
}
