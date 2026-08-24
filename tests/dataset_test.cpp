#include "lems/data/dataset.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <filesystem>
#include <fstream>

int main() {
  namespace fs = std::filesystem;
  const auto root = fs::temp_directory_path() / "lems_data_test";
  fs::remove_all(root);
  fs::create_directories(root / "sequences/00/image_0");
  fs::create_directories(root / "sequences/00/image_1");
  cv::Mat image(2, 3, CV_8U, cv::Scalar(127));
  cv::imwrite((root / "sequences/00/image_0/000000.png").string(), image);
  cv::imwrite((root / "sequences/00/image_1/000000.png").string(), image);
  std::ofstream(root / "sequences/00/times.txt") << "0.5\n";

  const auto yaml = root / "test.yaml";
  std::ofstream(yaml)
      << "dataset_type: KITTI\n"
      << "dataset_dir: .\n"
      << "sequence_name: 00\n"
      << "left_camera:\n"
      << "  resolution: [1241, 376]\n"
      << "  intrinsics: [718.856, 718.856, 607.1928, 185.2157]\n"
      << "  distortion_coefficients: [0, 0, 0, 0]\n"
      << "right_camera:\n"
      << "  resolution: [1241, 376]\n"
      << "  intrinsics: [718.856, 718.856, 607.1928, 185.2157]\n"
      << "  distortion_coefficients: [0, 0, 0, 0]\n"
      << "stereo:\n"
      << "  R21:\n"
      << "  - [1, 0, 0]\n"
      << "  - [0, 1, 0]\n"
      << "  - [0, 0, 1]\n"
      << "  T21: [-0.54, 0, 0]\n";

  auto config = lems::data::load_config(yaml);
  assert(config.cameras.size() == 2);
  assert(config.cameras[0].K(0, 0) == 718.856);
  assert(config.stereo && config.stereo->baseline == 0.54);
  auto dataset = lems::data::open_dataset(config);
  auto frame_set = dataset->iterate()->next();
  assert(frame_set && frame_set->frames.size() == 2);
  assert(frame_set->timestamp_ns == 500000000);
  assert(frame_set->frames[0].image.rows == 2);
  assert(frame_set->frames[0].K.isApprox(config.cameras[0].K));
  fs::remove_all(root);
}
