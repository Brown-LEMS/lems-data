#define PATCH_SIZE 7
#define ORTHOGONAL_SHIFT_MAG 5

#include "Dataset.h"
#include "Stereo_Iterator.h"
#include "Multinocular_Iterator.h"
#include "utility.h"

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<Frame, lems::data::Frame>);
static_assert(std::is_same_v<Camera_Pose, lems::data::CameraPose>);
static_assert(std::is_same_v<Edge, lems::data::Edge>);
static_assert(std::is_same_v<Edge_3D, lems::data::Edge_3D>);

static_assert(std::is_same_v<decltype(std::declval<Dataset>().get_left_calib_matrix()),
                             Eigen::Matrix3d>);
static_assert(std::is_same_v<decltype(std::declval<Dataset>().get_fund_mat_21()),
                             Eigen::Matrix3d>);
static_assert(std::is_same_v<decltype(std::declval<Dataset>().get_relative_transl_left_to_right()),
                             Eigen::Vector3d>);

namespace {

bool require(bool condition, const char* message) {
  if (condition) return true;
  std::cerr << "compat_test: " << message << '\n';
  return false;
}

bool test_num_imgs_counter() {
  namespace fs = std::filesystem;
  const auto token = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = fs::temp_directory_path() /
                    ("lems_vo_compat_counter_" + std::to_string(token));
  std::error_code error;
  fs::create_directories(root / "sequences/00/image_0", error);
  fs::create_directories(root / "sequences/00/image_1", error);
  if (error) {
    std::cerr << "compat_test: could not create counter fixture: "
              << error.message() << '\n';
    return false;
  }

  bool ok = true;
  try {
    const cv::Mat image = cv::Mat::zeros(1, 1, CV_8U);
    ok &= require(cv::imwrite(
                      (root / "sequences/00/image_0/000000.png").string(),
                      image),
                  "could not write counter fixture left image");
    ok &= require(cv::imwrite(
                      (root / "sequences/00/image_1/000000.png").string(),
                      image),
                  "could not write counter fixture right image");
    std::ofstream times(root / "sequences/00/times.txt");
    times << "0\n";
    times.close();
    std::ofstream calibration(root / "sequences/00/calib.txt");
    calibration << "P0: 1 0 0 0 0 1 0 0 0 0 1 0\n"
                << "P1: 1 0 0 -1 0 1 0 0 0 0 1 0\n";
    calibration.close();

    YAML::Node node;
    node["dataset_type"] = "KITTI";
    node["dataset_dir"] = root.string();
    node["sequence_name"] = "00";
    Dataset dataset(node);
    ok &= require(dataset.size() == 1,
                  "counter fixture should contain one dataset frame");
    ok &= require(dataset.get_num_imgs() == 0,
                  "processed-image counter must start at zero");
    dataset.increment_num_imgs();
    ok &= require(dataset.get_num_imgs() == 1,
                  "processed-image counter must increment independently of size");

    const auto malformed_pfm = root / "malformed.pfm";
    std::ofstream malformed(malformed_pfm, std::ios::binary);
    malformed << "Pf\nnot-a-dimension\n";
    malformed.close();
    bool rejected = false;
    try {
      (void)dataset.readPFM(malformed_pfm.string());
    } catch (const std::exception&) {
      rejected = true;
    }
    ok &= require(rejected, "malformed PFM headers must be rejected safely");
  } catch (const std::exception& error) {
    std::cerr << "compat_test: counter fixture failed: " << error.what() << '\n';
    ok = false;
  }
  fs::remove_all(root, error);
  return ok;
}

}  // namespace

int main() {
  StereoFrame stereo;
  Frame multinocular;
  bool ok = true;
  const UtilityOptions utility_options;
  ok &= require(utility_options.patch_size == 7,
                "UtilityOptions should keep its canonical patch-size default");
  ok &= require(std::abs(utility_options.orthogonal_shift - 5.0) < 1e-12,
                "UtilityOptions should keep its canonical shift default");
  ok &= require(stereo.left_image.empty(),
                "default StereoFrame should have no left image");
  ok &= require(multinocular.image.empty(),
                "canonical Frame alias should have no image by default");
  ok &= require(multinocular.timestamp == 0.0,
                "canonical Frame legacy timestamp should default to zero");

  const Eigen::Vector3d translation(1.0, 2.0, 3.0);
  const Camera_Pose pose(Eigen::Matrix3d::Identity(), translation);
  ok &= require(pose.center().isApprox(-translation),
                "Camera_Pose center should be -R^T t");

  multinocular.timestamp_seconds = 1.25;
  multinocular.ground_truth = pose;
  multinocular.metadata.disparity = cv::Mat::ones(1, 1, CV_32F);
  multinocular.synchronize_legacy_fields();
  ok &= require(std::abs(multinocular.timestamp - 1.25) < 1e-12,
                "Frame synchronization should copy timestamp_seconds");
  ok &= require(multinocular.has_ground_truth,
                "Frame synchronization should expose ground truth presence");
  ok &= require(multinocular.gt_camera_pose.center().isApprox(-translation),
                "Frame synchronization should copy the canonical Camera_Pose");
  ok &= require(!multinocular.disparity_map.empty(),
                "Frame synchronization should copy disparity metadata");

  multinocular.ground_truth.reset();
  multinocular.metadata.disparity.release();
  multinocular.synchronize_legacy_fields();
  ok &= require(!multinocular.has_ground_truth,
                "Frame synchronization should clear absent ground truth");
  ok &= require(multinocular.gt_camera_pose.center().isApprox(Eigen::Vector3d::Zero()),
                "Frame synchronization should reset absent pose");
  ok &= require(multinocular.disparity_map.empty(),
                "Frame synchronization should clear absent disparity");
  ok &= test_num_imgs_counter();
  return ok ? 0 : 1;
}
