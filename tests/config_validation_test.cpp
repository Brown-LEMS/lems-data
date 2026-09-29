#include "lems/data/config.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <utility>

namespace {

bool require(bool condition, const std::string& message) {
  if (condition) return true;
  std::cerr << "config_validation_test: " << message << '\n';
  return false;
}

std::string config_yaml(const std::string& camera_fields,
                        const std::string& stereo_fields = {}) {
  std::string yaml =
      "dataset_type: KITTI\n"
      "dataset_dir: /tmp/lems-data-config-validation\n"
      "sequence_name: 00\n"
      "left_camera:\n" +
      camera_fields;
  if (!stereo_fields.empty()) yaml += "stereo:\n" + stereo_fields;
  return yaml;
}

bool accepts(const std::string& label, const std::string& yaml,
             lems::data::DatasetConfig* output = nullptr) {
  try {
    auto config = lems::data::load_config(YAML::Load(yaml));
    if (output) *output = std::move(config);
    return true;
  } catch (const std::exception& error) {
    std::cerr << "config_validation_test: " << label
              << " unexpectedly rejected: " << error.what() << '\n';
    return false;
  }
}

bool rejects(const std::string& label, const std::string& yaml) {
  try {
    (void)lems::data::load_config(YAML::Load(yaml));
  } catch (const std::exception&) {
    return true;
  }
  std::cerr << "config_validation_test: " << label
            << " unexpectedly accepted\n";
  return false;
}

}  // namespace

int main() {
  bool ok = true;

  const std::string skewed_camera =
      "  resolution: [640, 480]\n"
      "  K: [700, 12, 320, 0, 710, 240, 0, 0, 1]\n"
      // Explicit K takes precedence, but must not lose its skew term.
      "  intrinsics: [700, 710, 321, 241]\n"
      "  distortion_coefficients: [0, 0, 0, 0]\n";
  const std::string rotated_stereo =
      "  R21: [0, -1, 0, 1, 0, 0, 0, 0, 1]\n"
      "  T21: [0.1, 0, 0]\n";
  lems::data::DatasetConfig valid_config;
  ok &= require(
      accepts("skewed K", config_yaml(skewed_camera, rotated_stereo),
               &valid_config),
      "valid skewed K and rotated stereo should parse");
  if (valid_config.cameras.size() == 1) {
    const auto& camera = valid_config.cameras.front();
    const Eigen::Matrix3d expected_K =
        (Eigen::Matrix3d() << 700, 12, 320, 0, 710, 240, 0, 0, 1).finished();
    ok &= require(camera.K.isApprox(expected_K),
                  "explicit K must preserve its skew term and full matrix");
    ok &= require(std::abs(camera.intrinsics[2] - 320.0) < 1e-12 &&
                      std::abs(camera.intrinsics[3] - 240.0) < 1e-12,
                  "intrinsics metadata must follow explicit K");
  } else {
    ok &= require(false, "valid config should contain one camera");
  }
  if (valid_config.stereo) {
    const Eigen::Matrix3d expected_R =
        (Eigen::Matrix3d() << 0, -1, 0, 1, 0, 0, 0, 0, 1).finished();
    ok &= require(valid_config.stereo->R_target_reference.isApprox(expected_R),
                  "valid rotated stereo R21 must be retained");
    ok &= require(std::abs(valid_config.stereo->baseline - 0.1) < 1e-12,
                  "stereo baseline must be derived from T21");
  } else {
    ok &= require(false, "valid rotated stereo should be present");
  }

  const std::string intrinsics_only =
      "  resolution: [640, 480]\n"
      "  intrinsics: [700, 710, 320, 240]\n"
      "  distortion_coefficients: null\n"
      "stereo: null\n";
  ok &= require(accepts("missing K", config_yaml(intrinsics_only)),
                "missing optional K should fall back to intrinsics");

  const std::string null_K =
      "  resolution: [640, 480]\n"
      "  K: null\n"
      "  intrinsics: [700, 710, 320, 240]\n"
      "  distortion_coefficients: null\n";
  ok &= require(accepts("null K", config_yaml(null_K)),
                "null optional K should fall back to intrinsics");

  const std::string valid_intrinsics =
      "  resolution: [640, 480]\n"
      "  intrinsics: [700, 710, 320, 240]\n";
  ok &= require(
      rejects("singular K", config_yaml(
                                  "  resolution: [640, 480]\n"
                                  "  K: [1, 0, 0, 0, 0, 0, 0, 0, 1]\n")),
      "singular K must be rejected");
  ok &= require(
      rejects("nonpositive focal length", config_yaml(
                                  "  resolution: [640, 480]\n"
                                  "  K: [0, 0, 320, 0, 710, 240, 0, 0, 1]\n")),
      "nonpositive focal length must be rejected");
  ok &= require(
      rejects("nonpositive resolution", config_yaml(
                                  "  resolution: [0, 0]\n" +
                                  valid_intrinsics)),
      "nonpositive resolution must be rejected");
  ok &= require(
      rejects("invalid stereo rotation", config_yaml(
                                  valid_intrinsics,
                                  "  R21: [2, 0, 0, 0, 1, 0, 0, 0, 1]\n"
                                  "  T21: [0.1, 0, 0]\n")),
      "non-rigid stereo R21 must be rejected");

  return ok ? 0 : 1;
}
