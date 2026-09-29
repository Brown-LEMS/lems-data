#include "lems/data/utility.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {

bool close(double lhs, double rhs, double tolerance = 1e-6) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool require(bool condition, const char* message) {
  if (!condition) std::cerr << "utility_test: " << message << '\n';
  return condition;
}

}  // namespace

int main() {
  using namespace lems::data;
  bool ok = true;
  Utility utility;

  const Eigen::Vector3d vector(1.0, 2.0, 3.0);
  ok &= require((utility.get_Skew_Symmetric_Matrix(vector) * vector).isApprox(
                    Eigen::Vector3d::Zero()),
                "skew matrix should annihilate its defining vector");

  double projected_x = 0.0;
  double projected_y = 0.0;
  const double normal_distance = utility.getNormalDistance2EpipolarLine(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(3.0, 4.0, 1.0),
      projected_x, projected_y);
  ok &= require(close(normal_distance, 3.0) && close(projected_x, 0.0) &&
                    close(projected_y, 4.0),
                "normal epipolar distance should project to the line");

  Eigen::VectorXd packed_edges(6);
  packed_edges << 1.0, 2.0, 0.1, 3.0, 4.0, 0.2;
  const double packed_distance = utility.getNormalDistance2EpipolarLine(
      Eigen::Vector3d(1.0, 0.0, 0.0), packed_edges, 1, projected_x,
      projected_y);
  ok &= require(close(packed_distance, 3.0) && close(projected_x, 0.0) &&
                    close(projected_y, 4.0),
                "packed edge distance must use the second [x,y,theta] triple");
  Eigen::VectorXd malformed_edges(4);
  malformed_edges << 1.0, 2.0, 0.1, 3.0;
  ok &= require(std::isnan(utility.getNormalDistance2EpipolarLine(
                               Eigen::Vector3d(1.0, 0.0, 0.0), malformed_edges,
                               0, projected_x, projected_y)),
                "malformed packed edge lengths must be rejected");

  double intersection_x = 0.0;
  double intersection_y = 0.0;
  const double tangential_distance = utility.getTangentialDistance2EpipolarLine(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(3.0, 4.0, 0.0),
      intersection_x, intersection_y);
  ok &= require(close(tangential_distance, 3.0) && close(intersection_x, 0.0) &&
                    close(intersection_y, 4.0),
                "tangential epipolar distance should handle horizontal edges");

  const auto shifted = utility.get_Orthogonal_Shifted_Points(
      Edge({16.0, 16.0}, 0.0, false, 0, 1));
  ok &= require(close(shifted.first.x, 16.0) && close(shifted.first.y, 11.0) &&
                    close(shifted.second.x, 16.0) && close(shifted.second.y, 21.0),
                "orthogonal patch points must preserve plus/minus ordering");

  cv::Mat image(32, 32, CV_32F);
  for (int y = 0; y < image.rows; ++y)
    for (int x = 0; x < image.cols; ++x)
      image.at<float>(y, x) = static_cast<float>(x + 2 * y);
  ok &= require(close(Bilinear_Interpolation<float>(image, {3.5, 4.5}), 12.5),
                "bilinear interpolation should blend neighboring pixels");
  ok &= require(std::isnan(Bilinear_Interpolation<float>(image, {-1.0, 1.0})),
                "bilinear interpolation should reject out-of-bounds points");

  Edge edge({16.0, 16.0}, 0.0, false, 0, 1);
  const auto patches = utility.get_edge_patches(edge, image);
  ok &= require(!patches.first.empty() && patches.first.type() == CV_32F &&
                    patches.first.rows == 7 && patches.first.cols == 7,
                "edge patches should use the established 7x7 float layout");
  ok &= require(ComputeNCC(patches.first, patches.first) > 0.999,
                "identical patches should have unit NCC");
  ok &= require(utility.get_patch_similarity(cv::Mat::ones(3, 3, CV_32F),
                                             cv::Mat::ones(3, 3, CV_32F)) < 0.0,
                "constant patches should be rejected by NCC");

  const Eigen::Vector3d point(0.2, 0.1, 2.0);
  const Eigen::Vector3d pixel1(point.x() / point.z(), point.y() / point.z(), 1.0);
  const Eigen::Vector3d translated(point.x() - 1.0, point.y(), point.z());
  const Eigen::Vector3d pixel2(translated.x() / translated.z(),
                               translated.y() / translated.z(), 1.0);
  const Eigen::Vector3d triangulated = utility.two_view_linear_triangulation(
      pixel1, pixel2, Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Identity(), Eigen::Vector3d(-1.0, 0.0, 0.0));
  ok &= require(triangulated.isApprox(point, 1e-6),
                "two-view triangulation should recover a known point");

  const Eigen::Matrix3d target_rotation =
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d target_translation(0.3, -0.2, 0.1);
  const auto project = [&point](const Eigen::Matrix3d& rotation,
                                const Eigen::Vector3d& translation) {
    const Eigen::Vector3d camera_point = rotation * point + translation;
    return Eigen::Vector2d(camera_point.x() / camera_point.z(),
                           camera_point.y() / camera_point.z());
  };
  const std::vector<Eigen::Vector2d> multiview_points = {
      Eigen::Vector2d(pixel1.x(), pixel1.y()),
      Eigen::Vector2d(pixel2.x(), pixel2.y()),
      project(target_rotation, target_translation)};
  const Eigen::Vector3d multiview_point = utility.multiview_linear_triangulation(
      3, multiview_points,
      {Eigen::Matrix3d::Identity(), target_rotation},
      {Eigen::Vector3d(-1.0, 0.0, 0.0), target_translation},
      Eigen::Matrix3d::Identity());
  ok &= require(multiview_point.isApprox(point, 1e-6),
                "multiview triangulation must consume N-1 target poses");
  ok &= require(!utility.multiview_linear_triangulation(
                           3, multiview_points,
                           {Eigen::Matrix3d::Identity(), target_rotation,
                            Eigen::Matrix3d::Identity()},
                           {Eigen::Vector3d(-1.0, 0.0, 0.0), target_translation,
                            Eigen::Vector3d::Zero()},
                           Eigen::Matrix3d::Identity())
                      .allFinite(),
                "multiview pose vectors must reject heuristic-sized inputs");
  const Eigen::Vector3d zero_baseline = utility.two_view_linear_triangulation(
      pixel1, pixel1, Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  ok &= require(!zero_baseline.allFinite(),
                "zero-baseline triangulation must reject rank-deficient geometry");

  const Eigen::Vector3d degenerate = utility.backproject_2D_point_to_3D_point_using_rays(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 1.0));
  ok &= require(!degenerate.allFinite(), "degenerate ray intersection should be explicit");

  return ok ? 0 : 1;
}
