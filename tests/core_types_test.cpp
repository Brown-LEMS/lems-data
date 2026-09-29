#include "lems/data/pipeline_types.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>

namespace {

bool close(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool require(bool condition, const char* message) {
  if (!condition) std::cerr << "core_types_test: " << message << '\n';
  return condition;
}

}  // namespace

int main() {
  using namespace lems::data;
  bool ok = true;

  Edge first({10.0, 20.0}, 0.2, false, 3, 7);
  Edge refined({12.0, 22.0}, 0.5, false, 3, 7);
  Edge different({10.0, 20.0}, 0.2, false, 3, 8);
  ok &= require(first == refined, "edge identity should survive geometric refinement");
  ok &= require(first != different, "edge identity must include detector index");
  std::unordered_set<Edge> identities;
  identities.insert(first);
  identities.insert(refined);
  identities.insert(different);
  ok &= require(identities.size() == 2, "edge hash must match identity equality");

  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  CameraPose pose(rotation, Eigen::Vector3d(1.0, 2.0, 3.0));
  ok &= require(pose.center().isApprox(-rotation.transpose() * pose.t),
                "camera center must invert world-to-camera extrinsics");
  ok &= require(pose.transform(pose.center()).norm() < 1e-9,
                "camera center should transform to the origin");
  ok &= require(pose.make_Rt_in_4x4().isApprox(pose.matrix()),
                "legacy pose matrix name should match canonical matrix");

  SpatialGrid grid(40, 40, 10);
  ok &= require(grid.add_edge_to_grids(4, cv::Point2f(15.0F, 15.0F)) == 5,
                "valid point should be inserted in its cell");
  ok &= require(grid.add_edge_to_grids(9, cv::Point2f(-0.1F, 15.0F)) == -1,
                "negative point must not alias the first cell");
  ok &= require(grid.getCandidatesWithinRadius(cv::Point2d(15.0, 15.0), 0.0).size() == 1,
                "grid query should return the inserted edge");
  ok &= require(grid.cell_for({std::numeric_limits<double>::quiet_NaN(), 1.0}) ==
                    std::pair<int, int>(-1, -1),
                "nonfinite grid coordinates must be rejected");
  ok &= require(grid.cell_for({std::numeric_limits<double>::max(), 1.0}) ==
                    std::pair<int, int>(-1, -1),
                "unrepresentable grid coordinates must be rejected");
  ok &= require(grid.getCandidatesWithinRadius(cv::Point2d(15.0, 15.0),
                                                std::numeric_limits<double>::max()).size() == 1,
                "huge finite grid radius must remain bounded and safe");
  ok &= require(grid.getCandidatesWithinRadius(cv::Point2d(15.0, 15.0),
                                                std::numeric_limits<double>::infinity()).empty(),
                "infinite grid radius must be rejected");

  Frame frame;
  frame.timestamp_seconds = 1.25;
  frame.metadata.disparity = cv::Mat::ones(2, 2, CV_32F);
  frame.ground_truth = pose;
  frame.synchronize_legacy_fields();
  ok &= require(close(frame.timestamp, 1.25), "legacy timestamp should synchronize");
  ok &= require(frame.has_ground_truth && frame.gt_camera_pose.center().isApprox(pose.center()),
                "legacy ground truth fields should synchronize");
  frame.ground_truth.reset();
  frame.synchronize_legacy_fields();
  ok &= require(!frame.has_ground_truth && frame.gt_camera_pose.R.isApprox(Eigen::Matrix3d::Identity()),
                "clearing optional ground truth must clear legacy validity");

  return ok ? 0 : 1;
}
