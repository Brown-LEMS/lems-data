#include <lems/data/pipeline_types.hpp>
#include <lems/data/types.hpp>
#include <lems/data/utility.hpp>

// These are the names included by the existing VO repositories.
#include <lems/vo/Dataset.h>
#include <lems/vo/Multinocular_Iterator.h>
#include <lems/vo/Stereo_Iterator.h>
#include <lems/vo/utility.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <type_traits>
#include <unordered_set>
#include <vector>

static_assert(std::is_same_v<Edge, lems::data::Edge>);
static_assert(std::is_same_v<Edge_3D, lems::data::Edge_3D>);
static_assert(std::is_same_v<Camera_Pose, lems::data::CameraPose>);
static_assert(std::is_same_v<Utility, lems::data::Utility>);

int main() {
  Edge edge(cv::Point2d(12.0, 13.0), 0.25, false, 0);
  edge.index = 7;
  std::vector<Edge> edges{edge};
  std::unordered_set<Edge> edge_set(edges.begin(), edges.end());
  if (edge_set.size() != 1 || edges.front().index != 7) return 1;

  Camera_Pose pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 2.0, 3.0));
  const auto center = pose.center();
  if ((center + pose.t).norm() > 1e-12) return 2;

  Utility utility;
  const auto skew = utility.getSkewSymmetricMatrix(Eigen::Vector3d::UnitX());
  if (skew(1, 2) != -1.0 || skew(2, 1) != 1.0) return 3;

  const auto shifted = utility.get_Orthogonal_Shifted_Points(edge, 1.0);
  if (shifted.first == shifted.second) return 4;

  lems::data::Frame frame;
  frame.edges.push_back(edge);
  Main_Observer observer(&frame);
  observer.edge_indices.push_back(0);
  observer.edges_3D.emplace_back(Eigen::Vector3d::Zero(),
                                 Eigen::Vector3d::UnitX(), false, 0, 7);
  observer.sift_descriptors.emplace_back(cv::Mat{}, cv::Mat{});
  observer.edge_patches.emplace_back(cv::Mat{}, cv::Mat{});
  if (!observer.check_observer_validity()) return 5;

  return 0;
}
