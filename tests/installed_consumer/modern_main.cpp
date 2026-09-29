#include <lems/data/edge.hpp>
#include <lems/data/pipeline_types.hpp>
#include <lems/data/utility.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <type_traits>
#include <unordered_set>
#include <vector>

static_assert(std::is_same_v<lems::data::Edge, lems::data::Edge>);
static_assert(std::is_same_v<lems::data::Edge_3D, lems::data::Edge_3D>);

int main() {
  lems::data::Edge edge(cv::Point2d(1.0, 2.0), 0.5, false, 0, 3);
  std::unordered_set<lems::data::Edge> edges{edge};
  lems::data::Utility utility;
  const auto skew = utility.getSkewSymmetricMatrix(Eigen::Vector3d::UnitX());
  if (edges.size() != 1 || skew(1, 2) != -1.0 || skew(2, 1) != 1.0)
    return 1;
  lems::data::Main_Observer observer;
  return observer.size() == 0 ? 0 : 2;
}
