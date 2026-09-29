#include <lems/vo/stereo_profile/Stereo_Iterator.h>
#include <lems/vo/stereo_profile/Dataset.h>
#include <lems/vo/stereo_profile/utility.h>
#include <lems/vo/Stereo_Pipeline_Types.h>

#include <lems/data/edge.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <type_traits>
#include <unordered_set>

static_assert(std::is_same_v<Edge, lems::data::Edge>);
static_assert(std::is_same_v<Edge_3D, lems::data::Edge_3D>);
static_assert(std::is_same_v<Camera_Pose, lems::data::CameraPose>);
static_assert(std::is_same_v<lems::vo::stereo::Edge, lems::data::Edge>);

int main() {
  Edge edge(cv::Point2d(4.0, 5.0), 0.25, false, 1, 9);
  std::unordered_set<Edge> edges{edge};
  Utility utility;
  const auto shifted = utility.get_Orthogonal_Shifted_Points(edge, 1.0);
  lems::vo::stereo::Stereo_Edge_Pairs pairs;
  return edges.size() == 1 && shifted.first != shifted.second && pairs.size() == 0
             ? 0
             : 1;
}
