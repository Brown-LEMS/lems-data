#include "Dataset.h"
#include "Stereo_Iterator.h"
#include "Multinocular_Iterator.h"

#include <type_traits>

static_assert(std::is_same_v<decltype(std::declval<Dataset>().get_left_calib_matrix()),
                             Eigen::Matrix3d>);
static_assert(std::is_same_v<decltype(std::declval<Dataset>().get_fund_mat_21()),
                             Eigen::Matrix3d>);
static_assert(std::is_same_v<decltype(std::declval<Dataset>().get_relative_transl_left_to_right()),
                             Eigen::Vector3d>);

int main() {
  StereoFrame stereo;
  Frame multinocular;
  return stereo.left_image.empty() && multinocular.image.empty() ? 0 : 1;
}
