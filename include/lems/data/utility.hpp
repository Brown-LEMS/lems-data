/**
 * @file utility.hpp
 * @brief Eigen/OpenCV geometry, interpolation, patch, and image helpers.
 * @ingroup utilities
 */
#pragma once

// CV/Eigen utilities consolidated from Brown-LEMS utility.h/.cpp. GUI,
// detector, SIFT/xfeatures2d, CUDA, and OpenMP-specific helpers stay in the
// consuming pipeline so this library remains small and deployable.
// Credits: LEMS, Brown University; Chiang-Heng Chien and Juehan Lin.

#include "lems/data/edge.hpp"
#include "lems/data/types.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lems::data {

// Keep these canonical names distinct from the legacy preprocessor macros
// still shipped by the Brown-LEMS headers.  Some consumers include those
// definitions before this header, so identifiers named PATCH_SIZE or
// ORTHOGONAL_SHIFT_MAG are not safe here.
/** @brief Default odd patch side length. @ingroup utilities */
inline constexpr int kDefaultPatchSize = 7;
/** @brief Default edge-side offset in pixels. @ingroup utilities */
inline constexpr double kDefaultOrthogonalShift = 5.0;

/**
 * @brief Numerical options normalized by @ref Utility's constructor.
 * @ingroup utilities
 */
struct UtilityOptions {
  int patch_size{kDefaultPatchSize}; ///< Odd square patch side in pixels.
  double orthogonal_shift{kDefaultOrthogonalShift}; ///< Edge-side offset in pixels.
  double epsilon{1e-10}; ///< Degeneracy threshold.
};

/**
 * @brief Retained geometry and image utility facade.
 * @ingroup utilities
 *
 * Detector, GUI, descriptor, CUDA, and OpenMP-specific behavior is outside
 * this class. Invalid numerical systems return NaN vectors/scalars or the
 * documented zero/empty result rather than throwing.
 */
class Utility {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Utility>;

  /// Construct with normalized patch, shift, and epsilon options.
  explicit Utility(UtilityOptions options = {}) : options_(options) {
    if (options_.patch_size < 1) options_.patch_size = 1;
    if ((options_.patch_size % 2) == 0) ++options_.patch_size;
    if (!(options_.orthogonal_shift >= 0.0) ||
        !std::isfinite(options_.orthogonal_shift))
      options_.orthogonal_shift = 5.0;
    if (!(options_.epsilon > 0.0) || !std::isfinite(options_.epsilon))
      options_.epsilon = 1e-10;
  }

  /// Return the normalized options used by this utility instance.
  const UtilityOptions& options() const noexcept { return options_; }

  /// Return the skew-symmetric matrix `[t]x`.
  Eigen::Matrix3d get_Skew_Symmetric_Matrix(
      const Eigen::Vector3d& t) const noexcept;
  Eigen::Matrix3d getSkewSymmetricMatrix(
      const Eigen::Vector3d& t) const noexcept {
    return get_Skew_Symmetric_Matrix(t);
  }

  /// Project an edge point to an epipolar line and return normal distance.
  double getNormalDistance2EpipolarLine(
      const Eigen::Vector3d& epipolar_line, const Eigen::Vector3d& edge,
      double& epiline_x, double& epiline_y) const;
  double getNormalDistance2EpipolarLine(
      const Eigen::Vector3d& epipolar_line, const Eigen::VectorXd& edges,
      int index, double& epiline_x, double& epiline_y) const;
  /// Intersect an edge tangent with an epipolar line and return distance.
  double getTangentialDistance2EpipolarLine(
      const Eigen::Vector3d& epipolar_line, const Eigen::Vector3d& edge,
      double& x_intersection, double& y_intersection) const;
  double getTangentialDistance2EpipolarLine(
      const Eigen::Vector3d& epipolar_line, const Eigen::VectorXd& edges,
      int index, double& x_intersection, double& y_intersection) const;

  /// Solve two-ray backprojection; rank-deficient input returns NaNs.
  Eigen::Vector3d backproject_2D_point_to_3D_point_using_rays(
      const Eigen::Matrix3d& relative_rotation,
      const Eigen::Vector3d& relative_translation,
      const Eigen::Vector3d& ray1, const Eigen::Vector3d& ray2) const;
  /// Reconstruct and normalize a tangent from two tangent planes.
  Eigen::Vector3d reconstruct_3D_Tangent_through_intersection_of_planes(
      const Eigen::Matrix3d& relative_rotation, const Eigen::Vector3d& gamma1,
      const Eigen::Vector3d& gamma2, const Eigen::Vector3d& tangent1,
      const Eigen::Vector3d& tangent2) const;
  /// Project and normalize a 3-D tangent into a calibrated 2-D ray plane.
  Eigen::Vector3d project_3D_Tangent_to_2D_Tangent(
      const Eigen::Vector3d& tangent_3d, const Eigen::Vector3d& gamma) const;

  /// Compose target world-to-camera with the inverse source pose.
  CameraPose get_Relative_Pose(const CameraPose& source_pose,
                               const CameraPose& target_pose) const;

  /// Return plus/minus edge-side points using configured shift.
  std::pair<cv::Point2d, cv::Point2d> get_Orthogonal_Shifted_Points(
      const Edge& edge) const;
  std::pair<cv::Point2d, cv::Point2d> get_Orthogonal_Shifted_Points(
      const Edge& edge, double shift_magnitude) const;
  /// Sample one rotated edge-side patch and its coordinate matrices.
  void get_patch_on_one_edge_side(cv::Point2d shifted_point, double theta,
                                  cv::Mat& patch_coord_x,
                                  cv::Mat& patch_coord_y, cv::Mat& patch_val,
                                  const cv::Mat& image) const;
  /// Return plus/minus CV_32F patches, or an empty pair for an empty image.
  std::pair<cv::Mat, cv::Mat> get_edge_patches(
      const Edge& edge, const cv::Mat& image, bool debug = false) const;
  /// Return NCC similarity, or -1 for incompatible/constant patches.
  double get_patch_similarity(const cv::Mat& patch_one,
                              const cv::Mat& patch_two) const;

  /// Triangulate one point from two calibrated views; invalid input is NaN.
  Eigen::Vector3d two_view_linear_triangulation(
      const Eigen::Vector3d& gamma1, const Eigen::Vector3d& gamma2,
      const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2,
      const Eigen::Matrix3d& relative_rotation,
      const Eigen::Vector3d& relative_translation) const;
  /// Triangulate N views with N-1 relative poses; invalid input is NaN.
  Eigen::Vector3d multiview_linear_triangulation(
      int number_of_views, const std::vector<Eigen::Vector2d>& points,
      const std::vector<Eigen::Matrix3d>& rotations,
      const std::vector<Eigen::Vector3d>& translations,
      const Eigen::Matrix3d& K) const;

  /// Format an OpenCV type as a string such as `8UC1`.
  std::string cvMat_Type(int type) const;

 private:
  UtilityOptions options_;
};

/** @addtogroup utilities
 *  @{ */

namespace detail {

inline double nan_value() { return std::numeric_limits<double>::quiet_NaN(); }

inline bool valid_patch_size(int size) { return size > 0 && (size % 2) == 1; }

inline bool valid_single_channel(const cv::Mat& image) {
  return !image.empty() && image.channels() == 1;
}

template <typename T>
inline double bilinear_interpolation(const cv::Mat& image, cv::Point2d point) {
  if (!valid_single_channel(image) || image.depth() != cv::DataType<T>::depth)
    return nan_value();
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0 ||
      point.y < 0.0 || point.x > image.cols - 1.0 ||
      point.y > image.rows - 1.0)
    return nan_value();

  const int x0 = static_cast<int>(std::floor(point.x));
  const int y0 = static_cast<int>(std::floor(point.y));
  const int x1 = std::min(x0 + 1, image.cols - 1);
  const int y1 = std::min(y0 + 1, image.rows - 1);
  const double ax = point.x - x0;
  const double ay = point.y - y0;
  const double v00 = image.at<T>(y0, x0);
  const double v10 = image.at<T>(y0, x1);
  const double v01 = image.at<T>(y1, x0);
  const double v11 = image.at<T>(y1, x1);
  return (1.0 - ax) * (1.0 - ay) * v00 + ax * (1.0 - ay) * v10 +
         (1.0 - ax) * ay * v01 + ax * ay * v11;
}

}  // namespace detail

// Kept as a template because the original pipeline explicitly calls both
// Bilinear_Interpolation<float> and Bilinear_Interpolation<double>.
template <typename T>
inline double Bilinear_Interpolation(const cv::Mat& image, cv::Point2d point) {
  return detail::bilinear_interpolation<T>(image, point);
}

inline double Bilinear_Interpolation(const cv::Mat& image,
                                     cv::Point2d point) {
  if (image.empty()) return detail::nan_value();
  if (image.depth() == CV_32F)
    return detail::bilinear_interpolation<float>(image, point);
  if (image.depth() == CV_64F)
    return detail::bilinear_interpolation<double>(image, point);
  cv::Mat converted;
  image.convertTo(converted, CV_64F);
  return detail::bilinear_interpolation<double>(converted, point);
}

inline void util_compute_Img_Gradients(const cv::Mat& image, cv::Mat& gx,
                                       cv::Mat& gy) {
  if (image.empty()) {
    gx.release();
    gy.release();
    return;
  }
  cv::Mat image_32f;
  image.convertTo(image_32f, CV_32F);
  cv::Sobel(image_32f, gx, CV_32F, 1, 0, 3, 1.0 / 8.0);
  cv::Sobel(image_32f, gy, CV_32F, 0, 1, 3, 1.0 / 8.0);
}

inline void util_make_rotated_patch_coords(const cv::Point2d& center,
                                           double theta,
                                           std::vector<cv::Point2d>& coords,
                                           int patch_size = 7) {
  coords.clear();
  if (!detail::valid_patch_size(patch_size)) return;
  coords.reserve(static_cast<std::size_t>(patch_size) * patch_size);
  const int half = patch_size / 2;
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  for (int row = -half; row <= half; ++row) {
    for (int col = -half; col <= half; ++col) {
      coords.emplace_back(center.x + ct * row - st * col,
                          center.y + st * row + ct * col);
    }
  }
}

inline float util_bilinear_Sample_F(const cv::Mat& image, double x, double y) {
  if (!detail::valid_single_channel(image) || image.depth() != CV_32F ||
      image.cols == 0 || image.rows == 0)
    return std::numeric_limits<float>::quiet_NaN();
  x = std::clamp(x, 0.0, static_cast<double>(image.cols - 1));
  y = std::clamp(y, 0.0, static_cast<double>(image.rows - 1));
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = std::min(x0 + 1, image.cols - 1);
  const int y1 = std::min(y0 + 1, image.rows - 1);
  const double a = x - x0;
  const double b = y - y0;
  return static_cast<float>((1.0 - a) * (1.0 - b) * image.at<float>(y0, x0) +
                            a * (1.0 - b) * image.at<float>(y0, x1) +
                            (1.0 - a) * b * image.at<float>(y1, x0) +
                            a * b * image.at<float>(y1, x1));
}

inline void util_sample_patch_at_coords(const cv::Mat& image,
                                        const std::vector<cv::Point2d>& coords,
                                        std::vector<double>& values) {
  values.resize(coords.size());
  for (std::size_t i = 0; i < coords.size(); ++i)
    values[i] = util_bilinear_Sample_F(image, coords[i].x, coords[i].y);
}

template <typename T>
inline T util_vector_mean(const std::vector<T>& values) {
  if (values.empty()) return T{};
  T sum{};
  for (const T& value : values) sum += value;
  return sum / static_cast<T>(values.size());
}

inline double ComputeAverage(const std::vector<int>& values) {
  if (values.empty()) return 0.0;
  return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

inline cv::Scalar PickUniqueColor(int index, int total) {
  if (total <= 0) return cv::Scalar(0, 0, 0);
  const int hue = ((index % total) + total) % total * 180 / total;
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const auto color = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(color[0], color[1], color[2]);
}

inline double ComputeNCC(const cv::Mat& patch_one, const cv::Mat& patch_two) {
  if (patch_one.empty() || patch_two.empty() || patch_one.size() != patch_two.size() ||
      patch_one.channels() != 1 || patch_two.channels() != 1)
    return -1.0;
  cv::Mat one, two;
  patch_one.convertTo(one, CV_64F);
  patch_two.convertTo(two, CV_64F);
  one = one.reshape(1, 1);
  two = two.reshape(1, 1);
  one -= cv::mean(one)[0];
  two -= cv::mean(two)[0];
  const double norm_one = cv::norm(one);
  const double norm_two = cv::norm(two);
  if (norm_one <= 1e-12 || norm_two <= 1e-12) return -1.0;
  return one.dot(two) / (norm_one * norm_two);
}

inline void BuildImagePyramids(
    const cv::Mat& current_left_image, const cv::Mat& current_right_image,
    const cv::Mat& next_left_image, const cv::Mat& next_right_image,
    int number_of_levels, std::vector<cv::Mat>& current_left_pyramid,
    std::vector<cv::Mat>& current_right_pyramid,
    std::vector<cv::Mat>& next_left_pyramid,
    std::vector<cv::Mat>& next_right_pyramid) {
  current_left_pyramid.clear();
  current_right_pyramid.clear();
  next_left_pyramid.clear();
  next_right_pyramid.clear();
  if (number_of_levels <= 0) return;
  cv::buildPyramid(current_left_image, current_left_pyramid,
                   number_of_levels - 1);
  cv::buildPyramid(current_right_image, current_right_pyramid,
                   number_of_levels - 1);
  cv::buildPyramid(next_left_image, next_left_pyramid, number_of_levels - 1);
  cv::buildPyramid(next_right_image, next_right_pyramid,
                   number_of_levels - 1);
}

inline Eigen::Matrix3d ConvertToEigenMatrix(
    const std::vector<std::vector<double>>& matrix) {
  if (matrix.size() != 3 || matrix[0].size() != 3 || matrix[1].size() != 3 ||
      matrix[2].size() != 3)
    throw std::invalid_argument("ConvertToEigenMatrix requires a 3x3 matrix");
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) result(row, col) = matrix[row][col];
  return result;
}

template <typename T>
inline T rad_to_deg(T theta) {
  return theta * static_cast<T>(180.0 / M_PI);
}

template <typename T>
inline T deg_to_rad(T theta) {
  return theta * static_cast<T>(M_PI / 180.0);
}

inline bool angle_in_wedge(double value, double wedge_min, double wedge_max) {
  if ((wedge_min >= 0.0 && wedge_max >= 0.0) ||
      (wedge_min < 0.0 && wedge_max < 0.0)) {
    if (wedge_min <= wedge_max) return value >= wedge_min && value <= wedge_max;
    return value >= wedge_min || value <= wedge_max;
  }
  if (wedge_min >= 0.0 && wedge_max < 0.0)
    return value >= wedge_min || value <= wedge_max;
  if (wedge_min < 0.0 && wedge_max >= 0.0)
    return value >= wedge_min && value <= wedge_max;
  return false;
}

inline bool find_closest_boundary_intersection(
    double A, double B, double C, int image_width, int image_height,
    double reference_x, double reference_y, double& output_x,
    double& output_y) {
  if (image_width < 0 || image_height < 0) return false;
  struct Candidate {
    double x;
    double y;
    double distance_squared;
  };
  std::vector<Candidate> candidates;
  const auto add = [&](double x, double y) {
    if (x < 0.0 || x > image_width || y < 0.0 || y > image_height) return;
    const double dx = x - reference_x;
    const double dy = y - reference_y;
    candidates.push_back({x, y, dx * dx + dy * dy});
  };
  if (std::abs(B) > 1e-12) {
    add(0.0, -C / B);
    add(static_cast<double>(image_width),
        -(C + A * image_width) / B);
  }
  if (std::abs(A) > 1e-12) {
    add(-C / A, 0.0);
    add(-(C + B * image_height) / A, static_cast<double>(image_height));
  }
  if (candidates.empty()) {
    output_x = 0.0;
    output_y = 0.0;
    return false;
  }
  const auto closest = std::min_element(
      candidates.begin(), candidates.end(),
      [](const Candidate& lhs, const Candidate& rhs) {
        return lhs.distance_squared < rhs.distance_squared;
      });
  output_x = closest->x;
  output_y = closest->y;
  return true;
}

inline double compute_epipolar_angle(double point_x, double point_y,
                                     double epipole_x, double epipole_y) {
  const double dx = point_x - epipole_x;
  const double dy = point_y - epipole_y;
  if (std::abs(dx) <= 1e-10)
    return dy >= 0.0 ? std::numeric_limits<double>::infinity()
                     : -std::numeric_limits<double>::infinity();
  return dy / dx;
}

inline bool point_in_wedge_robust(
    double point_x, double point_y, double epipole_x, double epipole_y,
    double wedge_min, double wedge_max, int image_width, int image_height,
    double infinity_epipole_scale = 1e4) {
  (void)image_height;
  const double width_scale = std::max(1.0, static_cast<double>(image_width));
  const bool rectified_limit = std::isfinite(epipole_x) &&
                               std::abs(epipole_x) >
                                   infinity_epipole_scale * width_scale;
  if (rectified_limit) {
    const double y1 = epipole_y - epipole_x * wedge_min;
    const double y2 = epipole_y - epipole_x * wedge_max;
    return point_y >= std::min(y1, y2) && point_y <= std::max(y1, y2);
  }
  return angle_in_wedge(compute_epipolar_angle(point_x, point_y, epipole_x,
                                                epipole_y),
                        wedge_min, wedge_max);
}

inline std::vector<int> find_Unique_Sorted_Numbers(std::vector<int> values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  return values;
}

/** @} */

}  // namespace lems::data
