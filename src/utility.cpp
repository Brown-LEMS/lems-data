#include "lems/data/utility.hpp"

// Geometry, interpolation, edge-patch, and triangulation implementations are
// adapted from Brown-LEMS utility.cpp (GPU_dev).

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace lems::data {
namespace {

Eigen::Vector3d nan_vector3() {
  return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

Eigen::Vector3d homogeneous_point(const Eigen::Vector2d& point) {
  return Eigen::Vector3d(point.x(), point.y(), 1.0);
}

Eigen::Vector3d solve_homogeneous_point(const Eigen::MatrixXd& equations,
                                        double epsilon) {
  if (equations.cols() != 4 || equations.rows() < 4) return nan_vector3();
  if (!equations.allFinite() || equations.fullPivLu().rank() < 3)
    return nan_vector3();
  const Eigen::Vector4d homogeneous =
      equations.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
  if (!homogeneous.allFinite() ||
      std::abs(homogeneous.w()) <= epsilon)
    return nan_vector3();
  return homogeneous.head<3>() / homogeneous.w();
}

}  // namespace

Eigen::Matrix3d Utility::get_Skew_Symmetric_Matrix(
    const Eigen::Vector3d& t) const noexcept {
  Eigen::Matrix3d skew;
  skew << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  return skew;
}

double Utility::getNormalDistance2EpipolarLine(
    const Eigen::Vector3d& epipolar_line, const Eigen::Vector3d& edge,
    double& epiline_x, double& epiline_y) const {
  const double a = epipolar_line.x();
  const double b = epipolar_line.y();
  const double denominator = a * a + b * b;
  if (denominator <= options_.epsilon) {
    epiline_x = edge.x();
    epiline_y = edge.y();
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double signed_distance = a * edge.x() + b * edge.y() + epipolar_line.z();
  epiline_x = edge.x() - a * signed_distance / denominator;
  epiline_y = edge.y() - b * signed_distance / denominator;
  return std::hypot(edge.x() - epiline_x, edge.y() - epiline_y);
}

double Utility::getNormalDistance2EpipolarLine(
    const Eigen::Vector3d& epipolar_line, const Eigen::VectorXd& edges,
    int index, double& epiline_x, double& epiline_y) const {
  // Packed detector rows have the deterministic [x, y, theta] layout.
  // Keep the same stride for both distance overloads; accepting a guessed
  // two-column shape silently changes which edge is measured.
  if (index < 0 || edges.size() < 3 || edges.size() % 3 != 0) {
    epiline_x = epiline_y = std::numeric_limits<double>::quiet_NaN();
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::Index start = 3 * static_cast<Eigen::Index>(index);
  if (start < 0 || start > edges.size() - 3) {
    epiline_x = epiline_y = std::numeric_limits<double>::quiet_NaN();
    return std::numeric_limits<double>::quiet_NaN();
  }
  return getNormalDistance2EpipolarLine(
      epipolar_line,
      Eigen::Vector3d(edges(start), edges(start + 1),
                      edges(start + 2)),
      epiline_x, epiline_y);
}

double Utility::getTangentialDistance2EpipolarLine(
    const Eigen::Vector3d& epipolar_line, const Eigen::Vector3d& edge,
    double& x_intersection, double& y_intersection) const {
  // The edge tangent line has direction (cos(theta), sin(theta)); this
  // coefficient form avoids tan(theta)'s singularity at vertical edges.
  const double tangent_a = std::sin(edge.z());
  const double tangent_b = -std::cos(edge.z());
  const double tangent_c = -(tangent_a * edge.x() + tangent_b * edge.y());
  const double denominator = epipolar_line.x() * tangent_b -
                             tangent_a * epipolar_line.y();
  if (std::abs(denominator) <= options_.epsilon) {
    x_intersection = y_intersection = std::numeric_limits<double>::quiet_NaN();
    return std::numeric_limits<double>::quiet_NaN();
  }
  x_intersection = (epipolar_line.y() * tangent_c - tangent_b * epipolar_line.z()) /
                   denominator;
  y_intersection = (epipolar_line.z() * tangent_a - tangent_c * epipolar_line.x()) /
                   denominator;
  return std::hypot(x_intersection - edge.x(), y_intersection - edge.y());
}

double Utility::getTangentialDistance2EpipolarLine(
    const Eigen::Vector3d& epipolar_line, const Eigen::VectorXd& edges,
    int index, double& x_intersection, double& y_intersection) const {
  if (index < 0 || edges.size() < 3 || edges.size() % 3 != 0) {
    x_intersection = y_intersection = std::numeric_limits<double>::quiet_NaN();
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::Index start = 3 * static_cast<Eigen::Index>(index);
  if (start < 0 || start > edges.size() - 3) {
    x_intersection = y_intersection = std::numeric_limits<double>::quiet_NaN();
    return std::numeric_limits<double>::quiet_NaN();
  }
  return getTangentialDistance2EpipolarLine(
      epipolar_line,
      Eigen::Vector3d(edges(start), edges(start + 1), edges(start + 2)),
      x_intersection, y_intersection);
}

Eigen::Vector3d Utility::backproject_2D_point_to_3D_point_using_rays(
    const Eigen::Matrix3d& relative_rotation,
    const Eigen::Vector3d& relative_translation, const Eigen::Vector3d& ray1,
    const Eigen::Vector3d& ray2) const {
  // R * (rho1 ray1) + T = rho2 ray2.
  Eigen::Matrix<double, 3, 2> equations;
  equations.col(0) = relative_rotation * ray1;
  equations.col(1) = -ray2;
  if (equations.fullPivLu().rank() < 2) return nan_vector3();
  const Eigen::Vector2d depths =
      equations.colPivHouseholderQr().solve(-relative_translation);
  if (!depths.allFinite()) return nan_vector3();
  return depths[0] * ray1;
}

Eigen::Vector3d Utility::reconstruct_3D_Tangent_through_intersection_of_planes(
    const Eigen::Matrix3d& relative_rotation, const Eigen::Vector3d& gamma1,
    const Eigen::Vector3d& gamma2, const Eigen::Vector3d& tangent1,
    const Eigen::Vector3d& tangent2) const {
  const Eigen::Vector3d normal1 = tangent1.cross(gamma1);
  const Eigen::Vector3d normal2 =
      relative_rotation.transpose() * tangent2.cross(gamma2);
  const Eigen::Vector3d tangent = normal1.cross(normal2);
  const double norm = tangent.norm();
  if (norm <= options_.epsilon || !std::isfinite(norm)) return Eigen::Vector3d::Zero();
  return tangent / norm;
}

Eigen::Vector3d Utility::project_3D_Tangent_to_2D_Tangent(
    const Eigen::Vector3d& tangent_3d, const Eigen::Vector3d& gamma) const {
  const Eigen::Vector3d projected = tangent_3d - tangent_3d.z() * gamma;
  const double norm = projected.norm();
  if (norm <= options_.epsilon || !std::isfinite(norm)) return Eigen::Vector3d::Zero();
  return projected / norm;
}

CameraPose Utility::get_Relative_Pose(const CameraPose& source_pose,
                                      const CameraPose& target_pose) const {
  const Eigen::Matrix3d relative_rotation =
      target_pose.R * source_pose.R.transpose();
  const Eigen::Vector3d relative_translation =
      target_pose.t - relative_rotation * source_pose.t;
  return CameraPose(target_pose.timestamp_ns, relative_rotation,
                    relative_translation);
}

std::pair<cv::Point2d, cv::Point2d> Utility::get_Orthogonal_Shifted_Points(
    const Edge& edge) const {
  return get_Orthogonal_Shifted_Points(edge, options_.orthogonal_shift);
}

std::pair<cv::Point2d, cv::Point2d> Utility::get_Orthogonal_Shifted_Points(
    const Edge& edge, double shift_magnitude) const {
  const double sine = std::sin(edge.orientation);
  const double cosine = std::cos(edge.orientation);
  const cv::Point2d plus(edge.location.x + shift_magnitude * sine,
                         edge.location.y - shift_magnitude * cosine);
  const cv::Point2d minus(edge.location.x - shift_magnitude * sine,
                          edge.location.y + shift_magnitude * cosine);
  return {plus, minus};
}

void Utility::get_patch_on_one_edge_side(
    cv::Point2d shifted_point, double theta, cv::Mat& patch_coord_x,
    cv::Mat& patch_coord_y, cv::Mat& patch_val, const cv::Mat& image) const {
  const int patch_size = options_.patch_size;
  patch_coord_x.create(patch_size, patch_size, CV_64F);
  patch_coord_y.create(patch_size, patch_size, CV_64F);
  patch_val.create(patch_size, patch_size, CV_64F);
  cv::Mat image_64f;
  if (!image.empty()) {
    if (image.channels() == 1) {
      image.convertTo(image_64f, CV_64F);
    } else {
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      gray.convertTo(image_64f, CV_64F);
    }
  }

  const int half = patch_size / 2;
  const double cosine = std::cos(theta);
  const double sine = std::sin(theta);
  for (int row = -half; row <= half; ++row) {
    for (int col = -half; col <= half; ++col) {
      const cv::Point2d point(cosine * row - sine * col + shifted_point.x,
                              sine * row + cosine * col + shifted_point.y);
      const int output_row = row + half;
      const int output_col = col + half;
      patch_coord_x.at<double>(output_row, output_col) = point.x;
      patch_coord_y.at<double>(output_row, output_col) = point.y;
      patch_val.at<double>(output_row, output_col) =
          image_64f.empty() ? std::numeric_limits<double>::quiet_NaN()
                            : Bilinear_Interpolation<double>(image_64f, point);
    }
  }
}

std::pair<cv::Mat, cv::Mat> Utility::get_edge_patches(
    const Edge& edge, const cv::Mat& image, bool debug) const {
  (void)debug;  // GUI/debug visualization intentionally lives outside the core.
  if (image.empty()) return {};
  const auto shifted_points = get_Orthogonal_Shifted_Points(edge);
  cv::Mat ignored_x_plus, ignored_y_plus, ignored_x_minus, ignored_y_minus;
  cv::Mat patch_plus, patch_minus;
  get_patch_on_one_edge_side(shifted_points.first, edge.orientation,
                             ignored_x_plus, ignored_y_plus, patch_plus, image);
  get_patch_on_one_edge_side(shifted_points.second, edge.orientation,
                             ignored_x_minus, ignored_y_minus, patch_minus,
                             image);
  patch_plus.convertTo(patch_plus, CV_32F);
  patch_minus.convertTo(patch_minus, CV_32F);
  return {patch_plus, patch_minus};
}

double Utility::get_patch_similarity(const cv::Mat& patch_one,
                                     const cv::Mat& patch_two) const {
  return ComputeNCC(patch_one, patch_two);
}

Eigen::Vector3d Utility::two_view_linear_triangulation(
    const Eigen::Vector3d& gamma1, const Eigen::Vector3d& gamma2,
    const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2,
    const Eigen::Matrix3d& relative_rotation,
    const Eigen::Vector3d& relative_translation) const {
  Eigen::Matrix<double, 3, 4> projection1 =
      Eigen::Matrix<double, 3, 4>::Zero();
  projection1.leftCols<3>().setIdentity();
  Eigen::Matrix<double, 3, 4> projection2;
  projection2.leftCols<3>() = relative_rotation;
  projection2.col(3) = relative_translation;
  if (!gamma1.allFinite() || !gamma2.allFinite() || !K1.allFinite() ||
      !K2.allFinite() || K1.fullPivLu().rank() < 3 ||
      K2.fullPivLu().rank() < 3 || !relative_rotation.allFinite() ||
      !relative_translation.allFinite())
    return nan_vector3();
  const Eigen::Vector3d point1 = K1.inverse() * gamma1;
  const Eigen::Vector3d point2 = K2.inverse() * gamma2;
  Eigen::Matrix<double, 4, 4> equations;
  equations.row(0) = point1.x() * projection1.row(2) - projection1.row(0);
  equations.row(1) = point1.y() * projection1.row(2) - projection1.row(1);
  equations.row(2) = point2.x() * projection2.row(2) - projection2.row(0);
  equations.row(3) = point2.y() * projection2.row(2) - projection2.row(1);
  return solve_homogeneous_point(equations, options_.epsilon);
}

Eigen::Vector3d Utility::multiview_linear_triangulation(
    int number_of_views, const std::vector<Eigen::Vector2d>& points,
    const std::vector<Eigen::Matrix3d>& rotations,
    const std::vector<Eigen::Vector3d>& translations,
    const Eigen::Matrix3d& K) const {
  if (number_of_views < 2 ||
      points.size() != static_cast<std::size_t>(number_of_views))
    return nan_vector3();
  // This is the Dataset.h contract: view 0 is the reference camera and
  // rotations/translations contain exactly one relative pose for each target
  // view (N-1 entries).
  if (rotations.size() != static_cast<std::size_t>(number_of_views - 1) ||
      translations.size() != static_cast<std::size_t>(number_of_views - 1))
    return nan_vector3();
  if (!K.allFinite() || K.fullPivLu().rank() < 3) return nan_vector3();
  for (int view = 0; view < number_of_views; ++view)
    if (!points[view].allFinite()) return nan_vector3();
  for (std::size_t view = 0; view < rotations.size(); ++view)
    if (!rotations[view].allFinite() || !translations[view].allFinite())
      return nan_vector3();

  Eigen::MatrixXd equations(2 * number_of_views, 4);
  const Eigen::Vector3d first = K.inverse() * homogeneous_point(points[0]);
  Eigen::Matrix<double, 3, 4> reference =
      Eigen::Matrix<double, 3, 4>::Zero();
  reference.leftCols<3>().setIdentity();
  equations.row(0) = first.x() * reference.row(2) - reference.row(0);
  equations.row(1) = first.y() * reference.row(2) - reference.row(1);
  for (int view = 1; view < number_of_views; ++view) {
    const std::size_t pose_index = static_cast<std::size_t>(view - 1);
    Eigen::Matrix<double, 3, 4> projection;
    projection.leftCols<3>() = rotations[pose_index];
    projection.col(3) = translations[pose_index];
    const Eigen::Vector3d point = K.inverse() * homogeneous_point(points[view]);
    equations.row(2 * view) = point.x() * projection.row(2) - projection.row(0);
    equations.row(2 * view + 1) =
        point.y() * projection.row(2) - projection.row(1);
  }
  return solve_homogeneous_point(equations, options_.epsilon);
}

std::string Utility::cvMat_Type(int type) const {
  std::string result;
  const int depth = type & CV_MAT_DEPTH_MASK;
  const int channels = 1 + (type >> CV_CN_SHIFT);
  switch (depth) {
    case CV_8U: result = "8U"; break;
    case CV_8S: result = "8S"; break;
    case CV_16U: result = "16U"; break;
    case CV_16S: result = "16S"; break;
    case CV_32S: result = "32S"; break;
    case CV_32F: result = "32F"; break;
    case CV_64F: result = "64F"; break;
    default: result = "User"; break;
  }
  result += "C" + std::to_string(channels);
  return result;
}

}  // namespace lems::data
