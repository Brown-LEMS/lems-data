/**
 * @file edge.hpp
 * @brief Shared 2-D and 3-D edge observations.
 * @ingroup edges
 */
#pragma once

// Shared edge data extracted from Brown-LEMS Multinocular-Edge-Visual-Odometry
// (GPU_dev), originally maintained by Chiang-Heng Chien and Juehan Lin.

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cstddef>
#include <functional>
#include <limits>

namespace lems::data {

/**
 * @brief A detected 2-D edge observation.
 * @ingroup edges
 *
 * `orientation` is an angle in radians. `frame_source` and `index` are the
 * stable identity used by matching containers; location, orientation, and
 * the empty marker may be refined without changing equality or hashing.
 */
struct Edge {
  cv::Point2d location{-1.0, -1.0}; ///< Pixel location; default is a sentinel.
  double orientation{-100.0}; ///< Edge angle in radians; default is a sentinel.
  bool b_isEmpty{true}; ///< Detector empty marker.
  int frame_source{-1}; ///< Source detector frame identity.
  int index{-1}; ///< Source edge index.

  Edge() = default;
  Edge(cv::Point2d point, double angle, bool empty, int source,
       int edge_index = -1)
      : location(point), orientation(angle), b_isEmpty(empty),
        frame_source(source), index(edge_index) {}

  /// Compare stable identity only; mutable geometry is intentionally ignored.
  bool operator==(const Edge& other) const noexcept {
    return frame_source == other.frame_source && index == other.index;
  }
  bool operator!=(const Edge& other) const noexcept { return !(*this == other); }
};

/**
 * @brief A triangulated edge point and tangent in camera coordinates.
 * @ingroup edges
 *
 * Identity follows the corresponding 2-D edge identity. The constructor does
 * not normalize `tangent`.
 */
struct Edge_3D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d location{-1.0, -1.0, -1.0}; ///< Camera-space point sentinel.
  Eigen::Vector3d tangent{-100.0, -100.0, -100.0}; ///< Camera-space tangent sentinel.
  bool b_isEmpty{true}; ///< Detector empty marker.
  int frame_source{-1}; ///< Source detector frame identity.
  int index{-1}; ///< Source edge index.

  Edge_3D() = default;
  Edge_3D(const Eigen::Vector3d& point, const Eigen::Vector3d& tangent_value,
          bool empty, int source, int edge_index = -1)
      : location(point), tangent(tangent_value), b_isEmpty(empty),
        frame_source(source), index(edge_index) {}

  /// Compare stable identity only; mutable geometry is intentionally ignored.
  bool operator==(const Edge_3D& other) const noexcept {
    return frame_source == other.frame_source && index == other.index;
  }
  bool operator!=(const Edge_3D& other) const noexcept { return !(*this == other); }
};

}  // namespace lems::data

namespace std {

/** @brief Hashes an @ref lems::data::Edge by frame source and index. @ingroup edges */
template <>
struct hash<lems::data::Edge> {
  std::size_t operator()(const lems::data::Edge& edge) const noexcept {
    // Equality is identity-based, so the hash must use exactly the identity
    // fields rather than the mutable sub-pixel geometry.
    const auto h_source = std::hash<int>{}(edge.frame_source);
    const auto h_index = std::hash<int>{}(edge.index);
    return h_source ^ (h_index + static_cast<std::size_t>(0x9e3779b9) +
                       (h_source << 6U) + (h_source >> 2U));
  }
};

/** @brief Hashes an @ref lems::data::Edge_3D by frame source and index. @ingroup edges */
template <>
struct hash<lems::data::Edge_3D> {
  std::size_t operator()(const lems::data::Edge_3D& edge) const noexcept {
    const auto h_source = std::hash<int>{}(edge.frame_source);
    const auto h_index = std::hash<int>{}(edge.index);
    return h_source ^ (h_index + static_cast<std::size_t>(0x9e3779b9) +
                       (h_source << 6U) + (h_source >> 2U));
  }
};

}  // namespace std
