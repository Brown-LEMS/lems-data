/**
 * @file pipeline_types.hpp
 * @brief Data-only edge matching, observer, and temporal records.
 * @ingroup pipeline
 */
#pragma once

// Edge-matching data-only structures extracted from Brown-LEMS
// Multinocular-Edge-Visual-Odometry (GPU_dev). Detector and CUDA ownership
// intentionally remain in the consuming pipeline.

#include "lems/data/types.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lems::data {

// Spatial index used by stereo and multinocular edge matching.  Coordinates
// are mapped with floor, not truncation, so negative points never alias cell 0.
/**
 * @brief Floor-based spatial index for edge candidates.
 * @ingroup pipeline
 *
 * Pixel coordinates use floor division, including negative values. Candidate
 * queries return stored edge indices from nearby cells; they do not perform a
 * second exact distance test.
 */
struct SpatialGrid {
  int cell_size{35}; ///< Cell width/height in pixels; reset clamps to at least one.
  int grid_width{0}; ///< Number of cells horizontally.
  int grid_height{0}; ///< Number of cells vertically.
  std::vector<std::vector<int>> grid; ///< Linearized cell buckets.

  SpatialGrid() = default;
  SpatialGrid(int image_width, int image_height, int cell_size_value = 35) {
    reset_dimensions(image_width, image_height, cell_size_value);
  }

  void reset_dimensions(int image_width, int image_height,
                        int cell_size_value = 35) {
    cell_size = std::max(1, cell_size_value);
    grid_width = image_width > 0 ? (image_width + cell_size - 1) / cell_size : 0;
    grid_height = image_height > 0 ? (image_height + cell_size - 1) / cell_size : 0;
    grid.assign(static_cast<std::size_t>(grid_width) *
                    static_cast<std::size_t>(grid_height),
                {});
  }

  /// Return whether integer cell coordinates are inside the grid.
  bool valid_cell(int x, int y) const {
    return x >= 0 && x < grid_width && y >= 0 && y < grid_height;
  }

  /// Map a pixel location to a floor-based cell, or `(-1,-1)` when invalid.
  std::pair<int, int> cell_for(cv::Point2d location) const {
    if (cell_size <= 0 || !std::isfinite(location.x) ||
        !std::isfinite(location.y))
      return {-1, -1};
    const double raw_x = std::floor(location.x / cell_size);
    const double raw_y = std::floor(location.y / cell_size);
    if (raw_x < static_cast<double>(std::numeric_limits<int>::min()) ||
        raw_x > static_cast<double>(std::numeric_limits<int>::max()) ||
        raw_y < static_cast<double>(std::numeric_limits<int>::min()) ||
        raw_y > static_cast<double>(std::numeric_limits<int>::max()))
      return {-1, -1};
    return {static_cast<int>(raw_x), static_cast<int>(raw_y)};
  }

  void addEdge(int edge_idx, cv::Point2f location) {
    add_edge_to_grids(edge_idx, cv::Point2d(location.x, location.y));
  }

  int add_edge_to_grids(int edge_idx, cv::Point2f location) {
    return add_edge_to_grids(edge_idx, cv::Point2d(location.x, location.y));
  }

  /// Add an edge index to a cell and return its linear index, or `-1`.
  int add_edge_to_grids(int edge_idx, cv::Point2d location) {
    const auto [x, y] = cell_for(location);
    if (!valid_cell(x, y)) return -1;
    grid[static_cast<std::size_t>(y) * grid_width + x].push_back(edge_idx);
    return y * grid_width + x;
  }

  /// Clear all cell buckets while retaining dimensions.
  void reset() {
    for (auto& cell : grid) cell.clear();
  }

  /// Return candidate indices around an edge location.
  std::vector<int> getCandidatesWithinRadius(const Edge& edge,
                                              double radius) const {
    return getCandidatesWithinRadius(edge.location, radius);
  }

  std::vector<int> getCandidatesWithinRadius(cv::Point2d location,
                                              double radius) const {
    std::vector<int> candidates;
    const auto [grid_x, grid_y] = cell_for(location);
    if (!valid_cell(grid_x, grid_y) || !std::isfinite(radius) || radius < 0.0)
      return candidates;
    const double raw_search_radius = std::ceil(
        radius / static_cast<double>(std::max(1, cell_size)));
    const int max_grid_radius = std::max(grid_width, grid_height);
    const int search_radius = static_cast<int>(std::min(
        raw_search_radius, static_cast<double>(max_grid_radius)));
    // The explicit end checks avoid incrementing INT_MAX when a very large
    // finite radius is queried against a maximally sized grid.
    for (int dy = -search_radius;; ++dy) {
      for (int dx = -search_radius;; ++dx) {
        const std::int64_t candidate_x = static_cast<std::int64_t>(grid_x) + dx;
        const std::int64_t candidate_y = static_cast<std::int64_t>(grid_y) + dy;
        if (candidate_x >= 0 && candidate_x < grid_width &&
            candidate_y >= 0 && candidate_y < grid_height) {
          const auto x = static_cast<std::size_t>(candidate_x);
          const auto y = static_cast<std::size_t>(candidate_y);
          const auto& cell = grid[y * static_cast<std::size_t>(grid_width) + x];
          candidates.insert(candidates.end(), cell.begin(), cell.end());
        }
        if (dx == search_radius) break;
      }
      if (dy == search_radius) break;
    }
    return candidates;
  }
};

/** @brief NCC/SIFT score pair retained by matching records. @ingroup pipeline */
struct scores {
  double ncc_score{0.0};
  double sift_score{0.0};
};

/** @brief Edge cluster and refinement data. @ingroup pipeline */
struct EdgeCluster {
  Edge center_edge;
  std::vector<Edge> contributing_edges;
  std::vector<int> contributing_edges_toed_indices;
  std::pair<cv::Mat, cv::Mat> center_edge_patches;
  std::pair<cv::Mat, cv::Mat> center_edge_descriptors;
  int previous_cluster_index{-1};
  std::vector<int> ancestor_cluster_indices;
  int paired_left_edge_index{-1};
  bool b_is_TP{false};
};

/** @brief Per-step edge evaluation statistics. @ingroup pipeline */
struct Evaluation_Statistics {
  std::map<std::string, std::vector<EdgeCluster>> edge_clusters_in_each_step;
  std::vector<double> refine_final_scores;
  std::vector<double> refine_confidences;
  std::vector<bool> refine_validities;
  std::vector<double> FN_dist_error_to_GT;
  std::vector<EdgeCluster> false_negative_edge_clusters;
};

/**
 * @brief Non-owning frame observer with edge, patch, and descriptor vectors.
 * @ingroup pipeline
 */
struct Observer {
  const Frame* frame{nullptr};
  std::vector<Edge_3D> edges_3D;
  std::vector<std::pair<cv::Mat, cv::Mat>> sift_descriptors;
  std::vector<std::pair<cv::Mat, cv::Mat>> edge_patches;

  Observer() = default;
  explicit Observer(const Frame* frame_ptr) : frame(frame_ptr) {}
  virtual ~Observer() = default;

  void clean_up_observer_base() {
    edges_3D.clear();
    sift_descriptors.clear();
    edge_patches.clear();
  }
};

/** @brief Main-camera observer and TOED-index mapping. @ingroup pipeline */
struct Main_Observer : public Observer {
  std::vector<int> edge_indices;
  std::unordered_map<int, int> toed_edge_index_to_edge_indices_map;

  Main_Observer() = default;
  explicit Main_Observer(const Frame* frame_ptr) : Observer(frame_ptr) {}

  void clean_up_vector_data_structures() {
    edge_indices.clear();
    clean_up_observer_base();
    toed_edge_index_to_edge_indices_map.clear();
  }

  void construct_toed_edge_index_to_edge_indices_map() {
    toed_edge_index_to_edge_indices_map.clear();
    for (std::size_t i = 0; i < edge_indices.size(); ++i)
      toed_edge_index_to_edge_indices_map[edge_indices[i]] =
          static_cast<int>(i);
  }

  /// Check frame pointer and all parallel vector sizes.
  bool check_observer_validity() const {
    return frame != nullptr && edge_indices.size() == edges_3D.size() &&
           edge_indices.size() == sift_descriptors.size() &&
           edge_indices.size() == edge_patches.size();
  }

  std::size_t size() const { return edge_indices.size(); }

  Edge get_edge_by_Frame_index(std::size_t i) const {
    if (frame == nullptr || i >= frame->edges.size()) return Edge();
    return frame->edges[i];
  }

  Edge get_edge_by_subset_edge_index(std::size_t i) const {
    if (frame == nullptr || i >= edge_indices.size() ||
        edge_indices[i] < 0 ||
        static_cast<std::size_t>(edge_indices[i]) >= frame->edges.size())
      return Edge();
    return frame->edges[static_cast<std::size_t>(edge_indices[i])];
  }
};

/** @brief Sub-camera observer and epipolar geometry records. @ingroup pipeline */
struct Sub_Observer : public Observer {
  int id{-1};
  std::map<int, std::vector<double>> epipolar_angles_from;
  std::map<int, Eigen::Vector3d> epipoles_from;
  std::map<int, Eigen::Matrix3d> F_from;
  Eigen::Matrix3d F_main_to_sub{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d F_main_stereo_to_sub{Eigen::Matrix3d::Zero()};

  Sub_Observer() = default;
  explicit Sub_Observer(const Frame* frame_ptr) : Observer(frame_ptr) {}

  void clean_up_vector_data_structures() {
    clean_up_observer_base();
    epipolar_angles_from.clear();
    epipoles_from.clear();
    F_from.clear();
  }
};

/** @brief Intermediate patch, score, and refinement vectors. @ingroup pipeline */
struct Internal_Matching_Edge_Clusters {
  std::vector<EdgeCluster> edge_clusters;
  std::vector<std::pair<cv::Mat, cv::Mat>> matching_edge_patches;
  std::vector<scores> matching_scores;
  std::vector<double> refine_final_scores;
  std::vector<double> refine_confidences;
  std::vector<bool> refine_validities;
};

/** @brief One source edge's match data for a target observer. @ingroup pipeline */
struct EdgeMatch {
  int source_edge_idx{-1};
  int target_observer_id{-1};
  Edge veridical_edge;
  cv::Point2d GT_location{-1.0, -1.0};
  Internal_Matching_Edge_Clusters matching_edge_clusters;
};

/** @brief Match records for one source edge. @ingroup pipeline */
using Edge_Loop = std::vector<EdgeMatch>;

/**
 * @brief Main/sub-camera observer set and all edge match loops.
 * @ingroup pipeline
 */
struct Camera_Set {
  int num_observers{0};
  Main_Observer main_observer;
  std::vector<Sub_Observer> sub_observers;
  std::vector<Edge_Loop> all_matches;
  bool has_GT{false};

  /// Check that each main edge has one match loop.
  bool consistency_check() const {
    return main_observer.edge_indices.size() == all_matches.size();
  }
  // Typo-compatible spelling from Dataset.h.
  bool concistency_check() const { return consistency_check(); }
};

/** @brief Final stereo edge pair and triangulated tangent data. @ingroup pipeline */
struct final_stereo_edge_pair {
  Edge left_edge;
  Edge right_edge;
  Eigen::Vector3d Gamma_in_left_cam_coord{Eigen::Vector3d::Zero()};
  bool b_is_TP{false};
  std::pair<cv::Mat, cv::Mat> left_edge_patches;
  std::pair<cv::Mat, cv::Mat> right_edge_patches;
  std::pair<cv::Mat, cv::Mat> left_edge_descriptors;
  std::pair<cv::Mat, cv::Mat> right_edge_descriptors;
};

/** @brief Stereo edge-pair matching state with non-owning frame pointers. @ingroup pipeline */
struct Stereo_Edge_Pairs {
  const Frame* focused_frame{nullptr};
  const Frame* candidate_frame{nullptr};
  std::vector<int> focused_edge_indices;
  std::vector<cv::Point2d> GT_locations_from_left_edges;
  std::vector<Internal_Matching_Edge_Clusters> matching_edge_clusters;

  Edge get_focused_edge_by_Stereo_Edge_Pairs_index(int index) const {
    if (focused_frame == nullptr || index < 0 ||
        static_cast<std::size_t>(index) >= focused_edge_indices.size())
      return Edge();
    const int edge_index = focused_edge_indices[static_cast<std::size_t>(index)];
    if (edge_index < 0 ||
        static_cast<std::size_t>(edge_index) >= focused_frame->edges.size())
      return Edge();
    return focused_frame->edges[static_cast<std::size_t>(edge_index)];
  }
};

/** @brief Candidate-frame temporal edge cluster. @ingroup pipeline */
struct Temporal_CF_Edge_Cluster {
  int cf_stereo_edge_mate_index{-1};
  std::vector<int> contributing_cf_stereo_indices;
  Edge center_edge;
  std::vector<Edge> contributing_edges;
  scores matching_scores;
  double refine_final_score{1e6};
  bool refine_validity{false};
};

/** @brief Temporal edge pair with a non-owning keyframe mate. @ingroup pipeline */
struct temporal_edge_pair {
  const final_stereo_edge_pair* KF_stereo_edge_mate{nullptr};
  Eigen::Vector3d projected_point{Eigen::Vector3d::Zero()};
  double projected_orientation{0.0};
  std::vector<int> veridical_CF_stereo_edge_mate_indices;
  std::vector<Temporal_CF_Edge_Cluster> matching_CF_edge_clusters;
};

/** @brief Verified temporal quad entry. @ingroup pipeline */
struct Veridical_Quad_Entry {
  int cf_stereo_edge_mate_index{-1};
  Edge left_center;
  Edge right_center;
};

/** @brief Candidate temporal quad entry with non-owning cluster pointers. @ingroup pipeline */
struct Candidate_Quad_Entry {
  const Temporal_CF_Edge_Cluster* CF_left{nullptr};
  const Temporal_CF_Edge_Cluster* CF_right{nullptr};
};

/** @brief Keyframe temporal quad collection. @ingroup pipeline */
struct KF_Temporal_Edge_Quads {
  const final_stereo_edge_pair* KF_stereo_mate{nullptr};
  std::vector<Veridical_Quad_Entry> veridical_quads;
  std::vector<Candidate_Quad_Entry> candidate_quads;
  Eigen::Vector3d projected_point_left{Eigen::Vector3d::Zero()};
  Eigen::Vector3d projected_point_right{Eigen::Vector3d::Zero()};
  double projected_orientation_left{0.0};
  double projected_orientation_right{0.0};
  std::vector<bool> b_is_TP;
};

/** @brief One temporal view's availability and scores. @ingroup pipeline */
struct Temporal_View_Match {
  Edge center_edge;
  bool is_available{false};
  double ncc_score{-1.0};
  double sift_score{900.0};
  double refine_score{1e6};
  bool refine_valid{false};
};

/** @brief Recursive temporal candidate and contributing candidates. @ingroup pipeline */
struct Temporal_Candidate {
  int cf_match_idx{-1};
  std::vector<Temporal_View_Match> views;
  std::vector<Temporal_Candidate> contributing_candidates;
};

/** @brief Keyframe temporal match and evaluation flags. @ingroup pipeline */
struct KF_Temporal_Match {
  int kf_match_idx{-1};
  std::vector<Eigen::Vector3d> gt_proj_points;
  std::vector<double> gt_proj_orientations;
  std::vector<Edge> veridical_edges;
  std::vector<int> veridical_cf_match_idx;
  std::vector<Temporal_Candidate> candidates;
  std::vector<bool> b_is_TP;
  std::vector<bool> b_is_evaluable;
};

}  // namespace lems::data
