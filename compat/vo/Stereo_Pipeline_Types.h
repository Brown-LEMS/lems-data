#pragma once

// Data-only records retained for the stereo Edge_Based_Visual_Odometry
// pipeline.  They intentionally live in a profile namespace: the GPU
// multinocular pipeline has a different Stereo_Edge_Pairs layout and must not
// be made to compile by an unsafe global alias.

#include <lems/data/types.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <map>
#include <memory>
#include <cmath>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lems::vo::stereo {

using Edge = lems::data::Edge;
using Edge_3D = lems::data::Edge_3D;
using Camera_Pose = lems::data::CameraPose;
using Frame = lems::data::Frame;

struct StereoFrame {
  lems::data::Frame left_frame;
  lems::data::Frame right_frame;
  cv::Mat left_image;
  cv::Mat right_image;
  cv::Mat left_image_undistorted;
  cv::Mat right_image_undistorted;
  double timestamp{};
  cv::Mat left_image_gradients_x;
  cv::Mat right_image_gradients_x;
  cv::Mat left_image_gradients_y;
  cv::Mat right_image_gradients_y;
  std::vector<Edge> left_edges;
  std::vector<Edge> right_edges;
  cv::Mat left_disparity_map;
  cv::Mat right_disparity_map;
  cv::Mat left_occlusion_mask;
  cv::Mat right_occlusion_mask;
  Camera_Pose gt_camera_pose;
};

struct GTPose {
  double timestamp{};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};

  GTPose() = default;
  GTPose(double ts, const Eigen::Matrix3d& R, const Eigen::Vector3d& T)
      : timestamp(ts), rotation(R), translation(T) {}
  bool operator<(const GTPose& other) const { return timestamp < other.timestamp; }
};

struct SpatialGrid {
  int cell_size{35};
  int grid_width{0};
  int grid_height{0};
  std::vector<std::vector<int>> grid;

  SpatialGrid() = default;
  SpatialGrid(int image_width, int image_height, int cell = 35)
      : cell_size(cell),
        grid_width((image_width + cell_size - 1) / cell_size),
        grid_height((image_height + cell_size - 1) / cell_size),
        grid(static_cast<std::size_t>(grid_width * grid_height)) {}

  void addEdge(int edge_index, cv::Point2f location) {
    const int x = static_cast<int>(location.x) / cell_size;
    const int y = static_cast<int>(location.y) / cell_size;
    if (x >= 0 && x < grid_width && y >= 0 && y < grid_height)
      grid[static_cast<std::size_t>(y * grid_width + x)].push_back(edge_index);
  }

  int add_edge_to_grids(int edge_index, cv::Point2f location) {
    const int x = static_cast<int>(location.x) / cell_size;
    const int y = static_cast<int>(location.y) / cell_size;
    if (x >= 0 && x < grid_width && y >= 0 && y < grid_height)
      grid[static_cast<std::size_t>(y * grid_width + x)].push_back(edge_index);
    return y * grid_width + x;
  }

  void reset() {
    for (auto& cell : grid) cell.clear();
  }

  std::vector<int> getCandidatesWithinRadius(const Edge& edge, double radius) const {
    return getCandidatesWithinRadius(edge.location, radius);
  }

  std::vector<int> getCandidatesWithinRadius(cv::Point2d location,
                                              double radius) const {
    std::vector<int> candidates;
    const int x = static_cast<int>(location.x) / cell_size;
    const int y = static_cast<int>(location.y) / cell_size;
    const int range = static_cast<int>(std::ceil(radius / cell_size));
    for (int dy = -range; dy <= range; ++dy) {
      for (int dx = -range; dx <= range; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height) {
          const auto& cell = grid[static_cast<std::size_t>(ny * grid_width + nx)];
          candidates.insert(candidates.end(), cell.begin(), cell.end());
        }
      }
    }
    return candidates;
  }
};

struct scores {
  double ncc_score{};
  double sift_score{};
};

struct EdgeCluster {
  Edge center_edge;
  std::vector<Edge> contributing_edges;
  std::vector<int> contributing_edges_toed_indices;
  int paired_left_edge_index{-1};
  bool b_is_TP{false};
};

struct Evaluation_Statistics {
  std::map<std::string, std::vector<EdgeCluster>> edge_clusters_in_each_step;
  std::vector<double> refine_final_scores;
  std::vector<double> refine_confidences;
  std::vector<double> refine_validities;
  std::vector<double> FN_dist_error_to_GT;
  std::vector<EdgeCluster> false_negative_edge_clusters;
};

struct Stereo_Matching_Edge_Clusters {
  std::vector<EdgeCluster> edge_clusters;
  std::vector<std::pair<cv::Mat, cv::Mat>> matching_edge_patches;
  std::vector<double> refine_final_scores;
  std::vector<double> refine_confidences;
  std::vector<bool> refine_validities;
};

struct Stereo_Edge_Pairs {
  const StereoFrame* stereo_frame{nullptr};
  cv::Mat left_disparity_map;
  cv::Mat right_disparity_map;
  std::vector<int> focused_edge_indices;
  std::vector<int> candidate_edge_indices;
  std::vector<cv::Point2d> GT_locations_from_left_edges;
  std::vector<std::vector<int>> veridical_right_edges_indices;
  std::vector<Eigen::Vector3d> Gamma_in_left_cam_coord;
  std::vector<Eigen::Vector3d> Gamma_in_right_cam_coord;
  std::vector<std::pair<cv::Mat, cv::Mat>> left_edge_descriptors;
  std::vector<int> grid_indices;
  std::vector<Eigen::Vector3d> epip_line_coeffs_of_left_edges;
  std::vector<std::pair<cv::Mat, cv::Mat>> left_edge_patches;
  std::unordered_map<int, std::size_t>
      toed_left_id_to_Stereo_Edge_Pairs_left_id_map;
  std::unordered_map<Edge, int> final_candidate_set;
  bool has_GT{false};
  std::vector<Stereo_Matching_Edge_Clusters> matching_edge_clusters;

  Stereo_Edge_Pairs() = default;
  explicit Stereo_Edge_Pairs(const StereoFrame* frame) : stereo_frame(frame) {}

  void clean_up_vector_data_structures() {
    focused_edge_indices.clear();
    candidate_edge_indices.clear();
    GT_locations_from_left_edges.clear();
    veridical_right_edges_indices.clear();
    Gamma_in_left_cam_coord.clear();
    Gamma_in_right_cam_coord.clear();
    left_edge_descriptors.clear();
    grid_indices.clear();
    epip_line_coeffs_of_left_edges.clear();
    left_edge_patches.clear();
    matching_edge_clusters.clear();
    toed_left_id_to_Stereo_Edge_Pairs_left_id_map.clear();
    final_candidate_set.clear();
  }

  void construct_toed_left_id_to_Stereo_Edge_Pairs_left_id_map() {
    toed_left_id_to_Stereo_Edge_Pairs_left_id_map.clear();
    for (std::size_t i = 0; i < focused_edge_indices.size(); ++i)
      toed_left_id_to_Stereo_Edge_Pairs_left_id_map[focused_edge_indices[i]] = i;
  }

  Edge get_left_edge_by_StereoFrame_index(std::size_t i) const {
    return stereo_frame && i < stereo_frame->left_edges.size()
               ? stereo_frame->left_edges[i]
               : Edge{};
  }
  Edge get_focused_edge_by_Stereo_Edge_Pairs_index(std::size_t i) const {
    return get_left_edge_by_StereoFrame_index(focused_edge_indices.at(i));
  }
  Edge get_focused_edge_by_toed_index(std::size_t i) const {
    const auto it = toed_left_id_to_Stereo_Edge_Pairs_left_id_map.find(
        static_cast<int>(i));
    return it == toed_left_id_to_Stereo_Edge_Pairs_left_id_map.end()
               ? Edge{}
               : get_focused_edge_by_Stereo_Edge_Pairs_index(it->second);
  }
  std::vector<Edge> get_focused_edges() const {
    std::vector<Edge> result;
    result.reserve(focused_edge_indices.size());
    for (int index : focused_edge_indices)
      result.push_back(get_left_edge_by_StereoFrame_index(index));
    return result;
  }
  std::vector<Edge> get_candidate_edges() const {
    std::vector<Edge> result;
    if (!stereo_frame) return result;
    result.reserve(candidate_edge_indices.size());
    for (int index : candidate_edge_indices) {
      if (index >= 0 && static_cast<std::size_t>(index) < stereo_frame->right_edges.size())
        result.push_back(stereo_frame->right_edges[index]);
    }
    return result;
  }
  int get_focused_toed_edge_index(std::size_t i) const {
    return focused_edge_indices.at(i);
  }
  int get_Stereo_Edge_Pairs_left_id_index(int toed_index) const {
    const auto it = toed_left_id_to_Stereo_Edge_Pairs_left_id_map.find(toed_index);
    return it == toed_left_id_to_Stereo_Edge_Pairs_left_id_map.end()
               ? -1
               : static_cast<int>(it->second);
  }
  int get_focused_edge_indices_size() const {
    return static_cast<int>(focused_edge_indices.size());
  }
  std::size_t size() const { return focused_edge_indices.size(); }
  bool b_is_size_consistent() const {
    return focused_edge_indices.size() == GT_locations_from_left_edges.size() &&
           focused_edge_indices.size() == veridical_right_edges_indices.size() &&
           focused_edge_indices.size() == Gamma_in_left_cam_coord.size() &&
           focused_edge_indices.size() == left_edge_descriptors.size();
  }
};

struct final_stereo_edge_pair {
  Edge left_edge;
  Edge right_edge;
  std::pair<cv::Mat, cv::Mat> left_edge_patches;
  std::pair<cv::Mat, cv::Mat> right_edge_patches;
  std::pair<cv::Mat, cv::Mat> left_edge_descriptors;
  std::pair<cv::Mat, cv::Mat> right_edge_descriptors;
  Eigen::Vector3d Gamma_in_left_cam_coord{Eigen::Vector3d::Zero()};
  Eigen::Vector3d Gamma_in_right_cam_coord{Eigen::Vector3d::Zero()};
  cv::Point2d gt_right_location{-1.0, -1.0};
  bool b_is_TP{false};
};

struct Temporal_CF_Edge_Cluster {
  int cf_stereo_edge_mate_index{-1};
  std::vector<int> contributing_cf_stereo_indices;
  Edge center_edge;
  std::vector<Edge> contributing_edges;
  scores matching_scores;
  double refine_final_score{1e6};
  bool refine_validity{false};
};

struct temporal_edge_pair {
  const final_stereo_edge_pair* KF_stereo_edge_mate{nullptr};
  Eigen::Vector3d projected_point{Eigen::Vector3d::Zero()};
  double projected_orientation{};
  std::vector<int> veridical_CF_stereo_edge_mate_indices;
  std::vector<Temporal_CF_Edge_Cluster> matching_CF_edge_clusters;
};

struct Veridical_Quad_Entry {
  int cf_stereo_edge_mate_index{-1};
  Edge left_center;
  Edge right_center;
};

struct Candidate_Quad_Entry {
  const Temporal_CF_Edge_Cluster* CF_left{nullptr};
  const Temporal_CF_Edge_Cluster* CF_right{nullptr};
};

}  // namespace lems::vo::stereo
