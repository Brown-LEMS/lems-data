// Compile this file against a migrated reference repository to verify the
// hand-off from the real TOED detector into the shared data structures.
// Example (from the lems-data checkout):
//   c++ -std=c++17 migration/reference_detector_smoke.cpp \
//     <reference>/src/toed/cpu_toed.cpp -I<reference>/include \
//     -I<lems-data>/include -I<lems-data>/compat/vo \
//     $(pkg-config --cflags --libs opencv4) -I<eigen> -fopenmp \
//     -L<lems-data-build> -llems_data -lyaml-cpp -o detector-smoke
//
// The reference detector header must first be migrated with
// migration/replace_detector_edges.sh --apply. No synthetic Edge is created
// here: every Edge is copied directly from ThirdOrderEdgeDetectionCPU.

#include <lems/data/pipeline_types.hpp>
#include <lems/data/types.hpp>
#include <lems/data/utility.hpp>

#include "toed/cpu_toed.hpp"

#include <opencv2/imgproc.hpp>

#include <iostream>

int main() {
  constexpr int kWidth = 64;
  constexpr int kHeight = 64;

  cv::Mat image(kHeight, kWidth, CV_8UC1, cv::Scalar(0));
  image.colRange(kWidth / 2, kWidth).setTo(cv::Scalar(255));

  ThirdOrderEdgeDetectionCPU detector(kHeight, kWidth);
  detector.get_Third_Order_Edges(image);
  if (detector.toed_edges.empty()) {
    std::cerr << "TOED detector returned no edges for the step image\n";
    return 1;
  }

  lems::data::Frame frame;
  frame.image = image;
  frame.timestamp_seconds = 0.0;
  frame.timestamp_ns = 0;
  frame.K = Eigen::Matrix3d::Identity();
  frame.edges = detector.toed_edges;
  cv::Sobel(image, frame.image_gradients_x, CV_32F, 1, 0, 3);
  cv::Sobel(image, frame.image_gradients_y, CV_32F, 0, 1, 3);

  for (std::size_t i = 0; i < frame.edges.size(); ++i) {
    frame.edges[i].frame_source = 0;
    frame.edges[i].index = static_cast<int>(i);
  }

  lems::data::SpatialGrid grid(kWidth, kHeight);
  for (std::size_t i = 0; i < frame.edges.size(); ++i) {
    grid.addEdge(static_cast<int>(i),
                 cv::Point2f(static_cast<float>(frame.edges[i].location.x),
                             static_cast<float>(frame.edges[i].location.y)));
  }
  const auto nearby = grid.getCandidatesWithinRadius(frame.edges.front(), 35.0);
  if (nearby.empty()) {
    std::cerr << "SpatialGrid did not retain the detector edge\n";
    return 2;
  }

  lems::data::Utility utility;
  const auto patches = utility.get_edge_patches(frame.edges.front(), image);
  if (patches.first.empty() || patches.second.empty()) {
    std::cerr << "Utility did not produce edge patches\n";
    return 3;
  }

  lems::data::Main_Observer observer(&frame);
  observer.edge_indices.push_back(0);
  observer.edges_3D.emplace_back(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), false, 0, 0);
  observer.sift_descriptors.emplace_back(cv::Mat{}, cv::Mat{});
  observer.edge_patches.push_back(patches);
  observer.construct_toed_edge_index_to_edge_indices_map();
  if (!observer.check_observer_validity()) {
    std::cerr << "Main_Observer rejected the shared Frame/Edge hand-off\n";
    return 4;
  }

  std::cout << "detector smoke: " << frame.edges.size()
            << " shared edges, " << nearby.size() << " grid candidates\n";
  return 0;
}
