#pragma once
#include "lems/data/types.hpp"
#include <filesystem>
#include <string>
#include <unordered_map>

namespace lems::data {

struct DatasetConfig {
  std::string type;
  std::filesystem::path root;
  std::string sequence;
  std::size_t skip_frames{0};
  std::int64_t sync_tolerance_ns{20'000'000};
  std::vector<Camera> cameras;
  std::optional<StereoCalibration> stereo;
  std::optional<std::filesystem::path> ground_truth_path;
  std::unordered_map<std::string, std::string> options;
};

// Small, dependency-free YAML subset reader for the shipped flat configuration files.
DatasetConfig load_config(const std::filesystem::path& path);

} // namespace lems::data
