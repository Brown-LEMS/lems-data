/**
 * @file config.hpp
 * @brief YAML and programmatic dataset configuration.
 * @ingroup datasets
 */
#pragma once
#include "lems/data/types.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <unordered_map>

namespace lems::data {

/**
 * @brief Reader selection, location, synchronization, and calibration policy.
 * @ingroup datasets
 *
 * `skip_frames` controls iterator stepping. `sync_tolerance_ns` is an integer
 * nanosecond matching tolerance and defaults to 20 ms. Unknown scalar YAML
 * values are retained in `options` for reader-specific switches.
 */
struct DatasetConfig {
  std::string type; ///< Built-in reader type: `kitti`, `euroc`, or `eth3d`.
  std::filesystem::path root; ///< Dataset root; relative file paths are resolved by the loader.
  std::string sequence; ///< Optional sequence/subdirectory name.
  std::size_t skip_frames{0}; ///< Iterator step is `skip_frames + 1`.
  std::int64_t sync_tolerance_ns{20'000'000}; ///< Timestamp matching tolerance in nanoseconds.
  std::vector<CameraCalibration> cameras; ///< Caller/native camera calibrations.
  std::optional<StereoCalibration> stereo; ///< Optional reference-to-target stereo calibration.
  std::optional<std::filesystem::path> ground_truth_path; ///< Optional reader-specific pose override.
  std::unordered_map<std::string, std::string> options; ///< Reader-specific scalar options.
};

/**
 * @brief Load a dataset configuration from a YAML file.
 * @ingroup datasets
 * @throws std::runtime_error for malformed mappings, matrices, paths, or
 * invalid calibration values.
 */
DatasetConfig load_config(const std::filesystem::path& path);

/**
 * @brief Load a dataset configuration from a YAML node.
 * @ingroup datasets
 * @param node Mapping containing dataset type and root.
 * @param base Base directory used to resolve relative paths.
 */
DatasetConfig load_config(const YAML::Node& node,
                          const std::filesystem::path& base = {});

} // namespace lems::data
