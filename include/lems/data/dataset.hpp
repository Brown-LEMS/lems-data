/**
 * @file dataset.hpp
 * @brief Dataset and resettable iterator interfaces.
 * @ingroup datasets
 */
#pragma once
#include "lems/data/config.hpp"
#include <memory>
#include <optional>

namespace lems::data {

/**
 * @brief Resettable stream of synchronized frame sets.
 * @ingroup datasets
 *
 * `next()` returns a complete materialized @ref FrameSet or `std::nullopt` at
 * end of stream. Implementations may apply the configured skip step; `size()`
 * reports remaining values under that step, not the original dataset size.
 */
class DatasetIterator {
public:
  virtual ~DatasetIterator() = default;
  /// Return and materialize the next complete frame set, or `std::nullopt`.
  virtual std::optional<FrameSet> next() = 0;
  /// Rewind this iterator to its first frame set.
  virtual void reset() = 0;
  /// Return whether @ref next can return another frame set.
  virtual bool has_next() const = 0;
  /// Legacy camel-case alias for @ref has_next.
  virtual bool hasNext() const { return has_next(); }
  /// Return remaining values under the configured step.
  virtual std::size_t size() const = 0;
};

/**
 * @brief Abstract dataset with calibration, metadata, and iteration access.
 * @ingroup datasets
 *
 * Built-in readers are constructed by @ref open_dataset. A consuming project
 * can implement this interface for another native layout without changing
 * the shared frame, camera, or edge types.
 */
class Dataset {
public:
  virtual ~Dataset() = default;
  /// Return the parsed configuration owned by this dataset.
  virtual const DatasetConfig& config() const = 0;
  /// Return camera calibrations in reader camera order.
  virtual const std::vector<CameraCalibration>& cameras() const = 0;
  /// Return optional target/reference stereo calibration.
  virtual const std::optional<StereoCalibration>& stereo_calibration() const = 0;
  /// Return legacy file identity and timestamp metadata.
  virtual const FileInfo& file_info() const = 0;
  /// Return the populated legacy camera mirror.
  virtual const CameraInfo& camera_info() const = 0;
  /// Return the number of synchronized sets before iterator skipping.
  virtual std::size_t size() const = 0;
  /// Return the decoded image width discovered by the reader.
  virtual int width() const = 0;
  /// Return the decoded image height discovered by the reader.
  virtual int height() const = 0;
  /// Return whether trajectory pose records are available.
  virtual bool has_ground_truth() const = 0;
  /// Create an independent resettable iterator over this dataset.
  virtual std::unique_ptr<DatasetIterator> iterate() const = 0;
};

/**
 * @brief Open a built-in KITTI, EuRoC, or ETH3D reader.
 * @ingroup datasets
 *
 * The configuration is moved into the dataset. Unsupported types or malformed
 * native layouts throw; additional formats should implement @ref Dataset and
 * @ref DatasetIterator in a consuming project.
 */
std::unique_ptr<Dataset> open_dataset(DatasetConfig config);

} // namespace lems::data
