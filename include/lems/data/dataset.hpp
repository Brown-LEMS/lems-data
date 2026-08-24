#pragma once
#include "lems/data/config.hpp"
#include <memory>
#include <optional>

namespace lems::data {

class DatasetIterator {
public:
  virtual ~DatasetIterator() = default;
  virtual std::optional<FrameSet> next() = 0;
  virtual void reset() = 0;
};

class Dataset {
public:
  virtual ~Dataset() = default;
  virtual const DatasetConfig& config() const = 0;
  virtual const std::vector<CameraCalibration>& cameras() const = 0;
  virtual const std::optional<StereoCalibration>& stereo_calibration() const = 0;
  virtual std::unique_ptr<DatasetIterator> iterate() const = 0;
};

// Built-ins: kitti, euroc, eth3d. New datasets register without changing consumers.
std::unique_ptr<Dataset> open_dataset(DatasetConfig config);

} // namespace lems::data
