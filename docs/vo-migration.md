# Replacing the VO dataset modules

`lems-data` now exposes Eigen/OpenCV-native core types and a compatibility target for the two Brown-LEMS repositories:

- `Edge_Based_Visual_Odometry`
- `Multinocular-Edge-Visual-Odometry` (`GPU_dev`)

## CMake integration

Add this repository as a submodule, for example at `third_party/lems-data`, then add:

```cmake
add_subdirectory(third_party/lems-data)
target_link_libraries(your_vo_target PRIVATE lems::vo_compat)
target_include_directories(your_vo_target BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lems-data/compat/vo)
```

Remove the old `src/Dataset.cpp` and `src/Stereo_Iterator.cpp` or `src/Multinocular_Iterator.cpp` from the VO target. The compatibility directory provides `Dataset.h`, `Stereo_Iterator.h`, and `Multinocular_Iterator.h` with the names the existing source already includes.

The `BEFORE` keyword matters for ordinary `#include "Dataset.h"` calls. A few command files in the reference repositories use an explicit relative include such as `#include "../include/Dataset.h"`; change those to `#include "Dataset.h"`, or replace the old local header with `compat/vo/Dataset.h`. Once migration is verified, delete the superseded local dataset and iterator source files.

## What remains unchanged in the VO code

Existing calls remain available:

```cpp
Dataset::Ptr dataset = std::make_shared<Dataset>(config_map);
dataset->load_dataset(dataset->get_dataset_type(), ...);

Eigen::Matrix3d K = dataset->get_left_calib_matrix();
Eigen::Matrix3d F = dataset->get_fund_mat_21();
Eigen::Matrix3d R21 = dataset->get_relative_rot_left_to_right();
Eigen::Vector3d T21 = dataset->get_relative_transl_left_to_right();
cv::Mat distortion = dataset->get_left_dist_coeff_mat();
```

Stereo iteration remains:

```cpp
while (dataset->stereo_iterator->hasNext()) {
  StereoFrame frame;
  dataset->stereo_iterator->getNext(frame);
  pipeline.current_frame = std::move(frame);
}
```

Multinocular iteration remains:

```cpp
while (dataset->multinocular_iterator->hasNext()) {
  std::vector<Frame> frames;
  dataset->multinocular_iterator->getNext(frames);
  pipeline.current_frame_frames = std::move(frames);
}
```

## Ownership boundary

Dataset inputs stay shared: images, timestamps, calibration, ground-truth pose, disparity, and masks. Algorithm results stay in each VO pipeline: undistorted images, gradients, TOED edges, matches, and GPU storage. The compatibility structs temporarily contain those algorithm fields so current pipeline code compiles; new code should keep them in pipeline-owned state.
