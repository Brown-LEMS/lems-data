# lems-data

`lems-data` is the shared Brown-LEMS C++17 data layer for stereo and
multi-camera edge-odometry pipelines. It owns the parts that should be
identical in every lab repository: dataset discovery, timestamped frames,
Eigen/OpenCV calibration, dense reference products, camera poses, and the
edge records consumed by the matching pipeline.

The supported built-in readers are KITTI odometry, EuRoC MAV, and ETH3D
stereo/SLAM layouts. A reader returns synchronized `FrameSet` values; every
frame carries its image path, decoded grayscale image, camera name and
timestamp, optional ground-truth pose, calibration `K`, and disparity/depth/
occlusion-mask metadata when the dataset provides it.

## Build and install

Dependencies are Eigen 3.3+, OpenCV (`core`, `imgcodecs`, and `imgproc`),
yaml-cpp, and a C++17 compiler:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DLEMS_DATA_BUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$HOME/.local"
```

An installed consumer uses the exported targets, so transitive Eigen,
OpenCV, and yaml-cpp dependencies are found by the package config:

```cmake
find_package(lemsData CONFIG REQUIRED)
add_executable(my_vo main.cpp)
target_link_libraries(my_vo PRIVATE lems::data)
```

The legacy adapters are opt-in at configure time
(`LEMS_DATA_BUILD_COMPAT=ON`, the default). The GPU/multinocular profile is
exported as `lems::vo_compat`; the stereo profile is exported as
`lems::stereo_compat`. Both install unambiguous forwarding headers below
`include/lems/vo`.

## Basic pipeline

```cpp
#include <lems/data/config.hpp>
#include <lems/data/dataset.hpp>

auto config = lems::data::load_config("configs/kitti.yaml");
auto dataset = lems::data::open_dataset(std::move(config));
auto iterator = dataset->iterate();

while (auto set = iterator->next()) {
  for (auto& frame : set->frames) {
    const Eigen::Matrix3d& K = frame.K;
    const cv::Mat& image = frame.image;
    // Run the lab's edge detector and store its Edge values in frame.edges.
    // Dense reference data is available as frame.metadata.disparity and
    // frame.metadata.occlusion_mask when present.
    (void)K;
    (void)image;
  }
}
```

`FrameSet::timestamp_ns` and `Frame::timestamp_ns` are integer nanoseconds;
`timestamp_seconds` is provided for the older iterator API. Poses are
explicitly optional. Camera `K`, relative stereo `R`/`t`, fundamental
matrices, distortion, rectification and projection matrices remain Eigen
types; OpenCV conversion helpers are provided on `CameraCalibration`.

## Edge and legacy VO integration

The public namespaced edge types are the one source of truth:
`lems::data::Edge` and `lems::data::Edge_3D`. They preserve the fields and
sentinel/default behavior used by the TOED detector, including location,
orientation, `b_isEmpty`, `frame_source`, and `index`, plus the Eigen 3-D
location/tangent representation. The canonical `Frame` also retains the
legacy edge, gradient, disparity, mask, pose, and calibration fields so the
pipeline can be migrated incrementally without copying into a second data
model.

For an existing Brown-LEMS VO checkout, link `lems::vo_compat`, put its
compatibility include directory before the old include directory, remove the
old dataset/iterator/utility translation units from that target, and update
the detector's duplicate `Edge` declaration to include the shared edge header
and use the shared type. The migration is intentionally explicit: relative
includes and the old stereo matching containers cannot be made safe by an
include-path trick. See [docs/vo-migration.md](docs/vo-migration.md).

## Evaluation and visualization

The repository keeps the evo integration as a thin external-process wrapper;
it does not vendor evo. This keeps the library focused and lets a lab choose
upstream evo or the lab's `evo_lems` fork:

```bash
python tools/evaluate.py ape kitti groundtruth.txt estimate.txt \
  --align origin --plot --save-results results/ape.zip
```

See [docs/architecture.md](docs/architecture.md) for ownership and
coordinate conventions, and [docs/vo-migration.md](docs/vo-migration.md) for
the exact adapter boundary.

The data contracts were extracted from Brown-LEMS
`Multinocular-Edge-Visual-Odometry`, branch `GPU_dev`, source commit
`2ec5e72fa47aff6ee663c2b072c7d161c4e76595`, and the stereo
`Edge_Based_Visual_Odometry` pipeline. The original repositories remain the
authority for algorithm-specific matching behavior; this library does not
silently reimplement those pipeline stages.
