# Getting started {#getting_started}

`lems-data` is a C++17 library. The installed package exports the modern
`lems::data` target and its Eigen, OpenCV, and yaml-cpp dependencies:

```cmake
find_package(lemsData CONFIG REQUIRED)

add_executable(inspect main.cpp)
target_link_libraries(inspect PRIVATE lems::data)
```

The repository itself can be built with:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DLEMS_DATA_BUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix /absolute/path/to/lems-data-stage
```

Use an existing configuration file, or create one with the examples in
@ref datasets_page "Dataset readers". Placeholders such as
`/absolute/path/to/...` are intentionally local paths, not downloadable
fixtures.

## Minimal reader

The following uses only public methods and works for any built-in format whose
configuration is valid:

```cpp
#include <lems/data/config.hpp>
#include <lems/data/dataset.hpp>

#include <filesystem>
#include <iostream>
#include <utility>

int main(int argc, char** argv) {
  if (argc != 2) return 2;

  auto config = lems::data::load_config(std::filesystem::path(argv[1]));
  auto dataset = lems::data::open_dataset(std::move(config));
  auto iterator = dataset->iterate();

  while (auto set = iterator->next()) {
    std::cout << "set " << set->index << " at "
              << set->timestamp_ns << " ns\n";
    for (auto& frame : set->frames) {
      std::cout << "  " << frame.camera << " " << frame.image.cols << "x"
                << frame.image.rows << "\n";
      // Run the consuming project's detector and append to frame.edges.
    }
  }
}
```

`next()` returns `std::nullopt` at end of stream. A call materializes each
returned frame's image and any discovered disparity or mask; the reader does
not return a partially loaded frame. `reset()` starts the same iterator over,
and `has_next()`/`hasNext()` only report whether another value is available.
The iterator's `size()` is the number of values remaining under its configured
skip step, while @ref lems::data::Dataset::size is the number of synchronized
sets before skipping.

## Inspect the model

Useful first calls after construction are:

```cpp
const auto& cfg = dataset->config();
const auto& cameras = dataset->cameras();
const auto& stereo = dataset->stereo_calibration();
const auto& files = dataset->file_info();
const bool poses = dataset->has_ground_truth();
```

Use @ref camera_conventions "Camera conventions" before composing transforms.
In particular, `CameraPose` is world-to-camera, while stereo calibration maps
reference-camera coordinates into target-camera coordinates. Use
`frame.timestamp_ns` for synchronization and `frame.timestamp_seconds` only
when an older floating-point API requires it.

## Linking a legacy VO checkout

The compatibility targets are opt-in migration surfaces. Follow
@ref migration_page "Migration" for the include path, detector alias, source
removals, and profile choice. Do not link both the GPU/multinocular and stereo
profiles into one target: their same-named legacy matching records intentionally
have different layouts.
