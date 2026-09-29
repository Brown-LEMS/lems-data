# Validation snapshot

Validated locally on 2026-09-25 with AppleClang 17, Eigen 3.4.0,
OpenCV 4.10.0, and yaml-cpp 0.8.0.

| Check | Result |
| --- | --- |
| Release library, readers, configuration, utilities, types, compatibility | 6/6 CTest suites passed |
| Independent installed-package consumers: modern, GPU-profile, stereo-profile | 3/3 passed |
| Shared library with compatibility adapters disabled | 5/5 suites passed |
| Actual GPU-dev TOED detector using shared types | 88 edges, 88 grid candidates |
| Actual stereo-repository TOED detector using shared types | 88 edges, 88 grid candidates |

Reader fixtures cover KITTI color calibration and camera-0 pose offsets,
EuRoC nanosecond timestamps and noncommuting body/camera transforms,
ETH3D SLAM and stereo-pair layouts, native per-pair calibration, temporal
windows, PFM byte order/scale, and disparity/mask dimensions. Release checks
remain active with `NDEBUG` defined.

The detector checks use the actual reference implementations with their
duplicate Edge definitions replaced by shared aliases. They are not full
visual-odometry or CUDA end-to-end tests. No real dataset sequence or remote
GitHub Actions run has been validated in this snapshot.

## Reproduce the library checks

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DLEMS_DATA_BUILD_TESTS=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/stage"
cmake -S tests/installed_consumer -B build-consumer \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/stage"
cmake --build build-consumer --parallel 2
ctest --test-dir build-consumer --output-on-failure
```

Add dependency installation prefixes to `CMAKE_PREFIX_PATH` when they are
not installed in standard system locations. See [vo-migration.md](vo-migration.md)
for the explicit changes required in either VO repository.
