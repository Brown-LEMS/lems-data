# Architecture

The library boundary is the data path used by both Brown-LEMS VO
repositories:

```text
YAML/native calibration files
        |
        v
  open_dataset(config)
        |
        v
 Dataset + FileInfo + CameraInfo
        |
        v
 resettable DatasetIterator
        |
        v
 FrameSet (one synchronized camera observation per frame)
        |
        +--> Edge detector --> Frame::edges / Frame::gradients
        +--> stereo/temporal matching --> pipeline-owned containers
        +--> evo wrapper / visualization
```

## Core ownership

`DatasetConfig` describes the input layout and synchronization policy.
`Dataset` owns the parsed configuration and exposes its populated
`FileInfo`, `CameraInfo`, camera calibration list, stereo calibration, stream
size, and ground-truth availability. `DatasetIterator::next()` returns an
optional `FrameSet`; a missing value is end-of-stream and no partially loaded
frame is discarded or returned.

`FrameSet` is a synchronized group. A stereo reader returns the left and right
camera observations for one timestamp. A multi-camera or temporal-window
reader may return more than two observations; consumers must use each frame's
`camera` and `timestamp_ns`, not assume that every vector is a stereo pair.

Each `Frame` carries decoded OpenCV input, the source path, camera identity,
authoritative integer nanosecond time, optional pose, `K`, and optional dense
metadata paths/matrices. The same canonical frame also exposes the legacy
gradients, edge vector, disparity map, occlusion mask and `CameraPose` fields
used by the existing pipeline; iterators synchronize those legacy fields from
the canonical metadata (OpenCV matrices may shallow-share their buffers).

`Edge` and `Edge_3D` are namespaced library types. The TOED detector should
populate `Frame::edges`; edge matching and GPU buffers remain algorithm-owned.
`pipeline_types.hpp` contains the data-only spatial grids, edge clusters,
stereo edge-pair records and temporal records extracted from the old
`Dataset.h`. It does not own a second detector or dataset implementation.

## Calibration and coordinate conventions

- Matrices and vectors in the public API are Eigen types; images and masks are
  OpenCV types.
- `CameraCalibration::K` is the 3x3 pinhole matrix. `intrinsics` is
  `[fx, fy, cx, cy]`; `distortion`, `R_rect`, `P`, and body extrinsics are
  retained independently.
- `StereoCalibration::R_target_reference` and
  `t_target_reference` map a point from the reference camera into the target
  camera. The inverse is `R.transpose()` and
  `-R.transpose()*t`.
- `CameraPose::R` and `t` represent the same world/camera transform as the
  source pipeline's `Camera_Pose`; use its `transform`, `detransform`, and
  `center` helpers rather than guessing a convention.
- API timestamps are signed nanoseconds. `timestamp_seconds` is an approximate
  floating-point convenience for older iterator code; `timestamp_ns` is
  authoritative.
- A missing pose is `std::nullopt`; an identity pose is a real identity pose.
- Optional disparity and occlusion paths are retained when the reader finds
  the corresponding files, and decoded matrices are materialized when a
  frame is iterated. Missing files remain empty. `has_ground_truth()` reports
  pose availability at the dataset level; the legacy `has_gt()` adapter keeps
  the dense-reference meaning for old VO code.

Native convention references used when implementing the readers are the
[EuRoC MAV dataset page](https://projects.asl.ethz.ch/datasets/euroc-mav/),
[ETH3D SLAM documentation](https://www.eth3d.net/slam_documentation), and
[ETH3D stereo documentation](https://www.eth3d.net/documentation). In
particular, ETH3D SLAM's camera poses are camera-to-world and are inverted for
the frame transform, while the stereo calibration/extrinsics are interpreted
according to the published camera-pair convention.

## Dataset reader responsibilities

The built-ins normalize each native layout into the same contract:

- KITTI odometry reads `image_0`, `image_1`, `times.txt`, and `calib.txt`,
  retaining the selected camera projection matrices and deriving stereo `R`,
  `t`, `F`, and baseline from the selected pair.
- EuRoC reads `mav0/cam{0,1}/data.csv` and `sensor.yaml`, synchronizes camera
  timestamps within the configured tolerance, and associates ground truth
  without reusing one pose for multiple unmatched frames.
- ETH3D reads stereo-pair folders or the RGB/RGB2 SLAM layout, PFM disparity,
  and non-occlusion masks while preserving source metadata.

Readers are resettable and do not prefetch by advancing past the last valid
window. New formats should implement `Dataset`/`DatasetIterator` and leave
the core edge and calibration types unchanged.

## What stays outside this package

TOED's numerical detector implementation, CUDA kernels, matching policy,
observer callbacks, and evo itself are not hidden in the data library. The
utility API contains the stable Eigen/OpenCV geometry and image helpers used by
those pipelines. `tools/evaluate.py` invokes an installed evo/evo_lems
command so plotting/evaluation versions remain under experiment control.
