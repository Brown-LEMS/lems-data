# lems-data

@htmlonly
<div class="lems-hero">
  <p><code>lems-data</code> is the small, format-aware data boundary shared by Brown-LEMS
  visual-odometry pipelines. It owns dataset discovery, timestamped frames,
  Eigen/OpenCV calibration, optional dense products, camera poses, and the edge
  records consumed by downstream matching code. The detector, CUDA kernels,
  matching policy, GUI, and evo remain in the consuming project.</p>
</div>
@endhtmlonly

@htmlonly
<div class="lems-card-grid">
  <div class="lems-card">
    <h2><a href="group__datasets.html">Datasets</a></h2>
    <p>Load KITTI, EuRoC, and ETH3D into synchronized frame sets.</p>
  </div>
  <div class="lems-card">
    <h2><a href="group__cameras.html">Cameras</a></h2>
    <p>Keep K, R, t, F, distortion, poses, and timestamps explicit.</p>
  </div>
  <div class="lems-card">
    <h2><a href="group__edges.html">Edges</a></h2>
    <p>Share stable edge identity across detector and matcher stages.</p>
  </div>
  <div class="lems-card">
    <h2><a href="group__utilities.html">Utilities</a></h2>
    <p>Use the retained Eigen/OpenCV geometry and image helpers.</p>
  </div>
  <div class="lems-card">
    <h2><a href="group__pipeline.html">Pipeline records</a></h2>
    <p>Pass data-only grids, observers, clusters, and temporal matches.</p>
  </div>
  <div class="lems-card">
    <h2><a href="group__compatibility.html">Compatibility</a></h2>
    <p>Migrate old Brown-LEMS targets through an explicit profile.</p>
  </div>
</div>
@endhtmlonly

The primary flow is:

```text
YAML or native calibration
          |
          v
lems::data::load_config -> lems::data::open_dataset
          |
          v
lems::data::Dataset + calibration metadata
          |
          v
lems::data::DatasetIterator::next -> lems::data::FrameSet
          |
          +--> detector fills Frame::edges
          +--> pipeline matches shared edge/pipeline records
```

## Start here

* @ref getting_started "Getting started" — build, link, and iterate a dataset.
* @ref datasets_page "Dataset readers" — native layouts, calibration, and metadata.
* @ref camera_conventions "Camera conventions" — transform directions and time precision.
* @ref edge_pipeline "Edge and pipeline data" — identity, ownership, and N-view records.
* @ref utilities_page "Utilities" — inputs, outputs, and degenerate cases.
* @ref migration_page "Migration" — compatibility targets and behavior changes.

## Public entry points

The modern API is intentionally small:

* @ref lems::data::DatasetConfig and @ref lems::data::load_config describe a
  dataset root, sequence, synchronization tolerance, cameras, and optional
  ground truth.
* @ref lems::data::open_dataset constructs the built-in reader selected by
  `DatasetConfig::type`.
* @ref lems::data::Dataset exposes calibrated cameras, stereo calibration,
  file metadata, dimensions, pose availability, and @ref lems::data::Dataset::iterate.
* @ref lems::data::DatasetIterator::next returns an optional
  @ref lems::data::FrameSet; its `Frame` values carry decoded grayscale images,
  authoritative nanosecond timestamps, paths, K, optional poses, and metadata.
* @ref lems::data::Edge and @ref lems::data::Edge_3D are the shared detector
  records; @ref lems::data::Utility contains the retained numerical helpers.

All public names are in the `lems::data` namespace. The generated class and
file pages are the reference for signatures; these guide pages explain the
contracts that are easiest to misuse.

## Coordinate and ownership snapshot

`CameraPose::R` and `CameraPose::t` are a world-to-camera transform:
`p_camera = R * p_world + t`. `StereoCalibration::R_target_reference` and
`t_target_reference` map a point from the reference camera to the target
camera. `Timestamp` is signed integer nanoseconds; floating-point seconds
are convenience and legacy views only. OpenCV `cv::Mat` values use
reference-counted, shallow copies, so clone a matrix before mutating it when
an isolated buffer is required.

For the repository's implementation notes and validation scope, see the
source-tree architecture and validation documents included with the checkout.
