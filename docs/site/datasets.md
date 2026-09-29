# Dataset readers {#datasets_page}

@ref lems::data::load_config accepts a YAML mapping or a YAML file and returns
@ref lems::data::DatasetConfig. Required top-level values are `type` (or
`dataset_type`) and `root` (or `dataset_dir`). `sequence` and
`sequence_name` are aliases. Relative paths in a file are resolved against
the configuration file's directory; a configured ground-truth path is then
resolved against the dataset root when it is relative.

The config object contains:

* `type`, `root`, and `sequence` — reader selection and native location.
* `skip_frames` — a non-negative iterator step setting; it does not reduce
  @ref lems::data::Dataset::size.
* `sync_tolerance_ns` — non-negative timestamp matching tolerance, default
  `20'000'000` (20 ms).
* `cameras` and optional `stereo` — caller-supplied calibration. Each reader
  combines these with its native files according to the format-specific rules
  below.
* `ground_truth_path` — optional override; format and coordinate conversion
  follow the selected reader.
* `options` — scalar reader options such as `kitti_camera_pair` and
  `num_cameras`.

Invalid mappings, negative skip/tolerance, malformed matrices, non-finite
values, non-rigid rotations, non-positive image dimensions, and singular K
matrices throw `std::runtime_error` (or a related standard exception).

## KITTI odometry

The reader looks for a sequence under either
`<root>/<sequence>/` or `<root>/sequences/<sequence>/`. A complete pair is
`image_0`/`image_1` (gray) or `image_2`/`image_3` (color). `calib.txt` is
parsed for the selected projection matrices; the default is gray when it is
available, and `kitti_camera_pair: color` selects P2/P3. `times.txt`, when
present, supplies seconds that are converted to integer nanoseconds. Without
it, the reader uses the sample index as seconds. Left/right filenames must
match exactly and image streams must have equal counts.

The default pose path is `<root>/poses/<sequence>.txt`, or a configured
`ground_truth_path` (a directory is completed with `<sequence>.txt`). KITTI
poses are expressed in the camera-0 reference frame; the reader applies the
selected P2/P3 camera offset and emits world-to-camera @ref
lems::data::CameraPose values. Stereo `R`, `t`, `F`, and baseline are derived
from the selected projections.

Example local configuration:

```yaml
dataset_type: kitti
dataset_dir: /absolute/path/to/KITTI/odometry/dataset
sequence_name: "00"
kitti_camera_pair: gray
skip_frames: 0
ground_truth_path: /absolute/path/to/KITTI/odometry/poses/00.txt
```

The path is a placeholder: the reader does not download or synthesize a
sequence.

## EuRoC MAV

The reader accepts a root containing `mav0/`, or a root/sequence containing
`mav0/`. It reads `cam0/data.csv`, `cam1/data.csv`, and each camera's
`sensor.yaml`. EuRoC image CSV timestamps are already nanoseconds. Samples
are paired by nearest unused timestamp within `sync_tolerance_ns`; unmatched
images are omitted rather than duplicated. The default pose path is
`mav0/state_groundtruth_estimate0/data.csv`. Ground-truth timestamps are
nanoseconds and EuRoC's body-to-world records are converted to the public
world-to-camera pose convention, including the body/camera extrinsics.

Example local configuration:

```yaml
dataset_type: euroc
dataset_dir: /absolute/path/to/EuRoC
sequence_name: MH_01_easy
sync_tolerance_ns: 20000000
ground_truth_path: /absolute/path/to/EuRoC/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv
skip_frames: 0
```

Native sensor calibration is used when no `cameras` list is supplied. A YAML
calibration supplied by the caller remains authoritative for the fields it
contains.

## ETH3D

The reader supports the two layouts implemented in this checkout:

1. A stereo-pair layout with `stereo_pairs/<pair>/im0.png` and `im1.png`.
   Each pair can carry its native `calib.txt`; optional `disp0GT.pfm`,
   `disp1GT.pfm`, `mask0nocc.png`, and `mask1nocc.png` are attached to the
   corresponding frames. Pair calibration stores camera matrices, image
   dimensions, and a baseline in millimetres; the reader converts the baseline
   to metres and derives the target-from-reference stereo transform.
2. An SLAM layout with `rgb.txt` and either `rgb2.txt` or a sibling `rgb2/`
   image directory. Timestamps are decimal seconds and are converted to
   nanoseconds. `groundtruth.txt` uses TUM timestamp/translation/quaternion
   rows; its camera-to-world poses are inverted for the public pose field.

For the SLAM layout, native global calibration may be read from
`calibration.txt`/`calibration2.txt`, `calib.txt`, and
`extrinsics_1_2.txt`. The latter is interpreted as CAMERA2 -> CAMERA1 and is
inverted to produce target-camera <- reference-camera. Supplied YAML
calibration takes precedence where present.

`num_cameras` requests complete sliding N-view windows. For N cameras the
reader consumes `(N + 1) / 2` consecutive stereo pairs; an odd N ends with a
left frame. Partial windows are discarded. The native stereo-pair layout
also reads per-pair calibration when available and only uses global fallback
calibration when both cameras and stereo calibration were explicitly supplied.

Example local configuration:

```yaml
dataset_type: eth3d
dataset_dir: /absolute/path/to/ETH3D
sequence_name: delivery_area
num_cameras: 2
sync_tolerance_ns: 20000000
ground_truth_path: /absolute/path/to/ETH3D/delivery_area/groundtruth.txt
skip_frames: 0
```

This documents the supported pair/SLAM layouts; it is not a promise to detect
arbitrary ETH3D directory variants. Disparity must be a single-channel image
or PFM with image-matching dimensions. Missing dense files are represented by
empty metadata, not fabricated zeros.

## Frame and iterator contract

@ref lems::data::Dataset::iterate creates an independent resettable iterator.
Each @ref lems::data::FrameSet contains synchronized observations; consumers
must inspect `Frame::camera` and `Frame::timestamp_ns` instead of assuming a
two-frame stereo vector. The image is decoded as grayscale when `next()`
materializes a frame. `Frame::image_path` and optional metadata paths remain
available before materialization in the stored frame copy.

`cv::Mat` uses OpenCV's reference-counted shallow-copy semantics. The
canonical metadata matrices and legacy `disparity_map`/`occlusion_mask` can
therefore share their pixel buffers. `Frame::synchronize_legacy_fields`
updates the legacy timestamp, dense aliases, pose flag, and pose value from
the canonical fields; call it after manually changing canonical metadata.

`Dataset::has_ground_truth()` answers whether pose records are available at the
dataset level. The compatibility adapter's legacy `Dataset::has_gt()` has a
different dense-reference meaning; see @ref migration_page "Migration".
