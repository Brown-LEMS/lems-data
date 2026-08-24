# LEMS Data

A small C++17 data contract for stereo/multi-camera odometry research. It gives pipelines one API for synchronized `FrameSet`s while keeping dataset conventions, image decoding, and algorithms separate.

Built-in readers currently cover KITTI odometry (`image_0`, `image_1`, `times.txt`), EuRoC MAV (`mav0/cam{0,1}/data.csv`), and ETH3D stereo pairs or SLAM-style `rgb`/`rgb2` folders. Images are represented as paths and loaded lazily by the consuming pipeline, so the core has no OpenCV, CUDA, ROS, or Eigen dependency.

Camera metadata includes `K`, rectification and projection matrices, distortion, resolution, body extrinsics, and stereo `R`, `t`, `F`, and baseline. Calibration can come from the existing lab YAML schema; KITTI `calib.txt` is detected automatically and takes precedence. ETH3D ground-truth disparity and non-occlusion masks are attached lazily to each camera frame.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
./build/lems-data-inspect configs/kitti.yaml
```

## Pipeline integration

```cpp
auto dataset = lems::data::open_dataset(lems::data::load_config("experiment.yaml"));
auto frames = dataset->iterate();
while (auto set = frames->next()) {
  // create/load camera images using your preferred backend
  pipeline.add_frame(*set);
}
```

Metadata is available without decoding the asset:

```cpp
const auto& K = dataset->cameras().at(0).K;
const auto& stereo = dataset->stereo_calibration();
for (const auto& frame : set->frames) {
  if (frame.metadata.disparity_path) load_pfm(*frame.metadata.disparity_path);
  if (frame.metadata.occlusion_mask_path) load_mask(*frame.metadata.occlusion_mask_path);
}
```

This mirrors the useful flow in Multinocular Edge VO—configuration → dataset factory → synchronized iterator → pipeline—but does not put edge detection, matching, or GPU buffers in the data model.

See [docs/architecture.md](docs/architecture.md) for the data boundary, time/pose conventions, and rationale.

## Evaluation and visualization

Install the lab evo fork (or upstream evo if lab-only flags are unnecessary), then use the stable wrapper:

```bash
python tools/evaluate.py ape kitti groundtruth.txt estimate.txt --align origin --plot --save-results results/ape.zip
python tools/evaluate.py rpe euroc groundtruth.csv estimate.txt --align se3 --delta 1
python tools/evaluate.py traj tum groundtruth.txt run1.txt run2.txt --align origin --plot
```

The wrapper intentionally shells out to evo instead of copying GPL evo source into this library. That keeps licenses and upgrades clean while exposing APE, RPE, trajectory plotting, origin/Horn alignment, and Sim(3) scale correction.

## Next extension points

- Add ground-truth pose parsing/association to `Frame::ground_truth`.
- Add a public reader registry for out-of-tree dataset plugins.
- Generalize ETH3D beyond paired cameras using explicit stream entries in config.
