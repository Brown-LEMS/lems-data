# LEMS Data

A small C++17 data contract for stereo/multi-camera odometry research. It gives pipelines one API for synchronized `FrameSet`s while keeping dataset conventions, image decoding, and algorithms separate.

Built-in readers cover KITTI odometry (`image_0`, `image_1`, `times.txt`), EuRoC MAV (`mav0/cam{0,1}/data.csv`), and ETH3D stereo pairs or SLAM-style `rgb`/`rgb2` folders. The public API deliberately uses the same Eigen and OpenCV types as the Brown-LEMS stereo and multinocular VO pipelines, so their dataset modules can be replaced without converting every calibration matrix and image.

Camera metadata includes `Eigen::Matrix3d K`, rectification and projection matrices, distortion, resolution, body extrinsics, and stereo `R`, `t`, `F`, and baseline. Calibration comes from the existing lab YAML schema, native KITTI `calib.txt`, or EuRoC `sensor.yaml`. Iterators return decoded grayscale `cv::Mat` images; ETH3D disparity PFM files and non-occlusion masks are decoded into each frame alongside their source paths.

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
  // set->frames[i].image is already a cv::Mat
  pipeline.add_frame(*set);
}
```

Both decoded data and its source path are available:

```cpp
const auto& K = dataset->cameras().at(0).K;
const auto& stereo = dataset->stereo_calibration();
for (const auto& frame : set->frames) {
  if (!frame.metadata.disparity.empty()) use(frame.metadata.disparity);
  if (!frame.metadata.occlusion_mask.empty()) use(frame.metadata.occlusion_mask);
}
```

This mirrors the useful flow in Multinocular Edge VO—configuration → dataset factory → synchronized iterator → pipeline—but does not put edge detection, matching, or GPU buffers in the data model.

See [docs/architecture.md](docs/architecture.md) for the data boundary, time/pose conventions, and rationale.
For replacing the dataset files in either existing VO repository, see [docs/vo-migration.md](docs/vo-migration.md).

## Evaluation and visualization

Install the lab evo fork (or upstream evo if lab-only flags are unnecessary), then use the stable wrapper:

```bash
python tools/evaluate.py ape kitti groundtruth.txt estimate.txt --align origin --plot --save-results results/ape.zip
python tools/evaluate.py rpe euroc groundtruth.csv estimate.txt --align se3 --delta 1
python tools/evaluate.py traj tum groundtruth.txt run1.txt run2.txt --align origin --plot
```

The wrapper intentionally shells out to evo instead of copying GPL evo source into this library. That keeps licenses and upgrades clean while exposing APE, RPE, trajectory plotting, origin/Horn alignment, and Sim(3) scale correction.

## Next extension points

- Add a public reader registry for out-of-tree dataset plugins.
- Add explicit arbitrary camera stream declarations for datasets beyond ETH3D's alternating stereo-pair layout.
