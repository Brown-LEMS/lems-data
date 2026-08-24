# Architecture

The stable boundary is deliberately small:

```text
YAML config -> open_dataset() -> Dataset -> DatasetIterator -> FrameSet
                                                            -> pipeline
trajectory files -> tools/evaluate.py -> evo_ape / evo_rpe / evo_traj
```

`FrameSet` is the unit handed to an algorithm. It contains synchronized observations from all cameras at one logical time. `Frame` owns decoded input pixels and dataset-provided reference data, but not algorithm outputs such as edges, gradients, matches, or GPU buffers. The compatibility layer adds the old algorithm-owned fields only for source compatibility.

## Coordinate and time conventions

- All timestamps in the API are signed integer nanoseconds.
- Quaternions are stored as `(w, x, y, z)`.
- `T_body_camera` maps a point in the camera frame into the body frame.
- Image paths are absolute or rooted at the configured dataset directory.
- A missing ground-truth pose is represented by `std::nullopt`, never an identity pose.
- Dense reference products retain their paths and are decoded into OpenCV matrices during iteration.

## Adding a dataset

A reader is responsible for discovering streams, converting timestamps to nanoseconds, synchronizing observations within `sync_tolerance_ns`, decoding dataset inputs, and attaching available ground truth. Rectification remains a pipeline operation because some algorithms require raw images. Initially readers live behind `open_dataset`; the intended next API is a public registry so lab projects can ship readers independently.

## Relationship to Multinocular Edge VO

The retained ideas are the config-driven factory, an abstract resettable iterator, a frame-set return value, and the main-loop shape. The generalized API replaces `hasNext()/getNext(out)` with `std::optional<FrameSet> next()` to make partial/stale output impossible. Pipeline stages and observer/matcher types are not data-layer concerns.
