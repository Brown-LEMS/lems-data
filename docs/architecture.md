# Architecture

The stable boundary is deliberately small:

```text
YAML config -> open_dataset() -> Dataset -> DatasetIterator -> FrameSet
                                                            -> pipeline
trajectory files -> tools/evaluate.py -> evo_ape / evo_rpe / evo_traj
```

`FrameSet` is the unit handed to an algorithm. It contains synchronized observations from all cameras at one logical time. `Frame` owns metadata, not decoded pixels or algorithm outputs. This avoids the coupling in the reference VO repository where dataset types also contain edges, gradients, disparity maps, matching statistics, and detector-specific state.

## Coordinate and time conventions

- All timestamps in the API are signed integer nanoseconds.
- Quaternions are stored as `(w, x, y, z)`.
- `T_body_camera` maps a point in the camera frame into the body frame.
- Image paths are absolute or rooted at the configured dataset directory.
- A missing ground-truth pose is represented by `std::nullopt`, never an identity pose.
- Dense reference products are paths in `FrameMetadata`; they are decoded only when an algorithm needs them.

## Adding a dataset

A reader is responsible for discovering streams, converting timestamps to nanoseconds, synchronizing observations within `sync_tolerance_ns`, and attaching available ground truth. It must not decode images or perform rectification. Initially readers live behind `open_dataset`; the intended next API is a public registry so lab projects can ship readers independently.

## Relationship to Multinocular Edge VO

The retained ideas are the config-driven factory, an abstract resettable iterator, a frame-set return value, and the main-loop shape. The generalized API replaces `hasNext()/getNext(out)` with `std::optional<FrameSet> next()` to make partial/stale output impossible. Pipeline stages and observer/matcher types are not data-layer concerns.
