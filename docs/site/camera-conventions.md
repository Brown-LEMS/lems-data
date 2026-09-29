# Camera conventions {#camera_conventions}

The public API keeps Eigen matrices and vectors in their source form. Images,
masks, and conversion helpers use OpenCV. Read the direction of every
transform from the field name and operation below before composing a pose.

## Camera pose

@ref lems::data::CameraPose stores a world-to-camera rigid transform:

```text
p_camera = R * p_world + t
T_camera_world = [ R  t ]
T_world_camera = [ R^T  -R^T t ]
camera_center_in_world = -R^T t
```

Therefore:

* `transform(point)` applies `R * point + t` (world to camera).
* `detransform(point)` applies `R.transpose() * (point - t)` (camera to
  world).
* `rotate(point)` applies only `R`.
* `center()` returns the camera center in world coordinates.
* `matrix3x4()`, `matrix()`, and `inverse_matrix()` expose the same transform
  without changing convention.

`q` is normalized by the quaternion constructors. `quat_to_R()` returns the
rotation represented by the normalized quaternion; it does not alter `R`.
The timestamped constructors preserve `timestamp_ns` as signed nanoseconds.

## Intrinsics and stereo

@ref lems::data::CameraCalibration contains:

* `K`, the 3x3 pinhole camera matrix;
* `intrinsics = [fx, fy, cx, cy]`;
* `distortion`, an Eigen vector retaining native coefficients;
* `R_rect`, the native rectification matrix;
* `P`, a 3x4 projection matrix; and
* `T_body_camera`, the body-from-camera extrinsic used by the readers.

The field name is historical: the implementation composes
`T_body_camera` as camera coordinates -> body coordinates (`body <- camera`),
and uses its inverse for body -> camera. Keep that direction explicit when
combining EuRoC body poses or a frame-to-body transform. The
`camera_matrix_cv()` and `distortion_cv()` methods make OpenCV copies for APIs
that require `cv::Mat`.

@ref lems::data::StereoCalibration uses the unambiguous target/reference
direction:

```text
p_target = R_target_reference * p_reference + t_target_reference
F_target_reference = K_target^-T [t_target_reference]x
                     R_target_reference K_reference^-1
```

`baseline` is the Euclidean norm of `t_target_reference`. The inverse
reference-from-target transform is
`R_target_reference.transpose()` and
`-R_target_reference.transpose() * t_target_reference`. Do not infer a
translation sign from a camera filename; use the stored direction.

@ref lems::data::CameraInfo and the legacy @ref lems::data::Camera mirror
selected calibration and relative stereo values for older Brown-LEMS code.
New code should prefer `Dataset::cameras()` and
`Dataset::stereo_calibration()`.

## Time and dense metadata

@ref lems::data::Timestamp is `std::int64_t` nanoseconds. `Frame::timestamp_ns`
and `FrameSet::timestamp_ns` are authoritative for synchronization and pose
association. `Frame::timestamp_seconds` and the legacy `Frame::timestamp`
are `double` convenience values; `FileInfo::GT_time_stamps` and
`Img_time_stamps` are also double seconds. Convert back to integer time only
when the precision loss is acceptable.

`Frame::ground_truth` is optional. `has_ground_truth` and the legacy
`gt_camera_pose` are synchronized by `Frame::synchronize_legacy_fields()`;
an absent pose is distinct from an explicitly stored identity pose. Dense
disparity, occlusion-mask, and depth paths live in `FrameMetadata`; decoded
matrices are populated when the iterator materializes a frame. The legacy
disparity and mask members can shallow-share those matrices.
