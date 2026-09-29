# Utilities {#utilities_page}

The utility layer keeps the numerical helpers used by the Brown-LEMS
pipelines. It deliberately excludes GUI, detector, SIFT/xfeatures2d, CUDA,
and OpenMP-specific code. None of these functions detects edges for you.

## Options and failure values

@ref lems::data::UtilityOptions defaults to a 7x7 odd patch,
`orthogonal_shift = 5.0`, and `epsilon = 1e-10`. The
@ref lems::data::Utility constructor clamps a patch size below one to one,
rounds an even size up to the next odd size, and restores defaults for an
invalid shift or epsilon.

Unless a function below says otherwise, invalid geometry is represented by a
NaN scalar or an all-NaN `Eigen::Vector3d`; degenerate tangent projections
return the zero vector. These are algorithm-visible outcomes, not thrown
exceptions.

## Geometry and epipolar helpers

* `Utility::get_Skew_Symmetric_Matrix(t)` returns `[t]x`; the camel-case
  `getSkewSymmetricMatrix` spelling is an alias.
* `getNormalDistance2EpipolarLine` projects an `[a,b,c]` line onto a point
  and writes the projected point to its output references. A near-zero
  `a*a+b*b` returns NaN and echoes the point. The packed-vector overload
  requires a `[x,y,theta]` stride and a valid index; otherwise all outputs are
  NaN.
* `getTangentialDistance2EpipolarLine` intersects an epipolar line with the
  edge tangent described by `[x,y,theta]`. Parallel/near-parallel lines
  return NaN outputs; the packed overload has the same stride validation.
* `backproject_2D_point_to_3D_point_using_rays` solves
  `R*(rho1*ray1) + T = rho2*ray2`; rank-deficient or non-finite systems
  return an all-NaN vector.
* `reconstruct_3D_Tangent_through_intersection_of_planes` and
  `project_3D_Tangent_to_2D_Tangent` normalize their result. A zero or
  non-finite norm returns `Eigen::Vector3d::Zero()`.
* `get_Relative_Pose(source, target)` composes target world-to-camera with the
  inverse of source world-to-camera and carries the target timestamp.

## Edge patches and image helpers

`get_Orthogonal_Shifted_Points` offsets an edge by the configured or supplied
distance using its radian orientation. `get_patch_on_one_edge_side` writes
rotated coordinate and value matrices; out-of-image samples are NaN. It
accepts grayscale or converts a multi-channel image to grayscale.

`get_edge_patches` returns the plus/minus side patches as `CV_32F` matrices.
An empty image returns an empty pair; the `debug` parameter is retained for
source compatibility and does not open a GUI. `get_patch_similarity` is NCC:
empty, mismatched, multi-channel, or constant patches return `-1.0`.

The inline helpers retain the following contracts:

* `Bilinear_Interpolation` returns NaN for an empty, non-single-channel, wrong
  depth, non-finite, or out-of-bounds sample. The untyped overload converts
  other depths to `CV_64F`.
* `util_bilinear_Sample_F` requires a single-channel `CV_32F` image and clamps
  finite coordinates to its bounds; invalid input returns a float NaN.
* `util_compute_Img_Gradients` clears outputs for an empty image and otherwise
  writes `CV_32F` Sobel x/y gradients.
* `util_make_rotated_patch_coords` clears its output for a non-positive or
  even patch size. `util_sample_patch_at_coords` preserves coordinate count
  and writes one sampled value per coordinate.
* `ComputeNCC` returns `-1.0` for incompatible/constant patches. `ComputeAverage`
  returns zero for an empty integer vector; `util_vector_mean` returns a
  default-constructed value when empty. `PickUniqueColor` returns black when
  `total <= 0`.
* `BuildImagePyramids` clears all outputs when `number_of_levels <= 0` and
  otherwise uses OpenCV's `buildPyramid`.

## Triangulation and scalar helpers

`two_view_linear_triangulation` expects image points in the same calibrated
coordinate convention, two invertible K matrices, and a relative pose
`p_target = R*p_reference+t`. `multiview_linear_triangulation` requires at
least two views, exactly N points, exactly N-1 relative rotations/translations,
and one shared invertible K. Shape, rank, or finite-value failures return an
all-NaN vector.

`cvMat_Type` formats an OpenCV type as strings such as `8UC1` or `32FC3`.
`ConvertToEigenMatrix` accepts exactly 3x3 nested values and throws
`std::invalid_argument` otherwise. `rad_to_deg` and `deg_to_rad` are scalar
unit conversions. `angle_in_wedge`, `compute_epipolar_angle`, and
`point_in_wedge_robust` retain the original wrap-around and rectified-limit
behavior. `find_closest_boundary_intersection` returns `false` with a zero
output for invalid/no intersections, and otherwise picks the closest valid
image-boundary intersection. `find_Unique_Sorted_Numbers` sorts and removes
duplicates.
