# Edge and pipeline data {#edge_pipeline}

The library supplies the data boundary used by the Brown-LEMS detectors and
matchers. It does not bundle a TOED detector, SIFT implementation, CUDA
kernel, OpenMP policy, or matching algorithm. A consuming pipeline detects
edges and stores them in `Frame::edges`, then uses the data-only records below
as needed.

## Edge identity

@ref lems::data::Edge is a 2-D observation:

| Field | Contract |
| --- | --- |
| `location` | Pixel coordinates as `cv::Point2d`; default sentinel `(-1,-1)`. |
| `orientation` | Edge angle in radians; default sentinel `-100.0`. No range normalization is imposed. |
| `b_isEmpty` | Detector/pipeline empty marker; default `true`. |
| `frame_source` | Source detector frame identity; default `-1`. |
| `index` | Source edge index; default `-1`. |

Equality is identity-only: `frame_source` and `index` are compared, while
location, orientation, and `b_isEmpty` may be refined without changing the
identity. The `std::hash` specializations use exactly those two identity
fields, so an `Edge` can remain in an unordered container while its geometry
changes. The constructor's `edge_index` default is explicit and deterministic.

@ref lems::data::Edge_3D carries a triangulated location and a 3-D tangent in
the camera coordinate system. Its identity and defaults mirror `Edge`; the
constructor does not normalize `tangent`, so normalize only when the consuming
algorithm's contract requires it.

## Spatial indexing

@ref lems::data::SpatialGrid maps pixel coordinates to cells of
`cell_size` (default 35). `cell_for()` uses `std::floor`, so negative
coordinates do not truncate into cell zero. `add_edge_to_grids()` returns a
linear cell index or `-1` for an invalid/non-finite location; `reset()` clears
all cells. Candidate queries return the indices stored in neighboring cells,
not a distance-filtered guarantee. Invalid locations, negative/non-finite
radii, or an uninitialized grid return an empty vector.

## Observer and match records

The `pipeline` group contains data-only records extracted from the
multinocular pipeline:

* @ref lems::data::Observer points to a source @ref lems::data::Frame and
  owns edge-3D, descriptor, and patch vectors. `clean_up_observer_base()`
  clears those vectors but does not delete the pointed-to frame.
* @ref lems::data::Main_Observer adds subset edge indices and a TOED-index map;
  `check_observer_validity()` checks pointer and vector-size consistency.
* @ref lems::data::Sub_Observer adds epipolar angles, epipoles, and
  fundamental matrices for a sub-camera.
* @ref lems::data::EdgeCluster, @ref lems::data::EdgeMatch, and
  @ref lems::data::Camera_Set carry matching and evaluation data without
  taking ownership of a detector.
* @ref lems::data::Stereo_Edge_Pairs, temporal edge/quads, and
  @ref lems::data::KF_Temporal_Match retain pointer/reference fields exactly
  where the original pipeline uses non-owning relationships.

`Camera_Set::consistency_check()` checks that one match loop exists per main
observer edge. `concistency_check()` is retained as a typo-compatible alias.
Bounds-checked lookup helpers return a default empty `Edge` when their frame
pointer or index is invalid.

## Frame ownership boundary

`Frame` and pipeline records are ordinary value types. A `Frame`'s
`cv::Mat` members are OpenCV reference-counted headers with shallow-copy pixel
ownership; a `Frame` does not promise exclusive image storage. Observer
`frame` pointers and temporal record pointers are non-owning and must outlive
the records that refer to them. The data layer can be used without linking a
detector, and a consuming project remains responsible for detector lifetime,
GPU buffers, descriptor policy, and match validity.
