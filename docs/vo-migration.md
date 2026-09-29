# Migrating the Brown-LEMS VO repositories

This adapter is for the current source trees of:

- `Brown-LEMS/Edge_Based_Visual_Odometry` (stereo), and
- `Brown-LEMS/Multinocular-Edge-Visual-Odometry` branch `GPU_dev`.

It gives both repositories the same dataset, iterator, camera, pose, edge,
and utility implementation. It is a migration boundary, not a promise that
the two matching pipelines have identical containers.

## 1. Link the installed package

```cmake
find_package(lemsData CONFIG REQUIRED)
target_link_libraries(your_vo_target PRIVATE lems::vo_compat)
```

Use the installed, unambiguous forwarding headers in source files:

```cpp
#include <lems/vo/Dataset.h>
#include <lems/vo/utility.h>
```

For a source checkout instead:

```cmake
add_subdirectory(third_party/lems-data)
target_link_libraries(your_vo_target PRIVATE lems::vo_compat)
# The source target supplies the equivalent build-tree include directory. If
# the VO project keeps its own include/ directory ahead of target includes,
# copy the forwarding files from compat/vo into that directory and include
# them as <Dataset.h>, <utility.h>, and <Multinocular_Iterator.h>.
```

For the stereo repository, add the profile target instead (it links the base
adapter and selects the stereo-specific global container aliases):

```cmake
target_link_libraries(your_stereo_vo_target PRIVATE lems::stereo_compat)
```

The profile installs `stereo_profile/Dataset.h`, `Stereo_Iterator.h`, and
`utility.h` ahead of the default adapter include directory. The profile's
`Stereo_Edge_Pairs`, `Stereo_Matching_Edge_Clusters`, and temporal records
remain under `lems::vo::stereo` and retain the stereo repository's fields.

Remove the old `Dataset.cpp`, `Stereo_Iterator.cpp`,
`Multinocular_Iterator.cpp`, and `utility.cpp` entries from that VO target.
Keep the TOED implementation source (`src/toed/cpu_toed.cpp`) if the project
still uses the detector.

The compatibility target contributes the adapter include directory. If a
repository has a local `include/Dataset.h`, `include/utility.h`, or iterator
header, either replace that header with the forwarding header from
`include/lems/vo` or copy the forwarding files into the local include tree.
Installed consumers should use fully qualified includes such as
`#include <lems/vo/Dataset.h>` and
`#include <lems/vo/stereo_profile/Dataset.h>`; source-tree consumers should
keep the adapter include directory ahead of the old local headers.

## 2. Make the detector use the shared Edge type

Both reference repositories currently define `struct Edge` in
`toed/cpu_toed.hpp`; the GPU branch also defines `Edge_3D`. A second definition
would make the library and detector incompatible. Apply the small, reviewable
script from [../migration/replace_detector_edges.sh](../migration/replace_detector_edges.sh)
to each checkout. It makes a backup, includes the library's Eigen/OpenCV edge
header through `lems/data/types.hpp`, removes the duplicate declarations and
hash specialization, and adds global aliases for old pipeline source:

```cpp
using lems::data::Edge;
using lems::data::Edge_3D;
```

The script refuses a header that does not contain the expected complete block;
review its diff before committing. It does not remove or rewrite detector
algorithms.

## 3. Existing source calls that remain available

The adapter keeps the old names and Eigen/OpenCV return types:

```cpp
Dataset::Ptr dataset = std::make_shared<Dataset>(config_map);
dataset->load_dataset(dataset->get_dataset_type(), disparities, masks);

Eigen::Matrix3d K = dataset->get_left_calib_matrix();
Eigen::Matrix3d F = dataset->get_fund_mat_21();
Eigen::Matrix3d R21 = dataset->get_relative_rot_left_to_right();
Eigen::Vector3d T21 = dataset->get_relative_transl_left_to_right();
cv::Mat distortion = dataset->get_left_dist_coeff_mat();

const FileInfo& files = dataset->file_info;
const CameraInfo& cameras = dataset->camera_info;
```

The stereo adapter exposes `stereo_iterator`; the multinocular adapter exposes
`multinocular_iterator`:

```cpp
while (dataset->stereo_iterator->hasNext()) {
  StereoFrame frame;
  if (!dataset->stereo_iterator->getNext(frame)) break;
  pipeline.current_frame = std::move(frame);
}
```

`StereoFrame::left_frame` and `right_frame` retain the canonical per-camera
observations (camera name, authoritative timestamp, source path, pose, and
optional metadata); the split `left_*`/`right_*` members are the legacy view.
The multinocular iterator returns canonical `Frame` values directly.

The iterator does not eagerly throw away a valid final frame. Dense disparity
and mask vectors are filled only from the corresponding `Frame::metadata`
records. The adapter's `Dataset::has_gt()` specifically reports dense
disparity availability for matching metrics; use `Dataset::has_ground_truth()`
for trajectory poses. This differs from the original generic `file_info.has_gt`
flag for pose-only datasets. Review metric gating when migrating such datasets.

## 4. Select one pipeline-specific matching profile

The default `lems::vo_compat` target exports the GPU/multinocular records from
`pipeline_types.hpp` (spatial grid, edge clusters, observers, and temporal
records) as the legacy global names. The separate `lems::stereo_compat`
target defines `LEMS_DATA_STEREO_COMPAT` and exports the stereo profile's
global `Stereo_Edge_Pairs`, `Stereo_Matching_Edge_Clusters`, temporal records,
and `StereoFrame` aliases from `compat/vo/Stereo_Pipeline_Types.h`; those
records retain the stereo repository's pointer ownership and SIFT/NCC fields.
Do not link both profiles into one target: their same-named matching records
intentionally have different layouts. This profile boundary does not claim
full stereo VO end-to-end validation; the installed consumer checks header,
alias, and link compatibility only.

## 5. Verify the migration

From this checkout, first inspect each detector header and then apply the
reviewed replacement:

```bash
./migration/replace_detector_edges.sh /path/to/Multinocular-Edge-Visual-Odometry
./migration/replace_detector_edges.sh --apply /path/to/Multinocular-Edge-Visual-Odometry
./migration/replace_detector_edges.sh /path/to/Edge_Based_Visual_Odometry
./migration/replace_detector_edges.sh --apply /path/to/Edge_Based_Visual_Odometry
```

The script creates `cpu_toed.hpp.lems-data-backup` beside each edited header
and refuses to overwrite an existing backup. Keep the detector source and its
OpenMP flags from the original checkout.

At minimum, compile a translation unit that includes the detector and adapter
headers (the `migration/reference_detector_smoke.cpp` harness exercises the
real detector), then run the repository's existing tests on a small dataset.
The library's CI performs the installed-consumer check; the detector smoke is
run against each migrated VO checkout because its source and OpenMP options
belong to that checkout. Check that:

1. `std::vector<Edge>` is the shared `lems::data::Edge` type.
2. `K`, `R`, `t`, `F`, and distortion remain Eigen/OpenCV values with no array
   conversion layer.
3. `Frame::camera`, `timestamp_ns`, disparity paths, and mask paths are not
   discarded by the old iterator adapter.
4. The dataset's `FileInfo` and `CameraInfo` views describe the same objects
   used by the reader, rather than copies maintained by a compatibility shim.

The source-linked detector smoke was run against both reference checkouts and
reported 88 shared edges and 88 grid candidates for each. This validates the
detector-to-data hand-off only; it is not full VO or matching-pipeline
end-to-end validation.

The extraction is attributed to Multinocular-Edge-Visual-Odometry `GPU_dev`
commit `2ec5e72fa47aff6ee663c2b072c7d161c4e76595` and the stereo
Edge_Based_Visual_Odometry source commit `9d5c0d`, with each repository's
matching algorithms remaining outside this package.
