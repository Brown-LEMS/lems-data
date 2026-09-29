# Migration {#migration_page}

The compatibility layer is an explicit bridge for the current Brown-LEMS
stereo and `GPU_dev` multinocular repositories. It does not make their
matching containers interchangeable, and it does not replace detector
algorithms.

## Targets and headers

For the modern data API, link:

```cmake
find_package(lemsData CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE lems::data)
```

For an existing VO checkout, link `lems::vo_compat` and include the installed
forwarding headers below `include/lems/vo`:

```cpp
#include <lems/vo/Dataset.h>
#include <lems/vo/utility.h>
```

The stereo repository selects `lems::stereo_compat`, whose forwarding headers
live below `include/lems/vo/stereo_profile`. Select one profile per target;
the GPU/multinocular and stereo profiles intentionally expose different
same-named matching record layouts and must not be linked together.

## Shared Edge migration

The detector's old local `Edge` and (for the GPU branch) `Edge_3D` declarations
must be removed in favor of @ref lems::data::Edge and
@ref lems::data::Edge_3D. The reviewed migration script in the source tree
backs up the expected detector header and adds aliases for old unqualified
source calls. Confirm the diff in the detector checkout: this library does
not rewrite detector algorithms or CUDA code.

The shared types preserve the old fields and sentinel defaults. Equality and
hashing use only `frame_source`/`index`, so detector refinement of location or
orientation does not invalidate identity-based containers. Edge orientation
is documented in radians.

## Remove duplicate implementation sources

After linking the adapter, remove the old VO target's duplicate
`Dataset.cpp`, `Stereo_Iterator.cpp`, `Multinocular_Iterator.cpp`, and
`utility.cpp` entries. Keep the repository's detector implementation and its
compiler/OpenMP/CUDA options. The source-tree architecture and migration
notes remain the authority for the exact include-order and backup procedure.

## Important behavior changes

* The modern reader returns @ref lems::data::FrameSet values. Images are
  materialized by iterator `next()`, and `Frame::camera`, `timestamp_ns`,
  metadata paths, and optional poses are retained rather than discarded by a
  legacy adapter.
* `Dataset::has_ground_truth()` reports trajectory pose availability in the
  modern API. The legacy adapter's `Dataset::has_gt()` reports dense
  disparity/reference availability for matching metrics. A pose-only dataset
  can therefore have modern ground truth while the adapter's dense `has_gt()`
  remains false; update metric gating accordingly.
* `FileInfo::has_gt` is a legacy file-level pose flag and should not be
  confused with the adapter's dense `has_gt()` method.
* `skip_frames` affects iterator stepping and `DatasetIterator::size()`'s
  remaining count; it does not change `Dataset::size()`.
* N-view requests produce complete sliding windows only. Incomplete final
  windows are not returned.

## Verify the hand-off

Compile a translation unit containing the real detector and selected adapter,
then check that `std::vector<Edge>` resolves to the shared type, K/R/t/F remain
Eigen values, and dense fields are read from `Frame::metadata`. The repository
validation snapshot checks both detector headers at the shared-type boundary;
that check is not a claim of full VO, CUDA, or matching end-to-end validation.
