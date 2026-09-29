#pragma once

// Legacy global include kept as a forwarding header.  The implementation and
// the Utility object are owned by lems-data; this header only imports the
// names used by the two Brown-LEMS repositories.
#include <lems/data/utility.hpp>

using Utility = lems::data::Utility;
using UtilityOptions = lems::data::UtilityOptions;

#ifndef PATCH_SIZE
inline constexpr int PATCH_SIZE = lems::data::kDefaultPatchSize;
#endif
#ifndef ORTHOGONAL_SHIFT_MAG
inline constexpr double ORTHOGONAL_SHIFT_MAG =
    lems::data::kDefaultOrthogonalShift;
#endif

using lems::data::Bilinear_Interpolation;
using lems::data::BuildImagePyramids;
using lems::data::ComputeAverage;
using lems::data::ComputeNCC;
using lems::data::ConvertToEigenMatrix;
using lems::data::PickUniqueColor;
using lems::data::angle_in_wedge;
using lems::data::compute_epipolar_angle;
using lems::data::deg_to_rad;
using lems::data::find_Unique_Sorted_Numbers;
using lems::data::find_closest_boundary_intersection;
using lems::data::point_in_wedge_robust;
using lems::data::rad_to_deg;
using lems::data::util_bilinear_Sample_F;
using lems::data::util_compute_Img_Gradients;
using lems::data::util_make_rotated_patch_coords;
using lems::data::util_sample_patch_at_coords;
using lems::data::util_vector_mean;
