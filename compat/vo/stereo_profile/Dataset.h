#pragma once

// Include this directory before compat/vo when compiling the legacy stereo
// repository.  The profile selects the stereo pipeline's container shapes;
// calibration, poses, images and Edge storage still come from lems-data.
#ifndef LEMS_DATA_STEREO_COMPAT
#define LEMS_DATA_STEREO_COMPAT 1
#endif

#include "../Stereo_Pipeline_Types.h"
using StereoFrame = lems::vo::stereo::StereoFrame;
#include "../Dataset.h"

using GTPose = lems::vo::stereo::GTPose;
using SpatialGrid = lems::vo::stereo::SpatialGrid;
using scores = lems::vo::stereo::scores;
using EdgeCluster = lems::vo::stereo::EdgeCluster;
using Evaluation_Statistics = lems::vo::stereo::Evaluation_Statistics;
using Stereo_Edge_Pairs = lems::vo::stereo::Stereo_Edge_Pairs;
using Stereo_Matching_Edge_Clusters =
    lems::vo::stereo::Stereo_Matching_Edge_Clusters;
using final_stereo_edge_pair = lems::vo::stereo::final_stereo_edge_pair;
using Temporal_CF_Edge_Cluster = lems::vo::stereo::Temporal_CF_Edge_Cluster;
using temporal_edge_pair = lems::vo::stereo::temporal_edge_pair;
using Veridical_Quad_Entry = lems::vo::stereo::Veridical_Quad_Entry;
using Candidate_Quad_Entry = lems::vo::stereo::Candidate_Quad_Entry;
