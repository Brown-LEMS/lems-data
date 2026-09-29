#pragma once

#ifndef LEMS_DATA_STEREO_COMPAT
#define LEMS_DATA_STEREO_COMPAT 1
#endif

#include "../Stereo_Pipeline_Types.h"
#if __has_include(<lems/vo/stereo_profile/Dataset.h>)
#include <lems/vo/stereo_profile/Dataset.h>
#else
#include "Dataset.h"
#endif
#include "../Stereo_Iterator.h"
