#pragma once
// Compatibility entry for LOCAL extensions; shared data/geometry live in jamming_common.
#include "jamming_common/types.hpp"
#include "jamming_common/geometry.hpp"
#include "jamming_common/speed_profile.hpp"
namespace local_planning {
double ReferenceAt(const Scene &, double x); // Soft home cue, never a collision boundary.
// Distance-normalized final score; does not certify collision validity.
double SegmentQualityCost(const Scene &, const Config &, const Pose &, const Pose &, unsigned soft_cells);
void BuildGridIndex(Grid &); // Current raster index; never changes probability/coverage.
Result Plan(Scene, const Config &);
} // namespace local_planning
