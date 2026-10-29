#pragma once
#include "jamming_common/types.hpp"

namespace local_planning {
// One longitudinal feasibility contract for normal and recovery candidates.
// Distances are rear-axle travel after the full-body sweep, not bumper range.
double BrakingDistance(double speed, double acceleration, const Config &);
double SpaceSpeed(double distance, double acceleration, double delay, const Config &);
Result TimeParameterize(const Scene &,const Config &,Result,const std::vector<double>& limits = {});
bool BuildSpeedProfile(const Scene &, const Config &, Result &);
} // namespace local_planning
