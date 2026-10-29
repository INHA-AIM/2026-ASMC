#pragma once
#include "local/planner.hpp"

namespace local_planning {
// One longitudinal feasibility contract for normal and recovery candidates.
// Distances are rear-axle travel after the full-body sweep, not bumper range.
double BrakingDistance(double speed, double acceleration, const Config &);
double SpaceSpeed(double distance, double acceleration, double delay, const Config &);
bool BuildSpeedProfile(const Scene &, const Config &, Result &);
} // namespace local_planning
