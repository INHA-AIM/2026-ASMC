#pragma once
#include "Frenet/Common/PlannerCommand.hpp"
#include <algorithm>
#include <cmath>

namespace asmc_planner::local {
// Opt-in NORMAL-to-LOCAL speed envelope, not a second speed profiler. The
// existing planner still applies curvature, braking and jerk exactly once.
inline void ApplyPreparationLimit(PlannerCommand& command, bool enabled,
                                  bool preparing, double receipt_age,
                                  double cap_kph) {
  if (!enabled || !preparing || !std::isfinite(receipt_age) ||
      receipt_age < 0 || receipt_age > .5 || !std::isfinite(cap_kph) ||
      cap_kph <= 0 || cap_kph > 25) return;
  const double limit = cap_kph/3.6;
  command.max_speed = command.has_max_speed ? std::min(command.max_speed,limit) : limit;
  command.has_max_speed = true;
}
}
