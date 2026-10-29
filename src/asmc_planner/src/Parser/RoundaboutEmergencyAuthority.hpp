#pragma once
#include <cmath>
namespace asmc_planner {
// Only the 8m desired-gap emergency surrogate is delegated. Native swept-body
// collision checks, imminent TTC, pedestrian/dynamic rules and watchdog remain.
inline bool NativeOwnsRoundaboutGapEmergency(bool roundabout, bool executing,
    bool command_matches, bool plan_valid, bool locked, bool objects_fresh,
    bool temporal_emergency, double feedback_age, bool execution_linked) {
  return roundabout && executing && command_matches && plan_valid && locked &&
      objects_fresh && !temporal_emergency && execution_linked &&
      std::isfinite(feedback_age) && feedback_age >= 0 && feedback_age <= .15;
}
}
