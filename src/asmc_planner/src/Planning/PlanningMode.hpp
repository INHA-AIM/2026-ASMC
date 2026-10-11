#pragma once

#include <cstdint>
#include <morai_msgs/GPSMessage.h>

namespace asmc_planner {
enum class PlanningMode : std::uint8_t { NORMAL = 0, JAMMING = 1 };

inline PlanningMode DetectPlanningMode(const morai_msgs::GPSMessage& gps) {
  return gps.latitude == 0.0 && gps.longitude == 0.0 &&
                 gps.altitude == 0.0
             ? PlanningMode::JAMMING : PlanningMode::NORMAL;
}

inline const char* PlanningModeName(PlanningMode mode) {
  return mode == PlanningMode::JAMMING ? "JAMMING" : "NORMAL";
}
}  // namespace asmc_planner
