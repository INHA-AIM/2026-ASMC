#pragma once
#include <algorithm>
#include <cmath>

namespace asmc_msgs::braking {
// Initial measurement thresholds, not actuator performance guarantees.
// An ordinary STOP may continue already-applied service deceleration. A
// maximum-brake escalation must demonstrate an ADDITIONAL physical response.
inline bool PhysicalResponse(unsigned level,double initial_speed,double initial_acceleration,
                             double speed,double acceleration) {
  if((level!=1 && level!=2) || !std::isfinite(initial_speed) ||
     !std::isfinite(initial_acceleration) || !std::isfinite(speed) ||
     !std::isfinite(acceleration))return false;
  const double threshold=level==2 ? std::min(-.2,initial_acceleration-.2) : -.2;
  return acceleration<=threshold && speed<=initial_speed-.02;
}
}  // namespace asmc_msgs::braking
