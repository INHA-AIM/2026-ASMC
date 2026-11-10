#pragma once
#include "local/planner.hpp"

namespace shadow_localization {
// Sensor mounting coordinates are in base_link. Use the same aligned attitude
// convention as OBB projection; mount height contributes XY on a bank or slope.
inline std::array<double,2> GpsMountXY(double x,double y,double z,
                                      double roll,double pitch,double grid_yaw) {
  const auto leveled=local_planning::Rotate(local_planning::AttitudeRotation(roll,pitch),x,y,z);
  const double c=std::cos(grid_yaw),s=std::sin(grid_yaw);
  return {c*leveled[0]-s*leveled[1],s*leveled[0]+c*leveled[1]};
}
}
