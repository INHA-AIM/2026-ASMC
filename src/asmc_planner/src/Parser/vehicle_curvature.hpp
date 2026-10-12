#pragma once

#include <cmath>
#include <limits>

namespace frenet_planner_ros {

inline double VehicleCurvature(double speed, double yaw_rate) {
    if (!std::isfinite(speed) || !std::isfinite(yaw_rate)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // Below this speed yaw_rate/speed amplifies stationary IMU noise. The
    // planner uses geometric low-speed paths; no history is held across modes.
    constexpr double kMinimumCurvatureSpeed = 0.5;
    return std::fabs(speed) > kMinimumCurvatureSpeed ? yaw_rate / speed : 0.0;
}

}  // namespace frenet_planner_ros
