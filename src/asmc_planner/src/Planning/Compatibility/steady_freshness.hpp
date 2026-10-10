#pragma once

#include <cmath>

#include <ros/time.h>

namespace frenet_planner_ros {

inline bool IsSteadyFresh(const ros::SteadyTime& received_at,
                          const ros::SteadyTime& now,
                          double timeout_seconds,
                          double* age_seconds = nullptr) {
    const double age = (now - received_at).toSec();
    if (age_seconds) *age_seconds = age;
    return !received_at.isZero() && std::isfinite(age) && age >= 0.0 &&
           std::isfinite(timeout_seconds) && timeout_seconds > 0.0 &&
           age <= timeout_seconds;
}

}  // namespace frenet_planner_ros
