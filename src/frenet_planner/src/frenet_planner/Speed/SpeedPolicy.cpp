#include "frenet_planner/Speed/SpeedPolicy.hpp"

#include <algorithm>

double ApplyTargetSpeedRateLimit(double raw_target, double last_target,
                                  bool has_history,
                                  double max_target_speed_accel,
                                  double replanning_period_s) {
    if (!has_history) return raw_target;
    if (raw_target <= last_target) return raw_target;
    const double allowed_increase = max_target_speed_accel * replanning_period_s;
    return std::min(raw_target, last_target + allowed_increase);
}

double ResolveVelocityKeepingBoundaryAcceleration(double raw_s_ddot,
                                                   double current_s_dot,
                                                   double resolved_target_speed,
                                                   double deadband) {
    const bool accelerating_intent =
        resolved_target_speed > current_s_dot + deadband;
    if (accelerating_intent && raw_s_ddot < 0.0) {
        return 0.0;
    }
    return raw_s_ddot;
}
