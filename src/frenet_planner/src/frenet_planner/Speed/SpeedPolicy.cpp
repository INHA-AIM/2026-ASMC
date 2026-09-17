#include "frenet_planner/Speed/SpeedPolicy.hpp"

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
