#include "MPC/Constraints/constraints.hpp"
#include "MPC/Math/utils.hpp"
#include <algorithm>
#include <cmath>

double SteeringRateLimit(double speed_mps, const MPCParams& params) {
    if (params.steering_rate_reference_speed_mps <= 0.0 ||
        std::abs(speed_mps) <= params.steering_rate_reference_speed_mps)
        return params.steering_rate_max;
    const double ratio = params.steering_rate_reference_speed_mps / std::abs(speed_mps);
    return std::clamp(params.steering_rate_max * ratio * ratio,
                      params.steering_rate_min, params.steering_rate_max);
}


void clipSteering(MPCControl& u, double max_steering) {
    u.delta = clip(u.delta, -max_steering, max_steering);
}

void clipAcceleration(MPCControl& u, double a_min, double a_max) {
    u.accel = clip(u.accel, a_min, a_max);
}

void clipControl(MPCControl& u, const MPCParams& p) {
    clipSteering(u, p.steering_max);
    clipAcceleration(u, p.accel_min, p.accel_max);
}

void clipSteeringRate(MPCControl& u_cur, const MPCControl& u_prev,
                      double max_rate, double dt) {
    double max_step = max_rate * dt;
    double diff = u_cur.delta - u_prev.delta;
    diff = clip(diff, -max_step, max_step);
    u_cur.delta = u_prev.delta + diff;
}
