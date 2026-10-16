#pragma once
#include <algorithm>
#include <cmath>
#include "MPC/Constraints/constraints.hpp"

// Scalar fallback when cached braking geometry cannot be used.
// Only for a valid planner STOP with fresh valid ego state. Watchdog failures
// and emergency STOP retain the original hard-stop policy. A recent successful
// steering command decays using the same speed-dependent envelope as the
// optimizer. elapsed_s is time since the previous valid published command,
// including earlier STOP commands, so changing speed cannot bypass the limit.
inline double BrakingSteering(double previous_delta, double elapsed_s,
                              double speed, const MPCParams& params) {
    if (!std::isfinite(previous_delta) || !std::isfinite(elapsed_s) ||
        !std::isfinite(speed) || elapsed_s < 0.0 || speed < 0.3) return 0.0;
    const double rate_limit = SteeringRateLimit(speed, params);
    if (!std::isfinite(rate_limit) || rate_limit <= 0.0) return 0.0;
    const double magnitude = std::max(
        0.0, std::fabs(previous_delta) - rate_limit * elapsed_s);
    return std::copysign(magnitude, previous_delta);
}
