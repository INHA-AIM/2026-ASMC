#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

// Advance the accepted acceleration ramp instead of restarting it from the
// measured response on every bypass replan. Brake release is opt-in: the caller
// must retain the speed cap and recheck the resulting moving candidate against
// every current obstacle. Maintenance braking must not consume this release.
// Measured position and speed remain authoritative.
inline double AvoidAccelerationSeed(double measured, double speed, double cap,
    bool bypass_active, const std::vector<double>& previous, double path_dt,
    double elapsed, double decel, double max_accel, bool may_release_brake = false,
    bool may_continue_accel_reduction = false) {
    const double observed = std::clamp(measured, -decel, max_accel);
    if (!bypass_active || previous.size() < 2 ||
        !std::isfinite(speed) || !std::isfinite(cap) ||
        !std::isfinite(elapsed) || elapsed <= 0 || elapsed > .5 ||
        !std::isfinite(path_dt) || path_dt <= 0) return observed;
    const double index = elapsed / path_dt;
    if (index >= static_cast<double>(previous.size() - 1)) return observed;
    const auto lo = static_cast<std::size_t>(index);
    const double prior = previous[lo] +
        (previous[lo + 1] - previous[lo]) * (index - lo);
    if (!std::isfinite(prior)) return observed;
    const double accepted = std::clamp(prior, -decel, max_accel);
    if (speed > cap + .1) return std::min(observed, accepted);
    // A valid dynamic bypass already tapering acceleration toward its cap
    // must advance that ramp before it exceeds the cap. Re-seeding from the
    // larger measured acceleration each 50ms otherwise repeats its first step.
    if (may_continue_accel_reduction && prior < previous.front() && accepted < observed)
        return accepted;
    // The 20 Hz replan otherwise restarts a 0.4 s brake-release ramp every
    // 0.05 s. An over-responsive brake can then slow far below the bypass cap.
    // A profile still adding braking cannot authorize this release.
    if (may_release_brake && speed < cap - .1 && observed < 0 &&
        (accepted >= 0 || prior > previous.front()))
        return std::max(observed, accepted);
    return observed;
}
