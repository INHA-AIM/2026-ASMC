#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

// Advance the last accepted acceleration profile instead of restarting its
// jerk ramp from a positive measured acceleration on every bypass replan.
// Measured speed remains authoritative; this changes only the feedforward seed.
inline double AvoidAccelerationSeed(double measured, double speed, double cap,
    bool bypass_active, const std::vector<double>& previous, double path_dt,
    double elapsed, double decel, double max_accel) {
    const double observed = std::clamp(measured, -decel, max_accel);
    if (!bypass_active || speed <= cap + .1 || previous.size() < 2 ||
        !std::isfinite(elapsed) || elapsed <= 0 || elapsed > .5 ||
        !std::isfinite(path_dt) || path_dt <= 0) return observed;
    const double index = elapsed / path_dt;
    if (index >= static_cast<double>(previous.size() - 1)) return observed;
    const auto lo = static_cast<std::size_t>(index);
    const double prior = previous[lo] +
        (previous[lo + 1] - previous[lo]) * (index - lo);
    if (!std::isfinite(prior)) return observed;
    return std::min(observed, std::clamp(prior, -decel, max_accel));
}
