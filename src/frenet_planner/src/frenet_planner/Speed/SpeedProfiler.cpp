#include "frenet_planner/Speed/SpeedProfiler.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

struct JerkSegment {
    double jerk = 0.0;
    double duration = 0.0;
};

double Clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

void AppendSegment(std::vector<JerkSegment>& segments, double jerk,
                   double duration) {
    if (duration > 1e-12) segments.push_back({jerk, duration});
}

// Advances (speed, acceleration) along the first dt seconds of the exact
// jerk-limited S-curve that reaches target_speed with zero final acceleration.
//
// Work is performed in an oriented coordinate q where the requested velocity
// change is positive.  With initial acceleration b and peak acceleration p,
// the two jerk ramps change velocity by
//
//        delta_v = (2 p^2 - b^2) / (2 j).
//
// If the requested delta is too small for the current positive acceleration,
// the complementary solution first jerks through zero to a negative valley n:
//
//        delta_v = (b^2 - 2 n^2) / (2 j).
//
// Those closed-form switching points account for the velocity still accrued
// while acceleration returns to zero.  Unlike a sign-switching bang-bang
// controller, the profile therefore terminates at the cap instead of crossing
// it and oscillating.  Acceleration plateaus are inserted when either physical
// acceleration bound is reached.
SpeedProfileOutput Advance(double speed, double acceleration,
                           double target_speed, double dt,
                           const SpeedProfileLimits& limits) {
    constexpr double kEpsilon = 1e-10;
    SpeedProfileOutput output;
    output.target_speed = speed;
    output.target_acceleration = acceleration;
    output.initialized = true;

    if (!std::isfinite(speed) || !std::isfinite(acceleration) ||
        !std::isfinite(target_speed) || !std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(limits.max_acceleration) ||
        !std::isfinite(limits.max_deceleration) ||
        !std::isfinite(limits.max_jerk) ||
        limits.max_acceleration <= 0.0 ||
        limits.max_deceleration <= 0.0 || limits.max_jerk <= 0.0) {
        return output;
    }

    speed = std::max(0.0, speed);
    target_speed = std::max(0.0, target_speed);
    acceleration = Clamp(acceleration, -limits.max_deceleration,
                         limits.max_acceleration);

    const double error = target_speed - speed;
    if (std::fabs(error) <= kEpsilon &&
        std::fabs(acceleration) <= kEpsilon) {
        output.target_speed = target_speed;
        output.target_acceleration = 0.0;
        return output;
    }

    // At exactly the requested speed, choose the direction opposing the
    // current acceleration.  The resulting two-ramp profile removes that
    // acceleration and returns to the same cap without a persistent limit
    // cycle.
    const double direction = std::fabs(error) > kEpsilon
        ? (error > 0.0 ? 1.0 : -1.0)
        : (acceleration > 0.0 ? -1.0 : 1.0);
    const double delta_v = std::max(0.0, direction * error);
    double oriented_acceleration = direction * acceleration;
    const double positive_limit = direction > 0.0
        ? limits.max_acceleration : limits.max_deceleration;
    const double negative_limit = direction > 0.0
        ? limits.max_deceleration : limits.max_acceleration;
    const double jerk_limit = limits.max_jerk;
    oriented_acceleration = Clamp(oriented_acceleration, -negative_limit,
                                  positive_limit);

    std::vector<JerkSegment> segments;
    const double positive_peak_squared =
        jerk_limit * delta_v +
        0.5 * oriented_acceleration * oriented_acceleration;
    const double positive_peak = std::sqrt(
        std::max(0.0, positive_peak_squared));

    if (positive_peak + kEpsilon >=
        std::max(0.0, oriented_acceleration)) {
        if (positive_peak <= positive_limit + kEpsilon) {
            const double peak = std::min(positive_peak, positive_limit);
            AppendSegment(segments, jerk_limit,
                          (peak - oriented_acceleration) / jerk_limit);
            AppendSegment(segments, -jerk_limit, peak / jerk_limit);
        } else {
            const double ramp_delta =
                (2.0 * positive_limit * positive_limit -
                 oriented_acceleration * oriented_acceleration) /
                (2.0 * jerk_limit);
            const double plateau_duration =
                std::max(0.0, (delta_v - ramp_delta) / positive_limit);
            AppendSegment(segments, jerk_limit,
                          (positive_limit - oriented_acceleration) /
                              jerk_limit);
            AppendSegment(segments, 0.0, plateau_duration);
            AppendSegment(segments, -jerk_limit,
                          positive_limit / jerk_limit);
        }
    } else {
        // Current acceleration would overshoot the cap even if maximum
        // opposite jerk starts now.  The exact solution contains a small
        // acceleration reversal before returning to zero.
        const double negative_valley_squared = std::max(
            0.0,
            (oriented_acceleration * oriented_acceleration -
             2.0 * jerk_limit * delta_v) /
                2.0);
        const double negative_valley = std::sqrt(negative_valley_squared);
        if (negative_valley <= negative_limit + kEpsilon) {
            const double valley = std::min(negative_valley, negative_limit);
            AppendSegment(segments, -jerk_limit,
                          (oriented_acceleration + valley) / jerk_limit);
            AppendSegment(segments, jerk_limit, valley / jerk_limit);
        } else {
            const double ramp_delta =
                (oriented_acceleration * oriented_acceleration -
                 2.0 * negative_limit * negative_limit) /
                (2.0 * jerk_limit);
            const double plateau_duration = std::max(
                0.0, (ramp_delta - delta_v) / negative_limit);
            AppendSegment(segments, -jerk_limit,
                          (oriented_acceleration + negative_limit) /
                              jerk_limit);
            AppendSegment(segments, 0.0, plateau_duration);
            AppendSegment(segments, jerk_limit,
                          negative_limit / jerk_limit);
        }
    }

    double remaining = dt;
    double oriented_speed = direction * speed;
    double total_duration = 0.0;
    for (const JerkSegment& segment : segments) {
        total_duration += segment.duration;
        if (remaining <= 0.0) break;
        const double step = std::min(remaining, segment.duration);
        oriented_speed += oriented_acceleration * step +
                          0.5 * segment.jerk * step * step;
        oriented_acceleration += segment.jerk * step;
        remaining -= step;
    }

    // Sampling beyond the analytical terminal time stays exactly at the cap;
    // this is what removes cap-near numerical chatter.
    if (dt + kEpsilon >= total_duration) {
        output.target_speed = target_speed;
        output.target_acceleration = 0.0;
        return output;
    }

    output.target_speed = std::max(0.0, direction * oriented_speed);
    output.target_acceleration = Clamp(
        direction * oriented_acceleration, -limits.max_deceleration,
        limits.max_acceleration);
    return output;
}

}  // namespace

SpeedProfileOutput SpeedProfiler::Preview(
    const SpeedProfileInput& input, const SpeedProfileLimits& limits) const {
    const double initial_speed = initialized_
        ? profiled_speed_ : std::max(0.0, input.ego_speed);
    double initial_acceleration = profiled_acceleration_;
    if (!initialized_) {
        // The profiler state is a normal-driving target reference, not a
        // second estimator of vehicle dynamics.  On restart, retain measured
        // acceleration only when it already points toward the requested cap.
        // A stale negative estimate while the cap is above ego speed is the
        // braking-memory condition handled by KEEP's boundary policy; seeding
        // it here would reintroduce the same unwanted initial slowdown.
        const double cap_error = input.raw_speed_cap - initial_speed;
        const bool aligned =
            (cap_error > 0.0 && input.ego_acceleration > 0.0) ||
            (cap_error < 0.0 && input.ego_acceleration < 0.0);
        initial_acceleration = aligned
            ? Clamp(input.ego_acceleration, -limits.max_deceleration,
                    limits.max_acceleration)
            : 0.0;
    }

    const bool valid_transition =
        std::isfinite(initial_speed) &&
        std::isfinite(input.raw_speed_cap) &&
        input.raw_speed_cap >= 0.0 &&
        std::isfinite(input.dt) && input.dt > 0.0 &&
        std::isfinite(limits.max_acceleration) &&
        limits.max_acceleration > 0.0 &&
        std::isfinite(limits.max_deceleration) &&
        limits.max_deceleration > 0.0 &&
        std::isfinite(limits.max_jerk) && limits.max_jerk > 0.0;
    if (valid_transition && input.raw_speed_cap < initial_speed) {
        // A cap is an environmental/planning constraint, not a physical state.
        // Never let an old profiled target remain above a newly lowered cap.
        // Reset the scalar profiler acceleration as well, otherwise its stale
        // recovery memory can re-raise the target on the following cycle.
        SpeedProfileOutput output;
        output.target_speed = input.raw_speed_cap;
        output.target_acceleration = 0.0;
        output.initialized = true;
        return output;
    }
    return Advance(initial_speed, initial_acceleration, input.raw_speed_cap,
                   input.dt, limits);
}

void SpeedProfiler::Commit(const SpeedProfileOutput& output) {
    if (!output.initialized || !std::isfinite(output.target_speed) ||
        !std::isfinite(output.target_acceleration)) {
        Reset();
        return;
    }
    initialized_ = true;
    profiled_speed_ = std::max(0.0, output.target_speed);
    profiled_acceleration_ = output.target_acceleration;
}

SpeedProfileOutput SpeedProfiler::Update(
    const SpeedProfileInput& input, const SpeedProfileLimits& limits) {
    const SpeedProfileOutput output = Preview(input, limits);
    Commit(output);
    return output;
}

void SpeedProfiler::Reset() {
    initialized_ = false;
    profiled_speed_ = 0.0;
    profiled_acceleration_ = 0.0;
}
