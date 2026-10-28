#include "local/speed_profile.hpp"
#include <algorithm>
#include <cmath>

namespace local_planning {
// Exact constant-jerk ramp from current acceleration to the configured brake.
// Positive acceleration is part of stopping distance, not instantaneous
// braking.
double BrakingDistance(double v, double a, const Config &c) {
  if (v <= 0)
    return 0.;
  a = std::max(-c.decel, a);
  double ramp = (a + c.decel) / c.jerk;
  double zero = (a + std::sqrt(a * a + 2 * c.jerk * v)) / c.jerk;
  double t = std::min(ramp, zero);
  double d = v * t + .5 * a * t * t - c.jerk * t * t * t / 6.;
  double rest = std::max(0., v + a * t - .5 * c.jerk * t * t);
  return d + rest * rest / (2 * c.decel);
}
double SpaceSpeed(double distance, double acceleration, double delay,
                  const Config &c) {
  double lo = 0., hi = std::max(15., c.speed_cap);
  for (int i = 0; i < 24; ++i) {
    double v = .5 * (lo + hi);
    if (v * delay + BrakingDistance(v, acceleration, c) <=
        std::max(0., distance))
      lo = v;
    else
      hi = v;
  }
  return lo;
}

bool BuildSpeedProfile(const Scene &s, const Config &c, Result &out) {
  if (out.path.size() < 2) { out.reason="no_speed_geometry"; return false; }
  out.trajectory.clear();
  double maxk = 0;
  for (auto &p : out.path)
    maxk = std::max(maxk, std::abs(p.k));
  out.raw_cap = c.speed_cap;
  out.curve_cap = std::sqrt(c.lateral_accel * .95 / std::max(1e-4, maxk));
  // Path s is already rear-axle travel after a full-body collision check; do
  // not subtract the front overhang a second time. Reserve jerk ramp +
  // execution lag.
  const double distance = out.path.back().s;
  out.available_stopping_m = distance;
  // In aligned scenes the vehicle's source->plan travel already shortens the
  // available sweep. Keep execution/jerk delay, do not charge that travel
  // twice.
  double lag = c.delay + (s.aligned_to_plan ? 0. : s.age);
  out.required_stopping_m = s.speed * lag + BrakingDistance(s.speed, s.acceleration, c);
  // A braking frame must not instantly raise the cruise cap and restart the
  // throttle/brake cycle. Keep a positive-acceleration envelope, plus measured
  // LOCAL-2B tracking overshoot reserve (0.35m/s). Actual-state stopping
  // feasibility below remains separate and is never relaxed by this governor.
  out.stopping_cap = SpaceSpeed(distance - .1, s.acceleration, lag, c);
  out.space_cap =
      std::max(0., SpaceSpeed(distance - .1, c.accel, lag, c) - .35);
  out.chosen_cap = std::min({out.raw_cap, out.curve_cap, out.space_cap});
  out.speed_reason = out.chosen_cap == out.space_cap ? "verified_stopping_space"
                     : out.chosen_cap == out.curve_cap ? "curvature"
                                                       : "requested_cap";
  if (out.assisted && !out.recovery && out.chosen_cap > 5. / 3.6) {
    out.chosen_cap = 5. / 3.6;
    out.speed_reason = "experimental_near_blind_assist";
  }
  if (out.recovery) {
    out.chosen_cap = std::min(3. / 3.6, out.chosen_cap);
    out.speed_reason = "near_field_recovery_3kph";
  }
  if (out.required_stopping_m > distance) {
    out.reason = "insufficient_stopping_space";
    return false;
  }
  double travelled = 0, v = s.speed, a = std::max(-c.decel, s.acceleration);
  size_t index = 1;
  for (int i = 0; i < 400; ++i) {
    while (index < out.path.size() - 1 && out.path[index].s < travelled)
      ++index;
    const auto &left = out.path[index - 1], &right = out.path[index];
    double q = std::clamp(
        (travelled - left.s) / std::max(1e-6, right.s - left.s), 0., 1.);
    out.trajectory.push_back({left.x + q * (right.x - left.x),
                              left.y + q * (right.y - left.y),
                              left.yaw + q * (right.yaw - left.yaw),
                              left.k + q * (right.k - left.k), v, a, i * .05});
    double remaining = std::max(0., distance - travelled);
    double cap =
        std::min(out.chosen_cap, SpaceSpeed(remaining - .1, a, lag, c));
    // LOCAL-2C: scalar SpeedProfiler clips a lower cap immediately and is
    // deliberately unsuitable for a physical v/a trajectory. Use its shared
    // analytical transition, retaining endpoint acceleration (not average
    // delta-v/dt). Otherwise a tiny overspeed provokes a brake/reaccelerate
    // loop.
    const auto step =
        AdvancePhysicalSpeed({cap, v, a, .05}, {c.accel, c.decel, c.jerk});
    const double new_v = step.target_speed;
    a = step.target_acceleration;
    travelled += .5 * (v + new_v) * .05;
    v = new_v;
    if (travelled > distance + .05) {
      out.reason = "terminal_braking_violation";
      out.trajectory.clear();
      return false;
    }
    if (i > 40 && v < .03 && remaining < .4) {
      out.trajectory.back()[4] = 0;
      break;
    }
  }
  return true;
}
} // namespace local_planning
