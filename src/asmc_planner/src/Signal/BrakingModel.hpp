#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <asmc_msgs/braking_response.hpp>

namespace asmc_planner::signal {

// Model limits, not measured guarantees. Both callers use a rear-axle distance
// to the final endpoint; neither this function nor the policy adds a buffer.
struct BrakeLimits {
  double deceleration = 4.0;
  double jerk = 4.8;
  double positive_acceleration = 2.4;
};
struct MotionProjection { double distance=0,speed=0; bool valid=false; };
inline MotionProjection ProjectMeasuredState(double speed,double acceleration,double age) {
  MotionProjection out;
  if(!std::isfinite(speed) || !std::isfinite(acceleration) || !std::isfinite(age) ||
     speed<0 || age<0) return out;
  const double t=acceleration<0 ? std::min(age,speed/-acceleration) : age;
  out.distance=speed*t+.5*acceleration*t*t;
  out.speed=std::max(0.0,speed+acceleration*t);out.valid=true;return out;
}

inline double ResponseBudget(double measured_max_delay) {
  return std::isfinite(measured_max_delay) && measured_max_delay >= 0
      ? std::max(0.5, measured_max_delay + 0.1) : 0.5;
}

inline double StopBoundaryAcceleration(double v, double raw_a,
                                       const BrakeLimits& limits) {
  const double brake = std::min(limits.deceleration,
      std::sqrt(2.0 * limits.jerk * std::max(0.0, v)));
  return std::clamp(raw_a, -brake, limits.positive_acceleration);
}

// Same jerk-limited down/hold/release construction as SpatialStop::Brake.
// Unrepresentable raw boundary conditions fail closed instead of silently
// turning measured acceleration into a more optimistic boundary condition.
inline double MinimumStoppingDistance(double v, double a,
                                     const BrakeLimits& limits,
                                     double pending_delay = 0.0) {
  const double inf = std::numeric_limits<double>::infinity();
  if (!std::isfinite(v) || !std::isfinite(a) || !std::isfinite(pending_delay) ||
      v < 0 || pending_delay < 0 || !std::isfinite(limits.deceleration) ||
      !std::isfinite(limits.jerk) || limits.deceleration <= 0 || limits.jerk <= 0)
    return inf;
  double s = 0;
  if (pending_delay > 0) {
    // If braking would already stop the car during the pending interval, use
    // the observed braking only until zero speed; never integrate in reverse.
    const double t = a < 0 ? std::min(pending_delay, v / -a) : pending_delay;
    s = v*t + .5*a*t*t;
    v = std::max(0.0, v+a*t);
    if (v == 0) return s;
  }
  // Measured residual deceleration at rest cannot move the car backwards.
  if (v <= 1e-9 && a <= 1e-9) return s;
  if (a > limits.deceleration + 1e-9) return inf;
  // A stronger measured deceleration is not evidence that stopping became
  // impossible. Raising a negative boundary toward the feasible release
  // envelope predicts a LONGER stop, so it is a conservative upper bound.
  // Retain the raw observation in records and in the physical response watch.
  a=std::max(a,-std::min(limits.deceleration,std::sqrt(2*limits.jerk*v)));
  const double j = limits.jerk;
  const double peak = std::min(limits.deceleration, std::sqrt(j*v+.5*a*a));
  if (peak <= 0) return inf;
  const bool releasing = a < -peak;
  const double down = std::abs(a+peak)/j;
  const double v1 = v+(releasing ? -1 : 1)*(a*a-peak*peak)/(2*j);
  const double hold = (v1-peak*peak/(2*j))/peak;
  if (hold < -1e-9) return inf;
  const auto advance = [&](double t, double jerk) {
    s += v*t+.5*a*t*t+jerk*t*t*t/6;
    v += a*t+.5*jerk*t*t;
    a += jerk*t;
  };
  advance(down, releasing ? j : -j);
  advance(std::max(0.0, hold), 0);
  advance(peak/j, j);
  return std::abs(v) < 1e-6 && std::abs(a) < 1e-6 ? s : inf;
}

enum class BrakeResponse { IDLE, WAITING, OBSERVED, FAILED };

class ResponseMonitor {
 public:
  // Only a stronger request starts a new interval. Heartbeats and target
  // changes must call neither Reset nor Begin with a different request level.
  void Begin(int level, double now, double speed, double acceleration,
             double budget, double source_stamp=0) {
    // A stationary HOLD has no measurable deceleration onset. It is neither
    // a response acknowledgement nor a timed-out moving brake request.
    if(speed<=.15 && acceleration<=.2) return;
    if (level <= level_) return;
    level_ = level; start_ = now; speed_ = speed; acceleration_ = acceleration;
    budget_ = budget; status_ = BrakeResponse::WAITING;
    last_source_=source_stamp;response_observed_=false;
  }
  BrakeResponse Observe(double now, double source_stamp, double speed,
                        double acceleration, bool valid) {
    if (status_ == BrakeResponse::IDLE) return status_;
    if (!std::isfinite(now) || now < start_) return status_;
    if (status_ == BrakeResponse::WAITING && now-start_ > budget_) {
      status_ = BrakeResponse::FAILED; ever_failed_ = true;
    }
    // INITIAL noise thresholds; release/operational enable requires measured
    // calibration. A prior service deceleration alone cannot acknowledge an
    // escalation to emergency braking.
    if (valid && source_stamp > last_source_ && std::isfinite(speed) &&
        std::isfinite(acceleration)) {
      last_source_ = source_stamp;
      if (asmc_msgs::braking::PhysicalResponse(level_,speed_,acceleration_,speed,acceleration)) {
        if (status_ != BrakeResponse::FAILED) status_ = BrakeResponse::OBSERVED;
        response_observed_ = true;
      }
    }
    return status_;
  }
  bool failed() const { return ever_failed_; }
  bool observed() const { return response_observed_; }
  double remaining(double now) const {
    return status_ == BrakeResponse::WAITING ? std::max(0.0, budget_-(now-start_)) : 0;
  }
  BrakeResponse status() const { return status_; }
 private:
  int level_ = 0;
  double start_ = 0, speed_ = 0, acceleration_ = 0, budget_ = .5, last_source_ = 0;
  bool ever_failed_ = false, response_observed_ = false;
  BrakeResponse status_ = BrakeResponse::IDLE;
};
}  // namespace asmc_planner::signal
