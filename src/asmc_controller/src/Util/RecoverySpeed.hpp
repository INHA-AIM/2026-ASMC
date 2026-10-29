#pragma once
#include <algorithm>
#include <cmath>
#include <optional>

// Forward solver boundary only; the signed measured state is retained.
inline std::optional<double> BoundedForwardSolverSpeed(double measured,double rollback=0.) {
  if(!std::isfinite(measured)||!std::isfinite(rollback)||rollback<0||rollback>.05||measured < -rollback)return {};
  return std::max(0.,measured);
}

// Limit only rising acceleration (including brake release). A smaller upper
// bound/stronger braking request always takes effect immediately.
inline double AccelerationRiseCeiling(double requested,double previous,double dt,double rate) {
  if(!(rate>0))return requested;
  if(!std::isfinite(previous+dt+rate)||dt<=0)return std::min(requested,0.);
  return std::min(requested,previous+rate*std::min(dt,.1));
}

inline double CombinedSpeedCap(double validation,double approach,bool approaching) {
  if(!approaching||approach<=0)return validation;
  return validation>0?std::min(validation,approach):approach;
}

// Extra bound for an authorized, currently linked internal recovery. The
// command's target is explicit; near-term reference acceleration must not
// reinforce an already excessive measured acceleration on every replan.
inline double RecoveryAccelerationLimit(double target,double speed,double acceleration,
    double response_budget,double minimum,double maximum) {
  if(!std::isfinite(target) || target<=0 || !std::isfinite(speed) || speed<0 ||
     !std::isfinite(acceleration) || !std::isfinite(response_budget) || response_budget<.5)
    return minimum;
  const double projected=speed+std::max(0.,acceleration)*response_budget;
  return std::clamp((target-projected)/response_budget,minimum,maximum);
}
