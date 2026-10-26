#pragma once
#include <algorithm>
#include <cmath>

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
