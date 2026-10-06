#pragma once
#include <algorithm>
#include <cmath>

// Only for a valid planner STOP with fresh valid ego state. Watchdog failures
// and emergency STOP retain the original hard-stop policy. A recent successful
// steering command decays at the configured rate; it is never latched indefinitely.
inline double BrakingSteering(double previous_delta,double successful_age,
                              double speed,double rate_limit) {
    if(!std::isfinite(previous_delta)||!std::isfinite(successful_age)||
       !std::isfinite(speed)||!std::isfinite(rate_limit)||successful_age<0||
       speed<.3||rate_limit<=0)return 0;
    const double magnitude=std::max(0.0,std::fabs(previous_delta)-rate_limit*successful_age);
    return std::copysign(magnitude,previous_delta);
}
