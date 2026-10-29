#pragma once
#include "Frenet/Speed/SpeedProfiler.hpp"
#include <algorithm>
#include <cmath>
namespace asmc_planner {
// Same physical jerk/deceleration transition as the shared speed profiler.
inline double ShadowApproachDistance(double speed,double target,double retained=0,double acceleration=0) {
  double v=std::abs(speed),a=acceleration,d=0;
  if(!std::isfinite(v+a+target)||target<=0)return std::max(5.,2*v);
  for(int i=0;i<400 && v>target+.01;++i){
    const auto next=AdvancePhysicalSpeed({target,v,a,.05},{1.,1.5,2.});
    d+=.5*(v+next.target_speed)*.05;v=next.target_speed;a=next.target_acceleration;
  }
  return std::min(250.,std::max(retained,std::max(2*std::abs(speed),d+.5*std::abs(speed)+2.)));
}
}
