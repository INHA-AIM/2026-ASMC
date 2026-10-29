#pragma once
#include <algorithm>
#include <cmath>

namespace asmc_planner {
// Known-zone preparation only. The retained episode distance prevents slowing
// from shrinking the window and releasing its own cap before the zone begins.
inline double ShadowApproachDistance(double speed,double target,double retained=0) {
  const double v=std::abs(speed),base=std::max(5.,2*v);
  if(!(target>0))return base;
  const double reserve=v>target?(v*v-target*target)/3.+.5*v+2.:0.;
  return std::min(250.,std::max(retained,base+reserve));
}
}
