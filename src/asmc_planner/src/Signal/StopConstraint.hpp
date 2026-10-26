#pragma once
#include "Signal/SignalPolicy.hpp"

namespace asmc_planner::signal {
// Native constraint. The opt-in explicit binding transports it; no existing
// ROS field is reinterpreted as this structure.
struct StopConstraint {
  Key key;
  std::uint64_t target_revision=0;
  double physical_line_s=std::numeric_limits<double>::quiet_NaN();
  double clearance=.5;
  Point physical_line{NAN,NAN}, approach{NAN,NAN};
  double position_error=.5;
  bool braking_only=false;
  bool valid() const {
    return key.valid() && target_revision>0 && std::isfinite(physical_line_s);
  }
};
}  // namespace asmc_planner::signal
