#pragma once
#include <asmc_msgs/ControlHandoff.h>
#include <cmath>

namespace asmc_msgs { namespace handoff {
inline bool Age(const ros::Time& stamp,const ros::Time& now,double limit) {
  const double age=(now-stamp).toSec();
  return !stamp.isZero() && age>=0 && age<=limit;
}
inline bool Candidate(const ControlHandoff& c,const ControlHandoff& request,
                      const ros::Time& now) {
  return request.valid && request.epoch && c.epoch==request.epoch &&
      c.target_mode==request.target_mode && c.target_mode<=1 && c.valid &&
      c.header.stamp>=request.header.stamp && Age(c.header.stamp,now,.1) &&
      Age(c.state_stamp,now,.2) && Age(c.trajectory_stamp,now,.25) &&
      !c.state_frame.empty() && c.longl_cmd_type==1 &&
      std::isfinite(c.steering+c.accel+c.brake) &&
      c.accel>=0 && c.accel<=1 && c.brake>=0 && c.brake<=1 &&
      std::abs(c.steering)<=.698131701 &&
      !(c.accel>0 && c.brake>0);
}
} }
