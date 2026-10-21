#pragma once
#include <cmath>
#include <optional>
#include <initializer_list>

namespace asmc_planner {
struct RecoveryPose { double stamp,x,y,yaw,speed,yaw_rate; };
// Compare two measurements at GPS source time. This does not change either
// estimate or invent a pose while GPS is unavailable.
inline std::optional<double> GpsPoseResidual(const RecoveryPose& state,
    double gps_stamp,double antenna_x,double antenna_y,double offset_x,double offset_y,
    double maximum_past_s,double maximum_future_s) {
  for(double value:{state.stamp,state.x,state.y,state.yaw,state.speed,state.yaw_rate,
      gps_stamp,antenna_x,antenna_y,offset_x,offset_y})
    if(!std::isfinite(value))return {};
  const double dt=gps_stamp-state.stamp;
  if(state.stamp<=0 || gps_stamp<=0 || dt< -maximum_past_s || dt>maximum_future_s)return {};
  const double heading=state.yaw+state.yaw_rate*dt;
  const double midpoint=state.yaw+.5*state.yaw_rate*dt;
  const double x=state.x+state.speed*dt*std::cos(midpoint)+
      offset_x*std::cos(heading)-offset_y*std::sin(heading);
  const double y=state.y+state.speed*dt*std::sin(midpoint)+
      offset_x*std::sin(heading)+offset_y*std::cos(heading);
  return std::hypot(antenna_x-x,antenna_y-y);
}
class GpsRecoveryGate {
 public:
  GpsRecoveryGate(double residual_limit=0.3,unsigned required=3)
      :limit_(residual_limit),required_(required) {}
  void reset() { count_=0;last_good_source_=0; }
  bool observe(double gps_source,std::optional<double> residual) {
    if(!residual || !std::isfinite(*residual) || *residual<0 || *residual>limit_ ||
       !std::isfinite(gps_source)||gps_source<=0) { count_=0;return false; }
    if(gps_source<last_good_source_)reset();
    if(gps_source>last_good_source_) {
      last_good_source_=gps_source;
      if(count_<required_)++count_;
    }
    return count_>=required_;
  }
  unsigned confirmations() const { return count_; }
 private:
  double limit_;
  unsigned required_,count_=0;
  double last_good_source_=0;
};
}  // namespace asmc_planner
