#pragma once
#include <algorithm>
#include "Signal/SignalPolicy.hpp"

namespace asmc_planner::signal {
// Position provenance only. It cannot validate a missing longitudinal speed,
// grant entry, or release a brake. GPS/IMU validity is checked by the caller.
class LocalizationContinuity {
 public:
  explicit LocalizationContinuity(Config config={}):config_(config),history_(config){}
  void Clear(){history_.Clear();last_={};received_=-1;++generation_;}
  void Push(const Pose& pose,double received) {
    if(!pose.valid || !std::isfinite(pose.stamp) || pose.stamp<=0 ||
       !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
       !std::isfinite(pose.speed) || pose.speed<0 || !std::isfinite(pose.acceleration) ||
       !std::isfinite(received)) {Clear();return;}
    if(last_.valid) {
      const double dt=pose.stamp-last_.stamp;
      if(dt==0)return; // Duplicates never renew freshness.
      if(dt<0){Clear();return;}
      if(dt>config_.interpolation_gap ||
         std::hypot(pose.x-last_.x,pose.y-last_.y)>
             2*config_.position_error+std::max(pose.speed,last_.speed)*dt)Clear();
    }
    history_.Push(pose);last_=pose;received_=received;
  }
  std::uint64_t generation()const{return generation_;}
  bool CoversMatched(double begin,const Pose& current,double ros_now,double steady_now)const {
    if(!std::isfinite(ros_now) || !std::isfinite(steady_now) ||
       !std::isfinite(current.stamp) || current.stamp<=0 ||
       ros_now<current.stamp || ros_now-current.stamp>config_.freshness ||
       received_<0 || steady_now<received_ || steady_now-received_>config_.freshness ||
       !std::isfinite(current.x) || !std::isfinite(current.y) || !std::isfinite(current.yaw))return false;
    Pose raw;
    return history_.At(current.stamp,raw) && history_.Covers(begin,current.stamp) &&
        std::hypot(raw.x-current.x,raw.y-current.y)<1e-6 &&
        std::abs(std::remainder(raw.yaw-current.yaw,2*std::acos(-1.0)))<1e-6;
  }
 private:
  Config config_;PoseHistory history_;Pose last_;double received_=-1;
  std::uint64_t generation_=1;
};
}
