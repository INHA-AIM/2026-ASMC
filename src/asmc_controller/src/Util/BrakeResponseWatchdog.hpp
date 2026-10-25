#pragma once
#include <asmc_msgs/control_binding.hpp>
#include <asmc_msgs/braking_response.hpp>

// Operational tD -> observed physical response monitor. ROS time is used once
// to account for transport from the verified common ROS clock; all subsequent
// deadline checks use this process's monotonic clock. Time is never ownership.
class BrakeResponseWatchdog {
 public:
  void Request(const asmc_msgs::binding::Request& r,double ros_now,double steady_now) {
    if(!r.brake_level)return;
    const bool same_epoch=controller_epoch_==r.token.controller_epoch && behavior_epoch_==r.token.behavior_epoch;
    if(same_epoch && generation_==r.brake_generation && level_>=r.brake_level)return;
    const bool retired_epoch=retired_controller_epoch_==r.token.controller_epoch &&
        retired_behavior_epoch_==r.token.behavior_epoch;
    if(retired_epoch && (r.brake_generation<retired_generation_ ||
       (r.brake_generation==retired_generation_ && r.brake_level<=retired_level_)))return;
    if(!r.brake_generation || r.brake_level>2 ||
       !std::isfinite(r.brake_decision_ros) || r.brake_decision_ros<=0 ||
       !std::isfinite(r.brake_speed) || r.brake_speed<0 ||
       !std::isfinite(r.brake_acceleration) || !std::isfinite(r.brake_budget) ||
       r.brake_budget<.5 || !std::isfinite(ros_now) || ros_now<r.brake_decision_ros ||
       ros_now-r.brake_decision_ros>5) {Fail(8);return;}
    generation_=r.brake_generation;level_=r.brake_level;
    controller_epoch_=r.token.controller_epoch;behavior_epoch_=r.token.behavior_epoch;
    baseline_speed_=r.brake_speed;baseline_acceleration_=r.brake_acceleration;
    last_stamp_=r.brake_state_stamp;
    deadline_=steady_now+r.brake_budget-(ros_now-r.brake_decision_ros);
    waiting_=baseline_speed_>.15 || baseline_acceleration_>.2;
    holding_=!waiting_;observed_=false;
  }
  bool Observe(double now,double stamp,double speed,double acceleration,bool valid) {
    if(waiting_ && now>deadline_) {Fail(1);waiting_=false;}
    if(valid && std::isfinite(speed) && std::isfinite(acceleration) && stamp>last_stamp_) {
      last_stamp_=stamp;
      if(holding_ && speed>.3)Fail(4);
      if(waiting_ && asmc_msgs::braking::PhysicalResponse(level_,baseline_speed_,
          baseline_acceleration_,speed,acceleration)) {waiting_=false;observed_=true;}
    }
    return failed_;
  }
  void Released() {if(!failed_) {Retire();waiting_=holding_=false;generation_=0;level_=0;}}
  bool failed() const {return failed_;}
  bool observed() const {return observed_;}
  std::uint64_t generation() const {return generation_;}
  unsigned level() const {return level_;}
  double remaining(double steady_now) const {
    return waiting_ ? std::max(0.,deadline_-steady_now) : 0.;
  }
  void Fail(unsigned reason=2) {failed_=true;failure_mask_|=reason;ever_failed_=true;}
  unsigned failure_mask() const {return failure_mask_;}
  bool ever_failed() const {return ever_failed_;}
  bool AcknowledgePathRecovery(bool safe) {
    if(!safe || failure_mask_!=2)return false;
    Retire();failed_=false;failure_mask_=0;waiting_=false;holding_=true;generation_=0;level_=0;
    return true; // The event log/ever_failed flag is retained.
  }
 private:
  void Retire() {
    if(!generation_)return;
    retired_generation_=generation_;retired_level_=level_;
    retired_controller_epoch_=controller_epoch_;retired_behavior_epoch_=behavior_epoch_;
  }
  std::string controller_epoch_,behavior_epoch_,retired_controller_epoch_,retired_behavior_epoch_;
  std::uint64_t retired_generation_=0;
  unsigned retired_level_=0;
  std::uint64_t generation_=0;
  unsigned level_=0;
  unsigned failure_mask_=0;
  double baseline_speed_=0,baseline_acceleration_=0,last_stamp_=0,deadline_=0;
  bool waiting_=false,holding_=false,observed_=false,failed_=false;
  bool ever_failed_=false;
};
