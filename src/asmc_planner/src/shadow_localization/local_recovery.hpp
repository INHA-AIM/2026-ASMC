#pragma once
// Optional LOCAL recovery is a trajectory source, never a second controller.
// Unknown failure reasons and moving/pending hazards cannot acquire permission.
#include "local/motion.hpp"
#include <asmc_msgs/LocalTrajectory.h>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace shadow_localization {
inline bool RecoverableLocalFailure(const std::string& reason) {
  return reason=="shadow_native_blocked" || reason=="shadow_start_unverified" ||
      reason=="shadow_no_verified_prefix" || reason=="reference_no_verified_prefix" ||
      reason=="localization_obb" || reason=="localization_map_boundary" ||
      reason=="insufficient_stopping_space" || reason=="no_speed_geometry";
}
// Map anchoring is not a prerequisite for a fresh, aligned local trajectory.
// Sensor/time/geometry failures are deliberately absent from this whitelist.
inline bool RecoverableAnchorFailure(const std::string& reason) {
  return reason=="motion_epoch_changed" || reason=="gps_innovation_exceeded" ||
      reason=="gps_recovery_unstable" || reason=="reanchor_wait_stopped" ||
      reason=="stopped_reanchor_warmup" || reason=="initial_anchor_warmup" ||
      reason=="epoch_reset" || reason=="anchor_time_limit" ||
      reason=="anchor_distance_limit" || reason=="anchor_no_past_sample";
}
class LocalRecoveryPolicy {
 public:
  bool active=false;
  std::string reason="disabled";
  void reset(){active=false;blocked_since_=healthy_since_=last_=-1;blocked_count_=0;reason="reset";}
  bool update(double now,bool allowed,bool native_progress,bool recoverable,
              bool candidate_valid,bool nearly_stopped,bool dynamic_risk) {
    if(!std::isfinite(now)||now<last_){reset();reason="clock_reset";return false;}
    last_=now;
    if(!allowed||dynamic_risk){reset();reason=dynamic_risk?"dynamic_or_pending_yield":"input_or_mode_invalid";return false;}
    if(native_progress){
      blocked_since_=-1;blocked_count_=0;
      if(healthy_since_<0)healthy_since_=now;
      if(!active||now-healthy_since_>=1.0){active=false;reason="localization_ready";return false;}
    } else {
      healthy_since_=-1;
      if(!recoverable){active=false;blocked_since_=-1;reason="nonrecoverable_native_failure";return false;}
      // Fresh LOCAL can take over before a full stop. Require consecutive
      // native failures and at least one nominal plan interval.
      if(blocked_since_<0)blocked_since_=now;
      ++blocked_count_;
    }
    if(!candidate_valid){
      // Withhold output while stale, but retain selected recovery ownership.
      // A subsequent fresh path can resume without first demanding a stop.
      // Epoch/input/hazard changes above still revoke ownership immediately.
      if(!active){blocked_since_=-1;blocked_count_=0;}
      reason="local_candidate_unavailable";return false;
    }
    if(!active&&blocked_since_>=0&&blocked_count_>=2&&now-blocked_since_>=.08)active=true;
    reason=active?(native_progress?"local_recovery_return_debounce":"local_recovery_active"):"waiting_for_repeat_native_failure";
    return active;
  }
 private:
  double blocked_since_=-1,healthy_since_=-1,last_=-1;
  unsigned blocked_count_=0;
};

// Both origins are at EXACTLY the candidate header time. Source timestamps,
// velocities and braking profile are preserved; no future sample/ego-motion
// extrapolation or fabricated free space is introduced by this conversion.
inline std::optional<asmc_msgs::LocalTrajectory> ReframeLocalRecovery(
    const asmc_msgs::LocalTrajectory& in,const local_planning::Motion& target,
    const std::string& frame,const std::string& run,double now,double received_age) {
  const double t=in.header.stamp.toSec(),scene=in.scene_stamp.toSec();
  const auto& e=in.ego_at_plan;const auto& q=e.pose.pose.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!in.valid||in.run_id!=run||in.points.size()<2||in.points.size()>400||
     in.header.frame_id.rfind("local/",0)!=0||frame.rfind("local/",0)!=0||
     e.header.frame_id!=in.header.frame_id||e.header.stamp!=in.header.stamp||
     e.child_frame_id!="base_link"||!std::isfinite(norm)||std::abs(norm-1)>.01||
     !std::isfinite(t+scene+now+received_age+target.t+target.x+target.y+target.yaw+target.v+target.w)||
     std::abs(target.t-t)>1e-6||t<=0||scene<=0||scene>t||now<t||
     now-t>.25||now-scene>.35||received_age<0||received_age>.25||
     !std::isfinite(in.dt)||in.dt<.01||in.dt>.2||
     !std::isfinite(e.pose.pose.position.x+e.pose.pose.position.y+e.twist.twist.linear.x)||
     std::abs(e.twist.twist.linear.x-target.v)>.5)return {};
  const double from_yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  const double delta=target.yaw-from_yaw,c=std::cos(delta),s=std::sin(delta);
  auto out=in;out.header.frame_id=frame;out.ego_at_plan.header=out.header;
  out.ego_at_plan.pose.pose.position.x=target.x;out.ego_at_plan.pose.pose.position.y=target.y;
  out.ego_at_plan.pose.pose.orientation.x=out.ego_at_plan.pose.pose.orientation.y=0;
  out.ego_at_plan.pose.pose.orientation.z=std::sin(target.yaw*.5);
  out.ego_at_plan.pose.pose.orientation.w=std::cos(target.yaw*.5);
  out.ego_at_plan.twist.twist.linear.x=target.v;out.ego_at_plan.twist.twist.angular.z=target.w;
  double progress=0;
  for(auto& point:out.points){
    if(!std::isfinite(point.x+point.y+point.yaw+point.curvature+point.velocity+point.acceleration)||point.velocity<0)return {};
    const double x=point.x-e.pose.pose.position.x,y=point.y-e.pose.pose.position.y;
    progress=std::max(progress,std::hypot(x,y));
    point.x=target.x+c*x-s*y;point.y=target.y+s*x+c*y;point.yaw+=delta;
  }
  // Producer already validated footprint, stopping space and speed. Do not
  // discard its short recovery solely for being below 1m / 0.3m/s. Never
  // increase its speed or convert a valid zero-speed wait to movement.
  if(progress<.2||std::none_of(out.points.begin(),out.points.end(),[](const auto& p){return p.velocity>.01;}))return {};
  return out;
}
} // namespace shadow_localization
