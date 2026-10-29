#pragma once
#include <asmc_msgs/control_handoff.hpp>
#include <morai_msgs/CtrlCmd.h>
#include <ros/ros.h>
#include "MPC/MPC.hpp"

// One bounded preparation phase; no second integrator or persistent second solve.
class HandoffParticipant {
  ros::Subscriber request_sub_,applied_sub_;
  ros::Publisher candidate_pub_;
  asmc_msgs::ControlHandoff request_;
  morai_msgs::CtrlCmd applied_;
  ros::SteadyTime received_,applied_at_;
  uint64_t seeded_=0;
  uint8_t target_=0;
 public:
  bool enabled=false;
  void setup(ros::NodeHandle& nh,bool on,uint8_t target) {
    enabled=on;target_=target;if(!on)return;
    candidate_pub_=nh.advertise<asmc_msgs::ControlHandoff>("/planning/jamming/handoff_candidate",4);
    request_sub_=nh.subscribe<asmc_msgs::ControlHandoff>("/planning/jamming/handoff_request",1,
      [this](const asmc_msgs::ControlHandoff::ConstPtr& m){if(m->epoch<request_.epoch)return;request_=*m;received_=ros::SteadyTime::now();});
    applied_sub_=nh.subscribe<morai_msgs::CtrlCmd>("/ctrl_cmd",1,
      [this](const morai_msgs::CtrlCmd::ConstPtr& m){applied_=*m;applied_at_=ros::SteadyTime::now();});
  }
  bool preparing() const {
    return enabled && request_.valid && request_.epoch && request_.target_mode==target_ &&
      !received_.isZero() && (ros::SteadyTime::now()-received_).toSec()<=.25;
  }
  void reset_seed() {seeded_=0;}
  void seed(double acceleration) {
    if(!preparing()||seeded_==request_.epoch)return;
    const bool fresh=!applied_at_.isZero()&&(ros::SteadyTime::now()-applied_at_).toSec()<=.25;
    ResetMPCForHandoff(fresh?applied_.steering:0.,acceleration);seeded_=request_.epoch;
  }
  void publish(bool valid,const morai_msgs::CtrlCmd& cmd,const ros::Time& state,
               const ros::Time& trajectory,uint64_t sequence,const std::string& frame,
               const std::string& reason) {
    if(!enabled||!preparing())return;
    asmc_msgs::ControlHandoff out;out.header.stamp=ros::Time::now();
    out.header.frame_id=ros::this_node::getName();out.epoch=request_.epoch;out.target_mode=target_;
    out.valid=valid;out.state_stamp=state;out.trajectory_stamp=trajectory;
    out.trajectory_sequence=sequence;out.state_frame=frame;out.reason=reason;
    out.longl_cmd_type=cmd.longlCmdType;out.steering=cmd.steering;out.accel=cmd.accel;out.brake=cmd.brake;
    candidate_pub_.publish(out);
  }
};
