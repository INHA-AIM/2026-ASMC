#pragma once

#include <morai_msgs/CtrlCmd.h>
#include <asmc_msgs/LocalTrajectory.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include "Planning/PlanningMode.hpp"

namespace asmc_planner {
class ControlSelector final {
 public:
  explicit ControlSelector(ros::NodeHandle nh);
 private:
  void modeCallback(const std_msgs::UInt8::ConstPtr& message);
  void normalCallback(const morai_msgs::CtrlCmd::ConstPtr& message);
  void jammingCallback(const morai_msgs::CtrlCmd::ConstPtr& message);
  void diagnosticsCallback(const ros::SteadyTimerEvent&);
  void watchdogCallback(const ros::SteadyTimerEvent&);
  void publishStop();
  void publishSelected(const morai_msgs::CtrlCmd&);
  bool commandFresh() const;
  bool localPlanFresh() const;
  bool handoffWaiting() const;
  void publishDiagnostics();

  ros::NodeHandle nh_;
  ros::Subscriber mode_sub_, normal_sub_, jamming_sub_;
  ros::Subscriber local_plan_sub_;
  asmc_msgs::LocalTrajectory::ConstPtr local_plan_;
  ros::Publisher control_pub_, diagnostics_pub_;
  ros::SteadyTimer diagnostics_timer_;
  ros::SteadyTimer watchdog_timer_;
  ros::SteadyTime selected_received_at_;
  ros::SteadyTime local_plan_received_, normal_received_at_, handoff_deadline_;
  bool local_handoff_enabled_ = false, handoff_pending_ = false;
  double command_timeout_s_ = 0.25;
  bool awaiting_selected_command_ = true;
  PlanningMode mode_ = PlanningMode::NORMAL;
  bool has_mode_ = false;
  bool normal_received_ = false;
  bool jamming_received_ = false;
  bool has_wire_command_ = false, handoff_steering_limited_ = false;
  double handoff_steering_rate_ = .1;
  ros::SteadyTime wire_issued_at_;
  morai_msgs::CtrlCmd last_wire_command_;
};
}  // namespace asmc_planner
