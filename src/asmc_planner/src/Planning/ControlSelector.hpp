#pragma once

#include <morai_msgs/CtrlCmd.h>
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
  bool commandFresh() const;
  void publishDiagnostics();

  ros::NodeHandle nh_;
  ros::Subscriber mode_sub_, normal_sub_, jamming_sub_;
  ros::Publisher control_pub_, diagnostics_pub_;
  ros::SteadyTimer diagnostics_timer_;
  ros::SteadyTimer watchdog_timer_;
  ros::SteadyTime selected_received_at_;
  double command_timeout_s_ = 0.25;
  bool awaiting_selected_command_ = true;
  PlanningMode mode_ = PlanningMode::NORMAL;
  bool has_mode_ = false;
  bool normal_received_ = false;
  bool jamming_received_ = false;
};
}  // namespace asmc_planner
