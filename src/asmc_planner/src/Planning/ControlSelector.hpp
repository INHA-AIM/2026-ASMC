#pragma once

#include <morai_msgs/CtrlCmd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <asmc_msgs/control_handoff.hpp>
#include <array>

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
  void handoffRequest(const asmc_msgs::ControlHandoff::ConstPtr&);
  void handoffCandidate(const asmc_msgs::ControlHandoff::ConstPtr&);
  bool completeHandoff();
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
  bool smooth_handoff_=false,pending_handoff_=false;
  bool initial_anchor_ready_=true;
  std::string jamming_controller_="/localization_controller",jamming_frame_prefix_="local/";
  PlanningMode pending_mode_=PlanningMode::NORMAL;
  ros::Subscriber request_sub_,candidate_sub_,anchor_sub_;
  ros::Publisher handoff_trace_pub_;
  asmc_msgs::ControlHandoff request_;
  std::array<asmc_msgs::ControlHandoff,2> candidates_;
  ros::SteadyTime pending_started_,request_received_;
};
}  // namespace asmc_planner
