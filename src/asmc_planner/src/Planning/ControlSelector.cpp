#include "Planning/ControlSelector.hpp"

#include <utility>
#include <cmath>

namespace asmc_planner {
ControlSelector::ControlSelector(ros::NodeHandle nh) : nh_(std::move(nh)) {
  ros::NodeHandle("~").param("command_timeout_s", command_timeout_s_, 0.25);
  if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0)
    command_timeout_s_ = 0.25;
  control_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
  diagnostics_pub_ = nh_.advertise<std_msgs::String>(
      "/planning/control_selector/diagnostics", 1, true);
  mode_sub_ = nh_.subscribe("/planning/mode", 1,
                            &ControlSelector::modeCallback, this);
  normal_sub_ = nh_.subscribe("/ctrl_cmd/normal", 1,
                              &ControlSelector::normalCallback, this);
  jamming_sub_ = nh_.subscribe("/ctrl_cmd/jamming", 1,
                               &ControlSelector::jammingCallback, this);
  diagnostics_timer_ = nh_.createSteadyTimer(
      ros::WallDuration(1.0), &ControlSelector::diagnosticsCallback, this);
  watchdog_timer_ = nh_.createSteadyTimer(
      ros::WallDuration(0.05), &ControlSelector::watchdogCallback, this);
  publishDiagnostics();
}

void ControlSelector::modeCallback(const std_msgs::UInt8::ConstPtr& message) {
  if (!message) return;
  if (message->data != static_cast<std::uint8_t>(PlanningMode::NORMAL) &&
      message->data != static_cast<std::uint8_t>(PlanningMode::JAMMING)) {
    ROS_ERROR_THROTTLE(1.0, "Control selector rejected unknown mode %u",
                       static_cast<unsigned>(message->data));
    has_mode_ = false;
    awaiting_selected_command_ = true;
    publishStop();
    publishDiagnostics();
    return;
  }
  const auto next = static_cast<PlanningMode>(message->data);
  if (!has_mode_ || next != mode_) {
    ROS_WARN("Control selector source: %s -> %s", has_mode_ ?
             PlanningModeName(mode_) : "NONE", PlanningModeName(next));
    awaiting_selected_command_ = true;
    publishStop();
  }
  has_mode_ = true;
  mode_ = next;
  publishDiagnostics();
}

void ControlSelector::normalCallback(
    const morai_msgs::CtrlCmd::ConstPtr& message) {
  if (!message) return;
  normal_received_ = true;
  if (has_mode_ && mode_ == PlanningMode::NORMAL) {
    selected_received_at_ = ros::SteadyTime::now();
    awaiting_selected_command_ = false;
    control_pub_.publish(*message);
  }
}

void ControlSelector::jammingCallback(
    const morai_msgs::CtrlCmd::ConstPtr& message) {
  if (!message) return;
  jamming_received_ = true;
  if (has_mode_ && mode_ == PlanningMode::JAMMING) {
    selected_received_at_ = ros::SteadyTime::now();
    awaiting_selected_command_ = false;
    control_pub_.publish(*message);
  }
}

bool ControlSelector::commandFresh() const {
  return has_mode_ && !awaiting_selected_command_ &&
      (ros::SteadyTime::now() - selected_received_at_).toSec() <= command_timeout_s_;
}

void ControlSelector::publishStop() {
  morai_msgs::CtrlCmd stop;
  stop.longlCmdType = 1;
  stop.brake = 1.0;
  control_pub_.publish(stop);
}

void ControlSelector::watchdogCallback(const ros::SteadyTimerEvent&) {
  // A silent controller must not leave the simulator holding its last throttle.
  // Unselected input traffic never refreshes this selected-source watchdog.
  if (!commandFresh()) publishStop();
}

void ControlSelector::diagnosticsCallback(const ros::SteadyTimerEvent&) {
  publishDiagnostics();
}

void ControlSelector::publishDiagnostics() {
  std_msgs::String message;
  message.data = std::string("mode=") +
      (has_mode_ ? PlanningModeName(mode_) : "UNKNOWN") +
      " selected_source=" +
      (has_mode_ ? (mode_ == PlanningMode::NORMAL ? "/ctrl_cmd/normal" :
                    "/ctrl_cmd/jamming") : "NONE") +
      " normal_received=" + (normal_received_ ? "true" : "false") +
      " jamming_received=" + (jamming_received_ ? "true" : "false") +
      " command_fresh=" + (commandFresh() ? "true" : "false");
  diagnostics_pub_.publish(message);
}
}  // namespace asmc_planner
