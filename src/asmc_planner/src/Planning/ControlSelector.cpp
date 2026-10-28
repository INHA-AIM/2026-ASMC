#include "Planning/ControlSelector.hpp"

#include <utility>
#include <cmath>

namespace asmc_planner {
ControlSelector::ControlSelector(ros::NodeHandle nh) : nh_(std::move(nh)) {
  ros::NodeHandle("~").param("command_timeout_s", command_timeout_s_, 0.25);
  if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0)
    command_timeout_s_ = 0.25;
  ros::NodeHandle("~").param("local_handoff_enabled", local_handoff_enabled_, false);
  if (local_handoff_enabled_) {
    local_plan_sub_ = nh_.subscribe<asmc_msgs::LocalTrajectory>(
        "/planning/local/trajectory", 1, [this](const asmc_msgs::LocalTrajectory::ConstPtr& plan) {
          local_plan_ = plan;
          local_plan_received_ = ros::SteadyTime::now();
        });
  }
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
    handoff_pending_ = false;
    publishStop();
    publishDiagnostics();
    return;
  }
  const auto next = static_cast<PlanningMode>(message->data);
  if (!has_mode_ || next != mode_) {
    ROS_WARN("Control selector source: %s -> %s", has_mode_ ?
             PlanningModeName(mode_) : "NONE", PlanningModeName(next));
    const auto now = ros::SteadyTime::now();
    const bool target_ready = next == PlanningMode::JAMMING ? localPlanFresh() :
        normal_received_ && (now-normal_received_at_).toSec() <= .1;
    handoff_pending_ = local_handoff_enabled_ && has_mode_ && commandFresh() && target_ready;
    handoff_deadline_ = now + ros::WallDuration(.1);
    awaiting_selected_command_ = true;
    // Leave the last actually issued command untouched for <=100ms. Do not
    // republish it, refresh its watchdog, or keep running the old source.
    if (!handoff_pending_) publishStop();
  }
  has_mode_ = true;
  mode_ = next;
  publishDiagnostics();
}

void ControlSelector::normalCallback(
    const morai_msgs::CtrlCmd::ConstPtr& message) {
  if (!message) return;
  normal_received_ = true;
  normal_received_at_ = ros::SteadyTime::now();
  if (has_mode_ && mode_ == PlanningMode::NORMAL) {
    selected_received_at_ = ros::SteadyTime::now();
    awaiting_selected_command_ = false;
    handoff_pending_ = false;
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
    handoff_pending_ = false;
    control_pub_.publish(*message);
  }
}

bool ControlSelector::commandFresh() const {
  return has_mode_ && !awaiting_selected_command_ &&
      (ros::SteadyTime::now() - selected_received_at_).toSec() <= command_timeout_s_;
}

bool ControlSelector::localPlanFresh() const {
  if (!local_plan_ || !local_plan_->valid || local_plan_->points.size() < 2 ||
      local_plan_->header.frame_id.rfind("local/", 0) != 0) return false;
  const auto now = ros::Time::now();
  const double age = (now-local_plan_->header.stamp).toSec();
  const double source_age = (now-local_plan_->scene_stamp).toSec();
  return age >= 0.0 && age <= .25 && source_age >= age && source_age <= .35 &&
      (ros::SteadyTime::now()-local_plan_received_).toSec() <= .15;
}

bool ControlSelector::handoffWaiting() const {
  const auto now = ros::SteadyTime::now();
  return handoff_pending_ && now < handoff_deadline_ &&
      (now-selected_received_at_).toSec() <= command_timeout_s_ &&
      (mode_ != PlanningMode::JAMMING || localPlanFresh());
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
  if (!commandFresh() && !handoffWaiting()) {
    handoff_pending_ = false;
    publishStop();
  }
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
      " command_fresh=" + (commandFresh() ? "true" : "false") +
      " local_ready=" + (localPlanFresh() ? "true" : "false") +
      " handoff_pending=" + (handoffWaiting() ? "true" : "false");
  diagnostics_pub_.publish(message);
}
}  // namespace asmc_planner
