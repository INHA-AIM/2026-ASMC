#include "Planning/GpsModeSource.hpp"

#include <utility>

namespace asmc_planner {
GpsModeSource::GpsModeSource(ros::NodeHandle nh) : nh_(std::move(nh)) {
  mode_pub_ = nh_.advertise<std_msgs::UInt8>("/planning/mode", 1, true);
  gps_sub_ = nh_.subscribe("/gps", 1, &GpsModeSource::gpsCallback, this);
  feedback_sub_ = nh_.subscribe("/planning/frenet_feedback_v2", 1,
                                &GpsModeSource::feedbackCallback, this);
  recovery_timer_ = nh_.createSteadyTimer(ros::WallDuration(0.05),
                                         &GpsModeSource::recoveryWatchdog, this);
  std_msgs::UInt8 initial;
  initial.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(initial);
  ROS_INFO("Planning mode NORMAL: waiting for raw /gps");
}

void GpsModeSource::gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message) {
  if (!message) return;
  const PlanningMode next = DetectPlanningMode(*message);
  if (next == PlanningMode::JAMMING) {
    recovering_ = false;
    ready_frames_ = 0;
    publishMode(next, "raw GPS jamming sentinel");
  } else if (mode_ == PlanningMode::JAMMING && !recovering_) {
    // GPS returning does not mean source-aligned pose, LiDAR objects and a
    // usable normal trajectory have already arrived. Keep the live camera
    // controller selected for this bounded handoff interval.
    recovering_ = true;
    recovery_started_ = ros::SteadyTime::now();
    recovery_source_time_ = ros::Time::now();
    last_ready_stamp_ = ros::Time();
    ready_frames_ = 0;
    ROS_INFO("GPS recovered: waiting for fresh normal trajectories (maximum 5 s)");
  }
}

void GpsModeSource::feedbackCallback(const asmc_msgs::FrenetFeedback::ConstPtr& message) {
  if (!message || !recovering_) return;
  const double age = (ros::Time::now() - message->header.stamp).toSec();
  if (!message->command_accepted || !message->plan_valid ||
      message->trajectory_source != asmc_msgs::FrenetFeedback::TRAJECTORY_NEW ||
      message->header.frame_id != "map" ||
      message->header.stamp <= recovery_source_time_ || age < 0.0 || age > 0.15) {
    ready_frames_ = 0;
    return;
  }
  if (message->header.stamp <= last_ready_stamp_) return;
  last_ready_stamp_ = message->header.stamp;
  if (++ready_frames_ >= 2) {
    recovering_ = false;
    publishMode(PlanningMode::NORMAL, "two fresh normal trajectories ready");
  }
}

void GpsModeSource::recoveryWatchdog(const ros::SteadyTimerEvent&) {
  if (recovering_ && (ros::SteadyTime::now() - recovery_started_).toSec() >= 5.0) {
    recovering_ = false;
    // Do not drive indefinitely with the camera after a failed handoff.
    // The normal controller/selector's existing safe stop remains authoritative.
    publishMode(PlanningMode::NORMAL, "GPS recovery deadline; normal safety checks apply");
  }
}

void GpsModeSource::publishMode(PlanningMode next, const char* reason) {
  if (next == mode_) return;
  ROS_WARN("Planning mode %s -> %s: %s", PlanningModeName(mode_),
           PlanningModeName(next), reason);
  mode_ = next;
  std_msgs::UInt8 output;
  output.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(output);
}
}  // namespace asmc_planner
