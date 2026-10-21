#include "Planning/GpsModeSource.hpp"

#include <utility>

namespace asmc_planner {
GpsModeSource::GpsModeSource(ros::NodeHandle nh) : nh_(std::move(nh)) {
  mode_pub_ = nh_.advertise<std_msgs::UInt8>("/planning/mode", 1, true);
  gps_sub_ = nh_.subscribe("/gps", 1, &GpsModeSource::gpsCallback, this);
  std_msgs::UInt8 initial;
  initial.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(initial);
  ROS_INFO("Planning mode NORMAL: waiting for raw /gps");
}

void GpsModeSource::gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message) {
  if (!message) return;
  const PlanningMode next = DetectPlanningMode(*message);
  if (next == mode_) return;
  ROS_WARN("Planning mode %s -> %s: raw GPS latitude/longitude/altitude %s",
           PlanningModeName(mode_), PlanningModeName(next),
           next == PlanningMode::JAMMING ? "all exactly zero" : "not all zero");
  mode_ = next;
  std_msgs::UInt8 output;
  output.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(output);
}
}  // namespace asmc_planner
