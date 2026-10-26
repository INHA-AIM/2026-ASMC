#pragma once

#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <asmc_msgs/FrenetFeedback.h>

#include "Planning/PlanningMode.hpp"

namespace asmc_planner {
class GpsModeSource final {
 public:
  explicit GpsModeSource(ros::NodeHandle nh);
 private:
  void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message);
  void feedbackCallback(const asmc_msgs::FrenetFeedback::ConstPtr& message);
  void recoveryWatchdog(const ros::SteadyTimerEvent&);
  void publishMode(PlanningMode next, const char* reason);
  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_;
  ros::Subscriber feedback_sub_;
  ros::SteadyTimer recovery_timer_;
  ros::Publisher mode_pub_;
  PlanningMode mode_ = PlanningMode::NORMAL;
  bool recovering_ = false;
  ros::SteadyTime recovery_started_;
  ros::Time recovery_source_time_, last_ready_stamp_;
  unsigned ready_frames_ = 0;
};
}  // namespace asmc_planner
