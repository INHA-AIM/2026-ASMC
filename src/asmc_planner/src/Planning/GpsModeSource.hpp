#pragma once

#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include "Planning/PlanningMode.hpp"

namespace asmc_planner {
class GpsModeSource final {
 public:
  explicit GpsModeSource(ros::NodeHandle nh);
 private:
  void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message);
  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_;
  ros::Publisher mode_pub_;
  PlanningMode mode_ = PlanningMode::NORMAL;
};
}  // namespace asmc_planner
