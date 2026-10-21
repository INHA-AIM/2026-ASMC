#pragma once

#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <asmc_msgs/VehicleState.h>
#include "Planning/GpsRecoveryGate.hpp"

#include "Planning/PlanningMode.hpp"

namespace asmc_planner {
class GpsModeSource final {
 public:
  explicit GpsModeSource(ros::NodeHandle nh);
 private:
  void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message);
  void stateCallback(const asmc_msgs::VehicleState::ConstPtr& message);
  void tryRecover();
  void publishMode(PlanningMode mode);
  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher mode_pub_;
  PlanningMode mode_ = PlanningMode::NORMAL;
  bool recovery_enabled_=false;
  double maximum_past_s_=0,maximum_future_s_=0,maximum_receive_age_s_=0;
  double gps_offset_x_=0,gps_offset_y_=0;
  GpsRecoveryGate recovery_;
  morai_msgs::GPSMessage gps_;
  asmc_msgs::VehicleState state_;
  ros::SteadyTime gps_received_,state_received_;
};
}  // namespace asmc_planner
