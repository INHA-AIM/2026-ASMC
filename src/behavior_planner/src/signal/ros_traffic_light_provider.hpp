#pragma once

#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "behavior_planner/signal/traffic_light_provider.hpp"

namespace behavior_planner {

// Converts the camera classifier's compound label into the aspect governing
// the movement selected by the route (for example, green_left permits both a
// straight route and a protected-left route).
TrafficLightState InterpretTrafficLightLabel(const std::string& label,
                                             SignalMovement movement);

class RosTrafficLightProvider : public TrafficLightProvider {
 public:
  RosTrafficLightProvider(ros::NodeHandle& nh, const std::string& topic,
                          double stale_seconds);
  TrafficLightState getState(
      const std::string& traffic_light_id,
      SignalMovement movement = SignalMovement::UNKNOWN) const override;
  std::string getRawLabel() const override;

 private:
  void callback(const std_msgs::String::ConstPtr& message);

  ros::Subscriber subscriber_;
  mutable std::mutex mutex_;
  std::string latest_label_ = "not_detected";
  ros::SteadyTime received_at_;
  double stale_seconds_ = 1.0;
  bool received_ = false;
};

}  // namespace behavior_planner
