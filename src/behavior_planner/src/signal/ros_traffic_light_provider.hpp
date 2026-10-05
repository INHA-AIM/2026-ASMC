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
bool IsDetectedTrafficLightLabel(const std::string& label);

// Stores the latest classifier observation independently of ROS time. Tests
// inject deterministic timestamps; the ROS provider supplies SteadyTime.
class TrafficLightObservationState {
 public:
  void update(const std::string& raw_label, double now_seconds);
  TrafficLightState getState(SignalMovement movement, double now_seconds,
                             double stale_seconds) const;
  std::string getRawLabel(double now_seconds, double stale_seconds) const;
  bool hasRecentDetection(double now_seconds, double stale_seconds,
                          double detection_grace_seconds) const;

 private:
  bool sourceFresh(double now_seconds, double stale_seconds) const;

  std::string latest_label_ = "not_detected";
  double received_at_seconds_ = 0.0;
  double last_valid_detection_at_seconds_ = 0.0;
  bool received_ = false;
  bool has_valid_detection_ = false;
};

class RosTrafficLightProvider : public TrafficLightProvider {
 public:
  RosTrafficLightProvider(ros::NodeHandle& nh, const std::string& topic,
                          double stale_seconds,
                          double detection_grace_seconds);
  TrafficLightState getState(
      const std::string& traffic_light_id,
      SignalMovement movement = SignalMovement::UNKNOWN) const override;
  bool hasRecentDetection(
      const std::string& traffic_light_id) const override;
  std::string getRawLabel() const override;

 private:
  void callback(const std_msgs::String::ConstPtr& message);

  ros::Subscriber subscriber_;
  mutable std::mutex mutex_;
  TrafficLightObservationState observation_;
  double stale_seconds_ = 1.0;
  double detection_grace_seconds_ = 1.0;
};

}  // namespace behavior_planner
