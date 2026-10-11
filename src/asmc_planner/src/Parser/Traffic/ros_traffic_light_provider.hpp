#pragma once

#include <mutex>
#include <string>

#include <ros/ros.h>
#include <asmc_msgs/TrafficLightDetection.h>
#include <std_msgs/String.h>

#include "Parser/Traffic/traffic_light_provider.hpp"

namespace behavior_planner {

// Converts the camera classifier's compound label into the aspect governing
// the movement selected by the route (for example, green_left permits both a
// straight route and a protected-left route).
TrafficLightState InterpretTrafficLightLabel(const std::string& label,
                                             SignalMovement movement);

// Pure selection contract used by the ROS adapter and its regression tests.
// Once a typed producer has been observed it remains authoritative: an
// invalid/stale typed result must fail safe instead of reviving a possibly
// inconsistent compatibility String sample.
struct TrafficLightSample {
  bool received = false;
  bool fresh = false;
  bool valid = false;
  bool stale = false;
  std::string label = "not_detected";
};

struct ResolvedTrafficLightSample {
  std::string label = "stale";
  std::string source = "none";
  bool actionable = false;
};

ResolvedTrafficLightSample ResolveTrafficLightSample(
    const TrafficLightSample& typed, const TrafficLightSample& legacy);

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
                          double stale_seconds, double detection_grace_seconds = 1.0);
  RosTrafficLightProvider(ros::NodeHandle& nh,
                          const std::string& typed_topic,
                          const std::string& legacy_topic,
                          double stale_seconds, double detection_grace_seconds = 1.0);
  TrafficLightState getState(
      const std::string& traffic_light_id,
      SignalMovement movement = SignalMovement::UNKNOWN) const override;
  bool hasRecentDetection(const std::string&) const override;
  std::string getRawLabel() const override;

 private:
  void typedCallback(
      const asmc_msgs::TrafficLightDetection::ConstPtr& message);
  void legacyCallback(const std_msgs::String::ConstPtr& message);
  ResolvedTrafficLightSample resolveLocked() const;

  ros::Subscriber typed_subscriber_;
  ros::Subscriber legacy_subscriber_;
  mutable std::mutex mutex_;
  std::string typed_label_ = "not_detected";
  std::string legacy_label_ = "not_detected";
  ros::SteadyTime typed_received_at_;
  ros::SteadyTime legacy_received_at_;
  double stale_seconds_ = 1.0;
  double detection_grace_seconds_ = 1.0;
  TrafficLightObservationState typed_observation_;
  TrafficLightObservationState legacy_observation_;
  bool typed_received_ = false;
  bool typed_valid_ = false;
  bool typed_stale_ = false;
  bool legacy_received_ = false;
};

}  // namespace behavior_planner
