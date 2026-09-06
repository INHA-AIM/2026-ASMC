#include "behavior_planner/signal/ros_traffic_light_provider.hpp"

#include <algorithm>
#include <cctype>

namespace behavior_planner {
namespace {

std::string Normalize(std::string label) {
  label.erase(std::remove_if(label.begin(), label.end(),
                             [](unsigned char c) { return std::isspace(c); }),
              label.end());
  std::transform(label.begin(), label.end(), label.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return label;
}

}  // namespace

TrafficLightState InterpretTrafficLightLabel(const std::string& raw_label,
                                             SignalMovement movement) {
  const std::string label = Normalize(raw_label);
  if (label.empty() || label == "unknown" || label == "not_detected") {
    return TrafficLightState::UNKNOWN;
  }

  const bool has_left_arrow =
      label == "left" || label == "red_left" || label == "green_left";
  const bool has_green_circle = label == "green" || label == "green_left";

  if (movement == SignalMovement::LEFT) {
    return has_left_arrow ? TrafficLightState::GREEN :
           (label == "yellow" || label == "red_yellow")
               ? TrafficLightState::YELLOW
               : TrafficLightState::RED;
  }

  // An unprotected left follows the circular straight aspect. A right turn is
  // marked unprotected in MGeo and is therefore not granted by a left arrow.
  if (has_green_circle) return TrafficLightState::GREEN;
  if (label == "yellow" || label == "red_yellow") {
    return TrafficLightState::YELLOW;
  }
  if (label == "red" || label == "red_left" || label == "left") {
    return TrafficLightState::RED;
  }
  return TrafficLightState::UNKNOWN;
}

RosTrafficLightProvider::RosTrafficLightProvider(
    ros::NodeHandle& nh, const std::string& topic, double stale_seconds)
    : stale_seconds_(stale_seconds > 0.0 ? stale_seconds : 1.0) {
  subscriber_ = nh.subscribe(topic, 1, &RosTrafficLightProvider::callback, this);
}

void RosTrafficLightProvider::callback(
    const std_msgs::String::ConstPtr& message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  latest_label_ = message->data;
  received_at_ = ros::SteadyTime::now();
  received_ = true;
}

TrafficLightState RosTrafficLightProvider::getState(
    const std::string&, SignalMovement movement) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!received_ || (ros::SteadyTime::now() - received_at_).toSec() >
                        stale_seconds_) {
    return TrafficLightState::UNKNOWN;
  }
  return InterpretTrafficLightLabel(latest_label_, movement);
}

std::string RosTrafficLightProvider::getRawLabel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!received_ || (ros::SteadyTime::now() - received_at_).toSec() >
                        stale_seconds_) {
    return "stale";
  }
  return latest_label_;
}

}  // namespace behavior_planner
