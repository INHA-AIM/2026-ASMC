#include "behavior_planner/signal/ros_traffic_light_provider.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

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

bool IsDetectedTrafficLightLabel(const std::string& raw_label) {
  const std::string label = Normalize(raw_label);
  return label == "red" || label == "yellow" || label == "green" ||
         label == "red_yellow" || label == "left" ||
         label == "red_left" || label == "green_left";
}

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

void TrafficLightObservationState::update(const std::string& raw_label,
                                          double now_seconds) {
  latest_label_ = raw_label;
  received_at_seconds_ = now_seconds;
  received_ = true;
  if (IsDetectedTrafficLightLabel(raw_label)) {
    last_valid_detection_at_seconds_ = now_seconds;
    has_valid_detection_ = true;
  }
}

bool TrafficLightObservationState::sourceFresh(
    double now_seconds, double stale_seconds) const {
  if (!received_ || !std::isfinite(now_seconds) ||
      !std::isfinite(received_at_seconds_) ||
      !std::isfinite(stale_seconds) || stale_seconds <= 0.0) {
    return false;
  }
  const double age = now_seconds - received_at_seconds_;
  return age >= 0.0 && age <= stale_seconds;
}

TrafficLightState TrafficLightObservationState::getState(
    SignalMovement movement, double now_seconds, double stale_seconds) const {
  if (!sourceFresh(now_seconds, stale_seconds)) {
    return TrafficLightState::UNKNOWN;
  }
  return InterpretTrafficLightLabel(latest_label_, movement);
}

std::string TrafficLightObservationState::getRawLabel(
    double now_seconds, double stale_seconds) const {
  return sourceFresh(now_seconds, stale_seconds) ? latest_label_ : "stale";
}

bool TrafficLightObservationState::hasRecentDetection(
    double now_seconds, double stale_seconds,
    double detection_grace_seconds) const {
  if (!sourceFresh(now_seconds, stale_seconds) || !has_valid_detection_ ||
      !std::isfinite(last_valid_detection_at_seconds_) ||
      !std::isfinite(detection_grace_seconds) ||
      detection_grace_seconds <= 0.0) {
    return false;
  }
  const double detection_age =
      now_seconds - last_valid_detection_at_seconds_;
  return detection_age >= 0.0 && detection_age <= detection_grace_seconds;
}

RosTrafficLightProvider::RosTrafficLightProvider(
    ros::NodeHandle& nh, const std::string& topic, double stale_seconds,
    double detection_grace_seconds)
    : stale_seconds_(stale_seconds > 0.0 ? stale_seconds : 1.0),
      detection_grace_seconds_(detection_grace_seconds > 0.0
                                   ? detection_grace_seconds
                                   : 1.0) {
  subscriber_ = nh.subscribe(topic, 1, &RosTrafficLightProvider::callback, this);
}

void RosTrafficLightProvider::callback(
    const std_msgs::String::ConstPtr& message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  observation_.update(message->data, ros::SteadyTime::now().toSec());
}

TrafficLightState RosTrafficLightProvider::getState(
    const std::string&, SignalMovement movement) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return observation_.getState(movement, ros::SteadyTime::now().toSec(),
                               stale_seconds_);
}

bool RosTrafficLightProvider::hasRecentDetection(const std::string&) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return observation_.hasRecentDetection(
      ros::SteadyTime::now().toSec(), stale_seconds_,
      detection_grace_seconds_);
}

std::string RosTrafficLightProvider::getRawLabel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return observation_.getRawLabel(ros::SteadyTime::now().toSec(),
                                  stale_seconds_);
}

}  // namespace behavior_planner
