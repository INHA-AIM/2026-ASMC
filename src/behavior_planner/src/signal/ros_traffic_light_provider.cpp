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

ResolvedTrafficLightSample ResolveTrafficLightSample(
    const TrafficLightSample& typed, const TrafficLightSample& legacy) {
  if (typed.received) {
    if (!typed.fresh || typed.stale) {
      return {"stale", "typed", false};
    }
    return {typed.label, "typed", typed.valid};
  }
  if (legacy.received && legacy.fresh) {
    return {legacy.label, "legacy", true};
  }
  return {"stale", "none", false};
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

RosTrafficLightProvider::RosTrafficLightProvider(
    ros::NodeHandle& nh, const std::string& topic, double stale_seconds)
    : RosTrafficLightProvider(nh, "", topic, stale_seconds) {}

RosTrafficLightProvider::RosTrafficLightProvider(
    ros::NodeHandle& nh, const std::string& typed_topic,
    const std::string& legacy_topic, double stale_seconds)
    : stale_seconds_(stale_seconds > 0.0 ? stale_seconds : 1.0) {
  if (!typed_topic.empty()) {
    typed_subscriber_ = nh.subscribe(
        typed_topic, 1, &RosTrafficLightProvider::typedCallback, this);
  }
  if (!legacy_topic.empty()) {
    legacy_subscriber_ = nh.subscribe(
        legacy_topic, 1, &RosTrafficLightProvider::legacyCallback, this);
  }
}

void RosTrafficLightProvider::typedCallback(
    const asmc_msgs::TrafficLightDetection::ConstPtr& message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  typed_label_ = message->label;
  typed_valid_ = message->valid;
  typed_stale_ = message->stale;
  typed_received_at_ = ros::SteadyTime::now();
  typed_received_ = true;
  ROS_INFO_ONCE("Behavior traffic-light consumer selected canonical typed input");
}

void RosTrafficLightProvider::legacyCallback(
    const std_msgs::String::ConstPtr& message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  legacy_label_ = message->data;
  legacy_received_at_ = ros::SteadyTime::now();
  legacy_received_ = true;
}

ResolvedTrafficLightSample RosTrafficLightProvider::resolveLocked() const {
  const ros::SteadyTime now = ros::SteadyTime::now();
  TrafficLightSample typed;
  typed.received = typed_received_;
  typed.fresh = typed_received_ &&
                (now - typed_received_at_).toSec() <= stale_seconds_;
  typed.valid = typed_valid_;
  typed.stale = typed_stale_;
  typed.label = typed_label_;

  TrafficLightSample legacy;
  legacy.received = legacy_received_;
  legacy.fresh = legacy_received_ &&
                 (now - legacy_received_at_).toSec() <= stale_seconds_;
  legacy.valid = true;
  legacy.label = legacy_label_;
  return ResolveTrafficLightSample(typed, legacy);
}

TrafficLightState RosTrafficLightProvider::getState(
    const std::string&, SignalMovement movement) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const ResolvedTrafficLightSample sample = resolveLocked();
  if (!sample.actionable) {
    return TrafficLightState::UNKNOWN;
  }
  return InterpretTrafficLightLabel(sample.label, movement);
}

std::string RosTrafficLightProvider::getRawLabel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return resolveLocked().label;
}

}  // namespace behavior_planner
