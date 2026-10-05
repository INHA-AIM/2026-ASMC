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
    ros::NodeHandle& nh, const std::string& topic, double stale_seconds, double detection_grace_seconds)
    : RosTrafficLightProvider(nh, "", topic, stale_seconds, detection_grace_seconds) {}

RosTrafficLightProvider::RosTrafficLightProvider(
    ros::NodeHandle& nh, const std::string& typed_topic,
    const std::string& legacy_topic, double stale_seconds, double detection_grace_seconds)
    : stale_seconds_(stale_seconds > 0.0 ? stale_seconds : 1.0),
      detection_grace_seconds_(detection_grace_seconds > 0.0 ? detection_grace_seconds : 1.0) {
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
  typed_observation_.update(typed_valid_ && !typed_stale_ ? typed_label_ : "not_detected", typed_received_at_.toSec());
  ROS_INFO_ONCE("Behavior traffic-light consumer selected canonical typed input");
}

void RosTrafficLightProvider::legacyCallback(
    const std_msgs::String::ConstPtr& message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  legacy_label_ = message->data;
  legacy_received_at_ = ros::SteadyTime::now();
  legacy_received_ = true;
  legacy_observation_.update(legacy_label_, legacy_received_at_.toSec());
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

bool RosTrafficLightProvider::hasRecentDetection(const std::string&) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto sample = resolveLocked();
  // Grace retains visibility only; invalid/stale typed input never borrows a
  // legacy GREEN or renews a previous semantic state.
  if (sample.source == "none" || sample.label == "stale") return false;
  const auto& history = typed_received_ ? typed_observation_ : legacy_observation_;
  return history.hasRecentDetection(ros::SteadyTime::now().toSec(),
                                    stale_seconds_, detection_grace_seconds_);
}

std::string RosTrafficLightProvider::getRawLabel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return resolveLocked().label;
}

}  // namespace behavior_planner

