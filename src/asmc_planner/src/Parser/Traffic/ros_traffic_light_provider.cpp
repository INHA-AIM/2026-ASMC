#include "Parser/Traffic/ros_traffic_light_provider.hpp"

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
    const std::string& legacy_topic, double stale_seconds, double detection_grace_seconds,
    bool record_observations)
    : stale_seconds_(stale_seconds > 0.0 ? stale_seconds : 1.0),
      detection_grace_seconds_(detection_grace_seconds > 0.0 ? detection_grace_seconds : 1.0) {
  if (!typed_topic.empty()) {
    typed_subscriber_ = nh.subscribe(
        typed_topic, 1, &RosTrafficLightProvider::typedCallback, this);
    if(record_observations) observation_subscriber_=nh.subscribe(
        typed_topic,64,&RosTrafficLightProvider::observationCallback,this);
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

std::vector<asmc_planner::signal::Observation>
RosTrafficLightProvider::takeSignalObservations(bool& overflow) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(original_frames_) {
    flushOriginalPendingLocked();
    if(!original_pending_.empty()) {
      asmc_planner::signal::Observation unknown;
      unknown.received=ros::SteadyTime::now().toSec();unknown.transport_epoch=observation_epoch_;
      appendObservationLocked(std::move(unknown));
    }
  }
  std::vector<asmc_planner::signal::Observation> observations(
      signal_observations_.begin(),signal_observations_.end());
  signal_observations_.clear();
  overflow=signal_observation_overflow_;signal_observation_overflow_=false;
  return observations;
}

void RosTrafficLightProvider::observationCallback(
    const asmc_msgs::TrafficLightDetection::ConstPtr& message) {
  if(!message)return;
  std::function<void()> wakeup;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool stop=!message->valid || message->stale ||
        (message->label!="green" && message->label!="green_left" && message->label!="left");
    if(stop && !previous_stop_observation_)wakeup=stop_observation_wakeup_;
    previous_stop_observation_=stop;
    recordObservationLocked(message);
  }
  // Never hold mutex_ across policy evaluation: it drains this same journal.
  // Repeated red frames do not run the full Behavior/FSM at camera rate.
  if(wakeup)wakeup();
}

void RosTrafficLightProvider::setStopObservationWakeup(std::function<void()> callback) {
  std::lock_guard<std::mutex> lock(mutex_);stop_observation_wakeup_=std::move(callback);
}

void RosTrafficLightProvider::recordObservationLocked(
    const asmc_msgs::TrafficLightDetection::ConstPtr& message) {
  asmc_planner::signal::Observation observation;
  observation.stamp=message->header.stamp.toSec();
  observation.original_stamp_ns=message->header.stamp.toNSec();
  observation.sequence=message->header.seq;observation.frame=message->header.frame_id;
  observation.received=ros::SteadyTime::now().toSec();
  observation.label=message->label;observation.valid=message->valid;observation.stale=message->stale;
  if(original_frames_) {
    if(!message->valid || message->stale) {
      original_frames_->Invalidate(message->header);
      OriginalCameraFrames::Original source;
      original_frames_->Resolve(message->header,source);
      observation.stamp=source.source_ns?source.ros_stamp:0;
      observation.original_stamp_ns=source.source_ns;
      observation.valid=false;observation.stale=true;
      appendObservationLocked(std::move(observation));
      return;
    }
    if(original_pending_.size()>=128) {
      original_pending_.pop_front();signal_observation_overflow_=true;
    }
    original_pending_.push_back({message,observation.received});
    flushOriginalPendingLocked();
    return;
  }
  appendObservationLocked(std::move(observation));
}

void RosTrafficLightProvider::appendObservationLocked(asmc_planner::signal::Observation observation) {
  // Wire sequence is only a conservative transport-discontinuity detector;
  // advancing seq with the same original stamp never counts as a new image.
  if(!observation.frame.empty()) {
    if(observation_sequence_seen_ && observation.sequence<last_observation_sequence_)++observation_epoch_;
    observation_sequence_seen_=true;last_observation_sequence_=observation.sequence;
  }
  observation.transport_epoch=observation_epoch_;
  if(signal_observations_.size()>=128) {
    signal_observations_.pop_front();signal_observation_overflow_=true;
  }
  signal_observations_.push_back(std::move(observation));
}

void RosTrafficLightProvider::flushOriginalPendingLocked() {
  if(!original_frames_)return;
  if(original_frames_epoch_!=original_frames_->epoch()) {
    ++observation_epoch_;original_frames_epoch_=original_frames_->epoch();
    asmc_planner::signal::Observation invalid;invalid.received=ros::SteadyTime::now().toSec();
    invalid.transport_epoch=observation_epoch_;appendObservationLocked(std::move(invalid));
  }
  while(!original_pending_.empty()) {
    const auto& pending=original_pending_.front();const auto& m=*pending.message;
    OriginalCameraFrames::Original source;
    const auto match=original_frames_->Resolve(m.header,source);
    if(match==OriginalCameraFrames::Match::MISSING &&
       ros::SteadyTime::now().toSec()-pending.received<=.3)break;
    asmc_planner::signal::Observation observation;
    observation.frame=m.header.frame_id;observation.sequence=m.header.seq;
    observation.stamp=source.source_ns?source.ros_stamp:0;observation.original_stamp_ns=source.source_ns;
    observation.received=pending.received;observation.label=m.label;
    observation.valid=m.valid && !m.stale && match==OriginalCameraFrames::Match::VALID;
    observation.stale=m.stale || !observation.valid;
    appendObservationLocked(std::move(observation));original_pending_.pop_front();
  }
}

void RosTrafficLightProvider::requireOriginalFrames(ros::NodeHandle& nh,const std::string& topic,double offset,double uncertainty) {
  original_frames_=std::make_unique<OriginalCameraFrames>(topic,offset,uncertainty);
  original_frames_epoch_=original_frames_->epoch();
  original_image_sub_=nh.subscribe<sensor_msgs::CompressedImage>(topic,64,
      [this](const sensor_msgs::CompressedImage::ConstPtr& m) {
        if(!m)return;std::lock_guard<std::mutex> lock(mutex_);
        if(original_frames_->Image(m->header)) {
          asmc_planner::signal::Observation invalid;invalid.received=ros::SteadyTime::now().toSec();
          appendObservationLocked(std::move(invalid));
        }
        flushOriginalPendingLocked();
      });
  original_timing_sub_=nh.subscribe<asmc_msgs::SensorFrameTiming>("/morai_sensor_bridge/frame_timing",128,
      [this](const asmc_msgs::SensorFrameTiming::ConstPtr& m) {
        if(!m)return;std::lock_guard<std::mutex> lock(mutex_);
        if(original_frames_->Timing(*m)) {
          asmc_planner::signal::Observation invalid;invalid.received=ros::SteadyTime::now().toSec();
          invalid.transport_epoch=observation_epoch_;appendObservationLocked(std::move(invalid));
        }
        flushOriginalPendingLocked();
      });
}

}  // namespace behavior_planner

