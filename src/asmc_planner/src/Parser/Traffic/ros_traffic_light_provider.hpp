#pragma once

#include <mutex>
#include <memory>
#include <deque>
#include <string>
#include <functional>

#include <ros/ros.h>
#include <asmc_msgs/TrafficLightDetection.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CompressedImage.h>
#include "Parser/Traffic/OriginalCameraFrames.hpp"

#include "Parser/Traffic/traffic_light_provider.hpp"
#include "Signal/SignalPolicy.hpp"

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
                          double stale_seconds, double detection_grace_seconds = 1.0,
                          bool record_observations = false);
  TrafficLightState getState(
      const std::string& traffic_light_id,
      SignalMovement movement = SignalMovement::UNKNOWN) const override;
  bool hasRecentDetection(const std::string&) const override;
  std::string getRawLabel() const override;
  // Observer-only journal; draining it never changes the operational sample.
  std::vector<asmc_planner::signal::Observation> takeSignalObservations(bool& overflow);
  void requireOriginalFrames(ros::NodeHandle&,const std::string& camera_topic,double offset_s,double uncertainty_s);
  // Optional active-policy wakeup, invoked only after releasing the journal
  // lock. The caller must share this provider's single callback queue.
  void setStopObservationWakeup(std::function<void()> callback);

 private:
  void typedCallback(
      const asmc_msgs::TrafficLightDetection::ConstPtr& message);
  void observationCallback(const asmc_msgs::TrafficLightDetection::ConstPtr& message);
  void recordObservationLocked(const asmc_msgs::TrafficLightDetection::ConstPtr& message);
  void legacyCallback(const std_msgs::String::ConstPtr& message);
  void flushOriginalPendingLocked();
  void appendObservationLocked(asmc_planner::signal::Observation);
  ResolvedTrafficLightSample resolveLocked() const;

  ros::Subscriber typed_subscriber_;
  ros::Subscriber legacy_subscriber_;
  mutable std::mutex mutex_;
  ros::Subscriber observation_subscriber_;
  ros::Subscriber original_image_sub_,original_timing_sub_;
  std::unique_ptr<OriginalCameraFrames> original_frames_;
  std::uint64_t original_frames_epoch_=0;
  struct PendingOriginal {asmc_msgs::TrafficLightDetection::ConstPtr message;double received=0;};
  std::deque<PendingOriginal> original_pending_;
  std::deque<asmc_planner::signal::Observation> signal_observations_;
  bool signal_observation_overflow_ = false;
  std::function<void()> stop_observation_wakeup_;
  bool previous_stop_observation_=false;
  std::uint64_t observation_epoch_=1;
  std::uint32_t last_observation_sequence_=0;
  bool observation_sequence_seen_=false;
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
