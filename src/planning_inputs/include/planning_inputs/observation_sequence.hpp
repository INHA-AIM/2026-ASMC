#pragma once

#include <asmc_msgs/LidarObservation.h>
#include <algorithm>
#include <deque>
#include <string>

namespace planning_inputs {
// One input producer, bounded restart tombstones. The caller also enforces
// source age; ancient packets cannot become fresh after a tombstone expires.
// This gate records invalid-status scans too, preventing an older valid scan
// from overriding a newer pose-invalid observation.
class ObservationSequence {
 public:
  bool Accept(const asmc_msgs::LidarObservation& input, std::string* reason) {
    const auto reject = [&](const char* value) { if (reason) *reason = value; return false; };
    if (input.producer_session_id.empty() || !input.generation || !input.scan_sequence ||
        input.config_id.empty() || input.calibration_id.empty() || input.objects.header.stamp.isZero())
      return reject("invalid_identity");
    const bool restart = session_ != input.producer_session_id;
    if (restart) {
      if (std::find(retired_.begin(), retired_.end(), input.producer_session_id) != retired_.end())
        return reject("retired_session");
    } else {
      if (input.generation < generation_) return reject("retired_generation");
      if (input.objects.header.stamp <= stamp_) return reject("duplicate_or_regressing_stamp");
      if (input.generation == generation_) {
        if (input.scan_sequence <= sequence_) return reject("duplicate_or_regressing_sequence");
        if (input.config_id != config_ || input.calibration_id != calibration_)
          return reject("identity_changed_without_reset");
      }
    }
    if (restart && !session_.empty()) {
      retired_.push_back(session_);
      while (retired_.size() > 64) retired_.pop_front();
    }
    session_ = input.producer_session_id; generation_ = input.generation;
    sequence_ = input.scan_sequence; stamp_ = input.objects.header.stamp;
    config_ = input.config_id; calibration_ = input.calibration_id;
    if (reason) reason->clear();
    return true;
  }
 private:
  std::string session_, config_, calibration_;
  std::uint64_t generation_ = 0, sequence_ = 0;
  ros::Time stamp_;
  std::deque<std::string> retired_;
};
}  // namespace planning_inputs
