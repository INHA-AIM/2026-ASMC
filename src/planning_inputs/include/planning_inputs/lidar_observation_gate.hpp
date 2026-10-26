#pragma once
#include <asmc_msgs/LidarPlanningObservation.h>
#include <planning_inputs/converter.hpp>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace planning_inputs {
// No fallback to Main, JSON diagnostics, stale arrays, or measured-zero speed.
// Unknown speed keeps the observed geometry with an explicit private validity
// bit and UNKNOWN occupancy policy. Missing/invalid map input still belongs to
// the existing planner freshness/STOP policy.
class LidarObservationGate {
 public:
  bool Convert(const asmc_msgs::LidarPlanningObservation& packet, double now,
               DetectedObjectArray* output, std::string* reason) {
    const auto reject = [&](const char* message) { *reason = message; return false; };
    const auto& o = packet.observation;
    const auto& a = o.objects;
    const auto stamp = a.header.stamp.toNSec();
    const double age = now - a.header.stamp.toSec();
    if (o.producer_session_id.empty() || o.config_id.empty() || o.calibration_id.empty())
      return reject("missing_observation_identity");
    if (!stamp || !std::isfinite(now) || age < -.05 || age > .30)
      return reject("observation_timestamp_invalid");
    if (stamp <= last_stamp_) return reject("duplicate_or_regressing_stamp");
    if (session_ != o.producer_session_id) {
      if (retired_.count(o.producer_session_id)) return reject("retired_session");
      if (retired_.size() >= 128) return reject("session_limit_restart_adapter");
      if (!session_.empty()) retired_.insert(session_);
      session_ = o.producer_session_id;
      generation_ = o.generation; sequence_ = 0; have_sequence_ = false; ids_.clear();
      config_ = o.config_id; calibration_ = o.calibration_id;
    } else if (o.generation < generation_) return reject("retired_generation");
    else if (o.generation > generation_) {
      generation_ = o.generation; sequence_ = 0; have_sequence_ = false; ids_.clear();
      config_ = o.config_id; calibration_ = o.calibration_id;
    }
    if (config_ != o.config_id || calibration_ != o.calibration_id)
      return reject("identity_changed_without_reset");
    if (have_sequence_ && o.scan_sequence <= sequence_) return reject("duplicate_or_regressing_scan");
    sequence_ = o.scan_sequence; have_sequence_ = true; last_stamp_ = stamp;
    if (a.header.frame_id != "map") return reject("map_frame_required");
    if (a.status != a.OBSERVED && a.status != a.OBSERVED_EMPTY) return reject("map_observation_unavailable");
    if ((a.status == a.OBSERVED_EMPTY) != a.objects.empty()) return reject("inconsistent_empty_status");
    if (a.objects.size() != packet.motion_states.size()) return reject("motion_count_mismatch");
    for (std::size_t i = 0; i < a.objects.size(); ++i) {
      const auto& object = a.objects[i];
      if (object.observation_state != object.OBSERVED || object.miss_count != 0)
        return reject("unobserved_object");
      if (packet.motion_states[i] > packet.DYNAMIC) return reject("invalid_motion_state");
      if (!object.absolute_velocity_valid && (packet.motion_states[i] != packet.UNKNOWN || object.dynamic_valid))
        return reject("unmeasured_velocity_requires_unknown");
      if ((packet.motion_states[i] == packet.LOW_THREAT || packet.motion_states[i] == packet.DYNAMIC) &&
          (!object.dynamic_valid || object.is_dynamic != (packet.motion_states[i] == packet.DYNAMIC)))
        return reject("inconsistent_motion_validity");
    }
    DetectedObjectArray converted;
    if (ConvertPlanningGeometry(a, &converted, nullptr, reason) != ConversionStatus::OK) return false;
    // Globally unique private IDs prevent planner caches reusing a retired
    // producer's track after reset/restart. Raw IDs remain in the public packet.
    for (std::size_t i = 0; i < converted.objects.size(); ++i) {
      auto& object = converted.objects[i];
      auto found = ids_.find(object.id);
      if (found == ids_.end()) {
        if (ids_.size() >= 100000) return reject("track_limit_restart_stack");
        if (next_id_ == std::numeric_limits<int32_t>::max()) return reject("id_limit_restart_stack");
        found = ids_.emplace(object.id, next_id_++).first;
      }
      object.id = found->second;
      object.motion_state = packet.motion_states[i];
      object.velocity_unavailable = !a.objects[i].absolute_velocity_valid;
      // Existing private planning contract: only measured LOW_THREAT is held
      // stationary for prediction. UNKNOWN/PENDING/DYNAMIC keep their velocity.
      if (object.motion_state == packet.LOW_THREAT) object.vx = object.vy = 0.;
    }
    *output = std::move(converted); *reason = "accepted"; return true;
  }
 private:
  std::string session_, config_, calibration_;
  std::set<std::string> retired_;
  std::map<int32_t, int32_t> ids_;
  int32_t next_id_ = 0;
  uint64_t generation_ = 0, sequence_ = 0, last_stamp_ = 0;
  bool have_sequence_ = false;
};
}  // namespace planning_inputs
