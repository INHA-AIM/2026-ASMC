#include "behavior_planner/decision/fsm.hpp"

#include <algorithm>
#include <cmath>

namespace behavior_planner {

BehaviorState Fsm::update(BehaviorType b){if(b==BehaviorType::FOLLOW)return BehaviorState::FOLLOW;if(b==BehaviorType::STOP)return BehaviorState::STOP;if(b==BehaviorType::EMERGENCY_STOP)return BehaviorState::EMERGENCY_STOP;if(b==BehaviorType::LEFT_CHANGE)return BehaviorState::LEFT_CHANGE_PREPARE;if(b==BehaviorType::RIGHT_CHANGE)return BehaviorState::RIGHT_CHANGE_PREPARE;if(b==BehaviorType::AVOID)return BehaviorState::AVOID_PREPARE;return BehaviorState::KEEP_LANE;}

void Fsm::setFollowReleaseDelay(double delay_s) {
  follow_release_delay_s_ = std::max(0.0, delay_s);
}

int32_t Fsm::updateFollowTarget(bool observed, int32_t target_id,
                                double timestamp) {
  const bool valid_time = std::isfinite(timestamp);
  if (observed && target_id >= 0 && valid_time) {
    last_follow_target_id_ = target_id;
    follow_last_seen_timestamp_ = timestamp;
    follow_release_grace_active_ = false;
    return target_id;
  }

  follow_release_grace_active_ =
      last_follow_target_id_ >= 0 && valid_time &&
      follow_last_seen_timestamp_ >= 0.0 &&
      timestamp >= follow_last_seen_timestamp_ &&
      timestamp - follow_last_seen_timestamp_ <= follow_release_delay_s_;
  if (follow_release_grace_active_) return last_follow_target_id_;

  last_follow_target_id_ = -1;
  follow_last_seen_timestamp_ = -1.0;
  return -1;
}

void Fsm::setEmergencyRelease(double exit_ttc, double hold_s) {
  emergency_exit_ttc_ = std::max(0.1, exit_ttc);
  emergency_release_hold_s_ = std::max(0.0, hold_s);
}

bool Fsm::updateEmergency(bool requested, double min_ttc, double timestamp) {
  if (requested) {
    emergency_latched_ = true;
    emergency_safe_since_ = -1.0;
    emergency_last_timestamp_ = timestamp;
    return true;
  }
  if (!emergency_latched_) return false;

  const bool monotonic_time = std::isfinite(timestamp) &&
      (emergency_last_timestamp_ < 0.0 || timestamp >= emergency_last_timestamp_);
  emergency_last_timestamp_ = timestamp;
  const bool safe = monotonic_time && std::isfinite(min_ttc) &&
                    min_ttc >= emergency_exit_ttc_;
  if (!safe) {
    emergency_safe_since_ = -1.0;
    return true;
  }
  if (emergency_safe_since_ < 0.0) emergency_safe_since_ = timestamp;
  if (timestamp - emergency_safe_since_ < emergency_release_hold_s_) return true;

  emergency_latched_ = false;
  emergency_safe_since_ = -1.0;
  return false;
}

}  // namespace behavior_planner
