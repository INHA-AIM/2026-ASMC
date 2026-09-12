#pragma once

#include <cstdint>

#include "behavior_planner/context/behavior_types.hpp"

namespace behavior_planner {

class Fsm {
 public:
  BehaviorState update(BehaviorType);
  bool updateEmergency(bool requested, double min_ttc, double timestamp);
  void setEmergencyRelease(double exit_ttc, double hold_s);
  bool emergencyLatched() const { return emergency_latched_; }

  void setFollowReleaseDelay(double delay_s);
  int32_t updateFollowTarget(bool observed, int32_t target_id,
                             double timestamp);
  bool followReleaseGraceActive() const {
    return follow_release_grace_active_;
  }

 private:
  bool emergency_latched_ = false;
  double emergency_safe_since_ = -1.0;
  double emergency_last_timestamp_ = -1.0;
  double emergency_exit_ttc_ = 3.0;
  double emergency_release_hold_s_ = 0.5;

  int32_t last_follow_target_id_ = -1;
  double follow_last_seen_timestamp_ = -1.0;
  double follow_release_delay_s_ = 0.4;
  bool follow_release_grace_active_ = false;
};

}  // namespace behavior_planner
