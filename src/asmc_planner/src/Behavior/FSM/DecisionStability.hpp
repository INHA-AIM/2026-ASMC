#pragma once

#include "Behavior/FSM/BehaviorTypes.hpp"

namespace behavior_planner {

// Decision hysteresis and short observation-drop grace only.
class DecisionStability {
 public:
  bool updateEmergency(bool requested, double min_ttc, double timestamp);
  void setEmergencyRelease(double exit_ttc, double hold_s);
  bool emergencyLatched() const { return emergency_latched_; }
  void setFollowReleaseDelay(double delay_s);
  int32_t updateFollowTarget(bool observed, int32_t target_id, double timestamp);
  bool followReleaseGraceActive() const { return follow_release_grace_active_; }
  void setAvoidReleaseGraceCycles(int cycles) {
    avoid_release_grace_cycles_ = cycles > 0 ? cycles : 0;
  }
  void resetAvoidRelease() { avoid_missing_cycles_ = 0; }
  BehaviorType retainAvoidCandidate(BehaviorType candidate);

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
  int avoid_release_grace_cycles_ = 3;
  int avoid_missing_cycles_ = 0;
};

}  // namespace behavior_planner
