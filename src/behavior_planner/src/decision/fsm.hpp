#pragma once

#include <cstdint>

#include "behavior_planner/context/behavior_types.hpp"
#include "behavior_planner/decision/planner_feedback_data.hpp"

namespace behavior_planner {

class Fsm {
 public:
  struct ManeuverOutput {
    BehaviorType behavior = BehaviorType::KEEP;
    BehaviorState state = BehaviorState::KEEP_LANE;
    ManeuverPhase phase = ManeuverPhase::NONE;
  };

  BehaviorState update(BehaviorType);  // Legacy stateless mapping.
  ManeuverOutput updateManeuver(BehaviorType candidate,
                                const PlannerFeedbackData& feedback,
                                uint32_t active_command_id,
                                bool stop_preempts_committed);
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

  BehaviorType active_lateral_behavior_ = BehaviorType::KEEP;
  ManeuverPhase maneuver_phase_ = ManeuverPhase::NONE;
  uint32_t maneuver_command_id_ = 0;
};

}  // namespace behavior_planner
