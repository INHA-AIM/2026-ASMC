#pragma once

#include "Behavior/FSM/DecisionStability.hpp"
#include "Execution/ManeuverLifecycle.hpp"

namespace behavior_planner {

// Temporary facade preserving the original Fsm API and per-instance lifetime.
// Production orchestration still invokes this facade until later phases.
class Fsm {
 public:
  using ManeuverOutput = ManeuverLifecycle::Output;
  BehaviorState update(BehaviorType);
  ManeuverOutput updateManeuver(BehaviorType candidate,
                                const PlannerFeedbackData& feedback,
                                uint32_t active_command_id,
                                bool stop_preempts_committed,
                                bool cancel_reclassified_avoid = false,
                                bool follow_owns_avoid_target = false);
  bool updateEmergency(bool requested, double min_ttc, double timestamp) {
    return stability_.updateEmergency(requested, min_ttc, timestamp);
  }
  void setEmergencyRelease(double exit_ttc, double hold_s) {
    stability_.setEmergencyRelease(exit_ttc, hold_s);
  }
  bool emergencyLatched() const { return stability_.emergencyLatched(); }
  void setFollowReleaseDelay(double delay_s) {
    stability_.setFollowReleaseDelay(delay_s);
  }
  void setAvoidReleaseGraceCycles(int cycles) {
    stability_.setAvoidReleaseGraceCycles(cycles);
  }
  int32_t updateFollowTarget(bool observed, int32_t target_id, double timestamp) {
    return stability_.updateFollowTarget(observed, target_id, timestamp);
  }
  bool followReleaseGraceActive() const {
    return stability_.followReleaseGraceActive();
  }
  // Retire the preempted lateral lifecycle only. Independent emergency and
  // FOLLOW stability keep their own release rules.
  void preemptLateralForSignal() {
    lifecycle_=ManeuverLifecycle{};
    stability_.resetAvoidRelease();
  }

 private:
  DecisionStability stability_;
  ManeuverLifecycle lifecycle_;
};

}  // namespace behavior_planner
