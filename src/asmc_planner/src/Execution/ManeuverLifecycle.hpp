#pragma once

#include "Behavior/FSM/BehaviorTypes.hpp"
#include "Execution/PlannerFeedbackData.hpp"

namespace behavior_planner {

class DecisionStability;

// The existing Behavior-side lifecycle, isolated for Phase 5 consolidation.
// The wire command manager and Frenet execution manager remain separate.
class ManeuverLifecycle {
 public:
  struct Output {
    BehaviorType behavior = BehaviorType::KEEP;
    BehaviorState state = BehaviorState::KEEP_LANE;
    ManeuverPhase phase = ManeuverPhase::NONE;
  };
  Output update(BehaviorType candidate, const PlannerFeedbackData& feedback,
                uint32_t active_command_id, bool stop_preempts_committed,
                bool cancel_reclassified_avoid, bool follow_owns_avoid_target,
                DecisionStability& stability);

 private:
  BehaviorType active_lateral_behavior_ = BehaviorType::KEEP;
  ManeuverPhase maneuver_phase_ = ManeuverPhase::NONE;
  uint32_t maneuver_command_id_ = 0;
};

}  // namespace behavior_planner
