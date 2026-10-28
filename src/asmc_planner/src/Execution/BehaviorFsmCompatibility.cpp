#include "Execution/BehaviorFsmCompatibility.hpp"

namespace behavior_planner {

BehaviorState Fsm::update(BehaviorType b){if(b==BehaviorType::MERGE)return BehaviorState::MERGE_PREPARE;if(b==BehaviorType::FOLLOW)return BehaviorState::FOLLOW;if(b==BehaviorType::STOP)return BehaviorState::STOP;if(b==BehaviorType::EMERGENCY_STOP)return BehaviorState::EMERGENCY_STOP;if(b==BehaviorType::LEFT_CHANGE)return BehaviorState::LEFT_CHANGE_PREPARE;if(b==BehaviorType::RIGHT_CHANGE)return BehaviorState::RIGHT_CHANGE_PREPARE;if(b==BehaviorType::AVOID)return BehaviorState::AVOID_PREPARE;return BehaviorState::KEEP_LANE;}

Fsm::ManeuverOutput Fsm::updateManeuver(
    BehaviorType candidate, const PlannerFeedbackData& feedback,
    uint32_t active_command_id, bool stop_preempts_committed,
    bool cancel_reclassified_avoid, bool follow_owns_avoid_target,bool allow_merge_left_prepare) {
  return lifecycle_.update(candidate, feedback, active_command_id,
                           stop_preempts_committed, cancel_reclassified_avoid,
                           follow_owns_avoid_target, stability_,allow_merge_left_prepare);
}

}  // namespace behavior_planner
