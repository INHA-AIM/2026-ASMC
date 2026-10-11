#include "Execution/ManeuverLifecycle.hpp"
#include "Behavior/FSM/DecisionStability.hpp"

namespace behavior_planner {

namespace {

bool IsLateralManeuver(BehaviorType behavior) {
  return behavior == BehaviorType::MERGE || behavior == BehaviorType::AVOID ||
         behavior == BehaviorType::LEFT_CHANGE ||
         behavior == BehaviorType::RIGHT_CHANGE;
}

BehaviorState StateFor(BehaviorType behavior, ManeuverPhase phase) {
  if (behavior == BehaviorType::MERGE) {
    return phase == ManeuverPhase::PREPARE ? BehaviorState::MERGE_PREPARE : BehaviorState::MERGE_EXECUTE;
  }
  if (behavior == BehaviorType::AVOID) {
    return phase == ManeuverPhase::PREPARE
               ? BehaviorState::AVOID_PREPARE
               : BehaviorState::AVOID_EXECUTE;
  }
  if (behavior == BehaviorType::LEFT_CHANGE) {
    return phase == ManeuverPhase::PREPARE
               ? BehaviorState::LEFT_CHANGE_PREPARE
               : BehaviorState::LEFT_CHANGE_EXECUTE;
  }
  if (behavior == BehaviorType::RIGHT_CHANGE) {
    return phase == ManeuverPhase::PREPARE
               ? BehaviorState::RIGHT_CHANGE_PREPARE
               : BehaviorState::RIGHT_CHANGE_EXECUTE;
  }
  if (behavior == BehaviorType::FOLLOW) return BehaviorState::FOLLOW;
  if (behavior == BehaviorType::STOP) return BehaviorState::STOP;
  if (behavior == BehaviorType::EMERGENCY_STOP) {
    return BehaviorState::EMERGENCY_STOP;
  }
  return BehaviorState::KEEP_LANE;
}

}  // namespace

ManeuverLifecycle::Output ManeuverLifecycle::update(
    BehaviorType candidate, const PlannerFeedbackData& feedback,
    uint32_t active_command_id, bool stop_preempts_committed,
    bool cancel_reclassified_avoid, bool follow_owns_avoid_target,
    DecisionStability& stability) {
  // Emergency is the only unconditional maneuver preemption.
  if (candidate == BehaviorType::EMERGENCY_STOP) {
    active_lateral_behavior_ = BehaviorType::KEEP;
    maneuver_phase_ = ManeuverPhase::NONE;
    maneuver_command_id_ = 0;
    stability.resetAvoidRelease();
    return {candidate, StateFor(candidate, ManeuverPhase::NONE),
            ManeuverPhase::NONE};
  }

  // FOLLOW/AVOID temporal arbitration belongs here, not in the per-frame
  // scorer. If the current AVOID observation is the same physical object as
  // the retained FOLLOW target, a brief LOW_THREAT/UNKNOWN classification
  // must not turn longitudinal tracking into a lateral maneuver.
  if (candidate == BehaviorType::AVOID && follow_owns_avoid_target) {
    candidate = BehaviorType::FOLLOW;
  }

  // A static-looking LiDAR target may mature into a moving vehicle/unknown,
  // or a retained FOLLOW target may briefly look static after AVOID started.
  // Both are explicit FSM preemption reasons, including after COMMIT.
  if ((cancel_reclassified_avoid || follow_owns_avoid_target) &&
      active_lateral_behavior_ == BehaviorType::AVOID) {
    active_lateral_behavior_ = BehaviorType::KEEP;
    maneuver_phase_ = ManeuverPhase::NONE;
    maneuver_command_id_ = 0;
    stability.resetAvoidRelease();
  }

  if (IsLateralManeuver(active_lateral_behavior_)) {
    if (maneuver_command_id_ == 0 && active_command_id != 0) {
      maneuver_command_id_ = active_command_id;
    } else if (active_command_id != 0 &&
               active_command_id != maneuver_command_id_ &&
               (maneuver_phase_ == ManeuverPhase::PREPARE ||
                maneuver_phase_ == ManeuverPhase::EXECUTE)) {
      // The wire manager can replace a preparation transaction when its
      // speed policy changes or a failed transaction is restarted. Bind to
      // that published command before consuming feedback; an old READY must
      // not execute a new command. Committed maneuvers retain their binding.
      maneuver_command_id_ = active_command_id;
      maneuver_phase_ = ManeuverPhase::PREPARE;
      stability.resetAvoidRelease();
    }
    const bool feedback_matches =
        feedback.valid && maneuver_command_id_ != 0 &&
        feedback.command_id == maneuver_command_id_;
    if (feedback_matches) {
      switch (feedback.execution_state) {
        case PlannerExecutionState::READY:
          maneuver_phase_ = ManeuverPhase::EXECUTE;
          break;
        case PlannerExecutionState::COMMITTED:
        case PlannerExecutionState::EXECUTING:
          maneuver_phase_ = ManeuverPhase::COMMITTED;
          break;
        case PlannerExecutionState::COMPLETED:
          maneuver_phase_ = ManeuverPhase::COMPLETE;
          break;
        case PlannerExecutionState::FAILED:
          maneuver_phase_ = ManeuverPhase::FAILED;
          break;
        default:
          break;
      }
    }

    const bool terminal = maneuver_phase_ == ManeuverPhase::COMPLETE ||
                          maneuver_phase_ == ManeuverPhase::FAILED ||
                          maneuver_phase_ == ManeuverPhase::CANCELLED;

    // AVOID target가 한두 frame 사라졌다고 PREPARE/READY transaction을
    // KEEP으로 취소하지 않는다. COMMITTED는 아래 기존 lock 규칙이 이미
    // 보호한다. terminal feedback과 STOP/E-STOP/FOLLOW 같은 명시적
    // 다른 상태/위험은 유예하지 않는다.
    if (active_lateral_behavior_ == BehaviorType::AVOID &&
        maneuver_phase_ != ManeuverPhase::COMMITTED && !terminal) {
      candidate = stability.retainAvoidCandidate(candidate);
    }

    if (!terminal && active_lateral_behavior_ == BehaviorType::MERGE &&
        candidate != BehaviorType::EMERGENCY_STOP && candidate != BehaviorType::STOP) {
      candidate = BehaviorType::MERGE;
    }

    if (terminal) {
      active_lateral_behavior_ = BehaviorType::KEEP;
      maneuver_phase_ = ManeuverPhase::NONE;
      maneuver_command_id_ = 0;
      stability.resetAvoidRelease();
    } else {
      const bool planned_stop_preemption =
          candidate == BehaviorType::STOP && stop_preempts_committed &&
          active_lateral_behavior_ != BehaviorType::MERGE;
      if (planned_stop_preemption) {
        active_lateral_behavior_ = BehaviorType::KEEP;
        maneuver_phase_ = ManeuverPhase::NONE;
        maneuver_command_id_ = 0;
        stability.resetAvoidRelease();
        return {candidate, StateFor(candidate, ManeuverPhase::NONE),
                ManeuverPhase::NONE};
      }

      // Before commitment a changed candidate cancels the preparation. Once
      // committed, ordinary scorer changes cannot replace the maneuver.
      if (maneuver_phase_ == ManeuverPhase::COMMITTED ||
          candidate == active_lateral_behavior_) {
        return {active_lateral_behavior_,
                StateFor(active_lateral_behavior_, maneuver_phase_),
                maneuver_phase_};
      }
      active_lateral_behavior_ = BehaviorType::KEEP;
      maneuver_phase_ = ManeuverPhase::NONE;
      maneuver_command_id_ = 0;
      stability.resetAvoidRelease();
    }
  }

  if (IsLateralManeuver(candidate)) {
    active_lateral_behavior_ = candidate;
    maneuver_phase_ = ManeuverPhase::PREPARE;
    // The wire manager allocates the new command_id after this decision.
    // Bind it on the next cycle rather than capturing the previous command.
    maneuver_command_id_ = 0;
    stability.resetAvoidRelease();
    return {candidate, StateFor(candidate, maneuver_phase_),
            maneuver_phase_};
  }

  return {candidate, StateFor(candidate, ManeuverPhase::NONE),
          ManeuverPhase::NONE};
}

}  // namespace behavior_planner
