#include "planner_baseline_fsm.hpp"

#include <algorithm>
#include <cmath>

namespace behavior_planner {

BehaviorState BaselineFsm::update(BehaviorType b){if(b==BehaviorType::MERGE)return BehaviorState::MERGE_PREPARE;if(b==BehaviorType::FOLLOW)return BehaviorState::FOLLOW;if(b==BehaviorType::STOP)return BehaviorState::STOP;if(b==BehaviorType::EMERGENCY_STOP)return BehaviorState::EMERGENCY_STOP;if(b==BehaviorType::LEFT_CHANGE)return BehaviorState::LEFT_CHANGE_PREPARE;if(b==BehaviorType::RIGHT_CHANGE)return BehaviorState::RIGHT_CHANGE_PREPARE;if(b==BehaviorType::AVOID)return BehaviorState::AVOID_PREPARE;return BehaviorState::KEEP_LANE;}

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

BaselineFsm::ManeuverOutput BaselineFsm::updateManeuver(
    BehaviorType candidate, const PlannerFeedbackData& feedback,
    uint32_t active_command_id, bool stop_preempts_committed,
    bool cancel_reclassified_avoid, bool follow_owns_avoid_target) {
  // Emergency is the only unconditional maneuver preemption.
  if (candidate == BehaviorType::EMERGENCY_STOP) {
    active_lateral_behavior_ = BehaviorType::KEEP;
    maneuver_phase_ = ManeuverPhase::NONE;
    maneuver_command_id_ = 0;
    avoid_missing_cycles_ = 0;
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
    avoid_missing_cycles_ = 0;
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
      avoid_missing_cycles_ = 0;
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
      if (candidate == BehaviorType::AVOID) {
        avoid_missing_cycles_ = 0;
      } else if (candidate == BehaviorType::KEEP &&
                 avoid_missing_cycles_ < avoid_release_grace_cycles_) {
        ++avoid_missing_cycles_;
        candidate = BehaviorType::AVOID;
      } else {
        avoid_missing_cycles_ = 0;
      }
    }

    if (!terminal && active_lateral_behavior_ == BehaviorType::MERGE &&
        candidate != BehaviorType::EMERGENCY_STOP && candidate != BehaviorType::STOP) {
      candidate = BehaviorType::MERGE;
    }

    if (terminal) {
      active_lateral_behavior_ = BehaviorType::KEEP;
      maneuver_phase_ = ManeuverPhase::NONE;
      maneuver_command_id_ = 0;
      avoid_missing_cycles_ = 0;
    } else {
      const bool planned_stop_preemption =
          candidate == BehaviorType::STOP && stop_preempts_committed &&
          active_lateral_behavior_ != BehaviorType::MERGE;
      if (planned_stop_preemption) {
        active_lateral_behavior_ = BehaviorType::KEEP;
        maneuver_phase_ = ManeuverPhase::NONE;
        maneuver_command_id_ = 0;
        avoid_missing_cycles_ = 0;
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
      avoid_missing_cycles_ = 0;
    }
  }

  if (IsLateralManeuver(candidate)) {
    active_lateral_behavior_ = candidate;
    maneuver_phase_ = ManeuverPhase::PREPARE;
    // The wire manager allocates the new command_id after this decision.
    // Bind it on the next cycle rather than capturing the previous command.
    maneuver_command_id_ = 0;
    avoid_missing_cycles_ = 0;
    return {candidate, StateFor(candidate, maneuver_phase_),
            maneuver_phase_};
  }

  return {candidate, StateFor(candidate, ManeuverPhase::NONE),
          ManeuverPhase::NONE};
}

void BaselineFsm::setFollowReleaseDelay(double delay_s) {
  follow_release_delay_s_ = std::max(0.0, delay_s);
}

int32_t BaselineFsm::updateFollowTarget(bool observed, int32_t target_id,
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

void BaselineFsm::setEmergencyRelease(double exit_ttc, double hold_s) {
  emergency_exit_ttc_ = std::max(0.1, exit_ttc);
  emergency_release_hold_s_ = std::max(0.0, hold_s);
}

bool BaselineFsm::updateEmergency(bool requested, double min_ttc, double timestamp) {
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
