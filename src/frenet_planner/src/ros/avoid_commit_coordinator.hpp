#pragma once

// AVOID core와 command manager의 COMMIT을 한 planning tick에서 함께 적용한다.
// 계약: docs/fsm-frenet-interface-v2.md 12.7.5절

#include <cmath>

#include "frenet_planner/frenet_planner.hpp"
#include "ros/command_execution_manager.hpp"

namespace frenet_planner_ros {

enum class AvoidPlanningRoute {
    PRE_COMMIT,
    POST_COMMIT,
    INVALID,
};

// AVOID planning의 경계는 execution_request가 아니라 manager state다.
// COMMITTED를 관측한 첫 planning tick에만 EXECUTING으로 올리고, 이후에는
// NotifyExecutionStarted()를 다시 호출하지 않는다.
inline AvoidPlanningRoute RouteAvoidPlanning(
    CommandExecutionManager& manager) {
    using Feedback = asmc_msgs::FrenetFeedback;
    switch (manager.execution_state()) {
        case Feedback::PREPARING:
        case Feedback::WAITING:
        case Feedback::READY:
            return AvoidPlanningRoute::PRE_COMMIT;
        case Feedback::COMMITTED:
            return manager.NotifyExecutionStarted()
                ? AvoidPlanningRoute::POST_COMMIT
                : AvoidPlanningRoute::INVALID;
        case Feedback::EXECUTING:
            return AvoidPlanningRoute::POST_COMMIT;
        default:
            return AvoidPlanningRoute::INVALID;
    }
}

inline bool ShouldResetAvoidContext(const CommandAcceptance& acceptance) {
    return !acceptance.should_plan() && acceptance.transaction_cancelled;
}

inline bool CompleteAvoidAtomically(
    CommandExecutionManager& manager,
    const FrenetPlanner::AvoidPrepareResult& result,
    bool trajectory_ready) {
    if (!trajectory_ready || !result.maneuver_completed ||
        !std::isfinite(result.maneuver_progress) ||
        result.maneuver_progress < 1.0) {
        return false;
    }
    return manager.NotifyManeuverCompleted();
}

struct FeedbackKinematics {
    double applied_speed_mps = -1.0;
    bool frenet_state_valid = false;
    double current_s = 0.0;
    double current_d = 0.0;
};

// trajectory source와 현재 Frenet state validity는 독립 축이다. 특히
// post-COMMIT AVOID는 새로 계산한 실제 궤적을 LOCKED로 발행하므로 속도와
// current s/d가 유효하다. terminal latch caller는 state_valid=false를 넘겨
// fresh state를 가장하지 않는다.
inline FeedbackKinematics ResolveFeedbackKinematics(
    uint8_t trajectory_source, bool has_avoid_fields,
    double avoid_applied_speed_mps, double normal_applied_speed_mps,
    bool caller_frenet_state_valid, double current_s, double current_d) {
    FeedbackKinematics result;
    if (has_avoid_fields &&
        (trajectory_source == asmc_msgs::FrenetFeedback::TRAJECTORY_NEW ||
         trajectory_source == asmc_msgs::FrenetFeedback::TRAJECTORY_LOCKED)) {
        result.applied_speed_mps = avoid_applied_speed_mps;
    } else if (trajectory_source ==
               asmc_msgs::FrenetFeedback::TRAJECTORY_NEW) {
        result.applied_speed_mps = normal_applied_speed_mps;
    }

    result.frenet_state_valid = caller_frenet_state_valid;
    if (caller_frenet_state_valid) {
        result.current_s = current_s;
        result.current_d = current_d;
    }
    return result;
}

inline bool CommitAvoidAtomically(
    CommandExecutionManager& manager, FrenetPlanner& planner,
    const FrenetPlanner::AvoidPrepareResult& result) {
    // manager를 바꾸기 전에 core의 모든 거부 조건을 read-only로 확인한다.
    if (!planner.CanCommitAvoid(result)) return false;
    if (!manager.NotifyCommitted()) return false;

    // 위 preflight 이후에는 같은 single-threaded tick에서 core 상태를 바꾸는
    // 호출이 없다. apply는 실패 반환이 없으므로 manager만 COMMITTED가 되는
    // 분기 자체가 없다.
    planner.ApplyAvoidCommit(result);
    return true;
}

}  // namespace frenet_planner_ros
