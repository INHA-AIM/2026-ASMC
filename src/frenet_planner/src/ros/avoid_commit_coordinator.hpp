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

inline bool ShouldResetAvoidForEmergencyPreemption(
    const asmc_msgs::BehaviorCommand& command,
    const CommandAcceptance& acceptance) {
    return command.maneuver_type ==
               asmc_msgs::BehaviorCommand::EMERGENCY_STOP &&
           acceptance.planning_target == PlanningTarget::INCOMING_COMMAND;
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
