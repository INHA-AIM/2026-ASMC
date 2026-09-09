#pragma once

// LANE_CHANGE core와 command manager의 COMMIT을 한 planning tick에서 함께
// 적용한다. avoid_commit_coordinator.hpp의 대응 함수를 그대로 재사용하는
// 대신 별도 파일로 둔 이유는 §12 - AVOID의 generic 대규모 refactor 없이
// LANE_CHANGE 전용 타입(LaneChangePrepareResult)에 맞춘 얇은 wrapper를
// 만들기 위해서다. AVOID regression risk를 만들지 않는다.
//
// 계약: docs/fsm-frenet-interface-v2.md 12.7.5절과 동일한 원자적 COMMIT 순서.

#include <cmath>

#include "frenet_planner/frenet_planner.hpp"
#include "command_execution_manager.hpp"

namespace frenet_planner_ros {

enum class LaneChangePlanningRoute {
    PRE_COMMIT,
    POST_COMMIT,
    INVALID,
};

// AVOID의 RouteAvoidPlanning과 동일한 계약 - LANE_CHANGE planning의 경계는
// execution_request가 아니라 manager state다. COMMITTED를 관측한 첫 planning
// tick에만 EXECUTING으로 올리고, 이후에는 NotifyExecutionStarted()를 다시
// 호출하지 않는다.
inline LaneChangePlanningRoute RouteLaneChangePlanning(
    CommandExecutionManager& manager) {
    using Feedback = asmc_msgs::FrenetFeedback;
    switch (manager.execution_state()) {
        case Feedback::PREPARING:
        case Feedback::WAITING:
        case Feedback::READY:
            return LaneChangePlanningRoute::PRE_COMMIT;
        case Feedback::COMMITTED:
            return manager.NotifyExecutionStarted()
                ? LaneChangePlanningRoute::POST_COMMIT
                : LaneChangePlanningRoute::INVALID;
        case Feedback::EXECUTING:
            return LaneChangePlanningRoute::POST_COMMIT;
        default:
            return LaneChangePlanningRoute::INVALID;
    }
}

inline bool ShouldResetLaneChangeContext(const CommandAcceptance& acceptance) {
    return !acceptance.should_plan() && acceptance.transaction_cancelled;
}

inline bool CompleteLaneChangeAtomically(
    CommandExecutionManager& manager,
    const FrenetPlanner::LaneChangePrepareResult& result,
    bool trajectory_ready) {
    if (!trajectory_ready || !result.maneuver_completed ||
        !std::isfinite(result.maneuver_progress) ||
        result.maneuver_progress < 1.0) {
        return false;
    }
    return manager.NotifyManeuverCompleted();
}

inline bool CommitLaneChangeAtomically(
    CommandExecutionManager& manager, FrenetPlanner& planner,
    const FrenetPlanner::LaneChangePrepareResult& result) {
    // manager를 바꾸기 전에 core의 모든 거부 조건을 read-only로 확인한다
    // (avoid_commit_coordinator.hpp의 CommitAvoidAtomically와 동일한 순서).
    if (!planner.CanCommitLaneChange(result)) return false;
    if (!manager.NotifyCommitted()) return false;

    // 위 preflight 이후에는 같은 single-threaded tick에서 core 상태를 바꾸는
    // 호출이 없다. apply는 실패 반환이 없으므로 manager만 COMMITTED가 되는
    // 분기 자체가 없다.
    planner.ApplyLaneChangeCommit(result);
    return true;
}

}  // namespace frenet_planner_ros
