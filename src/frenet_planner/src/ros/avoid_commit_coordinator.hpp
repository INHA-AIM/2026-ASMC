#pragma once

// AVOID core와 command manager의 COMMIT을 한 planning tick에서 함께 적용한다.
// 계약: docs/fsm-frenet-interface-v2.md 12.7.5절

#include "frenet_planner/frenet_planner.hpp"
#include "ros/command_execution_manager.hpp"

namespace frenet_planner_ros {

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
