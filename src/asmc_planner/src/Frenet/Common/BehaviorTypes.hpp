// 어떤 주행을 해야하는지, 어떤 방향으로 해야하는지에 대한 공용 타입.
// FSM(외부)과 Planner가 공유하는 maneuver 어휘다 - ROS 비의존.

#ifndef PLANNER_COMMON_BEHAVIOR_TYPES_HPP
#define PLANNER_COMMON_BEHAVIOR_TYPES_HPP

#include "Global/Global.hpp"

// 어떤 주행을 해야하는지 (ex: Lane_tracking, ACC 등)
enum BehaviorState {
    LANE_KEEPING,
    FOLLOWING,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
    INTERSECTION_WAIT,
    TURN_LEFT,
    TURN_RIGHT,
    AVOID,
    MERGE,
    STOP,
    EMERGENCY
};

// MERGE 타입 (ex: Roundabout, Highway 등)
using MergeType = asmc_planner::MergeType;

// 어떤 방향으로 주행을 해야하는지 (ex: 좌회전, 우회전 등)
using LateralDirection = asmc_planner::LateralDirection;

#endif  // PLANNER_COMMON_BEHAVIOR_TYPES_HPP
