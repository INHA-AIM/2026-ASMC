#pragma once
#include <cstdint>
#include "Global/Global.hpp"
namespace behavior_planner {
using TrafficLightState = asmc_planner::TrafficLightState;
// Signal movement governing traversal of a selected MGeo route link. This is
// separate from adjacent-lane LEFT_CHANGE/RIGHT_CHANGE behavior.
using SignalMovement = asmc_planner::SignalMovement;
using ObjectType = asmc_planner::ObjectType;
// Perception이 확정한 시간적 운동 상태. ObjectType(semantic class)과
// 독립적이며 Behavior/Frenet이 각자 다시 동정적을 추정하지 않는다.
using MotionState = asmc_planner::MotionState;
using BehaviorType = asmc_planner::BehaviorType;
enum class BehaviorState : uint8_t { KEEP_LANE=0, FOLLOW=1, LEFT_CHANGE_PREPARE=2, LEFT_CHANGE_EXECUTE=3, RIGHT_CHANGE_PREPARE=4, RIGHT_CHANGE_EXECUTE=5, STOP=6, EMERGENCY_STOP=7, AVOID_PREPARE=8, AVOID_EXECUTE=9, MERGE_PREPARE=10, MERGE_EXECUTE=11 };
using ManeuverPhase = asmc_planner::ExecutionPhase;
}
