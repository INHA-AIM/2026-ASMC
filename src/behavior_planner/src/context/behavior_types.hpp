#pragma once
#include <cstdint>
namespace behavior_planner {
enum class TrafficLightState : uint8_t { UNKNOWN=0, RED=1, YELLOW=2, GREEN=3 };
// Signal movement governing traversal of a selected MGeo route link. This is
// separate from adjacent-lane LEFT_CHANGE/RIGHT_CHANGE behavior.
enum class SignalMovement : uint8_t { UNKNOWN=0, STRAIGHT=1, LEFT=2, LEFT_UNPROTECTED=3, RIGHT_UNPROTECTED=4, UTURN=5 };
enum class ObjectType : uint8_t { UNKNOWN=0, VEHICLE=1, PEDESTRIAN=2, STATIC_OBSTACLE=3 };
enum class BehaviorType : uint8_t { KEEP=0, FOLLOW=1, LEFT_CHANGE=2, RIGHT_CHANGE=3, STOP=4, YIELD=5, EMERGENCY_STOP=6 };
enum class BehaviorState : uint8_t { KEEP_LANE=0, FOLLOW=1, LEFT_CHANGE_PREPARE=2, LEFT_CHANGE_EXECUTE=3, RIGHT_CHANGE_PREPARE=4, RIGHT_CHANGE_EXECUTE=5, STOP=6, EMERGENCY_STOP=7 };
}
