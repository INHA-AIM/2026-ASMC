#pragma once

#include <cstdint>

// Shared internal values only. Runtime instances remain owned by their callers.
namespace asmc_planner {

enum class BehaviorType : std::uint8_t { KEEP=0, FOLLOW=1, LEFT_CHANGE=2, RIGHT_CHANGE=3, STOP=4, YIELD=5, EMERGENCY_STOP=6, AVOID=7, MERGE=8 };

// Existing ManeuverPhase vocabulary; not the ROS feedback execution_state.
enum class ExecutionPhase : std::uint8_t { NONE=0, PREPARE=1, EXECUTE=2, COMMITTED=3, COMPLETE=4, FAILED=5, CANCELLED=6 };

// Planner policy categories, not camera mission16 group IDs. PEDESTRIAN keeps
// the person-specific response; it is not the whole dynamic_obstacle group.
// vehicle/dynamic_obstacle can be parked or stationary; static_obstacle does
// not prove rest. MotionState and velocity validity remain independent.
// planning_inputs currently emits 0/1/2 only: 3 exists here but is not yet a
// lossless camera-group contract. See docs/handoffs/perception-to-planning.md.
enum class ObjectType : std::uint8_t { UNKNOWN=0, VEHICLE=1, PEDESTRIAN=2, STATIC_OBSTACLE=3 };

enum class MotionState : std::uint8_t {
  UNKNOWN=0,
  LOW_THREAT=1,
  DYNAMIC_PENDING=2,
  DYNAMIC=3
};

enum class TrafficLightState : std::uint8_t { UNKNOWN=0, RED=1, YELLOW=2, GREEN=3 };

enum class SignalMovement : std::uint8_t { UNKNOWN=0, STRAIGHT=1, LEFT=2, LEFT_UNPROTECTED=3, RIGHT_UNPROTECTED=4, UTURN=5 };

enum class LateralDirection {
    NONE = 0,
    LEFT,
    RIGHT,
};

enum class MergeType {
    NONE = 0,
    ROUNDABOUT,
    HIGHWAY,
};

struct Point2D { double x = 0.0; double y = 0.0; };

// Parser snapshot used by Behavior/Map. Frenet CartesianState is distinct.
struct EgoState { double x=0,y=0,yaw=0,speed=0,stamp=0; std::uint8_t source=0; bool valid=false; double acceleration=0; bool acceleration_valid=false; };

}  // namespace asmc_planner
