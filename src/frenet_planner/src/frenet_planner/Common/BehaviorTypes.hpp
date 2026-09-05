// 어떤 주행을 해야하는지, 어떤 방향으로 해야하는지에 대한 공용 타입.
// FSM(외부)과 Planner가 공유하는 maneuver 어휘다 - ROS 비의존.

#ifndef PLANNER_COMMON_BEHAVIOR_TYPES_HPP
#define PLANNER_COMMON_BEHAVIOR_TYPES_HPP

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

enum class MergeType {
    NONE = 0,
    ROUNDABOUT,
    HIGHWAY,
};

// FSM이 요청한 횡방향 기동 방향 (D6-1).
//
// BehaviorCommand.direction의 ROS 비의존 대응물이다. AVOID는 세 값을 모두
// 허용하고(NONE = Frenet이 방향을 고른다), LANE_CHANGE는 LEFT/RIGHT만
// 허용한다. LANE_CHANGE는 방향이 mode 자체로 갈라지므로
// (LANE_CHANGE_LEFT / LANE_CHANGE_RIGHT) 이 필드는 AVOID가 주 사용자다.
//
// AVOID에 별도 필드가 필요한 이유는, mode=AVOID 하나로는 LEFT/RIGHT/NONE을
// 구분할 수 없기 때문이다. 구분하지 않으면 FSM이 LEFT를 지정했는데 Frenet이
// 우측으로 회피하는 계약 위반이 조용히 일어난다.
enum class LateralDirection {
    NONE = 0,
    LEFT,
    RIGHT,
};

#endif  // PLANNER_COMMON_BEHAVIOR_TYPES_HPP
