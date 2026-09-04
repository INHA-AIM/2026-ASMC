#include "ros/behavior_command_adapter.hpp"

namespace frenet_planner_ros {

namespace {

using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;

}  // namespace

bool ConvertManeuverType(uint8_t maneuver_type, uint8_t direction,
                         BehaviorState& out_mode) {
    // switch에 default를 두지 않고 정의된 값만 명시적으로 나열한다.
    // 새 maneuver가 추가되면 여기에 case를 추가해야 하고, 빠뜨리면 아래의
    // 최종 return false로 떨어져 INVALID_COMMAND가 된다 - 조용히 잘못된
    // 모드로 실행되는 것보다 안전한 실패 방향이다.
    switch (maneuver_type) {
        case Command::KEEP:
            out_mode = LANE_KEEPING;
            return true;

        case Command::FOLLOW:
            out_mode = FOLLOWING;
            return true;

        case Command::MERGE:
            out_mode = MERGE;
            return true;

        case Command::LANE_CHANGE:
            // 유일한 1:2 매핑. 방향이 없으면 모드를 정할 수 없다.
            if (direction == Command::DIRECTION_LEFT) {
                out_mode = LANE_CHANGE_LEFT;
                return true;
            }
            if (direction == Command::DIRECTION_RIGHT) {
                out_mode = LANE_CHANGE_RIGHT;
                return true;
            }
            return false;

        case Command::AVOID:
            out_mode = AVOID;
            return true;

        case Command::STOP:
            out_mode = STOP;
            return true;

        case Command::EMERGENCY_STOP:
            out_mode = EMERGENCY;
            return true;

        default:
            break;
    }
    return false;
}

uint8_t ConvertToFeedbackManeuver(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING:
            return Command::KEEP;

        case FOLLOWING:
            return Command::FOLLOW;

        case MERGE:
            return Command::MERGE;

        // 2:1 축약. 좌/우는 FSM이 보낸 direction으로 알 수 있다.
        case LANE_CHANGE_LEFT:
        case LANE_CHANGE_RIGHT:
            return Command::LANE_CHANGE;

        case AVOID:
            return Command::AVOID;

        case STOP:
            return Command::STOP;

        case EMERGENCY:
            return Command::EMERGENCY_STOP;

        // legacy 전용 모드. v2 계약에 대응 값이 없다.
        case INTERSECTION_WAIT:
        case TURN_LEFT:
        case TURN_RIGHT:
            return Feedback::MANEUVER_NONE;
    }
    return Feedback::MANEUVER_NONE;
}

}  // namespace frenet_planner_ros
