// D3-1: BehaviorCommand <-> 내부 BehaviorState enum 매핑 회귀.
//
// 이 테스트의 목적은 두 가지다.
//   1. 7개 maneuver의 정/역방향 매핑이 의도한 값인지
//   2. static_cast가 실제로 틀린다는 것을 코드로 고정
//
// (2)가 특히 중요하다. 두 enum은 번호 체계가 달라 직접 cast하면 MERGE가
// 차선변경으로, AVOID가 교차로 대기로, STOP이 좌회전으로 실행된다. 나중에
// 누가 "매핑 함수가 번거롭다"며 cast로 되돌리면 이 테스트가 막는다.
//
// 계약: docs/fsm-frenet-interface-v2.md 2절

#include "behavior_command_adapter.hpp"

#include <iostream>
#include <string>

namespace {

using frenet_planner_ros::ConvertManeuverType;
using frenet_planner_ros::ConvertToFeedbackManeuver;
using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

// 정방향 매핑이 성공하고 기대한 내부 모드가 나오는지.
void ExpectForward(uint8_t maneuver, uint8_t direction,
                   BehaviorState expected, const std::string& name) {
    BehaviorState mode = LANE_KEEPING;
    // 기대값이 LANE_KEEPING인 경우에도 "함수가 실제로 썼는지"를 보려면
    // 초기값을 다른 값으로 두어야 한다.
    mode = EMERGENCY;
    const bool ok = ConvertManeuverType(maneuver, direction, mode);
    Expect(ok && mode == expected, name);
}

void ExpectForwardRejected(uint8_t maneuver, uint8_t direction,
                           const std::string& name) {
    BehaviorState mode = TURN_LEFT;  // 변경되지 않아야 하는 sentinel
    const bool ok = ConvertManeuverType(maneuver, direction, mode);
    Expect(!ok && mode == TURN_LEFT, name);
}

}  // namespace

int main() {
    // --- 정방향: maneuver_type (+direction) -> BehaviorState --------------
    ExpectForward(Command::KEEP, Command::DIRECTION_NONE, LANE_KEEPING,
                  "KEEP -> LANE_KEEPING");
    ExpectForward(Command::FOLLOW, Command::DIRECTION_NONE, FOLLOWING,
                  "FOLLOW -> FOLLOWING");
    ExpectForward(Command::MERGE, Command::DIRECTION_NONE, MERGE,
                  "MERGE -> MERGE");
    ExpectForward(Command::LANE_CHANGE, Command::DIRECTION_LEFT,
                  LANE_CHANGE_LEFT, "LANE_CHANGE + LEFT -> LANE_CHANGE_LEFT");
    ExpectForward(Command::LANE_CHANGE, Command::DIRECTION_RIGHT,
                  LANE_CHANGE_RIGHT, "LANE_CHANGE + RIGHT -> LANE_CHANGE_RIGHT");
    ExpectForward(Command::AVOID, Command::DIRECTION_NONE, AVOID,
                  "AVOID -> AVOID");
    ExpectForward(Command::STOP, Command::DIRECTION_NONE, STOP,
                  "STOP -> STOP");
    ExpectForward(Command::EMERGENCY_STOP, Command::DIRECTION_NONE, EMERGENCY,
                  "EMERGENCY_STOP -> EMERGENCY");

    // MERGE/AVOID는 direction이 붙어도 같은 모드로 간다(방향은 Frenet이 결정).
    ExpectForward(Command::MERGE, Command::DIRECTION_LEFT, MERGE,
                  "MERGE + LEFT still maps to MERGE");
    ExpectForward(Command::AVOID, Command::DIRECTION_RIGHT, AVOID,
                  "AVOID + RIGHT still maps to AVOID");

    // --- 정방향 거부 케이스 ------------------------------------------------
    // LANE_CHANGE에 방향이 없으면 내부 모드를 정할 수 없다.
    ExpectForwardRejected(Command::LANE_CHANGE, Command::DIRECTION_NONE,
                          "LANE_CHANGE + NONE is rejected");
    // 정의되지 않은 maneuver_type.
    ExpectForwardRejected(7, Command::DIRECTION_NONE,
                          "undefined maneuver_type 7 is rejected");
    ExpectForwardRejected(255, Command::DIRECTION_NONE,
                          "undefined maneuver_type 255 is rejected");

    // --- 역방향: BehaviorState -> active_maneuver -------------------------
    Expect(ConvertToFeedbackManeuver(LANE_KEEPING) == Command::KEEP,
           "LANE_KEEPING -> KEEP");
    Expect(ConvertToFeedbackManeuver(FOLLOWING) == Command::FOLLOW,
           "FOLLOWING -> FOLLOW");
    Expect(ConvertToFeedbackManeuver(MERGE) == Command::MERGE,
           "MERGE -> MERGE");
    Expect(ConvertToFeedbackManeuver(LANE_CHANGE_LEFT) == Command::LANE_CHANGE,
           "LANE_CHANGE_LEFT -> LANE_CHANGE");
    Expect(ConvertToFeedbackManeuver(LANE_CHANGE_RIGHT) == Command::LANE_CHANGE,
           "LANE_CHANGE_RIGHT -> LANE_CHANGE");
    Expect(ConvertToFeedbackManeuver(AVOID) == Command::AVOID,
           "AVOID -> AVOID");
    Expect(ConvertToFeedbackManeuver(STOP) == Command::STOP,
           "STOP -> STOP");
    Expect(ConvertToFeedbackManeuver(EMERGENCY) == Command::EMERGENCY_STOP,
           "EMERGENCY -> EMERGENCY_STOP");

    // legacy 전용 모드는 v2 대응 값이 없다.
    Expect(ConvertToFeedbackManeuver(INTERSECTION_WAIT) == Feedback::MANEUVER_NONE,
           "INTERSECTION_WAIT -> MANEUVER_NONE");
    Expect(ConvertToFeedbackManeuver(TURN_LEFT) == Feedback::MANEUVER_NONE,
           "TURN_LEFT -> MANEUVER_NONE");
    Expect(ConvertToFeedbackManeuver(TURN_RIGHT) == Feedback::MANEUVER_NONE,
           "TURN_RIGHT -> MANEUVER_NONE");

    // --- Round-trip: LANE_CHANGE를 제외한 6개는 원래 값으로 돌아온다 ------
    {
        const uint8_t maneuvers[] = {
            Command::KEEP, Command::FOLLOW, Command::MERGE,
            Command::AVOID, Command::STOP, Command::EMERGENCY_STOP};
        for (uint8_t maneuver : maneuvers) {
            BehaviorState mode = TURN_LEFT;
            const bool ok =
                ConvertManeuverType(maneuver, Command::DIRECTION_NONE, mode);
            Expect(ok && ConvertToFeedbackManeuver(mode) == maneuver,
                   "round-trip preserves maneuver_type "
                       + std::to_string(static_cast<int>(maneuver)));
        }
    }
    // LANE_CHANGE는 좌/우가 축약되므로 round-trip이 maneuver만 보존한다.
    {
        for (uint8_t dir : {Command::DIRECTION_LEFT, Command::DIRECTION_RIGHT}) {
            BehaviorState mode = TURN_LEFT;
            const bool ok = ConvertManeuverType(Command::LANE_CHANGE, dir, mode);
            Expect(ok && ConvertToFeedbackManeuver(mode) == Command::LANE_CHANGE,
                   "LANE_CHANGE round-trip keeps maneuver (direction collapses)");
        }
    }

    // --- Negative: static_cast는 틀린다 -----------------------------------
    // 이 블록은 "매핑 함수가 필요한 이유"를 코드로 고정한다. 누군가
    // ConvertManeuverType을 static_cast로 되돌리면 여기서 잡힌다.
    {
        BehaviorState mapped = TURN_LEFT;

        ConvertManeuverType(Command::MERGE, Command::DIRECTION_NONE, mapped);
        Expect(mapped == MERGE &&
                   static_cast<BehaviorState>(Command::MERGE) == LANE_CHANGE_LEFT &&
                   mapped != static_cast<BehaviorState>(Command::MERGE),
               "static_cast would turn MERGE into LANE_CHANGE_LEFT");

        ConvertManeuverType(Command::AVOID, Command::DIRECTION_NONE, mapped);
        Expect(mapped == AVOID &&
                   static_cast<BehaviorState>(Command::AVOID) == INTERSECTION_WAIT &&
                   mapped != static_cast<BehaviorState>(Command::AVOID),
               "static_cast would turn AVOID into INTERSECTION_WAIT");

        ConvertManeuverType(Command::STOP, Command::DIRECTION_NONE, mapped);
        Expect(mapped == STOP &&
                   static_cast<BehaviorState>(Command::STOP) == TURN_LEFT &&
                   mapped != static_cast<BehaviorState>(Command::STOP),
               "static_cast would turn STOP into TURN_LEFT");

        ConvertManeuverType(Command::EMERGENCY_STOP, Command::DIRECTION_NONE,
                            mapped);
        Expect(mapped == EMERGENCY &&
                   static_cast<BehaviorState>(Command::EMERGENCY_STOP) == TURN_RIGHT &&
                   mapped != static_cast<BehaviorState>(Command::EMERGENCY_STOP),
               "static_cast would turn EMERGENCY_STOP into TURN_RIGHT");

        // KEEP/FOLLOW만 우연히 번호가 일치한다. 이 둘로 테스트하면 cast가
        // 안전해 보이므로, 일치 자체를 명시적으로 기록해둔다.
        Expect(static_cast<BehaviorState>(Command::KEEP) == LANE_KEEPING &&
                   static_cast<BehaviorState>(Command::FOLLOW) == FOLLOWING,
               "only KEEP/FOLLOW coincide numerically (why cast looks safe)");
    }

    if (g_failures != 0) {
        std::cerr << g_failures << " enum mapping check(s) failed\n";
        return 1;
    }
    std::cout << "all v2 enum mapping checks passed\n";
    return 0;
}
