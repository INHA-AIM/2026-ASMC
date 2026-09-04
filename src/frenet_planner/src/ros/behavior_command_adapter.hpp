#pragma once

// FSM(BehaviorCommand) <-> Frenet 내부 표현 사이의 명시적 변환 계층.
//
// D3-1은 enum 매핑만 담당한다. validation과 fingerprint는 D3-2에서 이 파일에
// 추가한다.
//
// ===== 왜 static_cast를 쓰면 안 되는가 =====
//
// BehaviorCommand.maneuver_type과 내부 BehaviorState는 번호 체계가 다르다.
//
//   maneuver_type          BehaviorState
//   KEEP           = 0     LANE_KEEPING      = 0
//   FOLLOW         = 1     FOLLOWING         = 1
//   MERGE          = 2     LANE_CHANGE_LEFT  = 2
//   LANE_CHANGE    = 3     LANE_CHANGE_RIGHT = 3
//   AVOID          = 4     INTERSECTION_WAIT = 4
//   STOP           = 5     TURN_LEFT         = 5
//   EMERGENCY_STOP = 6     TURN_RIGHT        = 6
//                          AVOID             = 7
//                          MERGE             = 8
//                          STOP              = 9
//                          EMERGENCY         = 10
//
// 따라서 static_cast<BehaviorState>(msg.maneuver_type)를 하면
//   MERGE(2) -> LANE_CHANGE_LEFT
//   AVOID(4) -> INTERSECTION_WAIT
//   STOP(5)  -> TURN_LEFT
// 이 되어, FSM이 합류를 지시했는데 차선변경을 실행하는 종류의 사고가 난다.
// 계약: docs/fsm-frenet-interface-v2.md 2절

#include <cstdint>

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/FrenetFeedback.h>

#include "frenet_planner/global/global.hpp"

namespace frenet_planner_ros {

// 정방향 변환: maneuver_type (+ direction) -> 내부 BehaviorState.
//
// LANE_CHANGE만 1:2 매핑이며 direction으로 좌/우가 갈린다. LANE_CHANGE에
// DIRECTION_NONE이 오면 내부 모드를 결정할 수 없으므로 false를 반환한다
// (계약 문서 2.2절, 3.3절).
//
// 반환값 false는 "이 명령으로는 내부 모드를 정할 수 없다"는 뜻이며 호출자는
// INVALID_COMMAND로 처리한다. out_mode는 false일 때 변경되지 않는다.
bool ConvertManeuverType(uint8_t maneuver_type, uint8_t direction,
                         BehaviorState& out_mode);

// 역방향 변환: 내부 BehaviorState -> FrenetFeedback.active_maneuver.
//
// LANE_CHANGE_LEFT/RIGHT는 둘 다 LANE_CHANGE로 축약된다(2:1). 좌/우 정보는
// 사라지지만 FSM은 자신이 보낸 direction을 알고 있고 command_id로 대응
// 관계를 확인할 수 있으므로 손실이 없다.
//
// legacy 전용 모드(INTERSECTION_WAIT / TURN_LEFT / TURN_RIGHT)는 v2 계약에
// 대응 값이 없으므로 MANEUVER_NONE(255)을 반환한다.
uint8_t ConvertToFeedbackManeuver(BehaviorState mode);

}  // namespace frenet_planner_ros
