#pragma once

// FSM(BehaviorCommand) <-> Frenet 내부 표현 사이의 명시적 변환 계층.
//
//   D3-1  enum 매핑
//   D3-2  command validation + transaction fingerprint + 입력 필드 변환
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
#include <string>

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <ros/time.h>

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

// =====================================================================
// Command validation (D3-2)
// =====================================================================

// 이 enum은 **BehaviorCommand 자체의 결함**만 표현한다.
//
// runtime 상태에서 오는 실패는 여기 들어가지 않는다:
//   INPUT_DATA_STALE          Ego/Object 입력이 오래됨 -> node의 freshness 검사
//   MANEUVER_NOT_IMPLEMENTED  아직 열리지 않은 maneuver -> node의 구현 상태 조회
//
// 두 검사는 명령의 내용과 무관하므로 ValidateBehaviorCommand의 책임이 아니다.
// 계약: docs/fsm-frenet-interface-v2.md 11.2절
enum class CommandValidationCode {
    OK = 0,
    // 형식·필수 필드 오류. 원인은 FSM 측이며, FSM은 명령을 고쳐 새
    // command_id로 재발행해야 한다.
    INVALID_COMMAND,
    // valid_until 만료. 명령 내용은 정상이고 전달이 늦은 것이므로 FSM은
    // 같은 명령을 재발행하면 회복된다.
    STALE_COMMAND,
};

struct CommandValidationResult {
    CommandValidationCode code = CommandValidationCode::OK;
    // 사람이 읽는 실패 사유. FrenetFeedback.result_detail로 전달한다.
    // 판단 로직에 쓰지 않는다.
    std::string detail;

    bool ok() const { return code == CommandValidationCode::OK; }
};

const char* CommandValidationCodeName(CommandValidationCode code);

// BehaviorCommand 하나를 검증한다. 계약 문서 3.1~3.3절의 matrix 구현.
//
// now는 valid_until 비교에만 쓴다. Ego/Object 입력의 freshness는 검사하지
// 않는다(위 enum 주석 참고).
//
// target_pose_map의 유한성·quaternion 노름까지 검사하지만, RefLine 투영과
// heading 검증(계약 3.4절)은 planner를 필요로 하므로 여기서 하지 않는다.
CommandValidationResult ValidateBehaviorCommand(
    const asmc_msgs::BehaviorCommand& message, const ros::Time& now);

// =====================================================================
// Transaction fingerprint (D3-2)
// =====================================================================

// 같은 command_id 안에서 변하면 안 되는 필드의 사본.
//
// command_id는 여기 들어가지 않는다 - transaction의 key이므로 fingerprint
// 바깥에서 별도로 비교한다. 아래 네 필드도 제외한다:
//
//   execution_request  PREPARE -> EXECUTE 전이가 정상 흐름이다. 이것을 넣으면
//                      정상 전이가 "다른 명령"으로 판정되어 handshake가 깨진다.
//   header.stamp       매 발행마다 갱신된다.
//   valid_until        매 발행마다 갱신된다.
//   reason             디버그 문자열이며 판단에 쓰지 않는다.
//
// 계약: docs/fsm-frenet-interface-v2.md 4절
struct CommandFingerprint {
    uint8_t maneuver_type = 0;
    uint8_t direction = 0;
    uint8_t merge_type = 0;

    std::string source_link_id;
    std::string target_link_id;

    bool has_target_pose = false;
    // has_target_pose=false이면 비교하지 않는다.
    double target_position_x = 0.0;
    double target_position_y = 0.0;
    double target_position_z = 0.0;
    double target_orientation_x = 0.0;
    double target_orientation_y = 0.0;
    double target_orientation_z = 0.0;
    double target_orientation_w = 0.0;

    double desired_speed_mps = 0.0;

    int32_t primary_object_id = -1;
    int32_t preferred_front_object_id = -1;
    int32_t preferred_rear_object_id = -1;

    bool use_custom_follow_gap = false;
    double desired_time_gap_s = 0.0;
    double minimum_gap_m = 0.0;
};

CommandFingerprint MakeCommandFingerprint(
    const asmc_msgs::BehaviorCommand& message);

// 두 fingerprint가 같은 명령을 나타내는지.
//
// 실수 필드는 정확한 동등 비교를 하지 않는다. FSM이 같은 값을 재발행할 때
// 부동소수 표현이 동일할 것으로 가정하지 않기 위함이다(계약 4.4절).
//   속도·gap        1e-6
//   pose position   1e-3 [m]
//   pose orientation 1e-3
bool SameCommandFingerprint(const CommandFingerprint& lhs,
                            const CommandFingerprint& rhs);

// =====================================================================
// 입력 필드 변환 (D3-2)
// =====================================================================

// 검증을 통과한 BehaviorCommand의 **외부 입력 필드**만 PlannerCommand로
// 옮긴다(PlannerCommand의 [A] 그룹, global.hpp 참고).
//
// planner 계산이 필요한 것은 여기서 하지 않는다:
//   target_pose_map -> stop_position   RefLine 투영 필요
//   target_object_id -> leader_s/speed 객체 조회 + Frenet 투영 필요
// 이들은 planner를 가진 호출자가 채운다.
//
// FOLLOW gap 매핑(계약 11.8절):
//   use_custom_follow_gap=false -> time_gap=0.0, min_gap=0.0
//   config 기본값 적용은 소비 지점(D5)에서 한다.
//
// 반환값 false는 enum 매핑 실패를 뜻한다. ValidateBehaviorCommand를 먼저
// 통과시켰다면 발생하지 않지만, 순서가 바뀌어도 안전하도록 확인한다.
bool ConvertBehaviorCommand(const asmc_msgs::BehaviorCommand& message,
                            PlannerCommand& out_command);

}  // namespace frenet_planner_ros
