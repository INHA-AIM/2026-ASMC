// 외부(FSM) 또는 planner 내부 legacy 로직이 채우는 주행 명령.

#ifndef PLANNER_COMMON_PLANNER_COMMAND_HPP
#define PLANNER_COMMON_PLANNER_COMMAND_HPP

#include <limits>

#include "frenet_planner/Common/BehaviorTypes.hpp"

// aggregate 구조를 유지하되(생성자 없음) 모든 필드에 안전한 기본값을 준다.
// PlannerCommand cmd; / PlannerCommand cmd{}; 두 형태 모두 미초기화 필드가
// 없도록 하는 것이 목적이다 (mode 미초기화로 인한 임의 모드 진입 방지).
struct PlannerCommand {
    // =====================================================================
    // [A] 외부 명령 입력 - v2에서 FSM(BehaviorCommand)이 채운다
    //
    // legacy 경로에서는 planner의 간이 FSM이 채우지만, v2에서는 이 그룹만이
    // 외부에서 들어오는 값이다. 여기 있는 필드를 planner가 계산해서 되써넣으면
    // "명령"과 "계산 결과"의 구분이 무너진다 - 그것이 D2 이전 상태였다.
    // 상세: docs/fsm-frenet-interface-v2.md 12.3절
    //
    // 알려진 예외 (D2에서 의도적으로 남김):
    //   STOP 실행부(frenet_planner.cpp의 APPROACH/BRAKING)는 cmd.mode와
    //   cmd.stop_position, cmd.target_speed를 후보 생성기 선택용 scratch로
    //   덮어쓴다. 예: APPROACH에서 cmd.mode=LANE_KEEPING으로 바꿔 velocity
    //   keeping 생성기에 위임하고, 외부 보고용 last_mode_는 STOP으로 유지한다.
    //   C4에서 MORAI로 검증한 코드이므로 D2에서는 건드리지 않는다
    //   (docs/d1-legacy-behavior-boundary.md 5.1절). 이 예외를 없애려면 후보
    //   생성기 선택을 cmd.mode와 분리하는 별도 작업이 필요하다.
    // =====================================================================

    // 어떤 maneuver를 수행할지. v2에서는 BehaviorCommand.maneuver_type +
    // direction을 명시적 매핑으로 변환해 채운다(static_cast 금지 - 두 enum의
    // 번호 체계가 다르다. 계약 문서 2절).
    BehaviorState mode = LANE_KEEPING;

    // 기동 대상 객체 ID. FOLLOW의 선행차와 AVOID의 회피 장애물이 같은 필드를
    // 쓴다 - BehaviorCommand가 둘 다 primary_object_id 하나로 주기 때문이다
    // (계약 문서 3.3절). 대상이 없는 maneuver(KEEP/STOP/EMERGENCY_STOP)는 -1.
    //
    // v2에서 이 값은 authoritative다. planner가 "더 가까운 차가 있다"는 이유로
    // 스스로 바꾸면 FSM은 자기가 지시한 대상을 추종 중이라고 오인한다
    // (docs/d1-legacy-behavior-boundary.md 0.1절 B 분류).
    int target_object_id = -1;

    // 외부가 지정한 목표속도. has_target_speed=false이면 planner 기본값 사용.
    // v2 매핑: desired_speed_mps < 0 -> false, >= 0 -> true (계약 문서 3.5절).
    bool has_target_speed = false;
    double target_speed = 0.0;              // [m/s]

    // 목표속도 상한. BehaviorCommand에 대응 필드가 없어 v2 경로에서는 쓰지
    // 않는다(항상 false). legacy/internal 경로를 위해 유지한다.
    bool has_max_speed = false;
    double max_speed = 0.0;                 // [m/s]

    // STOP 모드의 정지 목표 s [m].
    // 0.0은 실제로 유효한 Frenet s이므로 "미지정"을 0으로 표현할 수 없다.
    // 기본값을 NaN으로 두어 std::isfinite(stop_position)로 지정 여부를 판별한다.
    // v2에서는 target_pose_map을 RefLine에 투영해 채운다.
    double stop_position = std::numeric_limits<double>::quiet_NaN();

    // FOLLOW 간격 설정. v2 매핑: use_custom_follow_gap=true일 때
    // desired_time_gap_s / minimum_gap_m를 그대로, false이면 config 기본값.
    double time_gap = 0.0;                  // 목표 시간 간격 tau [s]
    double min_gap = 0.0;                   // 최소 거리 D0 [m]

    // MERGE 종류. v2에서는 BehaviorCommand.merge_type을 그대로 옮긴다.
    MergeType merge_type = MergeType::NONE;

    // MERGE Gap 선호 차량. FSM이 지정하지 않으면 -1이고, 그때는 planner의
    // gap selector가 자동 선택한다(계약 문서 3.3절).
    int merge_sa_id = -1;                   // Gap 앞 차량
    int merge_sb_id = -1;                   // Gap 뒤 차량

    // FSM이 요청한 횡방향 기동 방향 (D6-1).
    //
    // AVOID에서만 의미가 있다. mode=AVOID 하나로는 BehaviorCommand의
    // DIRECTION_LEFT / RIGHT / NONE을 구분할 수 없어, 변환 과정에서 방향이
    // 사라지고 있었다. LANE_CHANGE는 방향이 mode로 갈라지므로 이 필드를
    // 쓰지 않는다(LANE_CHANGE_LEFT / LANE_CHANGE_RIGHT).
    //
    // NONE은 "방향 미지정"이며 Frenet이 좌/우를 고른다 - 그것이
    // BehaviorCommand.msg가 정의한 의미다. legacy가 항상 양쪽을 만들어
    // 비교하던 동작은 NONE에 해당하고, LEFT/RIGHT가 오면 그 방향만
    // 평가해야 한다(계약 3.3절).
    LateralDirection requested_direction = LateralDirection::NONE;

    // FSM이 준 AVOID 목표점을 Frenet으로 투영한 결과 (D6-1a).
    //
    // ===== 왜 avoidance_d_offset에 바로 넣지 않는가 =====
    //
    // avoidance_d_offset은 [B] planner 계산 결과다. 외부에서 온 의도를 그
    // 필드에 넣으면 D2에서 만든 command/result 경계가 다시 흐려진다 -
    // "FSM이 지정한 목표"와 "planner가 후보 비교로 정한 목표"를 구분할 수
    // 없게 된다.
    //
    // 그래서 투영 결과는 여기 [A]에 두고, D6-2가 최종 목표를 정할 때
    //   has_avoid_target_d=true  -> pose에서 온 avoid_target_d 사용
    //   has_avoid_target_d=false -> 장애물 geometry + requested_direction으로 계산
    // 하여 결과를 avoidance_d_offset에 쓴다.
    //
    // BehaviorCommand.has_target_pose가 false이거나 투영이 실패하면 false다.
    // AVOID에서 target pose는 선택 사항이다(원본 메시지: "AVOID: 필요하면
    // 회피 목표점").
    //
    // 0.0은 유효한 d(차선 중앙)이므로 "미지정"을 0으로 표현할 수 없다.
    // 그래서 별도 유효성 플래그를 둔다(계약 1.3절과 같은 이유 - d는 음수가
    // 유효하므로 -1.0 sentinel도 쓸 수 없다).
    bool has_avoid_target_d = false;
    double avoid_target_d = 0.0;            // [m]

    // 위 투영의 종방향 좌표 [m]. has_avoid_target_d가 true일 때만 유효하다.
    // 회피 목표점이 장애물보다 앞/뒤인지 판단하는 데 쓴다(D6-2).
    double avoid_target_s = 0.0;

    // =====================================================================
    // [B] Planner 계산 결과 - planner가 채워 후보 생성기에 전달한다
    //
    // FSM은 이 값들을 주지 않는다. 대상 ID와 모드만 받아서 planner가 직접
    // 계산한다(docs/d1-legacy-behavior-boundary.md 6.2절). 구조체를 나누지
    // 않고 한 곳에 두는 이유는 ResolveManeuver가 단일 cmd를 받는 기존
    // 인터페이스를 유지하기 위함이다 - 경계는 이 주석과 필드 배치로 나타낸다.
    // =====================================================================

    // 횡방향 목표
    int target_lane = 0;                    // Frenet d 기준: -1 우, 0 유지, 1 좌
    double avoidance_d_offset = 0.0;        // AVOID 목표 횡오프셋 [m]

    // 종방향 gap 제어 이득 (config에서 옴)
    double gap_gain = 0.0;                  // 간격 오차를 접근 속도로 변환 [1/s]

    // 대상 객체를 Frenet으로 투영한 결과. target_object_id로 조회해서 채운다.
    double leader_s = 0.0;                  // 선행 차량 위치 [m]
    double leader_speed = 0.0;              // 선행 차량 속도 [m/s]
    double leader_accel = 0.0;              // 선행 차량 가속도 [m/s^2]

    // MERGE(Sec.V-A): 대상 차선 두 차량 sa(앞)/sb(뒤)의 등가속 예측 상태
    double merge_sa_s = 0.0;
    double merge_sa_speed = 0.0;
    double merge_sa_accel = 0.0;
    double merge_sb_s = 0.0;
    double merge_sb_speed = 0.0;
    double merge_sb_accel = 0.0;

    // MERGE 판정/진행 상태 (MergeContext에서 옴)
    double merge_target_d = 0.0;
    bool merge_gap_safe = false;
    double merge_conflict_s = 0.0;
    double merge_stop_s = 0.0;
    double merge_entry_time = 0.0;
    bool merge_committed = false;           // v2에서는 FSM EXECUTE가 trigger
    bool merge_crossing = false;
};

#endif  // PLANNER_COMMON_PLANNER_COMMAND_HPP
