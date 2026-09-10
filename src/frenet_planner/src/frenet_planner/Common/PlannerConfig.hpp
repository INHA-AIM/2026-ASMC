#ifndef PLANNER_COMMON_PLANNER_CONFIG_HPP
#define PLANNER_COMMON_PLANNER_CONFIG_HPP

#include <string>
#include <vector>

// =========================================================
// 곡률 기반 사전 감속 (mpc_controller의 옛 planner/path_planner.cpp 로직을
// Frenet Frame Planner 쪽으로 이식한 것). lookahead 구간 내 최대 곡률로
// velocity-keeping 목표속도를 정한다.
// =========================================================
struct CurveSpeedConfig {
    double target_vel        = 10.0;
    double curve_vel_sharp   = 4.0;
    double curve_vel_mid     = 5.5;
    double curve_vel_mild    = 7.0;
    double curve_th_sharp    = 0.080;
    double curve_th_mid      = 0.035;
    double curve_th_mild     = 0.015;
    double curve_lookahead_m = 15.0;
};

// =========================================================
// Following(Sec.V-A) 모드 파라미터. ResolveExternalFollowTarget이 FSM이
// 지정한 FOLLOW 대상(target_object_id)의 유효성을 검증할 때(탐색 범위 -
// FSM이 지정한 대상이라도 진행방향/같은 차선/탐색거리 조건을 벗어나면
// TARGET_OBJECT_LOST다)와 GenerateFollowingCandidates가
// constant-time-gap-law target을 만들 때(time_gap=tau, min_gap=D0) 쓴다.
// =========================================================
struct FollowingConfig {
    double time_gap = 1.0;              // [s] tau
    double min_gap  = 5.0;              // [m] D0
    double gap_gain = 0.3;              // [1/s] v_cmd = v_leader + gain * gap_error
    double max_leader_search_s = 60.0;  // [m] FSM이 지정한 대상까지 이 거리 안이어야 유효
    double exit_search_margin = 10.0;   // [m] 확보 후 유지용 탐색거리 hysteresis
    int dropout_grace_cycles = 5;       // [cycle] TrackedObject 일시 누락 허용(20Hz)
};

// =========================================================
// FSM이 지정한 AVOID 대상(target_object_id)의 유효성 검증 범위와 횡오프셋
// 파라미터. 어떤 장애물을 회피할지 planner가 스스로 고르지 않는다 -
// FSM이 지정한 대상이 이 범위/조건을 만족하는지만 검증한다.
// =========================================================
struct AvoidConfig {
    double detection_distance = 60.0; // [m] FSM이 지정한 대상까지 이 거리 안이어야 유효(실제값은 충돌 lookahead와 자동 조정)
    // [m] 회피 시 lateral_d1 격자를 밀어낼 크기. 자차 폭(1.9m 기준) + 장애물
    // 폭 절반씩 + 여유(0.3m)를 감안한 최소 이격(~2.2m)보다 확실히 크게 잡아야
    // 좌/우 클리어 판정이 "장애물 자신" 때문에 항상 막힌 것으로 나오지 않는다.
    double avoid_offset      = 3.0;
    // 동적 SHIFT 시작거리 = ego_speed * required_lateral_shift_time
    //                         + vehicle/obstacle clearance.
    // 아래 값은 지나치게 가깝거나 먼 결과를 막는 안전 경계다.
    double min_shift_start_distance = 3.0;
    double max_shift_start_distance = 70.0;
    double shift_time_margin = 0.5; // [s] 조향/제어 응답 여유
    double lateral_tolerance = 0.4;   // [m] SHIFT 완료 판정
    // VehicleState의 x/y 기준점(현재 MORAI IONIQ 5는 후륜축 GPS)에서
    // 뒤 범퍼까지 거리. AVOID의 접근/통과 종방향 여유를 차량 중심 기준
    // length/2로 계산하지 않도록 사용한다.
    double rear_overhang     = 0.79;  // [m]
    // 후륜축에서 뒤 범퍼까지의 거리와 장애물 반길이는 별도로 포함한다.
    // 기본값 0은 뒤 범퍼가 장애물 후단을 지난 즉시 RETURN한다는 뜻이다.
    double pass_clearance    = 0.0;   // [m] 차체 형상 외 추가 통과 여유
    bool prefer_right_when_equal = true; // 좌/우 후보 품질이 같을 때 우측 우선

    // COMMIT 때 잠근 횡목표가 최신 장애물 형상과 충돌하면 기존 회피
    // 방향의 바깥쪽으로만 목표를 확장해 다시 검사한다. 첫 목표가 유효하면
    // 재탐색하지 않으므로 정상 주행의 lock 의미는 그대로 유지된다.
    double committed_recovery_step = 0.5;          // [m]
    double committed_recovery_max_expansion = 2.0; // [m]

    // [cycles] v2 AVOID 대상의 일시적 미관측 유예 (D6-1).
    //
    // FollowingConfig.dropout_grace_cycles를 가져다 쓰지 않는다 - 서로 다른
    // 행동의 config가 강하게 결합되면 FOLLOW 튜닝이 AVOID 동작을 바꾼다.
    int dropout_grace_cycles = 5;
};

// =========================================================
// v2 LANE_CHANGE의 SHIFT -> HOLD_TARGET_LANE 전환 tolerance. 초기 개발값이며
// MORAI 튜닝 대상이다 - AVOID/MERGE와 마찬가지로 magic number를 코드에
// 흩뿌리지 않고 여기 모은다.
// =========================================================
struct LaneChangeConfig {
    double lateral_position_tolerance = 0.4;  // [m] 목표 d와의 허용 오차
    double lateral_speed_tolerance = 0.2;     // [m/s] 완료 시점 허용 d_dot
    double heading_tolerance = 0.05;          // [rad] 목표/참조 heading과의 허용 오차
};

// =========================================================
// 회전교차로 진입 conflict point와 time-gap 판단 파라미터.
// =========================================================
struct MergeConfig {
    double conflict_s = -1.0;           // [m] global path상의 회전교차로 진입 충돌점; <0이면 비활성
    double approach_distance = 45.0;    // [m] conflict point 전 MERGE 활성 구간
    double stop_buffer = 3.0;           // [m] gap이 없을 때 conflict point 전 정지 여유
    double conflict_radius = 6.0;       // [m] 순환 차량 궤적이 충돌점을 지난다고 볼 최근접거리
    double min_object_speed = 0.5;      // [m/s] 시간 예측에 사용할 최소 순환차 속도
    double min_front_time_gap = 1.5;    // [s] 먼저 지나간 차량 뒤 진입 여유
    double min_rear_time_gap = 2.0;     // [s] 진입 후 다음 차량 도착 여유
    double ego_clear_time = 1.5;        // [s] Ego가 conflict zone을 비우는 시간
    double max_wait_time = 10.0;        // [s] 탐색할 최대 진입 대기시간
    double completion_distance = 8.0;   // [m] conflict point 통과 후 MERGE 종료 거리
    double commit_distance = 15.0;      // [m] 이 거리 안에서 선택한 gap을 고정
    int safe_confirm_cycles = 3;        // 순간 오검출로 진입하지 않도록 연속 안전 확인
    double cross_speed_floor = 3.0;     // [m/s] COMMIT/CROSS 중 교차구역 이탈 최저 목표속도
};

// =========================================================
// 외부 STOP 명령의 실행 파라미터.
// AVOID/MERGE와 감속 특성이 달라(정지선 정확도가 중요) comfortable_decel을
// avoid_cfg와 공유하지 않고 별도로 둔다.
// =========================================================
struct StopConfig {
    // [m/s^2] APPROACH 감속 프로파일과 BRAKING 진입거리 계산에 쓰는 감속도.
    double comfortable_decel = 2.0;
    // [m] 제동거리에 더해 BRAKING으로 전환할 여유. 이 값이 있어야 정확히
    // 제동한계에서 전환하지 않고 한 박자 일찍 정지 다항식을 만들 수 있다.
    double braking_margin = 3.0;
    // [m] 정지 목표 앞에 남길 여유. APPROACH 속도 프로파일이 목표점이 아니라
    // 이 buffer만큼 앞을 향하도록 해 오버슈트를 줄인다.
    double stop_buffer = 0.5;
    // [m] 목표 도달 판정. 이 안에 들어오고 속도도 낮으면 HOLD.
    double position_tolerance = 0.7;
    // [m/s] HOLD 진입 속도 판정.
    double speed_tolerance = 0.15;
    // [m/s] 이 속도 이하이고 이미 차선 중심 허용 범위 안이면 정지 직전의
    // 불필요한 중앙 복귀 대신 현재 횡위치에서 d_dot=0으로 정렬한다.
    double lateral_hold_speed = 1.0;
    // [m] 저속 STOP에서 현재 횡위치를 유지해도 되는 차선 중심 오차.
    double lateral_error_tolerance = 0.25;
    // [m] Cartesian 정지 목표가 reference line에서 이만큼 넘게 떨어져 있으면
    // 그 명령은 이 경로에 대한 정지 지점이 아니라고 보고 거부한다.
    double max_lateral_offset = 5.0;
    // [m] 이만큼 넘게 목표가 바뀌면 새 STOP 명령으로 보고 실행 phase를 리셋한다.
    double target_change_tolerance = 2.0;
    // [m] 정지 목표를 s로 투영할 때 FindClosestS에 넘길 탐색창 반경.
    //
    // FindClosestS의 기본 창(30m)은 ego localization용이다 - 한 틱에 몇 m만
    // 움직이는 ego의 최근접점이 평행/헤어핀 구간에서 엉뚱한 s로 튀는 것을
    // 막으려고 좁게 잡혀 있다. 정지 목표는 ego보다 훨씬 앞에 있으므로 같은
    // 창을 쓰면 목표가 창 밖으로 나가고, 잘린 초기 추정값에서 시작한 Newton이
    // 창 경계에 머무는 게 아니라 트랙의 다른 구간으로 발산한다
    // (실측: ego_s=100에서 전방 100m 목표가 s=1640으로, 오차 735m).
    //
    // 반드시 실제 사용할 최대 stop lookahead보다 크게 잡아야 한다. 이 값보다
    // 먼 정지 목표는 투영이 틀어져 max_lateral_offset 검사에서 거부되고,
    // 정상 명령이 INVALID_COMMAND로 즉시 정지하게 된다.
    double projection_search_window = 100.0;
};

// Emergency braking has no fixed stop target.  This deceleration determines
// the predicted stop point of the direct emergency trajectory and must match
// the full-brake actuator assumption used by the controller.
struct EmergencyStopConfig {
    double decel = 8.9;  // [m/s^2]
};

// 고주로/본선 합류 구간. Global Path가 이미 램프에서 본선으로 이어지는 형상을
// 가지므로 횡경로는 d=0을 유지하고, 본선 corridor의 선행/후행 차량 사이 시간·
// 거리 gap에 맞춰 종방향 진입 시점만 결정한다.
struct HighwayMergeCheckpoint {
    double conflict_s = -1.0;           // [m] 충돌 가능 구간 진입점
    double clear_s = -1.0;              // [m] 해당 충돌 구간 이탈점
};

struct HighwayMergeZone {
    std::string name;
    double start_s = -1.0;              // [m] 합류 정책 활성 시작점
    double completion_s = -1.0;         // [m] 본선 중앙 정렬이 끝나는 점
    std::vector<HighwayMergeCheckpoint> checkpoints;
};

struct HighwayMergeConfig {
    bool enabled = false;
    std::vector<HighwayMergeZone> zones;
    double approach_distance = 50.0;    // [m] 향후 속도 기반 사전 감지 확장 여유
    double stop_buffer = 4.0;            // [m] unsafe일 때 conflict_s 전 정지 여유
    double target_corridor_half_width = 5.5; // [m] 합류 전 평행 본선까지 포함
    double object_search_distance = 100.0;   // [m] conflict 전후 차량 탐색 범위
    double min_heading_alignment = 0.5; // cos(delta heading) 하한
    double min_front_gap = 8.0;         // [m] 합류 완료 시 선행차 최소 거리
    double min_rear_gap = 10.0;         // [m] 합류 완료 시 후행차 최소 거리
    double min_front_ttc = 2.0;         // [s]
    double min_rear_ttc = 3.0;          // [s]
    double max_wait_time = 10.0;        // [s]
    double commit_distance = 20.0;      // [m]
    int safe_confirm_cycles = 3;
    double cross_speed_floor = 4.0;     // [m/s]
};

#endif  // PLANNER_COMMON_PLANNER_CONFIG_HPP
