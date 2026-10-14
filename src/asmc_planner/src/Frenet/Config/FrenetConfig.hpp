#ifndef PLANNER_CONFIG_FRENET_CONFIG_HPP
#define PLANNER_CONFIG_FRENET_CONFIG_HPP

#include <string>
#include <vector>

#include "Frenet/Generator/GeneratorConfig.hpp"

// =========================================================
// TrackedObject의 연속 heading으로 yaw-rate를 추정/보관하는 파라미터
// (frenet_planner.cpp의 PreparePlanningContext). 회전교차로/합류에서 순환
// 차량의 곡선 예측(Merge.cpp)에 쓰인다.
// =========================================================
struct ObjectTrackingConfig {
    // [rad/s] 관측된 yaw-rate의 절대값 상한. UDP heading 양자화/순간 점프로
    // 인한 비현실적인 값을 차량 수준으로 제한한다.
    double yaw_rate_clamp_rad_s;
    // 지수평활 계수(0-1). 새 관측치에 주는 가중치 - 클수록 더 빠르게
    // 반응하지만 노이즈에 더 민감해진다.
    double yaw_rate_smoothing_alpha;
    // [cycles] 이 사이클 수 넘게 관측되지 않은 객체 트랙을 제거한다.
    int missing_cycles_eviction;
    // [rad] Minimum velocity-direction change supported by a fresh history.
    double min_motion_direction_change_rad = .05;
};

// =========================================================
// 곡률 기반 사전 감속 (mpc_controller의 옛 planner/path_planner.cpp 로직을 Frenet Frame Planner 쪽으로 이식한 것).
// lookahead 구간 내 최대 곡률로 velocity-keeping 목표속도를 정한다.
// =========================================================
struct CurveSpeedConfig {
    double target_vel;
    // 속도 기반 제동거리 앞에 추가하는 곡률 preview 거리 [m].
    double curve_lookahead_m;
    // 횡가속도 한계의 몇 %만 사용할지(여유폭 확보). v_curve =
    // sqrt(max_lateral_accel * lateral_accel_safety_factor / |kappa|).
    double lateral_accel_safety_factor;
    // 실제 평균 감속도를 max_longitudinal_accel의 몇 배로 가정할지
    // (lookahead 제동거리 계산에 사용).
    double comfortable_decel_factor;
    // [m/s^2] usable_decel(=max_longitudinal_accel*comfortable_decel_factor)의
    // 최소 하한 - 설정된 감속 한계가 매우 작아도 lookahead 제동거리 계산이
    // 발산하지 않게 한다.
    double min_usable_decel;
};

// =========================================================
// Following(Sec.V-A) 모드 파라미터. ResolveExternalFollowTarget이 FSM이
// 지정한 FOLLOW 대상(target_object_id)의 유효성을 검증할 때(탐색 범위 -
// FSM이 지정한 대상이라도 진행방향/같은 차선/탐색거리 조건을 벗어나면
// TARGET_OBJECT_LOST다)와 GenerateFollowingCandidates가
// constant-time-gap-law target을 만들 때(time_gap=tau, min_gap=D0) 쓴다.
// =========================================================
struct FollowingConfig {
    double time_gap;              // [s] tau
    double min_gap;                // [m] D0
    double gap_gain;               // [1/s] v_cmd = v_leader + gain * gap_error
    double max_leader_search_s;    // [m] FSM이 지정한 대상까지 이 거리 안이어야 유효
    double exit_search_margin;     // [m] 확보 후 유지용 탐색거리 hysteresis
    int dropout_grace_cycles;      // [cycle] TrackedObject 일시 누락 허용(20Hz)
};

// =========================================================
// FSM이 지정한 AVOID 대상(target_object_id)의 유효성 검증 범위와 횡오프셋
// 파라미터. 어떤 장애물을 회피할지 planner가 스스로 고르지 않는다 -
// FSM이 지정한 대상이 이 범위/조건을 만족하는지만 검증한다.
// =========================================================
struct AvoidConfig {
    // Cooperative search slice; unfinished candidates resume from fresh inputs.
    double map_search_budget_s = 0.08;
    // Map-aware bypass cruise speed; independent of KEEP's low-speed fallback.
    double bypass_speed_cap = 2.0; // [m/s]
    double detection_distance; // [m] FSM이 지정한 대상까지 이 거리 안이어야 유효(실제값은 충돌 lookahead와 자동 조정)
    // [m] 회피 시 lateral_d1 격자를 밀어낼 크기. 자차 폭(1.9m 기준) + 장애물
    // 폭 절반씩 + 여유(0.3m)를 감안한 최소 이격(~2.2m)보다 확실히 크게 잡아야
    // 좌/우 클리어 판정이 "장애물 자신" 때문에 항상 막힌 것으로 나오지 않는다.
    double avoid_offset;
    // [m] legacy geometry 기반(target pose 없음) target_d 계산에서, 자차/
    // 장애물 반폭 합 + safety_margin에 추가로 더하는 여유.
    double direction_extra_margin;
    // 동적 SHIFT 시작거리 = ego_speed * required_lateral_shift_time
    //                         + vehicle/obstacle clearance.
    // 아래 값은 지나치게 가깝거나 먼 결과를 막는 안전 경계다.
    double min_shift_start_distance;
    double max_shift_start_distance;
    double shift_time_margin; // [s] 조향/제어 응답 여유
    // 동적으로 계산된 SHIFT 시작거리에 직접 더하는 signed 보정 [m].
    // 음수면 늦게, 양수면 일찍 시작하며 마지막에 min/max 안전경계를 적용한다.
    double shift_start_distance_offset;
    double lateral_tolerance;   // [m] SHIFT 완료 판정
    // VehicleState의 x/y 기준점(현재 MORAI IONIQ 5는 후륜축 GPS)에서
    // 뒤 범퍼까지 거리. AVOID의 접근/통과 종방향 여유를 차량 중심 기준
    // length/2로 계산하지 않도록 사용한다.
    double rear_overhang;  // [m]
    // 후륜축에서 뒤 범퍼까지의 거리와 장애물 반길이는 별도로 포함한다.
    // 0은 뒤 범퍼가 장애물 후단을 지난 즉시 RETURN한다는 뜻이다.
    double pass_clearance;   // [m] 차체 형상 외 추가 통과 여유
    bool prefer_right_when_equal; // 좌/우 후보 품질이 같을 때 우측 우선

    // COMMIT 때 잠근 횡목표가 최신 장애물 형상과 충돌하면 기존 회피
    // 방향의 바깥쪽으로만 목표를 확장해 다시 검사한다. 첫 목표가 유효하면
    // 재탐색하지 않으므로 정상 주행의 lock 의미는 그대로 유지된다.
    double committed_recovery_step;          // [m]
    double committed_recovery_max_expansion; // [m]

    // 정지/저속 AVOID는 시간 기반 d(t) 대신 거리 기반 d(s) 후보를 쓴다.
    // 최소 물리 전환거리의 배수를 샘플링해 하나의 고정 fallback 경로가
    // 아니라 곡률/충돌 검사를 통과하는 복수 후보를 만든다.
    double low_speed_transition_scale_min;
    double low_speed_transition_scale_max;
    double low_speed_transition_scale_step;
    double low_speed_path_extension; // [m] 횡이동 완료 뒤 검증 구간

    // [cycles] v2 AVOID 대상의 일시적 미관측 유예 (D6-1).
    //
    // FollowingConfig.dropout_grace_cycles를 가져다 쓰지 않는다 - 서로 다른
    // 행동의 config가 강하게 결합되면 FOLLOW 튜닝이 AVOID 동작을 바꾼다.
    int dropout_grace_cycles;

    // COMMIT 이후 target_index별 비용 분리폭. 잠긴(먼저 나열된) target을
    // 다른 target보다 항상 우선시키기 위해, 물리적 전환거리 비용 차이보다
    // 훨씬 크게 잡는다.
    double committed_target_index_cost_scale;
    // PREPARE 단계에서 좌/우 후보의 전환거리 비용이 동률일 때, 선호 방향이
    // 아닌 쪽에 더하는 미세 tie-break 가중치. 실제 궤적 품질에는 영향을
    // 주지 않고 방향 선택의 대칭만 깬다.
    double direction_tiebreak_cost;

    // LiDAR tracker가 동일 물체에 새 ID를 부여했을 때, 직전 AVOID target과
    // 공간적으로 유일하게 이어지는 관측을 같은 물체로 재연결한다.
    bool spatial_reconnect_enabled;
    double reconnect_max_longitudinal_delta; // [m]
    double reconnect_max_lateral_delta;      // [m]
    double reconnect_max_size_delta;         // [m]
};

// =========================================================
// v2 LANE_CHANGE의 SHIFT -> HOLD_TARGET_LANE 전환 tolerance. 초기 개발값이며
// MORAI 튜닝 대상이다 - AVOID/MERGE와 마찬가지로 magic number를 코드에
// 흩뿌리지 않고 여기 모은다.
// =========================================================
struct LaneChangeConfig {
    double lateral_position_tolerance;  // [m] 목표 d와의 허용 오차
    double lateral_speed_tolerance;     // [m/s] 완료 시점 허용 d_dot
    double heading_tolerance;           // [rad] 목표/참조 heading과의 허용 오차
};

// =========================================================
// 회전교차로 진입 conflict point와 time-gap 판단 파라미터.
// =========================================================
struct MergeConfig {
    double conflict_s;           // [m] global path상의 회전교차로 진입 충돌점; <0이면 비활성
    double approach_distance;    // [m] conflict point 전 MERGE 활성 구간
    double stop_buffer;          // [m] gap이 없을 때 conflict point 전 정지 여유
    double conflict_radius;      // [m] 순환 차량 궤적이 충돌점을 지난다고 볼 최근접거리
    double min_object_speed;     // [m/s] 시간 예측에 사용할 최소 순환차 속도
    double min_front_time_gap;   // [s] 먼저 지나간 차량 뒤 진입 여유
    double min_rear_time_gap;    // [s] 진입 후 다음 차량 도착 여유
    double ego_clear_time;       // [s] Ego가 conflict zone을 비우는 시간
    double max_wait_time;        // [s] 탐색할 최대 진입 대기시간
    double completion_distance;  // [m] conflict point 통과 후 MERGE 종료 거리
    double commit_distance;      // [m] 이 거리 안에서 선택한 gap을 고정
    int safe_confirm_cycles;     // 순간 오검출로 진입하지 않도록 연속 안전 확인
    double cross_speed_floor;    // [m/s] COMMIT/CROSS 중 교차구역 이탈 최저 목표속도
    // [m] 물리적 제동거리에 더하는 여유. 이 여유를 넘어서야 정지 quintic으로
    // 전환한다 - 여유가 없으면 먼 거리부터 모든 T 후보가 조기 감속한다.
    double braking_distance_margin;
    // [m/s] MERGE 도착/추종 목표속도의 최소 하한. entry_time 기반 계산이
    // 0에 가까운 크리프 속도를 요구하지 않도록 한다.
    double min_arrival_speed;
    // [s] 회전교차로 순환 차량 constant-turn-rate 예측의 시간 샘플링 간격.
    double prediction_dt;
    // [s] 진입시각 후보를 훑는 탐색 간격.
    double entry_search_dt;
    // [m/s] MinimumTravelTime의 최소 목표/현재 속도 가정(가속 시간 계산이
    // 0에 가까운 속도로 나누지 않도록).
    double min_travel_speed;
    // [m/s^2] MinimumTravelTime의 최소 가속도 가정.
    double min_travel_accel;
    // [s] GenerateMergeArrivalCandidates가 entry_time 주변 T 격자를 남길 때
    // 쓰는 최소 매칭 허용폭(time_horizon.step보다 좁으면 이 값이 우선한다).
    double arrival_time_match_tolerance;
    // [s] arrival_speed = distance / merge_entry_time 계산의 분모 하한.
    // entry_time이 이보다 짧으면 0에 가까운 시간으로 나눠 속도가 발산하므로
    // target_speed로 대체한다. entry_search_dt(진입시각 탐색 간격)와는 의미가
    // 달라 별도 파라미터로 둔다.
    double min_entry_time_for_arrival_speed;
};

// =========================================================
// 외부 STOP 명령의 실행 파라미터.
// AVOID/MERGE와 감속 특성이 달라(정지선 정확도가 중요) comfortable_decel을
// avoid_cfg와 공유하지 않고 별도로 둔다.
// =========================================================
struct StopConfig {
    // [m/s^2] APPROACH 감속 프로파일과 BRAKING 진입거리 계산에 쓰는 감속도.
    double comfortable_decel;
    // Ordinary STOP restart limits; do not change the braking limit.
    double reapproach_speed = 1.0;
    double reapproach_accel = 0.5;
    // [m/s^2] 일반 STOP에서 허용하는 최대 감속도. 남은 거리로 계산한 요구
    // 감속도가 comfortable_decel보다 크면 이 값까지 연속적으로 강화한다.
    // MPC constraints.acceleration.max_reverse와 같은 크기로 유지해야 한다.
    double max_service_decel;
    // [m] 제동거리에 더해 BRAKING으로 전환할 여유. 이 값이 있어야 정확히
    // 제동한계에서 전환하지 않고 한 박자 일찍 정지 다항식을 만들 수 있다.
    double braking_margin;
    // [m] 정지 목표 앞에 남길 여유. APPROACH 속도 프로파일이 목표점이 아니라
    // 이 buffer만큼 앞을 향하도록 해 오버슈트를 줄인다.
    double stop_buffer;
    // [m] 목표 도달 판정. 이 안에 들어오고 속도도 낮으면 HOLD.
    double position_tolerance;
    // [m/s] HOLD 진입 속도 판정.
    double speed_tolerance;
    // [m/s] 이 속도 이하이고 이미 차선 중심 허용 범위 안이면 정지 직전의
    // 불필요한 중앙 복귀 대신 현재 횡위치에서 d_dot=0으로 정렬한다.
    double lateral_hold_speed;
    // [m] 저속 STOP에서 현재 횡위치를 유지해도 되는 차선 중심 오차.
    double lateral_error_tolerance;
    // [m] Cartesian 정지 목표가 reference line에서 이만큼 넘게 떨어져 있으면
    // 그 명령은 이 경로에 대한 정지 지점이 아니라고 보고 거부한다.
    double max_lateral_offset;
    // [m] 이만큼 넘게 목표가 바뀌면 새 STOP 명령으로 보고 실행 phase를 리셋한다.
    double target_change_tolerance;
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
    double projection_search_window;
    // 공용 delta_s 격자([-3,+3]m)는 +쪽 후보가 정지 목표를 지나쳐서 멈춘다.
    // 정지선 semantics에서는 오버슈트가 곧 위반이므로 STOP/INTERSECTION_WAIT
    // 전용으로 더 좁은 범위를 쓴다(목표 앞에서만 후보 생성).
    SamplingRange delta_s_override;
    // [m/s^2] GenerateDecelToZeroPath(STOP/EMERGENCY 공유)가 실제로 쓰는
    // 감속도의 최소 하한. comfortable_decel/emergency_stop.decel은 이미
    // ConfigValidator가 >0으로 검증하므로 이 값은 "설정이 잘못됐을 때의
    // 방어"가 아니라, 0에 매우 가까운 유효값이 들어왔을 때 생성 루프가
    // 비정상적으로 길어지는 것을 막는 안전 하한이다.
    double min_decel;
};

// Emergency braking has no fixed stop target.  This deceleration determines
// the predicted stop point of the direct emergency trajectory and must match
// the full-brake actuator assumption used by the controller.
struct EmergencyStopConfig {
    double decel;  // [m/s^2]
};

// 고주로/본선 합류 구간. Global Path가 이미 램프에서 본선으로 이어지는 형상을
// 가지므로 횡경로는 d=0을 유지하고, 본선 corridor의 선행/후행 차량 사이 시간·
// 거리 gap에 맞춰 종방향 진입 시점만 결정한다.
struct HighwayMergeCheckpoint {
    double conflict_s;           // [m] 충돌 가능 구간 진입점
    double clear_s;              // [m] 해당 충돌 구간 이탈점
};

struct HighwayMergeZone {
    std::string name;  // 비어 있으면 로더가 "highway_merge"로 채운다.
    double start_s;              // [m] 합류 정책 활성 시작점
    double completion_s;         // [m] 본선 중앙 정렬이 끝나는 점
    std::vector<HighwayMergeCheckpoint> checkpoints;
};

struct HighwayMergeConfig {
    bool enabled;
    std::vector<HighwayMergeZone> zones;
    double approach_distance;    // [m] 향후 속도 기반 사전 감지 확장 여유
    double stop_buffer;          // [m] unsafe일 때 conflict_s 전 정지 여유
    double target_corridor_half_width; // [m] 합류 전 평행 본선까지 포함
    double object_search_distance;     // [m] conflict 전후 차량 탐색 범위
    double min_heading_alignment; // cos(delta heading) 하한
    double min_front_gap;         // [m] 합류 완료 시 선행차 최소 거리
    double min_rear_gap;          // [m] 합류 완료 시 후행차 최소 거리
    double min_front_ttc;         // [s]
    double min_rear_ttc;          // [s]
    double max_wait_time;         // [s]
    double commit_distance;       // [m]
    int safe_confirm_cycles;
    double cross_speed_floor;     // [m/s]
    // [s] 진입시각 후보를 훑는 탐색 간격.
    double entry_search_dt;
    // [s] entry/clear 구간 전체를 따라 ego footprint 충돌 재검사를 하는
    // 시간 샘플링 간격.
    double collision_check_dt;
    // [m/s] MinimumTravelTime의 최소 목표/현재 속도 가정.
    double min_travel_speed;
    // [m/s^2] MinimumTravelTime의 최소 가속도 가정.
    double min_travel_accel;
};

#endif  // PLANNER_CONFIG_FRENET_CONFIG_HPP

