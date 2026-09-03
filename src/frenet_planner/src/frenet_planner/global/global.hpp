// 기본적인 헤더 파일 및 planner 전역 입,출력 구조체 들어가는 곳

#ifndef PLANNER_GLOBAL_HPP
#define PLANNER_GLOBAL_HPP

#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>

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

// --- ObjectStatus 한 오브젝트 -------------
struct ObjectInfo {
    int id = -1;
    int type = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double width = 0.0;
    double length = 0.0;
    double height = 0.0;
    double yaw_rate = 0.0;  // [rad/s] planner가 연속 heading 관측으로 추정
};

// 외부(FSM) 또는 planner 내부 legacy 로직이 채우는 주행 명령.
// aggregate 구조를 유지하되(생성자 없음) 모든 필드에 안전한 기본값을 준다.
// PlannerCommand cmd; / PlannerCommand cmd{}; 두 형태 모두 미초기화 필드가
// 없도록 하는 것이 목적이다 (mode 미초기화로 인한 임의 모드 진입 방지).
struct PlannerCommand {
    // 1. 모드
    BehaviorState mode = LANE_KEEPING;

    // 2. 횡방향 목표
    int target_lane = 0;                    // Frenet d 기준: -1 우, 0 유지, 1 좌
    double avoidance_d_offset = 0.0;        // AVOID 모드 전용 연속값 [m]

    // 3. 종방향 목표
    bool has_target_speed = false;          // false이면 planner 기본 목표속도 사용
    double target_speed = 0.0;              // Velocity_keep, Lane_change [m/s]
    bool has_max_speed = false;             // true이면 최종 목표속도의 상한 적용
    double max_speed = 0.0;                 // 외부 command가 지정한 속도 상한 [m/s]
    // STOP 모드: 정지 목표 s 좌표 [m].
    // 0.0은 실제로 유효한 Frenet s이므로 "미지정"을 0으로 표현할 수 없다.
    // 기본값을 NaN으로 두어 std::isfinite(stop_position)로 지정 여부를 판별한다.
    double stop_position = std::numeric_limits<double>::quiet_NaN();

    // 4. FOLLOWING 전용
    int leader_id = -1;                     // coast 연장 충돌검사에서 제어 대상 식별
    double leader_s = 0.0;                  // 선두 차량 위치 [m]
    double leader_speed = 0.0;              // 선두 차량 속도 [m/s]
    double leader_accel = 0.0;              // 선두 차량 가속도
    double time_gap = 0.0;                  // taw
    double min_gap = 0.0;                   // D0 [m]
    double gap_gain = 0.0;                  // 간격 오차를 접근 속도로 변환하는 이득 [1/s]

    // 5. MERGE 전용(Sec.V-A) - 대상 차선의 두 차량 sa(앞)/sb(뒤) 등가속 예측
    double merge_sa_s = 0.0;
    int merge_sa_id = -1;
    double merge_sa_speed = 0.0;
    double merge_sa_accel = 0.0;
    double merge_sb_s = 0.0;
    int merge_sb_id = -1;
    double merge_sb_speed = 0.0;
    double merge_sb_accel = 0.0;
    double merge_target_d = 0.0;
    bool merge_gap_safe = false;
    double merge_conflict_s = 0.0;
    double merge_stop_s = 0.0;
    double merge_entry_time = 0.0;
    bool merge_committed = false;
    bool merge_crossing = false;
    MergeType merge_type = MergeType::NONE;
};

// =========================================================
// Ego 차량의 현재 Frenet 상태
// MakeQuintic/MakeQuartic의 Start 조건이 나오는 곳
// -> 후보 궤적을 만들기 위한 Start 조건
// =========================================================

struct FrenetState {
    // Longitudinal (along center line)
    double s;       // arc length
    double s_d;     // s_dot: speed
    double s_dd;    // s_ddot: accelration

    // Lateral (Perpendicular to center line)
    double d;       // Lateral Offset
    double d_d;     // d_dot: speed
    double d_dd;    // d_ddot: accelration
};

enum class RejectionReason {
    NONE = 0,
    CURVATURE,
    COLLISION,
};

// =========================================================
// 후보 궤적 하나 (Frenet 공간)
// PathGenerator가 생성, Cost/CollisionChecker가 읽음
// =========================================================

struct FrenetPath {
    // Time Sample Array
    std::vector<double> t;

    // Lateral
    std::vector<double> d;
    std::vector<double> d_d;
    std::vector<double> d_dd;

    // Longitudinal
    std::vector<double> s;
    std::vector<double> s_d;
    std::vector<double> s_dd;

    // Cost (우선 논문 기반 식으로 작성, C_tot = k_lat * C_lat + k_lon * C_lon)
    double cost_lat;
    double cost_lon;
    double cost_total;

    // --- cost.hpp가 비용을 조립하는 데 필요한 재료 ---
    // Jt = ∫(p''')^2 dt (Prop.1의 jerk 적분항). path_generator가 다항식 계수를
    // 갖고 있는 시점(생성 직후)에 닫힌 형태(해석적 공식)로 미리 계산해서 채워넣는다.
    // 샘플링된 d/s 배열만으로는 원본 다항식 계수를 복원할 수 없어서 사후에는 계산 불가.
    double jerk_cost_lat;   // Jt(d(t))
    double jerk_cost_lon;   // Jt(s(t))

    // 종료조건 오프셋 (Sec.V-A/B의 [s1-sd] = Δsi, [s_dot1-s_dot_d] = Δs_dot_i).
    // Following/Merging/Stopping 후보는 delta_s만, Velocity Keeping 후보는
    // delta_s_dot만 채우고 나머지는 0으로 둔다 (둘 다 0이면 자동으로 그 항이 사라짐).
    double delta_s;
    double delta_s_dot;

    // Available Flag
    bool valid;

    // 디버그 시각화용 탈락 정보. 결합 후보 이후의 필터가 갱신한다.
    RejectionReason rejection_reason = RejectionReason::NONE;
    int collision_object_id = -1;
    int collision_sample_index = -1;
};

// =========================================================
// Frenet -> Cartesian 변환 결과 (Tracking Controller 입력)
// 이 구조체가 MPC Controller로 넘어감.
// =========================================================

struct CartesianPath {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> kappa;
    std::vector<double> v;
    std::vector<double> a;
};

#endif
