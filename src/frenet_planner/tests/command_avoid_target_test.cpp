// D6-1: 외부 AVOID의 authoritative 대상 확보 회귀.
//
// D5-1의 FOLLOW와 완전히 같은 원칙을 AVOID에 적용한다.
//
// legacy는 FindStaticAvoidanceTarget()으로
//   모든 객체 탐색 -> 같은 차선 -> 저속/정지 -> 가장 가까운 것 ->
//   좌/우 방향까지 자동 결정
// 을 한다. v2에서 그 권한은 전부 FSM에 있다
// (docs/d1-legacy-behavior-boundary.md 0.1절 B 분류, 6.4절).
//
//   selector: "무엇을 피할까?"                     -> 금지
//   resolver: "FSM이 21번을 피하라고 했다. 유효한가?" -> 이것만 한다
//
// ===== D6-1의 범위 =====
//
// 대상 확보(binding)와 방향 보존까지다. 좌/우 후보 생성과 안전성 판정은
// D6-2(PREPARE dual-output)다. 그래서 대상 확보에 성공해도 Plan()은
// UNSUPPORTED_BEHAVIOR로 실패한다 - 확보만 하고 중앙 유지 궤적을 내보내면
// FSM 입장에서 "회피 성공"으로 보이면서 실제로는 장애물 쪽으로 직진한다.
//
// ===== D6-1a에서 추가한 범위 =====
//
// optional target pose의 Frenet 투영이다. AVOID에서 has_target_pose=true는
// 합법인데(원본 메시지: "AVOID: 필요하면 회피 목표점") D6-1까지는 그 pose가
// 그냥 버려졌다.
//
// 공용 primitive ProjectPointToFrenet()을 새로 두고, 기존 STOP 전용
// ProjectPointToS()는 그 wrapper로 남긴다 - STOP config에 묶인 탐색창과
// 허용 횡거리를 AVOID가 끌어다 쓰면 STOP 튜닝이 AVOID 동작을 바꾼다.
//
// heading 검증(계약 3.4절 4항)은 node 책임이므로 여기서 다루지 않는다.
// ProjectPointToFrenet은 s/d만 돌려주고, "이 heading 차이가 허용 가능한가"는
// ROS 계층이 판단한다(ReferenceHeadingAt과 같은 분담).
//
// ===== D6-2a에서 추가한 범위 =====
//
// 1. 저속 AVOID resolver 우회 제거. D6-1에서는 확보가 명령 조립 지점에
//    있었고 그곳은 저속 fallback보다 아래여서, ego < 0.5 m/s이면 확보 자체가
//    일어나지 않고 Plan()이 성공을 반환했다(계약 12.7.2절 실측 표).
// 2. 평가할 목표 d 결정. target pose가 있으면 그 d, 없으면 legacy geometry
//    계산(clearance = 0.5*(자차폭+장애물폭) + safety_margin + 0.3).
//    LEFT/RIGHT는 한쪽만, NONE은 양쪽.
//
// 후보 생성과 안전성 판정은 여전히 D6-2b다.
//
// 계약: docs/fsm-frenet-interface-v2.md 12.7.1절(D6-1/D6-1a/D6-2a),
//       12.7.2절, 3.3절, 8.2절

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

using Reason = FrenetPlanner::PlanFailureReason;

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

struct Waypoints {
    std::vector<double> x;
    std::vector<double> y;
};

Waypoints LoadWaypoints(const std::string& path) {
    std::ifstream input(path);
    Waypoints waypoints;
    double x = 0.0, y = 0.0, ignored = 0.0;
    while (input >> x >> y >> ignored) {
        waypoints.x.push_back(x);
        waypoints.y.push_back(y);
    }
    return waypoints;
}

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

Pose2D PoseAt(const Waypoints& waypoints, double target_s) {
    double s = 0.0;
    std::size_t index = 1;
    for (; index < waypoints.x.size(); ++index) {
        s += std::hypot(waypoints.x[index] - waypoints.x[index - 1],
                        waypoints.y[index] - waypoints.y[index - 1]);
        if (s >= target_s) break;
    }
    if (index >= waypoints.x.size()) index = waypoints.x.size() - 1;
    Pose2D pose;
    pose.x = waypoints.x[index];
    pose.y = waypoints.y[index];
    pose.yaw = std::atan2(waypoints.y[index] - waypoints.y[index - 1],
                          waypoints.x[index] - waypoints.x[index - 1]);
    return pose;
}

CartesianState MakeEgo(const Pose2D& pose, double speed) {
    CartesianState ego{};
    ego.x = pose.x;
    ego.y = pose.y;
    ego.yaw = pose.yaw;
    ego.v = speed;
    return ego;
}

// 경로 위 target_s, 횡오프셋 lateral_d 지점에 놓인 객체.
ObjectInfo MakeObstacle(const Waypoints& waypoints, int id, double target_s,
                        double speed, double lateral_d = 0.0) {
    const Pose2D pose = PoseAt(waypoints, target_s);
    ObjectInfo object;
    object.id = id;
    // Frenet d는 진행 방향 좌측이 양수다. 경로 접선에 수직으로 밀어낸다.
    object.x = pose.x - lateral_d * std::sin(pose.yaw);
    object.y = pose.y + lateral_d * std::cos(pose.yaw);
    object.heading = pose.yaw;
    object.speed = speed;
    object.vx = speed * std::cos(pose.yaw);
    object.vy = speed * std::sin(pose.yaw);
    object.width = 2.0;
    object.length = 4.5;
    object.height = 1.5;
    return object;
}

std::string MakeLocalConfig(const std::string& source,
                            const std::string& waypoints,
                            const std::string& out_path) {
    std::ifstream in(source);
    if (!in) return {};
    std::ofstream out(out_path);
    if (!out) return {};
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("waypoint_file:", 0) == 0) {
            out << "waypoint_file: \"" << waypoints << "\"\n";
        } else {
            out << line << '\n';
        }
    }
    return out_path;
}

PlannerCommand MakeAvoidCommand(
    int target_object_id,
    LateralDirection direction = LateralDirection::NONE) {
    PlannerCommand command{};
    command.mode = AVOID;
    command.target_object_id = target_object_id;
    command.requested_direction = direction;
    return command;
}

// D6-1a: node가 target pose를 투영해 [A] 필드에 넣어 보낸 상태를 흉내낸다.
// core는 투영 자체를 하지 않는다 - 그것은 node 책임이다(계약 3.4절).
PlannerCommand MakeAvoidCommandWithTargetD(
    int target_object_id, double target_s, double target_d,
    LateralDirection direction = LateralDirection::NONE) {
    PlannerCommand command = MakeAvoidCommand(target_object_id, direction);
    command.has_avoid_target_d = true;
    command.avoid_target_s = target_s;
    command.avoid_target_d = target_d;
    return command;
}

// 경로 위 target_s에서 횡으로 lateral_d 만큼 벗어난 map 좌표.
// MakeObstacle과 같은 부호 규약을 쓴다(진행 방향 좌측이 양수).
void MapPointAt(const Waypoints& waypoints, double target_s, double lateral_d,
                double& out_x, double& out_y) {
    const Pose2D pose = PoseAt(waypoints, target_s);
    out_x = pose.x - lateral_d * std::sin(pose.yaw);
    out_y = pose.y + lateral_d * std::cos(pose.yaw);
}

// D6-1은 확보까지만 구현했으므로 성공 확보의 관측 가능한 결과는
// "UNSUPPORTED_BEHAVIOR로 거부되었지만 대상은 잡혀 있다"다.
bool BoundButNotPlanned(const FrenetPlanner& planner, bool plan_ok,
                        int expected_id) {
    return !plan_ok &&
           planner.last_failure_reason() == Reason::UNSUPPORTED_BEHAVIOR &&
           planner.active_avoid_target_id() == expected_id;
}

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "command_avoid_target_test_params.yaml");
    if (config.empty()) {
        std::cerr << "[FAIL] could not prepare test config\n";
        return 1;
    }

    const Waypoints waypoints = LoadWaypoints(waypoint_file);
    if (waypoints.x.size() < 100) {
        std::cerr << "[FAIL] waypoint load\n";
        return 1;
    }

    // ego는 회전교차로/저속 fallback과 무관한 평범한 구간에 둔다.
    constexpr double kEgoS = 200.0;
    constexpr double kEgoSpeed = 6.0;
    const CartesianState ego = MakeEgo(PoseAt(waypoints, kEgoS), kEgoSpeed);

    // ===== 1. 지정 대상만 확보한다 =====================================
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(BoundButNotPlanned(planner, ok, 21),
               "AVOID binds the designated obstacle (candidates are D6-2)");
        Expect(planner.last_mode() == AVOID,
               "AVOID reports AVOID as the active mode");
        Expect(planner.avoid_missing_cycles() == 0,
               "a freshly bound target has no missing cycles");
    }

    // ===== 2. 핵심: 더 가까운 장애물이 있어도 지정 대상을 유지 ==========
    //
    // legacy FindStaticAvoidanceTarget()은 가장 가까운 정적 장애물을 고른다.
    // resolver를 그 함수로 바꾸면 이 검사가 깨진다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 17, kEgoS + 12.0, 0.0),  // 더 가깝다
            MakeObstacle(waypoints, 21, kEgoS + 30.0, 0.0),  // FSM 지정
        };
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(BoundButNotPlanned(planner, ok, 21),
               "a nearer obstacle does not steal the designated target");
        Expect(planner.active_avoid_target_id() != 17,
               "the nearer obstacle 17 is never selected");
        // ID만으로는 부족하다. resolver가 요청 ID를 그대로 기록하면 조회가
        // nearest selection으로 회귀해도 확보 ID는 정상으로 보인다.
        // 그래서 확보한 **geometry**가 실제로 21의 것인지 확인한다 - 17의
        // 것이면 후보 생성이 엉뚱한 곳을 비켜간다.
        Expect(std::fabs(planner.avoid_target_s() - (kEgoS + 30.0)) < 3.0,
               "the bound geometry belongs to the designated obstacle");
        Expect(planner.avoid_target_s() > kEgoS + 20.0,
               "the bound target is not the one 12 m ahead");
    }

    // ===== 3. 첫 binding 실패는 즉시 LOST ==============================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 17, kEgoS + 20.0, 0.0)};
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "an unobserved target fails immediately with TARGET_OBJECT_LOST");
        Expect(planner.active_avoid_target_id() == -1,
               "a failed binding reports no active target");
        Expect(planner.last_mode() == AVOID,
               "a lost AVOID target still reports AVOID (no silent KEEP)");
    }

    // ===== 4. 확보 후 짧은 dropout은 유예된다 ==========================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> without_target{
            MakeObstacle(waypoints, 17, kEgoS + 60.0, 0.0)};
        CartesianPath path;

        planner.Plan(ego, with_target, MakeAvoidCommand(21), path);
        Expect(planner.active_avoid_target_id() == 21,
               "grace: the target is acquired first");

        // 유예 사이클 이내에는 확보 상태가 유지된다. 기본값은 5다.
        bool held = true;
        for (int cycle = 0; cycle < 5; ++cycle) {
            planner.Plan(ego, without_target, MakeAvoidCommand(21), path);
            if (planner.active_avoid_target_id() != 21) held = false;
        }
        Expect(held, "grace: the target survives 5 unobserved cycles");
        Expect(planner.avoid_missing_cycles() == 5,
               "grace: missing cycles accumulate");
        Expect(planner.last_failure_reason() == Reason::UNSUPPORTED_BEHAVIOR,
               "grace: a held target is not reported as lost");
    }

    // ===== 5. 유예 초과는 LOST =========================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> without_target{
            MakeObstacle(waypoints, 17, kEgoS + 60.0, 0.0)};
        CartesianPath path;

        planner.Plan(ego, with_target, MakeAvoidCommand(21), path);
        for (int cycle = 0; cycle < 5; ++cycle) {
            planner.Plan(ego, without_target, MakeAvoidCommand(21), path);
        }
        // 6번째 미관측에서 유예를 초과한다.
        planner.Plan(ego, without_target, MakeAvoidCommand(21), path);
        Expect(planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "exceeding the grace window reports TARGET_OBJECT_LOST");
        Expect(planner.active_avoid_target_id() == -1,
               "an exhausted grace window clears the target");
    }

    // ===== 6. 재관측하면 유예가 초기화된다 =============================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> without_target{
            MakeObstacle(waypoints, 17, kEgoS + 60.0, 0.0)};
        CartesianPath path;

        planner.Plan(ego, with_target, MakeAvoidCommand(21), path);
        planner.Plan(ego, without_target, MakeAvoidCommand(21), path);
        planner.Plan(ego, without_target, MakeAvoidCommand(21), path);
        Expect(planner.avoid_missing_cycles() == 2,
               "recovery: missing cycles accumulated before re-observation");
        planner.Plan(ego, with_target, MakeAvoidCommand(21), path);
        Expect(planner.avoid_missing_cycles() == 0,
               "recovery: re-observation resets the grace counter");
        Expect(planner.active_avoid_target_id() == 21,
               "recovery: the target is still bound");
    }

    // ===== 7. 관측되었지만 조건을 벗어나면 즉시 LOST ===================
    //
    // dropout이 아니므로 유예를 주지 않는다. 예측으로 메꿀 대상이 아니라
    // 조건 자체가 틀렸다(계약 8.2절).
    {
        // 7a. 후방 객체
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> behind{
            MakeObstacle(waypoints, 21, kEgoS - 20.0, 0.0)};
        CartesianPath path;
        const bool ok = planner.Plan(ego, behind, MakeAvoidCommand(21), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a target behind the ego is rejected immediately");
    }
    {
        // 7b. 다른 corridor. lane_width의 절반을 넘는 횡오프셋.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> other_lane{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0, 3.5)};
        CartesianPath path;
        const bool ok =
            planner.Plan(ego, other_lane, MakeAvoidCommand(21), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a target in another corridor is rejected immediately");
    }
    {
        // 7c. 탐색 범위 밖. 유효 탐색거리는 84.8 m다(아래 8번 참고).
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> far_away{
            MakeObstacle(waypoints, 21, kEgoS + 150.0, 0.0)};
        CartesianPath path;
        const bool ok = planner.Plan(ego, far_away, MakeAvoidCommand(21), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a target beyond the search range is rejected");
    }
    {
        // 7d. geometry가 0이면 회피 오프셋을 계산할 수 없다.
        FrenetPlanner planner;
        planner.Init(config);
        ObjectInfo degenerate = MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0);
        degenerate.width = 0.0;
        degenerate.length = 0.0;
        CartesianPath path;
        const bool ok =
            planner.Plan(ego, {degenerate}, MakeAvoidCommand(21), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a zero-geometry target is rejected");
    }

    // ===== 8. 속도로 대상을 버리지 않는다 ==============================
    //
    // legacy에는 fabs(s_dot) <= trigger_max_speed(0.5) 조건이 있다. 그 값의
    // config 설명 자체가 "이 속도 이하만 정지 장애물로 간주(그 이상은
    // Following이 처리)"이므로 AVOID/FOLLOW 분류 policy다. FSM이 AVOID를
    // 지정했으면 Frenet이 다시 분류하지 않는다(D5에서 min_leader_speed를
    // 제거한 것과 같은 이유).
    {
        for (double speed : {0.0, 0.3, 2.0, 6.0}) {
            FrenetPlanner planner;
            planner.Init(config);
            const std::vector<ObjectInfo> objects{
                MakeObstacle(waypoints, 21, kEgoS + 30.0, speed)};
            CartesianPath path;
            const bool ok =
                planner.Plan(ego, objects, MakeAvoidCommand(21), path);
            char name[128];
            std::snprintf(name, sizeof(name),
                          "a target moving at %.1f m/s is still bound", speed);
            Expect(BoundButNotPlanned(planner, ok, 21), name);
        }
    }

    // ===== 9. 탐색거리는 충돌 horizon까지 확장된다 =====================
    //
    // config 값은 60 m지만 충돌 lookahead는 10.0 * 8.0 + 4.5 + 0.3 = 84.8 m다.
    // 확장하지 않으면 80 m 앞의 장애물이 v2에서만 LOST가 되고, 회피 대상으로
    // 등록하기 전에 충돌 필터가 중앙 후보를 전부 제거해 속도가 급락한다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 80.0, 0.0)};
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(BoundButNotPlanned(planner, ok, 21),
               "a target 80 m ahead is inside the collision horizon");
    }

    // ===== 10. 방향이 보존된다 =========================================
    //
    // mode=AVOID 하나로는 LEFT/RIGHT/NONE을 구분할 수 없어, D6-1 이전에는
    // 세 명령이 core에서 전부 같은 PlannerCommand가 되었다. 그러면 FSM이
    // LEFT를 지정했는데 우측으로 회피하는 계약 위반이 조용히 일어난다.
    {
        struct Case {
            LateralDirection requested;
            const char* name;
        };
        const Case cases[] = {
            {LateralDirection::NONE, "NONE"},
            {LateralDirection::LEFT, "LEFT"},
            {LateralDirection::RIGHT, "RIGHT"},
        };
        for (const Case& c : cases) {
            FrenetPlanner planner;
            planner.Init(config);
            const std::vector<ObjectInfo> objects{
                MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
            CartesianPath path;
            planner.Plan(ego, objects, MakeAvoidCommand(21, c.requested), path);
            char name[128];
            std::snprintf(name, sizeof(name),
                          "requested direction %s reaches the planner core",
                          c.name);
            Expect(planner.avoid_requested_direction() == c.requested, name);
        }
        // 세 값이 서로 구별되어야 한다. 하나로 뭉개지면 위 검사는 통과하지만
        // 방향 정보가 사라진 것이다.
        Expect(LateralDirection::NONE != LateralDirection::LEFT &&
                   LateralDirection::LEFT != LateralDirection::RIGHT &&
                   LateralDirection::NONE != LateralDirection::RIGHT,
               "the three lateral directions are distinct values");
    }

    // ===== 11. 대상 없는 AVOID는 INVALID_COMMAND =======================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        PlannerCommand no_target = MakeAvoidCommand(-1);
        const bool ok = planner.Plan(ego, objects, no_target, path);
        Expect(!ok && planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "AVOID without a target is INVALID_COMMAND, not LOST");
    }

    // ===== 12. 다른 모드로 넘어가면 AVOID context가 비워진다 ===========
    //
    // D5-2에서 FOLLOW로 실측 확인한 것과 같은 구조다. reset하지 않으면
    //   AVOID target=21   -> 확보
    //   E-STOP            -> context 잔존
    //   AVOID target=21   -> 21이 없어도 GRACE
    // 가 되어 첫 binding 실패인데 유예를 받는다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> without_target{
            MakeObstacle(waypoints, 17, kEgoS + 60.0, 0.0)};
        CartesianPath path;

        planner.Plan(ego, with_target, MakeAvoidCommand(21), path);
        Expect(planner.active_avoid_target_id() == 21,
               "reset: the target is acquired first");

        PlannerCommand estop{};
        estop.mode = EMERGENCY;
        planner.Plan(ego, with_target, estop, path);
        Expect(planner.active_avoid_target_id() == -1,
               "reset: EMERGENCY clears the v2 AVOID context");

        // 이제 21이 관측되지 않으면 유예 없이 즉시 LOST여야 한다.
        const bool ok =
            planner.Plan(ego, without_target, MakeAvoidCommand(21), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "reset: a fresh AVOID gets no grace from the stale context");
    }
    {
        // KEEP으로 넘어가도 같다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, with_target, MakeAvoidCommand(21), path);
        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        planner.Plan(ego, with_target, keep, path);
        Expect(planner.active_avoid_target_id() == -1,
               "reset: KEEP clears the v2 AVOID context");
    }

    // ===== 13. legacy AVOID context는 건드리지 않는다 ==================
    //
    // v2_avoidance_와 avoidance_는 별도 인스턴스다. legacy fingerprint를
    // 유지하려면 v2 경로가 legacy context를 갱신하지 않아야 한다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;

        // v2 AVOID로 대상을 확보한다.
        planner.Plan(ego, with_target, MakeAvoidCommand(21), path);
        Expect(planner.active_avoid_target_id() == 21,
               "isolation: v2 AVOID acquired its target");

        // 곧바로 legacy 경로(3인자 Plan)를 돌린다. legacy는 자기 자신의
        // FindStaticAvoidanceTarget()으로 대상을 고르며, v2 context를
        // 읽지 않아야 한다. 여기서는 legacy가 정상적으로 궤적을 만드는지만
        // 확인한다 - byte 단위 동일성은 legacy_fingerprint가 담당한다.
        const bool legacy_ok = planner.Plan(ego, with_target, path);
        Expect(legacy_ok, "isolation: the legacy path still plans normally");
    }

    // ===== D6-1a: optional target pose 투영 =============================

    // ---- 14. 공용 primitive가 s와 d를 모두 돌려준다 --------------------
    {
        FrenetPlanner planner;
        planner.Init(config);
        // 경로 위 정확히 그 지점: d는 0에 가까워야 한다.
        double x = 0.0, y = 0.0;
        MapPointAt(waypoints, kEgoS + 30.0, 0.0, x, y);
        double s = 0.0, d = 0.0;
        Expect(planner.ProjectPointToFrenet(x, y, kEgoS, 100.0, 4.0, s, d),
               "projection of an on-path point succeeds");
        Expect(std::fabs(s - (kEgoS + 30.0)) < 3.0,
               "projection recovers the longitudinal coordinate");
        Expect(std::fabs(d) < 0.5,
               "an on-path point projects to d near zero");
    }
    {
        // 좌측으로 2 m 벗어난 점 -> d > 0. 부호 규약은 CartesianToFrenet
        // 식 (4)와 같아야 한다(진행 방향 좌측이 양수).
        FrenetPlanner planner;
        planner.Init(config);
        double x = 0.0, y = 0.0;
        MapPointAt(waypoints, kEgoS + 30.0, 2.0, x, y);
        double s = 0.0, d = 0.0;
        Expect(planner.ProjectPointToFrenet(x, y, kEgoS, 100.0, 4.0, s, d),
               "projection of a left-offset point succeeds");
        Expect(d > 1.0, "a point left of the path has positive d");
        Expect(std::fabs(d - 2.0) < 0.5, "the left offset magnitude is kept");
    }
    {
        // 우측으로 2 m -> d < 0.
        FrenetPlanner planner;
        planner.Init(config);
        double x = 0.0, y = 0.0;
        MapPointAt(waypoints, kEgoS + 30.0, -2.0, x, y);
        double s = 0.0, d = 0.0;
        Expect(planner.ProjectPointToFrenet(x, y, kEgoS, 100.0, 4.0, s, d),
               "projection of a right-offset point succeeds");
        Expect(d < -1.0, "a point right of the path has negative d");
        Expect(std::fabs(d + 2.0) < 0.5, "the right offset magnitude is kept");
    }
    {
        // 허용 횡거리를 넘는 점은 거부한다. 이것이 없으면 옆 도로의 점이
        // 이 경로의 회피 목표로 둔갑한다.
        FrenetPlanner planner;
        planner.Init(config);
        double x = 0.0, y = 0.0;
        MapPointAt(waypoints, kEgoS + 30.0, 12.0, x, y);
        double s = 0.0, d = 0.0;
        Expect(!planner.ProjectPointToFrenet(x, y, kEgoS, 100.0, 4.0, s, d),
               "a point far from the path is rejected");
        // 같은 점이라도 허용 거리를 넓히면 통과해야 한다 - 거부 이유가
        // 거리 검사임을 확인한다(다른 이유로 실패한 것이 아니다).
        Expect(planner.ProjectPointToFrenet(x, y, kEgoS, 100.0, 20.0, s, d),
               "the same point passes with a wider lateral allowance");
        Expect(d > 10.0, "the wide-allowance projection reports the large d");
    }
    {
        // 파라미터 방어. 창이나 허용 거리가 잘못되면 조용히 통과시키지
        // 않는다 - 그러면 투영 방어가 사실상 사라진다.
        FrenetPlanner planner;
        planner.Init(config);
        double x = 0.0, y = 0.0;
        MapPointAt(waypoints, kEgoS + 30.0, 0.0, x, y);
        double s = 0.0, d = 0.0;
        Expect(!planner.ProjectPointToFrenet(x, y, kEgoS, 0.0, 4.0, s, d),
               "a non-positive search window is rejected");
        Expect(!planner.ProjectPointToFrenet(x, y, kEgoS, -10.0, 4.0, s, d),
               "a negative search window is rejected");
        Expect(!planner.ProjectPointToFrenet(x, y, kEgoS, 100.0, 0.0, s, d),
               "a non-positive lateral allowance is rejected");
        const double nan_value = std::nan("");
        Expect(!planner.ProjectPointToFrenet(x, y, kEgoS, nan_value, 4.0, s, d),
               "a NaN search window is rejected");
        Expect(!planner.ProjectPointToFrenet(x, y, kEgoS, 100.0, nan_value,
                                             s, d),
               "a NaN lateral allowance is rejected");
        Expect(!planner.ProjectPointToFrenet(nan_value, y, kEgoS, 100.0, 4.0,
                                             s, d),
               "a NaN coordinate is rejected");
    }

    // ---- 15. STOP wrapper 동작이 보존된다 ------------------------------
    //
    // ProjectPointToS()는 이제 ProjectPointToFrenet()의 wrapper다. STOP은
    // D4에서 MORAI로 검증했으므로 동작이 바뀌면 안 된다.
    //
    // 검증 방법: **STOP과 AVOID의 허용 횡거리 사이에 있는 점**을 쓴다.
    //   STOP  max_lateral_offset          = 5.0 m (params.yaml)
    //   AVOID max_target_lateral_offset   = 4.0 m (node 기본값)
    // 4.0과 5.0 사이의 점은 STOP에서는 통과하고 AVOID에서는 거부된다.
    //
    // 이 구분이 없으면 wrapper가 AVOID 값을 쓰도록 바꿔도 테스트가 통과한다
    // (돌연변이 MUT-D로 실제 확인했다 - 처음에는 1.0 m 점을 써서 두 값
    // 모두 통과했고, 그래서 config 결합 회귀를 놓쳤다).
    {
        FrenetPlanner planner;
        planner.Init(config);
        double x = 0.0, y = 0.0;
        MapPointAt(waypoints, kEgoS + 40.0, 4.5, x, y);

        double wrapper_s = 0.0;
        const bool wrapper_ok = planner.ProjectPointToS(x, y, kEgoS, wrapper_s);
        Expect(wrapper_ok,
               "the STOP wrapper accepts a target 4.5 m off the path "
               "(within its own 5 m allowance)");

        // 같은 점을 AVOID 기본 허용거리로 투영하면 거부되어야 한다. 즉 두
        // 용도의 임계값이 실제로 다르다.
        double avoid_s = 0.0, avoid_d = 0.0;
        Expect(!planner.ProjectPointToFrenet(x, y, kEgoS, 100.0, 4.0,
                                             avoid_s, avoid_d),
               "the same target is rejected under the AVOID allowance");

        // wrapper가 STOP 값을 쓰고 있음을 s 일치로 확인한다.
        double direct_s = 0.0, direct_d = 0.0;
        const bool direct_ok = planner.ProjectPointToFrenet(
            x, y, kEgoS, 100.0, 5.0, direct_s, direct_d);
        Expect(direct_ok && wrapper_ok &&
                   std::fabs(wrapper_s - direct_s) < 1e-9,
               "the STOP wrapper matches the primitive with STOP parameters");
        Expect(direct_ok && std::fabs(direct_d - 4.5) < 0.5,
               "the boundary point really is ~4.5 m off the path");
    }
    {
        // STOP의 허용 횡거리(5 m)를 넘는 점은 wrapper가 거부해야 한다.
        FrenetPlanner planner;
        planner.Init(config);
        double x = 0.0, y = 0.0;
        MapPointAt(waypoints, kEgoS + 40.0, 12.0, x, y);
        double s = 0.0;
        Expect(!planner.ProjectPointToS(x, y, kEgoS, s),
               "the STOP wrapper still rejects a far target");
    }

    // ---- 15b. 탐색창이 실제로 결과를 바꾼다 ----------------------------
    //
    // 탐색창은 Newton 반복의 **초기 추정값** 탐색 범위다(FindClosestS).
    // 이 테스트 트랙은 자기 자신과 62 m 이내로 접근하지 않으므로, 정상적인
    // 목표점(ego 앞 수십 m)에서는 창 값이 결과를 바꾸지 않는다 - 어느 초기값
    // 에서 출발해도 같은 근으로 수렴한다.
    //
    // 창이 관측 가능해지는 조건은 hint가 목표에서 아주 멀 때다. 그때 창 값에
    // 따라 초기 추정값이 달라지고, Newton이 **다른 국소 근**으로 수렴할 수
    // 있다. 실측(hint=200, 목표 s≈900):
    //
    //   window=100  -> s=1156.67, d=-152.63   (엉뚱한 근)
    //   window=500  -> s= 611.98, d= 244.32   (엉뚱한 근)
    //   window=700  -> s= 895.42, d=  -0.13   (정답)
    //
    // 두 가지를 함께 고정한다:
    //   1. 창 값이 실제로 전달된다(무시하면 아래 차이가 사라진다)
    //   2. 횡거리 검사가 엉뚱한 근을 잡아낸다 - Newton의 국소 수렴이 이
    //      방어를 필요하게 만드는 이유다
    {
        FrenetPlanner planner;
        planner.Init(config);
        // 목표는 s≈900. hint는 200으로 700 m 떨어뜨린다.
        const Pose2D far_target = PoseAt(waypoints, 900.0);
        constexpr double kFarHint = 200.0;

        // 넉넉한 허용치로는 어떤 창이든 "성공"하지만 s가 달라진다.
        double s_narrow = 0.0, d_narrow = 0.0;
        double s_wide = 0.0, d_wide = 0.0;
        const bool narrow_ok = planner.ProjectPointToFrenet(
            far_target.x, far_target.y, kFarHint, 100.0, 1e9,
            s_narrow, d_narrow);
        const bool wide_ok = planner.ProjectPointToFrenet(
            far_target.x, far_target.y, kFarHint, 700.0, 1e9,
            s_wide, d_wide);
        Expect(narrow_ok && wide_ok,
               "window probe: both windows return a root");
        Expect(std::fabs(s_narrow - s_wide) > 100.0,
               "the search window actually changes the projection result");

        // 넓은 창은 정답으로 수렴한다.
        Expect(std::fabs(s_wide - 900.0) < 10.0 && std::fabs(d_wide) < 1.0,
               "a window that reaches the target converges to the true root");

        // 좁은 창은 엉뚱한 근으로 갔고, 횡거리 검사가 그것을 잡아낸다.
        Expect(std::fabs(d_narrow) > 50.0,
               "a too-narrow window converges to a distant wrong root");
        double rejected_s = 0.0, rejected_d = 0.0;
        Expect(!planner.ProjectPointToFrenet(far_target.x, far_target.y,
                                             kFarHint, 100.0, 5.0,
                                             rejected_s, rejected_d),
               "the lateral gate rejects the wrong root");
    }

    // ---- 16. 투영 결과가 core까지 보존된다 -----------------------------
    //
    // node가 투영해서 [A] 필드로 넣어 보낸 값을 core가 그대로 들고 있어야
    // D6-2가 쓸 수 있다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;

        // pose 없음: 기존 D6-1 동작 그대로.
        planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(planner.active_avoid_target_id() == 21,
               "pose-less AVOID still binds its target");
        Expect(!planner.avoid_has_requested_target_d(),
               "pose-less AVOID reports no requested target_d");
    }
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;

        // 좌측 회피 목표.
        const bool ok = planner.Plan(
            ego, objects,
            MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, 2.5,
                                        LateralDirection::LEFT),
            path);
        Expect(BoundButNotPlanned(planner, ok, 21),
               "AVOID with a target pose still binds its target");
        Expect(planner.avoid_has_requested_target_d(),
               "the projected target_d reaches the planner core");
        Expect(std::fabs(planner.avoid_requested_target_d() - 2.5) < 1e-9,
               "the projected target_d value is preserved exactly");
        Expect(std::fabs(planner.avoid_requested_target_s() -
                         (kEgoS + 30.0)) < 1e-9,
               "the projected target_s value is preserved exactly");
        Expect(planner.avoid_requested_direction() == LateralDirection::LEFT,
               "the requested direction survives alongside the target pose");
    }
    {
        // 우측 목표. 음수 d가 그대로 보존되어야 한다 - d는 음수가 유효하므로
        // 어딘가에서 -1.0 sentinel이나 fabs를 쓰면 여기서 깨진다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, -2.5,
                                                 LateralDirection::RIGHT),
                     path);
        Expect(planner.avoid_has_requested_target_d(),
               "a right-side target pose is preserved");
        Expect(planner.avoid_requested_target_d() < 0.0,
               "a right-side target keeps its negative sign");
        Expect(std::fabs(planner.avoid_requested_target_d() + 2.5) < 1e-9,
               "the right-side target_d value is exact");
    }
    {
        // target_d = 0.0도 유효한 값이다(차선 중앙). 플래그로만 구분한다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, 0.0), path);
        Expect(planner.avoid_has_requested_target_d(),
               "target_d = 0.0 is a valid requested target, not 'unset'");
        Expect(planner.avoid_requested_target_d() == 0.0,
               "target_d = 0.0 is preserved");
    }
    {
        // 다른 모드로 넘어가면 투영 결과도 함께 비워진다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, 2.5), path);
        Expect(planner.avoid_has_requested_target_d(),
               "reset: the target pose is present first");
        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        planner.Plan(ego, objects, keep, path);
        Expect(!planner.avoid_has_requested_target_d(),
               "reset: KEEP clears the projected target too");
    }
    {
        // 같은 대상에 pose가 사라진 명령이 오면 투영 결과도 사라진다.
        // 남아 있으면 FSM이 목표를 거둬들였는데 예전 목표로 회피한다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, 2.5), path);
        Expect(planner.avoid_has_requested_target_d(),
               "handover: the target pose is present first");
        planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(!planner.avoid_has_requested_target_d(),
               "handover: dropping the pose clears the projected target");
        Expect(planner.active_avoid_target_id() == 21,
               "handover: the object binding itself is unaffected");
    }

    // ===== D6-2a: 저속에서도 대상을 확보한다 ===========================
    //
    // 계약 12.7.2절의 실측 표를 회귀로 고정한다. D6-1 배치(확보가 저속
    // fallback 아래)로 되돌리면 0.4 / 0.3 / 0.0 행이 깨진다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};

        // kLowSpeedFallbackV = 0.5 m/s. 그 위/아래를 모두 훑는다.
        const double speeds[] = {6.0, 2.0, 1.0, 0.6, 0.49, 0.4, 0.3, 0.1, 0.0};
        bool all_bound = true;
        bool none_faked_success = true;
        for (double v : speeds) {
            FrenetPlanner low;
            low.Init(config);
            const CartesianState slow_ego =
                MakeEgo(PoseAt(waypoints, kEgoS), v);
            CartesianPath path;
            const bool ok = low.Plan(slow_ego, objects, MakeAvoidCommand(21),
                                     path);
            if (low.active_avoid_target_id() != 21) all_bound = false;
            // 후보 생성이 아직 없으므로 성공을 반환하면 안 된다. 저속만
            // 다르게 성공시키면 "회피 성공" 오보가 그대로 남는다.
            if (ok) none_faked_success = false;
        }
        Expect(all_bound,
               "the AVOID target is bound at every speed including 0 m/s");
        Expect(none_faked_success,
               "low speed never reports AVOID success before candidates exist");
    }
    {
        // 저속에서 확보에 실패하면 여전히 TARGET_OBJECT_LOST다. 저속이라고
        // 조용히 통과시키지 않는다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 17, kEgoS + 25.0, 0.0)};
        const CartesianState slow_ego = MakeEgo(PoseAt(waypoints, kEgoS), 0.2);
        CartesianPath path;
        const bool ok = planner.Plan(slow_ego, objects, MakeAvoidCommand(21),
                                     path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "an unobserved target still fails at low speed");
        Expect(planner.active_avoid_target_id() == -1,
               "a failed low-speed binding reports no target");
    }
    {
        // 저속 dropout 유예도 정상속도와 같게 동작해야 한다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> without_target{
            MakeObstacle(waypoints, 17, kEgoS + 60.0, 0.0)};
        const CartesianState slow_ego = MakeEgo(PoseAt(waypoints, kEgoS), 0.3);
        CartesianPath path;
        planner.Plan(slow_ego, with_target, MakeAvoidCommand(21), path);
        Expect(planner.active_avoid_target_id() == 21,
               "low speed: the target is acquired");
        planner.Plan(slow_ego, without_target, MakeAvoidCommand(21), path);
        Expect(planner.active_avoid_target_id() == 21 &&
                   planner.avoid_missing_cycles() == 1,
               "low speed: dropout grace works the same way");
    }
    {
        // E-STOP은 여전히 AVOID보다 먼저 처리된다. 확보를 위로 올렸어도
        // 긴급정지가 회피 확보에 가려지면 안 된다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(planner.active_avoid_target_id() == 21,
               "estop order: AVOID bound first");

        PlannerCommand estop{};
        estop.mode = EMERGENCY;
        const bool ok = planner.Plan(ego, objects, estop, path);
        Expect(ok && planner.last_mode() == EMERGENCY,
               "estop order: EMERGENCY still plans its decel path");
        Expect(planner.active_avoid_target_id() == -1,
               "estop order: EMERGENCY clears the AVOID context");
    }

    // ===== D6-2a: 평가할 목표 d 결정 ===================================
    {
        // NONE: 양쪽을 평가한다. 목표는 legacy geometry 식과 같아야 한다.
        //   clearance = 0.5*(vehicle_width + obstacle_width)
        //               + safety_margin + 0.3
        // params.yaml: vehicle width 1.9, safety_margin 0.3
        // 장애물 폭 2.0 -> clearance = 0.5*(1.9+2.0) + 0.3 + 0.3 = 2.55
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommand(21, LateralDirection::NONE), path);
        Expect(planner.avoid_target_count() == 2,
               "DIRECTION_NONE evaluates both sides");
        const double left = planner.avoid_target_candidate(0);
        const double right = planner.avoid_target_candidate(1);
        Expect(left > 0.0 && right < 0.0,
               "the two candidates straddle the obstacle");
        // 장애물이 d~0에 있으므로 목표는 대략 +-2.55다.
        Expect(std::fabs(left - 2.55) < 0.4,
               "the left target matches the legacy clearance formula");
        Expect(std::fabs(right + 2.55) < 0.4,
               "the right target matches the legacy clearance formula");
        Expect(!planner.avoid_target_from_pose(),
               "a pose-less AVOID computes its target from geometry");
    }
    {
        // LEFT: 왼쪽만.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommand(21, LateralDirection::LEFT), path);
        Expect(planner.avoid_target_count() == 1,
               "DIRECTION_LEFT evaluates exactly one side");
        Expect(planner.avoid_target_candidate(0) > 0.0,
               "DIRECTION_LEFT targets the left side");
    }
    {
        // RIGHT: 오른쪽만. legacy처럼 양쪽을 만들고 고르면 이 검사가 깨진다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommand(21, LateralDirection::RIGHT), path);
        Expect(planner.avoid_target_count() == 1,
               "DIRECTION_RIGHT evaluates exactly one side");
        Expect(planner.avoid_target_candidate(0) < 0.0,
               "DIRECTION_RIGHT targets the right side");
    }
    {
        // 목표는 장애물의 실제 횡위치를 반영한다. 고정 ±avoid_offset(5.0)이
        // 아니다 - 그러면 불필요하게 멀리 이동한다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> shifted{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0, 1.0)};
        CartesianPath path;
        planner.Plan(ego, shifted,
                     MakeAvoidCommand(21, LateralDirection::LEFT), path);
        const double target = planner.avoid_target_candidate(0);
        // 장애물이 d=+1.0에 있으므로 왼쪽 목표는 대략 1.0 + 2.55 = 3.55다.
        Expect(std::fabs(target - 3.55) < 0.5,
               "the target follows the obstacle's actual lateral position");
        Expect(std::fabs(target - 5.0) > 1.0,
               "the target is not the fixed avoid_offset");
    }
    {
        // 장애물 폭이 크면 clearance도 커진다.
        FrenetPlanner planner;
        planner.Init(config);
        ObjectInfo wide = MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0);
        wide.width = 3.0;
        CartesianPath path;
        planner.Plan(ego, {wide},
                     MakeAvoidCommand(21, LateralDirection::LEFT), path);
        const double wide_target = planner.avoid_target_candidate(0);

        FrenetPlanner narrow_planner;
        narrow_planner.Init(config);
        const std::vector<ObjectInfo> narrow{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};   // width 2.0
        narrow_planner.Plan(ego, narrow,
                            MakeAvoidCommand(21, LateralDirection::LEFT),
                            path);
        const double narrow_target = narrow_planner.avoid_target_candidate(0);
        Expect(wide_target > narrow_target + 0.4,
               "a wider obstacle needs a larger lateral target");
    }

    // ===== D6-2a: target pose가 목표를 대신한다 =========================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, 1.8,
                                                 LateralDirection::LEFT),
                     path);
        Expect(planner.avoid_target_count() == 1,
               "a target pose yields exactly one evaluated target");
        Expect(planner.avoid_target_from_pose(),
               "the target is reported as coming from the pose");
        Expect(std::fabs(planner.avoid_target_candidate(0) - 1.8) < 1e-9,
               "the pose d is used verbatim, not recomputed from geometry");
        Expect(!planner.avoid_direction_conflict(),
               "a left pose with DIRECTION_LEFT is consistent");
    }
    {
        // 방향 요청과 pose가 모순이면 표시하되 거부하지 않는다. clearance
        // 부족은 후보 생성/충돌 검사가 판정하고 결과는 WAITING이다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, -2.0,
                                                 LateralDirection::LEFT),
                     path);
        Expect(planner.avoid_direction_conflict(),
               "a right-side pose with DIRECTION_LEFT is flagged as a conflict");
        Expect(planner.last_failure_reason() != Reason::INVALID_COMMAND,
               "a direction conflict is not a command defect");
        Expect(planner.avoid_target_count() == 1 &&
                   std::fabs(planner.avoid_target_candidate(0) + 2.0) < 1e-9,
               "the conflicting pose is still evaluated (collision decides)");
    }
    {
        // NONE + pose: pose를 그대로 쓰고 모순 판정은 하지 않는다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, -2.2,
                                                 LateralDirection::NONE),
                     path);
        Expect(planner.avoid_target_count() == 1 &&
                   std::fabs(planner.avoid_target_candidate(0) + 2.2) < 1e-9,
               "DIRECTION_NONE with a pose uses the pose d as-is");
        Expect(!planner.avoid_direction_conflict(),
               "DIRECTION_NONE never reports a direction conflict");
    }
    {
        // 지정 대상 geometry가 목표 계산에 쓰인다. 더 가까운 장애물의
        // 위치로 계산하면 엉뚱한 곳을 비켜간다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 17, kEgoS + 12.0, 0.0, -1.5),  // 더 가깝다
            MakeObstacle(waypoints, 21, kEgoS + 30.0, 0.0, 1.0),   // FSM 지정
        };
        CartesianPath path;
        planner.Plan(ego, objects,
                     MakeAvoidCommand(21, LateralDirection::LEFT), path);
        const double target = planner.avoid_target_candidate(0);
        // 21이 d=+1.0이므로 목표는 ~3.55. 17(d=-1.5)로 계산하면 ~1.05다.
        Expect(std::fabs(target - 3.55) < 0.5,
               "the target uses the designated obstacle's geometry");
        Expect(target > 2.0,
               "the nearer obstacle's position does not drive the target");
    }
    {
        // 확보 실패 시 목표 목록도 비워진다.
        //
        // **먼저 확보에 성공해야 이 검사가 유효하다.** 처음부터 없는 대상으로
        // 시작하면 목록이 원래 비어 있어 "비웠는지"를 구분할 수 없다
        // (돌연변이 MUT-8로 실제 확인 - reset을 제거해도 통과했다).
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> without_target{
            MakeObstacle(waypoints, 17, kEgoS + 20.0, 0.0)};
        CartesianPath path;

        planner.Plan(ego, with_target,
                     MakeAvoidCommand(21, LateralDirection::LEFT), path);
        const int bound_count = planner.avoid_target_count();
        Expect(bound_count == 1,
               "lost-clears: a target is evaluated while bound");

        // 유예를 모두 소진시켜 LOST로 만든다.
        for (int cycle = 0; cycle < 7; ++cycle) {
            planner.Plan(ego, without_target, MakeAvoidCommand(21), path);
        }
        Expect(planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "lost-clears: the target is lost after the grace window");
        Expect(planner.avoid_target_count() == 0,
               "lost-clears: losing the target clears the evaluated targets");
    }
    {
        // 한 번도 확보하지 못한 경우에도 목록은 비어 있다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 17, kEgoS + 20.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(planner.avoid_target_count() == 0,
               "a never-bound target leaves no evaluated targets");
    }
    {
        // 다른 모드로 넘어가면 목표 목록도 비워진다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(planner.avoid_target_count() > 0,
               "reset: targets exist while AVOID is active");
        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        planner.Plan(ego, objects, keep, path);
        Expect(planner.avoid_target_count() == 0,
               "reset: KEEP clears the evaluated targets");
    }

    if (g_failures != 0) {
        std::cerr << g_failures << " AVOID target check(s) failed\n";
        return 1;
    }
    std::cout << "all AVOID target binding checks passed\n";
    return 0;
}
