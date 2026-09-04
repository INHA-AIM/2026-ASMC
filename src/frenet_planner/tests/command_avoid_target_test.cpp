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
// 계약: docs/fsm-frenet-interface-v2.md 12.7.1절(D6-1), 3.3절, 8.2절

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

    if (g_failures != 0) {
        std::cerr << g_failures << " AVOID target check(s) failed\n";
        return 1;
    }
    std::cout << "all AVOID target binding checks passed\n";
    return 0;
}
