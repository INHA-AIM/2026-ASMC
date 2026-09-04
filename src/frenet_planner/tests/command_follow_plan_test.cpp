// D5-1: 외부 FOLLOW의 authoritative 대상 확보 회귀.
//
// 이 테스트의 핵심은 "FSM이 지정한 대상만 따라간다"를 고정하는 것이다.
//
// legacy는 FindLeader()로 같은 차선의 가장 가까운 차량을 스스로 고르고,
// 주행 중 더 가까운 차량이 나타나면 자동으로 handoff한다. v2에서 그 policy를
// 쓰면 FSM은 자기가 지시한 대상을 추종 중이라고 오인한다
// (docs/d1-legacy-behavior-boundary.md 0.1절 B 분류).
//
//   selector: "누구를 따라갈까?"                  -> 금지
//   resolver: "FSM이 15번을 지정했다. 유효한가?"   -> 이것만 한다
//
// 계약: docs/fsm-frenet-interface-v2.md 12.5절(D5), 8.2절

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

// 경로 위 target_s 지점에 놓인 차량. speed는 진행 방향 속도다.
ObjectInfo MakeVehicle(const Waypoints& waypoints, int id, double target_s,
                       double speed) {
    const Pose2D pose = PoseAt(waypoints, target_s);
    ObjectInfo object;
    object.id = id;
    object.x = pose.x;
    object.y = pose.y;
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

PlannerCommand MakeFollowCommand(int target_object_id) {
    PlannerCommand command{};
    command.mode = FOLLOWING;
    command.target_object_id = target_object_id;
    // adapter가 custom gap이 꺼져 있을 때 넘기는 sentinel(계약 11.8절).
    command.time_gap = 0.0;
    command.min_gap = 0.0;
    return command;
}

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "command_follow_plan_test_params.yaml");
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
    constexpr double kEgoSpeed = 8.0;
    const CartesianState ego = MakeEgo(PoseAt(waypoints, kEgoS), kEgoSpeed);
    const std::vector<TrackedObstacle> no_collision_objects;

    // --- 지정 대상만 있는 경우 --------------------------------------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        const std::vector<ObjectInfo> objects{
            MakeVehicle(waypoints, 15, kEgoS + 30.0, 5.0)};
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(ok && planner.last_failure_reason() == Reason::NONE,
               "FOLLOW with the designated object succeeds");
        Expect(planner.last_mode() == FOLLOWING,
               "FOLLOW reports FOLLOWING as the active mode");
        Expect(planner.active_leader_id() == 15,
               "FOLLOW reports the designated leader");
        Expect(!path.x.empty(), "FOLLOW produces a trajectory");
    }

    // --- 핵심: 더 가까운 차량이 있어도 지정 대상을 유지한다 ---------------
    // legacy FindLeader()는 가장 가까운 차량을 고른다. 그 policy를 타면
    // active_leader_id가 17이 되어 FSM은 15를 추종 중이라고 오인한다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        const std::vector<ObjectInfo> objects{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 5.0),
            MakeVehicle(waypoints, 17, kEgoS + 15.0, 5.0),  // 더 가까움
        };
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeFollowCommand(15), path);
        // 확보된 leader는 반드시 15다. 궤적 생성 성공 여부와는 별개 축이다 -
        // 17이 앞을 막고 있으면 충돌 필터가 후보를 전부 제거할 수 있고, 그것이
        // 의도된 동작이다(자동 handoff 대신 COLLISION_BLOCKED로 FSM에 알린다).
        Expect(planner.active_leader_id() == 15,
               "a closer vehicle does not steal the FOLLOW target");
        Expect(planner.active_leader_id() != 17,
               "the nearest vehicle is not selected automatically");
        // 실패했다면 그 사유가 대상 소실이 아니라 충돌 차단이어야 한다.
        // TARGET_OBJECT_LOST가 나오면 15를 놓친 것이므로 계약 위반이다.
        Expect(ok || planner.last_failure_reason() == Reason::COLLISION_BLOCKED,
               "blocking traffic is reported as COLLISION_BLOCKED, not a lost target");
    }

    // --- 주행 중 더 가까운 차량이 나타나도 handoff하지 않는다 -------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        // 먼저 15만 있는 상태로 확보한다.
        const std::vector<ObjectInfo> initial{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 5.0)};
        planner.Plan(ego, initial, MakeFollowCommand(15), path);
        Expect(planner.active_leader_id() == 15, "leader 15 is acquired first");

        // 이후 더 가까운 17이 등장한다.
        const std::vector<ObjectInfo> with_closer{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 5.0),
            MakeVehicle(waypoints, 17, kEgoS + 12.0, 5.0),
        };
        for (int cycle = 0; cycle < 5; ++cycle) {
            planner.Plan(ego, with_closer, MakeFollowCommand(15), path);
        }
        // 여러 사이클을 돌려도 대상이 바뀌지 않는다. legacy handoff는
        // leader_switch_margin(3m)만 넘으면 즉시 인계했다.
        Expect(planner.active_leader_id() == 15,
               "no automatic handoff to a closer vehicle mid-drive");
        Expect(planner.last_failure_reason() != Reason::TARGET_OBJECT_LOST,
               "blocking traffic does not make the designated target lost");
    }

    // --- 대상 변경은 새 명령으로만 -----------------------------------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        const std::vector<ObjectInfo> objects{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 5.0),
            MakeVehicle(waypoints, 17, kEgoS + 20.0, 5.0),
        };
        CartesianPath path;
        planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(planner.active_leader_id() == 15, "following 15");
        // FSM이 대상을 17로 바꾼 명령을 보낸다.
        planner.Plan(ego, objects, MakeFollowCommand(17), path);
        Expect(planner.active_leader_id() == 17,
               "the target changes only when the command says so");
    }

    // --- 대상이 없으면 TARGET_OBJECT_LOST ---------------------------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        // 지정한 15는 없고 다른 차량만 있다. 그 차량으로 대체하지 않는다.
        const std::vector<ObjectInfo> objects{
            MakeVehicle(waypoints, 17, kEgoS + 20.0, 5.0)};
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a missing target is TARGET_OBJECT_LOST on the first binding");
        Expect(planner.last_mode() == FOLLOWING,
               "a lost target still reports FOLLOWING (no automatic KEEP)");
        Expect(planner.active_leader_id() == -1,
               "a lost target reports no active leader");
    }

    // --- 관측되었지만 조건을 벗어난 경우 ------------------------------------
    // dropout이 아니므로 유예를 주지 않는다(계약 8.2절).
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        // ego 뒤에 있는 차량.
        const std::vector<ObjectInfo> behind{
            MakeVehicle(waypoints, 15, kEgoS - 20.0, 5.0)};
        Expect(!planner.Plan(ego, behind, MakeFollowCommand(15), path) &&
                   planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a target behind ego is TARGET_OBJECT_LOST");
    }
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        // 정지한 차량은 FOLLOW 대상이 아니다(min_leader_speed 미달).
        const std::vector<ObjectInfo> stopped{
            MakeVehicle(waypoints, 15, kEgoS + 30.0, 0.0)};
        Expect(!planner.Plan(ego, stopped, MakeFollowCommand(15), path) &&
                   planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a stopped target is TARGET_OBJECT_LOST");
    }
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        // 탐색 범위를 크게 벗어난 차량.
        const std::vector<ObjectInfo> far{
            MakeVehicle(waypoints, 15, kEgoS + 300.0, 5.0)};
        Expect(!planner.Plan(ego, far, MakeFollowCommand(15), path) &&
                   planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a target beyond the search range is TARGET_OBJECT_LOST");
    }

    // --- 탐색거리는 legacy와 동일하게 확장된다 -----------------------------
    // config의 max_leader_search_s(60m)만 쓰면 유지 한계가 70m가 되어,
    // legacy(충돌 horizon 84.8m로 확장 -> 94.8m)에서는 유효한 80m 앞의
    // leader가 v2에서만 TARGET_OBJECT_LOST가 된다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        const std::vector<ObjectInfo> objects{
            MakeVehicle(waypoints, 15, kEgoS + 80.0, 5.0)};
        planner.Plan(ego, objects, MakeFollowCommand(15), path);
        // 궤적 생성은 후보 문제로 실패할 수도 있으므로 확보 자체를 본다.
        Expect(planner.last_failure_reason() != Reason::TARGET_OBJECT_LOST,
               "a target at 80 m is not lost (search range matches legacy)");
        Expect(planner.active_leader_id() == 15,
               "a target at 80 m stays the active leader");
    }

    // --- 명령 검증: 대상 미지정 --------------------------------------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        PlannerCommand command = MakeFollowCommand(-1);
        CartesianPath path;
        const std::vector<ObjectInfo> objects{
            MakeVehicle(waypoints, 15, kEgoS + 30.0, 5.0)};
        Expect(!planner.Plan(ego, objects, command, path) &&
                   planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "FOLLOW without a target id is INVALID_COMMAND");
    }

    // --- gap 기본값 / custom 값 --------------------------------------------
    // 계약 11.8절: adapter는 custom이 꺼져 있으면 0.0 sentinel을 넘기고,
    // config 기본값 적용은 소비 지점(여기)에서 한다. 두 경우가 서로 다른
    // 궤적을 만들어야 sentinel이 실제로 소비된 것이다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        const std::vector<ObjectInfo> objects{
            MakeVehicle(waypoints, 15, kEgoS + 30.0, 5.0)};

        CartesianPath default_path;
        Expect(planner.Plan(ego, objects, MakeFollowCommand(15), default_path),
               "FOLLOW with default gap succeeds");

        FrenetPlanner custom_planner;
        if (!custom_planner.Init(config)) return 1;
        PlannerCommand custom = MakeFollowCommand(15);
        custom.time_gap = 3.0;   // config 기본값 1.0보다 크게
        custom.min_gap = 20.0;   // config 기본값 5.0보다 크게
        CartesianPath custom_path;
        Expect(custom_planner.Plan(ego, objects, custom, custom_path),
               "FOLLOW with a custom gap succeeds");

        // 더 큰 gap을 요구하면 더 보수적인(느린) 궤적이 나와야 한다.
        const bool default_faster =
            !default_path.v.empty() && !custom_path.v.empty() &&
            default_path.v.back() > custom_path.v.back();
        Expect(default_faster,
               "a larger custom gap yields a more conservative trajectory");
    }

    // --- 다른 모드로 전환하면 v2 FOLLOW 상태가 비워진다 --------------------
    // 남겨두면 다음 FOLLOW가 낡은 leader 예측으로 시작한다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        const std::vector<ObjectInfo> objects{
            MakeVehicle(waypoints, 15, kEgoS + 30.0, 5.0)};
        planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(planner.active_leader_id() == 15, "leader is active before KEEP");

        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        planner.Plan(ego, objects, keep, path);
        Expect(planner.active_leader_id() == -1,
               "switching to KEEP clears the v2 follow state");
    }

    std::remove(config.c_str());
    if (g_failures != 0) {
        std::cerr << g_failures << " follow plan check(s) failed\n";
        return 1;
    }
    std::cout << "all follow plan checks passed\n";
    return 0;
}
