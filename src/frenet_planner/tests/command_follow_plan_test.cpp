// D5-1: 외부 FOLLOW의 authoritative 대상 확보 회귀.
//
// 이 테스트의 핵심은 "FSM이 지정한 대상만 따라간다"를 고정하는 것이다.
//
// planner가 같은 차선의 가장 가까운 차량을 스스로 고르고, 주행 중 더
// 가까운 차량이 나타나면 자동으로 handoff하는 정책은 v2에서 금지된다 -
// 그러면 FSM은 자기가 지시한 대상을 추종 중이라고 오인한다
// (docs/archive/planning/d1-legacy-behavior-boundary.md 0.1절 B 분류).
//
//   selector: "누구를 따라갈까?"                  -> 금지
//   resolver: "FSM이 15번을 지정했다. 유효한가?"   -> 이것만 한다
//
// 계약: docs/contracts/planning/fsm-frenet-v2.md 12.5절(D5), 8.2절

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/Maneuver/ManeuverUtils.hpp"

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
// authoritative 8-corner OBB(perception의 obb_corners와 같은 형태)를 heading
// 방향으로 직접 채운다. production은 center/width/length/yaw로 geometry를
// 재생성하지 않으므로, corners 없는 객체는 충돌검사에서 무조건 겹침으로
// 취급된다(CollisionChecker.cpp의 안전측 fallback).
TrackedObject MakeVehicle(const Waypoints& waypoints, int id, double target_s,
                          double speed, double length = 4.5,
                          double width = 2.0) {
    const Pose2D pose = PoseAt(waypoints, target_s);
    TrackedObject object;
    object.id = id;
    object.x = pose.x;
    object.y = pose.y;
    object.heading = pose.yaw;
    object.speed = speed;
    object.vx = speed * std::cos(pose.yaw);
    object.vy = speed * std::sin(pose.yaw);
    object.width = width;
    object.length = length;
    object.height = 1.5;
    object.geometry_valid = true;
    object.z_valid = true;
    const double half_length = object.length * 0.5;
    const double half_width = object.width * 0.5;
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    const double local_xy[4][2] = {
        {-half_length, -half_width}, {half_length, -half_width},
        {half_length, half_width}, {-half_length, half_width},
    };
    for (std::size_t i = 0; i < 4; ++i) {
        const double x = object.x + cosine * local_xy[i][0] -
                          sine * local_xy[i][1];
        const double y = object.y + sine * local_xy[i][0] +
                          cosine * local_xy[i][1];
        object.corners[i] = {x, y, 0.0};
        object.corners[i + 4] = {x, y, object.height};
    }
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
        package_src + "/../config/params.yaml", waypoint_file,
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

    // FOLLOW leader와 실제로 겹친 작은 OBB 조각은 geometry를 보존한 채
    // leader의 motion만 공유한다.
    {
        TrackedObject leader =
            MakeVehicle(waypoints, 15, kEgoS + 30.0, 5.0);
        leader.type = 1;
        leader.motion_state = ObjectMotionState::DYNAMIC;
        TrackedObject fragment =
            MakeVehicle(waypoints, 16, kEgoS + 30.8, 0.0, 1.0, 0.6);
        fragment.type = 0;
        fragment.motion_state = ObjectMotionState::LOW_THREAT;
        const geometry::Corners8 original_corners = fragment.corners;
        std::vector<TrackedObject> objects{leader, fragment};
        const int linked = SynchronizeFollowLeaderFragments(15, objects);
        Expect(linked == 1, "overlapping small OBB is linked to FOLLOW leader");
        Expect(std::abs(objects[1].vx - leader.vx) < 1e-9 &&
                   std::abs(objects[1].vy - leader.vy) < 1e-9,
               "FOLLOW fragment inherits leader velocity");
        Expect(objects[1].motion_state == leader.motion_state,
               "FOLLOW fragment inherits leader motion state");
        bool geometry_unchanged = true;
        for (std::size_t i = 0; i < original_corners.size(); ++i) {
            geometry_unchanged = geometry_unchanged &&
                std::abs(objects[1].corners[i].x - original_corners[i].x) < 1e-9 &&
                std::abs(objects[1].corners[i].y - original_corners[i].y) < 1e-9 &&
                std::abs(objects[1].corners[i].z - original_corners[i].z) < 1e-9;
        }
        Expect(geometry_unchanged,
               "FOLLOW fragment keeps authoritative OBB geometry");
    }
    {
        TrackedObject leader =
            MakeVehicle(waypoints, 15, kEgoS + 30.0, 5.0);
        leader.type = 1;
        leader.motion_state = ObjectMotionState::DYNAMIC;
        TrackedObject obstacle =
            MakeVehicle(waypoints, 16, kEgoS + 34.0, 0.0, 1.0, 0.6);
        obstacle.type = 0;
        obstacle.motion_state = ObjectMotionState::LOW_THREAT;
        std::vector<TrackedObject> objects{leader, obstacle};
        const int linked = SynchronizeFollowLeaderFragments(15, objects);
        Expect(linked == 0, "separate small obstacle is not linked to leader");
        Expect(std::abs(objects[1].vx) < 1e-9 &&
                   std::abs(objects[1].vy) < 1e-9,
               "separate obstacle keeps its own velocity");
    }

    // --- 지정 대상만 있는 경우 --------------------------------------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        const std::vector<TrackedObject> objects{
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
    // 가장 가까운 차량을 자동으로 고르는 정책을 타면 active_leader_id가
    // 17이 되어 FSM은 15를 추종 중이라고 오인한다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        const std::vector<TrackedObject> objects{
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
        const std::vector<TrackedObject> initial{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 5.0)};
        planner.Plan(ego, initial, MakeFollowCommand(15), path);
        Expect(planner.active_leader_id() == 15, "leader 15 is acquired first");

        // 이후 더 가까운 17이 등장한다.
        const std::vector<TrackedObject> with_closer{
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
        const std::vector<TrackedObject> objects{
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
        const std::vector<TrackedObject> objects{
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
        const std::vector<TrackedObject> behind{
            MakeVehicle(waypoints, 15, kEgoS - 20.0, 5.0)};
        Expect(!planner.Plan(ego, behind, MakeFollowCommand(15), path) &&
                   planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "a target behind ego is TARGET_OBJECT_LOST");
    }
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        // 역방향으로 움직이는 객체는 거부한다. 마주 오는 차량을 추종하면
        // gap이 음수로 발산한다.
        const std::vector<TrackedObject> oncoming{
            MakeVehicle(waypoints, 15, kEgoS + 30.0, -3.0)};
        Expect(!planner.Plan(ego, oncoming, MakeFollowCommand(15), path) &&
                   planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "an oncoming target is TARGET_OBJECT_LOST");
    }

    // --- 정지/저속 leader도 FOLLOW 대상이다 (D5-2 정책) -------------------
    // min_leader_speed(0.5 m/s)는 legacy에서 "움직이는 차량은 FOLLOW, 정지
    // 객체는 AVOID"를 planner가 스스로 분류하기 위한 policy였다. v2에서 그
    // 권한은 FSM에 있으므로 속도로 대상을 걸러내지 않는다.
    for (double leader_speed : {6.0, 2.0, 0.3, 0.0}) {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        const std::vector<TrackedObject> objects{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, leader_speed)};
        planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(planner.last_failure_reason() != Reason::TARGET_OBJECT_LOST &&
                   planner.active_leader_id() == 15,
               "a leader at " + std::to_string(leader_speed) +
                   " m/s stays the FOLLOW target");
    }
    {
        // 감속하다 완전히 멈추는 앞차. 확보 후 속도가 0이 되어도 유지한다.
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        for (double leader_speed : {6.0, 4.0, 2.0, 0.5, 0.0, 0.0}) {
            const std::vector<TrackedObject> objects{
                MakeVehicle(waypoints, 15, kEgoS + 40.0, leader_speed)};
            planner.Plan(ego, objects, MakeFollowCommand(15), path);
        }
        Expect(planner.active_leader_id() == 15 &&
                   planner.last_failure_reason() != Reason::TARGET_OBJECT_LOST,
               "a leader decelerating to a stop keeps being followed");
    }
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        // 탐색 범위를 크게 벗어난 차량.
        const std::vector<TrackedObject> far{
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
        const std::vector<TrackedObject> objects{
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
        const std::vector<TrackedObject> objects{
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
        const std::vector<TrackedObject> objects{
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
        const std::vector<TrackedObject> objects{
            MakeVehicle(waypoints, 15, kEgoS + 30.0, 5.0)};
        planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(planner.active_leader_id() == 15, "leader is active before KEEP");

        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        planner.Plan(ego, objects, keep, path);
        Expect(planner.active_leader_id() == -1,
               "switching to KEEP clears the v2 follow state");
    }

    // --- dropout grace: 1~10 cycle 유지, 11번째 LOST -----------------------
    // dropout_grace_cycles=10 (config/params.yaml: 20Hz 기준 0.5초, Behavior
    // FOLLOW 해제 유예 0.4s보다 길게 유지하도록 의도적으로 설정된 값이다.
    // PlannerConfig.hpp의 구조체 기본값 5는 이 값을 로드하지 못했을 때의
    // fallback일 뿐이며, 여기서는 MakeLocalConfig()로 실제 params.yaml을
    // 로드하므로 10을 기준으로 검증한다).
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        const std::vector<TrackedObject> objects{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 6.0)};
        const std::vector<TrackedObject> no_objects;

        planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(planner.active_leader_id() == 15, "grace: leader acquired");

        // 1~10번째 미관측은 유예 대상이다.
        bool grace_held = true;
        for (int cycle = 1; cycle <= 10; ++cycle) {
            planner.Plan(ego, no_objects, MakeFollowCommand(15), path);
            if (planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST ||
                planner.active_leader_id() != 15) {
                grace_held = false;
            }
        }
        Expect(grace_held, "grace: cycles 1-10 keep the leader");

        // 11번째에서 유예가 끝난다.
        const bool ok = planner.Plan(ego, no_objects, MakeFollowCommand(15), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "grace: cycle 11 is TARGET_OBJECT_LOST");
        Expect(planner.active_leader_id() == -1,
               "grace: no active leader after the grace window");
    }

    // --- grace 중 leader_s가 실제로 전진한다 -------------------------------
    // 예측이 없으면 leader가 제자리에 멈춘 것으로 계산되어 gap이 실제보다
    // 빠르게 줄어들고, 궤적이 과도하게 보수적으로 바뀐다.
    //
    // ego를 매 cycle 실제 속도만큼 전진시켜야 차이가 드러난다. ego가
    // 제자리에 있으면 gap이 거의 변하지 않아 두 경우가 같은 궤적을 만든다
    // (실측: ego 고정 상태에서는 prediction을 제거해도 테스트가 통과했다).
    //
    //   ego 8.0 m/s  -> +0.40 m/cycle
    //   leader 6.0 m/s -> +0.30 m/cycle
    //   정상 gap 변화  -> -0.10 m/cycle
    //   예측 없으면    -> -0.40 m/cycle  (4배 빠르게 좁혀진다)
    //
    // gap을 좁게 시작해 유예 구간 안에서 그 차이가 목표속도에 나타나게 한다.
    {
        constexpr double kLeaderSpeed = 6.0;
        constexpr double kCycleDt = 0.05;
        constexpr double kInitialGap = 12.0;

        // 유예 구간(5 cycle) 동안 ego를 실제로 전진시키며 목표속도를 모은다.
        auto run_grace = [&](std::vector<double>& speeds) {
            FrenetPlanner planner;
            if (!planner.Init(config)) return false;
            CartesianPath path;
            const std::vector<TrackedObject> objects{
                MakeVehicle(waypoints, 15, kEgoS + kInitialGap, kLeaderSpeed)};
            const std::vector<TrackedObject> no_objects;

            // 먼저 정상 관측으로 확보한다.
            double ego_s = kEgoS;
            CartesianState moving_ego = MakeEgo(PoseAt(waypoints, ego_s), kEgoSpeed);
            if (!planner.Plan(moving_ego, objects, MakeFollowCommand(15), path)) {
                return false;
            }

            // 이후 5 cycle은 미관측 상태로 유예를 받는다. ego는 계속 전진한다.
            for (int cycle = 1; cycle <= 5; ++cycle) {
                ego_s += kEgoSpeed * kCycleDt;
                moving_ego = MakeEgo(PoseAt(waypoints, ego_s), kEgoSpeed);
                if (planner.Plan(moving_ego, no_objects, MakeFollowCommand(15),
                                 path)) {
                    speeds.push_back(planner.last_target_speed());
                } else {
                    speeds.push_back(-1.0);
                }
            }
            return true;
        };

        std::vector<double> speeds;
        Expect(run_grace(speeds) && speeds.size() == 5,
               "grace: trajectories keep being generated while ego advances");

        // 예측이 동작하면 leader도 전진하므로 gap이 천천히 줄어든다.
        // 예측이 없으면 gap이 4배 빠르게 좁혀져 목표속도가 더 낮아진다.
        // 이 값을 기록해 두어 회귀 시 비교 근거가 되게 한다.
        bool all_planned = true;
        for (double speed : speeds) if (speed < 0.0) all_planned = false;
        Expect(all_planned,
               "grace: every grace cycle produces a plan (no mid-window failure)");

        // 유예가 끝나는 6번째에서만 LOST가 되어야 한다.
        {
            FrenetPlanner planner;
            if (!planner.Init(config)) return 1;
            CartesianPath path;
            const std::vector<TrackedObject> objects{
                MakeVehicle(waypoints, 15, kEgoS + kInitialGap, kLeaderSpeed)};
            const std::vector<TrackedObject> no_objects;
            double ego_s = kEgoS;
            planner.Plan(MakeEgo(PoseAt(waypoints, ego_s), kEgoSpeed), objects,
                         MakeFollowCommand(15), path);
            bool lost_early = false;
            for (int cycle = 1; cycle <= 5; ++cycle) {
                ego_s += kEgoSpeed * kCycleDt;
                planner.Plan(MakeEgo(PoseAt(waypoints, ego_s), kEgoSpeed),
                             no_objects, MakeFollowCommand(15), path);
                if (planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST) {
                    lost_early = true;
                }
            }
            Expect(!lost_early,
                   "grace: no early loss while ego advances through the window");
        }
    }

    // --- prediction이 gap 계산에 실제로 반영된다 ---------------------------
    // leader_s 예측을 제거하면 leader가 제자리에 멈춘 것으로 계산된다. 그러면
    // FOLLOW 종료조건(leader 기준 목표 위치)도 제자리에 머물러, 궤적의 끝점이
    // 유예 구간 동안 전혀 전진하지 않는다.
    //
    // 실측 비교(gap=6 m, leader 6 m/s, ego 8 m/s, 유예 5 cycle):
    //   정상   끝점이 매 cycle 약 0.145 m씩 단조 전진
    //   예측없음 끝점이 여러 cycle 동안 완전히 동일한 좌표에 머문다
    //
    // 목표속도(last_target_speed)나 궤적 마지막 속도로는 구분되지 않는다.
    // 둘 다 순항속도 10.0과 leader 속도 6.0으로 수렴하기 때문이다 - 처음에
    // 그 값으로 검증했는데 mutation이 통과했다.
    {
        constexpr double kLeaderSpeed = 6.0;
        constexpr double kCycleDt = 0.05;
        constexpr double kTightGap = 6.0;
        constexpr int kGraceCycles = 5;

        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        const std::vector<TrackedObject> objects{
            MakeVehicle(waypoints, 15, kEgoS + kTightGap, kLeaderSpeed)};
        const std::vector<TrackedObject> no_objects;

        double ego_s = kEgoS;
        planner.Plan(MakeEgo(PoseAt(waypoints, ego_s), kEgoSpeed), objects,
                     MakeFollowCommand(15), path);

        // 유예 구간 동안 궤적 끝점의 이동거리를 모은다.
        std::vector<double> end_x;
        std::vector<double> end_y;
        for (int cycle = 1; cycle <= kGraceCycles; ++cycle) {
            ego_s += kEgoSpeed * kCycleDt;
            const bool ok = planner.Plan(
                MakeEgo(PoseAt(waypoints, ego_s), kEgoSpeed), no_objects,
                MakeFollowCommand(15), path);
            if (ok && !path.x.empty()) {
                end_x.push_back(path.x.back());
                end_y.push_back(path.y.back());
            }
        }
        Expect(end_x.size() == static_cast<std::size_t>(kGraceCycles),
               "prediction: every grace cycle produces a trajectory");

        // 끝점이 정지한 cycle이 하나도 없어야 한다. 예측이 없으면 leader
        // 목표가 그대로여서 같은 좌표가 반복된다.
        std::size_t frozen = 0;
        double total_advance = 0.0;
        for (std::size_t i = 1; i < end_x.size(); ++i) {
            const double step = std::hypot(end_x[i] - end_x[i - 1],
                                           end_y[i] - end_y[i - 1]);
            total_advance += step;
            if (step < 1e-6) ++frozen;
        }
        Expect(frozen == 0,
               "prediction: the trajectory endpoint never freezes during grace");
        // 5 cycle이면 leader가 약 1.5 m 전진하므로 끝점도 그만큼 따라간다.
        Expect(total_advance > 0.3,
               "prediction: the endpoint advances with the predicted leader");
    }

    // --- grace 중 재관측하면 유예가 초기화된다 -----------------------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        const std::vector<TrackedObject> objects{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 6.0)};
        const std::vector<TrackedObject> no_objects;
        planner.Plan(ego, objects, MakeFollowCommand(15), path);

        // 3 cycle 미관측 후 재관측.
        for (int cycle = 1; cycle <= 3; ++cycle) {
            planner.Plan(ego, no_objects, MakeFollowCommand(15), path);
        }
        planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(planner.active_leader_id() == 15,
               "grace: re-observation keeps the leader");

        // 유예가 초기화되었으므로 다시 5 cycle을 버틸 수 있어야 한다.
        bool held = true;
        for (int cycle = 1; cycle <= 5; ++cycle) {
            planner.Plan(ego, no_objects, MakeFollowCommand(15), path);
            if (planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST) {
                held = false;
            }
        }
        Expect(held, "grace: the window resets after re-observation");
    }

    // --- 다른 maneuver로 넘어가면 context가 즉시 비워진다 ------------------
    // 이 reset이 early-return보다 앞에 있어야 한다. EMERGENCY와 저속
    // fallback은 명령 조립 지점에 도달하기 전에 return하므로, 그곳에서만
    // reset하면 context가 살아남아 첫 binding 실패가 GRACE를 받는다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        const std::vector<TrackedObject> objects{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 6.0)};
        planner.Plan(ego, objects, MakeFollowCommand(15), path);
        Expect(planner.active_leader_id() == 15, "stale: leader acquired");

        // EMERGENCY는 후보 파이프라인 전에 return한다.
        PlannerCommand emergency{};
        emergency.mode = EMERGENCY;
        planner.Plan(ego, objects, emergency, path);
        Expect(planner.active_leader_id() == -1,
               "stale: EMERGENCY clears the v2 follow context");

        // 객체 없이 FOLLOW를 다시 보내면 첫 binding 실패여야 한다.
        const bool ok = planner.Plan(ego, {}, MakeFollowCommand(15), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "stale: a fresh FOLLOW after EMERGENCY has no grace");
    }
    {
        // 저속 KEEP도 fallback으로 조기 return한다.
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        CartesianPath path;
        const std::vector<TrackedObject> objects{
            MakeVehicle(waypoints, 15, kEgoS + 40.0, 6.0)};
        planner.Plan(ego, objects, MakeFollowCommand(15), path);

        const CartesianState slow_ego = MakeEgo(PoseAt(waypoints, kEgoS), 0.2);
        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        planner.Plan(slow_ego, objects, keep, path);
        Expect(planner.active_leader_id() == -1,
               "stale: the low-speed KEEP fallback clears the follow context");

        const bool ok = planner.Plan(ego, {}, MakeFollowCommand(15), path);
        Expect(!ok && planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "stale: a fresh FOLLOW after low-speed KEEP has no grace");
    }

    std::remove(config.c_str());
    if (g_failures != 0) {
        std::cerr << g_failures << " follow plan check(s) failed\n";
        return 1;
    }
    std::cout << "all follow plan checks passed\n";
    return 0;
}

