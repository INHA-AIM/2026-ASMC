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
#include "frenet_planner/obstacle/legacy_object_adapter.hpp"
#include "ros/avoid_commit_coordinator.hpp"

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

double TrackLength(const Waypoints& waypoints) {
    double length = 0.0;
    for (std::size_t i = 1; i < waypoints.x.size(); ++i) {
        length += std::hypot(waypoints.x[i] - waypoints.x[i - 1],
                             waypoints.y[i] - waypoints.y[i - 1]);
    }
    return length;
}

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

CartesianState MakeEgoAt(const Waypoints& waypoints, double target_s,
                         double lateral_d, double speed) {
    const Pose2D pose = PoseAt(waypoints, target_s);
    CartesianState ego = MakeEgo(pose, speed);
    ego.x -= lateral_d * std::sin(pose.yaw);
    ego.y += lateral_d * std::cos(pose.yaw);
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

asmc_msgs::BehaviorCommand MakeAvoidTransactionCommand(uint32_t id) {
    asmc_msgs::BehaviorCommand command;
    command.header.frame_id = "map";
    command.command_id = id;
    command.maneuver_type = asmc_msgs::BehaviorCommand::AVOID;
    command.execution_request = asmc_msgs::BehaviorCommand::PREPARE;
    command.direction = asmc_msgs::BehaviorCommand::DIRECTION_LEFT;
    command.merge_type = asmc_msgs::BehaviorCommand::MERGE_NONE;
    command.desired_speed_mps = -1.0;
    command.primary_object_id = 21;
    command.preferred_front_object_id = -1;
    command.preferred_rear_object_id = -1;
    command.desired_time_gap_s = -1.0;
    command.minimum_gap_m = -1.0;
    command.valid_until = ros::Time(1001.0);
    return command;
}

frenet_planner_ros::CommandExecutionManager MakeAvoidManagerReady(
    uint32_t command_id) {
    frenet_planner_ros::CommandExecutionManager manager;
    // Test harness에서 handshake 전이를 직접 검증하기 위한 설정이다. runtime의
    // production gate를 여는 SetImplementedManeuver 호출과는 무관하다.
    manager.SetImplementedManeuver(asmc_msgs::BehaviorCommand::AVOID, true);
    manager.Accept(MakeAvoidTransactionCommand(command_id));
    manager.NotifyPlanSucceeded();
    return manager;
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

// D6-2b: Cartesian 궤적 종단점의 Frenet d. candidate와 maintenance가 실제로
// 다른 궤적인지 숫자로 비교하는 데 쓴다.
double EndpointD(const FrenetPlanner& planner, const CartesianPath& path,
                 double s_hint) {
    if (path.x.empty()) return std::nan("");
    double s = 0.0, d = 0.0;
    if (!planner.ProjectPointToFrenet(path.x.back(), path.y.back(), s_hint,
                                      200.0, 1e9, s, d)) {
        return std::nan("");
    }
    return d;
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
        // D6-2a에서는 목표 목록에 남지만, D6-2b는 이 모순을 후보 차단
        // 사유로 쓴다 - INVALID_COMMAND로 거부하지는 않되 feasibility가
        // false가 되어 WAITING이다(계약 3.3.4절).
        Expect(planner.avoid_target_count() == 1 &&
                   std::fabs(planner.avoid_target_candidate(0) + 2.0) < 1e-9,
               "the conflicting pose stays in the evaluated list");
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

    // ===== D6-2b: PREPARE dual-output ==================================
    //
    // 계약 5.2절의 핵심. AVOID 후보 평가와 MPC에 보낼 유지 궤적을 한 번의
    // 전처리에서 만들고, 두 결과를 절대 같은 것으로 취급하지 않는다.
    using Prepare = FrenetPlanner::AvoidPrepareResult;

    // PrepareAvoid는 collision geometry가 obstacles와 1:1이어야 한다.
    auto boxes_for = [](const std::vector<ObjectInfo>& objects) {
        return MakeLegacyTrackedObstacles(objects);
    };

    {
        // ---- 방향별 평가 대상 ------------------------------------------
        // LEFT/RIGHT는 한쪽만, NONE은 양쪽. 실측 clearance는 2.55 m다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);

        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommand(21, LateralDirection::LEFT), maintenance, result);
        Expect(ok, "PREPARE LEFT produces a maintenance trajectory");
        Expect(result.candidate_valid, "PREPARE LEFT finds a safe candidate");
        Expect(result.selected_target_d > 0.0,
               "PREPARE LEFT selects a left-side target");
        Expect(!result.direction_conflict && !result.target_behind_ego,
               "PREPARE LEFT reports no request conflict");
    }
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommand(21, LateralDirection::RIGHT), maintenance, result);
        Expect(ok && result.candidate_valid,
               "PREPARE RIGHT finds a safe candidate");
        Expect(result.selected_target_d < 0.0,
               "PREPARE RIGHT selects a right-side target");
    }
    {
        // NONE은 양쪽을 평가하므로 생성 후보 수가 한쪽보다 많다.
        FrenetPlanner both;
        both.Init(config);
        FrenetPlanner one_side;
        one_side.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);

        CartesianPath m1, m2;
        Prepare r_both, r_left;
        both.PrepareAvoid(ego, objects, boxes,
                          MakeAvoidCommand(21, LateralDirection::NONE), m1,
                          r_both);
        one_side.PrepareAvoid(ego, objects, boxes,
                              MakeAvoidCommand(21, LateralDirection::LEFT), m2,
                              r_left);
        Expect(r_both.candidate_valid && r_left.candidate_valid,
               "both DIRECTION_NONE and DIRECTION_LEFT find candidates");
        Expect(r_both.generated_candidate_count >
                   r_left.generated_candidate_count,
               "DIRECTION_NONE generates candidates for both sides");
    }

    {
        // ---- **핵심**: candidate와 maintenance는 다른 궤적이다 ---------
        //
        // 실측: candidate 종단 d ~ +2.55, maintenance 종단 d ~ 0.0.
        // 실수로 out_path = result.candidate_path를 하면 이 검사가 깨진다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommand(21, LateralDirection::LEFT), maintenance, result);
        Expect(ok && result.candidate_valid,
               "dual-output: PREPARE succeeded with a candidate");

        const double candidate_d =
            EndpointD(planner, result.candidate_path, kEgoS);
        const double maintenance_d = EndpointD(planner, maintenance, kEgoS);
        Expect(std::isfinite(candidate_d) && std::isfinite(maintenance_d),
               "dual-output: both endpoints project onto the reference line");
        Expect(candidate_d > 2.0,
               "dual-output: the AVOID candidate really shifts laterally");
        Expect(std::fabs(maintenance_d) < 0.6,
               "dual-output: the published maintenance path stays centred");
        Expect(candidate_d - maintenance_d > 1.5,
               "dual-output: candidate and maintenance are different paths");
        // 궤적 자체가 다른 객체여야 한다. 같은 점 배열이면 발행이 새는 것이다.
        Expect(maintenance.x.size() != result.candidate_path.x.size() ||
                   std::fabs(maintenance.x.back() -
                             result.candidate_path.x.back()) > 0.1 ||
                   std::fabs(maintenance.y.back() -
                             result.candidate_path.y.back()) > 0.1,
               "dual-output: the maintenance path is not the candidate path");
    }
    {
        // selected_target_d는 **선택된 후보의 실제 종단 d**여야 한다.
        // 요청값을 그대로 보고하면 FSM이 회피 폭을 잘못 안다 - lateral 격자가
        // target_d +-lateral_target_tolerance(0.5)이므로 실제로 벗어난다.
        // 실측: 요청 +2.50 -> 실제 종단 +3.00.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, 2.5,
                                        LateralDirection::LEFT),
            maintenance, result);
        Expect(result.candidate_valid,
               "pose target: PREPARE finds a candidate");
        const double candidate_d =
            EndpointD(planner, result.candidate_path, kEgoS);
        Expect(std::isfinite(candidate_d),
               "pose target: the candidate endpoint projects");
        Expect(std::fabs(result.selected_target_d - candidate_d) < 0.2,
               "selected_target_d matches the candidate's actual endpoint");
    }

    {
        // ---- 방향 모순은 candidate_valid=false, terminal 실패 아님 ------
        //
        // 충돌 검사에만 맡기면 안 된다. LEFT 요청에 오른쪽 -2.5 m pose는
        // 물리적으로 안전할 수 있어 충돌 검사가 CLEAR를 준다. 그대로 READY로
        // 올라가면 FSM은 LEFT를 명령했는데 Frenet은 RIGHT 후보를 준비한
        // 계약 위반이 된다(계약 3.3.4절).
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, -2.5,
                                        LateralDirection::LEFT),
            maintenance, result);
        Expect(result.direction_conflict,
               "conflict: a right pose with DIRECTION_LEFT is a conflict");
        Expect(!result.candidate_valid,
               "conflict: a direction conflict makes the candidate invalid");
        Expect(ok, "conflict: PREPARE still produces a maintenance path");
        Expect(planner.last_failure_reason() == Reason::NONE,
               "conflict: it is not a planner failure (WAITING, not FAILED)");
        Expect(result.candidate_path.x.empty(),
               "conflict: no candidate path is produced");
        Expect(result.blocking_object_id == -1,
               "conflict: no collision object is blamed");
    }
    {
        // 반대 방향도 같다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, 2.5,
                                        LateralDirection::RIGHT),
            maintenance, result);
        Expect(result.direction_conflict && !result.candidate_valid,
               "conflict: a left pose with DIRECTION_RIGHT is also blocked");
    }
    {
        // NONE + pose는 모순이 없다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, -2.5,
                                        LateralDirection::NONE),
            maintenance, result);
        Expect(!result.direction_conflict,
               "DIRECTION_NONE with any pose is never a conflict");
        Expect(result.candidate_valid,
               "DIRECTION_NONE with a right pose still finds a candidate");
    }

    {
        // ---- target pose가 ego 후방이면 후보를 만들지 않는다 -----------
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommandWithTargetD(21, kEgoS - 40.0, 2.5,
                                        LateralDirection::LEFT),
            maintenance, result);
        Expect(result.target_behind_ego,
               "behind: a target pose behind the ego is detected");
        Expect(!result.candidate_valid,
               "behind: a rear target pose yields no candidate");
        Expect(ok && planner.last_failure_reason() == Reason::NONE,
               "behind: it is WAITING, not a planner failure");
    }
    {
        // 전방 목표는 정상이다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, 2.5,
                                        LateralDirection::LEFT),
            maintenance, result);
        Expect(!result.target_behind_ego,
               "behind: a forward target pose is accepted");
    }

    {
        // ---- 후보가 막히면 candidate_valid=false, 여전히 maintenance -----
        //
        // 장애물이 8 m 앞이면 회피 후보가 전부 충돌로 탈락한다(실측).
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 8.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommand(21, LateralDirection::NONE), maintenance, result);
        Expect(!result.candidate_valid,
               "blocked: a very close obstacle blocks every candidate");
        Expect(result.generated_candidate_count > 0,
               "blocked: candidates were actually generated and rejected");
        Expect(result.blocking_object_id == 21,
               "blocked: the blocking object is reported");
        Expect(ok, "blocked: PREPARE still produces a maintenance path");
        Expect(planner.last_failure_reason() == Reason::NONE,
               "blocked: a blocked candidate is not a terminal failure");
        Expect(!maintenance.x.empty(),
               "blocked: the maintenance path is non-empty");
    }

    {
        // ---- 저속 PREPARE -----------------------------------------------
        //
        // 확보는 하고, 후보는 아직 안 나오며, maintenance는 충돌 검사를
        // 통과한 crawl이다. 무조건 정지시키면
        //   0.3 -> WAITING -> stop -> 0 -> WAITING -> ...
        // 자기잠금이 된다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        const CartesianState slow_ego = MakeEgo(PoseAt(waypoints, kEgoS), 0.3);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            slow_ego, objects, boxes,
            MakeAvoidCommand(21, LateralDirection::NONE), maintenance, result);
        Expect(ok, "low speed: PREPARE produces a maintenance path");
        Expect(planner.active_avoid_target_id() == 21,
               "low speed: the target is still bound");
        Expect(result.low_speed_context,
               "low speed: the low-speed context is reported");
        Expect(!result.candidate_valid,
               "low speed: no AVOID candidate yet");
        // low_speed_context는 상황 정보이지 상태가 아니다. 정상 속도에서는
        // false여야 하고, 상태 판단은 candidate_valid만으로 한다.
        FrenetPlanner fast;
        fast.Init(config);
        CartesianPath fast_maintenance;
        Prepare fast_result;
        fast.PrepareAvoid(ego, objects, boxes,
                          MakeAvoidCommand(21, LateralDirection::NONE),
                          fast_maintenance, fast_result);
        Expect(!fast_result.low_speed_context,
               "low speed: the context flag is false at normal speed");
        Expect(fast_result.candidate_valid,
               "low speed: normal speed finds a candidate (same scene)");
    }
    {
        // ***** low_speed_context가 상태 판단에 쓰이지 않는다는 증거 *****
        //
        // 이전 이름(low_speed_waiting)이 위험했던 이유가 여기 있다. 저속에서
        // **후보가 실제로 성공하는** 상황이 존재한다.
        //
        // 실측: ego 0.3 m/s + 장애물 60 m 전방 -> candidate_valid=true 이면서
        // low_speed_context=true. 이 값을 "waiting"으로 읽어 상태를 정하면
        // 유효한 후보를 두고 WAITING을 보고하게 된다.
        //
        // 장애물이 25 m일 때는 후보가 막히므로 이 조합이 드러나지 않았다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> far_objects{
            MakeObstacle(waypoints, 21, kEgoS + 60.0, 0.0)};
        const auto far_boxes = boxes_for(far_objects);
        const CartesianState slow_ego = MakeEgo(PoseAt(waypoints, kEgoS), 0.3);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            slow_ego, far_objects, far_boxes,
            MakeAvoidCommand(21, LateralDirection::NONE), maintenance, result);

        Expect(ok, "low-speed valid: PREPARE succeeds");
        Expect(result.low_speed_context,
               "low-speed valid: the low-speed context is set");
        Expect(result.candidate_valid,
               "low-speed valid: a candidate IS found at 0.3 m/s with a far "
               "obstacle");
        // 이 두 값이 동시에 true인 것이 정상이다. low_speed_context를 후보
        // 차단이나 상태 판단에 쓰면 이 케이스가 깨진다.
        Expect(result.low_speed_context && result.candidate_valid,
               "low-speed valid: low speed and a valid candidate coexist");
        Expect(result.selected_target_d != 0.0,
               "low-speed valid: a lateral target was selected");
        Expect(result.generated_candidate_count > 0,
               "low speed: candidates are still attempted, not skipped");
        Expect(planner.last_failure_reason() == Reason::NONE,
               "low speed: PREPARE is not a failure");
        // crawl은 전진해야 한다. 정지 hold면 한 점이다.
        Expect(maintenance.x.size() > 5,
               "low speed: a clear crawl keeps moving (not a stop hold)");
        double max_v = 0.0;
        for (double v : maintenance.v) max_v = std::max(max_v, v);
        Expect(max_v > 0.0, "low speed: the crawl has non-zero speed");
    }
    {
        // 저속 + 장애물이 가까우면 crawl이 충돌로 막혀 감속/정지로 내려간다.
        // 실측: 30~9 m -> 21점 crawl / 7 m -> 3점 감속 / 5 m 이하 -> 1점 hold.
        FrenetPlanner far_planner;
        far_planner.Init(config);
        const std::vector<ObjectInfo> far_objects{
            MakeObstacle(waypoints, 21, kEgoS + 30.0, 0.0)};
        const CartesianState slow_ego = MakeEgo(PoseAt(waypoints, kEgoS), 0.3);
        CartesianPath far_maintenance;
        Prepare far_result;
        far_planner.PrepareAvoid(slow_ego, far_objects,
                                 boxes_for(far_objects),
                                 MakeAvoidCommand(21), far_maintenance,
                                 far_result);

        FrenetPlanner near_planner;
        near_planner.Init(config);
        const std::vector<ObjectInfo> near_objects{
            MakeObstacle(waypoints, 21, kEgoS + 4.0, 0.0)};
        CartesianPath near_maintenance;
        Prepare near_result;
        const bool near_ok = near_planner.PrepareAvoid(
            slow_ego, near_objects, boxes_for(near_objects),
            MakeAvoidCommand(21), near_maintenance, near_result);

        Expect(far_maintenance.x.size() > near_maintenance.x.size(),
               "low speed: a blocked crawl falls back to a shorter path");
        Expect(near_ok && !near_maintenance.x.empty(),
               "low speed: the fallback still produces a path");
        // 정지 계열이므로 마지막 속도가 0이어야 한다.
        Expect(!near_maintenance.v.empty() &&
                   std::fabs(near_maintenance.v.back()) < 1e-6,
               "low speed: the blocked-crawl fallback ends stopped");
    }

    {
        // ---- PREPARE는 target_d를 lock하지 않는다 -----------------------
        //
        // NONE이면 사이클마다 좌/우가 바뀔 수 있고, FSM이 EXECUTE를 주지
        // 않았으므로 그것이 정상이다. lock은 D6-3의 COMMIT에서 한다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(ego, objects, boxes,
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        Expect(result.candidate_valid && result.selected_target_d > 2.0,
               "no-lock: PREPARE selected a lateral target");
        Expect(planner.avoid_target_count() > 0,
               "no-lock: the evaluated target list is populated");
        // v2_avoidance_.target_d는 아직 0이어야 한다. 접근자가 없으므로
        // 진행률이 계산되지 않는 것으로 확인한다 - D6-3에서 lock을 붙일 때
        // 이 검사를 명시적 getter로 바꾼다.
        Expect(planner.avoid_requested_direction() == LateralDirection::LEFT,
               "no-lock: the requested direction is preserved");
    }

    {
        // ---- PrepareAvoid의 입력 방어 -----------------------------------
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;

        // AVOID가 아닌 모드로 PREPARE를 호출하면 거부한다.
        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        Expect(!planner.PrepareAvoid(ego, objects, boxes, keep, maintenance,
                                     result),
               "PrepareAvoid rejects a non-AVOID command");
        Expect(planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "a non-AVOID PREPARE is INVALID_COMMAND");

        // 대상 없는 AVOID도 거부한다.
        Expect(!planner.PrepareAvoid(ego, objects, boxes,
                                     MakeAvoidCommand(-1), maintenance,
                                     result),
               "PrepareAvoid rejects AVOID without a target");

        // collision geometry 개수가 어긋나면 거부한다.
        const std::vector<TrackedObstacle> empty_boxes;
        Expect(!planner.PrepareAvoid(ego, objects, empty_boxes,
                                     MakeAvoidCommand(21), maintenance,
                                     result),
               "PrepareAvoid rejects mismatched collision geometry");
    }
    {
        // 확보 실패는 PREPARE에서도 TARGET_OBJECT_LOST다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 17, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        Expect(!planner.PrepareAvoid(ego, objects, boxes,
                                     MakeAvoidCommand(21), maintenance,
                                     result),
               "PREPARE with an unobserved target fails");
        Expect(planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "PREPARE reports TARGET_OBJECT_LOST for a missing target");
        Expect(!result.candidate_valid,
               "a lost target yields no candidate");
    }
    {
        // NORMAL purpose(Plan)로 AVOID는 여전히 실행 경로가 없다.
        // D6-3에서 열린다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeAvoidCommand(21), path);
        Expect(!ok && planner.last_failure_reason() ==
                          Reason::UNSUPPORTED_BEHAVIOR,
               "NORMAL AVOID is still unsupported (execution is D6-3)");
        Expect(planner.active_avoid_target_id() == 21,
               "NORMAL AVOID still binds its target");
    }

    // ===== D6-2c: Feedback 매핑 =========================================
    //
    // node의 AVOID PREPARE 배선이 AvoidPrepareResult를 FrenetFeedback으로
    // 어떻게 옮기는지 고정한다.
    //
    // ===== 왜 여기서 검증하는가 =====
    //
    // node의 SetImplementedManeuver(AVOID)가 아직 열리지 않았으므로
    // command_manager_.Accept()가 planner 호출 전에 FAILED로 끝낸다. 즉
    // ROS 경로로는 PREPARE 블록이 실행되지 않는다 - 그것은 D6-4에서 gate를
    // 열 때 처음 검증된다.
    //
    // 그래서 여기서는 node가 쓰는 것과 **같은 매핑 규칙**을 core 결과에
    // 적용해 값이 맞는지 본다. ROS 왕복은 D6-4에서 덮는다.
    {
        // 매핑 규칙(node의 avoid_fields 조립과 동일):
        //   plan_valid       = candidate_valid
        //   target_d_valid   = candidate_valid
        //   target_d         = candidate_valid ? selected_target_d : 0.0
        //   selected_cost    = candidate_valid ? selected_cost : -1.0
        //   applied_speed    = maintenance 종단 속도
        //   result_code      = 항상 RESULT_NONE (COLLISION_BLOCKED 아님)
        auto map_feedback = [](const Prepare& r, const CartesianPath& maint,
                               bool& plan_valid, bool& target_d_valid,
                               double& target_d, double& selected_cost,
                               double& applied_speed) {
            plan_valid = r.candidate_valid;
            target_d_valid = r.candidate_valid;
            target_d = r.candidate_valid ? r.selected_target_d : 0.0;
            selected_cost = r.candidate_valid ? r.selected_cost : -1.0;
            applied_speed = maint.v.empty() ? -1.0 : maint.v.back();
        };

        // --- 후보 성공 ---------------------------------------------------
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(ego, objects, boxes,
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);

        bool plan_valid = false, target_d_valid = false;
        double target_d = 0.0, selected_cost = 0.0, applied_speed = 0.0;
        map_feedback(result, maintenance, plan_valid, target_d_valid, target_d,
                     selected_cost, applied_speed);

        Expect(plan_valid, "feedback: a valid candidate reports plan_valid");
        Expect(target_d_valid,
               "feedback: a valid candidate reports target_d_valid");
        Expect(target_d > 2.0,
               "feedback: target_d carries the selected lateral goal");
        Expect(selected_cost > 0.0,
               "feedback: a valid candidate reports a real cost");
        Expect(result.blocking_object_id == -1,
               "feedback: a valid candidate blames no blocking object");
        // applied_speed는 maintenance의 실제 속도다. last_target_speed_가
        // 아니다 - 그것은 maintenance 결정 전의 순항 목표속도다.
        Expect(applied_speed >= 0.0,
               "feedback: applied speed comes from the published trajectory");
        Expect(std::fabs(applied_speed - maintenance.v.back()) < 1e-9,
               "feedback: applied speed equals the maintenance endpoint speed");
    }
    {
        // --- 후보 막힘 (충돌) --------------------------------------------
        //
        // plan_valid=false지만 maintenance는 계속 존재한다. WAITING이
        // trajectory_source=SAFE_STOP을 뜻하지 않는다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 8.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommand(21, LateralDirection::NONE), maintenance, result);

        Expect(ok, "feedback: a blocked candidate still yields maintenance");
        Expect(!result.candidate_valid,
               "feedback: plan_valid is false when the candidate is blocked");
        Expect(result.selected_cost == -1.0,
               "feedback: a blocked candidate reports the -1.0 cost sentinel");
        Expect(result.blocking_object_id == 21,
               "feedback: the blocking object id is preserved for diagnostics");
        Expect(result.generated_candidate_count > 0 &&
                   result.valid_candidate_count == 0,
               "feedback: candidate counts show generated-but-all-rejected");
        // 계약: PREPARE에서 후보 없음은 terminal이 아니다. core도 실패로
        // 보고하지 않아야 node가 RESULT_NONE을 쓸 수 있다.
        Expect(planner.last_failure_reason() == Reason::NONE,
               "feedback: a blocked candidate is not a planner failure");
    }
    {
        // --- 감속/정지 maintenance의 applied_speed ------------------------
        //
        // **이것이 D6-2c에서 고쳐야 했던 문제다.** last_target_speed_를 쓰면
        // 실제 궤적은 0 m/s를 향하는데 순항 목표속도(예: 10)가 나간다.
        FrenetPlanner planner;
        planner.Init(config);
        // 장애물이 아주 가까우면 maintenance가 정지 계열로 내려간다(실측).
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 4.0, 0.0)};
        const auto boxes = boxes_for(objects);
        const CartesianState slow_ego = MakeEgo(PoseAt(waypoints, kEgoS), 0.3);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(slow_ego, objects, boxes, MakeAvoidCommand(21),
                             maintenance, result);

        Expect(!maintenance.v.empty(),
               "applied speed: the stop-tier maintenance has samples");
        const double endpoint_speed = maintenance.v.back();
        Expect(std::fabs(endpoint_speed) < 1e-6,
               "applied speed: the stop-tier maintenance ends at zero");
        // planner의 순항 목표속도는 0이 아니다. 그것을 보고하면 거짓이다.
        Expect(planner.last_target_speed() > 0.5,
               "applied speed: last_target_speed is a cruise value, not zero");
        Expect(std::fabs(planner.last_target_speed() - endpoint_speed) > 0.5,
               "applied speed: the cruise value would misreport the stop");
    }
    {
        // --- 방향 모순 / 후방 목표의 진단 값 -----------------------------
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(
            ego, objects, boxes,
            MakeAvoidCommandWithTargetD(21, kEgoS + 30.0, -2.5,
                                        LateralDirection::LEFT),
            maintenance, result);
        Expect(!result.candidate_valid && result.selected_cost == -1.0,
               "feedback: a direction conflict reports no cost");
        Expect(result.blocking_object_id == -1,
               "feedback: a direction conflict blames no collision object");
        Expect(result.generated_candidate_count == 0,
               "feedback: a direction conflict generates no candidates");
        Expect(planner.last_failure_reason() == Reason::NONE,
               "feedback: a direction conflict is WAITING, not FAILED");
    }
    {
        // --- 같은 PREPARE를 반복해도 상태가 왕복 가능하다 -----------------
        //
        // core 쪽에서 latch 성격의 상태가 남지 않는지 확인한다. manager의
        // 전이는 command_execution_manager_test가 담당한다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> clear_scene{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> blocked_scene{
            MakeObstacle(waypoints, 21, kEgoS + 8.0, 0.0)};
        CartesianPath maintenance;
        Prepare result;

        planner.PrepareAvoid(ego, clear_scene, boxes_for(clear_scene),
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        Expect(result.candidate_valid, "round trip: first cycle is valid");

        planner.PrepareAvoid(ego, blocked_scene, boxes_for(blocked_scene),
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        Expect(!result.candidate_valid, "round trip: blocked cycle is invalid");

        planner.PrepareAvoid(ego, clear_scene, boxes_for(clear_scene),
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        Expect(result.candidate_valid,
               "round trip: the candidate recovers when the scene clears");
        Expect(planner.last_failure_reason() == Reason::NONE,
               "round trip: no failure state is latched in core");
    }

    // ===== D6-3a: EXECUTE 재검증과 COMMIT target lock ==================

    {
        // ---- PREPARE는 lock하지 않는다 (드디어 직접 검증) --------------
        //
        // D6-2b/D6-2c에서 getter가 없어 간접 확인만 했던 항목이다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(ego, objects, boxes,
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        Expect(result.candidate_valid, "no-lock: PREPARE found a candidate");
        Expect(!planner.avoid_committed(),
               "no-lock: PREPARE does not commit");
        Expect(planner.avoid_locked_target_d() == 0.0,
               "no-lock: no target is locked after PREPARE");
        Expect(std::string(planner.avoid_phase_name()) == "TRACKING",
               "no-lock: the phase stays TRACKING before commit");
        // 여러 사이클을 돌려도 lock되지 않는다.
        for (int cycle = 0; cycle < 3; ++cycle) {
            planner.PrepareAvoid(ego, objects, boxes,
                                 MakeAvoidCommand(21, LateralDirection::LEFT),
                                 maintenance, result);
        }
        Expect(!planner.avoid_committed(),
               "no-lock: repeated PREPARE never commits by itself");
    }
    {
        // ---- CommitAvoid가 목표를 확정한다 -----------------------------
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(ego, objects, boxes,
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        const double selected = result.selected_target_d;

        Expect(planner.CommitAvoid(result), "commit: CommitAvoid succeeds");
        Expect(planner.avoid_committed(), "commit: the maneuver is committed");
        Expect(std::fabs(planner.avoid_locked_target_d() - selected) < 1e-9,
               "commit: the locked target equals the selected candidate d");
        Expect(std::string(planner.avoid_phase_name()) == "SHIFT",
               "commit: the phase advances to SHIFT");
        // start_d는 COMMIT 시점의 ego d다. 진행률 기준점이다.
        Expect(std::fabs(planner.avoid_start_d()) < 0.5,
               "commit: start_d records the ego lateral position at commit");
        Expect(std::fabs(planner.avoid_start_d() -
                         planner.avoid_locked_target_d()) > 1.5,
               "commit: start_d and target_d are far apart (a real shift)");
    }
    {
        // ---- CommitAvoid 입력 방어 --------------------------------------
        //
        // 확보하지 않은 목표를 잠그지 않는다.
        FrenetPlanner planner;
        planner.Init(config);
        Prepare invalid;   // candidate_valid=false
        Expect(!planner.CommitAvoid(invalid),
               "commit guard: an invalid result is rejected");
        Expect(!planner.avoid_committed(),
               "commit guard: nothing is locked on rejection");

        // ***** candidate_valid guard를 단독으로 검증한다 *****
        //
        // 위 검사는 대상도 확보되지 않은 상태라 active guard가 먼저 막는다.
        // 그러면 candidate_valid guard를 제거해도 통과한다(돌연변이로 확인).
        // 그래서 **대상은 확보하고 후보만 무효인** 상태를 만든다.
        const std::vector<ObjectInfo> blocked_objects{
            MakeObstacle(waypoints, 21, kEgoS + 8.0, 0.0)};   // 너무 가깝다
        CartesianPath blocked_maintenance;
        Prepare blocked_result;
        planner.PrepareAvoid(ego, blocked_objects,
                             boxes_for(blocked_objects),
                             MakeAvoidCommand(21, LateralDirection::NONE),
                             blocked_maintenance, blocked_result);
        Expect(planner.active_avoid_target_id() == 21,
               "commit guard: the target is acquired for the blocked case");
        Expect(!blocked_result.candidate_valid,
               "commit guard: the candidate is blocked (target still bound)");
        Expect(!planner.CommitAvoid(blocked_result),
               "commit guard: an acquired target with no safe candidate "
               "cannot be committed");
        Expect(!planner.avoid_committed(),
               "commit guard: a blocked candidate locks nothing");
        Expect(planner.avoid_locked_target_d() == 0.0,
               "commit guard: no target d is locked from a blocked candidate");

    }
    {
        // active guard: 대상을 확보하지 않은 planner는 잠글 수 없다.
        // **fresh planner여야 한다** - 위 블록에서 대상을 확보했으므로
        // 같은 인스턴스를 쓰면 이 guard가 아니라 다른 경로로 통과한다.
        FrenetPlanner fresh;
        fresh.Init(config);
        Prepare fabricated;
        fabricated.candidate_valid = true;
        fabricated.selected_target_d = 2.0;
        Expect(!fresh.CommitAvoid(fabricated),
               "commit guard: committing without an acquired target fails");
        Expect(!fresh.avoid_committed(),
               "commit guard: a fabricated result locks nothing");
    }
    {
        // 대상을 확보한 뒤라도 비유한 목표는 거부한다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(ego, objects, boxes,
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        Expect(result.candidate_valid,
               "commit guard: the baseline candidate is valid");
        Prepare nan_target = result;
        nan_target.selected_target_d = std::nan("");
        Expect(!planner.CommitAvoid(nan_target),
               "commit guard: a non-finite target is rejected");
        Expect(!planner.avoid_committed(),
               "commit guard: a NaN target locks nothing");
    }

    {
        // ---- D6-3a1: manager/core COMMIT atomicity ----------------------
        //
        // 기존 node 순서(NotifyCommitted -> CommitAvoid)는 이 입력에서 manager
        // 만 COMMITTED로 남겼다. coordinator는 동일한 core precondition을
        // mutation 전에 검사하므로 manager를 전혀 바꾸지 않는다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> blocked_objects{
            MakeObstacle(waypoints, 21, kEgoS + 8.0, 0.0)};
        CartesianPath maintenance;
        Prepare blocked;
        planner.PrepareAvoid(ego, blocked_objects, boxes_for(blocked_objects),
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, blocked);
        auto manager = MakeAvoidManagerReady(3001);
        Expect(
            manager.execution_state() == asmc_msgs::FrenetFeedback::READY &&
                !blocked.candidate_valid,
            "atomic commit: vulnerable READY plus invalid core input is reproduced");
        Expect(!frenet_planner_ros::CommitAvoidAtomically(manager, planner,
                                                          blocked),
               "atomic commit: core preflight rejects before manager mutation");
        Expect(manager.execution_state() == asmc_msgs::FrenetFeedback::READY &&
                   !planner.avoid_committed(),
               "atomic commit: manager COMMITTED/core unlocked is impossible");
    }
    {
        // 정상 READY + valid candidate는 두 상태와 target lock을 함께 올린다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        CartesianPath maintenance;
        Prepare valid;
        planner.PrepareAvoid(ego, objects, boxes_for(objects),
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, valid);
        auto manager = MakeAvoidManagerReady(3002);
        const double selected = valid.selected_target_d;
        Expect(valid.candidate_valid && planner.CanCommitAvoid(valid),
               "atomic commit: normal candidate passes pure preflight");
        Expect(frenet_planner_ros::CommitAvoidAtomically(manager, planner,
                                                         valid),
               "atomic commit: normal READY transaction commits");
        Expect(manager.execution_state() ==
                       asmc_msgs::FrenetFeedback::COMMITTED &&
                   planner.avoid_committed(),
               "atomic commit: manager and core become committed together");
        Expect(std::fabs(planner.avoid_locked_target_d() - selected) < 1e-9,
               "atomic commit: normal path locks the selected target d");
    }

    {
        // D6-3b: 첫 COMMIT 다음 tick은 maintenance로 돌아가지 않고 manager
        // state를 한 번만 EXECUTING으로 올린 뒤 committed core path를 쓴다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        auto manager = MakeAvoidManagerReady(3003);
        const PlannerCommand command =
            MakeAvoidCommand(21, LateralDirection::LEFT);

        CartesianPath first_maintenance;
        Prepare first;
        planner.PrepareAvoid(ego, objects, boxes, command,
                             first_maintenance, first);
        Expect(frenet_planner_ros::CommitAvoidAtomically(manager, planner,
                                                          first),
               "D6-3b probe: first EXECUTE commits the candidate");

        Expect(!planner.CommitAvoid(first),
               "D6-3b duplicate commit: second commit is rejected");
        const double locked = planner.avoid_locked_target_d();
        const double start_d = planner.avoid_start_d();
        Expect(std::fabs(locked - first.selected_target_d) < 1e-9 &&
                   std::fabs(planner.avoid_start_d() - start_d) < 1e-9 &&
                   std::string(planner.avoid_phase_name()) == "SHIFT",
               "D6-3b duplicate commit: lock/start/phase stay unchanged");

        Expect(frenet_planner_ros::RouteAvoidPlanning(manager) ==
                   frenet_planner_ros::AvoidPlanningRoute::POST_COMMIT &&
                   manager.execution_state() ==
                       asmc_msgs::FrenetFeedback::EXECUTING,
               "D6-3b route: COMMITTED advances once to EXECUTING");
        CartesianPath next_output;
        Prepare next;
        const bool next_ok = planner.PlanCommittedAvoid(
            ego, objects, boxes, command, next_output, next);
        Expect(next_ok && next.candidate_valid &&
                   std::fabs(next.selected_target_d - locked) < 1e-9 &&
                   std::fabs(EndpointD(planner, next_output,
                                       kEgoS + 20.0) - locked) < 1e-6,
               "D6-3b continuity: next tick publishes committed AVOID");
        Expect(frenet_planner_ros::RouteAvoidPlanning(manager) ==
                   frenet_planner_ros::AvoidPlanningRoute::POST_COMMIT &&
                   manager.execution_state() ==
                       asmc_msgs::FrenetFeedback::EXECUTING,
               "D6-3b route: EXECUTING stays post-COMMIT without retransition");
    }

    {
        // ---- **핵심**: COMMIT 이후 target_d가 흔들리지 않는다 -----------
        //
        // ResolveManeuver의 lateral 격자는 target_d ± 0.5 m다. COMMIT에서
        // 목표를 잠갔는데 이후 사이클이 격자 주변을 다시 탐색해 다른 값을
        // 고르면 "lock했다"는 의미가 사라진다.
        //
        // ===== 구현 중 실측으로 찾은 버그 =====
        //
        // DecideAvoidTargets는 매 사이클 **요청**(pose 또는 geometry)에서
        // 목표를 다시 계산한다. COMMIT 이후에도 그것을 쓰면 잠근 값이 생성에
        // 반영되지 않았다:
        //
        //   PREPARE  요청 2.8 -> 격자 탐색으로 +3.30 선택
        //   COMMIT   locked = +3.30
        //   post     생성 목표가 다시 +2.80 (요청값)   <- lock이 무의미
        //
        // 그래서 committed이면 목표 목록을 잠긴 값 하나로 대체한다.
        //
        // 관측 조건도 실측으로 찾았다. 기하 목표(2.55)는 우연히 격자 중심과
        // 일치해 drift가 보이지 않는다. **target pose 2.8**을 쓰면 갈린다.
        const std::vector<ObjectInfo> objects{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const auto boxes = boxes_for(objects);
        const PlannerCommand pose_cmd = MakeAvoidCommandWithTargetD(
            21, kEgoS + 30.0, 2.8, LateralDirection::LEFT);

        // (a) 요청값과 실제 선택이 다르다는 것을 먼저 확인한다. 이것이
        //     없으면 아래 exact-lock 검사가 무의미하다(요청과 lock이 같으면
        //     tolerance를 무시해도 통과한다).
        FrenetPlanner uncommitted;
        uncommitted.Init(config);
        CartesianPath m_unc;
        Prepare r_unc;
        uncommitted.PrepareAvoid(ego, objects, boxes, pose_cmd, m_unc, r_unc);
        Expect(r_unc.candidate_valid,
               "exact lock: the uncommitted cycle finds a candidate");
        Expect(std::fabs(r_unc.selected_target_d - 2.8) > 0.2,
               "exact lock: the selected candidate drifts off the request "
               "(so request != locked value)");

        // (b) COMMIT하면 이후 사이클이 잠긴 목표를 정확히 겨냥한다.
        FrenetPlanner committed;
        committed.Init(config);
        CartesianPath m_pre;
        Prepare r_pre;
        committed.PrepareAvoid(ego, objects, boxes, pose_cmd, m_pre, r_pre);
        Expect(committed.CommitAvoid(r_pre),
               "exact lock: the commit succeeds");
        const double locked = committed.avoid_locked_target_d();
        Expect(std::fabs(locked - r_pre.selected_target_d) < 1e-9,
               "exact lock: the locked value is the selected candidate d");
        Expect(std::fabs(locked - 2.8) > 0.2,
               "exact lock: the locked value differs from the raw request");

        bool all_exact = true;
        for (int cycle = 0; cycle < 5; ++cycle) {
            CartesianPath m_post;
            Prepare r_post;
            committed.PlanCommittedAvoid(ego, objects, boxes, pose_cmd,
                                         m_post, r_post);
            if (!r_post.candidate_valid ||
                std::fabs(r_post.selected_target_d - locked) > 1e-6) {
                all_exact = false;
            }
            // 생성 목표 목록 자체도 잠긴 값 하나여야 한다.
            if (committed.avoid_target_count() != 1 ||
                std::fabs(committed.avoid_target_candidate(0) - locked) >
                    1e-9) {
                all_exact = false;
            }
        }
        Expect(all_exact,
               "exact lock: every post-commit cycle targets the locked d");
    }
    {
        // 잠긴 목표가 이후 사이클에 막히면 후보가 무효가 되지만, **lock은
        // 풀리지 않는다.** 격자로 우회해 다른 목표를 고르지도 않는다 -
        // 그것이 lock의 의미다.
        //
        // 장면을 바꿔 막는다: COMMIT 후 잠긴 목표 경로에 새 장애물을 놓는다.
        const std::vector<ObjectInfo> clear_scene{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        FrenetPlanner committed;
        committed.Init(config);
        CartesianPath m1;
        Prepare r1;
        committed.PrepareAvoid(ego, clear_scene, boxes_for(clear_scene),
                               MakeAvoidCommand(21, LateralDirection::LEFT),
                               m1, r1);
        Expect(r1.candidate_valid, "blocked lock: the commit cycle is valid");
        auto manager = MakeAvoidManagerReady(3004);
        Expect(frenet_planner_ros::CommitAvoidAtomically(manager, committed,
                                                          r1),
               "blocked lock: the commit succeeds");
        frenet_planner_ros::RouteAvoidPlanning(manager);
        const double locked = committed.avoid_locked_target_d();

        // 잠긴 목표(약 +2.55) 쪽에 장애물을 추가한다.
        const std::vector<ObjectInfo> blocked_scene{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0),
            // 4번째 인자는 speed, 5번째가 lateral_d다.
            MakeObstacle(waypoints, 31, kEgoS + 18.0, 0.0, locked),
        };
        CartesianPath m2;
        Prepare r2;
        const bool ok = committed.PlanCommittedAvoid(
            ego, blocked_scene, boxes_for(blocked_scene),
            MakeAvoidCommand(21, LateralDirection::LEFT), m2, r2);
        Expect(!ok && !r2.candidate_valid &&
                   committed.last_failure_reason() ==
                       Reason::COLLISION_BLOCKED,
               "blocked lock: a blocker on the locked path invalidates the "
               "candidate");
        manager.NotifyPlanFailed(/*terminal=*/false);
        Expect(manager.execution_state() ==
                   asmc_msgs::FrenetFeedback::EXECUTING,
               "blocked lock: collision does not downgrade execution state");
        Expect(committed.avoid_committed(),
               "blocked lock: a blocked cycle does not un-commit");
        Expect(std::fabs(committed.avoid_locked_target_d() - locked) < 1e-9,
               "blocked lock: the locked target survives a blocked cycle");
        Expect(committed.avoid_target_count() == 1 &&
                   std::fabs(committed.avoid_target_candidate(0) - locked) <
                       1e-9,
               "blocked lock: generation does not wander to another target");
        Expect(committed.active_avoid_target_id() == 21,
               "blocked lock: the designated target is unchanged");
    }

    {
        // COMMIT 이후 target observation은 지속 조건이 아니다. grace보다 긴
        // dropout과 더 가까운 다른 객체 모두 locked geometry를 바꾸지 않는다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        const PlannerCommand command =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        planner.PrepareAvoid(ego, initial, boxes_for(initial), command,
                             maintenance, prepared);
        Expect(planner.CommitAvoid(prepared),
               "post-commit dropout: initial commit succeeds");
        const double locked = planner.avoid_locked_target_d();

        bool continuous = true;
        for (int cycle = 0; cycle < 8; ++cycle) {
            CartesianPath path;
            Prepare result;
            if (!planner.PlanCommittedAvoid(ego, {}, {}, command, path,
                                            result) ||
                planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST ||
                std::fabs(result.selected_target_d - locked) > 1e-9 ||
                !planner.avoid_committed()) {
                continuous = false;
            }
        }
        Expect(continuous,
               "post-commit dropout: execution survives beyond grace");

        const std::vector<ObjectInfo> closer_other{
            MakeObstacle(waypoints, 17, kEgoS + 12.0, 0.0, -4.0)};
        CartesianPath path;
        Prepare result;
        const bool ok = planner.PlanCommittedAvoid(
            ego, closer_other, boxes_for(closer_other), command, path, result);
        Expect(ok && planner.active_avoid_target_id() == 21 &&
                   std::fabs(planner.avoid_locked_target_d() - locked) < 1e-9,
               "post-commit ownership: a closer object cannot hand off target");
    }

    {
        // 저속에서도 중앙 fallback이 아니라 locked target 방향의 crawl을 쓰며,
        // 그 crawl은 fresh collision geometry로 검사한다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        const PlannerCommand command =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        planner.PrepareAvoid(ego, initial, boxes_for(initial), command,
                             maintenance, prepared);
        planner.CommitAvoid(prepared);
        const double locked = planner.avoid_locked_target_d();

        CartesianState slow_ego = ego;
        slow_ego.v = 0.3;
        CartesianPath crawl;
        Prepare clear;
        const bool clear_ok = planner.PlanCommittedAvoid(
            slow_ego, initial, boxes_for(initial), command, crawl, clear);
        Expect(clear_ok && clear.low_speed_context &&
                   std::fabs(clear.selected_target_d - locked) < 1e-9 &&
                   EndpointD(planner, crawl, kEgoS + 1.0) >
                       EndpointD(planner, CartesianPath{{ego.x}, {ego.y}},
                                 kEgoS),
               "post-commit low speed: crawl keeps the locked direction");

        const std::vector<ObjectInfo> blocked{
            MakeObstacle(waypoints, 31, kEgoS + 0.5, 0.0, 0.0)};
        CartesianPath rejected;
        Prepare blocked_result;
        const bool blocked_ok = planner.PlanCommittedAvoid(
            slow_ego, blocked, boxes_for(blocked), command, rejected,
            blocked_result);
        Expect(!blocked_ok && planner.last_failure_reason() ==
                                  Reason::COLLISION_BLOCKED &&
                   planner.avoid_committed() &&
                   std::fabs(planner.avoid_locked_target_d() - locked) < 1e-9,
               "post-commit low speed: blocked crawl is rejected without unlock");
    }

    {
        // D6-3c lifecycle: SHIFT -> PASS -> COMPLETED. 대상 observation 없이도
        // COMMIT 때 저장한 geometry로 진행하며 progress는 유한/단조/범위 내다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const PlannerCommand command =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareAvoid(ego, initial, boxes_for(initial), command,
                             maintenance, prepared);
        auto manager = MakeAvoidManagerReady(3100);
        Expect(frenet_planner_ros::CommitAvoidAtomically(manager, planner,
                                                          prepared) &&
                   std::string(planner.avoid_phase_name()) == "SHIFT",
               "lifecycle: COMMIT starts in SHIFT");
        frenet_planner_ros::RouteAvoidPlanning(manager);
        const double locked = planner.avoid_locked_target_d();
        const double start_d = planner.avoid_start_d();

        CartesianPath shift_path;
        Prepare shift;
        const bool shift_ok = planner.PlanCommittedAvoid(
            ego, {}, {}, command, shift_path, shift);
        Expect(shift_ok && !shift.maneuver_completed &&
                   std::string(planner.avoid_phase_name()) == "SHIFT" &&
                   std::isfinite(shift.maneuver_progress) &&
                   shift.maneuver_progress >= 0.0 &&
                   shift.maneuver_progress < 0.5,
               "lifecycle: outside tolerance remains SHIFT with bounded progress");

        const double sign = locked >= start_d ? 1.0 : -1.0;
        const CartesianState approaching = MakeEgoAt(
            waypoints, kEgoS, locked - sign * 0.6, kEgoSpeed);
        CartesianPath approaching_path;
        Prepare approaching_result;
        planner.PlanCommittedAvoid(approaching, {}, {}, command,
                                   approaching_path, approaching_result);
        Expect(std::string(planner.avoid_phase_name()) == "SHIFT" &&
                   approaching_result.maneuver_progress >=
                       shift.maneuver_progress &&
                   approaching_result.maneuver_progress < 0.5,
               "lifecycle: approaching target advances SHIFT progress");

        CartesianPath jitter_path;
        Prepare jitter;
        planner.PlanCommittedAvoid(ego, {}, {}, command, jitter_path, jitter);
        Expect(jitter.maneuver_progress >=
                   approaching_result.maneuver_progress,
               "lifecycle: localization regression cannot decrease progress");

        const CartesianState at_target =
            MakeEgoAt(waypoints, kEgoS, locked, kEgoSpeed);
        CartesianPath pass_path;
        Prepare pass;
        const bool pass_ok = planner.PlanCommittedAvoid(
            at_target, {}, {}, command, pass_path, pass);
        Expect(pass_ok && !pass.maneuver_completed &&
                   std::string(planner.avoid_phase_name()) == "PASS" &&
                   pass.maneuver_progress >= 0.5 &&
                   pass.maneuver_progress < 1.0,
               "lifecycle: entering tolerance advances SHIFT to PASS only");

        const CartesianState before_clear = MakeEgoAt(
            waypoints, planner.avoid_target_s(), locked, kEgoSpeed);
        CartesianPath before_clear_path;
        Prepare before_clear_result;
        planner.PlanCommittedAvoid(before_clear, {}, {}, command,
                                   before_clear_path, before_clear_result);
        Expect(!before_clear_result.maneuver_completed &&
                   manager.execution_state() ==
                       asmc_msgs::FrenetFeedback::EXECUTING &&
                   before_clear_result.maneuver_progress >= 0.5 &&
                   before_clear_result.maneuver_progress < 1.0,
               "lifecycle: PASS before longitudinal clearance keeps EXECUTING");

        const CartesianState after_clear = MakeEgoAt(
            waypoints, planner.avoid_target_s() + 20.0, locked, kEgoSpeed);
        CartesianPath final_path;
        Prepare completed;
        const bool final_ok = planner.PlanCommittedAvoid(
            after_clear, {}, {}, command, final_path, completed);
        Expect(final_ok && completed.maneuver_completed &&
                   completed.maneuver_progress == 1.0 &&
                   planner.avoid_completed(),
               "lifecycle: PASS plus longitudinal clearance completes");
        Expect(planner.avoid_committed() &&
                   planner.active_avoid_target_id() == 21 &&
                   std::fabs(planner.avoid_locked_target_d() - locked) < 1e-9 &&
                   std::string(planner.avoid_phase_name()) == "PASS",
               "lifecycle: completion retains committed context and lock");
        Expect(std::fabs(EndpointD(planner, final_path,
                                   planner.avoid_target_s() + 25.0) - locked) <
                   1e-6,
               "lifecycle: completion cycle still publishes final AVOID path");

        Expect(!frenet_planner_ros::CompleteAvoidAtomically(
                   manager, completed, /*trajectory_ready=*/false) &&
                   manager.execution_state() ==
                       asmc_msgs::FrenetFeedback::EXECUTING,
               "completion atomicity: unprepared trajectory cannot complete manager");
        Expect(frenet_planner_ros::CompleteAvoidAtomically(
                   manager, completed, /*trajectory_ready=*/true) &&
                   manager.execution_state() ==
                       asmc_msgs::FrenetFeedback::COMPLETED,
               "completion atomicity: prepared final trajectory completes manager");

        auto same = manager.active_command();
        same.execution_request = asmc_msgs::BehaviorCommand::EXECUTE;
        const auto latched = manager.Accept(same);
        Expect(latched.planning_target ==
                   frenet_planner_ros::PlanningTarget::NONE &&
                   latched.execution_state ==
                       asmc_msgs::FrenetFeedback::COMPLETED &&
                   manager.completed_normally() && planner.avoid_completed(),
               "completion latch: same id reuses final path without core reset");

        manager.SetImplementedManeuver(asmc_msgs::BehaviorCommand::KEEP, true);
        auto keep = MakeAvoidTransactionCommand(3101);
        keep.maneuver_type = asmc_msgs::BehaviorCommand::KEEP;
        keep.execution_request = asmc_msgs::BehaviorCommand::EXECUTE;
        keep.direction = asmc_msgs::BehaviorCommand::DIRECTION_NONE;
        keep.primary_object_id = -1;
        const auto new_keep = manager.Accept(keep);
        PlannerCommand keep_plan{};
        keep_plan.mode = LANE_KEEPING;
        CartesianPath center_path;
        planner.Plan(after_clear, {}, keep_plan, center_path);
        Expect(new_keep.planning_target ==
                   frenet_planner_ros::PlanningTarget::INCOMING_COMMAND &&
                   !planner.avoid_committed() &&
                   planner.active_avoid_target_id() == -1,
               "completion latch: new KEEP owns center return and resets AVOID");
    }

    {
        // completion geometry를 충족해도 현재 committed path가 막히면 collision이
        // 우선하며 manager/core 모두 완료되지 않는다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const PlannerCommand command =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareAvoid(ego, initial, boxes_for(initial), command,
                             maintenance, prepared);
        auto manager = MakeAvoidManagerReady(3102);
        frenet_planner_ros::CommitAvoidAtomically(manager, planner, prepared);
        frenet_planner_ros::RouteAvoidPlanning(manager);
        const double locked = planner.avoid_locked_target_d();
        const CartesianState at_target =
            MakeEgoAt(waypoints, kEgoS, locked, kEgoSpeed);
        CartesianPath pass_path;
        Prepare pass;
        planner.PlanCommittedAvoid(at_target, {}, {}, command, pass_path, pass);

        const double clear_s = planner.avoid_target_s() + 20.0;
        const CartesianState clear_ego =
            MakeEgoAt(waypoints, clear_s, locked, kEgoSpeed);
        const std::vector<ObjectInfo> blocked{
            MakeObstacle(waypoints, 31, clear_s + 0.5, 0.0, locked)};
        CartesianPath blocked_path;
        Prepare blocked_result;
        const bool blocked_ok = planner.PlanCommittedAvoid(
            clear_ego, blocked, boxes_for(blocked), command, blocked_path,
            blocked_result);
        manager.NotifyPlanFailed(/*terminal=*/false);
        Expect(!blocked_ok && planner.last_failure_reason() ==
                                  Reason::COLLISION_BLOCKED &&
                   !blocked_result.maneuver_completed &&
                   !planner.avoid_completed() &&
                   manager.execution_state() ==
                       asmc_msgs::FrenetFeedback::EXECUTING,
               "completion priority: collision blocks completion without state drop");
    }

    {
        // D6-3c1 probe: target은 seam 너머 전방에서 확보되어 obstacle_s가
        // unwrapped 값으로 저장된다. ego가 seam을 지난 뒤 raw subtraction을
        // 쓰면 gap이 거의 한 바퀴 양수가 되어 완료되지 않는다.
        const double track_length = TrackLength(waypoints);
        const double ego_before_s = track_length - 30.0;
        const double obstacle_after_s = 10.0;
        const CartesianState ego_before = MakeEgoAt(
            waypoints, ego_before_s, 0.0, kEgoSpeed);
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, obstacle_after_s, 0.0)};
        const PlannerCommand command =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        const bool prepare_ok = planner.PrepareAvoid(
            ego_before, initial, boxes_for(initial), command, maintenance,
            prepared);
        const bool commit_ok = prepare_ok && prepared.candidate_valid &&
            planner.CommitAvoid(prepared);
        const double locked = planner.avoid_locked_target_d();
        Expect(commit_ok && planner.avoid_target_s() > ego_before_s,
               "closed-loop lifecycle: seam-forward target stores unwrapped s");

        CartesianPath pass_path;
        Prepare pass;
        const bool pass_ok = commit_ok && planner.PlanCommittedAvoid(
            MakeEgoAt(waypoints, ego_before_s, locked, kEgoSpeed), {}, {},
            command, pass_path, pass);
        Expect(pass_ok && !pass.maneuver_completed &&
                   std::string(planner.avoid_phase_name()) == "PASS",
               "closed-loop lifecycle: both actors before seam enter PASS");
        CartesianPath center_path;
        Prepare center;
        const bool center_ok = pass_ok && planner.PlanCommittedAvoid(
            MakeEgoAt(waypoints, obstacle_after_s, locked, kEgoSpeed), {}, {},
            command, center_path, center);
        Expect(center_ok && !center.maneuver_completed &&
                   std::isfinite(center.maneuver_progress) &&
                   center.maneuver_progress >= pass.maneuver_progress,
               "closed-loop lifecycle: passing seam reaches obstacle center");
        CartesianPath clear_path;
        Prepare clear;
        const bool clear_ok = center_ok && planner.PlanCommittedAvoid(
            MakeEgoAt(waypoints, obstacle_after_s + 20.0, locked, kEgoSpeed),
            {}, {}, command, clear_path, clear);
        Expect(clear_ok && !center.maneuver_completed &&
                   clear.maneuver_completed &&
                   clear.maneuver_progress == 1.0 &&
                   clear.maneuver_progress >= center.maneuver_progress,
               "closed-loop lifecycle: seam crossing completes with monotonic progress");
    }

    {
        // D6-3c1 probe: terminal latch 뒤 새 AVOID ID는 manager에서는 새
        // transaction이지만, core reset이 없으면 old committed context를 쓴다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        const PlannerCommand old_command =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        planner.PrepareAvoid(ego, initial, boxes_for(initial), old_command,
                             maintenance, prepared);
        auto manager = MakeAvoidManagerReady(3200);
        frenet_planner_ros::CommitAvoidAtomically(manager, planner, prepared);
        frenet_planner_ros::RouteAvoidPlanning(manager);
        const double locked = planner.avoid_locked_target_d();
        CartesianPath pass_path;
        Prepare pass;
        planner.PlanCommittedAvoid(
            MakeEgoAt(waypoints, kEgoS, locked, kEgoSpeed), {}, {},
            old_command, pass_path, pass);
        CartesianPath completed_path;
        Prepare completed;
        planner.PlanCommittedAvoid(
            MakeEgoAt(waypoints, planner.avoid_target_s() + 20.0, locked,
                      kEgoSpeed),
            {}, {}, old_command, completed_path, completed);
        manager.NotifyManeuverCompleted();

        auto next_command = MakeAvoidTransactionCommand(3201);
        next_command.primary_object_id = 31;
        const auto started = manager.Accept(next_command);
        if (started.transaction_started) planner.ResetExternalAvoidance();
        CartesianPath next_maintenance;
        Prepare next;
        const bool next_ok = planner.PrepareAvoid(
            ego, {}, {}, MakeAvoidCommand(31, LateralDirection::LEFT),
            next_maintenance, next);
        Expect(started.planning_target ==
                   frenet_planner_ros::PlanningTarget::INCOMING_COMMAND &&
                   started.transaction_started &&
                   completed.maneuver_completed &&
                   !next_ok && !planner.avoid_committed() &&
                   planner.avoid_missing_cycles() == 0 &&
                   planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "transaction boundary: new AVOID cannot inherit completed context");

        const std::vector<ObjectInfo> replacement{
            MakeObstacle(waypoints, 31, kEgoS + 25.0, 0.0)};
        CartesianPath replacement_maintenance;
        Prepare replacement_result;
        const bool replacement_ok = planner.PrepareAvoid(
            ego, replacement, boxes_for(replacement),
            MakeAvoidCommand(31, LateralDirection::LEFT),
            replacement_maintenance, replacement_result);
        manager.NotifyPlanSucceeded();
        auto execute = next_command;
        execute.execution_request = asmc_msgs::BehaviorCommand::EXECUTE;
        manager.Accept(execute);
        Expect(replacement_ok && replacement_result.candidate_valid &&
                   frenet_planner_ros::CommitAvoidAtomically(
                       manager, planner, replacement_result) &&
                   planner.active_avoid_target_id() == 31,
               "transaction boundary: replacement AVOID can prepare and commit");
    }

    {
        // PRE_COMMIT(READY)에서도 새 AVOID ID는 old binding/dropout grace를
        // 상속하지 않는다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareAvoid(
            ego, initial, boxes_for(initial),
            MakeAvoidCommand(21, LateralDirection::LEFT), maintenance,
            prepared);
        auto manager = MakeAvoidManagerReady(3300);
        auto next_command = MakeAvoidTransactionCommand(3301);
        next_command.primary_object_id = 31;
        const auto started = manager.Accept(next_command);
        if (started.transaction_started) planner.ResetExternalAvoidance();

        CartesianPath next_maintenance;
        Prepare next;
        const bool next_ok = planner.PrepareAvoid(
            ego, {}, {}, MakeAvoidCommand(31, LateralDirection::LEFT),
            next_maintenance, next);
        Expect(started.transaction_started && !next_ok &&
                   !planner.avoid_committed() &&
                   planner.avoid_missing_cycles() == 0 &&
                   planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "transaction boundary: pre-commit AVOID replacement is fresh");
    }

    {
        // 새 KEEP transaction이 input stale로 planner에 도달하지 않는 cycle도
        // manager boundary에서 이전 AVOID context를 즉시 끊는다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareAvoid(
            ego, initial, boxes_for(initial),
            MakeAvoidCommand(21, LateralDirection::LEFT), maintenance,
            prepared);
        auto manager = MakeAvoidManagerReady(3400);
        frenet_planner_ros::CommitAvoidAtomically(manager, planner, prepared);
        frenet_planner_ros::RouteAvoidPlanning(manager);
        manager.NotifyManeuverCompleted();
        manager.SetImplementedManeuver(
            asmc_msgs::BehaviorCommand::KEEP, true);
        auto keep = MakeAvoidTransactionCommand(3401);
        keep.maneuver_type = asmc_msgs::BehaviorCommand::KEEP;
        keep.execution_request = asmc_msgs::BehaviorCommand::EXECUTE;
        keep.direction = asmc_msgs::BehaviorCommand::DIRECTION_NONE;
        keep.primary_object_id = -1;
        const auto started = manager.Accept(keep);
        if (started.transaction_started) planner.ResetExternalAvoidance();
        // stale-input 조기 반환을 흉내 내어 Plan()은 호출하지 않는다.
        Expect(started.transaction_started && !planner.avoid_committed() &&
                   planner.active_avoid_target_id() == -1,
               "transaction boundary: stale-cycle KEEP clears completed AVOID");
    }

    {
        // 방어적 zero-span COMMIT에서도 progress 계산은 0으로 나누지 않는다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const PlannerCommand command =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareAvoid(ego, initial, boxes_for(initial), command,
                             maintenance, prepared);
        prepared.selected_target_d = planner.last_d();
        planner.CommitAvoid(prepared);
        CartesianPath path;
        Prepare result;
        const bool ok = planner.PlanCommittedAvoid(
            ego, {}, {}, command, path, result);
        Expect(ok && std::isfinite(result.maneuver_progress) &&
                   result.maneuver_progress >= 0.0 &&
                   result.maneuver_progress <= 1.0,
               "progress defense: zero lateral span remains finite and bounded");
    }

    {
        // manager ownership과 core context를 함께 확인한다: 거부된 KEEP/CANCEL은
        // active AVOID를 계속 계획하고, E-STOP만 preempt/reset한다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        const PlannerCommand avoid =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        planner.PrepareAvoid(ego, initial, boxes_for(initial), avoid,
                             maintenance, prepared);
        auto manager = MakeAvoidManagerReady(100);
        frenet_planner_ros::CommitAvoidAtomically(manager, planner, prepared);
        frenet_planner_ros::RouteAvoidPlanning(manager);
        const double locked = planner.avoid_locked_target_d();

        auto keep = MakeAvoidTransactionCommand(101);
        keep.maneuver_type = asmc_msgs::BehaviorCommand::KEEP;
        keep.execution_request = asmc_msgs::BehaviorCommand::EXECUTE;
        keep.direction = asmc_msgs::BehaviorCommand::DIRECTION_NONE;
        keep.primary_object_id = -1;
        const auto keep_result = manager.Accept(keep);
        if (keep_result.transaction_started) planner.ResetExternalAvoidance();
        CartesianPath keep_cycle;
        Prepare keep_diag;
        const bool keep_plan = planner.PlanCommittedAvoid(
            ego, initial, boxes_for(initial), avoid, keep_cycle, keep_diag);
        Expect(keep_result.planning_target ==
                   frenet_planner_ros::PlanningTarget::ACTIVE_TRANSACTION &&
                   keep_result.result_code ==
                       asmc_msgs::FrenetFeedback::TOO_LATE_TO_CANCEL &&
                   !keep_result.transaction_started &&
                   manager.maneuver_type() ==
                       asmc_msgs::BehaviorCommand::AVOID && keep_plan &&
                   std::fabs(keep_diag.selected_target_d - locked) < 1e-9,
               "post-commit ownership: rejected KEEP continues active AVOID");

        auto cancel = manager.active_command();
        cancel.execution_request = asmc_msgs::BehaviorCommand::CANCEL;
        const auto cancel_result = manager.Accept(cancel);
        if (cancel_result.transaction_started) planner.ResetExternalAvoidance();
        Expect(cancel_result.planning_target ==
                   frenet_planner_ros::PlanningTarget::ACTIVE_TRANSACTION &&
                   cancel_result.result_code ==
                       asmc_msgs::FrenetFeedback::TOO_LATE_TO_CANCEL &&
                   !cancel_result.transaction_cancelled &&
                   !cancel_result.transaction_started &&
                   !frenet_planner_ros::ShouldResetAvoidContext(cancel_result) &&
                   planner.avoid_committed(),
               "post-commit ownership: rejected CANCEL does not reset AVOID");

        manager.SetImplementedManeuver(
            asmc_msgs::BehaviorCommand::EMERGENCY_STOP, true);
        auto emergency = MakeAvoidTransactionCommand(102);
        emergency.maneuver_type =
            asmc_msgs::BehaviorCommand::EMERGENCY_STOP;
        emergency.execution_request = asmc_msgs::BehaviorCommand::EXECUTE;
        emergency.direction = asmc_msgs::BehaviorCommand::DIRECTION_NONE;
        emergency.primary_object_id = -1;
        const auto emergency_result = manager.Accept(emergency);
        PlannerCommand emergency_plan{};
        emergency_plan.mode = EMERGENCY;
        CartesianPath stop;
        const bool stop_ok = planner.Plan(ego, {}, emergency_plan, stop);
        Expect(emergency_result.planning_target ==
                   frenet_planner_ros::PlanningTarget::INCOMING_COMMAND &&
                   manager.maneuver_type() ==
                       asmc_msgs::BehaviorCommand::EMERGENCY_STOP && stop_ok &&
                   !planner.avoid_committed() &&
                   planner.active_avoid_target_id() == -1,
               "post-commit ownership: E-STOP preempts and clears AVOID");
    }

    {
        // D6-3c 착수 probe: node가 manager에서 E-STOP을 accept한 직후 입력
        // stale로 return하면 PlanInternal(EMERGENCY)에 도달하지 못한다.
        const std::vector<ObjectInfo> initial{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare prepared;
        const PlannerCommand avoid =
            MakeAvoidCommand(21, LateralDirection::LEFT);
        planner.PrepareAvoid(ego, initial, boxes_for(initial), avoid,
                             maintenance, prepared);
        auto manager = MakeAvoidManagerReady(103);
        frenet_planner_ros::CommitAvoidAtomically(manager, planner, prepared);
        manager.SetImplementedManeuver(
            asmc_msgs::BehaviorCommand::EMERGENCY_STOP, true);
        auto emergency = MakeAvoidTransactionCommand(104);
        emergency.maneuver_type =
            asmc_msgs::BehaviorCommand::EMERGENCY_STOP;
        emergency.execution_request = asmc_msgs::BehaviorCommand::EXECUTE;
        emergency.direction = asmc_msgs::BehaviorCommand::DIRECTION_NONE;
        emergency.primary_object_id = -1;
        const auto accepted = manager.Accept(emergency);
        if (accepted.transaction_started) planner.ResetExternalAvoidance();

        Expect(!planner.avoid_committed() &&
                   planner.active_avoid_target_id() == -1,
               "D6-3c stale E-STOP hardening: preemption clears core before inputs");
        CartesianPath next_maintenance;
        Prepare next;
        const bool next_ok = planner.PrepareAvoid(
            ego, {}, {}, avoid, next_maintenance, next);
        Expect(!next_ok && planner.avoid_missing_cycles() == 0 &&
                   planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
               "D6-3c stale E-STOP hardening: next AVOID gets no stale grace");
    }

    {
        // ---- ResetExternalAvoidance -------------------------------------
        //
        // CANCEL이 성공하면 manager가 planner를 호출하지 않으므로
        // PlanInternal의 context reset이 일어나지 않는다. 그러면 다음 AVOID가
        // stale context로 유예를 받는다(D5-2와 같은 구조).
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> without_target{
            MakeObstacle(waypoints, 17, kEgoS + 60.0, 0.0)};
        const auto boxes = boxes_for(with_target);
        CartesianPath maintenance;
        Prepare result;

        planner.PrepareAvoid(ego, with_target, boxes,
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        planner.CommitAvoid(result);
        Expect(planner.active_avoid_target_id() == 21 &&
                   planner.avoid_committed(),
               "reset api: the maneuver is bound and committed first");

        planner.ResetExternalAvoidance();
        Expect(planner.active_avoid_target_id() == -1,
               "reset api: the bound target is cleared");
        Expect(!planner.avoid_committed(),
               "reset api: the commit is cleared");
        Expect(planner.avoid_locked_target_d() == 0.0,
               "reset api: the locked target is cleared");
        Expect(planner.avoid_target_count() == 0,
               "reset api: the evaluated target list is cleared");
        Expect(std::string(planner.avoid_phase_name()) == "NONE",
               "reset api: the phase reports NONE");

        // 이제 21이 관측되지 않으면 유예 없이 즉시 LOST여야 한다.
        CartesianPath m2;
        Prepare r2;
        const bool ok = planner.PrepareAvoid(
            ego, without_target, boxes_for(without_target),
            MakeAvoidCommand(21), m2, r2);
        Expect(!ok && planner.last_failure_reason() ==
                          Reason::TARGET_OBJECT_LOST,
               "reset api: a fresh AVOID gets no grace after the reset");
    }
    {
        // reset 없이 같은 순서를 재현하면 stale context가 유예를 준다.
        // 이것이 ResetExternalAvoidance가 필요한 이유의 증거다.
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<ObjectInfo> with_target{
            MakeObstacle(waypoints, 21, kEgoS + 25.0, 0.0)};
        const std::vector<ObjectInfo> without_target{
            MakeObstacle(waypoints, 17, kEgoS + 60.0, 0.0)};
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareAvoid(ego, with_target, boxes_for(with_target),
                             MakeAvoidCommand(21, LateralDirection::LEFT),
                             maintenance, result);
        Expect(planner.active_avoid_target_id() == 21,
               "stale evidence: the target is bound");

        // reset을 하지 않고 대상이 사라진 상태로 넘어간다.
        CartesianPath m2;
        Prepare r2;
        planner.PrepareAvoid(ego, without_target, boxes_for(without_target),
                             MakeAvoidCommand(21), m2, r2);
        Expect(planner.active_avoid_target_id() == 21 &&
                   planner.avoid_missing_cycles() == 1,
               "stale evidence: without a reset the context grants grace");
    }

    if (g_failures != 0) {
        std::cerr << g_failures << " AVOID target check(s) failed\n";
        return 1;
    }
    std::cout << "all AVOID target binding checks passed\n";
    return 0;
}
