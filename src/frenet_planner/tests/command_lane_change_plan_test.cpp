// D7: 외부 LANE_CHANGE의 PREPARE -> READY -> EXECUTE -> COMMITTED ->
// EXECUTING -> COMPLETED 회귀.
//
// AVOID(D6)의 D6-1~D6-4 패턴을 그대로 재사용한다 - command_avoid_target_test.cpp
// 와 같은 fixture 스타일(실제 track waypoint 파일 + authoritative OBB)을 쓴다.
//
// LANE_CHANGE가 AVOID와 다른 점:
//   - 회피 대상 객체가 없다. 목표는 항상 PlannerCommand.has_lane_change_target_d
//     ([A] 필드, node의 target_pose_map 투영에 대응)다.
//   - 좌/우가 mode(LANE_CHANGE_LEFT/RIGHT)로 이미 갈리므로 좌/우 후보 경쟁이
//     없다 - 평가할 후보는 항상 하나뿐이다.
//   - PASS phase가 없다 - 완료 판정은 위치+속도+heading 정렬이다(§17).
//
// 계약: docs/fsm-frenet-interface-v2.md 12.7절(D6 대응), §1~§18

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/Geometry/PolygonUtils.hpp"
#include "frenet_planner/Math/Utils.hpp"
#include "lane_change_commit_coordinator.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

using Reason = FrenetPlanner::PlanFailureReason;
using Prepare = FrenetPlanner::LaneChangePrepareResult;

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

// 경로 위 target_s, 횡오프셋 lateral_d 지점에 놓인 authoritative OBB 장애물.
// command_avoid_target_test.cpp의 MakeObstacle과 동일한 8-corner 구성.
TrackedObject MakeObstacle(const Waypoints& waypoints, int id, double target_s,
                          double speed, double lateral_d = 0.0) {
    const Pose2D pose = PoseAt(waypoints, target_s);
    TrackedObject object;
    object.id = id;
    object.x = pose.x - lateral_d * std::sin(pose.yaw);
    object.y = pose.y + lateral_d * std::cos(pose.yaw);
    object.heading = pose.yaw;
    object.speed = speed;
    object.vx = speed * std::cos(pose.yaw);
    object.vy = speed * std::sin(pose.yaw);
    object.width = 2.0;
    object.length = 4.5;
    object.height = 1.5;

    const double half_length = object.length * 0.5;
    const double half_width = object.width * 0.5;
    const double cosine = std::cos(object.heading);
    const double sine = std::sin(object.heading);
    const double local_xy[4][2] = {
        {-half_length, -half_width},
        { half_length, -half_width},
        { half_length,  half_width},
        {-half_length,  half_width},
    };
    for (std::size_t i = 0; i < 4; ++i) {
        const double corner_x =
            object.x + cosine * local_xy[i][0] - sine * local_xy[i][1];
        const double corner_y =
            object.y + sine * local_xy[i][0] + cosine * local_xy[i][1];
        object.corners[i] = {corner_x, corner_y, 0.0};
        object.corners[i + 4] = {corner_x, corner_y, object.height};
    }
    object.geometry_valid = geometry::ValidateCorners8(object.corners).valid;
    object.z_valid = true;
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

// node가 target_pose_map을 투영해 [A] 필드에 넣어 보낸 상태를 흉내낸다.
// core는 투영 자체를 하지 않는다(계약 3.4절) - 그것은 node 책임이다.
PlannerCommand MakeLaneChangeCommand(BehaviorState mode, double target_s,
                                     double target_d) {
    PlannerCommand command{};
    command.mode = mode;
    command.has_lane_change_target_d = true;
    command.lane_change_target_s = target_s;
    command.lane_change_target_d = target_d;
    command.has_target_speed = true;
    command.target_speed = 6.0;
    return command;
}

asmc_msgs::BehaviorCommand MakeLaneChangeTransactionCommand(uint32_t id) {
    asmc_msgs::BehaviorCommand command;
    command.header.frame_id = "map";
    command.command_id = id;
    command.maneuver_type = asmc_msgs::BehaviorCommand::LANE_CHANGE;
    command.execution_request = asmc_msgs::BehaviorCommand::PREPARE;
    command.direction = asmc_msgs::BehaviorCommand::DIRECTION_LEFT;
    command.merge_type = asmc_msgs::BehaviorCommand::MERGE_NONE;
    command.desired_speed_mps = -1.0;
    command.primary_object_id = -1;
    command.preferred_front_object_id = -1;
    command.preferred_rear_object_id = -1;
    command.desired_time_gap_s = -1.0;
    command.minimum_gap_m = -1.0;
    command.has_target_pose = true;
    command.valid_until = ros::Time(1001.0);
    return command;
}

frenet_planner_ros::CommandExecutionManager MakeLaneChangeManagerReady(
    uint32_t command_id) {
    frenet_planner_ros::CommandExecutionManager manager;
    manager.SetImplementedManeuver(asmc_msgs::BehaviorCommand::LANE_CHANGE,
                                   true);
    manager.Accept(MakeLaneChangeTransactionCommand(command_id));
    manager.NotifyPlanSucceeded();
    return manager;
}

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

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/../config/params.yaml", waypoint_file,
        "command_lane_change_plan_test_params.yaml");
    if (config.empty()) {
        std::cerr << "[FAIL] could not prepare test config\n";
        return 1;
    }

    const Waypoints waypoints = LoadWaypoints(waypoint_file);
    if (waypoints.x.size() < 100) {
        std::cerr << "[FAIL] waypoint load\n";
        return 1;
    }

    constexpr double kEgoS = 200.0;
    constexpr double kEgoSpeed = 6.0;
    const CartesianState ego = MakeEgo(PoseAt(waypoints, kEgoS), kEgoSpeed);
    constexpr double kLaneWidth = 3.5;
    constexpr double kTargetAheadS = kEgoS + 40.0;

    // ===== 1/2. LEFT/RIGHT target projection -> LEFT/RIGHT trajectory ====
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(
            ego, {},
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS, kLaneWidth),
            maintenance, result);
        Expect(ok && result.candidate_valid && result.selected_target_d > 0.0,
               "1: LEFT target projection produces a LEFT (positive d) "
               "candidate");
    }
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(
            ego, {},
            MakeLaneChangeCommand(LANE_CHANGE_RIGHT, kTargetAheadS,
                                  -kLaneWidth),
            maintenance, result);
        Expect(ok && result.candidate_valid && result.selected_target_d < 0.0,
               "2: RIGHT target projection produces a RIGHT (negative d) "
               "candidate");
    }

    // ===== 3/4. final d near selected target; lateral rate stable ========
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareLaneChange(
            ego, {},
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS, kLaneWidth),
            maintenance, result);
        Expect(result.candidate_valid &&
                   std::fabs(EndpointD(planner, result.candidate_path, kEgoS) -
                             result.selected_target_d) < 0.6,
               "3: candidate final d is near the selected target");
        // 종단부 yaw 변화율(연속 두 점의 yaw 차이 / dt 근사)로 lateral
        // stability를 확인한다 - CartesianPath는 d_dot을 직접 노출하지
        // 않으므로(x/y/yaw/kappa/v/a만 있음) yaw curvature 급변 여부로
        // "요동치지 않고 안정적으로 수렴하는가"를 본다.
        const auto& yaw = result.candidate_path.yaw;
        bool terminal_yaw_stable = true;
        if (yaw.size() >= 3) {
            const double d1 = angleDiff(yaw[yaw.size() - 1], yaw[yaw.size() - 2]);
            const double d2 = angleDiff(yaw[yaw.size() - 2], yaw[yaw.size() - 3]);
            terminal_yaw_stable = std::fabs(d1 - d2) < 0.2;
        }
        Expect(terminal_yaw_stable,
               "4: candidate terminal lateral rate is bounded/stable "
               "(no yaw oscillation near the end)");
    }

    // ===== 5/6/7. PREPARE candidate success + maintenance keeps lane =====
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(
            ego, {},
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS, kLaneWidth),
            maintenance, result);
        Expect(ok, "6: PREPARE succeeds and produces a maintenance path");
        Expect(result.candidate_valid, "5: PREPARE candidate succeeds");
        const double maintenance_end_d =
            EndpointD(planner, maintenance, kEgoS);
        Expect(std::isfinite(maintenance_end_d) &&
                   std::fabs(maintenance_end_d) < 1.0,
               "6: PREPARE maintenance stays in the current lane (d~0)");
        Expect(maintenance.x.size() >= 2 &&
                   maintenance.v.back() > 0.5,
               "7: PREPARE maintenance keeps forward motion");
    }

    // ===== 8. LC candidate blocked -> candidate_valid=false, maintenance
    //          keeps rolling ======================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        // 목표 차로 전체를 막는 장애물 - LEFT 후보를 모두 충돌로 탈락시킨다.
        const std::vector<TrackedObject> blockers{
            MakeObstacle(waypoints, 90, kEgoS + 15.0, 0.0, kLaneWidth),
            MakeObstacle(waypoints, 91, kEgoS + 22.0, 0.0, kLaneWidth),
        };
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(
            ego, blockers,
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS, kLaneWidth),
            maintenance, result);
        Expect(ok && !result.candidate_valid,
               "8: blocked LC candidate reports candidate_valid=false");
        Expect(maintenance.x.size() >= 2 && maintenance.v.front() > 0.1,
               "8: maintenance keeps rolling even when candidate is blocked");
    }

    // ===== 9/10. COMMIT locks selected target_d; later request drift
    //             does not change the locked target ===================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4001);
        Expect(prepared.candidate_valid &&
                   planner.CanCommitLaneChange(prepared),
               "9: valid candidate passes commit preflight");
        Expect(frenet_planner_ros::CommitLaneChangeAtomically(
                   manager, planner, prepared),
               "9: COMMIT locks manager and core together");
        const double locked = planner.lane_change_locked_target_d();
        Expect(std::fabs(locked - prepared.selected_target_d) < 1e-9,
               "9: locked target_d matches the committed candidate");

        // 10: COMMIT 이후 다른 요청 target으로 흔들리지 않는다.
        const PlannerCommand drifted_request = MakeLaneChangeCommand(
            LANE_CHANGE_LEFT, kTargetAheadS, kLaneWidth + 2.0);
        CartesianPath next_path;
        Prepare next;
        planner.PlanCommittedLaneChange(ego, {}, drifted_request, next_path,
                                        next);
        Expect(std::fabs(planner.lane_change_locked_target_d() - locked) <
                   1e-9,
               "10: post-COMMIT request drift does not change the locked "
               "target");
    }

    // ===== 11/12. post-COMMIT replans every cycle; progress monotonic ===
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4002);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);

        double last_progress = -1.0;
        bool monotonic = true;
        bool replanned_each_cycle = true;
        // 차량이 목표 쪽으로 조금씩 이동하는 여러 snapshot을 시뮬레이션한다.
        for (int step = 0; step <= 4; ++step) {
            const double d_now = (kLaneWidth) * (step / 4.0);
            CartesianState stepped_ego = ego;
            const Pose2D pose = PoseAt(waypoints, kEgoS);
            stepped_ego.x = pose.x - d_now * std::sin(pose.yaw);
            stepped_ego.y = pose.y + d_now * std::cos(pose.yaw);
            CartesianPath step_path;
            Prepare step_result;
            const bool ok = planner.PlanCommittedLaneChange(
                stepped_ego, {}, command, step_path, step_result);
            replanned_each_cycle = replanned_each_cycle && ok &&
                !step_path.x.empty();
            if (step_result.maneuver_progress + 1e-9 < last_progress) {
                monotonic = false;
            }
            last_progress = step_result.maneuver_progress;
        }
        Expect(replanned_each_cycle,
               "11: post-COMMIT replans a fresh trajectory every cycle");
        Expect(monotonic, "12: maneuver_progress is monotonic across cycles");
    }

    // ===== 13. completion criterion ====================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4003);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);

        // ego를 목표 d/heading 근방(도착 상태)에 정확히 놓는다.
        const Pose2D pose = PoseAt(waypoints, kEgoS);
        CartesianState arrived = ego;
        const double locked = planner.lane_change_locked_target_d();
        arrived.x = pose.x - locked * std::sin(pose.yaw);
        arrived.y = pose.y + locked * std::cos(pose.yaw);
        arrived.yaw = pose.yaw;

        CartesianPath final_path;
        Prepare final_result;
        planner.PlanCommittedLaneChange(arrived, {}, command, final_path,
                                        final_result);
        Expect(final_result.maneuver_completed &&
                   final_result.maneuver_progress >= 1.0 - 1e-6,
               "13: reaching target d/heading with stable rate completes "
               "the maneuver");
    }

    // ===== 14. LEFT/RIGHT vs projected target direction mismatch =========
    {
        // LEFT를 요청했는데 투영된 목표가 오른쪽(음수 delta_d)인 상태를
        // core 레벨에서 흉내낸다 - node의 delta_d 검증은 별도(node 코드
        // 참고)이므로, 여기서는 core가 방향 모순 자체로 후보를 막지 않고
        // (AVOID의 direction_conflict와 달리 core에는 방향 필드가 없음)
        // node 레벨 검증에 의존한다는 계약을 문서화한다. 이 테스트는
        // core가 최소한 물리적으로 반대 방향 후보를 만들지 않는지 확인한다.
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare result;
        // RIGHT 목표(-lane_width)를 LANE_CHANGE_LEFT mode로 보낸다 - node가
        // 이런 조합을 만들지 않지만(위 direction consistency 검증), core가
        // 방어적으로 요청받은 target_d를 그대로 겨냥하는지 확인한다.
        planner.PrepareLaneChange(
            ego, {},
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS,
                                  -kLaneWidth),
            maintenance, result);
        Expect(!result.candidate_valid ||
                   result.selected_target_d < 0.0,
               "14: core targets exactly what target_pose projection "
               "specifies (direction mismatch is a node-level guard, not "
               "silently corrected by core)");
    }

    // ===== 15. NaN/invalid target defense ================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        PlannerCommand nan_command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kTargetAheadS,
                                  std::nan(""));
        CartesianPath path;
        const bool ok = planner.Plan(ego, {}, nan_command, path);
        Expect(!ok && planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "15: NaN lane change target is rejected as INVALID_COMMAND");

        PlannerCommand no_target{};
        no_target.mode = LANE_CHANGE_RIGHT;
        no_target.has_lane_change_target_d = false;
        CartesianPath path2;
        const bool ok2 = planner.Plan(ego, {}, no_target, path2);
        Expect(!ok2 &&
                   planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "15: missing lane change target projection is rejected as "
               "INVALID_COMMAND");
    }

    if (g_failures == 0) {
        std::cout << "All LANE_CHANGE command plan tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
