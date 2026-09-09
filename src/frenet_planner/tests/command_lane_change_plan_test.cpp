// 외부 LANE_CHANGE의 PREPARE -> READY -> EXECUTE -> COMMITTED -> EXECUTING ->
// HOLD_TARGET_LANE 회귀.
//
// AVOID의 PREPARE/COMMIT/lifecycle transaction 패턴을 재사용한다 -
// command_avoid_target_test.cpp와 같은 fixture 스타일(실제 track waypoint
// 파일 + authoritative OBB)을 쓴다.
//
// LANE_CHANGE가 AVOID와 다른 핵심 지점:
//   - 회피 대상 객체가 없다.
//   - 좌/우가 mode(LANE_CHANGE_LEFT/RIGHT)로 이미 갈리므로 좌/우 후보 경쟁이
//     없다 - 평가할 후보는 항상 하나뿐이다.
//   - 실행 목표(target_d)는 FSM target_pose_map 투영이 아니라 PREPARE를 시작한
//     시점의 실제 ego d에서 lane_width 한 폭만큼 이동한 값이다. 이 테스트의
//     PlannerCommand는 target_pose_map 투영에 대응하는 [A]
//     target_pose_projection_* 필드를 채우지만, 이는 sanity 데이터일 뿐 실행
//     목표를 결정하지 않는다 - 실행 목표는 ego 초기 위치와 lane_width만으로
//     결정된다.
//   - 목표 도달 후 COMPLETED로 latch되지 않는다. HOLD_TARGET_LANE으로
//     전환해 잠긴 target_d를 유지한 채 계속 전진한다.

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

// 경로 위 target_s, 경로 접선 기준 lateral_d만큼 옆으로 이동한 Cartesian
// pose. ego와 장애물 배치 양쪽에 쓴다 - 부호 규약은 FrenetToCartesian의
// n_r(진행 방향 좌측이 양수)과 같다.
Pose2D OffsetPoseAt(const Waypoints& waypoints, double target_s,
                    double lateral_d) {
    const Pose2D pose = PoseAt(waypoints, target_s);
    Pose2D offset = pose;
    offset.x -= lateral_d * std::sin(pose.yaw);
    offset.y += lateral_d * std::cos(pose.yaw);
    return offset;
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
    const Pose2D pose = OffsetPoseAt(waypoints, target_s, lateral_d);
    TrackedObject object;
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

// node가 target_pose_map을 투영해 [A] sanity 필드에 넣어 보낸 상태를
// 흉내낸다. 이 값은 실행 목표가 아니다 - 그냥 채워서 core가 그것을 무시하고
// start_d±lane_width로 target을 정하는지 확인하는 데 쓴다.
PlannerCommand MakeLaneChangeCommand(BehaviorState mode,
                                     double target_pose_s = 0.0,
                                     double target_pose_d = 0.0) {
    PlannerCommand command{};
    command.mode = mode;
    command.has_target_pose_projection = true;
    command.target_pose_projection_s = target_pose_s;
    command.target_pose_projection_d = target_pose_d;
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
    constexpr double kLaneWidth = 3.5;  // planner.hpp의 lane_width_ 기본값과 동일

    // ===== 1/2. LEFT/RIGHT는 각각 start_d + lane_width / start_d - lane_width
    //            방향으로 이동한다 =====================================
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(
            ego, {}, MakeLaneChangeCommand(LANE_CHANGE_LEFT), maintenance,
            result);
        Expect(ok && result.candidate_valid && result.selected_target_d > 0.0,
               "1: LEFT selects a target_d one lane width to the left of "
               "start_d");
        Expect(std::fabs(planner.lane_change_start_d()) < 0.05,
               "1: start_d captures the ego d at PREPARE start (~0 on a "
               "straight lane)");
    }
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(
            ego, {}, MakeLaneChangeCommand(LANE_CHANGE_RIGHT), maintenance,
            result);
        Expect(ok && result.candidate_valid && result.selected_target_d < 0.0,
               "2: RIGHT selects a target_d one lane width to the right of "
               "start_d");
    }

    // start_d가 0이 아닌 경우에도 정확히 한 차로 폭만큼 이동하는지 확인한다
    // (target_d를 ±lane_width로 hard-code하지 않았는지의 핵심 검증).
    {
        FrenetPlanner planner;
        planner.Init(config);
        const CartesianState offset_ego =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, 0.6), kEgoSpeed);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareLaneChange(
            offset_ego, {}, MakeLaneChangeCommand(LANE_CHANGE_LEFT),
            maintenance, result);
        Expect(std::fabs(planner.lane_change_start_d() - 0.6) < 0.2,
               "1b: start_d reflects the actual non-zero ego d at PREPARE "
               "start");
        Expect(std::fabs(result.selected_target_d -
                         (planner.lane_change_start_d() + kLaneWidth)) < 0.6,
               "1b: LEFT target is exactly start_d + lane_width, not a fixed "
               "constant");
    }

    // ===== 3/4. PREPARE candidate evaluates lateral shift feasibility;
    //            PREPARE maintenance stays in the current lane ===========
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(
            ego, {}, MakeLaneChangeCommand(LANE_CHANGE_LEFT), maintenance,
            result);
        Expect(ok && result.candidate_valid,
               "3: PREPARE candidate evaluates lateral shift feasibility");
        const double maintenance_end_d = EndpointD(planner, maintenance, kEgoS);
        Expect(std::isfinite(maintenance_end_d) &&
                   std::fabs(maintenance_end_d) < 1.0,
               "4: PREPARE published maintenance stays in the current lane");
    }

    // ===== 5. WAITING keeps forward motion =============================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<TrackedObject> blockers{
            MakeObstacle(waypoints, 90, kEgoS + 15.0, 0.0, kLaneWidth),
            MakeObstacle(waypoints, 91, kEgoS + 22.0, 0.0, kLaneWidth),
        };
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(
            ego, blockers, MakeLaneChangeCommand(LANE_CHANGE_LEFT),
            maintenance, result);
        Expect(ok && !result.candidate_valid,
               "5a: a fully blocked target lane reports candidate_valid=false "
               "(WAITING)");
        Expect(maintenance.x.size() >= 2 && maintenance.v.front() > 0.1,
               "5b: WAITING still keeps forward motion via rolling "
               "maintenance");
    }

    // ===== 6. COMMIT locks target_d ======================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4001);
        Expect(prepared.candidate_valid &&
                   planner.CanCommitLaneChange(prepared),
               "6a: valid candidate passes commit preflight");
        Expect(frenet_planner_ros::CommitLaneChangeAtomically(
                   manager, planner, prepared),
               "6b: COMMIT locks manager and core together");
        const double locked = planner.lane_change_locked_target_d();
        Expect(std::fabs(locked - prepared.selected_target_d) < 1e-9,
               "6c: locked target_d matches the committed candidate");
    }

    // ===== 7. post-COMMIT replans every cycle from a fresh snapshot ======
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4002);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);

        const double locked = planner.lane_change_locked_target_d();
        bool replanned_each_cycle = true;
        for (int step = 0; step <= 3; ++step) {
            const double d_now = locked * (step / 3.0);
            const CartesianState stepped_ego =
                MakeEgo(OffsetPoseAt(waypoints, kEgoS, d_now), kEgoSpeed);
            CartesianPath step_path;
            Prepare step_result;
            const bool ok = planner.PlanCommittedLaneChange(
                stepped_ego, {}, command, step_path, step_result);
            replanned_each_cycle = replanned_each_cycle && ok &&
                !step_path.x.empty();
        }
        Expect(replanned_each_cycle,
               "7: post-COMMIT replans a fresh trajectory every cycle");
    }

    // ===== 8. SHIFT progress is monotonic ================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4003);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);

        const double locked = planner.lane_change_locked_target_d();
        double last_progress = -1.0;
        bool monotonic = true;
        for (int step = 0; step <= 4; ++step) {
            const double d_now = locked * (step / 4.0);
            const CartesianState stepped_ego =
                MakeEgo(OffsetPoseAt(waypoints, kEgoS, d_now), kEgoSpeed);
            CartesianPath step_path;
            Prepare step_result;
            planner.PlanCommittedLaneChange(stepped_ego, {}, command,
                                            step_path, step_result);
            if (step_result.maneuver_progress + 1e-9 < last_progress) {
                monotonic = false;
            }
            last_progress = step_result.maneuver_progress;
        }
        Expect(monotonic, "8: SHIFT maneuver_progress is monotonic across "
                          "cycles");
    }

    // ===== 9. completion criterion: position + lateral rate + heading ====
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4004);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);

        const double locked = planner.lane_change_locked_target_d();
        const CartesianState arrived =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked), kEgoSpeed);

        CartesianPath final_path;
        Prepare final_result;
        planner.PlanCommittedLaneChange(arrived, {}, command, final_path,
                                        final_result);
        Expect(final_result.reached_target_lane &&
                   final_result.maneuver_progress >= 1.0 - 1e-6,
               "9: reaching target d with a stable heading/rate transitions "
               "to HOLD_TARGET_LANE");
    }

    // ===== 10. target reached -> phase == HOLD_TARGET_LANE ===============
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4005);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double locked = planner.lane_change_locked_target_d();
        const CartesianState arrived =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked), kEgoSpeed);
        CartesianPath final_path;
        Prepare final_result;
        planner.PlanCommittedLaneChange(arrived, {}, command, final_path,
                                        final_result);
        Expect(planner.lane_change_holding_target_lane(),
               "10: FrenetPlanner reports HOLD_TARGET_LANE after arrival");
    }

    // ===== 11. HOLD reports progress == 1.0 ==============================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4006);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double locked = planner.lane_change_locked_target_d();
        const CartesianState arrived =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked), kEgoSpeed);
        CartesianPath final_path;
        Prepare final_result;
        planner.PlanCommittedLaneChange(arrived, {}, command, final_path,
                                        final_result);
        Expect(std::fabs(planner.lane_change_maneuver_progress() - 1.0) <
                   1e-9,
               "11: HOLD_TARGET_LANE reports maneuver_progress == 1.0");
    }

    // ===== 12. HOLD still generates a forward trajectory =================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4007);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double locked = planner.lane_change_locked_target_d();
        const CartesianState arrived =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked), kEgoSpeed);
        CartesianPath final_path;
        Prepare final_result;
        const bool ok = planner.PlanCommittedLaneChange(
            arrived, {}, command, final_path, final_result);
        Expect(ok && final_path.x.size() >= 2 && final_path.v.front() > 0.5,
               "12: HOLD_TARGET_LANE keeps generating a forward-moving "
               "trajectory (not a zero-speed hold)");
    }

    // ===== 13. HOLD keeps d near target_d ================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4008);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double locked = planner.lane_change_locked_target_d();
        // 목표에서 살짝 벗어난 상태(약간의 drift)에서 HOLD가 다시 target_d로
        // 수렴시키는지 확인한다.
        const CartesianState drifted =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked - 0.2), kEgoSpeed);
        Prepare warm;
        CartesianPath warm_path;
        planner.PlanCommittedLaneChange(
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked), kEgoSpeed), {},
            command, warm_path, warm);
        CartesianPath hold_path;
        Prepare hold_result;
        const bool ok = planner.PlanCommittedLaneChange(
            drifted, {}, command, hold_path, hold_result);
        const double end_d = EndpointD(planner, hold_path, kEgoS);
        Expect(ok && std::isfinite(end_d) &&
                   std::fabs(end_d - locked) < 1.0,
               "13: HOLD_TARGET_LANE keeps d converging back to target_d");
    }

    // ===== 14. HOLD never returns to d=0 ==================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4009);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double locked = planner.lane_change_locked_target_d();
        const CartesianState arrived =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked), kEgoSpeed);
        CartesianPath final_path;
        Prepare final_result;
        planner.PlanCommittedLaneChange(arrived, {}, command, final_path,
                                        final_result);
        const double end_d = EndpointD(planner, final_path, kEgoS);
        Expect(std::isfinite(end_d) && std::fabs(end_d) > kLaneWidth * 0.5,
               "14: HOLD_TARGET_LANE trajectory does not fall back toward "
               "d=0");
    }

    // ===== 15. a new transaction does not reuse the previous LC context ==
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand first_command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, first_command, maintenance,
                                  prepared);
        auto manager = MakeLaneChangeManagerReady(4010);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double first_locked = planner.lane_change_locked_target_d();

        // 새 transaction 시작을 흉내낸다: node가 하는 것과 동일하게 명시적
        // reset을 먼저 호출한다(frenet_planner_node.cpp의
        // acceptance.transaction_started 분기 참고).
        planner.ResetExternalLaneChange();

        const CartesianState shifted_ego =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, first_locked), kEgoSpeed);
        CartesianPath second_maintenance;
        Prepare second_prepared;
        planner.PrepareLaneChange(shifted_ego, {}, first_command,
                                  second_maintenance, second_prepared);
        Expect(std::fabs(planner.lane_change_start_d() - first_locked) < 0.3,
               "15: a new transaction captures a fresh start_d instead of "
               "reusing the previous one");
    }

    // ===== 16. pre-COMMIT CANCEL leaves no stale target/context ==========
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command = MakeLaneChangeCommand(LANE_CHANGE_LEFT);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        Expect(prepared.candidate_valid,
               "16: baseline candidate is valid before cancel");

        // pre-COMMIT CANCEL: node의 ShouldResetLaneChangeContext 경로를
        // 흉내내 core context를 명시적으로 비운다.
        planner.ResetExternalLaneChange();

        Expect(!planner.lane_change_committed(),
               "16: cancelled transaction is not committed");
        CartesianPath maintenance_again;
        Prepare fresh;
        planner.PrepareLaneChange(ego, {}, command, maintenance_again, fresh);
        Expect(std::fabs(planner.lane_change_start_d()) < 0.05,
               "16: a fresh PREPARE after cancel recaptures start_d instead "
               "of an old one");
    }

    // ===== NaN/invalid target sanity defense ==============================
    {
        FrenetPlanner planner;
        planner.Init(config);
        PlannerCommand nan_command = MakeLaneChangeCommand(
            LANE_CHANGE_LEFT, kEgoS + 40.0, std::nan(""));
        CartesianPath path;
        const bool ok = planner.Plan(ego, {}, nan_command, path);
        Expect(!ok && planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "NaN target_pose_projection_d is rejected as INVALID_COMMAND");

        PlannerCommand no_projection{};
        no_projection.mode = LANE_CHANGE_RIGHT;
        no_projection.has_target_pose_projection = false;
        CartesianPath path2;
        const bool ok2 = planner.Plan(ego, {}, no_projection, path2);
        Expect(!ok2 &&
                   planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "missing target_pose_projection is rejected as "
               "INVALID_COMMAND");
    }

    if (g_failures == 0) {
        std::cout << "All LANE_CHANGE command plan tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
