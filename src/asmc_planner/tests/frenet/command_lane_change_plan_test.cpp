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
//   - 실행 목표(target_d)는 node가 target_pose_map을 현재 RefLine에 투영해
//     방향/heading sanity 검증까지 마친 값이다(command.target_pose_projection_d).
//     이 테스트는 node의 투영을 흉내내어 PlannerCommand의
//     target_pose_projection_*를 직접 채운다 - core는 그 값을 그대로
//     execution target_d로 신뢰하며, lane_width 기반 근사값으로 대체하지
//     않는다.
//   - Behavior가 WHERE(목표 차로 중심)를 정하고 Frenet은 HOW(어떻게 그
//     목표에 도달할지: duration/충돌회피/kinematic feasibility/cost)만
//     최적화한다 - PREPARE/COMMIT/SHIFT/HOLD 전 구간에서 종단 lateral
//     target은 절대 값이 갈리지 않는다. 그래서 target_d 일치 검증은 느슨한
//     허용오차가 아니라 부동소수점 수준의 정확한 값(1e-6)으로 한다.
//   - 목표 도달 후 COMPLETED로 latch되지 않는다. HOLD_TARGET_LANE으로
//     전환해 잠긴 target_d를 유지한 채 계속 전진한다.

#include "Frenet/frenet_planner.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include "Frenet/Math/Utils.hpp"
#include "Planning/Compatibility/lane_change_commit_coordinator.hpp"

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
// pose. ego와 장애물 배치, 그리고 target_pose_map을 흉내내는 데 공통으로
// 쓴다 - 부호 규약은 FrenetToCartesian의 n_r(진행 방향 좌측이 양수)과 같다.
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

// node가 target_pose_map을 투영해 [A] 필드에 실어 보낸 상태를 흉내낸다.
// 이 값이 그대로 core의 execution target_d가 된다 - lane_width로 대체되지
// 않는지 확인하는 데 쓴다.
PlannerCommand MakeLaneChangeCommand(BehaviorState mode,
                                     double target_pose_s,
                                     double target_pose_d) {
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
        package_src + "/../config/frenet/params.yaml", waypoint_file,
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
    constexpr double kLaneWidth = 3.5;  // planner.hpp의 lane_width_ 기본값
    // target_d 일치 검증에 쓰는 허용오차. 종단 d는 quintic 경계조건 자체이므로
    // (MakeQuintic(..., d1=target_d, ..., T)) 부동소수점 오차 이상으로 벌어질
    // 이유가 없다 - 느슨한 ±0.5m 허용은 PREPARE가 target_d ± tolerance 범위에서
    // 값을 골라도 테스트를 통과시켜, 정확히 이 exact-target 회귀를 놓친다.
    constexpr double kExactTargetEpsilon = 1e-6;

    // ===== [A] AUTHORITATIVE TARGET GEOMETRY ============================

    // 1. LEFT: projected target이 그대로, 정확히 쓰인다(고정 lane_width가
    //    아니고, PREPARE의 전역 lateral_target_tolerance로 흔들리지도 않는다).
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        constexpr double kProjectedD = 3.72;  // != start_d(~=0) + 3.5
        const PlannerCommand command = MakeLaneChangeCommand(
            LANE_CHANGE_LEFT, kEgoS, kProjectedD);
        CartesianPath maintenance;
        Prepare result;
        const bool ok =
            planner.PrepareLaneChange(ego, {}, command, maintenance, result);
        Expect(ok && result.candidate_valid &&
                   std::fabs(result.selected_target_d - kProjectedD) <
                       kExactTargetEpsilon,
               "1: LEFT PREPARE selected_target_d exactly equals the "
               "projected FSM target_d (tight tolerance, not +-0.5m)");
        Expect(std::fabs(result.selected_target_d -
                         (planner.lane_change_start_d() + kLaneWidth)) > 0.05,
               "1: LEFT target is not start_d + nominal lane_width");
    }

    // 2. RIGHT: projected target이 그대로, 정확히 쓰인다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        constexpr double kProjectedD = -3.66;
        const PlannerCommand command = MakeLaneChangeCommand(
            LANE_CHANGE_RIGHT, kEgoS, kProjectedD);
        CartesianPath maintenance;
        Prepare result;
        const bool ok =
            planner.PrepareLaneChange(ego, {}, command, maintenance, result);
        Expect(ok && result.candidate_valid &&
                   std::fabs(result.selected_target_d - kProjectedD) <
                       kExactTargetEpsilon,
               "2: RIGHT PREPARE selected_target_d exactly equals the "
               "projected FSM target_d (tight tolerance, not +-0.5m)");
        Expect(std::fabs(result.selected_target_d -
                         (planner.lane_change_start_d() - kLaneWidth)) > 0.05,
               "2: RIGHT target is not start_d - nominal lane_width");
    }

    // 3. ego d가 0이 아닌 경우에도 투영값을 정확히 그대로 쓰는지 확인한다
    //    (start_d + lane_width로 우연히 근사되는 경우도, target ± tolerance로
    //    흔들리는 경우도 모두 배제).
    {
        FrenetPlanner planner;
        planner.Init(config);
        const CartesianState offset_ego =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, 0.6), kEgoSpeed);
        constexpr double kProjectedD = 3.9;  // 0.6 + 3.5와 다른 값
        const PlannerCommand command = MakeLaneChangeCommand(
            LANE_CHANGE_LEFT, kEgoS, kProjectedD);
        CartesianPath maintenance;
        Prepare result;
        planner.PrepareLaneChange(offset_ego, {}, command, maintenance,
                                  result);
        Expect(std::fabs(result.selected_target_d - kProjectedD) <
                   kExactTargetEpsilon,
               "3: non-zero start_d still uses the projected target_d "
               "exactly, not start_d + lane_width and not target +- 0.5m");
    }

    // 3. target geometry가 없으면 lane_width로 fallback하지 않고 실패한다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        PlannerCommand no_projection{};
        no_projection.mode = LANE_CHANGE_LEFT;
        no_projection.has_target_pose_projection = false;
        CartesianPath path;
        const bool ok = planner.Plan(ego, {}, no_projection, path);
        Expect(!ok && planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "4: missing target geometry fails safely, no lane_width "
               "fallback");
    }

    // ===== [B] VALIDATION =================================================

    // 5/6. delta_d 기준 반대쪽 target은 ValidateExternalCommand 이전 단계
    // (node)에서 거부되어야 하지만, core도 델타 부호를 스스로 강제하지
    // 않는다는 것만 여기서 확인한다 - core는 command가 준 값을 신뢰하되,
    // NaN/Inf 방어는 core 레벨에서 한다(아래 7/8). 실제 "반대쪽 lateral
    // side" 거부는 node의 delta_d sanity 검증(frenet_planner_node.cpp)
    // 책임이며 이 unit test의 범위 밖이다(node 레벨 회귀는
    // v2_ros_regression_test 몫).

    // 7. 투영 실패/NaN target은 안전하게 실패한다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand nan_command = MakeLaneChangeCommand(
            LANE_CHANGE_LEFT, kEgoS + 40.0, std::nan(""));
        CartesianPath path;
        const bool ok = planner.Plan(ego, {}, nan_command, path);
        Expect(!ok && planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "7: NaN target_pose_projection_d is rejected as "
               "INVALID_COMMAND");
    }

    // 8. has_target_pose_projection=false는 명령 실패로 수렴한다(heading
    // 검증 자체는 node 책임이므로 core 레벨에서는 이 계약만 확인한다).
    {
        FrenetPlanner planner;
        planner.Init(config);
        PlannerCommand no_projection{};
        no_projection.mode = LANE_CHANGE_RIGHT;
        no_projection.has_target_pose_projection = false;
        CartesianPath path2;
        const bool ok2 = planner.Plan(ego, {}, no_projection, path2);
        Expect(!ok2 &&
                   planner.last_failure_reason() == Reason::INVALID_COMMAND,
               "8: missing target_pose_projection is rejected as "
               "INVALID_COMMAND");
    }

    // ===== [C] PREPARE =====================================================

    // 9. PREPARE candidate evaluates lateral shift feasibility; PREPARE
    //    maintenance stays in the current lane (candidate not published).
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare result;
        const bool ok =
            planner.PrepareLaneChange(ego, {}, command, maintenance, result);
        Expect(ok && result.candidate_valid,
               "9a: PREPARE candidate evaluates lateral shift feasibility");
        const double maintenance_end_d = EndpointD(planner, maintenance, kEgoS);
        Expect(std::isfinite(maintenance_end_d) &&
                   std::fabs(maintenance_end_d) < 1.0,
               "9b: PREPARE published maintenance stays in the current lane");
    }

    // 10. 여러 time horizon(T) 후보가 평가돼도 모든 유효 LC candidate의 종단
    //     target d는 같은 authoritative target_d다 - Frenet은 duration만
    //     최적화하고 lateral 목적지 자체를 흔들지 않는다는 exact-target
    //     ownership contract의 직접 검증이다. path_cfg_.time_horizon이 여러
    //     T를 만든다는 전제이므로, 생성된 후보가 1개뿐이면(단일 T 설정) 이
    //     확인은 자명하게 통과한다.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare result;
        const bool ok =
            planner.PrepareLaneChange(ego, {}, command, maintenance, result);
        Expect(ok && result.candidate_valid &&
                   std::fabs(result.selected_target_d - kLaneWidth) <
                       kExactTargetEpsilon,
               "10: every evaluated LC candidate's terminal target is the "
               "same authoritative target_d regardless of time horizon");
    }

    // 11. WAITING keeps forward motion when the candidate is blocked.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const std::vector<TrackedObject> blockers{
            MakeObstacle(waypoints, 90, kEgoS + 15.0, 0.0, kLaneWidth),
            MakeObstacle(waypoints, 91, kEgoS + 22.0, 0.0, kLaneWidth),
        };
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(ego, blockers, command,
                                                   maintenance, result);
        Expect(ok && !result.candidate_valid,
               "11a: a fully blocked target lane reports "
               "candidate_valid=false (WAITING)");
        Expect(maintenance.x.size() >= 2 && maintenance.v.front() > 0.1,
               "11b: WAITING still keeps forward motion via rolling "
               "maintenance");
    }

    // ===== [D] LOW SPEED ===================================================

    // 12. a very-low-speed PREPARE cannot report candidate_valid=true solely
    //     because a current-lane/centerline candidate exists.
    //     GenerateLateralCandidates ignores the requested lateral_d1 grid
    //     below kLowSpeedThreshold and instead offers only start.d/0.0
    //     candidates(수학적 특이점 회피) - none of those equal the requested
    //     LC target_d, so the exact-match guard in LaneChangeProcess must
    //     reject them rather than reporting a false READY.
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianState slow_ego = ego;
        slow_ego.v = 0.2;  // < kLowSpeedFallbackV(0.5 m/s)
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare result;
        const bool ok = planner.PrepareLaneChange(slow_ego, {}, command,
                                                   maintenance, result);
        Expect(ok, "12a: low-speed PREPARE still publishes safe "
                   "maintenance");
        Expect(!result.candidate_valid,
               "12b: low-speed PREPARE does not report candidate_valid=true "
               "from a centerline/current-lane candidate that never reaches "
               "target_d");
        Expect(result.valid_candidate_count == 0,
               "12c: valid_candidate_count only counts candidates that "
               "exactly reach target_d, not generic-valid current-lane/"
               "centerline candidates that never reach it");
        Expect(maintenance.x.size() >= 2,
               "12d: maintenance behavior remains safe (non-empty rolling "
               "path) even while the LC candidate is rejected");
    }

    // ===== [E] COMMIT ======================================================

    // 13/14. COMMIT locks the projected target_d, and the locked value
    //        equals the ORIGINAL projected target directly (not merely
    //        prepared.selected_target_d, which would not catch PREPARE
    //        target drift).
    {
        FrenetPlanner planner;
        planner.Init(config);
        constexpr double kProjectedD = kLaneWidth;
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kProjectedD);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4001);
        Expect(prepared.candidate_valid &&
                   planner.CanCommitLaneChange(prepared),
               "13: valid candidate passes commit preflight (EXECUTE "
               "revalidation)");
        Expect(frenet_planner_ros::CommitLaneChangeAtomically(
                   manager, planner, prepared),
               "14a: COMMIT locks manager and core together");
        const double locked = planner.lane_change_locked_target_d();
        Expect(std::fabs(locked - prepared.selected_target_d) < 1e-9,
               "14b: locked target_d matches the committed candidate");
        Expect(std::fabs(locked - command.target_pose_projection_d) <
                   kExactTargetEpsilon,
               "14c: locked target_d equals the ORIGINAL projected "
               "command.target_pose_projection_d directly, not just the "
               "PREPARE-selected candidate");
    }

    // 15. after COMMIT, changing current ego d does not move the locked
    //     target.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4002);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double locked_before = planner.lane_change_locked_target_d();

        const CartesianState drifted_ego =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked_before * 0.3),
                    kEgoSpeed);
        CartesianPath step_path;
        Prepare step_result;
        planner.PlanCommittedLaneChange(drifted_ego, {}, command, step_path,
                                        step_result);
        Expect(std::fabs(planner.lane_change_locked_target_d() -
                         locked_before) < 1e-9,
               "15: changing current ego d does not move the locked target");
    }

    // 16. after COMMIT, a rejected/alternate incoming target pose does not
    //     mutate the locked active transaction target (PrepareExternalLaneChangeTarget
    //     early-returns true for the committed branch without re-reading
    //     command.target_pose_projection_d).
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4003);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double locked_before = planner.lane_change_locked_target_d();

        const PlannerCommand alternate_command = MakeLaneChangeCommand(
            LANE_CHANGE_LEFT, kEgoS, kLaneWidth + 5.0);
        CartesianPath step_path;
        Prepare step_result;
        planner.PlanCommittedLaneChange(ego, {}, alternate_command, step_path,
                                        step_result);
        Expect(std::fabs(planner.lane_change_locked_target_d() -
                         locked_before) < 1e-9,
               "16: an alternate incoming target pose does not mutate the "
               "locked active transaction target");
    }

    // ===== [F] SHIFT / HOLD ================================================

    // 17. post-COMMIT replans every cycle from a fresh snapshot.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4004);
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
               "17: post-COMMIT replans a fresh trajectory every cycle");
    }

    // 18. SHIFT converges toward the locked projected target_d (monotonic
    //     progress).
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        auto manager = MakeLaneChangeManagerReady(4005);
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
        Expect(monotonic, "18: SHIFT maneuver_progress converges "
                          "monotonically toward the locked target_d");
    }

    // 19. target-lane tolerances cause SHIFT -> HOLD_TARGET_LANE.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
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
        Expect(final_result.reached_target_lane &&
                   planner.lane_change_holding_target_lane(),
               "19: reaching target_d with a stable heading/rate transitions "
               "SHIFT -> HOLD_TARGET_LANE");
    }

    // 20. HOLD: progress==1.0, stays EXECUTING(diagnostic-level: phase stays
    //     HOLD_TARGET_LANE, not re-armed to SHIFT), forward-moving path
    //     continues at locked target_d, no snap back toward d=0.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
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
        Expect(std::fabs(planner.lane_change_maneuver_progress() - 1.0) <
                   1e-9,
               "20a: HOLD_TARGET_LANE reports maneuver_progress == 1.0");
        Expect(ok && final_path.x.size() >= 2 && final_path.v.front() > 0.5,
               "20b: HOLD_TARGET_LANE keeps generating a forward-moving "
               "trajectory (not a zero-speed hold)");
        const double end_d = EndpointD(planner, final_path, kEgoS);
        Expect(std::isfinite(end_d) && std::fabs(end_d) > kLaneWidth * 0.5,
               "20c: HOLD_TARGET_LANE trajectory does not fall back toward "
               "d=0");
        Expect(std::isfinite(end_d) &&
                   std::fabs(end_d - locked) < kExactTargetEpsilon,
               "20d: HOLD_TARGET_LANE keeps the exact locked target_d, not "
               "an approximate one");

        // 살짝 벗어난 상태(drift)에서 HOLD가 다시 정확한 target_d로
        // 수렴시키는지도 함께 확인한다.
        const CartesianState drifted =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, locked - 0.2), kEgoSpeed);
        CartesianPath hold_path;
        Prepare hold_result;
        const bool hold_ok = planner.PlanCommittedLaneChange(
            drifted, {}, command, hold_path, hold_result);
        const double hold_end_d = EndpointD(planner, hold_path, kEgoS);
        Expect(hold_ok && std::isfinite(hold_end_d) &&
                   std::fabs(hold_end_d - locked) < kExactTargetEpsilon,
               "20e: HOLD_TARGET_LANE keeps d converging back to the exact "
               "target_d after drift");
    }

    // ===== [G] RESET / TRANSACTION =========================================

    // 21. a new LC transaction clears stale LC context.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand first_command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, first_command, maintenance,
                                  prepared);
        auto manager = MakeLaneChangeManagerReady(4008);
        frenet_planner_ros::CommitLaneChangeAtomically(manager, planner,
                                                       prepared);
        const double first_locked = planner.lane_change_locked_target_d();

        // 새 transaction 시작을 흉내낸다: node가 하는 것과 동일하게 명시적
        // reset을 먼저 호출한다(frenet_planner_node.cpp의
        // acceptance.transaction_started 분기 참고).
        planner.ResetExternalLaneChange();

        const CartesianState shifted_ego =
            MakeEgo(OffsetPoseAt(waypoints, kEgoS, first_locked), kEgoSpeed);
        constexpr double kSecondProjectedD = kLaneWidth + 2.0;
        const PlannerCommand second_command = MakeLaneChangeCommand(
            LANE_CHANGE_LEFT, kEgoS, first_locked + kSecondProjectedD);
        CartesianPath second_maintenance;
        Prepare second_prepared;
        planner.PrepareLaneChange(shifted_ego, {}, second_command,
                                  second_maintenance, second_prepared);
        Expect(std::fabs(second_prepared.selected_target_d -
                         (first_locked + kSecondProjectedD)) <
                   kExactTargetEpsilon,
               "21: a new transaction captures the new command's exact "
               "target_d instead of reusing the previous locked one");
    }

    // 22. successful pre-COMMIT CANCEL clears LC context.
    {
        FrenetPlanner planner;
        planner.Init(config);
        const PlannerCommand command =
            MakeLaneChangeCommand(LANE_CHANGE_LEFT, kEgoS, kLaneWidth);
        CartesianPath maintenance;
        Prepare prepared;
        planner.PrepareLaneChange(ego, {}, command, maintenance, prepared);
        Expect(prepared.candidate_valid,
               "22a: baseline candidate is valid before cancel");

        // pre-COMMIT CANCEL: node의 ShouldResetLaneChangeContext 경로를
        // 흉내내 core context를 명시적으로 비운다.
        planner.ResetExternalLaneChange();

        Expect(!planner.lane_change_committed(),
               "22b: cancelled transaction is not committed");
        CartesianPath maintenance_again;
        Prepare fresh;
        planner.PrepareLaneChange(ego, {}, command, maintenance_again, fresh);
        Expect(std::fabs(fresh.selected_target_d - kLaneWidth) <
                   kExactTargetEpsilon,
               "22c: a fresh PREPARE after cancel recaptures the exact "
               "target_d from the current command instead of an old one");
    }

    if (g_failures == 0) {
        std::cout << "All LANE_CHANGE command plan tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
