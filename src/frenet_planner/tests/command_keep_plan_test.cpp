#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

CartesianState LoadPoseNearArcLength(const std::string& path,
                                     double target_s,
                                     double speed) {
    std::ifstream input(path);
    std::vector<double> xs;
    std::vector<double> ys;
    double x = 0.0;
    double y = 0.0;
    double ignored = 0.0;
    while (input >> x >> y >> ignored) {
        xs.push_back(x);
        ys.push_back(y);
    }

    std::size_t index = 1;
    double s = 0.0;
    for (; index < xs.size(); ++index) {
        s += std::hypot(xs[index] - xs[index - 1],
                        ys[index] - ys[index - 1]);
        if (s >= target_s) break;
    }
    if (index >= xs.size()) index = xs.size() - 1;

    CartesianState pose{};
    pose.x = xs[index];
    pose.y = ys[index];
    pose.yaw = std::atan2(ys[index] - ys[index - 1],
                          xs[index] - xs[index - 1]);
    pose.v = speed;
    return pose;
}

CartesianState OffsetPose(const CartesianState& pose, double lateral_offset) {
    CartesianState shifted = pose;
    shifted.x += -std::sin(pose.yaw) * lateral_offset;
    shifted.y += std::cos(pose.yaw) * lateral_offset;
    return shifted;
}

bool IsCartesianTrajectoryTimeConsistent(const CartesianPath& path,
                                         double dt) {
    const std::size_t count = std::min(
        {path.x.size(), path.y.size(), path.v.size()});
    if (count < 2) return false;
    double geometric_distance = 0.0;
    double integrated_speed_distance = 0.0;
    for (std::size_t i = 1; i < count; ++i) {
        if (!std::isfinite(path.x[i]) || !std::isfinite(path.y[i]) ||
            !std::isfinite(path.v[i])) {
            return false;
        }
        geometric_distance += std::hypot(path.x[i] - path.x[i - 1],
                                         path.y[i] - path.y[i - 1]);
        integrated_speed_distance +=
            0.5 * (std::fabs(path.v[i - 1]) + std::fabs(path.v[i])) * dt;
    }
    const double scale = std::max(1.0, geometric_distance);
    return std::fabs(geometric_distance - integrated_speed_distance) /
               scale < 0.15;
}

// authoritative 8-corner OBB(perception의 obb_corners와 같은 형태)를
// 직접 채운 객체. production은 더 이상 center/width/length로 geometry를
// 재생성하지 않으므로, corners 없는 객체는 애초에 유효하지 않다.
TrackedObject MakeAuthoritativeBox(int id, double x, double y) {
    TrackedObject obstacle;
    obstacle.id = id;
    obstacle.x = x;
    obstacle.y = y;
    obstacle.z = 1.0;
    obstacle.width = 2.0;
    obstacle.length = 4.0;
    obstacle.height = 2.0;
    obstacle.geometry_valid = true;
    obstacle.z_valid = true;
    const double xy[4][2] = {
        {x - 2.0, y - 1.0}, {x + 2.0, y - 1.0},
        {x + 2.0, y + 1.0}, {x - 2.0, y + 1.0},
    };
    for (std::size_t i = 0; i < 4; ++i) {
        obstacle.corners[i] = {xy[i][0], xy[i][1], 0.0};
        obstacle.corners[i + 4] = {xy[i][0], xy[i][1], 2.0};
    }
    return obstacle;
}

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string config = package_src + "/../config/params.yaml";
    const std::string waypoints = FRENET_TEST_WAYPOINT_FILE;

    FrenetPlanner planner;
    if (!planner.Init(config)) {
        std::cerr << "[FAIL] planner initialization\n";
        return 1;
    }

    // VehicleState 실측 문제 구간(s≈680~745)의 representative regression.
    // reference curvature만으로는 허용되지만 d!=0에서 중앙으로 복귀하는
    // actual Cartesian candidate의 곡률이 더 큰 조건을 여러 지점/양쪽 d에서
    // 찾는다. bag 숫자를 하드코딩하지 않고 production global path 자체를
    // 사용한다.
    bool found_recenter_regeneration = false;
    bool recenter_plan_remained_valid = false;
    bool recenter_trajectory_consistent = false;
    bool recenter_candidate_cap_applied = false;
    double representative_s = -1.0;
    double representative_d = 0.0;
    for (double sample_s : {680.0, 700.0, 720.0, 740.0}) {
        for (double sample_d : {-1.5, 1.5}) {
            FrenetPlanner recenter_planner;
            if (!recenter_planner.Init(config)) continue;
            CartesianState recenter_ego = OffsetPose(
                LoadPoseNearArcLength(waypoints, sample_s, 9.5), sample_d);
            PlannerCommand recenter_keep{};
            recenter_keep.mode = LANE_KEEPING;
            recenter_keep.has_target_speed = true;
            recenter_keep.target_speed = 10.0;
            CartesianPath recenter_path;
            const bool recenter_ok = recenter_planner.Plan(
                recenter_ego, {}, recenter_keep, recenter_path);
            if (recenter_planner.last_candidate_aware_regenerations() <= 0) {
                continue;
            }

            found_recenter_regeneration = true;
            recenter_plan_remained_valid = recenter_ok && !recenter_path.x.empty();
            recenter_trajectory_consistent = recenter_ok &&
                IsCartesianTrajectoryTimeConsistent(
                    recenter_path, recenter_planner.sample_dt());
            recenter_candidate_cap_applied = recenter_ok &&
                recenter_planner.last_raw_speed_cap() <
                    recenter_planner.last_base_target_speed() - 1e-6 &&
                std::fabs(recenter_planner.last_target_speed() -
                          recenter_planner.last_raw_speed_cap()) < 1e-9;
            representative_s = sample_s;
            representative_d = sample_d;
            std::cout << "  recenter representative s=" << sample_s
                      << " d=" << sample_d
                      << " base=" << recenter_planner.last_base_target_speed()
                      << " cap="
                      << recenter_planner.last_candidate_curvature_speed_cap()
                      << " final=" << recenter_planner.last_target_speed()
                      << " raw=" << recenter_planner.last_raw_speed_cap()
                      << " max_kappa="
                      << recenter_planner.last_candidate_max_abs_curvature()
                      << " max_lat="
                      << recenter_planner.last_candidate_max_lateral_accel()
                      << " regenerations="
                      << recenter_planner.last_candidate_aware_regenerations()
                      << '\n';
            break;
        }
        if (found_recenter_regeneration) break;
    }
    Expect(found_recenter_regeneration,
           "production s=680~745 lateral recenter lowers the reference-only "
           "KEEP target");
    Expect(recenter_plan_remained_valid,
           "candidate-aware slowdown keeps a valid KEEP trajectory (no "
           "candidate starvation)");
    Expect(recenter_trajectory_consistent,
           "regenerated KEEP x/y and velocity remain time-consistent");
    Expect(recenter_candidate_cap_applied,
           "I candidate curvature lower cap reaches the generator in the same "
           "planning cycle");

    // 외부 속도 상한은 candidate-aware logic이 절대 올리지 않는다.
    if (representative_s >= 0.0) {
        FrenetPlanner capped_planner;
        const bool capped_init = capped_planner.Init(config);
        CartesianState capped_ego = OffsetPose(
            LoadPoseNearArcLength(waypoints, representative_s, 9.5),
            representative_d);
        PlannerCommand capped_keep{};
        capped_keep.mode = LANE_KEEPING;
        capped_keep.has_target_speed = true;
        capped_keep.target_speed = 8.33;
        CartesianPath capped_path;
        const bool capped_ok = capped_init && capped_planner.Plan(
            capped_ego, {}, capped_keep, capped_path);
        Expect(capped_ok &&
                   capped_planner.last_base_target_speed() <= 8.33 + 1e-9 &&
                   capped_planner.last_raw_speed_cap() <= 8.33 + 1e-9,
               "candidate-aware KEEP never raises Behavior's external raw "
               "8.33m/s cap");
        Expect(capped_ok && std::fabs(capped_planner.last_target_speed() -
                                      capped_planner.last_raw_speed_cap()) < 1e-9,
               "new external cap is the immediate generator target while the "
               "trajectory retains physical continuity");
    }

    // Roundabout MERGE conflict_s(885.76m)의 approach window 안쪽이다.
    const CartesianState ego = LoadPoseNearArcLength(waypoints, 870.0, 5.0);
    const std::vector<TrackedObject> no_objects;
    CartesianPath path;

    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    const bool keep_ok = planner.Plan(ego, no_objects, keep, path);
    Expect(keep_ok && planner.last_mode() == LANE_KEEPING && !path.x.empty(),
           "external KEEP plans normally inside the roundabout merge-zone window");

    const std::vector<TrackedObject> outside_objects = {
        MakeAuthoritativeBox(31, ego.x + 100.0, ego.y + 100.0)};
    Expect(planner.Plan(ego, outside_objects, keep, path) && !path.x.empty(),
           "authoritative OBB outside path preserves KEEP trajectory");

    const std::vector<TrackedObject> blocking_objects = {
        MakeAuthoritativeBox(32, ego.x, ego.y)};
    Expect(!planner.Plan(ego, blocking_objects, keep, path) &&
               planner.last_failure_reason() ==
                   FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED,
           "authoritative OBB on ego path blocks all candidates through SAT");

    keep.has_target_speed = true;
    keep.target_speed = 15.0;
    const bool desired_ok = planner.Plan(ego, no_objects, keep, path);
    Expect(desired_ok && planner.last_target_speed() <= 10.0 + 1e-9,
           "desired speed cannot exceed planner curve-safe speed");

    keep.has_target_speed = false;
    keep.has_max_speed = true;
    keep.max_speed = 3.0;
    const bool max_ok = planner.Plan(ego, no_objects, keep, path);
    Expect(max_ok && planner.last_base_target_speed() <= 3.0 + 1e-9 &&
               planner.last_raw_speed_cap() <= 3.0 + 1e-9,
           "max-only command caps the raw planner speed constraint");
    Expect(max_ok && std::fabs(planner.last_target_speed() -
                               planner.last_raw_speed_cap()) < 1e-9,
           "max-only downward cap reaches the generator in the same cycle");

    // Trajectory-level continuity: the scalar target changes immediately, but
    // the production KEEP quartic still starts at measured ego speed and owns
    // acceleration/jerk continuity.
    FrenetPlanner continuity_planner;
    const bool continuity_init = continuity_planner.Init(config);
    CartesianState continuity_ego =
        LoadPoseNearArcLength(waypoints, 100.0, 11.0);
    continuity_ego.a = 0.45;
    PlannerCommand continuity_keep{};
    continuity_keep.mode = LANE_KEEPING;
    continuity_keep.has_target_speed = true;
    continuity_keep.target_speed = 13.89;
    CartesianPath before_cap_drop;
    const bool before_ok = continuity_init && continuity_planner.Plan(
        continuity_ego, no_objects, continuity_keep, before_cap_drop);
    continuity_keep.target_speed = 8.0;
    CartesianPath after_cap_drop;
    const bool after_ok = before_ok && continuity_planner.Plan(
        continuity_ego, no_objects, continuity_keep, after_cap_drop);
    const double trajectory_dt = continuity_planner.sample_dt();
    bool physical_limits_hold = after_ok && after_cap_drop.a.size() >= 2;
    constexpr double kMaxLongitudinalAcceleration = 3.0;
    constexpr double kMaxLongitudinalJerk = 6.0;
    for (std::size_t i = 0; i < after_cap_drop.a.size(); ++i) {
        physical_limits_hold = physical_limits_hold &&
            std::isfinite(after_cap_drop.a[i]) &&
            std::fabs(after_cap_drop.a[i]) <=
                kMaxLongitudinalAcceleration + 1e-6;
        if (i > 0) {
            const double jerk = (after_cap_drop.a[i] -
                                 after_cap_drop.a[i - 1]) / trajectory_dt;
            physical_limits_hold = physical_limits_hold &&
                std::fabs(jerk) <= kMaxLongitudinalJerk + 1e-5;
        }
    }
    Expect(after_ok && continuity_planner.last_raw_speed_cap() <= 8.0 + 1e-9 &&
               std::fabs(continuity_planner.last_target_speed() -
                         continuity_planner.last_raw_speed_cap()) < 1e-9,
           "trajectory-level downward raw cap is the scalar generator target "
           "in the same cycle");
    Expect(after_ok && !after_cap_drop.v.empty() &&
               std::fabs(after_cap_drop.v.front() - continuity_ego.v) < 1e-6 &&
               std::fabs(after_cap_drop.v.front() -
                         continuity_planner.last_target_speed()) > 1.0,
           "lower target does not teleport the first trajectory velocity");
    const std::size_t early_count = std::min<std::size_t>(
        after_cap_drop.v.size(), 6);
    bool early_nonaccelerating = early_count >= 2;
    for (std::size_t i = 1; i < early_count; ++i) {
        early_nonaccelerating = early_nonaccelerating &&
            after_cap_drop.v[i] <= after_cap_drop.v[i - 1] + 1e-6;
    }
    Expect(early_nonaccelerating,
           "deceleration intent does not create an initially accelerating KEEP profile");
    Expect(physical_limits_hold,
           "immediate scalar cap drop still respects configured acceleration "
           "and jerk limits in the physical trajectory");
    Expect(after_ok &&
               IsCartesianTrajectoryTimeConsistent(before_cap_drop,
                                                    trajectory_dt) &&
               IsCartesianTrajectoryTimeConsistent(after_cap_drop,
                                                    trajectory_dt),
           "profiled target regenerates mutually consistent x/y/velocity "
           "trajectories instead of editing velocity fields");

    CartesianState stopped_ego = ego;
    stopped_ego.v = 0.0;
    keep.max_speed = 0.2;
    const bool low_speed_ok = planner.Plan(
        stopped_ego, no_objects, keep, path);
    bool low_speed_capped = low_speed_ok && planner.last_mode() == LANE_KEEPING;
    for (double speed : path.v) low_speed_capped = low_speed_capped && speed <= 0.2 + 1e-9;
    Expect(low_speed_capped,
           "external KEEP low-speed fallback honors speed cap");

    PlannerCommand unsupported{};
    unsupported.mode = FOLLOWING;
    Expect(!planner.Plan(ego, no_objects, unsupported, path),
           "external non-KEEP command remains unsupported");

    if (g_failures == 0) {
        std::cout << "All command KEEP plan tests passed\n";
        return 0;
    }
    return 1;
}
