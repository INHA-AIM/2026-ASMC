#include "frenet_planner/frenet_planner.hpp"

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

TrackedObject MakeObjectInfo(int id, double x, double y) {
    TrackedObject object;
    object.id = id;
    object.x = x;
    object.y = y;
    object.z = 1.0;
    object.width = 2.0;
    object.length = 4.0;
    object.height = 2.0;
    return object;
}

TrackedObject MakeAuthoritativeBox(int id, double x, double y) {
    TrackedObject obstacle;
    obstacle.id = id;
    obstacle.geometry_source = ObjectGeometrySource::LIDAR_CORNERS;
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
    const std::string config = package_src + "/frenet_planner/config/params.yaml";
    const std::string waypoints = FRENET_TEST_WAYPOINT_FILE;

    FrenetPlanner planner;
    if (!planner.Init(config)) {
        std::cerr << "[FAIL] planner initialization\n";
        return 1;
    }

    // Roundabout MERGE conflict_s(885.76m)의 approach window 안쪽이다.
    const CartesianState ego = LoadPoseNearArcLength(waypoints, 870.0, 5.0);
    const std::vector<TrackedObject> no_objects;
    CartesianPath path;

    // 먼저 legacy 호출로 내부 MERGE context가 활성화될 수 있는 조건을 만든다.
    planner.Plan(ego, no_objects, path);
    Expect(planner.last_mode() == MERGE,
           "legacy planner activates MERGE inside merge zone");

    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    const bool keep_ok = planner.Plan(ego, no_objects, keep, path);
    Expect(keep_ok && planner.last_mode() == LANE_KEEPING && !path.x.empty(),
           "external KEEP ignores legacy merge-zone behavior selection");

    const std::vector<TrackedObject> outside_objects = {
        MakeObjectInfo(31, ego.x + 100.0, ego.y + 100.0)};
    const std::vector<TrackedObject> outside_collision_objects = {
        MakeAuthoritativeBox(31, ego.x + 100.0, ego.y + 100.0)};
    Expect(planner.Plan(ego, outside_objects, outside_collision_objects,
                        keep, path) && !path.x.empty(),
           "authoritative OBB outside path preserves KEEP trajectory");

    const std::vector<TrackedObject> blocking_objects = {
        MakeObjectInfo(32, ego.x, ego.y)};
    const std::vector<TrackedObject> blocking_collision_objects = {
        MakeAuthoritativeBox(32, ego.x, ego.y)};
    Expect(!planner.Plan(ego, blocking_objects, blocking_collision_objects,
                         keep, path) &&
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
    Expect(max_ok && planner.last_target_speed() <= 3.0 + 1e-9,
           "max-only command caps planner default speed");

    CartesianState stopped_ego = ego;
    stopped_ego.v = 0.0;
    keep.max_speed = 0.2;
    const bool low_speed_ok = planner.Plan(
        stopped_ego, no_objects, keep, path);
    bool low_speed_capped = low_speed_ok && planner.last_mode() == LANE_KEEPING;
    for (double speed : path.v) low_speed_capped = low_speed_capped && speed <= 0.2 + 1e-9;
    Expect(low_speed_capped,
           "external KEEP low-speed fallback ignores merge state and honors speed cap");

    planner.Plan(ego, no_objects, path);
    Expect(planner.last_mode() == MERGE,
           "external KEEP does not erase legacy merge context");

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
