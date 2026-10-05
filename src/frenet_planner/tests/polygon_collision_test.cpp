#include "frenet_planner/Evaluation/CollisionChecker.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "frenet_planner/Geometry/PolygonUtils.hpp"
#include "frenet_planner/Maneuver/ManeuverUtils.hpp"

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

// authoritative 8-corner OBB를 직접 구성한다(production은 이제 이런
// center/width/length/yaw 기반 geometry 합성을 하지 않는다 - 테스트만
// 자체 fixture로 명시적 corners를 만든다). length=width=2.0, height=1.0
// 직사각형을 yaw만큼 회전해 두 z 층(0, height)에 배치한다.
TrackedObject MakeObstacle(double x, double y, double vx = 0.0,
                             double vy = 0.0, double yaw = 0.0,
                             double length = 2.0, double width = 2.0) {
    TrackedObject object;
    object.id = 1;
    object.x = x;
    object.y = y;
    object.heading = yaw;
    object.length = length;
    object.width = width;
    object.height = 1.0;
    object.vx = vx;
    object.vy = vy;

    const double half_length = object.length * 0.5;
    const double half_width = object.width * 0.5;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const double local_xy[4][2] = {
        {-half_length, -half_width},
        { half_length, -half_width},
        { half_length,  half_width},
        {-half_length,  half_width},
    };
    for (std::size_t i = 0; i < 4; ++i) {
        const double corner_x = x + cosine * local_xy[i][0] - sine * local_xy[i][1];
        const double corner_y = y + sine * local_xy[i][0] + cosine * local_xy[i][1];
        object.corners[i] = {corner_x, corner_y, 0.0};
        object.corners[i + 4] = {corner_x, corner_y, object.height};
    }
    object.geometry_valid = geometry::ValidateCorners8(object.corners).valid;
    object.z_valid = false;
    return object;
}

FrenetPath MakePath(const std::vector<double>& lateral) {
    FrenetPath path{};
    path.valid = true;
    for (std::size_t i = 0; i < lateral.size(); ++i) {
        path.t.push_back(0.1 * static_cast<double>(i));
        path.s.push_back(10.0);
        path.d.push_back(lateral[i]);
        path.s_d.push_back(0.0);
    }
    return path;
}

}  // namespace

int main() {
    CartesianState ego{};
    ego.x = 0.0;
    ego.y = 0.0;
    ego.yaw = 0.0;
    const VehicleShape ego_shape{2.0, 2.0, -1.0};
    const geometry::Polygon2D ego_footprint =
        MakeEgoFootprint(ego, ego_shape, 0.0);

    const TrackedObject separated = MakeObstacle(3.1, 0.0);
    const TrackedObject overlapping = MakeObstacle(1.5, 0.0);
    const TrackedObject touching = MakeObstacle(2.0, 0.0);
    const TrackedObject rotated = MakeObstacle(1.8, 0.0, 0.0, 0.0, 0.7);
    Expect(!CheckObstacleOverlap(ego_footprint, {}, separated, 0.0),
           "polygon collision rejects separated obstacle");
    Expect(CheckObstacleOverlap(ego_footprint, {}, overlapping, 0.0),
           "polygon collision detects overlap");
    Expect(CheckObstacleOverlap(ego_footprint, {}, touching, 0.0),
           "polygon collision treats boundary touch as collision");
    Expect(CheckObstacleOverlap(ego_footprint, {}, rotated, 0.0),
           "polygon collision detects rotated obstacle");

    const TrackedObject moving = MakeObstacle(5.0, 0.0, -1.5, 0.0);
    Expect(!CheckObstacleOverlap(ego_footprint, {}, moving, 0.0),
           "moving obstacle starts separated");
    Expect(CheckObstacleOverlap(ego_footprint, {}, moving, 2.0),
           "velocity translation predicts future collision");
    const geometry::Polygon2D current = PredictObstacleFootprint(moving, 0.0);
    const geometry::Polygon2D future = PredictObstacleFootprint(moving, 2.0);
    Expect(std::abs(geometry::PolygonArea(current) - geometry::PolygonArea(future)) < 1e-9,
           "future obstacle translation preserves footprint area");

    TrackedObject heading_disagrees = MakeObstacle(
        5.0, 0.0, -1.5, 0.0, 0.5 * std::acos(-1.0));
    heading_disagrees.speed = 1.5;
    const PredictedObjectPose velocity_pose =
        PredictTrackedObjectPose(heading_disagrees, 2.0);
    Expect(std::fabs(velocity_pose.x - 2.0) < 1e-9 &&
               std::fabs(velocity_pose.y) < 1e-9,
           "object center prediction uses vx/vy rather than OBB heading");

    TrackedObject turning = heading_disagrees;
    turning.x = 0.0;
    turning.y = 0.0;
    turning.vx = 1.0;
    turning.vy = 0.0;
    turning.yaw_rate = 0.5 * std::acos(-1.0);
    const PredictedObjectPose turning_pose =
        PredictTrackedObjectPose(turning, 1.0);
    const double expected_turn_radius = 2.0 / std::acos(-1.0);
    Expect(std::fabs(turning_pose.x - expected_turn_radius) < 1e-9 &&
               std::fabs(turning_pose.y - expected_turn_radius) < 1e-9,
           "turn-rate prediction rotates the vx/vy vector, not OBB heading");

    TrackedObject low_threat = moving;
    low_threat.motion_state = ObjectMotionState::LOW_THREAT;
    Expect(!CheckObstacleOverlap(ego_footprint, {}, low_threat, 2.0),
           "LOW_THREAT ignores noisy velocity for future collision");
    const PredictedObjectPose static_pose =
        PredictTrackedObjectPose(low_threat, 2.0);
    Expect(std::fabs(static_pose.x - low_threat.x) < 1e-9 &&
               std::fabs(static_pose.y - low_threat.y) < 1e-9,
           "LOW_THREAT center remains stationary while raw velocity is retained");

    TrackedObject invalid = moving;
    invalid.geometry_valid = false;
    Expect(CheckObstacleOverlap(ego_footprint, {}, invalid, 0.0),
           "invalid obstacle geometry is conservatively colliding");

    TrackedObject elevated = overlapping;
    elevated.z_valid = true;
    for (auto& corner : elevated.corners) corner.z += 10.0;
    const geometry::ZRange ego_z{0.0, 2.0, true};
    Expect(!CheckObstacleOverlap(ego_footprint, ego_z, elevated, 0.0),
           "trusted separated Z ranges suppress XY collision");
    Expect(CheckObstacleOverlap(ego_footprint, {}, elevated, 0.0),
           "invalid ego Z conservatively keeps XY collision active");

    const TrackedObject near_margin = MakeObstacle(2.4, 0.0);
    Expect(!CheckObstacleOverlap(ego_footprint, {}, near_margin, 0.0),
           "near-margin obstacle is separated without safety contour");
    const geometry::Polygon2D expanded = MakeEgoFootprint(ego, ego_shape, 0.5);
    Expect(CheckObstacleOverlap(expanded, {}, near_margin, 0.0),
           "ego safety margin expands collision contour");

    // production VehicleState는 후륜축 좌표다. 같은 x/y에서도 차체가
    // 중심 기준으로 앞뒤 대칭 배치되지 않고 전방으로 더 길어야 한다.
    const VehicleShape rear_axle_shape{2.0, 4.635, 0.79};
    const geometry::Polygon2D rear_axle_footprint =
        MakeEgoFootprint(ego, rear_axle_shape, 0.0);
    double min_x = rear_axle_footprint.front().x;
    double max_x = rear_axle_footprint.front().x;
    for (const auto& point : rear_axle_footprint) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
    }
    Expect(std::fabs(min_x + 0.79) < 1e-9 &&
               std::fabs(max_x - 3.845) < 1e-9,
           "rear-axle ego footprint uses physical front/rear extents");

    // 실차 재현값을 ego frame으로 회전한 배치: 객체 중심은 후륜축 거의
    // 옆이고 두 차량 heading은 거의 반대다.
    const TrackedObject beside_rear_axle =
        MakeObstacle(-0.116, -2.581, 0.0, 0.0, 3.027, 4.5, 1.8);
    const geometry::Polygon2D centered_margin =
        MakeEgoFootprint(ego, VehicleShape{1.9, 4.635, -1.0}, 0.5);
    const geometry::Polygon2D rear_axle_margin =
        MakeEgoFootprint(ego, VehicleShape{1.9, 4.635, 0.79}, 0.5);
    Expect(CheckObstacleOverlap(centered_margin, {}, beside_rear_axle, 0.0),
           "centered footprint reproduces the side false positive");
    Expect(!CheckObstacleOverlap(rear_axle_margin, {}, beside_rear_axle, 0.0),
           "rear-axle footprint removes the side false positive");

    ReferenceConfig straight_ref_config{};
    straight_ref_config.closure_tolerance_m = 0.05;
    straight_ref_config.smoothing_window_m = 10.0;
    straight_ref_config.resample_ds = 0.5;
    straight_ref_config.relax_max_iter = 300;
    straight_ref_config.relax_step = 0.3;
    straight_ref_config.ego_projection_search_window = 30.0;
    const RefLine straight_ref =
        BuildRefLine({-10.0, 10.0}, {0.0, 0.0}, straight_ref_config, /*max_curvature=*/1.0);
    const CollisionCheckConfig collision_cfg{0.5, 0.0, 1.0};
    const TrackedObject lateral_near_margin = MakeObstacle(0.0, 2.4);
    const TrackedObject lateral_overlapping = MakeObstacle(0.0, 1.5);

    std::vector<FrenetPath> escaping{MakePath({0.0, -0.2, -0.4})};
    FilterByCollision(escaping, straight_ref, {lateral_near_margin}, ego_shape,
                      collision_cfg, -1, lateral_near_margin.id);
    Expect(escaping.front().valid,
           "avoid target may escape an existing inflated margin");

    std::vector<FrenetPath> longitudinal_only{MakePath({0.0, 0.0, 0.0})};
    longitudinal_only.front().s = {10.0, 11.0, 12.0};
    FilterByCollision(longitudinal_only, straight_ref,
                      {lateral_near_margin}, ego_shape, collision_cfg, -1,
                      lateral_near_margin.id);
    Expect(!longitudinal_only.front().valid,
           "longitudinal center-distance growth is not lateral escape");

    std::vector<FrenetPath> approaching{MakePath({0.0, 0.2, 0.4})};
    FilterByCollision(approaching, straight_ref, {lateral_near_margin}, ego_shape,
                      collision_cfg, -1, lateral_near_margin.id);
    Expect(!approaching.front().valid,
           "avoid target cannot move deeper into inflated margin");

    std::vector<FrenetPath> physical_collision{
        MakePath({0.0, -0.2, -0.4})};
    FilterByCollision(physical_collision, straight_ref, {lateral_overlapping},
                      ego_shape, collision_cfg, -1, lateral_overlapping.id);
    Expect(!physical_collision.front().valid,
           "avoid escape never permits physical OBB collision");

    std::vector<FrenetPath> non_target_escape{
        MakePath({0.0, -0.2, -0.4})};
    FilterByCollision(non_target_escape, straight_ref, {lateral_near_margin},
                      ego_shape, collision_cfg, -1, -1);
    Expect(!non_target_escape.front().valid,
           "inflated margin remains mandatory for non-target objects");

    CartesianPath low_speed_escape;
    low_speed_escape.x = {0.0, 0.1, 0.2};
    low_speed_escape.y = {0.0, -0.2, -0.4};
    low_speed_escape.yaw = {0.0, 0.0, 0.0};
    int low_speed_blocking_id = -1;
    Expect(CartesianTrajectoryClear(
               low_speed_escape, {lateral_near_margin}, ego_shape,
               collision_cfg, 0.1, &low_speed_blocking_id,
               lateral_near_margin.id),
           "low-speed avoid may escape laterally with the standard margin");

    CartesianPath low_speed_longitudinal = low_speed_escape;
    low_speed_longitudinal.x = {0.0, 0.5, 1.0};
    low_speed_longitudinal.y = {0.0, 0.0, 0.0};
    Expect(!CartesianTrajectoryClear(
               low_speed_longitudinal, {lateral_near_margin}, ego_shape,
               collision_cfg, 0.1, &low_speed_blocking_id,
               lateral_near_margin.id),
           "low-speed longitudinal motion is not an avoid escape");

    if (g_failures == 0) {
        std::cout << "All polygon collision tests passed\n";
        return 0;
    }
    return 1;
}
