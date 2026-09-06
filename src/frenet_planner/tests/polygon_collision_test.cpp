#include "frenet_planner/Evaluation/CollisionChecker.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "frenet_planner/Geometry/PolygonUtils.hpp"

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
                             double vy = 0.0, double yaw = 0.0) {
    TrackedObject object;
    object.id = 1;
    object.x = x;
    object.y = y;
    object.heading = yaw;
    object.length = 2.0;
    object.width = 2.0;
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
    object.geometry_source = ObjectGeometrySource::LIDAR_CORNERS;
    object.geometry_valid = geometry::ValidateCorners8(object.corners).valid;
    object.z_valid = false;
    return object;
}

}  // namespace

int main() {
    CartesianState ego{};
    ego.x = 0.0;
    ego.y = 0.0;
    ego.yaw = 0.0;
    const VehicleShape ego_shape{2.0, 2.0};
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

    if (g_failures == 0) {
        std::cout << "All polygon collision tests passed\n";
        return 0;
    }
    return 1;
}
