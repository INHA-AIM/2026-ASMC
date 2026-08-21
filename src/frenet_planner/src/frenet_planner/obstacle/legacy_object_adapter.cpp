#include "frenet_planner/obstacle/legacy_object_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/geometry/polygon_utils.hpp"

TrackedObstacle MakeLegacyTrackedObstacle(const ObjectInfo& object) {
    TrackedObstacle tracked;
    tracked.id = object.id;
    tracked.type = object.type;
    tracked.vx = std::isfinite(object.vx) ? object.vx : 0.0;
    tracked.vy = std::isfinite(object.vy) ? object.vy : 0.0;
    tracked.geometry_source = ObstacleGeometrySource::LEGACY_DERIVED;

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

    const bool finite_z = std::isfinite(object.z);
    const bool finite_height = std::isfinite(object.height) && object.height > 0.0;
    const double center_z = finite_z ? object.z : 0.0;
    const double half_height = finite_height ? object.height * 0.5 : 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        const double x = object.x + cosine * local_xy[i][0] - sine * local_xy[i][1];
        const double y = object.y + sine * local_xy[i][0] + cosine * local_xy[i][1];
        tracked.corners[i] = {x, y, center_z - half_height};
        tracked.corners[i + 4] = {x, y, center_z + half_height};
    }

    tracked.geometry_valid = geometry::ValidateCorners8(tracked.corners).valid;
    // Legacy packet의 Z는 map-frame임이 보장되지 않으므로 XY 충돌검사에서
    // 장애물을 제거하는 근거로 사용하지 않는다.
    tracked.z_valid = false;
    return tracked;
}

std::vector<TrackedObstacle> MakeLegacyTrackedObstacles(
    const std::vector<ObjectInfo>& objects) {
    std::vector<TrackedObstacle> tracked;
    tracked.reserve(objects.size());
    for (const ObjectInfo& object : objects) {
        tracked.push_back(MakeLegacyTrackedObstacle(object));
    }
    return tracked;
}
