#include "frenet_planner/obstacle/legacy_object_adapter.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "frenet_planner/geometry/polygon_utils.hpp"

namespace {

constexpr double kTolerance = 1e-9;
int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

bool Near(double a, double b) { return std::abs(a - b) <= kTolerance; }

ObjectInfo MakeObject() {
    ObjectInfo object;
    object.id = 17;
    object.type = 2;
    object.x = 10.0;
    object.y = -4.0;
    object.z = 1.5;
    object.heading = 0.5 * 3.14159265358979323846;
    object.vx = 2.5;
    object.vy = -0.75;
    object.length = 4.0;
    object.width = 2.0;
    object.height = 1.8;
    return object;
}

}  // namespace

int main() {
    const ObjectInfo object = MakeObject();
    const TrackedObstacle tracked = MakeLegacyTrackedObstacle(object);
    Expect(tracked.id == object.id && tracked.type == object.type,
           "adapter preserves object identity");
    Expect(Near(tracked.vx, object.vx) && Near(tracked.vy, object.vy),
           "adapter preserves map velocity components");
    Expect(tracked.geometry_source == ObstacleGeometrySource::LEGACY_DERIVED,
           "adapter marks legacy-derived geometry");
    Expect(tracked.geometry_valid, "valid legacy box produces valid corners");
    Expect(!tracked.z_valid, "legacy Z is not trusted as map-frame height");

    const geometry::Polygon2D hull = geometry::ComputeConvexHull(
        geometry::ProjectCornersToXY(tracked.corners));
    Expect(hull.size() == 4, "legacy prism projects to four hull points");
    Expect(Near(geometry::PolygonArea(hull), 8.0),
           "legacy adapter preserves length times width area");

    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto& point : hull) {
        center_x += point.x;
        center_y += point.y;
    }
    center_x /= static_cast<double>(hull.size());
    center_y /= static_cast<double>(hull.size());
    Expect(Near(center_x, object.x) && Near(center_y, object.y),
           "rotated legacy hull remains centered on input pose");

    bool vertical_pairs_match = true;
    for (std::size_t i = 0; i < 4; ++i) {
        vertical_pairs_match = vertical_pairs_match &&
            Near(tracked.corners[i].x, tracked.corners[i + 4].x) &&
            Near(tracked.corners[i].y, tracked.corners[i + 4].y);
    }
    Expect(vertical_pairs_match, "top and bottom corners share XY coordinates");

    ObjectInfo invalid = object;
    invalid.length = 0.0;
    const TrackedObstacle invalid_tracked = MakeLegacyTrackedObstacle(invalid);
    Expect(!invalid_tracked.geometry_valid, "zero-length legacy box is rejected");

    const std::vector<TrackedObstacle> batch =
        MakeLegacyTrackedObstacles({object, invalid});
    Expect(batch.size() == 2 && batch[0].id == object.id,
           "batch adapter preserves input count and order");

    if (g_failures == 0) {
        std::cout << "All legacy object adapter tests passed\n";
        return 0;
    }
    return 1;
}
