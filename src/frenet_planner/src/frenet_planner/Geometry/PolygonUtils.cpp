#include "frenet_planner/Geometry/PolygonUtils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace geometry {
namespace {

bool IsFinite(const Point2D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

double Cross(const Point2D& origin, const Point2D& a, const Point2D& b) {
    return (a.x - origin.x) * (b.y - origin.y) -
           (a.y - origin.y) * (b.x - origin.x);
}

bool NearlyEqual(const Point2D& a, const Point2D& b, double epsilon) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy <= epsilon * epsilon;
}

bool IsValidPolygonForSAT(const Polygon2D& polygon, double epsilon) {
    if (polygon.size() < 3) return false;
    if (!std::all_of(polygon.begin(), polygon.end(), IsFinite)) return false;
    return PolygonArea(polygon) > epsilon;
}

bool HasSeparatingAxis(const Polygon2D& source, const Polygon2D& a,
                       const Polygon2D& b, double epsilon) {
    for (std::size_t i = 0; i < source.size(); ++i) {
        const Point2D& p0 = source[i];
        const Point2D& p1 = source[(i + 1) % source.size()];
        const double edge_x = p1.x - p0.x;
        const double edge_y = p1.y - p0.y;
        const double norm = std::hypot(edge_x, edge_y);
        if (norm <= epsilon) continue;

        const double axis_x = -edge_y / norm;
        const double axis_y = edge_x / norm;

        double a_min = std::numeric_limits<double>::infinity();
        double a_max = -std::numeric_limits<double>::infinity();
        for (const Point2D& point : a) {
            const double projection = point.x * axis_x + point.y * axis_y;
            a_min = std::min(a_min, projection);
            a_max = std::max(a_max, projection);
        }

        double b_min = std::numeric_limits<double>::infinity();
        double b_max = -std::numeric_limits<double>::infinity();
        for (const Point2D& point : b) {
            const double projection = point.x * axis_x + point.y * axis_y;
            b_min = std::min(b_min, projection);
            b_max = std::max(b_max, projection);
        }

        if (a_max < b_min - epsilon || b_max < a_min - epsilon) return true;
    }
    return false;
}

}  // namespace

bool AreCornersFinite(const Corners8& corners) {
    return std::all_of(corners.begin(), corners.end(), [](const Point3D& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) &&
               std::isfinite(point.z);
    });
}

Polygon2D ProjectCornersToXY(const Corners8& corners) {
    Polygon2D projected;
    projected.reserve(corners.size());
    for (const Point3D& corner : corners) {
        projected.push_back({corner.x, corner.y});
    }
    return projected;
}

Polygon2D RemoveDuplicatePoints(const Polygon2D& points, double epsilon) {
    const double safe_epsilon = std::max(0.0, epsilon);
    Polygon2D unique;
    unique.reserve(points.size());
    for (const Point2D& point : points) {
        const bool duplicate = std::any_of(
            unique.begin(), unique.end(), [&](const Point2D& candidate) {
                return NearlyEqual(point, candidate, safe_epsilon);
            });
        if (!duplicate) unique.push_back(point);
    }
    return unique;
}

Polygon2D ComputeConvexHull(const Polygon2D& points, double epsilon) {
    const double safe_epsilon = std::max(0.0, epsilon);
    if (!std::all_of(points.begin(), points.end(), IsFinite)) return {};
    Polygon2D sorted = RemoveDuplicatePoints(points, safe_epsilon);
    if (sorted.size() <= 2) return sorted;

    std::sort(sorted.begin(), sorted.end(), [](const Point2D& a, const Point2D& b) {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    });

    Polygon2D hull(2 * sorted.size());
    std::size_t count = 0;
    for (const Point2D& point : sorted) {
        while (count >= 2 && Cross(hull[count - 2], hull[count - 1], point) <= safe_epsilon) {
            --count;
        }
        hull[count++] = point;
    }

    const std::size_t lower_size = count;
    for (std::size_t i = sorted.size() - 1; i > 0; --i) {
        const Point2D& point = sorted[i - 1];
        while (count > lower_size &&
               Cross(hull[count - 2], hull[count - 1], point) <= safe_epsilon) {
            --count;
        }
        hull[count++] = point;
    }

    if (count > 1) --count;
    hull.resize(count);
    EnsureCounterClockwise(hull);
    return hull;
}

double PolygonSignedArea(const Polygon2D& polygon) {
    if (polygon.size() < 3) return 0.0;
    double twice_area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Point2D& a = polygon[i];
        const Point2D& b = polygon[(i + 1) % polygon.size()];
        twice_area += a.x * b.y - a.y * b.x;
    }
    return 0.5 * twice_area;
}

double PolygonArea(const Polygon2D& polygon) {
    return std::abs(PolygonSignedArea(polygon));
}

void EnsureCounterClockwise(Polygon2D& polygon) {
    if (PolygonSignedArea(polygon) < 0.0) {
        std::reverse(polygon.begin(), polygon.end());
    }
}

ZRange ComputeZRange(const Corners8& corners) {
    if (!AreCornersFinite(corners)) return {};

    ZRange range;
    range.min = corners.front().z;
    range.max = corners.front().z;
    for (const Point3D& corner : corners) {
        range.min = std::min(range.min, corner.z);
        range.max = std::max(range.max, corner.z);
    }
    range.valid = true;
    return range;
}

bool CheckZOverlap(const ZRange& a, const ZRange& b, double margin,
                   bool conservative_on_invalid) {
    const bool a_valid = a.valid && std::isfinite(a.min) &&
                         std::isfinite(a.max) && a.min <= a.max;
    const bool b_valid = b.valid && std::isfinite(b.min) &&
                         std::isfinite(b.max) && b.min <= b.max;
    if (!a_valid || !b_valid || !std::isfinite(margin)) {
        return conservative_on_invalid;
    }

    const double safe_margin = std::max(0.0, margin);
    return a.max + safe_margin >= b.min &&
           b.max + safe_margin >= a.min;
}

bool CheckConvexPolygonOverlapSAT(const Polygon2D& a, const Polygon2D& b,
                                  double epsilon) {
    const double safe_epsilon = std::max(0.0, epsilon);
    if (!IsValidPolygonForSAT(a, safe_epsilon) ||
        !IsValidPolygonForSAT(b, safe_epsilon)) {
        return true;
    }

    return !HasSeparatingAxis(a, a, b, safe_epsilon) &&
           !HasSeparatingAxis(b, a, b, safe_epsilon);
}

Corners8 TranslateCorners(const Corners8& corners, double vx, double vy,
                          double prediction_time) {
    Corners8 translated = corners;
    const double dx = vx * prediction_time;
    const double dy = vy * prediction_time;
    for (Point3D& corner : translated) {
        corner.x += dx;
        corner.y += dy;
    }
    return translated;
}

CornerValidationResult ValidateCorners8(const Corners8& corners,
                                         double duplicate_epsilon,
                                         double min_xy_area) {
    CornerValidationResult result;
    if (!AreCornersFinite(corners)) {
        result.error = CornerValidationError::NON_FINITE;
        return result;
    }

    const Polygon2D unique = RemoveDuplicatePoints(
        ProjectCornersToXY(corners), std::max(0.0, duplicate_epsilon));
    result.unique_xy_count = unique.size();
    if (unique.size() < 3) {
        result.error = CornerValidationError::INSUFFICIENT_UNIQUE_XY;
        return result;
    }

    const Polygon2D hull = ComputeConvexHull(unique);
    result.xy_area = PolygonArea(hull);
    if (!std::isfinite(result.xy_area) ||
        result.xy_area < std::max(0.0, min_xy_area)) {
        result.error = CornerValidationError::DEGENERATE_XY_AREA;
        return result;
    }

    result.valid = true;
    result.error = CornerValidationError::NONE;
    return result;
}

}  // namespace geometry
