#ifndef FRENET_GEOMETRY_TYPES_HPP
#define FRENET_GEOMETRY_TYPES_HPP

#include <array>
#include <cstddef>
#include <vector>

namespace geometry {

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

using Corners8 = std::array<Point3D, 8>;
using Polygon2D = std::vector<Point2D>;

struct ZRange {
    double min = 0.0;
    double max = 0.0;
    bool valid = false;
};

enum class CornerValidationError {
    NONE = 0,
    NON_FINITE,
    INSUFFICIENT_UNIQUE_XY,
    DEGENERATE_XY_AREA,
};

struct CornerValidationResult {
    bool valid = false;
    CornerValidationError error = CornerValidationError::NONE;
    std::size_t unique_xy_count = 0;
    double xy_area = 0.0;
};

}  // namespace geometry

#endif  // FRENET_GEOMETRY_TYPES_HPP
