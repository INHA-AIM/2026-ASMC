#ifndef FRENET_TRACKED_OBSTACLE_HPP
#define FRENET_TRACKED_OBSTACLE_HPP

#include "frenet_planner/geometry/geometry_types.hpp"

enum class ObstacleGeometrySource {
    LEGACY_DERIVED = 0,
    LIDAR_CORNERS,
};

struct TrackedObstacle {
    int id = -1;
    int type = 0;
    geometry::Corners8 corners{};
    double vx = 0.0;
    double vy = 0.0;
    bool geometry_valid = false;
    bool z_valid = false;
    ObstacleGeometrySource geometry_source =
        ObstacleGeometrySource::LEGACY_DERIVED;
};

#endif  // FRENET_TRACKED_OBSTACLE_HPP
