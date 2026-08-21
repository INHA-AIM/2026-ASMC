#ifndef FRENET_LEGACY_OBJECT_ADAPTER_HPP
#define FRENET_LEGACY_OBJECT_ADAPTER_HPP

#include <vector>

#include "frenet_planner/global/global.hpp"
#include "frenet_planner/obstacle/tracked_obstacle.hpp"

TrackedObstacle MakeLegacyTrackedObstacle(const ObjectInfo& object);

std::vector<TrackedObstacle> MakeLegacyTrackedObstacles(
    const std::vector<ObjectInfo>& objects);

#endif  // FRENET_LEGACY_OBJECT_ADAPTER_HPP
