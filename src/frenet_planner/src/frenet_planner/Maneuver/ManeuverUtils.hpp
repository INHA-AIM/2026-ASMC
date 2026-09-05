#ifndef FRENET_MANEUVER_MANEUVER_UTILS_HPP
#define FRENET_MANEUVER_MANEUVER_UTILS_HPP

#include <vector>

#include "frenet_planner/Common/CartesianTypes.hpp"
#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/obstacle/tracked_obstacle.hpp"

// 여러 Maneuver/*.cpp가 공유하는 package-internal helper. FrenetPlanner
// public API가 아니므로 frenet_planner.hpp에는 선언하지 않는다.

// 폐루프 트랙에서 current_s -> target_s의 전방 호길이. STOP(stop_distance
// 계산, Maneuver/Stop.cpp)과 AVOID(post-COMMIT gap, Maneuver/Avoid.cpp)가
// 쓴다.
double ForwardArcDistance(double current_s, double target_s, double s_max);

// fresh Cartesian collision recheck. AVOID(저속 crawl/PREPARE,
// Maneuver/Avoid.cpp)와 MERGE(M2: locked replay/commit_pending candidate
// 검증, Maneuver/Merge.cpp)가 쓴다.
bool CartesianTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObstacle>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id = nullptr);

#endif  // FRENET_MANEUVER_MANEUVER_UTILS_HPP
