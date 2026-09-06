// 외부(perception/시뮬레이터)에서 들어오는 주변 객체 표현.

#ifndef PLANNER_COMMON_OBJECT_TYPES_HPP
#define PLANNER_COMMON_OBJECT_TYPES_HPP

#include "frenet_planner/Geometry/GeometryTypes.hpp"

enum class ObjectGeometrySource {
    LEGACY_DERIVED = 0,
    LIDAR_CORNERS,
};

// asmc_msgs/DetectedObject.msg 기반 canonical 객체 표현. 스칼라 motion 필드
// (ObjectInfo)와 authoritative OBB corner geometry(TrackedObstacle)를 하나로
// 합친 것이다 - 같은 객체를 두 개의 병렬 vector로 나눠 들고 다니지 않는다.
struct TrackedObject {
    int id = -1;
    int type = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double width = 0.0;
    double length = 0.0;
    double height = 0.0;
    double yaw_rate = 0.0;  // [rad/s] planner가 연속 heading 관측으로 추정

    // ---- 아래는 충돌검사용 authoritative geometry (구 TrackedObstacle) ----
    geometry::Corners8 corners{};
    bool geometry_valid = false;
    bool z_valid = false;
    ObjectGeometrySource geometry_source = ObjectGeometrySource::LEGACY_DERIVED;
};

#endif  // PLANNER_COMMON_OBJECT_TYPES_HPP
