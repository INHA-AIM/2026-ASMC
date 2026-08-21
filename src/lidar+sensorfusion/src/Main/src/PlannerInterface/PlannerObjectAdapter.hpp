#pragma once

#include <cstdint>
#include <vector>

#include <Global/Global.hpp>

struct PlannerObject {
    std::int16_t id = 0;
    std::int16_t type = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float heading_deg = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
    float speed_kph = 0.0f;
};

// LiDAR tracker의 velodyne-frame OBB를 Frenet planner가 기대하는 map-frame
// ObjectInfo 값으로 변환한다. 기존 LiDAR tracking/OBB와 planner 알고리즘은
// 이 경계 타입을 알지 않는다.
std::vector<PlannerObject> MakePlannerObjects(
    const LiDAR& lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle,
    std::size_t max_objects);
