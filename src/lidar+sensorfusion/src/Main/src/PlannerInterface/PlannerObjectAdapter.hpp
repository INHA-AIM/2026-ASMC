#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <vector>

#include <Global/Global.hpp>

struct PlannerPoint3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct PlannerObject {
    // LiDAR tracker의 원래 ID. Legacy UDP의 id==0 sentinel 보정(+1)은
    // serializer 경계에서만 수행한다.
    std::int32_t id = -1;
    std::int16_t type = 0;
    std::string classification;
    bool is_dynamic = false;
    std::int32_t age = 0;
    std::int32_t miss_count = 0;

    PlannerPoint3D cluster_center_map;
    PlannerPoint3D obb_center_map;
    std::array<PlannerPoint3D, 8> obb_corners_map{};
    double obb_yaw_map_rad = 0.0;

    PlannerPoint3D velocity_map_mps;
    double speed_kph = 0.0;

    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
};

// LiDAR tracker의 velodyne-frame OBB를 Frenet planner가 기대하는 map-frame
// ObjectInfo 값으로 변환한다. 기존 LiDAR tracking/OBB와 planner 알고리즘은
// 이 경계 타입을 알지 않는다.
std::vector<PlannerObject> MakePlannerObjects(
    const LiDAR& lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle,
    std::size_t max_objects);

// CorrectingPoints의 pitch 전처리 역회전 후, 점에는 장착 translation과
// GPS ENU translation을 모두 적용한다.
PlannerPoint3D TransformLidarPointToMap(
    const PlannerPoint3D& point_lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle);

// 벡터에는 회전만 적용한다. 현재 tracker의 f32_Velocity/f32_DirX,Y는
// Tracking.cpp에서 CAN 병진속도가 이미 더해진 값이므로 CAN 속도를 여기서
// 다시 더하지 않는다. LiDAR 장착점의 회전 레버암 속도(omega x r)는
// MakePlannerObjects에서 정확히 한 번 더한 뒤 이 함수로 map 회전한다.
PlannerPoint3D RotateLidarVectorToMap(
    const PlannerPoint3D& vector_lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle);
