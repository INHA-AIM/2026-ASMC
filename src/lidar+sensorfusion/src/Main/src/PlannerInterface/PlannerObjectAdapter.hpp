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
    // YOLO 원본 라벨. classification이 car/person/unknown으로 접힌 값인 반면
    // 이쪽은 truck/bus까지 구분된다.
    std::string yolo_class = "unknown";
    bool is_dynamic = false;
    std::int32_t age = 0;
    std::int32_t miss_count = 0;

    PlannerPoint3D cluster_center_map;
    PlannerPoint3D obb_center_map;
    std::array<PlannerPoint3D, 8> obb_corners_map{};
    double obb_yaw_map_rad = 0.0;

    PlannerPoint3D cluster_center_lidar;
    PlannerPoint3D obb_center_lidar;
    std::array<PlannerPoint3D, 8> obb_corners_lidar{};
    PlannerPoint3D velocity_lidar_mps;

    PlannerPoint3D velocity_map_mps;
    double speed_kph = 0.0;

    // MotionClassifier의 별도 판정값. velocity_*와 speed_kph는 현재
    // 프레임 원본이며 아래 값으로 대체하지 않는다.
    double decision_vx_kph = 0.0;
    double decision_vy_kph = 0.0;
    double decision_speed_kph = 0.0;
    double map_displacement_speed_kph = 0.0;
    double motion_velocity_jump_kph = 0.0;
    std::int32_t motion_state = 0;
    std::int32_t motion_valid_observations = 0;
    std::int32_t motion_accepted_samples = 0;
    std::int32_t motion_candidate_samples = 0;

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
// 다시 더하지 않는다. LiDAR 장착점의 회전 레버암 속도
// (omega x r_mount)는 현재 적용하지 않는다.
PlannerPoint3D RotateLidarVectorToMap(
    const PlannerPoint3D& vector_lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle);
