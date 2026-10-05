#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <Lidar/ImuRansac/GroundEvidence.hpp>

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
    PlannerPoint3D raw_velocity_lidar_mps;
    PlannerPoint3D velocity_lidar_mps;

    PlannerPoint3D raw_velocity_map_mps;
    PlannerPoint3D velocity_map_mps;
    double raw_speed_kph = 0.0;
    double speed_kph = 0.0;

    // raw_velocity_*는 KF 원본, velocity_*와 speed_kph는 motion state를
    // 반영해 planner에 실제 전달되는 값이다. 아래 값은 판정 진단이다.
    double decision_vx_kph = 0.0;
    double decision_vy_kph = 0.0;
    double decision_speed_kph = 0.0;
    double map_displacement_speed_kph = 0.0;
    double corrected_displacement_speed_kph = 0.0;
    double motion_velocity_jump_kph = 0.0;
    std::int32_t motion_state = 0;
    std::int32_t motion_valid_observations = 0;
    std::int32_t motion_accepted_samples = 0;
    std::int32_t motion_candidate_samples = 0;

    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
    ObjectGroundEvidence ground_evidence;
};

