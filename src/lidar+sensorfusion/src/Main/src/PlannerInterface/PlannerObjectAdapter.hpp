#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <vector>

#include <Global/Global.hpp>
#include <PlannerInterface/PlannerObjectTypes.hpp>

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
