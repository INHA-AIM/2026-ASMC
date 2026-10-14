#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <Geometry/FrameBoundary.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMpsToKph = 3.6;
constexpr float kMinBoxSide = 0.05f;

double NormalizeAngle(double angle) {
    return std::remainder(angle, 2.0 * kPi);
}

bool IsFinitePositive(float value) {
    return std::isfinite(value) && value >= kMinBoxSide;
}

bool IsNoise(const std::string& object_class) {
    return object_class == "noise" || object_class == "지면노이즈";
}

std::int16_t MapObjectType(const std::string& object_class) {
    if (object_class == "person" || object_class == "pedestrian") return 1;
    if (object_class == "vehicle" || object_class == "car" || object_class == "truck" ||
        object_class == "bus" || object_class == "bicycle" ||
        object_class == "motorcycle") return 2;
    return 3;
}

Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

Eigen::Matrix3d VehicleFromLidarRotation(const VehicleSpec& vehicle) {
    return RotationFromRpy(
        Deg2Rad(vehicle.st_LiDARRotationDeg.f32_X),
        Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Y),
        Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Z));
}

Eigen::Matrix3d MapFromVehicleRotation(const GPS_IMU& gps_imu) {
    return RotationFromRpy(
        gps_imu.st_IMU.f32_Roll_rad,
        gps_imu.st_IMU.f32_Pitch_rad,
        gps_imu.st_IMU.f32_Yaw_rad);
}

Eigen::Matrix3d RawLidarFromPreprocessedRotation(const GPS_IMU& gps_imu) {
    // ImuFusion::CorrectingPoints()가 모든 후단 cloud/OBB/KF 좌표에
    // Ry(+imu_pitch)를 미리 적용한다. map 변환에서 pitch를 중복 적용하지
    // 않도록 정확한 역회전을 먼저 수행한다.
    return Eigen::AngleAxisd(
        -gps_imu.st_IMU.f32_Pitch_rad,
        Eigen::Vector3d::UnitY()).toRotationMatrix();
}

Eigen::Vector3d ToEigen(const PlannerPoint3D& point) {
    return {point.x, point.y, point.z};
}

PlannerPoint3D FromEigen(const Eigen::Vector3d& point) {
    return {point.x(), point.y(), point.z()};
}

bool IsFinite(const PlannerPoint3D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

}  // namespace

PlannerPoint3D TransformLidarPointToMap(
    const PlannerPoint3D& point_lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle) {
    if(frame_boundary::Enabled()) {
        const auto& context=frame_boundary::Current();
        const auto base=asmc::lidar::base_from_input(context);
        if(!base || !context.map_pose.valid) return {NAN,NAN,NAN};
        return FromEigen(context.map_pose.map_from_base * *base * ToEigen(point_lidar));
    }
    const Eigen::Vector3d lidar_in_gps_vehicle(
        vehicle.st_LiDARPos.f32_X - vehicle.st_GPSPos.f32_X,
        vehicle.st_LiDARPos.f32_Y - vehicle.st_GPSPos.f32_Y,
        vehicle.st_LiDARPos.f32_Z - vehicle.st_GPSPos.f32_Z);
    const Eigen::Vector3d gps_map(
        gps_imu.st_GPS.f64_E,
        gps_imu.st_GPS.f64_N,
        gps_imu.st_GPS.f64_U);

    const Eigen::Vector3d point_vehicle =
        VehicleFromLidarRotation(vehicle) *
            RawLidarFromPreprocessedRotation(gps_imu) * ToEigen(point_lidar) +
        lidar_in_gps_vehicle;
    return FromEigen(
        gps_map + MapFromVehicleRotation(gps_imu) * point_vehicle);
}

PlannerPoint3D RotateLidarVectorToMap(
    const PlannerPoint3D& vector_lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle) {
    if(frame_boundary::Enabled()) {
        const auto& context=frame_boundary::Current();
        const auto base=asmc::lidar::base_from_input(context);
        if(!base || !context.map_pose.valid) return {NAN,NAN,NAN};
        return FromEigen(context.map_pose.map_from_base.rotation()*base->rotation()*ToEigen(vector_lidar));
    }
    return FromEigen(MapFromVehicleRotation(gps_imu) *
                     VehicleFromLidarRotation(vehicle) *
                     RawLidarFromPreprocessedRotation(gps_imu) *
                     ToEigen(vector_lidar));
}

std::vector<PlannerObject> MakePlannerObjects(
    const LiDAR& lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle,
    std::size_t max_objects) {
    std::vector<PlannerObject> result;
    result.reserve(std::min(max_objects, lidar.vec_Objects.size()));

    for (const auto& object : lidar.vec_Objects) {
        if (result.size() >= max_objects) break;

        // miss 객체는 KF 중심만 이동하고 OBB 꼭짓점은 실측 시각에 남으므로,
        // 서로 다른 시각의 geometry를 섞지 않고 실측 객체만 내보낸다.
        if (object.s32_Id < 0 || object.s32_MissCount != 0 ||
            object.pcl_ClusterPoints.empty() || IsNoise(object.str_ObjectClass)) {
            continue;
        }
        if (object.s32_Id >= std::numeric_limits<std::int16_t>::max()) continue;
        if (!IsFinitePositive(object.f32_OBB_Width) ||
            !IsFinitePositive(object.f32_OBB_Height)) {
            continue;
        }

        PlannerObject planner_object;
        planner_object.id = object.s32_Id;
        planner_object.type = MapObjectType(
            object.str_ObjectClass == "dynamic_obstacle" && object.str_YoloClass == "person"
                ? "person" : object.str_ObjectClass);
        planner_object.classification = object.str_ObjectClass;
        planner_object.yolo_class =
            object.str_YoloClass.empty() ? "unknown" : object.str_YoloClass;
        planner_object.is_dynamic = object.b_IsDynamic;
        planner_object.age = object.s32_Age;
        planner_object.miss_count = object.s32_MissCount;

        planner_object.cluster_center_map = TransformLidarPointToMap(
            {object.f32_CoreX, object.f32_CoreY, object.f32_CoreZ},
            gps_imu, vehicle);
        planner_object.obb_center_map = TransformLidarPointToMap(
            {object.f32_OBB_CenterX, object.f32_OBB_CenterY,
             object.f32_OBB_CenterZ},
            gps_imu, vehicle);
        for (std::size_t i = 0; i < planner_object.obb_corners_map.size(); ++i) {
            const auto& corner = object.OBB_Corners_Body[i];
            planner_object.obb_corners_lidar[i] = {
                corner.x, corner.y, corner.z};
            planner_object.obb_corners_map[i] = TransformLidarPointToMap(
                {corner.x, corner.y, corner.z}, gps_imu, vehicle);
        }
        planner_object.cluster_center_lidar = {
            object.f32_CoreX, object.f32_CoreY, object.f32_CoreZ};
        planner_object.obb_center_lidar = {
            object.f32_OBB_CenterX, object.f32_OBB_CenterY,
            object.f32_OBB_CenterZ};

        // OBB의 긴 축(+x)을 회전시켜 map XY에 투영한다. 위치 translation과
        // 속도 방향을 OBB heading에 섞지 않는다.
        const PlannerPoint3D long_axis_map = RotateLidarVectorToMap(
            {std::cos(object.f32_OBB_YawRad),
             std::sin(object.f32_OBB_YawRad), 0.0},
            gps_imu, vehicle);
        planner_object.obb_yaw_map_rad = NormalizeAngle(
            std::atan2(long_axis_map.y, long_axis_map.x));

        planner_object.length = object.f32_OBB_Width;
        planner_object.width = object.f32_OBB_Height;
        planner_object.height = std::max(
            kMinBoxSide, object.f32_OBB_ZMax - object.f32_OBB_ZMin);
        planner_object.ground_evidence = object.ground_evidence;

        PlannerPoint3D raw_velocity_lidar{};
        if (std::isfinite(object.f32_AbsVx) &&
            std::isfinite(object.f32_AbsVy)) {
            raw_velocity_lidar.x = object.f32_AbsVx;
            raw_velocity_lidar.y = object.f32_AbsVy;
        }
        planner_object.raw_velocity_lidar_mps = raw_velocity_lidar;
        planner_object.raw_velocity_map_mps = RotateLidarVectorToMap(
            raw_velocity_lidar, gps_imu, vehicle);
        planner_object.raw_speed_kph = kMpsToKph * std::sqrt(
            planner_object.raw_velocity_map_mps.x *
                planner_object.raw_velocity_map_mps.x +
            planner_object.raw_velocity_map_mps.y *
                planner_object.raw_velocity_map_mps.y +
            planner_object.raw_velocity_map_mps.z *
                planner_object.raw_velocity_map_mps.z);

        PlannerPoint3D planner_velocity_lidar{};
        if (std::isfinite(object.f32_PlannerVxKPH) &&
            std::isfinite(object.f32_PlannerVyKPH)) {
            planner_velocity_lidar.x = object.f32_PlannerVxKPH / kMpsToKph;
            planner_velocity_lidar.y = object.f32_PlannerVyKPH / kMpsToKph;
        }
        planner_object.velocity_lidar_mps = planner_velocity_lidar;
        planner_object.velocity_map_mps = RotateLidarVectorToMap(
            planner_velocity_lidar, gps_imu, vehicle);
        planner_object.speed_kph = kMpsToKph * std::sqrt(
            planner_object.velocity_map_mps.x *
                planner_object.velocity_map_mps.x +
            planner_object.velocity_map_mps.y *
                planner_object.velocity_map_mps.y +
            planner_object.velocity_map_mps.z *
                planner_object.velocity_map_mps.z);
        planner_object.decision_vx_kph = object.f32_DecisionVxKPH;
        planner_object.decision_vy_kph = object.f32_DecisionVyKPH;
        planner_object.decision_speed_kph = object.f32_DecisionSpeedKPH;
        planner_object.map_displacement_speed_kph =
            object.f32_MapDisplacementSpeedKPH;
        planner_object.corrected_displacement_speed_kph =
            object.f32_CorrectedDisplacementSpeedKPH;
        planner_object.motion_velocity_jump_kph =
            object.f32_MotionVelocityJumpKPH;
        planner_object.motion_state = object.s32_MotionState;
        planner_object.motion_valid_observations =
            object.s32_MotionValidObservations;
        planner_object.motion_accepted_samples =
            object.s32_MotionAcceptedSamples;
        planner_object.motion_candidate_samples =
            object.s32_MotionCandidateSamples;

        if (!IsFinite(planner_object.cluster_center_map) ||
            !IsFinite(planner_object.obb_center_map) ||
            !IsFinite(planner_object.velocity_map_mps) ||
            !std::isfinite(planner_object.obb_yaw_map_rad)) {
            continue;
        }
        const bool finite_corners = std::all_of(
            planner_object.obb_corners_map.begin(),
            planner_object.obb_corners_map.end(), IsFinite);
        if (!finite_corners) continue;

        result.push_back(std::move(planner_object));
    }

    return result;
}
