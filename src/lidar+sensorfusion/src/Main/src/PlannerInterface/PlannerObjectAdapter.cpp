#include <PlannerInterface/PlannerObjectAdapter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kMpsToKph = 3.6;
constexpr double kMinDirectionSpeedMps = 0.5;
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
    if (object_class == "car" || object_class == "truck" ||
        object_class == "bus") return 2;
    return 3;
}

struct Pose2D {
    double x;
    double y;
    double yaw;
};

Pose2D TransformObbToMap(const LiDARObjectInfo& object,
                         const GPS_IMU& gps_imu,
                         const VehicleSpec& vehicle) {
    const double mount_yaw = Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Z);
    const double mount_cos = std::cos(mount_yaw);
    const double mount_sin = std::sin(mount_yaw);

    // velodyne 원점에서 GPS 안테나 원점으로 옮긴 vehicle-frame 좌표.
    const double body_x =
        mount_cos * object.f32_OBB_CenterX -
        mount_sin * object.f32_OBB_CenterY +
        vehicle.st_LiDARPos.f32_X - vehicle.st_GPSPos.f32_X;
    const double body_y =
        mount_sin * object.f32_OBB_CenterX +
        mount_cos * object.f32_OBB_CenterY +
        vehicle.st_LiDARPos.f32_Y - vehicle.st_GPSPos.f32_Y;

    const double ego_yaw = gps_imu.st_IMU.f32_Yaw_rad;
    const double ego_cos = std::cos(ego_yaw);
    const double ego_sin = std::sin(ego_yaw);

    Pose2D result{};
    result.x = gps_imu.st_GPS.f64_E + ego_cos * body_x - ego_sin * body_y;
    result.y = gps_imu.st_GPS.f64_N + ego_sin * body_x + ego_cos * body_y;
    result.yaw = NormalizeAngle(
        ego_yaw + mount_yaw + object.f32_OBB_YawRad);
    return result;
}

double MotionHeadingInMap(const LiDARObjectInfo& object,
                          const GPS_IMU& gps_imu,
                          const VehicleSpec& vehicle,
                          double fallback_heading) {
    const double direction_norm = std::hypot(object.f32_DirX, object.f32_DirY);
    if (!std::isfinite(direction_norm) || direction_norm < 1e-3 ||
        !std::isfinite(object.f32_Velocity) ||
        object.f32_Velocity < kMinDirectionSpeedMps) {
        return fallback_heading;
    }

    const double sensor_heading = std::atan2(object.f32_DirY, object.f32_DirX);
    return NormalizeAngle(
        gps_imu.st_IMU.f32_Yaw_rad +
        Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Z) + sensor_heading);
}

}  // namespace

std::vector<PlannerObject> MakePlannerObjects(
    const LiDAR& lidar,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle,
    std::size_t max_objects) {
    std::vector<PlannerObject> result;
    result.reserve(std::min(max_objects, lidar.vec_Objects.size()));

    for (const auto& object : lidar.vec_Objects) {
        if (result.size() >= max_objects) break;

        // miss 객체는 Kalman 중심만 예측되고 OBB 중심/꼭짓점은 함께 이동하지
        // 않으므로, 현재 1차 인터페이스에서는 실측 OBB만 내보낸다.
        if (object.s32_Id < 0 || object.s32_MissCount != 0 ||
            object.pcl_ClusterPoints.empty() || IsNoise(object.str_ObjectClass)) {
            continue;
        }
        if (!IsFinitePositive(object.f32_OBB_Width) ||
            !IsFinitePositive(object.f32_OBB_Height)) {
            continue;
        }

        const Pose2D map_pose = TransformObbToMap(object, gps_imu, vehicle);
        if (!std::isfinite(map_pose.x) || !std::isfinite(map_pose.y) ||
            !std::isfinite(map_pose.yaw)) {
            continue;
        }

        // ObjectInfoReceiver에서 id==0은 빈 슬롯이므로 tracker ID에 1을 더한다.
        if (object.s32_Id >= std::numeric_limits<std::int16_t>::max()) continue;

        PlannerObject planner_object;
        planner_object.id = static_cast<std::int16_t>(object.s32_Id + 1);
        planner_object.type = MapObjectType(object.str_ObjectClass);
        planner_object.x = static_cast<float>(map_pose.x);
        planner_object.y = static_cast<float>(map_pose.y);
        planner_object.z = object.f32_OBB_CenterZ;
        planner_object.heading_deg = static_cast<float>(
            MotionHeadingInMap(object, gps_imu, vehicle, map_pose.yaw) * kRadToDeg);

        // L-shape 구현은 긴 축을 w/yaw 축으로 정규화한다.
        planner_object.length = object.f32_OBB_Width;
        planner_object.width = object.f32_OBB_Height;
        planner_object.height = std::max(
            kMinBoxSide, object.f32_OBB_ZMax - object.f32_OBB_ZMin);
        const double speed_mps = std::isfinite(object.f32_Velocity)
            ? std::max(0.0, static_cast<double>(object.f32_Velocity))
            : 0.0;
        planner_object.speed_kph = static_cast<float>(speed_mps * kMpsToKph);
        result.push_back(planner_object);
    }

    return result;
}
