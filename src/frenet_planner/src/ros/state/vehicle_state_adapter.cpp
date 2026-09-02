#include "ros/state/vehicle_state_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "udp_network/gps_transform.hpp"

namespace frenet_planner_ros {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
bool IsFinite(double value) { return std::isfinite(value); }
bool ValidStamp(const ros::Time& stamp) { return !stamp.isZero(); }

bool Fresh(const ros::Time& now, const ros::Time& stamp, double timeout_s) {
    if (!ValidStamp(stamp) || now < stamp) return false;
    return (now - stamp).toSec() <= timeout_s;
}

bool ValidGps(const morai_msgs::GPSMessage& gps) {
    return gps.status > 0 && ValidStamp(gps.header.stamp) &&
           IsFinite(gps.latitude) && gps.latitude >= -90.0 &&
           gps.latitude <= 90.0 && IsFinite(gps.longitude) &&
           gps.longitude >= -180.0 && gps.longitude <= 180.0 &&
           IsFinite(gps.eastOffset) && IsFinite(gps.northOffset);
}

bool ValidImu(const sensor_msgs::Imu& imu) {
    const auto& q = imu.orientation;
    const double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return ValidStamp(imu.header.stamp) && IsFinite(norm_sq) && norm_sq > 1e-12 &&
           IsFinite(imu.angular_velocity.z) &&
           IsFinite(imu.linear_acceleration.x);
}

ros::Time Older(const ros::Time& lhs, const ros::Time& rhs) {
    return lhs < rhs ? lhs : rhs;
}

}  // namespace

GpsImuStateEstimator::GpsImuStateEstimator(const StateAdapterConfig& config)
    : config_(config) {
    config_.speed_filter_alpha =
        std::clamp(config_.speed_filter_alpha, 0.0, 1.0);
}

void GpsImuStateEstimator::updateGps(const morai_msgs::GPSMessage& gps) {
    gps_ = gps;
    has_gps_ = true;
    if (!ValidGps(gps)) return;

    double easting = 0.0;
    double northing = 0.0;
    GpsToUtm52N(gps.latitude, gps.longitude, easting, northing);
    const double x = easting - gps.eastOffset;
    const double y = northing - gps.northOffset;

    if (has_previous_fix_ && gps.header.stamp > previous_gps_stamp_) {
        const double dt = (gps.header.stamp - previous_gps_stamp_).toSec();
        if (dt >= config_.minimum_speed_dt_s && dt <= config_.maximum_speed_dt_s) {
            const double measured_speed =
                std::hypot(x - previous_x_, y - previous_y_) / dt;
            if (IsFinite(measured_speed) &&
                measured_speed <= config_.maximum_speed_mps) {
                speed_mps_ = has_speed_
                    ? config_.speed_filter_alpha * measured_speed +
                          (1.0 - config_.speed_filter_alpha) * speed_mps_
                    : measured_speed;
                has_speed_ = true;
            }
        }
    }
    previous_x_ = x;
    previous_y_ = y;
    previous_gps_stamp_ = gps.header.stamp;
    has_previous_fix_ = true;
}

void GpsImuStateEstimator::updateImu(const sensor_msgs::Imu& imu) {
    imu_ = imu;
    has_imu_ = true;
}

asmc_msgs::VehicleState GpsImuStateEstimator::makeState(
    const ros::Time& now) const {
    asmc_msgs::VehicleState state;
    state.header.frame_id = "map";
    state.source = asmc_msgs::VehicleState::SOURCE_GPS_IMU;
    if (!has_gps_ || !has_imu_) return state;

    state.header.stamp = Older(gps_.header.stamp, imu_.header.stamp);
    if (!ValidGps(gps_) || !ValidImu(imu_) || !has_speed_ ||
        !Fresh(now, gps_.header.stamp, config_.gps_timeout_s) ||
        !Fresh(now, imu_.header.stamp, config_.imu_timeout_s)) {
        return state;
    }

    double easting = 0.0;
    double northing = 0.0;
    GpsToUtm52N(gps_.latitude, gps_.longitude, easting, northing);
    state.x = easting - gps_.eastOffset;
    state.y = northing - gps_.northOffset;
    const auto& q = imu_.orientation;
    const double norm =
        std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    state.yaw =
        QuaternionToYaw(q.w / norm, q.x / norm, q.y / norm, q.z / norm);
    state.speed = speed_mps_;
    state.acceleration = imu_.linear_acceleration.x;
    state.yaw_rate = imu_.angular_velocity.z;
    state.valid = IsFinite(state.x) && IsFinite(state.y) &&
                  IsFinite(state.yaw) && IsFinite(state.speed) &&
                  IsFinite(state.acceleration) && IsFinite(state.yaw_rate);
    return state;
}

asmc_msgs::VehicleState ConvertEgoVehicleStatus(
    const morai_msgs::EgoVehicleStatus& ego) {
    asmc_msgs::VehicleState state;
    state.header = ego.header;
    state.header.frame_id = "map";
    state.source = asmc_msgs::VehicleState::SOURCE_EGO_STATUS;
    state.x = ego.position.x;
    state.y = ego.position.y;
    state.yaw = ego.heading * kDegreesToRadians;
    state.speed = std::hypot(ego.velocity.x, ego.velocity.y);
    state.acceleration = ego.acceleration.x;
    state.yaw_rate = 0.0;
    state.valid = ValidStamp(ego.header.stamp) && IsFinite(state.x) &&
                  IsFinite(state.y) && IsFinite(state.yaw) &&
                  IsFinite(state.speed) && IsFinite(state.acceleration);
    return state;
}

}  // namespace frenet_planner_ros
