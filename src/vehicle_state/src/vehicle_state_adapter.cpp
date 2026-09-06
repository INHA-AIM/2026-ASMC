#include "vehicle_state/vehicle_state_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "vehicle_state/gps_transform.hpp"
#include "vehicle_state/steady_freshness.hpp"

namespace frenet_planner_ros {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
bool IsFinite(double value) { return std::isfinite(value); }
bool ValidStamp(const ros::Time& stamp) { return !stamp.isZero(); }

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
    config_.gps_correction_gain =
        std::clamp(config_.gps_correction_gain, 0.0, 1.0);
    config_.imu_acceleration_filter_alpha =
        std::clamp(config_.imu_acceleration_filter_alpha, 0.0, 1.0);
    config_.speed_window_duration_s =
        std::max(config_.speed_window_duration_s,
                 config_.minimum_speed_baseline_s);
}

void GpsImuStateEstimator::updateGps(
    const morai_msgs::GPSMessage& gps, const ros::SteadyTime& received_at) {
    gps_ = gps;
    has_gps_ = true;
    gps_received_at_ = received_at;
    if (!ValidGps(gps)) return;

    double easting = 0.0;
    double northing = 0.0;
    GpsToUtm52N(gps.latitude, gps.longitude, easting, northing);
    const double x = easting - gps.eastOffset;
    const double y = northing - gps.northOffset;

    if (!position_window_.empty() &&
        received_at <= position_window_.back().received_at) {
        position_window_.clear();
        has_stationary_anchor_ = false;
    }
    position_window_.push_back({x, y, received_at});
    while (position_window_.size() > 2 &&
           (received_at - position_window_.front().received_at).toSec() >
               config_.speed_window_duration_s &&
           (received_at - position_window_[1].received_at).toSec() >=
               config_.minimum_speed_baseline_s) {
        position_window_.pop_front();
    }

    if (!has_stationary_anchor_) {
        stationary_anchor_ = {x, y, received_at};
        has_stationary_anchor_ = true;
    } else if (std::hypot(x - stationary_anchor_.x,
                          y - stationary_anchor_.y) >=
               config_.stationary_distance_m) {
        stationary_anchor_ = {x, y, received_at};
    }

    const bool stationary =
        has_stationary_anchor_ &&
        (received_at - stationary_anchor_.received_at).toSec() >=
            config_.stationary_duration_s &&
        std::hypot(x - stationary_anchor_.x, y - stationary_anchor_.y) <
            config_.stationary_distance_m;

    const PositionSample& oldest = position_window_.front();
    const double dt = (received_at - oldest.received_at).toSec();
    if (dt < config_.minimum_speed_baseline_s) return;

    const double dx = x - oldest.x;
    const double dy = y - oldest.y;
    double distance = std::hypot(dx, dy);
    if (has_imu_ && ValidImu(imu_)) {
        const auto& q = imu_.orientation;
        const double norm = std::sqrt(q.w * q.w + q.x * q.x +
                                      q.y * q.y + q.z * q.z);
        const double yaw = QuaternionToYaw(
            q.w / norm, q.x / norm, q.y / norm, q.z / norm);
        distance = std::max(0.0, dx * std::cos(yaw) + dy * std::sin(yaw));
    }
    double gps_speed = distance / dt;
    if (!IsFinite(gps_speed) || gps_speed > config_.maximum_speed_mps) return;

    // Position differencing estimates the average velocity over the baseline.
    // Advance it from the baseline midpoint to this GPS receive time using the
    // latest filtered longitudinal acceleration.
    if (has_filtered_acceleration_) {
        gps_speed += filtered_acceleration_mps2_ * 0.5 * dt;
    }
    gps_speed = std::clamp(gps_speed, 0.0, config_.maximum_speed_mps);

    if (!has_speed_) {
        speed_mps_ = gps_speed;
    } else {
        const double innovation = gps_speed - speed_mps_;
        const double limited_innovation = std::clamp(
            innovation, -config_.gps_innovation_limit_mps,
            config_.gps_innovation_limit_mps);
        speed_mps_ += config_.gps_correction_gain * limited_innovation;
    }

    if (stationary &&
        speed_mps_ <= config_.stationary_speed_threshold_mps &&
        std::abs(filtered_acceleration_mps2_) <=
            config_.stationary_acceleration_threshold_mps2) {
        speed_mps_ = 0.0;
    }
    speed_mps_ = std::clamp(
        speed_mps_, 0.0, config_.maximum_speed_mps);
    has_speed_ = true;
}

void GpsImuStateEstimator::updateImu(
    const sensor_msgs::Imu& imu, const ros::SteadyTime& received_at) {
    imu_ = imu;
    has_imu_ = true;
    imu_received_at_ = received_at;
    if (!ValidImu(imu)) {
        previous_imu_received_at_ = received_at;
        return;
    }

    double acceleration = std::clamp(
        imu.linear_acceleration.x, config_.imu_acceleration_min_mps2,
        config_.imu_acceleration_max_mps2);
    if (std::abs(acceleration) < config_.imu_acceleration_deadband_mps2) {
        acceleration = 0.0;
    }
    filtered_acceleration_mps2_ = has_filtered_acceleration_
        ? config_.imu_acceleration_filter_alpha * acceleration +
              (1.0 - config_.imu_acceleration_filter_alpha) *
                  filtered_acceleration_mps2_
        : acceleration;
    has_filtered_acceleration_ = true;

    if (has_speed_ && !previous_imu_received_at_.isZero()) {
        const double dt = (received_at - previous_imu_received_at_).toSec();
        if (dt > 0.0 && dt <= config_.imu_maximum_dt_s) {
            speed_mps_ = std::clamp(
                speed_mps_ + filtered_acceleration_mps2_ * dt,
                0.0, config_.maximum_speed_mps);
        }
    }
    previous_imu_received_at_ = received_at;
}

asmc_msgs::VehicleState GpsImuStateEstimator::makeState(
    const ros::SteadyTime& now) const {
    asmc_msgs::VehicleState state;
    state.header.frame_id = "map";
    state.source = asmc_msgs::VehicleState::SOURCE_GPS_IMU;
    if (!has_gps_ || !has_imu_) return state;

    state.header.stamp = Older(gps_.header.stamp, imu_.header.stamp);
    if (!ValidGps(gps_) || !ValidImu(imu_) || !has_speed_ ||
        !IsSteadyFresh(gps_received_at_, now, config_.gps_timeout_s) ||
        !IsSteadyFresh(imu_received_at_, now, config_.imu_timeout_s)) {
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
    state.acceleration = filtered_acceleration_mps2_;
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
