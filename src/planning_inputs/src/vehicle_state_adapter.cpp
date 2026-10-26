#include "planning_inputs/vehicle_state_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "planning_inputs/gps_transform.hpp"
#include "planning_inputs/steady_freshness.hpp"

namespace frenet_planner_ros {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kHistoryDurationS = 2.0;
constexpr double kStampEpsilonS = 1e-9;
bool IsFinite(double value) { return std::isfinite(value); }
bool ValidStamp(const ros::Time& stamp) { return !stamp.isZero(); }

double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool ValidGps(const morai_msgs::GPSMessage& gps) {
    // (0,0) with status=1 is MORAI's jamming sentinel, outside this UTM 52N
    // map. Never let it pollute the position/speed history or yaw rotation.
    if (gps.latitude == 0.0 && gps.longitude == 0.0) return false;
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

}  // namespace

GpsImuStateEstimator::GpsImuStateEstimator(const StateAdapterConfig& config)
    : config_(config) {
    config_.gps_correction_gain =
        std::clamp(config_.gps_correction_gain, 0.0, 1.0);
    if (!IsFinite(config_.gps_gain_reference_speed_mps) ||
        config_.gps_gain_reference_speed_mps < 0.0)
        config_.gps_gain_reference_speed_mps = 0.0;
    config_.gps_gain_minimum = IsFinite(config_.gps_gain_minimum)
        ? std::clamp(config_.gps_gain_minimum, 0.0, config_.gps_correction_gain)
        : config_.gps_correction_gain;
    config_.position_gps_correction_gain =
        std::clamp(config_.position_gps_correction_gain, 0.0, 1.0);
    config_.imu_acceleration_filter_alpha =
        std::clamp(config_.imu_acceleration_filter_alpha, 0.0, 1.0);
    config_.speed_window_duration_s =
        std::max(config_.speed_window_duration_s,
                 config_.minimum_speed_baseline_s);
}

void GpsImuStateEstimator::updateGps(
    const morai_msgs::GPSMessage& gps, const ros::SteadyTime& received_at) {
    has_gps_ = true;
    if (!ValidGps(gps)) {
        beginGpsRecovery();
        latest_gps_valid_ = false;
        gps_received_at_ = received_at;
        return;
    }
    if (!latest_gps_source_stamp_.isZero() &&
        gps.header.stamp <= latest_gps_source_stamp_) {
        return;
    }

    if (!latest_gps_source_stamp_.isZero() &&
        ((gps.header.stamp - latest_gps_source_stamp_).toSec() > config_.gps_timeout_s + 1e-6 ||
         (received_at - gps_received_at_).toSec() > config_.gps_timeout_s + 1e-6)) {
        beginGpsRecovery();
    }
    latest_gps_valid_ = true;
    gps_received_at_ = received_at;
    latest_gps_source_stamp_ = gps.header.stamp;

    if (config_.imu_yaw_is_true_enu) {
        const double rotation = Utm52NYawOffset(gps.latitude, gps.longitude);
        const double change = NormalizeAngle(rotation - imu_to_grid_yaw_);
        // GPS can arrive after IMU. Retained orientations must use one grid
        // basis, including samples received before the first usable fix.
        for (auto& imu : imu_history_) imu.yaw = NormalizeAngle(imu.yaw + change);
        for (auto& pose : pose_history_) pose.yaw = NormalizeAngle(pose.yaw + change);
        predicted_yaw_ = NormalizeAngle(predicted_yaw_ + change);
        imu_to_grid_yaw_ = rotation;
    }

    double easting = 0.0;
    double northing = 0.0;
    GpsToUtm52N(gps.latitude, gps.longitude, easting, northing);
    pending_gps_.push_back(
        {gps.header.stamp, easting - gps.eastOffset,
         northing - gps.northOffset});
    processPendingGps();
}

void GpsImuStateEstimator::updateImu(
    const sensor_msgs::Imu& imu, const ros::SteadyTime& received_at) {
    has_imu_ = true;
    if (!ValidImu(imu)) {
        latest_imu_valid_ = false;
        imu_received_at_ = received_at;
        return;
    }
    if (!latest_imu_source_stamp_.isZero() &&
        imu.header.stamp <= latest_imu_source_stamp_) {
        return;
    }

    latest_imu_valid_ = true;
    imu_received_at_ = received_at;

    const auto& q = imu.orientation;
    const double norm =
        std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    const double yaw = NormalizeAngle(QuaternionToYaw(
        q.w / norm, q.x / norm, q.y / norm, q.z / norm) + imu_to_grid_yaw_);

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

    const ros::Time previous_imu_stamp = latest_imu_source_stamp_;
    if (has_speed_ && !previous_imu_stamp.isZero()) {
        const double dt = (imu.header.stamp - previous_imu_stamp).toSec();
        if (dt > 0.0 && dt <= config_.imu_maximum_dt_s) {
            speed_mps_ = std::clamp(
                speed_mps_ + filtered_acceleration_mps2_ * dt,
                0.0, config_.maximum_speed_mps);
        }
    }
    latest_imu_source_stamp_ = imu.header.stamp;

    imu_history_.push_back({imu.header.stamp, yaw,
                            imu.angular_velocity.z,
                            filtered_acceleration_mps2_});

    if (pose_initialized_ && imu.header.stamp > state_source_stamp_) {
        const double dt = (imu.header.stamp - state_source_stamp_).toSec();
        if (dt > 0.0 && dt <= config_.imu_maximum_dt_s) {
            const double delta_yaw = NormalizeAngle(yaw - predicted_yaw_);
            const double midpoint_yaw =
                NormalizeAngle(predicted_yaw_ + 0.5 * delta_yaw);
            const double previous_pose_speed = predicted_speed_mps_;
            const double current_speed = has_speed_ ? speed_mps_ : 0.0;
            const double midpoint_speed =
                0.5 * (previous_pose_speed + current_speed);
            predicted_x_ += midpoint_speed * std::cos(midpoint_yaw) * dt;
            predicted_y_ += midpoint_speed * std::sin(midpoint_yaw) * dt;
            predicted_yaw_ = yaw;
            predicted_speed_mps_ = current_speed;
            predicted_acceleration_mps2_ = filtered_acceleration_mps2_;
            predicted_yaw_rate_ = imu.angular_velocity.z;
            state_source_stamp_ = imu.header.stamp;
            appendCurrentPoseToHistory();
        }
    }

    // GPS commonly arrives after an IMU sample with the same source time.
    // Delay that GPS only until its source-time yaw can be interpolated.
    processPendingGps();
    pruneHistories(imu.header.stamp);
}

asmc_msgs::VehicleState GpsImuStateEstimator::makeState(
    const ros::SteadyTime& now) const {
    asmc_msgs::VehicleState state;
    state.header.frame_id = "map";
    state.source = asmc_msgs::VehicleState::SOURCE_GPS_IMU;
    if (!has_gps_ || !has_imu_) return state;

    // This stamp is the sensor source time represented coherently by
    // x/y/yaw/speed below. It is never replaced with publication time.
    state.header.stamp = state_source_stamp_;
    if (!latest_gps_valid_ || gps_recovery_pending_ || !latest_imu_valid_ || !pose_initialized_ ||
        !has_speed_ ||
        !IsSteadyFresh(gps_received_at_, now, config_.gps_timeout_s) ||
        !IsSteadyFresh(imu_received_at_, now, config_.imu_timeout_s)) {
        return state;
    }

    state.x = predicted_x_;
    state.y = predicted_y_;
    state.yaw = predicted_yaw_;
    state.speed = predicted_speed_mps_;
    state.acceleration = predicted_acceleration_mps2_;
    state.yaw_rate = predicted_yaw_rate_;
    state.valid = IsFinite(state.x) && IsFinite(state.y) &&
                  IsFinite(state.yaw) && IsFinite(state.speed) &&
                  IsFinite(state.acceleration) && IsFinite(state.yaw_rate);
    return state;
}

StateEstimatorDiagnostics GpsImuStateEstimator::diagnostics() const {
    StateEstimatorDiagnostics diagnostics;
    diagnostics.latest_gps_source_stamp = latest_gps_source_stamp_;
    diagnostics.state_source_stamp = state_source_stamp_;
    if (!latest_gps_source_stamp_.isZero() &&
        !state_source_stamp_.isZero()) {
        diagnostics.gps_to_state_source_lag_s =
            (state_source_stamp_ - latest_gps_source_stamp_).toSec();
    }
    diagnostics.last_gps_position_innovation_m =
        last_gps_position_innovation_m_;
    return diagnostics;
}

bool GpsImuStateEstimator::imuAt(const ros::Time& stamp,
                                 ImuSample& sample) const {
    if (imu_history_.empty() || stamp < imu_history_.front().source_stamp ||
        stamp > imu_history_.back().source_stamp) {
        return false;
    }
    for (std::size_t i = 0; i < imu_history_.size(); ++i) {
        const auto& right = imu_history_[i];
        const double exact_dt = (right.source_stamp - stamp).toSec();
        if (std::abs(exact_dt) <= kStampEpsilonS) {
            sample = right;
            sample.source_stamp = stamp;
            return true;
        }
        if (right.source_stamp > stamp && i > 0) {
            const auto& left = imu_history_[i - 1];
            const double interval =
                (right.source_stamp - left.source_stamp).toSec();
            if (interval <= 0.0) return false;
            const double ratio =
                (stamp - left.source_stamp).toSec() / interval;
            const double yaw_delta = NormalizeAngle(right.yaw - left.yaw);
            sample.source_stamp = stamp;
            sample.yaw = NormalizeAngle(left.yaw + ratio * yaw_delta);
            sample.yaw_rate = left.yaw_rate +
                ratio * (right.yaw_rate - left.yaw_rate);
            sample.acceleration_mps2 = left.acceleration_mps2 +
                ratio * (right.acceleration_mps2 - left.acceleration_mps2);
            return true;
        }
    }
    return false;
}

bool GpsImuStateEstimator::poseAt(const ros::Time& stamp,
                                  PoseSample& pose) const {
    if (pose_history_.empty() || stamp < pose_history_.front().source_stamp ||
        stamp > pose_history_.back().source_stamp) {
        return false;
    }
    for (std::size_t i = 0; i < pose_history_.size(); ++i) {
        const auto& right = pose_history_[i];
        const double exact_dt = (right.source_stamp - stamp).toSec();
        if (std::abs(exact_dt) <= kStampEpsilonS) {
            pose = right;
            pose.source_stamp = stamp;
            return true;
        }
        if (right.source_stamp > stamp && i > 0) {
            const auto& left = pose_history_[i - 1];
            const double interval =
                (right.source_stamp - left.source_stamp).toSec();
            if (interval <= 0.0) return false;
            const double ratio =
                (stamp - left.source_stamp).toSec() / interval;
            pose.source_stamp = stamp;
            pose.x = left.x + ratio * (right.x - left.x);
            pose.y = left.y + ratio * (right.y - left.y);
            pose.yaw = NormalizeAngle(
                left.yaw + ratio * NormalizeAngle(right.yaw - left.yaw));
            pose.speed_mps = left.speed_mps +
                ratio * (right.speed_mps - left.speed_mps);
            return true;
        }
    }
    return false;
}

void GpsImuStateEstimator::processPendingGps() {
    while (!pending_gps_.empty()) {
        ImuSample aligned_imu;
        if (!imuAt(pending_gps_.front().source_stamp, aligned_imu)) {
            // Wait only when the GPS source time is ahead of the newest IMU.
            // A GPS older than the retained IMU history can no longer be
            // corrected at a coherent source time, so reject it.
            if (imu_history_.empty() ||
                pending_gps_.front().source_stamp >
                    imu_history_.back().source_stamp) {
                return;
            }
            pending_gps_.pop_front();
            continue;
        }
        processGpsMeasurement(pending_gps_.front(), aligned_imu);
        pending_gps_.pop_front();
    }
}

void GpsImuStateEstimator::beginGpsRecovery() {
    // An outage breaks the GPS velocity baseline and absolute pose continuity.
    // Never advertise the old dead-reckoned pose as valid while a recovered
    // fix is still waiting for its matching IMU sample.
    gps_recovery_pending_ = true;
    pending_gps_.clear();
    position_window_.clear();
    has_stationary_anchor_ = false;
    has_speed_ = false;
}

void GpsImuStateEstimator::processGpsMeasurement(
    const GpsMeasurement& measurement, const ImuSample& aligned_imu) {
    const double cosine = std::cos(aligned_imu.yaw);
    const double sine = std::sin(aligned_imu.yaw);
    const double sensor_map_x =
        cosine * config_.gps_sensor_offset_x_m -
        sine * config_.gps_sensor_offset_y_m;
    const double sensor_map_y =
        sine * config_.gps_sensor_offset_x_m +
        cosine * config_.gps_sensor_offset_y_m;
    const PositionSample rear_axle{
        measurement.antenna_x - sensor_map_x,
        measurement.antenna_y - sensor_map_y,
        measurement.source_stamp};

    updateSpeedFromGps(rear_axle, aligned_imu.yaw);
    if (!pose_initialized_ || gps_recovery_pending_) {
        initializePose(rear_axle, aligned_imu);
        // Keep re-anchoring while collecting the new velocity baseline; a
        // zero-speed prediction during that interval must not pull GPS back.
        gps_recovery_pending_ = !has_speed_;
    } else {
        applyPositionCorrection(rear_axle, aligned_imu);
    }
}

void GpsImuStateEstimator::updateSpeedFromGps(
    const PositionSample& position, double aligned_yaw) {
    position_window_.push_back(position);
    while (position_window_.size() > 2 &&
           (position.source_stamp -
            position_window_.front().source_stamp).toSec() >
               config_.speed_window_duration_s &&
           (position.source_stamp -
            position_window_[1].source_stamp).toSec() >=
               config_.minimum_speed_baseline_s) {
        position_window_.pop_front();
    }

    if (!has_stationary_anchor_) {
        stationary_anchor_ = position;
        has_stationary_anchor_ = true;
    } else if (std::hypot(position.x - stationary_anchor_.x,
                          position.y - stationary_anchor_.y) >=
               config_.stationary_distance_m) {
        stationary_anchor_ = position;
    }
    const bool stationary = has_stationary_anchor_ &&
        (position.source_stamp - stationary_anchor_.source_stamp).toSec() >=
            config_.stationary_duration_s &&
        std::hypot(position.x - stationary_anchor_.x,
                   position.y - stationary_anchor_.y) <
            config_.stationary_distance_m;

    const PositionSample& oldest = position_window_.front();
    const double dt =
        (position.source_stamp - oldest.source_stamp).toSec();
    if (dt < config_.minimum_speed_baseline_s) return;

    const double dx = position.x - oldest.x;
    const double dy = position.y - oldest.y;
    const double longitudinal_distance =
        std::max(0.0, dx * std::cos(aligned_yaw) +
                          dy * std::sin(aligned_yaw));
    double gps_speed = longitudinal_distance / dt;
    if (!IsFinite(gps_speed) || gps_speed > config_.maximum_speed_mps) return;
    if (has_filtered_acceleration_) {
        gps_speed += filtered_acceleration_mps2_ * 0.5 * dt;
    }
    gps_speed = std::clamp(gps_speed, 0.0, config_.maximum_speed_mps);

    double correction = 0.0;
    if (!has_speed_) {
        speed_mps_ = gps_speed;
        has_speed_ = true;
    } else {
        PoseSample aligned_pose;
        const double estimated_speed =
            poseAt(position.source_stamp, aligned_pose)
                ? aligned_pose.speed_mps
                : speed_mps_;
        const double limited_innovation = std::clamp(
            gps_speed - estimated_speed, -config_.gps_innovation_limit_mps,
            config_.gps_innovation_limit_mps);
        double gain = config_.gps_correction_gain;
        // Use the smaller speed so braking/stopping restores the stronger
        // correction promptly. IMU propagation remains responsive to real
        // acceleration; only the noisy absolute GPS correction is reduced.
        const double speed = std::min(gps_speed, estimated_speed);
        if (config_.gps_gain_reference_speed_mps > 0.0 &&
            speed > config_.gps_gain_reference_speed_mps) {
            const double ratio = config_.gps_gain_reference_speed_mps / speed;
            gain = std::max(config_.gps_gain_minimum, gain * ratio * ratio);
        }
        correction = gain * limited_innovation;
        speed_mps_ = std::clamp(speed_mps_ + correction, 0.0,
                                config_.maximum_speed_mps);
    }

    if (stationary &&
        speed_mps_ <= config_.stationary_speed_threshold_mps &&
        std::abs(filtered_acceleration_mps2_) <=
            config_.stationary_acceleration_threshold_mps2) {
        correction -= speed_mps_;
        speed_mps_ = 0.0;
    }

    if (pose_initialized_ && std::abs(correction) > 0.0) {
        for (auto& pose : pose_history_) {
            if (pose.source_stamp >= position.source_stamp) {
                pose.speed_mps = std::clamp(
                    pose.speed_mps + correction, 0.0,
                    config_.maximum_speed_mps);
            }
        }
        predicted_speed_mps_ = std::clamp(
            predicted_speed_mps_ + correction, 0.0,
            config_.maximum_speed_mps);
    } else if (pose_initialized_ && !pose_history_.empty() &&
               pose_history_.back().source_stamp == state_source_stamp_) {
        predicted_speed_mps_ = speed_mps_;
        pose_history_.back().speed_mps = speed_mps_;
    }
}

void GpsImuStateEstimator::initializePose(
    const PositionSample& position, const ImuSample& aligned_imu) {
    predicted_x_ = position.x;
    predicted_y_ = position.y;
    predicted_yaw_ = aligned_imu.yaw;
    predicted_speed_mps_ = has_speed_ ? speed_mps_ : 0.0;
    predicted_acceleration_mps2_ = aligned_imu.acceleration_mps2;
    predicted_yaw_rate_ = aligned_imu.yaw_rate;
    last_gps_position_innovation_m_ = 0.0;
    state_source_stamp_ = position.source_stamp;
    pose_initialized_ = true;
    pose_history_.clear();
    appendCurrentPoseToHistory();

    // If the first usable GPS arrived late, replay only bounded, valid IMU
    // intervals. This keeps the state source time honest without integrating
    // across an unobserved timestamp gap.
    for (const auto& imu : imu_history_) {
        if (imu.source_stamp <= state_source_stamp_) continue;
        const double dt = (imu.source_stamp - state_source_stamp_).toSec();
        if (dt <= 0.0 || dt > config_.imu_maximum_dt_s) break;
        const double delta_yaw = NormalizeAngle(imu.yaw - predicted_yaw_);
        const double midpoint_yaw =
            NormalizeAngle(predicted_yaw_ + 0.5 * delta_yaw);
        predicted_x_ += predicted_speed_mps_ * std::cos(midpoint_yaw) * dt;
        predicted_y_ += predicted_speed_mps_ * std::sin(midpoint_yaw) * dt;
        predicted_yaw_ = imu.yaw;
        predicted_acceleration_mps2_ = imu.acceleration_mps2;
        predicted_yaw_rate_ = imu.yaw_rate;
        state_source_stamp_ = imu.source_stamp;
        appendCurrentPoseToHistory();
    }
}

void GpsImuStateEstimator::applyPositionCorrection(
    const PositionSample& position, const ImuSample& aligned_imu) {
    PoseSample predicted_at_gps;
    if (!poseAt(position.source_stamp, predicted_at_gps)) {
        if (position.source_stamp > state_source_stamp_) {
            initializePose(position, aligned_imu);
        }
        return;
    }

    // Innovation is deliberately computed at one sensor source time. The same
    // translation is then applied to the retained prediction and current pose;
    // the state is not rewound to an older GPS measurement.
    const double correction_x = config_.position_gps_correction_gain *
        (position.x - predicted_at_gps.x);
    const double correction_y = config_.position_gps_correction_gain *
        (position.y - predicted_at_gps.y);
    last_gps_position_innovation_m_ = std::hypot(
        position.x - predicted_at_gps.x,
        position.y - predicted_at_gps.y);
    for (auto& pose : pose_history_) {
        if (pose.source_stamp >= position.source_stamp) {
            pose.x += correction_x;
            pose.y += correction_y;
        }
    }
    const auto insertion = std::lower_bound(
        pose_history_.begin(), pose_history_.end(), position.source_stamp,
        [](const PoseSample& pose, const ros::Time& stamp) {
            return pose.source_stamp < stamp;
        });
    if (insertion == pose_history_.end() ||
        insertion->source_stamp != position.source_stamp) {
        PoseSample corrected_at_gps = predicted_at_gps;
        corrected_at_gps.x += correction_x;
        corrected_at_gps.y += correction_y;
        pose_history_.insert(insertion, corrected_at_gps);
    }
    predicted_x_ += correction_x;
    predicted_y_ += correction_y;
}

void GpsImuStateEstimator::appendCurrentPoseToHistory() {
    if (!pose_history_.empty() &&
        pose_history_.back().source_stamp == state_source_stamp_) {
        pose_history_.back() = {state_source_stamp_, predicted_x_, predicted_y_,
                                predicted_yaw_, predicted_speed_mps_};
        return;
    }
    pose_history_.push_back({state_source_stamp_, predicted_x_, predicted_y_,
                             predicted_yaw_, predicted_speed_mps_});
}

void GpsImuStateEstimator::pruneHistories(const ros::Time& newest_stamp) {
    while (imu_history_.size() > 2 &&
           (newest_stamp - imu_history_.front().source_stamp).toSec() >
               kHistoryDurationS) {
        imu_history_.pop_front();
    }
    while (pose_history_.size() > 2 &&
           (newest_stamp - pose_history_.front().source_stamp).toSec() >
               kHistoryDurationS) {
        pose_history_.pop_front();
    }
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

