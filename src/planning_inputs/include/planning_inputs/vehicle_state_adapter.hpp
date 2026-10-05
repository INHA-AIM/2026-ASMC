#pragma once

#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>

#include <deque>

namespace frenet_planner_ros {

struct StateAdapterConfig {
    // MORAI GPS reports the antenna position. Express the antenna lever arm in
    // the vehicle frame (x forward, y left) so /vehicle/state remains the rear
    // axle reference expected by Behavior/Frenet/MPC.
    double gps_sensor_offset_x_m = 0.35;
    double gps_sensor_offset_y_m = 0.0;
    double gps_timeout_s = 0.20;
    double imu_timeout_s = 0.20;
    double speed_window_duration_s = 0.18;
    double minimum_speed_baseline_s = 0.12;
    double maximum_speed_mps = 60.0;
    double position_gps_correction_gain = 1.0;
    double gps_correction_gain = 0.35;
    double gps_innovation_limit_mps = 3.0;
    double imu_acceleration_filter_alpha = 0.50;
    double imu_acceleration_deadband_mps2 = 0.10;
    double imu_acceleration_min_mps2 = -12.0;
    double imu_acceleration_max_mps2 = 8.0;
    double imu_maximum_dt_s = 0.10;
    double stationary_duration_s = 0.25;
    double stationary_distance_m = 0.05;
    double stationary_speed_threshold_mps = 0.50;
    double stationary_acceleration_threshold_mps2 = 0.20;
};

struct StateEstimatorDiagnostics {
    ros::Time latest_gps_source_stamp;
    ros::Time state_source_stamp;
    // How far the coherent predicted state has advanced beyond the latest GPS
    // correction in sensor source time (not wall/receive time).
    double gps_to_state_source_lag_s = 0.0;
    // Magnitude before applying position_gps_correction_gain.
    double last_gps_position_innovation_m = 0.0;
};

class GpsImuStateEstimator {
public:
    explicit GpsImuStateEstimator(const StateAdapterConfig& config);
    void updateGps(const morai_msgs::GPSMessage& gps,
                   const ros::SteadyTime& received_at);
    void updateImu(const sensor_msgs::Imu& imu,
                   const ros::SteadyTime& received_at);
    asmc_msgs::VehicleState makeState(const ros::SteadyTime& now) const;
    StateEstimatorDiagnostics diagnostics() const;

private:
    struct ImuSample {
        ros::Time source_stamp;
        double yaw = 0.0;
        double yaw_rate = 0.0;
        double acceleration_mps2 = 0.0;
    };
    struct GpsMeasurement {
        ros::Time source_stamp;
        double antenna_x = 0.0;
        double antenna_y = 0.0;
    };
    StateAdapterConfig config_;
    bool has_gps_ = false;
    bool has_imu_ = false;
    bool latest_gps_valid_ = false;
    bool latest_imu_valid_ = false;
    bool has_speed_ = false;
    struct PositionSample {
        double x = 0.0;
        double y = 0.0;
        ros::Time source_stamp;
    };
    struct PoseSample {
        ros::Time source_stamp;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double speed_mps = 0.0;
    };
    std::deque<PositionSample> position_window_;
    std::deque<ImuSample> imu_history_;
    std::deque<PoseSample> pose_history_;
    std::deque<GpsMeasurement> pending_gps_;
    ros::SteadyTime gps_received_at_;
    ros::SteadyTime imu_received_at_;
    ros::Time latest_gps_source_stamp_;
    ros::Time latest_imu_source_stamp_;
    PositionSample stationary_anchor_{};
    bool has_stationary_anchor_ = false;
    bool has_filtered_acceleration_ = false;
    double filtered_acceleration_mps2_ = 0.0;
    double speed_mps_ = 0.0;

    bool pose_initialized_ = false;
    ros::Time state_source_stamp_;
    double predicted_x_ = 0.0;
    double predicted_y_ = 0.0;
    double predicted_yaw_ = 0.0;
    double predicted_speed_mps_ = 0.0;
    double predicted_acceleration_mps2_ = 0.0;
    double predicted_yaw_rate_ = 0.0;
    double last_gps_position_innovation_m_ = 0.0;

    bool imuAt(const ros::Time& stamp, ImuSample& sample) const;
    bool poseAt(const ros::Time& stamp, PoseSample& pose) const;
    void processPendingGps();
    void processGpsMeasurement(const GpsMeasurement& measurement,
                               const ImuSample& aligned_imu);
    void updateSpeedFromGps(const PositionSample& position,
                            double aligned_yaw);
    void initializePose(const PositionSample& position,
                        const ImuSample& aligned_imu);
    void applyPositionCorrection(const PositionSample& position,
                                 const ImuSample& aligned_imu);
    void appendCurrentPoseToHistory();
    void pruneHistories(const ros::Time& newest_stamp);
};

asmc_msgs::VehicleState ConvertEgoVehicleStatus(
    const morai_msgs::EgoVehicleStatus& ego);

}  // namespace frenet_planner_ros

