#pragma once

#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>

#include <deque>

namespace frenet_planner_ros {

struct StateAdapterConfig {
    double gps_timeout_s = 0.20;
    double imu_timeout_s = 0.20;
    double speed_window_duration_s = 0.18;
    double minimum_speed_baseline_s = 0.12;
    double maximum_speed_mps = 60.0;
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

class GpsImuStateEstimator {
public:
    explicit GpsImuStateEstimator(const StateAdapterConfig& config);
    void updateGps(const morai_msgs::GPSMessage& gps,
                   const ros::SteadyTime& received_at);
    void updateImu(const sensor_msgs::Imu& imu,
                   const ros::SteadyTime& received_at);
    asmc_msgs::VehicleState makeState(const ros::SteadyTime& now) const;

private:
    StateAdapterConfig config_;
    morai_msgs::GPSMessage gps_;
    sensor_msgs::Imu imu_;
    bool has_gps_ = false;
    bool has_imu_ = false;
    bool has_speed_ = false;
    struct PositionSample {
        double x;
        double y;
        ros::SteadyTime received_at;
    };
    std::deque<PositionSample> position_window_;
    ros::SteadyTime gps_received_at_;
    ros::SteadyTime imu_received_at_;
    ros::SteadyTime previous_imu_received_at_;
    PositionSample stationary_anchor_{};
    bool has_stationary_anchor_ = false;
    bool has_filtered_acceleration_ = false;
    double filtered_acceleration_mps2_ = 0.0;
    double speed_mps_ = 0.0;
};

asmc_msgs::VehicleState ConvertEgoVehicleStatus(
    const morai_msgs::EgoVehicleStatus& ego);

}  // namespace frenet_planner_ros
