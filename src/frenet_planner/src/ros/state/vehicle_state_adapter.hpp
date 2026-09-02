#pragma once

#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>

namespace frenet_planner_ros {

struct StateAdapterConfig {
    double gps_timeout_s = 0.20;
    double imu_timeout_s = 0.20;
    double minimum_speed_dt_s = 0.02;
    double maximum_speed_dt_s = 1.0;
    double maximum_speed_mps = 60.0;
    double speed_filter_alpha = 0.35;
};

class GpsImuStateEstimator {
public:
    explicit GpsImuStateEstimator(const StateAdapterConfig& config);
    void updateGps(const morai_msgs::GPSMessage& gps);
    void updateImu(const sensor_msgs::Imu& imu);
    asmc_msgs::VehicleState makeState(const ros::Time& now) const;

private:
    StateAdapterConfig config_;
    morai_msgs::GPSMessage gps_;
    sensor_msgs::Imu imu_;
    bool has_gps_ = false;
    bool has_imu_ = false;
    bool has_previous_fix_ = false;
    bool has_speed_ = false;
    double previous_x_ = 0.0;
    double previous_y_ = 0.0;
    ros::Time previous_gps_stamp_;
    double speed_mps_ = 0.0;
};

asmc_msgs::VehicleState ConvertEgoVehicleStatus(
    const morai_msgs::EgoVehicleStatus& ego);

}  // namespace frenet_planner_ros
