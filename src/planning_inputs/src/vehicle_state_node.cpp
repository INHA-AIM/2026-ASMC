#include <algorithm>
#include <memory>

#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "planning_inputs/vehicle_state_adapter.hpp"

namespace {
class GpsImuVehicleStateNode {
public:
    GpsImuVehicleStateNode() : private_nh_("~") {
        frenet_planner_ros::StateAdapterConfig config;
        private_nh_.param("gps_sensor_offset/x",
                          config.gps_sensor_offset_x_m, 0.35);
        private_nh_.param("gps_sensor_offset/y",
                          config.gps_sensor_offset_y_m, 0.0);
        private_nh_.param("timeouts/gps", config.gps_timeout_s, 0.20);
        private_nh_.param("timeouts/imu", config.imu_timeout_s, 0.20);
        private_nh_.param("speed/window_duration", config.speed_window_duration_s, 0.18);
        private_nh_.param("speed/minimum_baseline", config.minimum_speed_baseline_s, 0.12);
        private_nh_.param("speed/maximum_mps", config.maximum_speed_mps, 60.0);
        private_nh_.param("position/gps_correction_gain",
                          config.position_gps_correction_gain, 1.0);
        private_nh_.param("orientation/imu_yaw_is_true_enu",
                          config.imu_yaw_is_true_enu, false);
        private_nh_.param("speed/gps_correction_gain",
                          config.gps_correction_gain, 0.35);
        private_nh_.param("speed/gps_gain_reference_speed_mps",
                          config.gps_gain_reference_speed_mps, 0.0);
        private_nh_.param("speed/gps_gain_minimum", config.gps_gain_minimum, 0.08);
        private_nh_.param("speed/gps_innovation_limit_mps",
                          config.gps_innovation_limit_mps, 3.0);
        private_nh_.param("speed/imu_acceleration_filter_alpha",
                          config.imu_acceleration_filter_alpha, 0.50);
        private_nh_.param("speed/imu_acceleration_deadband_mps2",
                          config.imu_acceleration_deadband_mps2, 0.10);
        private_nh_.param("speed/imu_acceleration_min_mps2",
                          config.imu_acceleration_min_mps2, -12.0);
        private_nh_.param("speed/imu_acceleration_max_mps2",
                          config.imu_acceleration_max_mps2, 8.0);
        private_nh_.param("speed/imu_maximum_dt",
                          config.imu_maximum_dt_s, 0.10);
        private_nh_.param("speed/stationary_duration",
                          config.stationary_duration_s, 0.25);
        private_nh_.param("speed/stationary_distance",
                          config.stationary_distance_m, 0.05);
        private_nh_.param("speed/stationary_speed_threshold_mps",
                          config.stationary_speed_threshold_mps, 0.50);
        private_nh_.param("speed/stationary_acceleration_threshold_mps2",
                          config.stationary_acceleration_threshold_mps2, 0.20);
        estimator_ = std::make_unique<frenet_planner_ros::GpsImuStateEstimator>(
            config);

        if (config.gps_gain_reference_speed_mps > 0.0)
            ROS_INFO("GPS speed gain scheduling: reference=%.2f m/s minimum=%.3f",
                     config.gps_gain_reference_speed_mps, config.gps_gain_minimum);

        std::string gps_topic, imu_topic, output_topic;
        private_nh_.param<std::string>("topics/gps", gps_topic, "/gps");
        private_nh_.param<std::string>("topics/imu", imu_topic, "/imu");
        private_nh_.param<std::string>(
            "publish_topic", output_topic, "/planning/vehicle/state");
        double frequency = 50.0;
        private_nh_.param("publish_frequency", frequency, 50.0);
        publisher_ = nh_.advertise<asmc_msgs::VehicleState>(output_topic, 1);
        gps_subscriber_ = nh_.subscribe(
            gps_topic, 1, &GpsImuVehicleStateNode::gpsCallback, this);
        imu_subscriber_ = nh_.subscribe(
            imu_topic, 1, &GpsImuVehicleStateNode::imuCallback, this);
        timer_ = nh_.createSteadyTimer(
            ros::WallDuration(1.0 / std::max(1.0, frequency)),
            &GpsImuVehicleStateNode::timerCallback, this);
        ROS_INFO(
            "GPS/IMU state source enabled; rear-axle lever arm correction "
            "x=%.3f y=%.3f m; position GPS gain=%.2f; "
            "IMU true-ENU to UTM grid correction=%s; "
            "EgoStatus fallback is disabled",
            config.gps_sensor_offset_x_m, config.gps_sensor_offset_y_m,
            config.position_gps_correction_gain,
            config.imu_yaw_is_true_enu ? "enabled" : "disabled");
    }

private:
    void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message) {
        estimator_->updateGps(*message, ros::SteadyTime::now());
    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
        estimator_->updateImu(*message, ros::SteadyTime::now());
    }
    void timerCallback(const ros::SteadyTimerEvent&) {
        const auto state = estimator_->makeState(ros::SteadyTime::now());
        publisher_.publish(state);
        const auto diagnostics = estimator_->diagnostics();
        ROS_DEBUG_THROTTLE(
            2.0,
            "GPS/IMU state timing: gps_source=%.6f state_source=%.6f "
            "gps_to_state_lag=%.3f s last_position_innovation=%.3f m",
            diagnostics.latest_gps_source_stamp.toSec(),
            diagnostics.state_source_stamp.toSec(),
            diagnostics.gps_to_state_source_lag_s,
            diagnostics.last_gps_position_innovation_m);
        if (!state.valid) {
            ROS_WARN_THROTTLE(
                2.0, "GPS/IMU vehicle state unavailable, invalid, or stale");
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::unique_ptr<frenet_planner_ros::GpsImuStateEstimator> estimator_;
    ros::Subscriber gps_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher publisher_;
    ros::SteadyTimer timer_;
};
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_imu_vehicle_state_adapter");
    GpsImuVehicleStateNode node;
    ros::spin();
    return 0;
}

