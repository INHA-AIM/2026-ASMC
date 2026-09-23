#include <algorithm>
#include <memory>

#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "vehicle_state/vehicle_state_adapter.hpp"

namespace {
class GpsImuVehicleStateNode {
public:
    GpsImuVehicleStateNode()
        : private_nh_("~"), started_at_(ros::SteadyTime::now()) {
        frenet_planner_ros::StateAdapterConfig config;
        private_nh_.param("timeouts/gps", config.gps_timeout_s, 0.20);
        private_nh_.param("timeouts/imu", config.imu_timeout_s, 0.20);
        private_nh_.param("speed/window_duration", config.speed_window_duration_s, 0.18);
        private_nh_.param("speed/minimum_baseline", config.minimum_speed_baseline_s, 0.12);
        private_nh_.param("speed/maximum_mps", config.maximum_speed_mps, 60.0);
        private_nh_.param("speed/gps_correction_gain",
                          config.gps_correction_gain, 0.35);
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

        std::string gps_topic, imu_topic, output_topic;
        private_nh_.param<std::string>("topics/gps", gps_topic, "/gps");
        private_nh_.param<std::string>("topics/imu", imu_topic, "/imu");
        private_nh_.param<std::string>(
            "publish_topic", output_topic, "/vehicle/state");
        double frequency = 50.0;
        private_nh_.param("publish_frequency", frequency, 50.0);
        private_nh_.param(
            "diagnostics/startup_grace", startup_grace_s_, 1.0);
        publisher_ = nh_.advertise<asmc_msgs::VehicleState>(output_topic, 1);
        gps_subscriber_ = nh_.subscribe(
            gps_topic, 1, &GpsImuVehicleStateNode::gpsCallback, this);
        imu_subscriber_ = nh_.subscribe(
            imu_topic, 1, &GpsImuVehicleStateNode::imuCallback, this);
        timer_ = nh_.createSteadyTimer(
            ros::WallDuration(1.0 / std::max(1.0, frequency)),
            &GpsImuVehicleStateNode::timerCallback, this);
        ROS_INFO("GPS/IMU state source enabled; EgoStatus fallback is disabled");
    }

private:
    void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message) {
        estimator_->updateGps(*message, ros::SteadyTime::now());
    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
        estimator_->updateImu(*message, ros::SteadyTime::now());
    }
    void timerCallback(const ros::SteadyTimerEvent&) {
        const auto now = ros::SteadyTime::now();
        const auto state = estimator_->makeState(now);
        publisher_.publish(state);
        if (state.valid) {
            has_published_valid_state_ = true;
        } else if (!has_published_valid_state_ &&
                   (now - started_at_).toSec() < startup_grace_s_) {
            ROS_INFO_ONCE(
                "GPS/IMU vehicle state is waiting for its initial valid sensor window");
        } else {
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
    ros::SteadyTime started_at_;
    double startup_grace_s_ = 1.0;
    bool has_published_valid_state_ = false;
};
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_imu_vehicle_state_adapter");
    GpsImuVehicleStateNode node;
    ros::spin();
    return 0;
}
