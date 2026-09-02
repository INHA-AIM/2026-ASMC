#include <algorithm>
#include <memory>

#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "ros/state/vehicle_state_adapter.hpp"

namespace {
class GpsImuVehicleStateNode {
public:
    GpsImuVehicleStateNode() : private_nh_("~") {
        frenet_planner_ros::StateAdapterConfig config;
        private_nh_.param("timeouts/gps", config.gps_timeout_s, 0.20);
        private_nh_.param("timeouts/imu", config.imu_timeout_s, 0.20);
        private_nh_.param("speed/minimum_dt", config.minimum_speed_dt_s, 0.02);
        private_nh_.param("speed/maximum_dt", config.maximum_speed_dt_s, 1.0);
        private_nh_.param("speed/maximum_mps", config.maximum_speed_mps, 60.0);
        private_nh_.param("speed/filter_alpha", config.speed_filter_alpha, 0.35);
        estimator_ = std::make_unique<frenet_planner_ros::GpsImuStateEstimator>(
            config);

        std::string gps_topic, imu_topic, output_topic;
        private_nh_.param<std::string>("topics/gps", gps_topic, "/gps");
        private_nh_.param<std::string>("topics/imu", imu_topic, "/imu");
        private_nh_.param<std::string>(
            "publish_topic", output_topic, "/vehicle/state");
        double frequency = 50.0;
        private_nh_.param("publish_frequency", frequency, 50.0);
        publisher_ = nh_.advertise<asmc_msgs::VehicleState>(output_topic, 1);
        gps_subscriber_ = nh_.subscribe(
            gps_topic, 1, &GpsImuVehicleStateNode::gpsCallback, this);
        imu_subscriber_ = nh_.subscribe(
            imu_topic, 1, &GpsImuVehicleStateNode::imuCallback, this);
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(1.0, frequency)),
            &GpsImuVehicleStateNode::timerCallback, this);
        ROS_INFO("GPS/IMU state source enabled; EgoStatus fallback is disabled");
    }

private:
    void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message) {
        estimator_->updateGps(*message);
    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
        estimator_->updateImu(*message);
    }
    void timerCallback(const ros::TimerEvent&) {
        const auto state = estimator_->makeState(ros::Time::now());
        publisher_.publish(state);
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
    ros::Timer timer_;
};
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_imu_vehicle_state_adapter");
    GpsImuVehicleStateNode node;
    ros::spin();
    return 0;
}
