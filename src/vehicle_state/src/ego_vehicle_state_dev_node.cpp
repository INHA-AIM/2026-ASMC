#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <ros/ros.h>

#include "vehicle_state/vehicle_state_adapter.hpp"

namespace {
class EgoVehicleStateDevNode {
public:
    EgoVehicleStateDevNode() : private_nh_("~") {
        std::string input_topic, output_topic;
        private_nh_.param<std::string>("input_topic", input_topic, "/Ego_topic");
        private_nh_.param<std::string>(
            "publish_topic", output_topic, "/vehicle/state");
        publisher_ = nh_.advertise<asmc_msgs::VehicleState>(output_topic, 1);
        subscriber_ = nh_.subscribe(
            input_topic, 1, &EgoVehicleStateDevNode::callback, this);
        ROS_WARN("Development EgoStatus state source enabled; not for competition");
    }

private:
    void callback(const morai_msgs::EgoVehicleStatus::ConstPtr& message) {
        publisher_.publish(
            frenet_planner_ros::ConvertEgoVehicleStatus(*message));
    }
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
};
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "ego_vehicle_state_dev_adapter");
    EgoVehicleStateDevNode node;
    ros::spin();
    return 0;
}
