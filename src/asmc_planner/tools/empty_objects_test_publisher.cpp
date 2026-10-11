#include <algorithm>
#include <string>

#include <planning_inputs/DetectedObjectArray.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "empty_objects_test_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string topic;
    double frequency = 20.0;
    private_nh.param<std::string>(
        "publish_topic", topic, "/planning/objects");
    private_nh.param("publish_frequency", frequency, 20.0);
    auto publisher =
        nh.advertise<planning_inputs::DetectedObjectArray>(topic, 1);
    ROS_WARN("Publishing empty objects for obstacle-free KEEP verification only");
    ros::SteadyTimer timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / std::max(1.0, frequency)),
        [publisher](const ros::SteadyTimerEvent&) mutable {
        planning_inputs::DetectedObjectArray message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = "map";
        publisher.publish(message);
        });
    ros::spin();
    return 0;
}
