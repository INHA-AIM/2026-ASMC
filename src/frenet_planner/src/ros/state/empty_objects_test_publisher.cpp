#include <algorithm>
#include <string>

#include <asmc_msgs/DetectedObjectArray.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "empty_objects_test_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string topic;
    double frequency = 20.0;
    private_nh.param<std::string>(
        "publish_topic", topic, "/perception/objects");
    private_nh.param("publish_frequency", frequency, 20.0);
    auto publisher =
        nh.advertise<asmc_msgs::DetectedObjectArray>(topic, 1);
    ros::Rate rate(std::max(1.0, frequency));
    ROS_WARN("Publishing empty objects for obstacle-free KEEP verification only");
    while (ros::ok()) {
        asmc_msgs::DetectedObjectArray message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = "map";
        publisher.publish(message);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
