#pragma once

#include <cstddef>

#include <ros/ros.h>

#include <Global/Global.hpp>

class PlannerObjectRosPublisher {
public:
    explicit PlannerObjectRosPublisher(ros::NodeHandle& node_handle);

    void Publish(const LiDAR& lidar, const GPS_IMU& gps_imu,
                 const VehicleSpec& vehicle, const ros::Time& stamp);

private:
    ros::Publisher objects_publisher_;
    ros::Publisher markers_publisher_;
    std::size_t max_objects_ = 200;
    double velocity_arrow_seconds_ = 1.0;
    ros::Time last_stamp_;
};
