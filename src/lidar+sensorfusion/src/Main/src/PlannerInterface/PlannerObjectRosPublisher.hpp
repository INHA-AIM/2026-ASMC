#pragma once

#include <cstddef>
#include <string>

#include <opencv2/core.hpp>
#include <ros/ros.h>

#include <Global/Global.hpp>

struct PlannerObject;

class PlannerObjectRosPublisher {
public:
    explicit PlannerObjectRosPublisher(ros::NodeHandle& node_handle);

    void Publish(const LiDAR& lidar, const GPS_IMU& gps_imu,
                 const VehicleSpec& vehicle, const ros::Time& stamp);

private:
    void LoadDrivableMask(const std::string& image_path,
                          const std::string& metadata_path,
                          const std::string& expected_map_id,
                          double margin_m);
    double DrivableRatio(const PlannerObject& object) const;

    ros::Publisher objects_publisher_;
    ros::Publisher markers_publisher_;
    ros::Publisher last_objects_publisher_;
    ros::Publisher last_markers_publisher_;
    ros::Publisher nogps_objects_publisher_;
    ros::Publisher nogps_markers_publisher_;
    std::size_t max_objects_ = 200;
    double velocity_arrow_seconds_ = 1.0;
    cv::Mat drivable_mask_;
    double pixels_per_meter_ = 0.0;
    double world_offset_x_ = 0.0;
    double world_offset_y_ = 0.0;
    double drivable_ratio_threshold_ = 0.3;
    int drivable_samples_ = 5;
    double tunnel_ceiling_center_z_m_ = 1.3;
    double tunnel_wall_min_length_m_ = 15.0;
    double tunnel_wall_min_aspect_ratio_ = 6.0;
    double tunnel_wall_min_abs_y_m_ = 4.0;
    ros::Time last_stamp_;
};
