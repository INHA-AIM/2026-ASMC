#include "SensorFusion.hpp"

bool g_show_text = true;
std_msgs::ColorRGBA g_box_color;

void FusionProcess(RAW_YOLO_DATA& raw, LiDAR& lidar, CAMERA &camera,
                   const ros::Time& lidar_stamp)
{
    runCameraFusion(raw, lidar, camera, lidar_stamp);
    FusionLiDAR(lidar);
}

void FusionProcessAt(RAW_YOLO_DATA& raw, LiDAR& lidar, CAMERA& camera,
                     const ros::Time& stamp, const ros::WallTime& decision_time) {
    runCameraFusion(raw, lidar, camera, stamp, decision_time);
    FusionLiDAR(lidar);
}
