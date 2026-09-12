#include "SensorFusion.hpp"

bool g_show_text = true;
std_msgs::ColorRGBA g_box_color;

void FusionProcess(RAW_YOLO_DATA& raw, LiDAR& lidar, CAMERA &camera,
                   const ros::Time& lidar_stamp)
{
    runCameraFusion(raw, lidar, camera, lidar_stamp);
    FusionLiDAR(lidar);
}
