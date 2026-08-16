#include "SensorFusion.hpp"

bool g_show_text = true;
std_msgs::ColorRGBA g_box_color;

void FusionProcess(RAW_YOLO_DATA& raw, LiDAR& lidar, CAMERA &camera)
{
    runCameraFusion(raw, lidar, camera);
    FusionLiDAR(lidar);
}