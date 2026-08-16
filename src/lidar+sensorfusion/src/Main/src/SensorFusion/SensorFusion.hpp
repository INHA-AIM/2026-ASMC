#ifndef SENSORFUSION_HPP
#define SENSORFUSION_HPP

#include <Global/Global.hpp>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <string>
#include <ros/ros.h>

extern CAMERA st_Camera;
extern LiDAR  st_LiDAR;
extern bool g_show_text;
extern std_msgs::ColorRGBA g_box_color;

// 카메라 오케스트레이션
void runCameraFusion(RAW_YOLO_DATA& raw, LiDAR& lidar, CAMERA& camera);

// LiDAR 시각화
void InitFusionLidarViz(ros::NodeHandle& nh);
void FusionLiDAR(LiDAR &st_LiDAR);
void SetLiDARObjectClassAll(LiDAR& st_LiDAR, const std::string& class_name);
void SetLiDARClassWithGroundNoise(LiDAR& st_LiDAR, const std::string& class_name, float z_floor);

void FusionProcess(RAW_YOLO_DATA& raw, LiDAR& lidar, CAMERA &camera);

#endif