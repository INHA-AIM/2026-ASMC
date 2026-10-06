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
void runCameraFusion(RAW_YOLO_DATA& raw, LiDAR& lidar, CAMERA& camera,
                     const ros::Time& lidar_stamp);

// Parser callbacks enqueue every source frame here.  Keeping the queues at
// the callback boundary prevents the slower processing thread from silently
// replacing intermediate camera/YOLO frames with only the newest value.
void BufferFusionCameraFrame(const ros::Time& stamp,
                             const std::vector<uint8_t>& compressed);
void BufferFusionYoloFrame(uint64 frame_seq, const ros::Time& image_stamp,
                           const std::vector<BBox>& boxes);
// Called after worker shutdown; includes the last partial 5-second interval.
void ReportFusionTotals();

// LiDAR 시각화
void InitFusionLidarViz(ros::NodeHandle& nh);
void FusionLiDAR(LiDAR &st_LiDAR);
void SetLiDARObjectClassAll(LiDAR& st_LiDAR, const std::string& class_name);
void SetLiDARClassWithGroundNoise(LiDAR& st_LiDAR, const std::string& class_name, float z_floor);

void FusionProcess(RAW_YOLO_DATA& raw, LiDAR& lidar, CAMERA &camera,
                   const ros::Time& lidar_stamp);

#endif
