#ifndef LIDARPROCESS_HPP
#define LIDARPROCESS_HPP

#include <Global/Global.hpp>
#include "../Clustering/Clustering.hpp"
#include "../RoiVoxel/RoiVoxel.hpp"
#include "../Kalman/kalman_utils.hpp"
#include "../HungarianAlg/HungarianAlg.hpp"
#include "../ImuRansac/ImuRansac.hpp"
#include "../UDPReceive/LiDARReceiver.hpp"
#include "../Tracking/tracking_utils.hpp"
#include "../Visualization/Visualization.hpp"
#include "../FrameFilter/FrameFilter.hpp"

class MotionClassifier;

void LidarProcess(const RAW_LIDAR_DATA& st_RawLiDAR,
                  const GPS_IMU& st_GPSIMU,
                  LiDAR& st_LiDAR,
                  MotionClassifier& motion_classifier);
void InitLidarViz(ros::NodeHandle& nh);
pcl::PointCloud<pcl::PointXYZI>::Ptr
MakePointCloudFromRaw(const RAW_LIDAR_DATA& st_RawLiDAR);
#endif
