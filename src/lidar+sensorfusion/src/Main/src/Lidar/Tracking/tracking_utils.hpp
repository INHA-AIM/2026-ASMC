
#ifndef FILTER_TRACKING_UTILS_HPP
#define FILTER_TRACKING_UTILS_HPP

#include <Global/Global.hpp>
#include "../HungarianAlg/HungarianAlg.hpp"

void Objectinfo_for_km(vector<LiDARObjectInfo> &vec_NowObject,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_Cloud4,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_Cloud5,
                        vector<pcl::PointIndices> vec_AllClusterIndices, const GPS_IMU& st_GPSIMU, VehicleSpec& st_VehicleSpec);

void CompareCentroids(const deque<vector<LiDARObjectInfo>> &past_objects_queue,
                      vector<LiDARObjectInfo> &now_object,
                      float64 f64_Dt,const GPS_IMU& st_GPSIMU);

#endif // FILTER_TRACKING_UTILS_H
