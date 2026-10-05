
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

// 같은 프레임에서 중심과 XY 점유가 거의 같은 수직 분할 클러스터를 하나의
// 실측 객체로 합친다. 서로 떨어진 객체나 단순히 가까운 객체는 합치지 않는다.
void ConsolidateCurrentDetections(vector<LiDARObjectInfo>& now_object);

#endif // FILTER_TRACKING_UTILS_H
