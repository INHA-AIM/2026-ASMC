#ifndef CLUSTERING_HPP   
#define CLUSTERING_HPP
#include <Global/Global.hpp>

float AdaptiveClusterTolerance(
    float horizontal_range_m,
    const LiDARClustering& config);

// Clustering 함수 선언
void Clustering(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud4,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud5,
    std::vector<pcl::PointIndices> &vp_AllClusterIndices,
    const GPS_IMU& st_GPSIMU,
    const LiDARClustering& config
);

#endif 
