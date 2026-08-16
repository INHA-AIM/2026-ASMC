#ifndef ROI_VOXEL_HPP
#define ROI_VOXEL_HPP
#include <Global/Global.hpp>
// ROI 필터링 함수
void Roi(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud1,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud2,
    const LiDAR& lidar,
    const VehicleSpec& vehicle
);

// Voxel Grid 필터링 함수
void Voxel(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud2,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud3
);

#endif // ROI_VOXEL_H
