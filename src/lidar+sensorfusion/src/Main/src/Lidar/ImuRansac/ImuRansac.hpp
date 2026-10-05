#ifndef IMU_RANSAC_HPP
#define IMU_RANSAC_HPP

#include <Global/Global.hpp>
// Only the plane actually used for the current processed frame is exposed.
GroundPlaneSnapshot CurrentGroundPlaneSnapshot();
// IMU 보정 함수 (ImuFusion.cpp에서 구현)
void CorrectingPoints(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output,const GPS_IMU& st_GPSIMU
);

// RANSAC 필터링 함수 (Ransac.cpp에서 구현)
void NormalVectorFiltering(
    pcl::PointCloud<pcl::PointXYZI>::Ptr sky4,
    pcl::PointCloud<pcl::PointXYZI>::Ptr sky5,
    const GPS_IMU& st_GPSIMU
);

#endif // IMU_RANSAC_H
