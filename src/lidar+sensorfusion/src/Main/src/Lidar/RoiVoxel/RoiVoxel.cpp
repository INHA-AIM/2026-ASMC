#include <Lidar/RoiVoxel/RoiVoxel.hpp>
#include <Eigen/Geometry>




// ROI 필터링
void Roi(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud1,
         pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud2,
         const LiDAR& lidar,
         const VehicleSpec& vehicle) {

    // LiDAR 좌표계 기준 자차가 차지하는 직사각형 영역
    const float ego_x_min = -vehicle.f32_RearOverhang;
    const float ego_x_max = vehicle.f32_WheelBase + vehicle.f32_FrontOverhang;
    const float ego_y_min = -0.5f * vehicle.f32_Width;
    const float ego_y_max = 0.5f * vehicle.f32_Width;

    const float roll = Deg2Rad(vehicle.st_LiDARRotationDeg.f32_X);
    const float pitch = Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Y);
    const float yaw = Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Z);
    const Eigen::Matrix3f vehicle_from_lidar_rotation =
        (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
    const Eigen::Vector3f vehicle_from_lidar_translation(
        vehicle.st_LiDARPos.f32_X,
        vehicle.st_LiDARPos.f32_Y,
        vehicle.st_LiDARPos.f32_Z);

    // 함수가 재사용되더라도 이전 프레임의 점이 누적되지 않게 초기화한다.
    pcl_Cloud2->clear();
    pcl_Cloud2->points.reserve(pcl_Cloud1->points.size());

    for (const auto& pcl_Point : pcl_Cloud1->points) {
        const float range = std::sqrt(pcl_Point.x * pcl_Point.x +
                                      pcl_Point.y * pcl_Point.y +
                                      pcl_Point.z * pcl_Point.z);
        const bool inside_roi =
            range >= lidar.st_Range.f32_Min && range <= lidar.st_Range.f32_Max &&
            pcl_Point.x >= lidar.st_ROI.f32_MinX && pcl_Point.x <= lidar.st_ROI.f32_MaxX &&
            pcl_Point.y >= lidar.st_ROI.f32_MinY && pcl_Point.y <= lidar.st_ROI.f32_MaxY &&
            pcl_Point.z >= lidar.st_ROI.f32_MinZ && pcl_Point.z <= lidar.st_ROI.f32_MaxZ;

        if (!inside_roi) continue;

        const Eigen::Vector3f point_vehicle =
            vehicle_from_lidar_rotation *
                Eigen::Vector3f(pcl_Point.x, pcl_Point.y, pcl_Point.z) +
            vehicle_from_lidar_translation;
        const bool inside_vehicle =
            point_vehicle.x() >= ego_x_min && point_vehicle.x() <= ego_x_max &&
            point_vehicle.y() >= ego_y_min && point_vehicle.y() <= ego_y_max;

        if (inside_vehicle) continue;

        pcl_Cloud2->points.push_back(pcl_Point);
    }
    pcl_Cloud2->width = static_cast<uint32_t>(pcl_Cloud2->points.size());
    pcl_Cloud2->height = 1;
    pcl_Cloud2->is_dense = false;
}

// Voxel Grid 필터링
void Voxel(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud2, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud3) {
    pcl::VoxelGrid<pcl::PointXYZI> pcl_Sor;
    pcl_Sor.setInputCloud(pcl_Cloud2);              // 입력
    pcl_Sor.setLeafSize(0.1f, 0.1f, 0.1f);   // leaf size 10cm
    pcl_Sor.filter(*pcl_Cloud3);
}
