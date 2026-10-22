// ROI/ego-column removal and voxel grid.
// Ported from Main src/Lidar/RoiVoxel/RoiVoxel.cpp at 8f93c22e. Roi and Voxel
// have no legacy/common branch in Main; both run in the raw sensor frame.

#include <asmc_lidar/pipeline/preprocessing.hpp>

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <cstdint>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

// Same value as glibc M_PI (Main uses M_PI); spelled out so strict -std=c++17 builds too.
constexpr double kMainPi = 3.14159265358979323846;

// Ported from Main src/Global/Global.cpp:10-13 (8f93c22e)
// Float in, double multiply/divide (M_PI is double), float out.
float32 MainDeg2Rad(const float32& f32_Deg) { return f32_Deg * kMainPi / 180.0f; }

}  // namespace

// Ported from Main src/Lidar/RoiVoxel/RoiVoxel.cpp:8-62 (8f93c22e)
// Main reads Range/ROI from LiDAR st_Range/st_ROI; here they are explicit parameters.
void Roi(const CloudPtr& input, const CloudPtr& output, const RangeConfig& range,
         const RoiConfig& roi, const VehicleSpec& vehicle) {
  // Ego-vehicle rectangle in the rear-axle vehicle frame (no z limit).
  const float ego_x_min = -vehicle.f32_RearOverhang;
  const float ego_x_max = vehicle.f32_WheelBase + vehicle.f32_FrontOverhang;
  const float ego_y_min = -0.5f * vehicle.f32_Width;
  const float ego_y_max = 0.5f * vehicle.f32_Width;

  const float roll = MainDeg2Rad(vehicle.st_LiDARRotationDeg.f32_X);
  const float pitch = MainDeg2Rad(vehicle.st_LiDARRotationDeg.f32_Y);
  const float yaw = MainDeg2Rad(vehicle.st_LiDARRotationDeg.f32_Z);
  const Eigen::Matrix3f vehicle_from_lidar_rotation =
      (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
       Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
          .toRotationMatrix();
  const Eigen::Vector3f vehicle_from_lidar_translation(
      vehicle.st_LiDARPos.f32_X, vehicle.st_LiDARPos.f32_Y, vehicle.st_LiDARPos.f32_Z);

  // Cleared first so a reused output never accumulates the previous scan.
  output->clear();
  output->points.reserve(input->points.size());

  for (const auto& pcl_Point : input->points) {
    const float point_range = std::sqrt(pcl_Point.x * pcl_Point.x + pcl_Point.y * pcl_Point.y +
                                        pcl_Point.z * pcl_Point.z);
    // Inclusive bounds; NaN/Inf fail every comparison and are dropped.
    const bool inside_roi =
        point_range >= range.f32_Min && point_range <= range.f32_Max &&
        pcl_Point.x >= roi.f32_MinX && pcl_Point.x <= roi.f32_MaxX &&
        pcl_Point.y >= roi.f32_MinY && pcl_Point.y <= roi.f32_MaxY &&
        pcl_Point.z >= roi.f32_MinZ && pcl_Point.z <= roi.f32_MaxZ;

    if (!inside_roi) continue;

    const Eigen::Vector3f point_vehicle =
        vehicle_from_lidar_rotation * Eigen::Vector3f(pcl_Point.x, pcl_Point.y, pcl_Point.z) +
        vehicle_from_lidar_translation;
    const bool inside_vehicle =
        point_vehicle.x() >= ego_x_min && point_vehicle.x() <= ego_x_max &&
        point_vehicle.y() >= ego_y_min && point_vehicle.y() <= ego_y_max;

    if (inside_vehicle) continue;

    output->points.push_back(pcl_Point);
  }
  output->width = static_cast<uint32_t>(output->points.size());
  output->height = 1;
  output->is_dense = false;
}

// Ported from Main src/Lidar/RoiVoxel/RoiVoxel.cpp:65-70 (8f93c22e)
// Main hard-codes 0.1f on all three axes; leaf_m defaults to that value.
void Voxel(const CloudPtr& input, const CloudPtr& output, float leaf_m) {
  pcl::VoxelGrid<pcl::PointXYZI> pcl_Sor;
  pcl_Sor.setInputCloud(input);
  pcl_Sor.setLeafSize(leaf_m, leaf_m, leaf_m);
  pcl_Sor.filter(*output);
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
