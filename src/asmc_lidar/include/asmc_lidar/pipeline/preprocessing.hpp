#pragma once
// ROI/ego removal, voxel grid and roll/pitch leveling.
// Ported from Main RoiVoxel.cpp and ImuFusion.cpp (common-geometry branch) at 8f93c22e.

#include <asmc_lidar/pipeline/types.hpp>

namespace asmc {
namespace lidar {
namespace pipeline {

// Main LiDARFilterRange / LiDARROI (setting.yaml LiDAR.Range / LiDAR.ROI).
struct RangeConfig {
  float32 f32_Min = 0.5f;
  float32 f32_Max = 100.f;
};
struct RoiConfig {
  float32 f32_MinX = -30.f, f32_MaxX = 50.f;
  float32 f32_MinY = -20.f, f32_MaxY = 20.f;
  float32 f32_MinZ = -2.f, f32_MaxZ = 3.f;
};

// Main Roi(): raw sensor frame. Keeps points inside the 3-D range and the ROI box,
// then removes the ego-vehicle column (rear-axle frame, no z limit).
void Roi(const CloudPtr& input, const CloudPtr& output, const RangeConfig& range,
         const RoiConfig& roi, const VehicleSpec& vehicle);

// Main Voxel(): pcl::VoxelGrid, leaf 0.1 m on all axes (hard-coded in Main).
void Voxel(const CloudPtr& input, const CloudPtr& output, float leaf_m = 0.1f);

// Main CorrectingPoints(), common branch: L = asmc::lidar::leveling_rotation(
// roll, pitch, mount, use_roll, true); p_out = (L * p.cast<double>()).cast<float>(),
// intensity kept. Returns the matrix actually applied (to be stored in the scan
// Context). Throws std::invalid_argument on non-finite angles (roll too, even with
// use_roll=false), like the core; output is untouched then. Like Main only
// output->points is replaced: width/height/is_dense are never written.
asmc::lidar::Rot LevelPoints(const CloudPtr& input, const CloudPtr& output, float32 roll_rad,
                             float32 pitch_rad, const asmc::lidar::Rot& mount_rotation,
                             bool use_roll);

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
