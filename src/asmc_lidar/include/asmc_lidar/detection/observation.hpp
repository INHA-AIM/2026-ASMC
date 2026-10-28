#pragma once
#include <asmc_lidar/core/types.hpp>

namespace asmc::lidar::pipeline {
struct CameraPointSupport {
  std::int64_t stamp_ns=0;
  std::uint64_t epoch=0;
  int box_index=-1;
  std::string identity,source_class;
};
struct LowObstacleCue {
  std::int64_t stamp_ns=0;
  std::uint64_t camera_epoch=0;
  std::string source_class;
};
// One scan observation in the leveled LiDAR frame. The cluster is owned here once.
struct DetectionObservation {
  CameraPointSupport camera_point_support;
  LowObstacleCue low_obstacle_cue;
  Cloud pcl_ClusterPoints;
  ObjectGroundEvidence ground_evidence;

  uint16 u16_Idx = 0;
  float32 f32_CoreX = 0.f, f32_CoreY = 0.f, f32_CoreZ = 0.f;
  float32 f32_Dist = 0.f;
  float32 f32_MinX = 0.f, f32_MinY = 0.f, f32_MinZ = 0.f;
  float32 f32_MaxX = 0.f, f32_MaxY = 0.f, f32_MaxZ = 0.f;

  // L-shape OBB. length follows yaw, width is the perpendicular XY
  // extent (not the vertical size). Corners 0-3 at ZMin, 4-7 at ZMax, counter-
  // clockwise from +z; edge 0->1 is parallel to +-yaw.
  float f32_OBB_CenterX = 0.f, f32_OBB_CenterY = 0.f, f32_OBB_CenterZ = 0.f;
  // Former f32_OBB_Width / f32_OBB_Height: both are horizontal extents.
  float obb_length_m = 0.f, obb_width_m = 0.f;
  float f32_OBB_ZMin = 0.f, f32_OBB_ZMax = 0.f;
  float f32_OBB_YawRad = 0.f;
  // Former OBB_Corners_Body: these are leveled sensor coordinates.
  std::array<P3, 8> obb_corners_level{};

};

}  // namespace asmc::lidar::pipeline
