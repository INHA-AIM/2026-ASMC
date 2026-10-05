#pragma once
#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>
#include <array>
#include <vector>

namespace asmc_lidar {
struct GroundPlaneAttempt {
  int attempt=0;
  std::size_t candidate_points=0, inlier_points=0;
  bool valid=false, accepted_ground=false, height_valid=false;
  std::array<double,3> normal{}, centroid{};
  double height_at_origin_m=0;
};
// Pinned ground-stage entrypoint; no clustering or OBB.
tunnel_filter::FrameResult ProcessGeneralGroundLeveled(
    const pcl::PointCloud<pcl::PointXYZI>& leveled,
    std::vector<GroundPlaneAttempt>* review_attempts=nullptr,
    bool validate_sensor_height=false);
// Frame-level portion of the pinned Main normal-road path: pitch leveling,
// at most three plane fits, then 1.5 m Euclidean clusters and L-shape OBB.
tunnel_filter::FrameResult ProcessGeneralReference(
    const pcl::PointCloud<pcl::PointXYZI>& voxel,float pitch_rad,
    const tunnel_filter::ObbFunction& obb_function);
}  // namespace asmc_lidar
