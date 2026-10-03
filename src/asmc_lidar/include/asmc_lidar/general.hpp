#pragma once
#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>

namespace asmc_lidar {
// Pinned ground-stage entrypoint; no clustering or OBB.
tunnel_filter::FrameResult ProcessGeneralGroundLeveled(
    const pcl::PointCloud<pcl::PointXYZI>& leveled);
// Frame-level portion of the pinned Main normal-road path: pitch leveling,
// at most three plane fits, then 1.5 m Euclidean clusters and L-shape OBB.
tunnel_filter::FrameResult ProcessGeneralReference(
    const pcl::PointCloud<pcl::PointXYZI>& voxel,float pitch_rad,
    const tunnel_filter::ObbFunction& obb_function);
}  // namespace asmc_lidar
