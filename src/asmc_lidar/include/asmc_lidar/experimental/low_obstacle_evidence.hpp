#pragma once
#include <asmc_lidar/experimental/tunnel_route.hpp>

namespace asmc::lidar::pipeline {
// Scan-local evidence only: never written into a confirmed camera class or
// used to set object velocity. Indices always refer to the original voxel.
struct LowObstacleHint {std::string source_class;std::vector<int> support;std::uint64_t camera_epoch=0;};
std::vector<LowObstacleHint> LowObstacleEvidence(
    const Cloud& sensor, const tunnel_filter::FrameResult& frame,
    const VehicleSpec& vehicle, std::int64_t stamp,
    const TunnelCameraInput* camera, const TunnelRouteConfig& config);
}
