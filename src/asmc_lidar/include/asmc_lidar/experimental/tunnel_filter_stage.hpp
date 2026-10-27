#pragma once
// Experimental connection point for the preserved TunnelFilterCore (PR #102).
// Default OFF. When OFF the stage is never constructed and removes nothing.
// When ON it runs the same call on every scan; GPS state is not an input, and
// the core's IsGpsDenied/ShouldRunExperiment helpers are not used.
// Placement: after the common RANSAC ground removal, before clustering.
// The core labels the sensor-frame voxel cloud; points it labels ceiling/wall
// are removed from the common ground output by voxel index (frame independent).

#include <asmc_lidar/core/types.hpp>

#include <memory>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

struct TunnelFilterStageConfig {
  bool enabled = false;
  bool suppress_ceiling = true;
  bool suppress_wall = true;
};

struct TunnelFilterStageReport {
  bool ran = false;
  int input_points = 0;
  int ceiling_labeled = 0;
  int wall_labeled = 0;
  int ground_labeled = 0;
  int removed_points = 0;  // removed from the common ground output
  double core_ms = 0.;
};

class TunnelFilterStage {
 public:
  explicit TunnelFilterStage(const TunnelFilterStageConfig& config);
  ~TunnelFilterStage();
  TunnelFilterStage(const TunnelFilterStage&) = delete;
  TunnelFilterStage& operator=(const TunnelFilterStage&) = delete;

  // voxel: sensor-frame ROI+voxel cloud (the core's documented input).
  // ground_output/ground_kept_indices: common ground output and, for every
  // output point, its index in `voxel` (leveling preserves order). Both are
  // filtered in place.
  TunnelFilterStageReport Apply(const CloudPtr& voxel, float32 roll_rad, float32 pitch_rad,
                                const CloudPtr& ground_output,
                                std::vector<int>& ground_kept_indices);

 private:
  struct Impl;
  TunnelFilterStageConfig config_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
