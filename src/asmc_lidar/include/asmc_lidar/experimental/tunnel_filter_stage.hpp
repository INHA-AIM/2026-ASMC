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
#include <array>
#include <map>
#include <string>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

struct TunnelFilterStageConfig {
  bool enabled = false;
  bool suppress_ceiling = true;
  bool suppress_wall = true;
};

struct TunnelWallEvidence {
  int side=0; bool allowed=false,valid=false;
  std::array<float,4> plane{};float span=0;
  std::vector<std::pair<float,float>> intervals;
  std::string reason;
};
struct TunnelFilterStageReport {
  int low_obstacle_verified=0,low_obstacle_rescued=0,low_obstacle_rescued_points=0;
  std::vector<std::pair<std::array<float,7>,std::string>> low_obstacle_decisions;
  std::vector<TunnelWallEvidence> wall_evidence;
  bool original_pipeline = false, protection_available = false;
  int boundary_rescued_clusters=0, boundary_rescued_points=0;
  std::map<int,int> temporal_candidate_kinds,temporal_required_frames;
  bool temporal_enabled=false, temporal_entry_guard=false;
  int temporal_boundary_exempt=0;
  int temporal_hold_frames=0, temporal_candidates=0, temporal_hidden=0, temporal_hidden_predictions=0, temporal_max_run=0, temporal_removed_points=0;
  bool motion_integrated = false, motion_valid = false, motion_reliable = false;
  bool wall_left_reanchored = false, wall_right_reanchored = false;
  float motion_dt_s = 0, motion_dyaw_rad = 0, motion_dx_m = 0, motion_dy_m = 0;
  int motion_steps = 0;
  std::string wall_left_state, wall_right_state;
  std::vector<std::pair<std::array<float,7>,std::string>> residual_decisions;
  int exterior_surface_removed=0;
  int protected_points = 0, objects_removed = 0;
  bool cluster_z_enabled = false;
  float cluster_z_min_sensor_m = -1.30f;
  int cluster_z_removed = 0, cluster_z_removed_points = 0, cluster_z_removed_tracks = 0;
  int wall_ambiguous_skipped = 0;
  int clusters_before_filters = 0, clusters_after_filters = 0, unclustered_points = 0;
  float wall_fit_distance_m = 0, wall_remove_distance_m = 0, wall_track_max_offset_m = 0;
  bool ground_fallback_used = false;
  bool wall_left_removal_allowed = false, wall_right_removal_allowed = false;
  float cluster_tolerance_m = 0;
  bool ran = false;
  int input_points = 0;
  int ceiling_labeled = 0;
  int wall_labeled = 0;
  int ground_labeled = 0;
  int removed_points = 0;  // removed from the common ground output
  bool residual_clusters_reused = false;
  double core_ms = 0.;
  // Diagnostic-only; residual sub-times are included in wall_remove, not additive.
  std::map<std::string,double> timings_ms;
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
