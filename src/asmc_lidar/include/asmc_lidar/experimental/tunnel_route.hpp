#pragma once
// Complete 8dde9b69 tunnel processing, adapted to the common local/map boundary.
// No legacy ROS subscriber, publisher, global state or tracker is introduced.
#include <asmc_lidar/experimental/tunnel_filter_core.hpp>
#include <asmc_lidar/experimental/tunnel_filter_stage.hpp>
#include <asmc_lidar/fusion/camera_observations.hpp>
#include <asmc_lidar/tracking/track.hpp>
#include <functional>

namespace asmc::lidar::pipeline {
struct TunnelProtectionConfig {
  bool enabled = true;
  bool multiview = false;
  bool motion_compensated_fallback = false;
  double max_time_offset_s = .05;
  float box_margin_ratio = .10f, box_margin_px = 8.f;
  float depth_vehicle_m = 6.f, depth_person_m = 1.5f;
  float depth_front_margin_m = 1.f, depth_ref_percentile = .10f;
  int min_box_points = 3;
  float ambiguous_max_range_m = 30.f;
};
struct TunnelWallConfig {
  float fit_distance_m = .15f, remove_distance_m = .25f, track_max_offset_m = .6f;
};
void ValidateTunnelWall(const TunnelWallConfig& config);
inline TunnelWallConfig TunnelWallFromParams(const tunnel_filter::Params& p) {
  return {p.wall_fit_distance_m,p.wall_remove_distance_m,p.wall_track_max_offset_m};
}
inline void ApplyTunnelWall(tunnel_filter::Params& p,const TunnelWallConfig& c) {
  ValidateTunnelWall(c);
  p.wall_fit_distance_m=c.fit_distance_m;p.wall_remove_distance_m=c.remove_distance_m;
  p.wall_track_max_offset_m=c.track_max_offset_m;
}
struct TunnelClusterZConfig {
  bool enabled = false;
  float min_sensor_z_m = -1.30f;
};
void ValidateTunnelClusterZ(const TunnelClusterZConfig& config);
struct TunnelRouteConfig {
  // false preserves the old label-only stage for historical replay profiles.
  bool original_pipeline = false;
  bool integrated_motion = false;
  bool boundary_cluster_rescue = false;
  bool low_obstacle_evidence = false;
  tunnel_filter::Params params;
  TunnelProtectionConfig protection;
  TunnelClusterZConfig cluster_z;
  TunnelRouteConfig() {
    // Exact original main.launch overrides; all other Params defaults unchanged.
    params.wall_search_min_abs_y_m = 1.75f;
    params.wall_remove_distance_m = .25f;
  }
};
struct TunnelCameraInput;
struct TunnelCameraView {
  std::string camera_id;
  bool common_projection=false; // M1 uses the canonical optical-axis transform
  bool calibrated = false, available = false;
  camera::Calibration calibration;
  std::vector<camera::Box> boxes;
  std::int64_t stamp_ns=0;
  int image_width=0,image_height=0;
  std::string identity;
  std::function<bool()> still_current;
  std::function<bool()> source_current;
  std::shared_ptr<const TunnelCameraInput> low_obstacle_camera;
  bool motion_compensated=false;
  bool source_active=false;
  std::uint64_t camera_epoch=0;
  asmc::lidar::Pose image_sensor_from_scan_sensor=asmc::lidar::Pose::Identity();
};
struct TunnelCameraInput : TunnelCameraView {
  std::vector<TunnelCameraView> views; // empty preserves the front rollback path
  // Per-view causal, motion-compensated input for measured point refinement.
  // Separate from the synchronous wall-protection views above.
  std::vector<std::shared_ptr<const TunnelCameraInput>> refinement_views;
};
struct TunnelRouteOutput {
  CloudPtr leveled{new Cloud}, filtered{new Cloud}, centroids{new Cloud};
  std::vector<int> kept_indices, pre_filter_indices;
  std::vector<pcl::PointIndices> clusters;  // compact filtered-cloud indices
  std::vector<std::string> protected_classes;
  std::vector<LowObstacleCue> low_obstacle_cues;
  GroundPlaneSnapshot plane;
  asmc::lidar::Rot leveling = asmc::lidar::Rot::Identity();
  TunnelFilterStageReport report;
};
class TunnelRoute {
 public:
  explicit TunnelRoute(const TunnelRouteConfig& config);
  // Called once per accepted scan. Bounded, causal GPS map validity already
  // includes invalid/stale GPS and the existing three-sample recovery latch.
  bool Select(bool map_pose_valid);
  void SetUpdatesAllowed(bool allowed) {
    updates_allowed_=allowed;
    core_.SetScopedUpdates(allowed&&config_.params.sparse_cluster_enable,
                          allowed&&config_.params.wall_geometry_precedence);
  }
  void SetClusterZConfig(const TunnelClusterZConfig& config) {
    ValidateTunnelClusterZ(config); config_.cluster_z = config;
  }
  void SetWallConfig(const TunnelWallConfig& config);
  void RestrictRoi(const CloudPtr& cloud) const;
  // Healthy-GPS entrance ceiling cap. Returned indices refer to the input.
  void EntranceCeiling(const CloudPtr& input, const CloudPtr& output,
                       std::vector<int>& kept_indices) const;
  bool EntranceStructure(const TrackedObject& object) const;
  bool InsideCurrentWalls(const TrackedObject& object,float end_margin_m=3.f) const;
  TunnelRouteOutput Process(const CloudPtr& voxel, const LocalState& local,
                            std::int64_t stamp_ns, const VehicleSpec& vehicle,
                            const TunnelCameraInput* camera,
                            const tunnel_filter::EgoMotion* motion = nullptr,
                            const CloudPtr& measured_vehicle_support = {},float vehicle_match_m=.25f);
  const tunnel_filter::FrameResult& last() const { return core_.LastResult(); }
  const tunnel_filter::TunnelPipeline& wall_core() const { return core_; }
  void SeedWallHistory(const tunnel_filter::TunnelPipeline& other) { core_.CopyWallHistoryFrom(other); }
 private:
  TunnelRouteConfig config_;
  tunnel_filter::TunnelPipeline core_;
  bool updates_allowed_=true;
  bool active_ = false, anchor_valid_ = false;
  std::int64_t anchor_stamp_ = 0;
  float anchor_yaw_ = 0, anchor_speed_ = 0;
};
}  // namespace asmc::lidar::pipeline
