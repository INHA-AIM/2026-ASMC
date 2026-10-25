#pragma once
#include <asmc_lidar/tracking/track.hpp>
// One LiDAR scan through the ported common-geometry path:
//   ROI/ego -> Voxel -> roll/pitch leveling -> RANSAC ground -> [experimental
//   tunnel stage, default OFF] -> clustering -> centroid/OBB -> tracking.
// Equivalent to Main TemporalThread (after the local lookup) + LidarProcess at
// 8f93c22e with enable_common_geometry=true. Not included: debug publishers,
// markers, public MotionClassifier output, planning filters, fusion (see FrontCameraFusion).
// A diagnostic-only Main motion decision now runs on copies at full scan rate.

#include <asmc_lidar/detection/clustering.hpp>
#include <asmc_lidar/pipeline/stage_observer.hpp>
#include <asmc_lidar/preprocessing/ground.hpp>
#include <asmc_lidar/preprocessing/preprocessing.hpp>
#include <asmc_lidar/geometry/scan_frame.hpp>
#include <asmc_lidar/tracking/tracker.hpp>
#include <asmc_lidar/experimental/tunnel_filter_stage.hpp>

#include <memory>
#include <string>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

struct PipelineConfig {
  RangeConfig range;
  RoiConfig roi;
  HeightRoiConfig height_roi;
  float voxel_leaf_m = 0.1f;
  // Roll+pitch leveling is the shadow default. false = Main's pitch-only
  // common profile (lidar_level_roll=false).
  bool use_roll = true;
  GroundConfig ground;
  ClusteringConfig clustering;
  TrackingConfig tracking;
  VehicleSpec vehicle;
  std::string calibration_id;  // non-empty
  TunnelFilterStageConfig tunnel;
  // Keep detections/centroids/stage clouds in ScanResult (replay and tests).
  bool capture_debug = false;
  // Monitoring-only observation hook (stage_observer.hpp), non-owning. nullptr =
  // no observation; never changes an algorithm input or output.
  StageObserver* observer = nullptr;
};

struct StageCounts {
  int raw_points = 0;
  int roi_points = 0;
  int voxel_points = 0;
  int leveled_points = 0;
  int height_kept_points = 0, height_removed_points = 0;
  int ground_kept_points = 0;   // Main g_LidarStageProfile.ransac_points
  int tunnel_removed_points = 0;
  int cluster_input_points = 0;  // after the tunnel stage (== ground_kept when OFF)
  int clusters = 0;
  int centroids = 0;  // Main g_LidarStageProfile.cluster_points
  int detections = 0;
  // capture_debug only (0 otherwise): ConsolidateCurrentDetections size,
  // measured on a copy (Main consolidates inside FrameFilter).
  int consolidated_detections = 0;
  int tracks = 0;           // objects.size()
  int observed_tracks = 0;  // objects with s32_MissCount == 0
  int predicted_tracks = 0; // objects with s32_MissCount > 0
};

struct StageTimings {
  double roi_ms = 0, voxel_ms = 0, level_ms = 0, height_ms = 0, ground_ms = 0, tunnel_ms = 0;
  double cluster_ms = 0, detection_ms = 0, tracking_ms = 0, post_ms = 0, total_ms = 0;
};

struct ScanInput {
  asmc::lidar::Observation time;  // stamp_ns = PointCloud2 header stamp
  CloudPtr points;                // sensor frame (velodyne), x/y/z/intensity
  std::uint64_t scan_sequence = 0;
  std::uint64_t generation = 1;  // processing-state epoch, not a per-scan counter
};

// When !processed only stamp_ns, previous_stamp_ns and reason are set (the
// scan context was still Prepare()d, see Pipeline::frame()).
struct ScanResult {
  bool processed = false;
  std::string reason;  // set when !processed: "duplicate_or_regressing_stamp", "empty_scan"
  std::int64_t stamp_ns = 0;
  std::int64_t previous_stamp_ns = 0;
  double dt_s = 0.;
  StageCounts counts;
  StageTimings timings;
  GroundPlaneSnapshot plane;
  asmc::lidar::Context context;  // time, applied leveling, mount, map pose
  // Main st_LiDAR.vec_Objects before fusion: tracks (observed and carried)
  // sorted by f32_Dist.
  std::vector<TrackedObject> objects;
  // capture_debug only.
  std::vector<TrackedObject> detections;  // Objectinfo_for_km output
  CloudPtr centroids;
  std::vector<int> cluster_sizes;  // cluster order
  struct Digests {
    std::string roi, voxel, leveled, ground_kept;  // CloudDigest of each stage output
  } digests;
  TunnelFilterStageReport tunnel;
};

// Lowercase hex SHA-256 over float x, y, z, intensity (16 bytes per point, in
// cloud order). Used to compare stage clouds with the Main baseline exactly.
std::string CloudDigest(const Cloud& cloud);

class Pipeline {
 public:
  explicit Pipeline(const PipelineConfig& config);
  const PipelineConfig& config() const { return config_; }
  // Single processing worker only, between scans. Tracking state is preserved.
  void SetClusteringConfig(const ClusteringConfig& config) { config_.clustering = config; }

  // Called only after the local IMU/CAN lookup succeeded for this scan
  // (Main skips LidarProcess otherwise and the tracker is not advanced).
  // Like Main, an empty scan still consumes its stamp (a later scan with the
  // same stamp is a duplicate). Non-finite roll/pitch make LevelPoints throw
  // std::invalid_argument after the stamp was consumed; the tracker is untouched.
  ScanResult Process(const ScanInput& scan, const LocalState& local,
                     const asmc::lidar::PoseSelection& map_pose);

  Tracker& tracker() { return tracker_; }
  const Tracker& tracker() const { return tracker_; }
  const ScanFrame& frame() const { return frame_; }

 private:
  PipelineConfig config_;
  GroundFilter ground_;
  Tracker tracker_;
  motion_display::MotionClassifier display_motion_;
  ScanFrame frame_;
  std::unique_ptr<TunnelFilterStage> tunnel_;
  std::int64_t last_raw_ns_ = 0;
  std::int64_t previous_stamp_ns_ = 0;  // Main ts_PrevTime (0 before the first scan)
};

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
