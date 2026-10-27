#pragma once
#include <asmc_lidar/tracking/track.hpp>
// Shadow runtime shared by the ROS node (lidar_pipeline_node) and the bag
// replay (lidar_replay). Both call the same entry points (OnImu,
// OnCan, OnGps, AcceptScan, ProcessScan). They differ in clocks (steady/ROS time
// on the node, a logical arrival clock in the replay) and in scheduling: the
// node waits up to local_wait_timeout_s for IMU/CAN and keeps only the newest
// unprocessed scan (overwritten scans are counted), the replay processes every
// scan in bag order with no wait.
//
// Per scan (Main TemporalThread order, common geometry):
//   local IMU/CAN lookup (bounded wait) -> INPUT_INVALID if not ready
//   -> bounded map pose (GPS) -> pipeline (ROI..tracking) -> front camera fusion
//   -> typed local (base_link) / map arrays.

#include <asmc_lidar/fusion/front_camera_fusion.hpp>
#include <asmc_lidar/filtering/hd_map_audit.hpp>
#include <asmc_lidar/filtering/delivery_preview.hpp>
#include <asmc_lidar/filtering/delivery_filters.hpp>
#include <asmc_lidar/pipeline/pipeline.hpp>
#include <asmc_lidar/geometry/pose_history.hpp>
#include <asmc_lidar/experimental/tunnel_motion.hpp>

#include <asmc_msgs/LidarObjectArray.h>
#include <asmc_msgs/LidarTrackingState.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace asmc {
namespace lidar {
namespace runtime {

struct RuntimeConfig {
  pipeline::PipelineConfig pipeline;
  pipeline::PoseHistoryConfig pose;
  pipeline::GpsIngestConfig gps;
  double local_wait_timeout_s = 0.05;  // Main lidar_pose_wait_timeout_s
  std::size_t max_points = 62500;      // Main RAW_LIDAR_DATA capacity check
  bool enable_camera_fusion = true;
  pipeline::fusion::FusionConfig fusion;
  std::string stamp_provenance = "bridge_receive_ns";
  // Post-fusion off/audit/filter policy. Loaded once before runtime creation; shared
  // read-only across explicit resets, never loaded while holding epoch_mutex.
  std::string hd_map_audit_mode = "off", hd_map_manifest, hd_map_manifest_sha256;
  std::string hd_map_audit_error;
  std::shared_ptr<const filtering::HdMapAudit> hd_map_audit;
  filtering::DeliveryPolicy delivery_preview;
  std::shared_ptr<const filtering::DeliveryFilters> delivery_filters;

  // Paths/identity resolved by LoadRuntimeConfig (for diagnostics).
  std::string config_path, config_sha256, mount_config_path, mount_config_sha256;
  std::string config_id, calibration_id;
};

// Reads config/lidar_pipeline.yaml (schema_version 1) and the LiDAR mount
// file it names, checks the mount against the vehicle block (1e-6, like Main
// frame_boundary::Configure), the GPS age bound (0, 0.30] and that the map
// altitude origin equals the map origin z. Throws std::runtime_error.
// relative paths in the file are resolved against `package_share_dir`.
// Every key is required (no silent struct defaults). Also rejected like Main
// at start-up: camera_class_profile not legacy|mission16_v1, fusion_yolo_policy
// not unique|bounded_reuse. pipeline.calibration_id =
// "shadow:" + sha256(yaml) + ":" + sha256(mount file) (not Main's id string).
// Recompute identity after launch/replay overrides, including camera calibration.
void FinalizeRuntimeIdentity(RuntimeConfig& config, const std::string& camera_calibration_id = "");

RuntimeConfig LoadRuntimeConfig(const std::string& yaml_path, const std::string& package_share_dir);

// Main FrameBoundaryPublisher Publish()/Invalid() semantics, transport-free.
class TypedOutput {
 public:
  struct Arrays {
    bool accepted = false;  // false: zero or non-increasing stamp, nothing to publish
    asmc_msgs::LidarObjectArray local;  // frame base_link
    asmc_msgs::LidarObjectArray map;    // frame map
    std::vector<asmc_msgs::LidarTrackingState> local_tracking, map_tracking;
    // true only when Observed() accepted the scan (Main Publish() past Accept):
    // exactly then Main sends the base_link -> lidar_leveled TF, even if an
    // object later turns the arrays INPUT_INVALID. base_from_level is that TF.
    bool observed = false;
    asmc::lidar::Pose base_from_level = asmc::lidar::Pose::Identity();
  };
  explicit TypedOutput(std::string stamp_provenance = "bridge_receive_ns");
  // Observed scan: only tracks with Id>=0, MissCount==0, non-empty cluster and a
  // non-noise class are exported, transformed by asmc::lidar::transform with the
  // scan Context. map is POSE_UNAVAILABLE (reason = map pose reason) when the map
  // pose is invalid. Any invalid object makes both arrays INPUT_INVALID.
  Arrays Observed(const pipeline::ScanResult& result, const std::vector<pipeline::TrackedObject>& objects);
  Arrays Invalid(std::int64_t stamp_ns, const std::string& reason);

  std::uint64_t processed() const { return processed_; }
  std::uint64_t invalid() const { return invalid_; }
  std::uint64_t stamp_errors() const { return stamp_errors_; }
  const std::string& input_reason() const { return input_reason_; }
  const std::string& map_reason() const { return map_reason_; }

 private:
  bool Accept(std::int64_t stamp_ns);
  std::string provenance_;
  std::int64_t last_stamp_ns_ = 0;
  std::uint64_t processed_ = 0, invalid_ = 0, stamp_errors_ = 0;
  std::string input_reason_ = "waiting_for_input", map_reason_ = "gps_missing";
};

struct ScanTimes {
  std::int64_t receipt_ns = 0;     // arrival of the scan (steady or logical)
  // Start of processing (same clock as receipt). Main re-reads the steady clock
  // after the local wait; the runtime uses this caller value (validity only).
  std::int64_t processing_ns = 0;
  std::int64_t decision_ns = 0;    // fusion decision clock (same clock as receipt)
  std::int64_t ros_now_ns = 0;     // ROS time (camera freshness checks)
  std::int64_t steady_now_ns = 0;  // camera buffer receipt clock
  // Live node: re-reads decision/ros/steady "now" right before fusion, after the
  // local wait and the pipeline (Main decision_time = WallTime::now() at the
  // fusion call). Without it a status heartbeat received meanwhile is newer than
  // the stale steady_now and the receiver gate reports status_stale. The replay
  // leaves it empty and keeps its logical clock.
  std::function<void(ScanTimes&)> refresh_before_fusion;
};

struct ScanOutput {
  bool pose_ready = false;
  bool processed = false;
  std::string reason;  // pose/input reason when not processed
  pipeline::LocalInputSelection selected;
  // When fusion is enabled, result.objects is Main's st_LiDAR.vec_Objects after
  // FusionProcess (camera classes, then FusionLiDAR's AABB/Core rewrite).
  pipeline::ScanResult result;
  pipeline::fusion::FusionReport fusion;
  bool fusion_ran = false;
  TypedOutput::Arrays arrays;
  filtering::AuditReport hd_map_audit;
  double wait_ms = 0., total_ms = 0.;
  double ego_speed_mps = 0.;
};

class LidarRuntime {
 public:
  // camera may be null (LiDAR-only). The runtime never calls camera services.
  LidarRuntime(const RuntimeConfig& config,
                std::shared_ptr<pipeline::fusion::CameraSource> camera,
                const asmc::lidar::camera::Calibration* front_calibration);
  const RuntimeConfig& config() const { return config_; }
  // Single processing worker only, between scans (not a sensor callback).
  void SetTunnelWallConfig(const pipeline::TunnelWallConfig& config,const std::string& config_id) {
    pipeline_.SetTunnelWallConfig(config);
    pipeline::ApplyTunnelWall(config_.pipeline.tunnel_route.params,config);
    config_.config_id=config_id;
  }
  void SetTunnelClusterZConfig(const pipeline::TunnelClusterZConfig& config, const std::string& config_id) {
    pipeline_.SetTunnelClusterZConfig(config);
    config_.pipeline.tunnel_route.cluster_z=config;
    config_.config_id=config_id;
  }

  void OnImu(const pipeline::ImuInput& input, std::int64_t receipt_ns, std::int64_t ros_now_ns);
  void OnCan(const pipeline::CanInput& input, std::int64_t receipt_ns, std::int64_t ros_now_ns);
  void OnGps(const pipeline::GpsInput& input, std::int64_t receipt_ns, std::int64_t ros_now_ns);

  // Main common LiDAR callback gate: rejects a zero stamp, more than
  // max_points points, and a stamp not newer than the last accepted one.
  bool AcceptScan(std::int64_t stamp_ns, std::size_t point_count, std::string* reason);

  ScanOutput ProcessScan(const pipeline::ScanInput& scan, double wait_timeout_s, const ScanTimes& times);

  pipeline::Pipeline& pipeline() { return pipeline_; }
  const pipeline::PoseHistory& pose() const { return pose_; }
  const TypedOutput& output() const { return output_; }
  const pipeline::fusion::FrontCameraFusion* fusion() const { return fusion_.get(); }
  const std::string& delivery_status() const { return delivery_filters_.status(); }
  std::uint64_t rejected_scans() const { return rejected_scans_; }
  std::uint64_t imu_rejected() const { return imu_rejected_; }
  std::uint64_t can_rejected() const { return can_rejected_; }
  std::uint64_t gps_rejected() const { return gps_rejected_; }
  // "accepted" or the GpsIngest rejection reason of the last GPS message.
  std::string last_gps_reason() const {
    std::lock_guard<std::mutex> lock(gps_mutex_);
    return last_gps_reason_;
  }

  // Threading: OnImu/OnCan/OnGps may run on sensor callback threads while one
  // worker calls ProcessScan (PoseHistory has its own lock, GPS sanity is
  // serialized like Main's g_GpsSanityMutex). AcceptScan belongs to the single
  // LiDAR callback thread; ProcessScan/pipeline()/output() to one worker.

 private:
  void Audit(ScanOutput& out) const;
  void FilterDelivery(ScanOutput& out);
  RuntimeConfig config_;
  pipeline::PoseHistory pose_;
  pipeline::GpsIngest gps_;
  pipeline::Pipeline pipeline_;
  pipeline::TunnelMotionHistory tunnel_motion_history_;
  std::mutex tunnel_motion_input_mutex_;  // commit pose acceptance and motion sample together
  std::int64_t tunnel_motion_anchor_stamp_=0;
  float tunnel_motion_anchor_yaw_=0;
  std::shared_ptr<pipeline::fusion::CameraSource> camera_;
  std::unique_ptr<asmc::lidar::camera::Calibration> tunnel_camera_calibration_;
  std::unique_ptr<pipeline::fusion::FrontCameraFusion> fusion_;
  TypedOutput output_;
  filtering::DeliveryFilters delivery_filters_;
  std::int64_t last_accepted_scan_ns_ = 0;
  std::atomic<std::uint64_t> rejected_scans_{0}, imu_rejected_{0}, can_rejected_{0}, gps_rejected_{0};
  mutable std::mutex gps_mutex_;  // guards gps_ and last_gps_reason_
  std::string last_gps_reason_;
};

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
