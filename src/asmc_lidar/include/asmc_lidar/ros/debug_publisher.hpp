#pragma once
// Monitoring-only stage publisher of lidar_pipeline_node (default OFF).
//
// Implements pipeline::StageObserver: for an admitted scan (debug enabled,
// a subscriber present, rate limit passed) it copies a bounded stride sample
// of the raw/ROI/voxel/leveled stage clouds plus per-point ground/cluster
// labels and compact OBB summaries on the pipeline worker thread, and a
// separate thread serializes and publishes them:
//
//   <prefix>/raw, <prefix>/roi, <prefix>/voxel  sensor_msgs/PointCloud2, sensor frame
//   <prefix>/leveled                            sensor_msgs/PointCloud2, leveled frame,
//                                               fields x y z intensity label(uint8) cluster(int32)
//                                               label 0 = clustering input (RANSAC non-ground),
//                                               1 = removed as ground, 2 = removed by the tunnel stage
//   <prefix>/scan                               std_msgs/String JSON (asmc_lidar_debug_scan_v1)
//
// Every message of one scan carries the scan's source stamp so a consumer can
// group them. Latest-wins: a scan admitted while the previous one is still
// being serialized replaces it (counted). Nothing here feeds back into the
// pipeline; the algorithm inputs and outputs are untouched.

#include <asmc_lidar/pipeline/stage_observer.hpp>
#include <asmc_lidar/ros/runtime.hpp>
#include <asmc_lidar/ros/delivery_display_history.hpp>

#include <ros/ros.h>

#include <array>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace asmc {
namespace lidar {
namespace runtime {

struct DebugPublisherConfig {
  bool enabled = false;
  std::string topic_prefix = "debug";  // relative to the caller namespace
  double rate_hz = 5.0;          // admitted scans per second (<= 0: every scan)
  std::size_t max_points = 20000;  // per stage cloud; larger clouds are stride-sampled
  std::string leveled_frame = "lidar_pipeline/lidar_leveled";
};

struct DebugCounters {
  std::uint64_t admitted = 0, published = 0, skipped_disabled = 0, skipped_no_subscriber = 0;
  std::uint64_t skipped_throttle = 0, overwritten = 0;
  double capture_ms_last = 0., capture_ms_max = 0., encode_ms_last = 0., encode_ms_max = 0.;
};

class DebugPublisher : public pipeline::StageObserver {
 public:
  // Topic names derived from the prefix (for the node's output guard).
  static std::vector<std::string> TopicNames(const std::string& prefix);

  DebugPublisher(ros::NodeHandle& nh, const DebugPublisherConfig& config, const RuntimeConfig& runtime);
  ~DebugPublisher() override;
  DebugPublisher(const DebugPublisher&) = delete;
  DebugPublisher& operator=(const DebugPublisher&) = delete;

  const DebugPublisherConfig& config() const { return config_; }

  // Pipeline worker thread, before LidarRuntime::ProcessScan of this scan.
  void SetScanContext(const std::string& sensor_frame, std::uint64_t seq, std::int64_t arrival_ros_ns,
                      const std::string& session="", std::uint64_t generation=1, const std::string& config_id="");
  // StageObserver (pipeline worker thread, inside Pipeline::Process).
  bool Begin(std::int64_t stamp_ns) override;
  void OnScan(const pipeline::StageView& view) override;
  // Pipeline worker thread, after ProcessScan: completes the admitted snapshot
  // with the post-fusion tracks and typed arrays and hands it to the publisher
  // thread. A scan that never reached the pipeline (pose not ready) is
  // published as a stage-less scan notice when admitted.
  void Finish(std::int64_t stamp_ns, const ScanOutput& out);

  DebugCounters counters() const;

  struct StageSample {
    std::vector<float> xyzi;  // 4 floats per sent point
    std::uint32_t actual = 0, sent = 0, stride = 1;
  };
  struct ObjectSummary {
    int idx = 0, id = -1, age = 0, miss = 0, points = 0;
    std::string cls, raw_cls;
    pipeline::CameraClassEvidence camera; // small identifiers, no images or point clouds
    float dist = 0.f, cx = 0.f, cy = 0.f, cz = 0.f, width = 0.f, height = 0.f, zmin = 0.f, zmax = 0.f, yaw = 0.f;
    std::array<pipeline::P3, 8> corners{};
    bool kf_initialized = false;
    float abs_vx = 0.f, abs_vy = 0.f, velocity = 0.f;
    pipeline::motion_classifier::MotionObject motion;
  };
  struct Snapshot {
    std::int64_t stamp_ns = 0, arrival_ns = 0;
    std::uint64_t seq = 0;
    std::string sensor_frame, producer_session_id, config_id;
    std::uint64_t generation=1;
    bool stages = false;  // stage clouds captured (scan reached the pipeline)
    StageSample raw, roi, voxel, leveled;
    std::vector<std::uint8_t> leveled_label;   // per sent leveled point
    std::vector<std::uint8_t> raw_label;
    std::vector<std::uint8_t> leveled_road; // 0 unknown, 1 outside, 2 road boundary, 3 inset road
    std::map<int,DeliveryDisplayHistory::Entry> delivery_history;
    std::vector<std::int32_t> leveled_track;  // exact support ownership, -1 if none/ambiguous
    std::vector<std::int32_t> leveled_cluster;  // per sent leveled point, -1 = none
    pipeline::StageCounts counts;
    pipeline::StageTimings timings;
    pipeline::GroundPlaneSnapshot plane;
    asmc::lidar::Rot leveling = asmc::lidar::Rot::Identity();
    asmc::lidar::Pose base_from_sensor = asmc::lidar::Pose::Identity();
    std::optional<asmc::lidar::Pose> base_from_leveled;
    bool map_valid = false;
    std::string map_reason;
    asmc::lidar::Pose map_from_base = asmc::lidar::Pose::Identity();
    float roll_rad = 0.f, pitch_rad = 0.f, yaw_rad = 0.f, speed_mps = 0.f;
    bool tunnel_ran = false;
    int tunnel_removed = 0;
    std::vector<ObjectSummary> detections, tracks;
    bool pose_ready = false, processed = false;
    std::string reason;
    pipeline::fusion::FusionReport fusion;
    bool fusion_ran = false, camera_active = false;
    std::string camera_reason;
    TypedOutput::Arrays arrays;
    filtering::AuditReport hd_map_audit;
    double capture_ms = 0.;
  };

  // JSON text of a snapshot (asmc_lidar_debug_scan_v1); exposed for tests.
  std::string ScanJson(const Snapshot& s) const;

 private:
  bool Admit(std::int64_t stamp_ns);
  void Enqueue(std::unique_ptr<Snapshot> snapshot);
  void PublishLoop();
  void Publish(const Snapshot& s);

  DebugPublisherConfig config_;
  std::shared_ptr<const filtering::HdMapAudit> road_map_;
  DeliveryDisplayHistory delivery_history_;
  bool use_roll_ = true, tunnel_enabled_ = false;
  pipeline::VehicleSpec vehicle_;  // outline/LiDAR position for the page
  pipeline::RangeConfig range_;
  pipeline::RoiConfig roi_;
  pipeline::HeightRoiConfig height_roi_;
  ros::Publisher raw_pub_, roi_pub_, voxel_pub_, leveled_pub_, scan_pub_;

  // Worker-thread state.
  std::string sensor_frame_;
  std::string producer_session_id_, config_id_;
  std::uint64_t generation_=1;
  std::uint64_t seq_ = 0;
  std::int64_t arrival_ns_ = 0;
  std::int64_t last_admit_steady_ns_ = 0;
  std::unique_ptr<Snapshot> pending_;

  // Hand-over to the publisher thread (latest wins).
  std::mutex slot_mutex_;
  std::condition_variable slot_cv_;
  std::unique_ptr<Snapshot> slot_;
  bool stop_ = false;
  std::thread thread_;

  mutable std::mutex counters_mutex_;
  DebugCounters counters_;
};

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
