#include <asmc_lidar/tracking/planning_velocity.hpp>
#include <asmc_lidar/detection/detections.hpp>
#include <asmc_lidar/core/source_time.hpp>
#include <asmc_lidar/tracking/track.hpp>
#include <asmc_lidar/pipeline/pipeline.hpp>

#include <openssl/sha.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

using Clock = std::chrono::steady_clock;

// Main LIDAR_STAGE (PipelineProfile.hpp): steady_clock milliseconds.
double Ms(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

void PutFloatLe(float value, unsigned char* out) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  out[0] = static_cast<unsigned char>(bits & 0xffu);
  out[1] = static_cast<unsigned char>((bits >> 8) & 0xffu);
  out[2] = static_cast<unsigned char>((bits >> 16) & 0xffu);
  out[3] = static_cast<unsigned char>((bits >> 24) & 0xffu);
}

}  // namespace

// docs/scan-trace-schema.md "digests": 16 bytes per point (float x, y, z, intensity, little
// endian) in cloud order; padding bytes of PointXYZI are never hashed.
std::string CloudDigest(const Cloud& cloud) {
  SHA256_CTX state;
  SHA256_Init(&state);
  unsigned char buffer[16 * 512];
  std::size_t used = 0;
  for (const auto& point : cloud.points) {
    PutFloatLe(point.x, buffer + used);
    PutFloatLe(point.y, buffer + used + 4);
    PutFloatLe(point.z, buffer + used + 8);
    PutFloatLe(point.intensity, buffer + used + 12);
    used += 16;
    if (used == sizeof(buffer)) {
      SHA256_Update(&state, buffer, used);
      used = 0;
    }
  }
  if (used) SHA256_Update(&state, buffer, used);
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &state);
  static const char kHex[] = "0123456789abcdef";
  std::string hex(2 * SHA256_DIGEST_LENGTH, '0');
  for (int i = 0; i < SHA256_DIGEST_LENGTH; ++i) {
    hex[2 * i] = kHex[digest[i] >> 4];
    hex[2 * i + 1] = kHex[digest[i] & 0x0f];
  }
  return hex;
}

// Ported from Main src/main.cpp:633-636 (InitVehicleSpec + frame_boundary::Configure) (8f93c22e)
Pipeline::Pipeline(const PipelineConfig& config)
    : config_(config), ground_(config.ground), tracker_(config.tracking) {
  if (config_.calibration_id.empty())
    throw std::invalid_argument("pipeline calibration_id must be non-empty");
  frame_.Configure(config_.vehicle, config_.calibration_id);
  // Experimental stage, default OFF: never constructed unless enabled.
  if (config_.tunnel.enabled) tunnel_ = std::make_unique<TunnelFilterStage>(config_.tunnel);
}

// Ported from Main src/main.cpp:492-495 + src/Lidar/LidarProcess/LidarProcess.cpp:151-318 (8f93c22e)
// (PrepareScan before LidarProcess; common-geometry branch only, legacy branches dropped).
ScanResult Pipeline::Process(const ScanInput& scan, const LocalState& local,
                             const asmc::lidar::PoseSelection& map_pose) {
  const auto start = Clock::now();
  ScanResult result;
  result.stamp_ns = scan.time.stamp_ns;
  result.previous_stamp_ns = previous_stamp_ns_;

  // TemporalThread prepares the scan context before LidarProcess, so a scan
  // LidarProcess rejects below still replaces the context (main.cpp:495).
  frame_.Prepare(scan.time, local, map_pose, scan.scan_sequence);

  // Main `static uint64_t last_raw`: equal or older source stamps are skipped
  // before any state changes (LidarProcess.cpp:161-167).
  // Legacy gate (u64_Timestamp ms, equality only) is not ported.
  const std::int64_t raw = scan.time.stamp_ns;
  if (raw <= last_raw_ns_) {
    result.reason = "duplicate_or_regressing_stamp";
    return result;
  }
  last_raw_ns_ = raw;

  // Main advances last_raw before the empty check (LidarProcess.cpp:171-174).
  const std::size_t raw_count = scan.points ? scan.points->points.size() : 0;
  if (raw_count == 0) {
    result.reason = "empty_scan";
    return result;
  }

  StageCounts& counts = result.counts;
  StageTimings& timings = result.timings;
  const bool debug = config_.capture_debug;
  // Monitoring hook (stage_observer.hpp): asked per scan; no work unless it says yes.
  const bool observe = config_.observer && config_.observer->Begin(raw);
  counts.raw_points = static_cast<int>(raw_count);

  CloudPtr roi_cloud(new Cloud);
  CloudPtr voxeled_cloud(new Cloud);
  CloudPtr leveled_cloud(new Cloud);
  CloudPtr ground_cloud(new Cloud);  // Main ransac_cloud
  CloudPtr centroid_cloud(new Cloud);  // Main clustering_cloud
  std::vector<pcl::PointIndices> all_cluster_indices;
  std::vector<TrackedObject> object;

  // MakePointCloudFromRaw is a plain copy of the PointCloud2 points; the input
  // cloud is read directly (Roi does not modify it).
  auto begin = Clock::now();
  Roi(scan.points, roi_cloud, config_.range, config_.roi, config_.vehicle);
  auto end = Clock::now();
  timings.roi_ms = Ms(begin, end);
  counts.roi_points = static_cast<int>(roi_cloud->points.size());
  if (debug) result.digests.roi = CloudDigest(*roi_cloud);

  begin = Clock::now();
  Voxel(roi_cloud, voxeled_cloud, config_.voxel_leaf_m);
  end = Clock::now();
  timings.voxel_ms = Ms(begin, end);
  counts.voxel_points = static_cast<int>(voxeled_cloud->points.size());
  if (debug) result.digests.voxel = CloudDigest(*voxeled_cloud);

  // Main CorrectingPoints common branch (ImuFusion.cpp:55-59): the mount is
  // SensorMount(vehicle).rotation(), the applied matrix goes to the context.
  begin = Clock::now();
  const asmc::lidar::Rot leveling =
      LevelPoints(voxeled_cloud, leveled_cloud, local.st_IMU.f32_Roll_rad,
                  local.st_IMU.f32_Pitch_rad, ScanFrame::SensorMount(config_.vehicle).rotation(),
                  config_.use_roll);
  frame_.StoreLeveling(leveling);
  end = Clock::now();
  timings.level_ms = Ms(begin, end);
  counts.leveled_points = static_cast<int>(leveled_cloud->points.size());
  if (debug) result.digests.leveled = CloudDigest(*leveled_cloud);

  // Cheap gravity-aligned cap before RANSAC/cluster/OBB. The mount's vertical
  // lever is rotated once per scan; no GPS or per-point map projection is used.
  CloudPtr height_cloud=leveled_cloud;
  std::vector<int> height_kept_indices;
  begin=Clock::now();
  if(config_.height_roi.enabled) {
    height_cloud.reset(new Cloud);
    const auto mount=ScanFrame::SensorMount(config_.vehicle);
    const auto upright_from_base=(leveling*mount.rotation().transpose()).eval();
    const double sensor_height=config_.height_roi.nominal_base_height_m+
        (upright_from_base*mount.translation()).z();
    HeightRoi(leveled_cloud,height_cloud,sensor_height,config_.height_roi,height_kept_indices);
  }
  counts.height_kept_points=static_cast<int>(height_cloud->size());
  counts.height_removed_points=counts.leveled_points-counts.height_kept_points;
  timings.height_ms=Ms(begin,Clock::now());

  // Main NormalVectorFiltering (LidarProcess.cpp:211-216). ransac_points is
  // taken here, before Clustering pushes its dummy point.
  std::vector<int> ground_kept_indices;
  begin = Clock::now();
  ground_.Filter(height_cloud, ground_cloud, &ground_kept_indices);
  if(config_.height_roi.enabled)
    for(auto& i:ground_kept_indices)i=height_kept_indices.at(static_cast<std::size_t>(i));
  end = Clock::now();
  timings.ground_ms = Ms(begin, end);
  counts.ground_kept_points = static_cast<int>(ground_cloud->points.size());
  // Main pipeline_trace::Ground (trace only) is replaced by the stage digests.
  if (debug) result.digests.ground_kept = CloudDigest(*ground_cloud);

  // Observer copy of the pre-tunnel indices (the stage filters them in place).
  std::vector<int> pre_tunnel_kept_indices;
  if (observe && tunnel_) pre_tunnel_kept_indices = ground_kept_indices;

  // Experimental tunnel stage (not in Main): after RANSAC, before clustering.
  // It levels with the same roll choice as the common path (roll 0 when pitch-only).
  if (tunnel_) {
    begin = Clock::now();
    result.tunnel = tunnel_->Apply(voxeled_cloud, config_.use_roll ? local.st_IMU.f32_Roll_rad : 0.f,
                                   local.st_IMU.f32_Pitch_rad, ground_cloud, ground_kept_indices);
    end = Clock::now();
    timings.tunnel_ms = Ms(begin, end);
    counts.tunnel_removed_points = result.tunnel.removed_points;
  }
  counts.cluster_input_points = static_cast<int>(ground_cloud->points.size());

  // Main Clustering (LidarProcess.cpp:242-247). It mutates ground_cloud.
  begin = Clock::now();
  Clustering(ground_cloud, centroid_cloud, all_cluster_indices, config_.clustering);
  end = Clock::now();
  timings.cluster_ms = Ms(begin, end);
  counts.centroids = static_cast<int>(centroid_cloud->points.size());
  counts.clusters = static_cast<int>(all_cluster_indices.size());
  // Main debug cloud publishers (/lidar/debug_*, debug_clustered_map) are not ported.

  // Main Objectinfo_for_km (LidarProcess.cpp:257-259) on the mutated cloud.
  begin = Clock::now();
  BuildDetections(object, ground_cloud, centroid_cloud, all_cluster_indices);
  end = Clock::now();
  timings.detection_ms = Ms(begin, end);
  counts.detections = static_cast<int>(object.size());
  if (debug) {
    result.detections = object;
    result.centroids = centroid_cloud;
    result.cluster_sizes.reserve(all_cluster_indices.size());
    for (const auto& cluster : all_cluster_indices)
      result.cluster_sizes.push_back(static_cast<int>(cluster.indices.size()));
    // Main consolidates inside FrameFilter; the count is measured on a copy.
    std::vector<TrackedObject> consolidated = object;
    ConsolidateCurrentDetections(consolidated, config_.tracking);
    counts.consolidated_detections = static_cast<int>(consolidated.size());
  }
  // Observer copy of the pre-tracking detections (Tracker::Update rewrites `object`).
  std::vector<TrackedObject> observed_detections;
  if (observe) observed_detections = object;

  // Main FrameFilter (LidarProcess.cpp:260-261); f64_Dt = (ts_CurrTime -
  // ts_PrevTime).toSec() with ts_PrevTime = ros::Time(0,0) before the first scan.
  const double dt_s = SourceDeltaSeconds(raw, previous_stamp_ns_);
  begin = Clock::now();
  tracker_.Update(object, dt_s, local, frame_);
  end = Clock::now();
  timings.tracking_ms = Ms(begin, end);
  result.dt_s = dt_s;
  previous_stamp_ns_ = raw;  // Main ts_PrevTime = ts_CurrTime (LidarProcess.cpp:269)
  result.context = frame_.Current();  // Main st_LiDAR.frame_context (LidarProcess.cpp:272)

  // Main decision history runs at full scan rate, never at the throttled web
  // refresh rate. Only a diagnostic copy is modified; tracker/KF and typed
  // velocity/dynamic validity remain untouched.
  std::vector<motion_display::MotionObject> motion_objects(object.size());
  const auto base_from_level = asmc::lidar::base_from_input(result.context);
  const bool motion_map_valid = result.context.map_pose.valid && base_from_level.has_value();
  if (motion_map_valid) {
    const auto map_from_level = result.context.map_pose.map_from_base * *base_from_level;
    for (std::size_t i=0;i<object.size();++i) {
      const auto& o=object[i];auto& m=motion_objects[i];m=o.tracking.display_motion;
      m.s32_Id=o.tracking.s32_Id;m.s32_MissCount=o.tracking.s32_MissCount;
      const auto center=(map_from_level * asmc::lidar::Vec(o.detection.f32_CoreX,o.detection.f32_CoreY,o.detection.f32_CoreZ)).eval();
      m.f32_CoreE=static_cast<float>(center.x());m.f32_CoreN=static_cast<float>(center.y());
      // Classifier and optional planning companion use one compensation rule.
      // CAN translation belongs to base axes, not the tilted sensor axes.
      const auto velocity=PlanningVelocity(o, result);
      m.map_vx=velocity ? velocity->x() : NAN;
      m.map_vy=velocity ? velocity->y() : NAN;
      m.f32_AbsVx=m.map_vx; m.f32_AbsVy=m.map_vy;
      m.min_e=m.min_n=std::numeric_limits<double>::infinity();
      m.max_e=m.max_n=-std::numeric_limits<double>::infinity();
      m.bounds_valid=true;
      for(const auto& corner:o.detection.obb_corners_level) {
        const auto p=(map_from_level * asmc::lidar::Vec(corner.x,corner.y,corner.z)).eval();
        if(!p.allFinite()){m.bounds_valid=false;break;}
        m.min_e=std::min(m.min_e,p.x());m.max_e=std::max(m.max_e,p.x());
        m.min_n=std::min(m.min_n,p.y());m.max_n=std::max(m.max_n,p.y());
      }
      m.bounds_valid=m.bounds_valid && m.max_e-m.min_e>.05 && m.max_n-m.min_n>.05;
    }
    display_motion_.Update(motion_objects, SourceDeltaSeconds(raw, 0));
  } else display_motion_.InvalidateMapHistory(motion_objects);
  for(std::size_t i=0;i<object.size();++i)object[i].tracking.display_motion=motion_objects[i];

  begin = Clock::now();
  std::sort(object.begin(), object.end(),
            [](const TrackedObject& a, const TrackedObject& b) { return a.detection.f32_Dist < b.detection.f32_Dist; });
  // Main LidarProcess.cpp:297-306.
  const GroundPlaneSnapshot ground_plane = ground_.CurrentPlane();
  for (auto& tracked_object : object) {
    if (tracked_object.tracking.s32_MissCount == 0) frame_.RecordTrackingFrame(tracked_object);
    tracked_object.detection.ground_evidence = ObjectGroundEvidence{};
    if (tracked_object.tracking.s32_MissCount == 0) {
      tracked_object.detection.ground_evidence =
          MeasureObjectGroundEvidence(tracked_object.detection.pcl_ClusterPoints.points, ground_plane);
    }
  }
  // Main LidarProcess.cpp:307-313: vec_Objects and the 3-frame queue.
  tracker_.Commit(object);
  // Main /lidar_tracking_markers (LidarProcess.cpp:320-344) are not ported.
  end = Clock::now();
  timings.post_ms = Ms(begin, end);

  result.plane = ground_plane;
  counts.tracks = static_cast<int>(object.size());
  for (const auto& tracked_object : object) {
    if (tracked_object.tracking.s32_MissCount == 0)
      ++counts.observed_tracks;
    else
      ++counts.predicted_tracks;
  }
  result.objects = std::move(object);
  result.processed = true;
  timings.total_ms = Ms(start, Clock::now());
  if (observe) {
    // Read-only views of this scan's stage results; the observer copies what it needs.
    const StageView view{scan,
                         local,
                         result,
                         *roi_cloud,
                         *voxeled_cloud,
                         *leveled_cloud,
                         height_kept_indices,
                         *ground_cloud,
                         ground_kept_indices,
                         tunnel_ ? pre_tunnel_kept_indices : ground_kept_indices,
                         all_cluster_indices,
                         observed_detections};
    config_.observer->OnScan(view);
  }
  return result;
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
