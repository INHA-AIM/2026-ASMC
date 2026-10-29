#include <asmc_lidar/experimental/confirmation_wall_evidence.hpp>
#include <asmc_lidar/tracking/planning_velocity.hpp>
#include <asmc_lidar/fusion/fusion_policy.hpp>
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
#include <set>
#include <tuple>

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
    : config_(config), ground_(config.ground), tracker_(config.tracking), motion_classifier_(config.motion) {
  if (config_.calibration_id.empty())
    throw std::invalid_argument("pipeline calibration_id must be non-empty");
  frame_.Configure(config_.vehicle, config_.calibration_id);
  ValidateTunnelNoise(config_.tunnel_noise);
  ValidateCameraPointRefinement(config_.camera_point_refinement);
  ValidateShadowWall(config_.shadow_wall);
  if(config_.shadow_wall.enabled) {
    auto params=config_.tunnel_route.params;
    // Use the same explicitly configured wall-removal band on approach and
    // inside the shadow zone; ground and wall-fitting tolerances stay separate.
    params.residual_wall_surface_enable=false;
    entrance_wall_core_=std::make_unique<tunnel_filter::TunnelPipeline>(params);
  }
  // Construct only when enabled; shipped YAML enables GPS-denied routing.
  if (config_.tunnel.enabled) {
    if (config_.tunnel_route.original_pipeline) tunnel_route_ = std::make_unique<TunnelRoute>(config_.tunnel_route);
    else tunnel_ = std::make_unique<TunnelFilterStage>(config_.tunnel);
  }
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
  const auto wall_gate=ShadowWallGate(config_.shadow_wall,map_pose,entrance_wall_active_);
  if(wall_gate.active&&previous_original_tunnel_&&entrance_wall_core_&&tunnel_route_)
    entrance_wall_core_->CopyWallHistoryFrom(tunnel_route_->wall_core());
  const bool seed_entry=!map_pose.valid&&entrance_wall_active_&&entrance_wall_core_&&tunnel_route_;
  const bool original_tunnel = tunnel_route_ && tunnel_route_->Select(map_pose.valid);
  if(seed_entry&&original_tunnel)tunnel_route_->SeedWallHistory(*entrance_wall_core_);
  if(entrance_wall_active_&&!wall_gate.active&&entrance_wall_core_)entrance_wall_core_->Reset();
  entrance_wall_active_=wall_gate.active;
  previous_original_tunnel_=original_tunnel;
  Roi(scan.points, roi_cloud, config_.range, config_.roi, config_.vehicle);
  if (original_tunnel) tunnel_route_->RestrictRoi(roi_cloud);
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

  std::vector<int> height_kept_indices, ground_kept_indices, pre_tunnel_kept_indices;
  std::vector<std::string> tunnel_classes;
  std::vector<LowObstacleCue> low_obstacle_cues;
  GroundPlaneSnapshot effective_ground_plane;
  std::vector<MeasuredVehicleHistory> predicted_vehicles;
  CloudPtr vehicle_support_sensor(new Cloud);
  if(config_.camera_point_refinement.enabled&&config_.camera_point_refinement.vehicle_history_enabled&&scan.current_sensor_from_past_sensor) {
    for(const auto& entry:vehicle_history_) {
      const auto& old=entry.second;
      const auto* previous_tracks=tracker_.LastFrame();
      if(!previous_tracks||std::none_of(previous_tracks->begin(),previous_tracks->end(),[&](const auto& o){
          return o.tracking.s32_Id==old.track_id&&o.tracking.s32_MissCount==0&&o.camera.str_ObjectClass=="vehicle";}))continue;
      if(old.confirmations<5||!old.points||raw<=old.stamp_ns||raw-old.stamp_ns>200000000LL||
          raw-old.camera_stamp_ns>static_cast<std::int64_t>(config_.camera_point_refinement.vehicle_history_max_camera_age_s*1e9))continue;
      const auto transform=scan.current_sensor_from_past_sensor(old.stamp_ns);if(!transform||!pose_valid(*transform))continue;
      auto predicted=old;predicted.points.reset(new Cloud);
      for(const auto& p:*old.points) {
        const Vec q=(*transform)*Vec(p.x,p.y,p.z);auto v=p;v.x=q.x();v.y=q.y();v.z=q.z();
        predicted.points->push_back(v);vehicle_support_sensor->push_back(v);
      }
      predicted_vehicles.push_back(std::move(predicted));
    }
  }
  if(original_tunnel) {
    begin=Clock::now();
    auto migrated=tunnel_route_->Process(voxeled_cloud,local,raw,config_.vehicle,scan.tunnel_camera.get(),scan.tunnel_motion.get(),vehicle_support_sensor,config_.camera_point_refinement.vehicle_history_match_m);
    leveled_cloud=migrated.leveled;
    frame_.StoreLeveling(migrated.leveling);
    ground_cloud=migrated.filtered;
    ground_kept_indices=std::move(migrated.kept_indices);
    pre_tunnel_kept_indices=std::move(migrated.pre_filter_indices);
    centroid_cloud=migrated.centroids;
    all_cluster_indices=std::move(migrated.clusters);
    tunnel_classes=std::move(migrated.protected_classes);
    low_obstacle_cues=std::move(migrated.low_obstacle_cues);
    effective_ground_plane=migrated.plane;
    result.tunnel=migrated.report;
    timings.tunnel_ms=Ms(begin,Clock::now());
    // Height ROI is deliberately bypassed by the original tunnel route; keep
    // observer voxel identities complete instead of labelling them all cropped.
    if(config_.height_roi.enabled)for(std::size_t i=0;i<voxeled_cloud->size();++i)height_kept_indices.push_back(i);
    counts.height_kept_points=counts.leveled_points;
    counts.ground_kept_points=static_cast<int>(ground_cloud->size());
    counts.tunnel_removed_points=result.tunnel.removed_points;
    counts.cluster_input_points=static_cast<int>(ground_cloud->size());
    counts.centroids=static_cast<int>(centroid_cloud->size());
    counts.clusters=static_cast<int>(all_cluster_indices.size());
    if(debug) {result.digests.leveled=CloudDigest(*leveled_cloud);result.digests.ground_kept=CloudDigest(*ground_cloud);}
  } else {
  // Cheap gravity-aligned cap before RANSAC/cluster/OBB. The mount's vertical
  // lever is rotated once per scan; no GPS or per-point map projection is used.
  CloudPtr height_cloud=leveled_cloud;
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
  if (observe && tunnel_) pre_tunnel_kept_indices = ground_kept_indices;

  // Experimental tunnel stage (not in Main): after RANSAC, before clustering.
  // It levels with the same roll choice as the common path (roll 0 when pitch-only).
  if (tunnel_ && !map_pose.valid) {
    begin = Clock::now();
    result.tunnel = tunnel_->Apply(voxeled_cloud, config_.use_roll ? local.st_IMU.f32_Roll_rad : 0.f,
                                   local.st_IMU.f32_Pitch_rad, ground_cloud, ground_kept_indices);
    end = Clock::now();
    timings.tunnel_ms = Ms(begin, end);
    counts.tunnel_removed_points = result.tunnel.removed_points;
  }
  if(wall_gate.active&&entrance_wall_core_) {
    const auto wall_begin=Clock::now();const auto& plane=ground_.CurrentPlane();
    tunnel_filter::Plane p;p.a=plane.coefficients[0];p.b=plane.coefficients[1];
    p.c=plane.coefficients[2];p.d=plane.coefficients[3];
    const auto& walls=entrance_wall_core_->ProcessWallsOnly(*leveled_cloud,ground_kept_indices,
        plane.valid,p,scan.tunnel_motion.get());
    CloudPtr filtered(new Cloud);std::vector<int> kept;
    for(int i:ground_kept_indices)if(walls.labels.at(i)==tunnel_filter::kKept) {
      filtered->push_back(leveled_cloud->at(i));kept.push_back(i);
    }
    ground_cloud=filtered;ground_kept_indices=std::move(kept);
    auto& report=result.tunnel;report.ran=true;report.pre_entry_wall=true;
    report.wall_labeled=report.removed_points=walls.wall_removed;
    report.input_points=walls.input_points;report.ground_fallback_used=walls.ground_fallback_used;
    report.wall_left_removal_allowed=walls.walls[0].removal_allowed;
    report.wall_right_removal_allowed=walls.walls[1].removal_allowed;
    for(std::size_t side=0;side<2;++side)for(const auto& seg:walls.walls[side].segments) {
      TunnelWallEvidence e;e.side=side;e.valid=seg.valid;e.allowed=walls.walls[side].removal_allowed;
      e.plane={seg.plane.a,seg.plane.b,seg.plane.c,seg.plane.d};e.intervals=seg.intervals;
      e.span=seg.span;e.reason=seg.reject_reason;report.wall_evidence.push_back(std::move(e));
    }
    report.core_ms=Ms(wall_begin,Clock::now());timings.tunnel_ms+=report.core_ms;
    counts.tunnel_removed_points=report.removed_points;
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

    effective_ground_plane=ground_.CurrentPlane();
  }

  result.tunnel.wall_gate_reason=original_tunnel?"gps_denied_tunnel":wall_gate.reason;
  result.tunnel.route_station_m=wall_gate.station_m;
  result.tunnel.entry_distance_m=wall_gate.distance_to_entry_m;
  result.tunnel.route_lateral_m=wall_gate.lateral_m;
  // Refine CURRENT-scan measurements before OBB fitting or tracker association.
  // Keep unclassified residuals; delete only confirmed wall support belonging
  // to a cluster from which an object was successfully extracted.
  auto refinement=RefineCameraPoints(ground_cloud,all_cluster_indices,frame_.Current(),
      scan.tunnel_camera.get(),result.tunnel.wall_evidence,config_.camera_point_refinement);
  // A brief missing camera result must not repeatedly split the measured
  // object and reset fragment acquisition. Continue ONLY current LiDAR pairs
  // matching the previous measured support; never republish predicted geometry.
  const double steel_dt=SourceDeltaSeconds(raw,previous_stamp_ns_);
  if(config_.camera_point_refinement.steel_fragment_merge&&steel_dt>0&&steel_dt<=.2&&
      std::isfinite(local.st_CAN.f32_CurrentVel_MPS)&&std::abs(local.st_CAN.f32_CurrentVel_MPS)<=.1f) {
    const auto base=base_from_input(frame_.Current());
    if(base) {
      const Vec travel=base->linear().transpose()*frame_.Current().ego_linear_base*steel_dt;
      if(travel.norm()<=.8) {
        std::vector<SteelFragmentHistory> history;
        if(const auto* previous_frame=tracker_.LastFrame())for(const auto& previous:*previous_frame) {
          const auto& support=previous.detection.camera_point_support;
          if(previous.tracking.s32_MissCount!=0||support.source_class!="steel_barricade"||
              support.stamp_ns<=0||raw-support.stamp_ns>750000000LL||
              previous.detection.pcl_ClusterPoints.size()<6||previous.detection.pcl_ClusterPoints.size()>96)continue;
          if(history.size()>=8){history.clear();break;}
          const auto transform=frame_.PreviousTrackingTransform(previous,local.st_IMU.f32_Yaw_rad);
          if(!transform)continue;
          SteelFragmentHistory h;h.support=support;h.points.reset(new Cloud);
          for(const auto& p:previous.detection.pcl_ClusterPoints){const Vec q=(*transform)*Vec(p.x,p.y,p.z)-travel;pcl::PointXYZI v=p;v.x=q.x();v.y=q.y();v.z=q.z();h.points->push_back(v);}
          history.push_back(std::move(h));
        }
        const bool had_refinement=refinement.report.applied;
        auto continued=ContinueMeasuredSteelFragments(ground_cloud,had_refinement?refinement.clusters:all_cluster_indices,history,
            frame_.Current(),scan.tunnel_camera.get(),result.tunnel.wall_evidence,config_.camera_point_refinement,
            had_refinement?&refinement.support:nullptr);
        if(continued.report.applied) {
          if(had_refinement) {
            for(std::size_t i=0;i<continued.parent_clusters.size();++i) {
              const int p=continued.parent_clusters[i];
              if(p>=0){continued.support[i]=refinement.support[p];continued.parent_clusters[i]=refinement.parent_clusters[p];}
            }
            continued.removed_wall_indices=std::move(refinement.removed_wall_indices);
            refinement.report.decisions.insert(refinement.report.decisions.end(),continued.report.decisions.begin(),continued.report.decisions.end());
            refinement.report.extracted_objects+=continued.report.extracted_objects;
            refinement.report.ms+=continued.report.ms;refinement.report.reason="applied_with_steel_continuation";
            continued.report=std::move(refinement.report);
          }
          refinement=std::move(continued);
        }
      }
    }
  }
  std::vector<int> measured_vehicle_ids;
  if(!predicted_vehicles.empty()) {
    const auto base=base_from_input(frame_.Current());
    if(base) {
      const Pose input_from_sensor=base->inverse()*frame_.Current().base_from_sensor;
      for(auto& old:predicted_vehicles)for(auto& p:*old.points) {
        const Vec q=input_from_sensor*Vec(p.x,p.y,p.z);p.x=q.x();p.y=q.y();p.z=q.z();
      }
      const bool had=refinement.report.applied;
      auto continued=ContinueMeasuredVehicles(ground_cloud,had?refinement.clusters:all_cluster_indices,
          predicted_vehicles,result.tunnel.wall_evidence,config_.camera_point_refinement,
          had?&refinement.support:nullptr,&refinement.wall_veto_indices,&measured_vehicle_ids);
      if(continued.report.applied) {
        if(had)for(std::size_t i=0;i<continued.parent_clusters.size();++i) {
          const int p=continued.parent_clusters[i];
          if(p>=0){continued.support[i]=refinement.support[p];continued.parent_clusters[i]=refinement.parent_clusters[p];}
        }
        continued.removed_wall_indices.insert(continued.removed_wall_indices.end(),refinement.removed_wall_indices.begin(),refinement.removed_wall_indices.end());
        refinement.report.decisions.insert(refinement.report.decisions.end(),continued.report.decisions.begin(),continued.report.decisions.end());
        refinement.report.extracted_objects+=continued.report.extracted_objects;refinement.report.ms+=continued.report.ms;
        refinement.report.removed_wall_points=continued.removed_wall_indices.size();refinement.report.applied=true;
        refinement.report.reason="applied_with_vehicle_history";continued.report=std::move(refinement.report);
        refinement=std::move(continued);
      } else {
        refinement.report.decisions.insert(refinement.report.decisions.end(),continued.report.decisions.begin(),continued.report.decisions.end());
        measured_vehicle_ids.clear();
      }
    }
  }
  result.camera_point_refinement=refinement.report;
  std::vector<CameraPointSupport> camera_support;
  if(refinement.report.applied) {
    std::vector<std::string> classes;std::vector<LowObstacleCue> cues;
    for(std::size_t i=0;i<refinement.parent_clusters.size();++i) {
      const int parent=refinement.parent_clusters[i];const auto& support=refinement.support[i];
      const bool extracted=support.box_index>=0;
      classes.push_back(!extracted&&parent>=0&&static_cast<std::size_t>(parent)<tunnel_classes.size()?tunnel_classes[parent]:"unknown");
      LowObstacleCue cue=parent>=0&&static_cast<std::size_t>(parent)<low_obstacle_cues.size()?low_obstacle_cues[parent]:LowObstacleCue{};
      if(extracted&&!cue.source_class.empty()&&cue.source_class!=support.source_class)cue=LowObstacleCue{};
      cues.push_back(std::move(cue));
    }
    tunnel_classes=std::move(classes);low_obstacle_cues=std::move(cues);
    all_cluster_indices=std::move(refinement.clusters);camera_support=std::move(refinement.support);
    if(!refinement.removed_wall_indices.empty()) {
      std::vector<bool> removed(ground_cloud->size(),false);
      for(int i:refinement.removed_wall_indices)removed.at(i)=true;
      std::vector<int> remap(ground_cloud->size(),-1),kept;
      CloudPtr compact(new Cloud);compact->header=ground_cloud->header;
      for(std::size_t i=0;i<ground_cloud->size();++i)if(!removed[i]) {
        remap[i]=compact->size();compact->push_back(ground_cloud->at(i));
        // The common clusterer appends one sentinel without a voxel identity.
        if(i<ground_kept_indices.size())kept.push_back(ground_kept_indices[i]);
      }
      for(auto& cluster:all_cluster_indices)for(auto& i:cluster.indices)i=remap.at(i);
      ground_cloud=compact;ground_kept_indices=std::move(kept);
      counts.tunnel_removed_points+=refinement.removed_wall_indices.size();
      counts.cluster_input_points=ground_cloud->size();
      if(debug)result.digests.ground_kept=CloudDigest(*ground_cloud);
    }
    centroid_cloud.reset(new Cloud);
    for(const auto& cluster:all_cluster_indices) {
      pcl::PointXYZI center{};
      for(int i:cluster.indices){const auto& p=ground_cloud->at(i);center.x+=p.x;center.y+=p.y;center.z+=p.z;}
      const float size=cluster.indices.size();center.x/=size;center.y/=size;center.z/=size;
      centroid_cloud->push_back(center);
    }
    counts.clusters=all_cluster_indices.size();counts.centroids=centroid_cloud->size();
  }

  // Main Objectinfo_for_km (LidarProcess.cpp:257-259) on the mutated cloud.
  begin = Clock::now();
  BuildDetections(object, ground_cloud, centroid_cloud, all_cluster_indices);
  end = Clock::now();
  timings.detection_ms = Ms(begin, end);
  for(std::size_t i=0;i<object.size()&&i<camera_support.size();++i)
    object[i].detection.camera_point_support=std::move(camera_support[i]);
  for(std::size_t i=0;i<object.size()&&i<measured_vehicle_ids.size();++i)
    object[i].detection.measured_vehicle_track_id=measured_vehicle_ids[i];
  if(original_tunnel) {
    for(std::size_t i=0;i<object.size()&&i<low_obstacle_cues.size();++i)object[i].detection.low_obstacle_cue=low_obstacle_cues[i];
    for(std::size_t i=0;i<object.size()&&i<tunnel_classes.size();++i)if(tunnel_classes[i]!="unknown") {
      if(!config_.tunnel_route.protection.multiview) {
        object[i].camera.str_ObjectClass=tunnel_classes[i];object[i].camera.str_YoloClass=tunnel_classes[i];
      } // M1 preprocessing protection is not a confirmed detector observation.
    }
  }
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
  if(config_.camera_point_refinement.enabled&&config_.camera_point_refinement.vehicle_history_enabled) {
    std::map<int,MeasuredVehicleHistory> next;
    const auto base=base_from_input(frame_.Current());
    if(base)for(const auto& o:object) {
      if(o.tracking.s32_MissCount!=0||o.tracking.s32_Age<20||o.detection.pcl_ClusterPoints.size()<10||o.detection.pcl_ClusterPoints.size()>2048||next.size()>=32)continue;
      const int id=o.tracking.s32_Id;const auto previous=vehicle_history_.find(id);
      MeasuredVehicleHistory h;if(previous!=vehicle_history_.end())h=previous->second;h.track_id=id;
      const auto& cs=o.detection.camera_point_support;
      const bool fresh=cs.box_index>=0&&(cs.source_class=="car"||cs.source_class=="truck"||cs.source_class=="bus")&&
        cs.stamp_ns>0&&raw>=cs.stamp_ns&&raw-cs.stamp_ns<=150000000LL&&o.camera.str_ObjectClass=="vehicle";
      const bool continued=o.detection.measured_vehicle_track_id==id&&h.confirmations>=5&&
        raw-h.camera_stamp_ns<=static_cast<std::int64_t>(config_.camera_point_refinement.vehicle_history_max_camera_age_s*1e9);
      if(!fresh&&!continued)continue;
      if(fresh&&cs.identity!=h.camera_identity) {++h.confirmations;h.camera_stamp_ns=cs.stamp_ns;h.camera_identity=cs.identity;}
      h.stamp_ns=raw;h.points.reset(new Cloud);
      const Pose sensor_from_input=frame_.Current().base_from_sensor.inverse()* *base;
      for(const auto& p:o.detection.pcl_ClusterPoints) {const Vec q=sensor_from_input*Vec(p.x,p.y,p.z);auto v=p;v.x=q.x();v.y=q.y();v.z=q.z();h.points->push_back(v);}
      next[id]=std::move(h);
    }
    vehicle_history_=std::move(next);
  }
  end = Clock::now();
  timings.tracking_ms = Ms(begin, end);
  result.dt_s = dt_s;
  previous_stamp_ns_ = raw;  // Main ts_PrevTime = ts_CurrTime (LidarProcess.cpp:269)
  result.context = frame_.Current();  // Main st_LiDAR.frame_context (LidarProcess.cpp:272)

  // Main decision history runs at full scan rate, never at the throttled web
  // refresh rate. The copy supplies public applied velocity/motion, while the
  // raw tracker/KF remains unchanged. Map loss invalidates this estimate.
  std::vector<motion_classifier::MotionObject> motion_objects(object.size());
  const auto base_from_level = asmc::lidar::base_from_input(result.context);
  const bool motion_map_valid = result.context.map_pose.valid && base_from_level.has_value();
  if (motion_map_valid) {
    const auto map_from_level = result.context.map_pose.map_from_base * *base_from_level;
    for (std::size_t i=0;i<object.size();++i) {
      const auto& o=object[i];auto& m=motion_objects[i];m=o.tracking.motion;
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
    if(!config_.motion.semantic_static_enabled)motion_classifier_.Update(motion_objects, SourceDeltaSeconds(raw, 0));
  } else if(!config_.motion.semantic_static_enabled)motion_classifier_.InvalidateMapHistory(motion_objects);
  if(config_.motion.semantic_static_enabled) {
    // Capture before RecordTrackingFrame/UpdateAABB, exactly like the baseline.
    motion_inputs_=motion_objects;motion_map_valid_=motion_map_valid;motion_input_stamp_=raw;
  }
  for(std::size_t i=0;i<object.size();++i)object[i].tracking.motion=motion_objects[i];

  begin = Clock::now();
  std::sort(object.begin(), object.end(),
            [](const TrackedObject& a, const TrackedObject& b) { return a.detection.f32_Dist < b.detection.f32_Dist; });
  // Main LidarProcess.cpp:297-306.
  const GroundPlaneSnapshot ground_plane = effective_ground_plane;
  for (auto& tracked_object : object) {
    if (tracked_object.tracking.s32_MissCount == 0) frame_.RecordTrackingFrame(tracked_object);
    tracked_object.detection.ground_evidence = ObjectGroundEvidence{};
    if (tracked_object.tracking.s32_MissCount == 0) {
      tracked_object.detection.ground_evidence =
          MeasureObjectGroundEvidence(tracked_object.detection.pcl_ClusterPoints.points, ground_plane);
    }
  }
  // Main LidarProcess.cpp:307-313: vec_Objects and the 3-frame queue.
  // A threshold edit must also retire carried low tracks, not leave old noise
  // predicted on screen or in the tracker history after its detections vanish.
  if(original_tunnel && config_.tunnel_route.cluster_z.enabled) {
    const auto sensor_from_level=result.context.applied_leveling.transpose().eval();
    object.erase(std::remove_if(object.begin(),object.end(),[&](const auto& o) {
      const auto z=(sensor_from_level*asmc::lidar::Vec(o.detection.f32_CoreX,o.detection.f32_CoreY,o.detection.f32_CoreZ)).z();
      const auto& cue=o.detection.low_obstacle_cue;
      // A camera-confirmed LOW detection may use the EXISTING tracker for a
      // short dropout. This is a prediction, never a new measurement, class
      // confirmation or point restoration. Predictions never renew this lease.
      const bool short_cue_prediction=config_.tunnel_route.low_obstacle_evidence &&
          cue.stamp_ns>0 && !cue.source_class.empty() && cue.camera_epoch!=0 &&
          raw>=cue.stamp_ns && raw-cue.stamp_ns<=200000000LL && o.tracking.s32_MissCount<=2 &&
          scan.tunnel_camera && scan.tunnel_camera->source_active && scan.tunnel_camera->camera_epoch==cue.camera_epoch &&
          scan.tunnel_camera->source_current && scan.tunnel_camera->source_current();
      if(o.tracking.s32_MissCount>0 && z < config_.tunnel_route.cluster_z.min_sensor_z_m && !short_cue_prediction) {
        ++result.tunnel.cluster_z_removed_tracks;return true;
      }
      return false;
    }),object.end());
  }
  tracker_.Commit(object);
  // Preserve the complete tracker state, then suppress short lived shape
  // candidates from delivered objects AND their current point support.
  TunnelNoiseGate::Stats noise_stats;
  if(!original_tunnel)tunnel_noise_entry_stamp_=0;
  else if(!tunnel_noise_entry_stamp_)tunnel_noise_entry_stamp_=raw;
  const bool noise_interior=original_tunnel&&raw-tunnel_noise_entry_stamp_>=2000000000LL;
  std::set<int> boundary_exempt;
  if(original_tunnel)for(const auto& o:object)
    if(!tunnel_route_->InsideCurrentWalls(o))boundary_exempt.insert(o.tracking.s32_Id);
  std::map<int,ConfirmationKind> confirmation_kinds;
  const auto confirmation_walls=original_tunnel && config_.tunnel_noise.adaptive_confirmation?
      ConfirmationWallSegments(tunnel_route_->last(),config_.tunnel_route.params):
      std::vector<tunnel_filter::WallSegment>{};
  if(original_tunnel && config_.tunnel_noise.adaptive_confirmation)for(const auto& o:object) {
    const auto& d=o.detection;
    ConfirmationEvidence e;
    e.length=d.obb_length_m;e.width=d.obb_width_m;e.height=d.f32_OBB_ZMax-d.f32_OBB_ZMin;
    e.ground_valid=d.ground_evidence.valid;
    e.median_ground_clearance=d.ground_evidence.median_clearance_m;
    const double axis=d.f32_OBB_YawRad+(d.obb_width_m>d.obb_length_m?1.5707963267948966:0);
    const double dx=std::cos(axis),dy=std::sin(axis);
    for(const auto& seg:confirmation_walls) {
        const double norm=std::hypot(seg.plane.a,seg.plane.b);
        if(norm<=0 || d.pcl_ClusterPoints.empty())continue;
        int support=0;
        for(const auto& q:d.pcl_ClusterPoints) {
          const double dist=std::fabs(seg.plane.a*q.x+seg.plane.b*q.y+seg.plane.c*q.z+seg.plane.d)/norm;
          if(dist<=.8 && seg.InIntervals(seg.AlongWall(q.x,q.y),0))++support;
        }
        const double fraction=double(support)/d.pcl_ClusterPoints.size();
        if(!e.wall_valid || fraction>e.wall_support_fraction) {
          e.wall_valid=true;e.wall_support_fraction=fraction;
          e.wall_axis_sine=std::fabs(seg.plane.a*dx+seg.plane.b*dy)/norm;
        }
      }
    confirmation_kinds[o.tracking.s32_Id]=ClassifyConfirmation(e);
    result.tunnel.temporal_candidate_kinds[o.tracking.s32_Id]=static_cast<int>(confirmation_kinds[o.tracking.s32_Id]);
  }
  const auto hidden=tunnel_noise_gate_.Update(object,ground_plane,
      result.tunnel.wall_left_removal_allowed||result.tunnel.wall_right_removal_allowed,
      (config_.tunnel_noise.confirmation_all_unknown?original_tunnel:noise_interior),raw,config_.tunnel_noise,noise_stats,
      config_.tunnel_noise.confirmation_all_unknown?nullptr:&boundary_exempt,&confirmation_kinds);
  if(original_tunnel) {
    result.tunnel.temporal_required_frames=noise_stats.required_frames;
    result.tunnel.temporal_enabled=config_.tunnel_noise.enabled;
    result.tunnel.temporal_entry_guard=!config_.tunnel_noise.confirmation_all_unknown&&!noise_interior;
    for(const auto& o:object)if(!config_.tunnel_noise.confirmation_all_unknown&&!o.tracking.s32_MissCount&&boundary_exempt.count(o.tracking.s32_Id))++result.tunnel.temporal_boundary_exempt;
    result.tunnel.temporal_hold_frames=config_.tunnel_noise.confirmation_all_unknown?
        config_.tunnel_noise.confirmation_frames-1:config_.tunnel_noise.hold_frames;
    result.tunnel.temporal_candidates=noise_stats.candidates;
    result.tunnel.temporal_hidden=noise_stats.hidden;
    result.tunnel.temporal_hidden_predictions=noise_stats.hidden_predictions;
    result.tunnel.temporal_max_run=noise_stats.max_run;
  }
  if(!hidden.empty()) {
    using Key=std::tuple<float,float,float>;
    std::set<Key> rejected;
    for(const auto& o:object)if(!o.tracking.s32_MissCount&&hidden.count(o.tracking.s32_Id))
      for(const auto& q:o.detection.pcl_ClusterPoints)rejected.emplace(q.x,q.y,q.z);
    // Shared support belongs to a surviving observed object, so keep it.
    for(const auto& o:object)if(!o.tracking.s32_MissCount&&!hidden.count(o.tracking.s32_Id))
      for(const auto& q:o.detection.pcl_ClusterPoints)rejected.erase(Key(q.x,q.y,q.z));
    std::vector<int> remap(ground_cloud->size(),-1),kept;
    CloudPtr filtered(new Cloud);filtered->header=ground_cloud->header;
    for(std::size_t k=0;k<ground_cloud->size();++k) {
      const auto& q=ground_cloud->at(k);
      if(rejected.count(Key(q.x,q.y,q.z))){++result.tunnel.temporal_removed_points;continue;}
      remap[k]=filtered->size();filtered->push_back(q);kept.push_back(ground_kept_indices.at(k));
    }
    ground_cloud=filtered;ground_kept_indices=std::move(kept);
    CloudPtr centers(new Cloud);std::vector<pcl::PointIndices> clusters;
    for(const auto& c:all_cluster_indices) {
      pcl::PointIndices mapped;pcl::PointXYZI center{};
      for(int i:c.indices)if(remap.at(i)>=0){mapped.indices.push_back(remap[i]);const auto& q=filtered->at(remap[i]);center.x+=q.x;center.y+=q.y;center.z+=q.z;}
      if(mapped.indices.empty())continue;
      center.x/=mapped.indices.size();center.y/=mapped.indices.size();center.z/=mapped.indices.size();
      clusters.push_back(std::move(mapped));centers->push_back(center);
    }
    all_cluster_indices=std::move(clusters);centroid_cloud=centers;
    auto removed_detection=[&](const auto& o) {
      return !o.detection.pcl_ClusterPoints.empty()&&std::all_of(o.detection.pcl_ClusterPoints.begin(),o.detection.pcl_ClusterPoints.end(),[&](const auto& q){return rejected.count(Key(q.x,q.y,q.z));});
    };
    observed_detections.erase(std::remove_if(observed_detections.begin(),observed_detections.end(),removed_detection),observed_detections.end());
    result.detections.erase(std::remove_if(result.detections.begin(),result.detections.end(),removed_detection),result.detections.end());
    object.erase(std::remove_if(object.begin(),object.end(),[&](const auto& o){return hidden.count(o.tracking.s32_Id);}),object.end());
    result.tunnel.removed_points+=result.tunnel.temporal_removed_points;
    counts.ground_kept_points=counts.cluster_input_points=ground_cloud->size();
    counts.tunnel_removed_points=result.tunnel.removed_points;
    counts.clusters=counts.centroids=all_cluster_indices.size();
    counts.detections=all_cluster_indices.size();
    result.tunnel.clusters_after_filters=all_cluster_indices.size();
    result.tunnel.unclustered_points=ground_cloud->size();
    for(const auto& c:all_cluster_indices)result.tunnel.unclustered_points-=c.indices.size();
    if(debug) {
      result.digests.ground_kept=CloudDigest(*ground_cloud);result.centroids=centroid_cloud;
      result.cluster_sizes.clear();for(const auto& c:all_cluster_indices)result.cluster_sizes.push_back(c.indices.size());
    }
  }
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
                         (tunnel_ || original_tunnel) ? pre_tunnel_kept_indices : ground_kept_indices,
                         all_cluster_indices,
                         observed_detections};
    config_.observer->OnScan(view);
  }
  return result;
}

void Pipeline::FinalizeMotion(ScanResult& result) {
  if(!config_.motion.semantic_static_enabled||motion_input_stamp_!=result.stamp_ns)return;
  for(auto& m:motion_inputs_) {
    auto obj=fusion_state::Find(result.objects,m.s32_Id);
    m.semantic_static_eligible=false;
    if(obj) {
      const auto& c=obj->camera;
      const auto age=result.stamp_ns-static_cast<std::int64_t>(c.camera_observation_ns);
      m.semantic_static_eligible=c.str_ObjectClass=="static_obstacle"&&c.observed_class=="static_obstacle"&&
        !c.group_conflict&&c.class_miss_counter==0&&c.camera_observation_ns>0&&age>=-50000000&&age<=350000000;
    }
  }
  if(motion_map_valid_)motion_classifier_.Update(motion_inputs_,SourceDeltaSeconds(result.stamp_ns,0));
  else motion_classifier_.InvalidateMapHistory(motion_inputs_);
  for(const auto& m:motion_inputs_) {
    if(auto o=fusion_state::Find(result.objects,m.s32_Id))o->tracking.motion=m;
    if(auto h=tracker_.LastFrame())if(auto o=fusion_state::Find(*h,m.s32_Id))o->tracking.motion=m;
  }
  motion_input_stamp_=0;motion_inputs_.clear();
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
