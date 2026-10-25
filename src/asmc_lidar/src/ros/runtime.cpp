#include <asmc_lidar/tracking/track.hpp>
// Shadow runtime: sensor input gates and the per-scan sequence of Main's
// TemporalThread (common geometry), transport-free (see shadow/runtime.hpp).
#include <asmc_lidar/ros/runtime.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace asmc {
namespace lidar {
namespace runtime {
namespace {

using Clock = std::chrono::steady_clock;

double Ms(Clock::time_point from, Clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}

// Ported from Main src/main.cpp:507-515 (8f93c22e): the scan's GPS antenna
// position in the map frame, or NaN/jamming when the map pose is unavailable.
void BackFillAntenna(const asmc::lidar::PoseSelection& map, const pipeline::VehicleSpec& vehicle,
                     pipeline::LocalState* local) {
  if (map.valid) {
    const Eigen::Vector3d gps = map.map_from_base.translation() + map.map_from_base.rotation() *
        Eigen::Vector3d(vehicle.st_GPSPos.f32_X, vehicle.st_GPSPos.f32_Y, vehicle.st_GPSPos.f32_Z);
    local->st_GPS.f64_E = gps.x(); local->st_GPS.f64_N = gps.y(); local->st_GPS.f64_U = gps.z();
    local->st_GPS.b_isJamming = false;
  } else {
    local->st_GPS.f64_E = local->st_GPS.f64_N = local->st_GPS.f64_U = NAN;
    local->st_GPS.b_isJamming = true;
  }
}

// Ported from Main src/SensorFusion/FusionLiDAR.cpp:94-132 (UpdateAABB, called
// by FusionLiDAR :224-229 from FusionProcess, SensorFusion.cpp:6-11) (8f93c22e).
// Rewrites only the AABB and Core of the current frame (never the tracker
// history or the OBB used by the typed output). Main's `f32_Yaw = 0` is dropped:
// that legacy AABB yaw field is not part of pipeline::TrackedObject.
// PublishBoxesAndPcl (visualization) is not ported.
void UpdateAABB(std::vector<pipeline::TrackedObject>& objects) {
  for (auto& obj : objects) {
    if (obj.detection.pcl_ClusterPoints.empty()) {
      obj.detection.f32_MinX = obj.detection.f32_MaxX = obj.detection.f32_CoreX;
      obj.detection.f32_MinY = obj.detection.f32_MaxY = obj.detection.f32_CoreY;
      obj.detection.f32_MinZ = obj.detection.f32_MaxZ = obj.detection.f32_CoreZ;
      continue;
    }
    pipeline::float32 minX = std::numeric_limits<pipeline::float32>::max();
    pipeline::float32 minY = std::numeric_limits<pipeline::float32>::max();
    pipeline::float32 minZ = std::numeric_limits<pipeline::float32>::max();
    pipeline::float32 maxX = -std::numeric_limits<pipeline::float32>::max();
    pipeline::float32 maxY = -std::numeric_limits<pipeline::float32>::max();
    pipeline::float32 maxZ = -std::numeric_limits<pipeline::float32>::max();
    for (const auto& pt : obj.detection.pcl_ClusterPoints.points) {
      if (pt.x < minX) minX = pt.x;
      if (pt.y < minY) minY = pt.y;
      if (pt.z < minZ) minZ = pt.z;
      if (pt.x > maxX) maxX = pt.x;
      if (pt.y > maxY) maxY = pt.y;
      if (pt.z > maxZ) maxZ = pt.z;
    }
    obj.detection.f32_MinX = minX; obj.detection.f32_MaxX = maxX;
    obj.detection.f32_MinY = minY; obj.detection.f32_MaxY = maxY;
    obj.detection.f32_MinZ = minZ; obj.detection.f32_MaxZ = maxZ;
    obj.detection.f32_CoreX = 0.5f * (minX + maxX);
    obj.detection.f32_CoreY = 0.5f * (minY + maxY);
    obj.detection.f32_CoreZ = 0.5f * (minZ + maxZ);
  }
}

}  // namespace

LidarRuntime::LidarRuntime(const RuntimeConfig& config,
                             std::shared_ptr<pipeline::fusion::CameraSource> camera,
                             const asmc::lidar::camera::Calibration* front_calibration)
    : config_(config), pose_(config.pose), gps_(config.gps), pipeline_(config.pipeline),
      camera_(std::move(camera)), output_(config.stamp_provenance),
      delivery_filters_(config.delivery_filters ? *config.delivery_filters : filtering::DeliveryFilters{}) {
  if (config_.enable_camera_fusion && camera_) {
    if (!front_calibration)
      throw std::invalid_argument("LidarRuntime: camera fusion needs the front camera calibration");
    fusion_ = std::make_unique<pipeline::fusion::FrontCameraFusion>(config_.fusion, *front_calibration);
  }
}

// Ported from Main src/Parser/Parser.cpp:566-612 (IMUParser_Callback,
// common branch) (8f93c22e). The raw IMU buffer write (:614-618) only feeds
// legacy/camera consumers and is not ported.
void LidarRuntime::OnImu(const pipeline::ImuInput& input, std::int64_t receipt_ns, std::int64_t ros_now_ns) {
  const pipeline::ImuData data = pipeline::ImuFromInput(input);
  std::string sanity_reason;
  // Main passes stamp_ms of (header or ros::Time::now()) and the raw header ns.
  const auto stamp_ms = pipeline::SourceStampMs(input.stamp_ns, static_cast<std::uint64_t>(ros_now_ns));
  if (!pose_.AddImu(stamp_ms, data, &sanity_reason, input.stamp_ns, static_cast<std::uint64_t>(receipt_ns)))
    ++imu_rejected_;
}

// Ported from Main src/Parser/Parser.cpp:519-543 (CANParser_Callback, common
// branch) (8f93c22e). The raw CAN buffer write (:545-549) is not ported.
void LidarRuntime::OnCan(const pipeline::CanInput& input, std::int64_t receipt_ns, std::int64_t ros_now_ns) {
  const pipeline::CanData data = pipeline::CanFromInput(input);
  std::string sanity_reason;
  const auto stamp_ms = pipeline::SourceStampMs(input.stamp_ns, static_cast<std::uint64_t>(ros_now_ns));
  if (!pose_.AddCan(stamp_ms, data, &sanity_reason, input.stamp_ns, static_cast<std::uint64_t>(receipt_ns)))
    ++can_rejected_;
}

// Ported from Main src/Parser/Parser.cpp:398-468 (GPSParser_Callback, common
// branch) (8f93c22e). Only the sanity decision is serialized (g_GpsSanityMutex);
// the history has its own lock. The raw GPS buffer write (:487-491) is not ported.
void LidarRuntime::OnGps(const pipeline::GpsInput& input, std::int64_t receipt_ns, std::int64_t ros_now_ns) {
  pipeline::GpsIngestResult result;
  {
    std::lock_guard<std::mutex> lock(gps_mutex_);
    result = gps_.Evaluate(input, static_cast<std::uint64_t>(ros_now_ns));
    last_gps_reason_ = result.accepted ? "accepted" : result.reason;
  }
  if (!result.accepted) ++gps_rejected_;
  pipeline::GpsIngest::Apply(result, static_cast<std::uint64_t>(receipt_ns), pose_);
}

// Ported from Main src/Parser/Parser.cpp:274-277, 281-285, 310-319
// (LiDARParser_Callback common gates) (8f93c22e). Main keeps the stamp of the
// last accepted scan in the raw buffer (source_stamp_ns); here it is a member.
bool LidarRuntime::AcceptScan(std::int64_t stamp_ns, std::size_t point_count, std::string* reason) {
  const auto reject = [&](const char* why) {
    ++rejected_scans_;
    if (reason) *reason = why;
    return false;
  };
  // Negative stamps cannot come from a ROS header; treated as missing.
  if (stamp_ns <= 0) return reject("zero_source_stamp");
  if (point_count > config_.max_points) return reject("raw_buffer_capacity_exceeded");
  const auto ns = static_cast<std::uint64_t>(stamp_ns);
  const auto source_ms = ns / 1000000ULL;
  // Preserved quirk: a positive stamp below 1 ms (!source_ms) is rejected with
  // the duplicate/regressing reason, as in Main.
  if (!source_ms || ns <= static_cast<std::uint64_t>(last_accepted_scan_ns_))
    return reject("duplicate_or_regressing_source_stamp");
  last_accepted_scan_ns_ = stamp_ns;
  if (reason) reason->clear();
  return true;
}

// Ported from Main src/main.cpp:483-540 (TemporalThread, common geometry)
// (8f93c22e). Dropped here: CopyData/DeadReckoningProcess/ProcessCameraStream
// (no LiDAR-output effect), traces, benchmark, legacy-only pose lookup,
// MotionClassifier, SetLegacyReason/SuppressMap and the planner/UDP outputs.
ScanOutput LidarRuntime::ProcessScan(const pipeline::ScanInput& scan, double wait_timeout_s,
                                      const ScanTimes& times) {
  const auto start = Clock::now();
  ScanOutput out;
  const std::int64_t scan_stamp = scan.time.stamp_ns;
  // Main starts from st_GPSIMU (latest raw GPS/IMU/CAN); the common lookup
  // overwrites st_IMU/st_CAN and the back-fill st_GPS E/N/U. The remaining
  // fields (GPS lat/lon/alt, b_Jamming) are not read by the common path and
  // stay default here.
  pipeline::LocalState local;
  std::string pose_reason;
  out.pose_ready = pose_.WaitForLocalLookupNs(static_cast<std::uint64_t>(scan_stamp), wait_timeout_s,
                                              &local, &out.selected, &pose_reason);
  out.wait_ms = Ms(start, Clock::now());
  if (!out.pose_ready) {
    delivery_filters_.BeginFrame(false);
    // Main skips LidarProcess: the tracker is not advanced.
    out.reason = pose_reason;
    out.arrays = output_.Invalid(scan_stamp, pose_reason);
    Audit(out);
    out.total_ms = Ms(start, Clock::now());
    return out;
  }
  // Main ScanTime(raw) after the wait (FrameBoundary.cpp:141-145).
  pipeline::ScanInput input = scan;
  input.time = asmc::lidar::Observation{scan_stamp, times.receipt_ns, times.processing_ns, "ros",
                                        config_.stamp_provenance, "steady"};
  const auto map = pose_.MapAt(input.time, local);
  out.ego_speed_mps = local.st_CAN.f32_CurrentVel_MPS;
  BackFillAntenna(map, config_.pipeline.vehicle, &local);
  out.result = pipeline_.Process(input, local, map);
  out.processed = out.result.processed;
  if (!out.processed) {
    delivery_filters_.BeginFrame(false);
    out.reason = "empty_or_invalid_lidar_input";
    out.arrays = output_.Invalid(scan_stamp, out.reason);
    Audit(out);
    out.total_ms = Ms(start, Clock::now());
    return out;
  }
  // Main FusionProcess (enable_sensor_fusion): runCameraFusion, then FusionLiDAR.
  // Without a camera source runCameraFusion is skipped; this equals Main with
  // no camera status/YOLO traffic, where the call leaves every class unchanged.
  if (config_.enable_camera_fusion) {
    if (fusion_ && camera_) {
      ScanTimes now = times;
      if (times.refresh_before_fusion) times.refresh_before_fusion(now);
      out.fusion = fusion_->Process(now.decision_ns, out.result.stamp_ns, out.result.context,
                                    out.result.objects, pipeline_.tracker().LastFrame(), *camera_,
                                    now.ros_now_ns, now.steady_now_ns);
      out.fusion_ran = true;
    }
    UpdateAABB(out.result.objects);
  }
  out.arrays = output_.Observed(out.result, out.result.objects);
  Audit(out);
  FilterDelivery(out);
  out.total_ms = Ms(start, Clock::now());
  return out;
}

void LidarRuntime::Audit(ScanOutput& out) const {
  if (config_.hd_map_audit_mode == "off") return;
  const auto start = Clock::now();
  auto& report = out.hd_map_audit;
  report.enabled = true;
  report.minimum_support_points=config_.delivery_preview.minimum_points;
  report.minimum_road_points=config_.delivery_preview.minimum_road_points;
  report.minimum_road_ratio=config_.delivery_preview.minimum_road_ratio;
  report.minimum_obb_road_ratio=config_.delivery_preview.minimum_obb_road_ratio;
  report.road_inset_m=config_.hd_map_audit?config_.hd_map_audit->road_inset_m():config_.delivery_preview.road_inset_m;
  report.roof_limit_base_z=config_.delivery_preview.roof_base_z_m+config_.delivery_preview.roof_margin_m;
  if(config_.delivery_preview.enabled) report.preview_config_id=config_.config_id;
  report.bundle_sha256 = config_.hd_map_manifest_sha256;
  const auto& mapped = out.arrays.map;
  const bool valid_map = out.arrays.accepted && (mapped.status == 1 || mapped.status == 2)
                         && out.result.context.map_pose.valid;
  const auto& objects = valid_map ? mapped : out.arrays.local;
  const auto& audit = config_.hd_map_audit;
  report.reason = audit ? (objects.objects.empty() ?
      (valid_map ? "valid_empty" : "input_or_pose_unavailable") : "evaluated") : "map_audit_unavailable";
  report.available = bool(audit);
  if (audit) {
    report.map_id = audit->map_id(); report.map_sha256 = audit->map_sha256();
    report.bundle_sha256 = audit->bundle_sha256(); report.policy_sha256 = audit->policy_sha256();
  }
  // Identity is derived from the pinned bundle and effective map-origin config,
  // not a new claim that the sensor measured a map ID or a position error bound.
  filtering::AuditFrame frame;
  frame.frame_id = valid_map ? "map" : objects.header.frame_id;
  frame.status = objects.status;
  frame.map_id = report.map_id;
  for (int i=0;i<3;++i) frame.map_origin[i] = config_.gps.map_origin[i];
  frame.stamp_ns = static_cast<std::int64_t>(objects.header.stamp.toNSec());
  frame.pose_stamp_ns = frame.stamp_ns;
  frame.pose_valid = valid_map;
  const auto& stamps = out.result.context.map_pose.sample_stamps;
  if (!stamps.empty()) frame.source_age_ms = double(frame.stamp_ns -
      static_cast<std::int64_t>(*std::max_element(stamps.begin(), stamps.end()))) / 1e6;
  report.pose_source_age_ms=frame.source_age_ms;
  report.pose_method=out.result.context.map_pose.reason;
  // Deliberately false: no independent positional bound/approach validation exists.
  frame.position_error_verified = false; frame.mission_scope_verified = false;
  report.objects.reserve(objects.objects.size());
  const auto base_from_level=asmc::lidar::base_from_input(out.result.context);
  const Pose map_from_level=base_from_level ?
      Pose(out.result.context.map_pose.map_from_base * *base_from_level) : Pose::Identity();
  for (const auto& obj : objects.objects) {
    filtering::AuditObject input;
    input.track_id = obj.track_id;
    input.observation_state = obj.observation_state == asmc_msgs::LidarObject::PREDICTED ? "predicted" : "observed";
    for (const auto& p : obj.obb_corners) input.corners.push_back({p.x,p.y,p.z});
    try {
        const auto object_start=Clock::now();
        filtering::AuditDecision decision;
        if(audit) decision=audit->Evaluate(frame,input);
        else {
          decision.track_id=input.track_id;decision.input_state=input.observation_state;
          decision.reason="map_audit_unavailable";
        }
        if(config_.delivery_preview.enabled) {
          const auto source=std::find_if(out.result.objects.begin(),out.result.objects.end(),
              [&](const auto& o){return o.tracking.s32_Id==obj.track_id;});
          filtering::PointEvidence evidence;
          std::string matched_group="unknown";
          double bottom=std::numeric_limits<double>::quiet_NaN();
          if(source!=out.result.objects.end()&&base_from_level) {
            bottom=INFINITY;
            for(const auto& p:source->detection.obb_corners_level)
              bottom=std::min(bottom,((*base_from_level)*Vec(p.x,p.y,p.z)).z());
            for(const auto& p:source->detection.pcl_ClusterPoints.points) {
              const Vec point(p.x,p.y,p.z);
              const double base_z=base_from_level->linear().row(2).dot(point)+base_from_level->translation().z();
              const Vec mapped=valid_map?map_from_level*point:Vec(NAN,NAN,NAN);
              if(audit) audit->Accumulate(evidence,{mapped.x(),mapped.y(),mapped.z()},base_z);
              else {
                ++evidence.total;++evidence.unknown;
                if(std::isfinite(base_z)){++evidence.finite;evidence.min_base_z=std::min(evidence.min_base_z,base_z);}
              }
            }
            const auto& c=source->camera;
            const bool positive=filtering::PositiveCameraEvidence(frame.stamp_ns,c.camera_observation_ns,
              fusion_?fusion_->consumer_epoch():0,c.camera_observation_epoch,c.class_miss_counter,
              config_.delivery_preview.camera_age_ms,config_.fusion.max_time_offset_s*1e3);
            const auto group=pipeline::fusion_class::ObservedMissionGroup(c);
            decision.camera_observed_group=group;
            decision.camera_observed_raw=c.observed_raw_class;
            if(positive)matched_group=group;
          }
          filtering::ApplyDeliveryPreview(decision,evidence,config_.delivery_preview,valid_map&&bool(audit),matched_group,bottom);
        }
        decision.comparison_ms=Ms(object_start,Clock::now());
        report.objects.push_back(std::move(decision));
    } catch (const std::exception&) {
      // Diagnostic failure is visible but cannot suppress a scan or mutate tracks.
      filtering::AuditDecision decision;
      decision.track_id=input.track_id; decision.input_state=input.observation_state;
      decision.reason="map_audit_calculation_error"; report.objects.push_back(std::move(decision));
      report.reason="calculation_error";
    }
  }
  report.calculation_ms = Ms(start, Clock::now());
}

void LidarRuntime::FilterDelivery(ScanOutput& out) {
  auto& report=out.hd_map_audit;
  report.delivery_input_count=out.arrays.local.objects.size();
  report.delivery_output_count=report.delivery_input_count;
  const bool map_filter=config_.hd_map_audit_mode=="filter"&&report.enabled;
  if((!map_filter&&!delivery_filters_.enabled())||!out.arrays.accepted||
     (out.arrays.local.status!=asmc_msgs::LidarObjectArray::OBSERVED&&
      out.arrays.local.status!=asmc_msgs::LidarObjectArray::OBSERVED_EMPTY))return;
  report.delivery_applied=true;
  std::unordered_set<int> excluded;
  for(auto& d:report.objects) {
    d.delivery_excluded=map_filter&&d.preview_enabled&&d.preview_decision=="exclude_candidate";
    if(d.delivery_excluded)excluded.insert(d.track_id);
  }
  const bool valid_map=out.result.context.map_pose.valid&&
      (out.arrays.map.status==asmc_msgs::LidarObjectArray::OBSERVED||
       out.arrays.map.status==asmc_msgs::LidarObjectArray::OBSERVED_EMPTY);
  delivery_filters_.BeginFrame(valid_map);
  if(delivery_filters_.enabled()&&valid_map) {
    report.enabled=true;
    const auto& vehicle=config_.pipeline.vehicle;
    const Vec antenna=out.result.context.map_pose.map_from_base *
        Vec(vehicle.st_GPSPos.f32_X,vehicle.st_GPSPos.f32_Y,vehicle.st_GPSPos.f32_Z);
    for(std::size_t i=0;i<out.arrays.map.objects.size();++i) {
      const auto& object=out.arrays.map.objects[i];
      if(excluded.count(object.track_id))continue;
      auto decision=std::find_if(report.objects.begin(),report.objects.end(),
          [&](const auto& d){return d.track_id==object.track_id;});
      // Preserve PR #108's explicit fresh-camera protection, including deer/stroller.
      if(decision!=report.objects.end()&&(decision->camera_dynamic_protected||decision->camera_vehicle_protected))continue;
      const auto original=std::find_if(out.result.objects.begin(),out.result.objects.end(),
          [&](const auto& o){return o.tracking.s32_Id==object.track_id;});
      if(original==out.result.objects.end())continue;
      filtering::DeliveryObject candidate;
      candidate.id=object.track_id;candidate.classification=object.classification;
      candidate.yolo_class=object.camera_class;candidate.is_dynamic=object.is_dynamic;
      candidate.motion_state=out.arrays.map_tracking[i].motion_state;
      const auto& center=out.arrays.map_tracking[i].cluster_center;
      candidate.cluster_center_map={center.x,center.y,center.z};
      candidate.obb_center_map={object.center.x,object.center.y,object.center.z};
      for(unsigned k=0;k<8;++k){const auto& p=object.obb_corners[k];candidate.obb_corners_map[k]={p.x,p.y,p.z};}
      candidate.length=object.length;candidate.width=object.width;candidate.height=object.height;
      candidate.ground_evidence=original->detection.ground_evidence;
      const auto reason=delivery_filters_.Exclusion(candidate,antenna.x(),antenna.y(),out.ego_speed_mps,vehicle.f32_Length);
      if(reason.empty())continue;
      excluded.insert(object.track_id);
      if(decision==report.objects.end()) {
        filtering::AuditDecision fresh;fresh.track_id=object.track_id;
        report.objects.push_back(std::move(fresh));decision=std::prev(report.objects.end());
      }
      decision->delivery_excluded=true;decision->preview_enabled=true;
      decision->preview_decision="exclude_candidate";decision->preview_reason=reason;
      decision->reason=reason;
    }
  }
  // One decision set for both coordinate frames and both observation wrappers.
  // Stable in-place compaction: no cloud copies, tracker deletion or OBB edits.
  const auto filter=[&](auto& array){
    const auto before=array.objects.size();
    array.objects.erase(std::remove_if(array.objects.begin(),array.objects.end(),
      [&](const auto& obj){return excluded.count(obj.track_id);}),array.objects.end());
    if(before&&!array.objects.size()&&array.status==asmc_msgs::LidarObjectArray::OBSERVED){
      array.status=asmc_msgs::LidarObjectArray::OBSERVED_EMPTY;
      array.reason="delivery_filtered_empty";
    }
  };
  filter(out.arrays.local);filter(out.arrays.map);
  for (auto* tracking : {&out.arrays.local_tracking, &out.arrays.map_tracking}) {
    tracking->erase(std::remove_if(tracking->begin(), tracking->end(),
        [&](const auto& state) { return excluded.count(state.track_id); }), tracking->end());
  }
  report.delivery_output_count=out.arrays.local.objects.size();
  report.delivery_excluded_count=report.delivery_input_count-report.delivery_output_count;
}

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
