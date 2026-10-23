#include <asmc_lidar/tracking/track.hpp>
// Shadow runtime: sensor input gates and the per-scan sequence of Main's
// TemporalThread (common geometry), transport-free (see shadow/runtime.hpp).
#include <asmc_lidar/ros/runtime.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
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
      camera_(std::move(camera)), output_(config.stamp_provenance) {
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
    // Main skips LidarProcess: the tracker is not advanced.
    out.reason = pose_reason;
    out.arrays = output_.Invalid(scan_stamp, pose_reason);
    out.total_ms = Ms(start, Clock::now());
    return out;
  }
  // Main ScanTime(raw) after the wait (FrameBoundary.cpp:141-145).
  pipeline::ScanInput input = scan;
  input.time = asmc::lidar::Observation{scan_stamp, times.receipt_ns, times.processing_ns, "ros",
                                        config_.stamp_provenance, "steady"};
  const auto map = pose_.MapAt(input.time, local);
  BackFillAntenna(map, config_.pipeline.vehicle, &local);
  out.result = pipeline_.Process(input, local, map);
  out.processed = out.result.processed;
  if (!out.processed) {
    out.reason = "empty_or_invalid_lidar_input";
    out.arrays = output_.Invalid(scan_stamp, out.reason);
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
  out.total_ms = Ms(start, Clock::now());
  return out;
}

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
