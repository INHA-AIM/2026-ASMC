#pragma once
#include <asmc_lidar/tracking/track.hpp>
// asmc_lidar_scan_trace_v1: one JSON object per input PointCloud2.
// Written as JSONL by lidar_replay and published (std_msgs/String,
// only while subscribed) by lidar_pipeline_node. Field names and order follow
// the schema in docs/scan-trace-schema.md:
//   schema, producer, seq, stamp_ns, arrival_ns, accepted, accept_reason,
//   pose_ready, reason, imu_ns, can_ns, local, map, processed, process_reason,
//   dt_s, counts, digests*, plane, leveling, base_from_input, clusters*,
//   detections*, tracks, typed            (* capture_debug only)
// Numbers use 17 significant digits (a float widened to double round-trips).
// Missing data is an absent field. JSON has no NaN/Inf: non-finite numbers are
// written as null. Header-only; it serializes LidarRuntime output, hence the
// shadow namespace.

#include <asmc_lidar/geometry/pose_history.hpp>
#include <asmc_lidar/ros/runtime.hpp>

#include <asmc_msgs/LidarObjectArray.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace asmc {
namespace lidar {
namespace runtime {
namespace trace {

inline void Str(std::ostream& out, const std::string& text) {
  out << '"';
  for (const char ch : text) {
    const auto c = static_cast<unsigned char>(ch);
    if (c == '"') out << "\\\"";
    else if (c == '\\') out << "\\\\";
    else if (c == '\n') out << "\\n";
    else if (c == '\r') out << "\\r";
    else if (c == '\t') out << "\\t";
    else if (c < 0x20) {
      char buffer[8];
      std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(c));
      out << buffer;
    } else {
      out << ch;  // UTF-8 bytes pass through
    }
  }
  out << '"';
}

inline void Num(std::ostream& out, double value) {
  if (std::isfinite(value)) out << value;
  else out << "null";
}

inline void Bool(std::ostream& out, bool value) { out << (value ? "true" : "false"); }

inline void Vec3(std::ostream& out, double x, double y, double z) {
  out << '[';
  Num(out, x);
  out << ',';
  Num(out, y);
  out << ',';
  Num(out, z);
  out << ']';
}

// Row-major dump of a fixed-size Eigen matrix.
template <class Matrix>
void RowMajor(std::ostream& out, const Matrix& m) {
  out << '[';
  for (int r = 0; r < m.rows(); ++r) {
    for (int c = 0; c < m.cols(); ++c) {
      if (r || c) out << ',';
      Num(out, static_cast<double>(m(r, c)));
    }
  }
  out << ']';
}

// <object>: idx, core, min, max, dist, points, obb, corners (no braces).
inline void ObjectFields(std::ostream& out, const pipeline::TrackedObject& o) {
  out << "\"idx\":" << static_cast<unsigned>(o.detection.u16_Idx) << ",\"core\":";
  Vec3(out, o.detection.f32_CoreX, o.detection.f32_CoreY, o.detection.f32_CoreZ);
  out << ",\"min\":";
  Vec3(out, o.detection.f32_MinX, o.detection.f32_MinY, o.detection.f32_MinZ);
  out << ",\"max\":";
  Vec3(out, o.detection.f32_MaxX, o.detection.f32_MaxY, o.detection.f32_MaxZ);
  out << ",\"dist\":";
  Num(out, o.detection.f32_Dist);
  out << ",\"points\":" << o.detection.pcl_ClusterPoints.size() << ",\"obb\":{\"center\":";
  Vec3(out, o.detection.f32_OBB_CenterX, o.detection.f32_OBB_CenterY, o.detection.f32_OBB_CenterZ);
  out << ",\"width\":";
  Num(out, o.detection.obb_length_m);
  out << ",\"height\":";
  Num(out, o.detection.obb_width_m);
  out << ",\"zmin\":";
  Num(out, o.detection.f32_OBB_ZMin);
  out << ",\"zmax\":";
  Num(out, o.detection.f32_OBB_ZMax);
  out << ",\"yaw\":";
  Num(out, o.detection.f32_OBB_YawRad);
  out << "},\"corners\":[";
  for (std::size_t i = 0; i < o.detection.obb_corners_level.size(); ++i) {
    if (i) out << ',';
    const auto& p = o.detection.obb_corners_level[i];
    Vec3(out, p.x, p.y, p.z);
  }
  out << ']';
}

// <track fields> (no braces, leading comma).
inline void TrackFields(std::ostream& out, const pipeline::TrackedObject& o) {
  out << ",\"id\":" << o.tracking.s32_Id << ",\"age\":" << o.tracking.s32_Age << ",\"miss\":" << o.tracking.s32_MissCount
      << ",\"kf_initialized\":";
  Bool(out, o.tracking.b_KfInitialized);
  out << ",\"kf_init_count\":" << o.tracking.s32_KfInitCount << ",\"kf_ca\":[";
  for (int i = 0; i < 6; ++i) {
    if (i) out << ',';
    Num(out, o.tracking.Eigen_KfState_CA(i));
  }
  out << "],\"kf_p\":";
  RowMajor(out, o.tracking.Eigen_KfP_CA);
  out << ",\"prev_yaw\":";
  Num(out, o.tracking.f32_PrevYaw);
  out << ",\"pred_core\":[";
  Num(out, o.tracking.f32_PredCoreX);
  out << ',';
  Num(out, o.tracking.f32_PredCoreY);
  out << "],\"abs_v\":[";
  Num(out, o.tracking.f32_AbsVx);
  out << ',';
  Num(out, o.tracking.f32_AbsVy);
  out << "],\"velocity\":";
  Num(out, o.tracking.f32_Velocity);
  out << ",\"tracking_frame_valid\":";
  Bool(out, o.tracking.tracking_frame_valid);
  out << ",\"tracking_height_input\":";
  Num(out, o.tracking.tracking_height_input);
  out << ",\"class\":";
  Str(out, o.camera.str_ObjectClass);
  out << ",\"raw_class\":";
  Str(out, o.camera.str_YoloClass);
  out << ",\"class_misses\":" << o.camera.class_miss_counter << ",\"continuity\":" << o.camera.continuity_counter
      << ",\"ground_evidence\":{\"valid\":";
  Bool(out, o.detection.ground_evidence.valid);
  out << ",\"samples\":" << o.detection.ground_evidence.sample_count << ",\"median_clearance\":";
  Num(out, o.detection.ground_evidence.median_clearance_m);
  out << ",\"band\":";
  Num(out, o.detection.ground_evidence.rejection_band_m);
  out << '}';
}

inline void Point(std::ostream& out, const geometry_msgs::Point& p) { Vec3(out, p.x, p.y, p.z); }

// <array>; arrays that were not accepted carry only {"accepted":false}.
inline void Array(std::ostream& out, bool accepted, const asmc_msgs::LidarObjectArray& a) {
  if (!accepted) {
    out << "{\"accepted\":false}";
    return;
  }
  out << "{\"accepted\":true,\"status\":" << static_cast<unsigned>(a.status) << ",\"reason\":";
  Str(out, a.reason);
  out << ",\"frame\":";
  Str(out, a.header.frame_id);
  out << ",\"stamp_ns\":" << a.header.stamp.toNSec() << ",\"provenance\":";
  Str(out, a.stamp_provenance);
  out << ",\"objects\":[";
  for (std::size_t i = 0; i < a.objects.size(); ++i) {
    const auto& o = a.objects[i];
    if (i) out << ',';
    out << "{\"track_id\":" << o.track_id << ",\"detection_index\":" << o.detection_index
        << ",\"center\":";
    Point(out, o.center);
    out << ",\"corners\":[";
    for (std::size_t c = 0; c < o.obb_corners.size(); ++c) {
      if (c) out << ',';
      Point(out, o.obb_corners[c]);
    }
    out << "],\"yaw\":";
    Num(out, o.yaw);
    out << ",\"length\":";
    Num(out, o.length);
    out << ",\"width\":";
    Num(out, o.width);
    out << ",\"height\":";
    Num(out, o.height);
    out << ",\"points\":" << o.source_point_count << ",\"classification\":";
    Str(out, o.classification);
    out << ",\"camera_class\":";
    Str(out, o.camera_class);
    out << '}';
  }
  out << "]}";
}

// The IMU/CAN values the runtime stored for each accepted sample (same
// ImuFromInput/CanFromInput math), so the trace can report the local inputs
// selected for a scan (ScanOutput carries only their stamps). Not thread-safe.
class LocalInputCache {
 public:
  explicit LocalInputCache(std::size_t capacity = 4096) : capacity_(capacity) {}
  void AddImu(std::uint64_t stamp_ns, const pipeline::ImuData& data) { Push(imu_, stamp_ns, data); }
  void AddCan(std::uint64_t stamp_ns, const pipeline::CanData& data) { Push(can_, stamp_ns, data); }
  // Latest accepted sample with exactly this stamp (an equal stamp replaces, as
  // in Main InsertOrdered); null when unknown.
  const pipeline::ImuData* Imu(std::uint64_t stamp_ns) const { return Find(imu_, stamp_ns); }
  const pipeline::CanData* Can(std::uint64_t stamp_ns) const { return Find(can_, stamp_ns); }

 private:
  template <class T>
  void Push(std::deque<std::pair<std::uint64_t, T>>& samples, std::uint64_t stamp_ns, const T& data) {
    samples.emplace_back(stamp_ns, data);
    while (samples.size() > capacity_) samples.pop_front();
  }
  template <class T>
  static const T* Find(const std::deque<std::pair<std::uint64_t, T>>& samples, std::uint64_t stamp_ns) {
    if (stamp_ns == 0) return nullptr;
    for (auto it = samples.rbegin(); it != samples.rend(); ++it)
      if (it->first == stamp_ns) return &it->second;
    return nullptr;
  }
  std::size_t capacity_;
  std::deque<std::pair<std::uint64_t, pipeline::ImuData>> imu_;
  std::deque<std::pair<std::uint64_t, pipeline::CanData>> can_;
};

struct LineInput {
  std::string producer;
  std::string session_id, config_id, calibration_id;
  std::uint64_t generation = 1;
  std::uint64_t seq = 0;          // 1-based index on the points topic
  std::int64_t stamp_ns = 0;      // PointCloud2 header stamp
  std::int64_t arrival_ns = 0;    // bag record time (replay) / ROS receipt time (node)
  bool accepted = false;          // LidarRuntime::AcceptScan
  std::string accept_reason;
  const ScanOutput* output = nullptr;      // null when not accepted
  const pipeline::ImuData* imu = nullptr;  // selected samples, when known
  const pipeline::CanData* can = nullptr;
  double input_to_publish_ms = -1;  // live node only, monotonic callback entry -> arrays enqueued
  bool debug = false;  // digests/clusters/detections (PipelineConfig::capture_debug)
};

inline std::string Line(const LineInput& in) {
  std::ostringstream out;
  out << std::setprecision(17);
  out << "{\"schema\":\"asmc_lidar_scan_trace_v1\",\"producer\":";
  Str(out, in.producer);
  out << ",\"producer_session_id\":";
  Str(out, in.session_id);
  out << ",\"generation\":" << in.generation << ",\"config_id\":";
  Str(out, in.config_id);
  out << ",\"calibration_id\":";
  Str(out, in.calibration_id);
  out << ",\"seq\":" << in.seq << ",\"stamp_ns\":" << in.stamp_ns << ",\"arrival_ns\":" << in.arrival_ns
      << ",\"accepted\":";
  Bool(out, in.accepted && in.output);
  out << ",\"accept_reason\":";
  Str(out, in.accepted ? std::string() : in.accept_reason);
  if (!in.accepted || !in.output) {
    out << ",\"typed\":{\"local\":{\"accepted\":false},\"map\":{\"accepted\":false}}}";
    return out.str();
  }
  const ScanOutput& o = *in.output;
  const pipeline::ScanResult& r = o.result;
  out << ",\"pose_ready\":";
  Bool(out, o.pose_ready);
  out << ",\"reason\":";
  Str(out, o.pose_ready ? std::string() : o.reason);
  out << ",\"imu_ns\":" << o.selected.imu_stamp_ns << ",\"can_ns\":" << o.selected.can_stamp_ns;
  if (o.pose_ready) {
    if (in.imu && in.can) {
      out << ",\"local\":{\"roll_rad\":";
      Num(out, in.imu->f32_Roll_rad);
      out << ",\"pitch_rad\":";
      Num(out, in.imu->f32_Pitch_rad);
      out << ",\"yaw_rad\":";
      Num(out, in.imu->f32_Yaw_rad);
      out << ",\"yaw_deg\":";
      Num(out, in.imu->f32_Yaw_deg);
      out << ",\"speed_mps\":";
      Num(out, in.can->f32_CurrentVel_MPS);
      out << '}';
    }
    // The scan Context (map pose included) exists only for processed scans.
    if (r.processed) {
      const auto& map = r.context.map_pose;
      out << ",\"map\":{\"valid\":";
      Bool(out, map.valid);
      out << ",\"reason\":";
      Str(out, map.reason);
      out << ",\"map_from_base\":";
      RowMajor(out, map.map_from_base.matrix());
      out << ",\"gps_stamps\":[";
      for (std::size_t i = 0; i < map.sample_stamps.size(); ++i) out << (i ? "," : "") << map.sample_stamps[i];
      out << "]}";
    }
    out << ",\"processed\":";
    Bool(out, r.processed);
    out << ",\"process_reason\":";
    Str(out, r.reason);
    if (r.processed) {
      const auto& n = r.counts;
      out << ",\"dt_s\":";
      Num(out, r.dt_s);
      out << ",\"counts\":{\"raw\":" << n.raw_points << ",\"roi\":" << n.roi_points
          << ",\"height_kept\":"<<n.height_kept_points<<",\"height_removed\":"<<n.height_removed_points
          << ",\"voxel\":" << n.voxel_points << ",\"leveled\":" << n.leveled_points
          << ",\"ground_kept\":" << n.ground_kept_points << ",\"tunnel_removed\":" << n.tunnel_removed_points
          << ",\"cluster_input\":" << n.cluster_input_points << ",\"clusters\":" << n.clusters
          << ",\"centroids\":" << n.centroids << ",\"detections\":" << n.detections
          << ",\"tracks\":" << n.tracks << ",\"observed\":" << n.observed_tracks
          << ",\"predicted\":" << n.predicted_tracks << '}';
      if (in.debug) {
        out << ",\"digests\":{\"roi\":";
        Str(out, r.digests.roi);
        out << ",\"voxel\":";
        Str(out, r.digests.voxel);
        out << ",\"leveled\":";
        Str(out, r.digests.leveled);
        out << ",\"ground_kept\":";
        Str(out, r.digests.ground_kept);
        out << '}';
      }
      out << ",\"plane\":{\"valid\":";
      Bool(out, r.plane.valid);
      out << ",\"coefficients\":[";
      for (std::size_t i = 0; i < r.plane.coefficients.size(); ++i) {
        if (i) out << ',';
        Num(out, r.plane.coefficients[i]);
      }
      out << "]},\"leveling\":";
      RowMajor(out, r.context.applied_leveling);
      if (const auto base = asmc::lidar::base_from_input(r.context)) {
        out << ",\"base_from_input\":";
        RowMajor(out, base->matrix());
      }
      if (in.debug) {
        out << ",\"clusters\":[";
        for (std::size_t i = 0; i < r.cluster_sizes.size(); ++i) {
          if (i) out << ',';
          out << "{\"size\":" << r.cluster_sizes[i];
          if (r.centroids && i < r.centroids->size()) {
            const auto& c = (*r.centroids)[i];
            out << ",\"centroid\":";
            Vec3(out, c.x, c.y, c.z);
          }
          out << '}';
        }
        out << "],\"detections\":[";
        for (std::size_t i = 0; i < r.detections.size(); ++i) {
          out << (i ? ",{" : "{");
          ObjectFields(out, r.detections[i]);
          out << '}';
        }
        out << ']';
      }
      out << ",\"tracks\":[";
      for (std::size_t i = 0; i < r.objects.size(); ++i) {
        out << (i ? ",{" : "{");
        ObjectFields(out, r.objects[i]);
        TrackFields(out, r.objects[i]);
        out << '}';
      }
      out << ']';
    }
  }
  out << ",\"timings_ms\":{\"total\":" << o.total_ms
      << ",\"roi\":" << r.timings.roi_ms << ",\"voxel\":" << r.timings.voxel_ms
      << ",\"level\":" << r.timings.level_ms << ",\"height\":" << r.timings.height_ms << ",\"ground\":" << r.timings.ground_ms
      << ",\"cluster\":" << r.timings.cluster_ms << ",\"detection\":" << r.timings.detection_ms
      << ",\"tracking\":" << r.timings.tracking_ms << '}';
  out << ",\"typed\":{\"local\":";
  Array(out, o.arrays.accepted, o.arrays.local);
  out << ",\"map\":";
  Array(out, o.arrays.accepted, o.arrays.map);
  out << '}';
  if (in.input_to_publish_ms >= 0) out << ",\"input_to_publish_ms\":" << in.input_to_publish_ms;
  if (o.hd_map_audit.enabled) {
    out << ",\"hd_map_audit\":";
    filtering::WriteAuditReport(out, o.hd_map_audit);
  }
  out << '}';
  return out.str();
}

}  // namespace trace
}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
