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
  out<<",\"semantic_static_applied\":";Bool(out,o.tracking.motion.semantic_static_applied);
  out<<",\"semantic_reason\":";Str(out,o.tracking.motion.semantic_reason);
  out<<",\"group_conflict\":";Bool(out,o.camera.group_conflict);
  out<<",\"protection_group\":";Str(out,o.camera.protection_group);
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
    // Same published values and validity; the monitor must not infer these
    // from its diagnostic KF copy or equate UNKNOWN with measured zero speed.
    out << ",\"absolute_velocity\":";
    Vec3(out, o.absolute_velocity_mps.x, o.absolute_velocity_mps.y, o.absolute_velocity_mps.z);
    out << ",\"absolute_velocity_valid\":";
    Bool(out, o.absolute_velocity_valid);
    out << ",\"relative_velocity_valid\":";
    Bool(out, o.relative_velocity_valid);
    out << ",\"dynamic_valid\":";
    Bool(out, o.dynamic_valid);
    out << ",\"is_dynamic\":";
    Bool(out, o.is_dynamic);
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

// Shared by full trace and sampled web snapshots; no independent matching here.
inline void Fusion(std::ostream& out, const pipeline::fusion::FusionReport& f, bool ran) {
  out << "{\"ran\":"; Bool(out, ran);
  out << ",\"camera_active\":"; Bool(out, f.camera_active);
  out << ",\"camera_reason\":"; Str(out, f.camera_reason);
  out << ",\"diagnostics_enabled\":"; Bool(out, f.diagnostics_enabled);
  out << ",\"emitting_scan_sequence\":" << f.emitting_scan_sequence
      << ",\"emitting_scan_stamp_ns\":" << f.emitting_scan_stamp_ns
      << ",\"pending_scans\":" << f.pending_scans
      << ",\"decisions_omitted\":" << f.decisions_omitted << ",\"views\":[";
  bool comma = false;
  for (const auto& v : f.views) {
    if (comma) out << ',';
    comma = true;
    out << "{\"camera_id\":"; Str(out, v.frame_id);
    out << ",\"matching_implemented\":"; Bool(out, v.matching_implemented);
    out << ",\"reason\":"; Str(out, v.reason);
    out << ",\"stamp_ns\":" << v.stamp_ns << ",\"image_minus_scan_ns\":" << v.image_minus_scan_ns
        << ",\"candidates\":" << v.candidates << ",\"generation\":" << v.generation
        << ",\"wire_sequence\":" << v.wire_sequence;
    out << ",\"session\":"; Str(out, v.session);
    out << ",\"calibration_id\":"; Str(out, v.calibration_id);
    out << ",\"model_id\":"; Str(out, v.model_id);
    out << ",\"config_id\":"; Str(out, v.config_id); out << '}';
  }
  out << "],\"decisions\":["; comma = false;
  for (const auto& d : f.decisions) {
    if (comma) out << ',';
    comma = true;
    out << "{\"scan_sequence\":" << d.scan_sequence << ",\"scan_stamp_ns\":" << d.scan_stamp_ns
        << ",\"wait_ns\":" << d.wait_ns << ",\"image_stamp_ns\":" << d.image_stamp_ns
        << ",\"image_minus_scan_ns\":" << d.image_minus_scan_ns
        << ",\"generation\":" << d.generation << ",\"wire_sequence\":" << d.wire_sequence;
    out << ",\"action\":"; Str(out, d.action);
    out << ",\"reason\":"; Str(out, d.reason);
    out << ",\"observation_id\":"; Str(out, d.observation_id);
    out << ",\"camera_id\":"; Str(out, d.camera_id);
    out << ",\"session\":"; Str(out, d.session);
    out << ",\"calibration_id\":"; Str(out, d.calibration_id);
    out << ",\"model_id\":"; Str(out, d.model_id);
    out << ",\"config_id\":"; Str(out, d.config_id);
    out << ",\"reused\":"; Bool(out, d.reused);
    out << ",\"projection_valid\":"; Bool(out, d.projection_valid);
    out << ",\"boxes_received\":" << d.boxes_received << ",\"boxes_admitted\":" << d.boxes_admitted
        << ",\"associations_omitted\":" << d.associations_omitted << ",\"associations\":[";
    bool c = false;
    for (const auto& a : d.associations) {
      if (c) out << ',';
      c = true;
      out << "{\"track_id\":" << a.track_id << ",\"source_box_index\":" << a.source_box_index;
      out << ",\"iou\":"; Num(out, a.iou);
      out << ",\"applied\":"; Bool(out, a.applied);
      out << ",\"projected\":"; Bool(out, a.projected);
      out << ",\"reason\":"; Str(out, a.reason);
      out << ",\"raw_class\":"; Str(out, a.raw_class);
      out << ",\"group\":"; Str(out, a.group);
      out << ",\"class_before\":"; Str(out, a.class_before);
      out << ",\"class_after\":"; Str(out, a.class_after);
      out << ",\"projected_box\":[";
      for (std::size_t i=0;i<4;++i) { if(i) out << ','; Num(out,a.projected_box[i]); }
      out << "],\"camera_box\":[";
      for (std::size_t i=0;i<4;++i) { if(i) out << ','; Num(out,a.camera_box[i]); }
      out << "]}";
    }
    out << "]}";
  }
  out << "]}";
}

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

inline void Tunnel(std::ostream& out,const pipeline::TunnelFilterStageReport& t) {
  out << "{\"ran\":";Bool(out,t.ran);
  out << ",\"ground_removed\":" << t.ground_labeled << ",\"wall_removed\":" << t.wall_labeled
      << ",\"removed\":" << t.removed_points
      << ",\"left_allowed\":";Bool(out,t.wall_left_removal_allowed);
  out << ",\"right_allowed\":";Bool(out,t.wall_right_removal_allowed);
  out << ",\"left_reanchored\":";Bool(out,t.wall_left_reanchored);
  out << ",\"right_reanchored\":";Bool(out,t.wall_right_reanchored);
  out << ",\"left_state\":";Str(out,t.wall_left_state);
  out << ",\"right_state\":";Str(out,t.wall_right_state);
  out << ",\"motion_integrated\":";Bool(out,t.motion_integrated);
  out << ",\"motion_valid\":";Bool(out,t.motion_valid);
  out << ",\"motion_reliable\":";Bool(out,t.motion_reliable);
  out << ",\"motion_dt_s\":";Num(out,t.motion_dt_s);
  out << ",\"motion_dyaw_rad\":";Num(out,t.motion_dyaw_rad);
  out << ",\"motion_dx_m\":";Num(out,t.motion_dx_m);
  out << ",\"motion_dy_m\":";Num(out,t.motion_dy_m);
  out << ",\"boundary_rescued_clusters\":" << t.boundary_rescued_clusters << ",\"boundary_rescued_points\":" << t.boundary_rescued_points;
  out << ",\"temporal_enabled\":";Bool(out,t.temporal_enabled);
  out << ",\"temporal_entry_guard\":";Bool(out,t.temporal_entry_guard);
  out << ",\"temporal_boundary_exempt\":" << t.temporal_boundary_exempt;
  out << ",\"temporal_hold_frames\":" << t.temporal_hold_frames
      << ",\"temporal_candidates\":" << t.temporal_candidates
      << ",\"temporal_hidden\":" << t.temporal_hidden
      << ",\"temporal_hidden_predictions\":" << t.temporal_hidden_predictions
      << ",\"temporal_max_run\":" << t.temporal_max_run
      << ",\"temporal_removed_points\":" << t.temporal_removed_points;
  out << ",\"temporal_candidate_kinds\":[";
  bool first_kind=true;
  for(const auto& k:t.temporal_candidate_kinds) {
    if(!first_kind)out << ',';
    first_kind=false;out << '[' << k.first << ',' << k.second << ']';
  }
  out << ']';
  out << ",\"temporal_required_frames\":[";
  first_kind=true;
  for(const auto& k:t.temporal_required_frames) {
    if(!first_kind)out << ',';
    first_kind=false;out << '[' << k.first << ',' << k.second << ']';
  }
  out << ']';
  out << ",\"exterior_surface_removed\":" << t.exterior_surface_removed;
  out << ",\"residual_decisions\":[";
  bool first_decision=true;
  for(const auto& d:t.residual_decisions) {
    if(!first_decision)out<<',';
    first_decision=false;out<<"{\"obb\":[";
    for(std::size_t i=0;i<d.first.size();++i){if(i)out<<',';Num(out,d.first[i]);}
    out<<"],\"reason\":";Str(out,d.second);out<<'}';
  }
  out<<']';
  out << ",\"wall_evidence\":[";
  bool first_wall=true;
  for(const auto& w:t.wall_evidence) {
    if(!first_wall)out << ',';
    first_wall=false;
    out << "{\"side\":" << w.side << ",\"allowed\":";Bool(out,w.allowed);
    out << ",\"valid\":";Bool(out,w.valid);
    out << ",\"plane\":[";for(int i=0;i<4;++i){if(i)out<<',';Num(out,w.plane[i]);}
    out << "],\"span\":";Num(out,w.span);out << ",\"reason\":";Str(out,w.reason);
    out << ",\"intervals\":[";bool first_interval=true;
    for(const auto& interval:w.intervals){if(!first_interval)out<<',';first_interval=false;out<<'[';Num(out,interval.first);out<<',';Num(out,interval.second);out<<']';}
    out << "]}";
  }
  out << ']';
  out << ",\"motion_steps\":" << t.motion_steps << '}';
}

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
  out << ",\"input_selection\":{\"imu_latest_seen_ns\":" << o.selected.imu_latest_seen_ns
      << ",\"imu_latest_receipt_ns\":" << o.selected.imu_latest_receipt_ns
      << ",\"imu_invalid_ns\":" << o.selected.imu_invalid_ns << ",\"imu_rejection_reason\":";
  Str(out,o.selected.imu_rejection_reason);out << '}';
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
      out << ",\"tunnel\":";Tunnel(out,r.tunnel);
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
  if (o.fusion_ran && o.fusion.diagnostics_enabled) {
    out << ",\"fusion\":"; Fusion(out, o.fusion, true);
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
