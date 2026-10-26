#include <asmc_lidar/tracking/track.hpp>
// Typed local (base_link) / map LidarObjectArray builder.
// Ported from Main src/Geometry/FrameBoundaryPublisher.cpp (8f93c22e) without the
// ROS transport: publishers, TF broadcast, the 1 Hz diagnostics timer and the
// subscriber-count shortcuts belong to the node.
#include <asmc_lidar/ros/runtime.hpp>

#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace asmc {
namespace lidar {
namespace runtime {
namespace {

using Array = asmc_msgs::LidarObjectArray;

ros::Time StampOf(std::int64_t stamp_ns) {
  return ros::Time().fromNSec(static_cast<std::uint64_t>(stamp_ns));
}

// Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:9-11 (8f93c22e).
void Point(const asmc::lidar::Vec& source, geometry_msgs::Point* target) {
  target->x = source.x(); target->y = source.y(); target->z = source.z();
}

// Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:12-29 (8f93c22e).
asmc_msgs::LidarObject Message(const pipeline::TrackedObject& original,
                               const asmc::lidar::Geometry& geometry,
                               const asmc::lidar::Rot& output_from_level,
                               bool map_valid) {
  asmc_msgs::LidarObject out;
  out.track_id = original.tracking.s32_Id; out.detection_index = original.detection.u16_Idx;
  out.observation_state = out.OBSERVED;
  out.age = std::max(0, original.tracking.s32_Age); out.miss_count = 0;
  out.classification = original.camera.str_ObjectClass; out.camera_class = original.camera.str_YoloClass;
  Point(geometry.center, &out.center);
  for (unsigned i = 0; i < 8; ++i) Point(geometry.corners[i], &out.obb_corners[i]);
  out.yaw = geometry.yaw.value_or(0.);
  out.length = original.detection.obb_length_m; out.width = original.detection.obb_width_m;
  out.height = original.detection.f32_OBB_ZMax - original.detection.f32_OBB_ZMin;  // float subtraction, as Main
  out.source_point_count = original.detection.pcl_ClusterPoints.size();
  // Port Main PlannerObjectAdapter's *applied* velocity, not its raw KF
  // derivative. LOW_THREAT zeros and UNKNOWN warmup estimates are the existing
  // MotionClassifier policy. Valid means a finite source-time estimate exists,
  // never that the estimate has independent-GT accuracy or confirmed motion.
  const auto& m = original.tracking.motion;
  const Vec applied = output_from_level * Vec(m.f32_PlannerVxKPH / 3.6,
                                              m.f32_PlannerVyKPH / 3.6, 0.);
  out.relative_velocity_valid = false;  // moving-frame derivative is not certified
  out.absolute_velocity_valid = map_valid && m.s32_Id == out.track_id &&
      m.s32_MotionValidObservations > 0 && applied.allFinite() &&
      std::isfinite(original.tracking.f32_AbsVx) && std::isfinite(original.tracking.f32_AbsVy) &&
      m.s32_MotionState >= 0 && m.s32_MotionState <= 3;
  if (out.absolute_velocity_valid) {
    out.absolute_velocity_mps.x = applied.x();
    out.absolute_velocity_mps.y = applied.y();
    out.absolute_velocity_mps.z = applied.z();
  }
  out.dynamic_valid = out.absolute_velocity_valid && m.motion_valid;
  out.is_dynamic = out.dynamic_valid && m.b_IsDynamic;
  return out;
}

asmc_msgs::LidarTrackingState Tracking(const pipeline::TrackedObject& object,
                                      const Pose& output_from_level, bool motion_valid) {
  asmc_msgs::LidarTrackingState out;
  out.track_id = object.tracking.s32_Id;
  const auto& m = object.tracking.motion;
  out.motion_state = motion_valid && m.s32_Id == out.track_id && m.motion_valid &&
      m.s32_MotionState >= 1 && m.s32_MotionState <= 3 ? m.s32_MotionState : out.UNKNOWN;
  Point(output_from_level * Vec(object.detection.f32_CoreX,
                               object.detection.f32_CoreY, object.detection.f32_CoreZ), &out.cluster_center);
  return out;
}

// Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:30-34 (8f93c22e).
// The provenance is the constructor value (Main hard-codes "bridge_receive_ns").
Array Empty(const ros::Time& stamp, const char* frame, std::uint8_t status,
            const std::string& reason, const std::string& provenance) {
  Array out; out.header.stamp = stamp; out.header.frame_id = frame;
  out.status = status; out.reason = reason; out.stamp_provenance = provenance;
  return out;
}

}  // namespace

TypedOutput::TypedOutput(std::string stamp_provenance) : provenance_(std::move(stamp_provenance)) {}

// Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:57-62 (8f93c22e).
// Compared as int64 ns; a negative stamp (unreachable from a ROS header) is a
// stamp error here instead of ros::Time's range exception. last_received_
// (diagnostic silence clock) is the node's.
bool TypedOutput::Accept(std::int64_t stamp_ns) {
  if (stamp_ns <= 0 || (last_stamp_ns_ != 0 && stamp_ns <= last_stamp_ns_)) {
    ++stamp_errors_; return false;
  }
  last_stamp_ns_ = stamp_ns; return true;
}

// Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:63-69 (8f93c22e).
// Both arrays are always built (Main serializes each only for subscribers).
TypedOutput::Arrays TypedOutput::Invalid(std::int64_t stamp_ns, const std::string& reason) {
  Arrays out;
  if (!Accept(stamp_ns)) return out;
  input_reason_ = reason; map_reason_ = "input_invalid"; ++invalid_;
  const auto stamp = StampOf(stamp_ns);
  out.accepted = true;
  out.local = Empty(stamp, "base_link", Array::INPUT_INVALID, reason, provenance_);
  out.map = Empty(stamp, "map", Array::INPUT_INVALID, reason, provenance_);
  return out;
}

// Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:70-128 (8f93c22e).
// Main computes map geometry only when /objects_map has subscribers (it forces
// map_pose.valid=false otherwise) and fills each array only for subscribers;
// here both arrays are always computed (local_needed = map_needed = true).
TypedOutput::Arrays TypedOutput::Observed(const pipeline::ScanResult& result,
                                          const std::vector<pipeline::TrackedObject>& objects) {
  const auto& context = result.context;
  const std::int64_t stamp_ns = context.time.stamp_ns;
  const auto time_error = asmc::lidar::validate_time(context.time);
  if (!time_error.empty()) return Invalid(stamp_ns, time_error);
  const auto base_from_level = asmc::lidar::base_from_input(context);
  if (!base_from_level) return Invalid(stamp_ns, "leveling_transform_invalid");
  Arrays out;
  if (!Accept(stamp_ns)) return out;
  out.accepted = true; out.observed = true; out.base_from_level = *base_from_level;
  ++processed_; input_reason_ = "valid"; map_reason_ = context.map_pose.reason;
  const auto stamp = StampOf(stamp_ns);
  out.local = Empty(stamp, "base_link", Array::OBSERVED_EMPTY, "observed_empty", provenance_);
  out.map = Empty(stamp, "map", context.map_pose.valid ? Array::OBSERVED_EMPTY : Array::POSE_UNAVAILABLE,
                  context.map_pose.valid ? "observed_empty" : context.map_pose.reason, provenance_);
  out.local.stamp_provenance = out.map.stamp_provenance = context.time.stamp_provenance;
  for (const auto& original : objects) {
    if (original.tracking.s32_Id < 0 || original.tracking.s32_MissCount != 0 || original.detection.pcl_ClusterPoints.empty() ||
        original.camera.str_ObjectClass == "noise" || original.camera.str_ObjectClass == "지면노이즈") continue;
    asmc::lidar::Object object; object.id = original.tracking.s32_Id; object.classification = original.camera.str_ObjectClass;
    object.center = asmc::lidar::Vec(original.detection.f32_OBB_CenterX, original.detection.f32_OBB_CenterY, original.detection.f32_OBB_CenterZ);
    // std::cos/std::sin of the float yaw (float overloads), as Main.
    object.long_axis = asmc::lidar::Vec(std::cos(original.detection.f32_OBB_YawRad), std::sin(original.detection.f32_OBB_YawRad), 0);
    for (unsigned i = 0; i < 8; ++i) {
      const auto& p = original.detection.obb_corners_level[i]; object.corners[i] = asmc::lidar::Vec(p.x, p.y, p.z);
    }
    auto transformed = asmc::lidar::transform(object, context);
    const double dimensions[] = {original.detection.obb_length_m, original.detection.obb_width_m,
                                 original.detection.f32_OBB_ZMax - original.detection.f32_OBB_ZMin};
    for (double dimension : dimensions) if (!std::isfinite(dimension) || dimension < 0) {
      transformed.valid = false; transformed.reason = "invalid_object_dimensions";
    }
    if (!std::isfinite(original.detection.f32_CoreX) || !std::isfinite(original.detection.f32_CoreY) ||
        !std::isfinite(original.detection.f32_CoreZ)) {
      transformed.valid = false; transformed.reason = "invalid_tracking_center";
    }
    if (!transformed.valid) {
      out.local.status = Array::INPUT_INVALID; out.local.reason = transformed.reason; out.local.objects.clear();
      out.map.status = Array::INPUT_INVALID; out.map.reason = transformed.reason; out.map.objects.clear();
      out.local_tracking.clear(); out.map_tracking.clear();
      input_reason_ = transformed.reason; ++invalid_; break;
    }
    out.local.status = Array::OBSERVED; out.local.reason = "observed";
    out.local.objects.push_back(Message(original, transformed.base, base_from_level->linear(),
                                        context.map_pose.valid));
    out.local_tracking.push_back(Tracking(original, *base_from_level, out.local.objects.back().dynamic_valid));
    if (transformed.map) {
      out.map.status = Array::OBSERVED; out.map.reason = context.map_pose.reason;
      const Pose map_from_level = context.map_pose.map_from_base * *base_from_level;
      out.map.objects.push_back(Message(original, *transformed.map, map_from_level.linear(), true));
      out.map_tracking.push_back(Tracking(original, map_from_level, out.map.objects.back().dynamic_valid));
    }
  }
  return out;
}

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
