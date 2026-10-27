#pragma once
// Header-only front-camera fusion policy, ported from Main at 8f93c22e:
//   fusion_sync    <- SensorFusion/FusionTimeMatcher.hpp (time matching)
//   fusion_class   <- SensorFusion/FusionClassPolicy.hpp (class state machine)
//   fusion_state   <- SensorFusion/FusionClassState.hpp (class field propagation)
//   fusion_adapter <- scripts/camera_objects_yolo_bbox_adapter.py convert() box
//                     filter + Parser.cpp YoloCallback BBox mapping
// Main's function-static fusion_class::MissionProfile() is an explicit
// `mission_profile` argument (FusionConfig::mission_profile).

#include <asmc_lidar/fusion/camera_observations.hpp>
#include <asmc_lidar/tracking/track.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

namespace fusion_sync {

// Frames are any type with `std::int64_t stamp_ns` and a `boxes` container
// (Main: ros::Time stamp, toNSec() is uint64).

// A newly admitted normal empty result invalidates older nonempty evidence,
// including while an older LiDAR snapshot is still waiting for association.
// Ported from Main FusionTimeMatcher.hpp:13-19 (8f93c22e)
template <class Frames>
std::uint64_t LatestEmptyStamp(const Frames& frames) {
  std::uint64_t barrier = 0;
  for (const auto& frame : frames)
    if (frame.boxes.empty()) barrier = std::max(barrier, static_cast<std::uint64_t>(frame.stamp_ns));
  return barrier;
}

// Empty observations are valid geometry/time matches too. Suppressing every
// older stamp starves a waiting LiDAR snapshot in empty scenes.
// Ported from Main FusionTimeMatcher.hpp:21-27 (8f93c22e)
template <class Frame>
std::int64_t EligibleObservationStamp(const Frame& frame, std::uint64_t empty_barrier) {
  if (!frame.boxes.empty() && static_cast<std::uint64_t>(frame.stamp_ns) < empty_barrier) return 0;
  return static_cast<std::int64_t>(frame.stamp_ns);
}

// Returns whether the frame had been used before. unique erases it.
// Ported from Main FusionTimeMatcher.hpp:29-35 (8f93c22e)
template <class Frames>
bool ConsumeMatchedFrame(Frames& frames, std::size_t index, bool reusable) {
  const bool used_before = frames.at(index).consumed;
  if (reusable) frames.at(index).consumed = true;
  else frames.erase(frames.begin() + index);
  return used_before;
}

// Ported from Main FusionTimeMatcher.hpp:37-47 (8f93c22e)
enum class MatchAction {
  Wait,
  Match,
  Drop,
};

struct MatchDecision {
  MatchAction action = MatchAction::Wait;
  std::size_t yolo_index = std::numeric_limits<std::size_t>::max();
  std::int64_t offset_ns = std::numeric_limits<std::int64_t>::max();
};

// Decide only after a frame at/after the LiDAR stamp has arrived. At that
// point no later frame can be closer than the first future frame. The timeout
// prevents a stopped camera stream from retaining a LiDAR snapshot forever; it
// does not relax max_offset_ns. Ties go to the lower index (strict <).
// Ported from Main FusionTimeMatcher.hpp:49-85 (8f93c22e)
inline MatchDecision DecideMatch(std::int64_t lidar_stamp_ns,
                                 const std::vector<std::int64_t>& yolo_stamps_ns,
                                 std::int64_t waited_ns, std::int64_t max_offset_ns,
                                 std::int64_t wait_timeout_ns) {
  MatchDecision decision;
  bool has_future_frame = false;

  for (std::size_t index = 0; index < yolo_stamps_ns.size(); ++index) {
    const std::int64_t stamp_ns = yolo_stamps_ns[index];
    if (stamp_ns <= 0) continue;  // An invalidated observation is never a candidate.
    if (stamp_ns >= lidar_stamp_ns) has_future_frame = true;
    const std::int64_t offset_ns =
        stamp_ns >= lidar_stamp_ns ? stamp_ns - lidar_stamp_ns : lidar_stamp_ns - stamp_ns;
    if (offset_ns < decision.offset_ns) {
      decision.offset_ns = offset_ns;
      decision.yolo_index = index;
    }
  }

  const bool timed_out = waited_ns >= wait_timeout_ns;
  if (!has_future_frame && !timed_out) return decision;

  if (decision.yolo_index != std::numeric_limits<std::size_t>::max() &&
      decision.offset_ns <= max_offset_ns) {
    decision.action = MatchAction::Match;
  } else {
    decision.action = MatchAction::Drop;
  }
  return decision;
}

}  // namespace fusion_sync

namespace fusion_class {

// Ported from Main FusionClassPolicy.hpp:12-22 (8f93c22e)
inline std::string MissionGroup(const std::string& label) {
  if (label == "vehicle" || label == "dynamic_obstacle" || label == "static_obstacle") return label;
  if (label == "car" || label == "truck" || label == "bus") return "vehicle";
  if (label == "person" || label == "dog" || label == "deer" || label == "bicycle" ||
      label == "stroller")
    return "dynamic_obstacle";
  if (label == "motorcycle" || label == "bollard" || label == "box" || label == "concrete_barrier" ||
      label == "steel_barricade" || label == "traffic_drum" || label == "plastic_barrier" ||
      label == "traffic_cone")
    return "static_obstacle";
  return "unknown";
}

// Ported from Main FusionClassPolicy.hpp:25-35 (8f93c22e)
inline std::string Normalize(std::string label, bool mission_profile) {
  std::transform(label.begin(), label.end(), label.begin(),
                 [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  if (mission_profile) return MissionGroup(label);
  if (label == "person" || label == "dog" || label == "bicycle" || label == "motorcycle") return label;
  if (label == "car" || label == "truck" || label == "bus") return "car";
  return "unknown";
}

// Ported from Main FusionClassPolicy.hpp:37-41 (8f93c22e)
inline bool IsConfirmed(const std::string& label, bool mission_profile) {
  if (mission_profile)
    return label == "vehicle" || label == "dynamic_obstacle" || label == "static_obstacle";
  return label == "car" || label == "person" || label == "dog" || label == "bicycle" ||
         label == "motorcycle";
}

// Detector label lowercased without grouping (keeps truck/bus).
// Ported from Main FusionClassPolicy.hpp:43-50 (8f93c22e)
inline std::string RawLabel(std::string label) {
  std::transform(label.begin(), label.end(), label.begin(),
                 [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  return label.empty() ? "unknown" : label;
}

// Protection may use only the latest matched detector class, never the motion
// classifier, latched class or a previous unconfirmed class. Timing/epoch/age
// validation remains the caller's separate gate.
inline std::string ObservedMissionGroup(const CameraClassEvidence& evidence) {
  const auto group = Normalize(evidence.observed_class, true);
  return group != "unknown" && group == Normalize(evidence.observed_raw_class, true)
             ? group : "unknown";
}

// Ported from Main FusionCamera.cpp:28-32 (8f93c22e)
inline bool IsGroundNoise(const TrackedObject& object) {
  return object.camera.str_ObjectClass == "noise" || object.camera.str_ObjectClass == "지면노이즈";
}

// A repeated image is not another independent confirmation/miss.
// Ported from Main FusionClassPolicy.hpp:52-61 (8f93c22e)
inline void InvalidateObservation(TrackedObject& object, uint64 epoch) {
  if (object.camera.str_ObjectClass == "noise" || object.camera.str_ObjectClass == "지면노이즈") return;
  object.camera.str_ObjectClass = object.camera.str_YoloClass = "unknown";
  object.camera.last_seen_class = "unknown";
  object.camera.continuity_counter = object.camera.class_miss_counter = 0;
  object.camera.is_class_locked = false;
  object.camera.camera_observation_ns = 0;
  object.camera.camera_observation_epoch = epoch;
  object.camera.observed_class = object.camera.observed_raw_class = "unknown";
}

// false when this epoch already applied an image at or after stamp_ns.
// Ported from Main FusionClassPolicy.hpp:63-74 (8f93c22e)
inline bool BeginObservation(TrackedObject& object, uint64 stamp_ns, uint64 epoch) {
  if (object.camera.camera_observation_epoch == epoch && stamp_ns <= object.camera.camera_observation_ns)
    return false;
  if (object.camera.camera_observation_epoch != epoch) {
    object.camera.last_seen_class = "unknown";
    object.camera.continuity_counter = 0;
    object.camera.class_miss_counter = 0;
  }
  object.camera.camera_observation_epoch = epoch;
  object.camera.camera_observation_ns = stamp_ns;
  object.camera.observed_class = object.camera.observed_raw_class = "unknown";
  return true;
}

// Ported from Main FusionClassPolicy.hpp:76-116 (8f93c22e)
inline void Observe(TrackedObject& object, const std::string& label, const std::string& yolo_label,
                    int initial_confirmations, int change_confirmations, bool mission_profile) {
  const std::string observed = Normalize(label, mission_profile);
  object.camera.observed_class = observed;
  object.camera.observed_raw_class = RawLabel(yolo_label);
  object.camera.class_miss_counter = 0;
  object.camera.is_class_locked = false;

  if (observed == "unknown") {
    if (!IsConfirmed(object.camera.str_ObjectClass, mission_profile)) {
      object.camera.str_ObjectClass = "unknown";
      object.camera.str_YoloClass = "unknown";
    }
    object.camera.last_seen_class = "unknown";
    object.camera.continuity_counter = 0;
    return;
  }
  if (object.camera.str_ObjectClass == observed) {
    // Same class: still refresh the raw label from the latest observation.
    object.camera.str_YoloClass = RawLabel(yolo_label);
    object.camera.last_seen_class = "unknown";
    object.camera.continuity_counter = 0;
    return;
  }
  if (object.camera.last_seen_class == observed) {
    ++object.camera.continuity_counter;
  } else {
    object.camera.last_seen_class = observed;
    object.camera.continuity_counter = 1;
  }

  const int required = IsConfirmed(object.camera.str_ObjectClass, mission_profile)
                           ? std::max(1, change_confirmations)
                           : std::max(1, initial_confirmations);
  if (object.camera.continuity_counter >= required) {
    object.camera.str_ObjectClass = observed;
    object.camera.str_YoloClass = RawLabel(yolo_label);
    object.camera.last_seen_class = "unknown";
    object.camera.continuity_counter = 0;
  }
}

// Ported from Main FusionClassPolicy.hpp:118-127 (8f93c22e)
inline void Miss(TrackedObject& object, int miss_grace) {
  object.camera.observed_class = object.camera.observed_raw_class = "unknown";
  ++object.camera.class_miss_counter;
  object.camera.last_seen_class = "unknown";
  object.camera.continuity_counter = 0;
  object.camera.is_class_locked = false;
  if (object.camera.class_miss_counter > std::max(0, miss_grace)) {
    object.camera.str_ObjectClass = "unknown";
    object.camera.str_YoloClass = "unknown";
  }
}

}  // namespace fusion_class

// Keep tracking geometry/velocity out of classification propagation. `unlock`
// keeps Main's distinction between fusion copies (always unlocked) and
// persisting/carrying history (copy the flag).
namespace fusion_state {

// Ported from Main FusionClassState.hpp:11-21 (8f93c22e)
inline void Copy(const TrackedObject& source, TrackedObject& target, bool unlock = false) {
  target.camera.str_ObjectClass = source.camera.str_ObjectClass;
  target.camera.str_YoloClass = source.camera.str_YoloClass;
  target.camera.last_seen_class = source.camera.last_seen_class;
  target.camera.continuity_counter = source.camera.continuity_counter;
  target.camera.class_miss_counter = source.camera.class_miss_counter;
  target.camera.camera_observation_ns = source.camera.camera_observation_ns;
  target.camera.camera_observation_epoch = source.camera.camera_observation_epoch;
  target.camera.observed_class = source.camera.observed_class;
  target.camera.observed_raw_class = source.camera.observed_raw_class;
  target.camera.is_class_locked = unlock ? false : source.camera.is_class_locked;
}

// Ported from Main FusionClassState.hpp:23-27 (8f93c22e)
inline TrackedObject* Find(std::vector<TrackedObject>& objects, int32 id) {
  auto it = std::find_if(objects.begin(), objects.end(),
                         [id](const TrackedObject& object) { return object.tracking.s32_Id == id; });
  return it == objects.end() ? nullptr : &*it;
}

// Ported from Main FusionClassState.hpp:29-33 (8f93c22e)
inline void Persist(const std::vector<TrackedObject>& current, std::vector<TrackedObject>& history) {
  for (const auto& object : current)
    if (auto* previous = Find(history, object.tracking.s32_Id)) Copy(object, *previous);
}

}  // namespace fusion_state

namespace fusion_adapter {

// Main BBox (Global.hpp:120-128) as filled by YoloCallback (Parser.cpp:213-242)
// from the adapter's Yolo_bbox: Min = (xMin, yMin) top-left, Max = (xMax, yMax).
struct YoloBox {
  float32 min_x = 0.f, min_y = 0.f, max_x = 0.f, max_y = 0.f;
  std::string str_class;      // source_class_name; normalized when matched
  std::string str_raw_class;  // source_class_name as published (original case)
  int source_index = -1;  // Original observation index, before adapter/hood filtering.
};

// asmc_camera/detection/mission.py CLASSES (adapter mission16_v1 default).
inline const std::vector<std::string>& MissionNames() {
  static const std::vector<std::string> names{
      "person", "dog", "deer", "bicycle", "car", "motorcycle", "bus", "truck", "bollard", "box",
      "concrete_barrier", "steel_barricade", "traffic_drum", "plastic_barrier", "traffic_cone",
      "stroller"};
  return names;
}

// Adapter ALLOWED_NAMES (legacy profile default).
inline const std::vector<std::string>& LegacyNames() {
  static const std::vector<std::string> names{"person", "dog", "bicycle", "car", "motorcycle",
                                              "bus", "truck"};
  return names;
}

// Python str.lower() for ASCII; non-ASCII bytes are kept (Python folds a few
// non-ASCII letters, e.g. U+212A, to ASCII; not reproduced).
inline std::string LowerAscii(std::string value) {
  for (auto& c : value)
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  return value;
}

// Python str.strip() for ASCII whitespace.
inline std::string StripAscii(const std::string& value) {
  const char* space = " \t\n\r\v\f";
  const auto first = value.find_first_not_of(space);
  if (first == std::string::npos) return {};
  return value.substr(first, value.find_last_not_of(space) - first + 1);
}

// Python int(x) of a finite float written back to a float32 field: truncation
// toward zero, -0.0 becomes +0.0 (int has no signed zero).
inline float32 TruncateTowardZero(float value) {
  const double truncated = std::trunc(static_cast<double>(value));
  return static_cast<float32>(truncated == 0.0 ? 0.0 : truncated);
}

// Adapter validate_threshold(): finite and in [0, 1].
inline bool ValidThreshold(double threshold) {
  return std::isfinite(threshold) && threshold >= 0. && threshold <= 1.;
}

// Per-box filter and coordinate truncation of the adapter, in box order. The
// whole-frame drop (valid, frame_id, zero stamp/size) is enforced by the
// receiver and FrontCameraFusion. conf_thres must satisfy ValidThreshold().
// Ported from Main scripts/camera_objects_yolo_bbox_adapter.py:35-50 (8f93c22e)
inline std::vector<YoloBox> ConvertBoxes(const std::vector<camera::Box>& boxes,
                                         const std::vector<std::string>& allowed_names,
                                         double conf_thres) {
  std::vector<std::string> allowed;
  allowed.reserve(allowed_names.size());
  for (const auto& name : allowed_names) allowed.push_back(LowerAscii(StripAscii(name)));
  std::vector<YoloBox> output;
  for (std::size_t source_index = 0; source_index < boxes.size(); ++source_index) {
    const auto& box = boxes[source_index];
    const double confidence = static_cast<double>(box.confidence);
    const std::string name = LowerAscii(box.source_class_name);
    if (!box.confidence_valid || !std::isfinite(confidence) || confidence < conf_thres ||
        std::find(allowed.begin(), allowed.end(), name) == allowed.end() ||
        !std::isfinite(box.xmin) || !std::isfinite(box.ymin) || !std::isfinite(box.xmax) ||
        !std::isfinite(box.ymax))
      continue;
    YoloBox out;
    out.source_index = static_cast<int>(source_index);
    out.min_x = TruncateTowardZero(box.xmin);
    out.min_y = TruncateTowardZero(box.ymin);
    out.max_x = TruncateTowardZero(box.xmax);
    out.max_y = TruncateTowardZero(box.ymax);
    // Yolo_bbox.classes = source_class_name; YoloCallback sets str_class = str_raw_class = classes.
    out.str_class = box.source_class_name;
    out.str_raw_class = box.source_class_name;
    output.push_back(std::move(out));
  }
  return output;
}

}  // namespace fusion_adapter

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
