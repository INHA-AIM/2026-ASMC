// Front-camera class fusion (Main runCameraFusion / LidarToImage / IOUFusion),
// common-geometry branch only. See front_camera_fusion.hpp.

#include <asmc_lidar/pipeline/front_camera_fusion.hpp>
#include <asmc_lidar/projection.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace fusion {

namespace {

using fusion_adapter::YoloBox;

// Main TimedYoloFrame (FusionCamera.cpp:46-54) after admission. camera_id and
// calibration_id were trace-only in Main; the identity replaces them.
struct TimedYoloFrame {
  std::int64_t stamp_ns = 0;
  std::vector<YoloBox> boxes;
  std::string identity;
  std::uint64_t epoch = 0;
  bool consumed = false;
  const CameraFrame* source = nullptr;  // into this call's snapshot
};

// ros::Duration(double) in ns, as Main builds it for stamp arithmetic
// (rostime DurationBase::fromSec: floor seconds + rounded nanoseconds).
std::int64_t RosDurationNs(double seconds) {
  const std::int64_t sec = static_cast<std::int64_t>(std::floor(seconds));
  const std::int64_t nsec = static_cast<std::int32_t>(std::round((seconds - static_cast<double>(sec)) * 1e9));
  return sec * 1000000000LL + nsec;
}

// Main's static_cast<int>(double) with x86 cvttsd2si semantics for values
// outside int (and NaN): INT_MIN, which the image bounds check rejects.
int TruncateToInt(double value) {
  if (value > -2147483649.0 && value < 2147483648.0) return static_cast<int>(value);
  return std::numeric_limits<int>::min();
}

// Takes cv::Rect r.y and r.height (the only fields Main's band test reads).
// Ported from Main FusionCamera.cpp:266-275 (8f93c22e)
inline bool overlapsYBand(int r_y, int r_height, int ymin_px, int ymax_px, int H) {
  if (ymin_px < 0 || ymax_px <= ymin_px) return false;
  ymin_px = std::max(0, std::min(ymin_px, H));
  ymax_px = std::max(0, std::min(ymax_px, H));
  int top = r_y;
  int bot = r_y + r_height;
  int overlap = std::max(0, std::min(bot, ymax_px) - std::max(top, ymin_px));
  return overlap > 0;
}

// YOLO part only (no JPEG history). The receiver re-returns retained frames on
// every Read, so the cut is also kept as a watermark for later admissions.
// Ported from Main FusionCamera.cpp:106-128 (8f93c22e)
void PruneInputsBefore(std::vector<TimedYoloFrame>& yolo_frames, std::int64_t processed_lidar_stamp_ns,
                       double max_time_offset_sec, std::int64_t* watermark_ns) {
  const std::int64_t oldest_useful_yolo = processed_lidar_stamp_ns - RosDurationNs(max_time_offset_sec);
  while (!yolo_frames.empty() && yolo_frames.front().stamp_ns < oldest_useful_yolo)
    yolo_frames.erase(yolo_frames.begin());
  *watermark_ns = std::max(*watermark_ns, oldest_useful_yolo);
}

}  // namespace

// Ported from Main FusionCamera.cpp:282-349 (8f93c22e)
bool LidarToImage(const std::vector<TrackedObject>& objects, const asmc::lidar::Context& context,
                  const camera::Calibration& camera, const FusionConfig& config,
                  std::vector<LidarImageBox>* boxes) {
  boxes->clear();

  // Legacy fixed raw-sensor matrix (enable_common_geometry=false) not ported.
  // Main: camera_mount = bird_eye_view_transform^-1, K = intrinsic.leftCols<3>().
  const auto p = asmc::lidar::camera_projection(context, camera.base_from_camera, camera.intrinsic);
  if (!p) return false;  // Main: ROS_ERROR_THROTTLE "Invalid scan projection context", no boxes
  Eigen::Matrix<double, 3, 4> projection = *p;

  // Optional legacy hood y-band (Main ~exclude_ymin_px/~exclude_ymax_px/
  // ~fusion_exclude_hood_y_band, default off).
  const int ex_ymin = config.exclude_ymin_px, ex_ymax = config.exclude_ymax_px;
  const bool exclude_hood_y_band = config.exclude_hood_y_band;
  // Main: decoded st_Camera.bgr_image cols/rows; here the calibration size.
  const int cols = static_cast<int>(camera.width);
  const int rows = static_cast<int>(camera.height);

  for (std::size_t i = 0; i < objects.size(); ++i) {
    const auto& det = objects[i];
    // Do not associate a detection with a prediction-only ghost: its stored
    // point cloud is from an older LiDAR frame.
    if (det.s32_MissCount > 0) continue;
    if (fusion_class::IsGroundNoise(det)) continue;
    // cv::boundingRect over the kept integer points.
    bool any_point = false;
    int min_u = 0, min_v = 0, max_u = 0, max_v = 0;

    for (const auto& pt : det.pcl_ClusterPoints.points) {
      Eigen::Vector4d pt_vec(pt.x, pt.y, pt.z, 1.0);
      Eigen::Vector3d cam = projection * pt_vec;

      if (cam[2] <= 0.0) continue;  // behind the camera

      int u = TruncateToInt(cam[0] / cam[2]);
      int v = TruncateToInt(cam[1] / cam[2]);

      if (u >= 0 && u < cols && v >= 0 && v < rows) {
        if (!any_point) {
          min_u = max_u = u;
          min_v = max_v = v;
          any_point = true;
        } else {
          min_u = std::min(min_u, u);
          max_u = std::max(max_u, u);
          min_v = std::min(min_v, v);
          max_v = std::max(max_v, v);
        }
      }
    }

    if (any_point) {
      // cv::Rect(min_u, min_v, max_u - min_u + 1, max_v - min_v + 1); br() is exclusive.
      const int rect_x = min_u, rect_y = min_v;
      const int rect_width = max_u - min_u + 1, rect_height = max_v - min_v + 1;

      if (exclude_hood_y_band && overlapsYBand(rect_y, rect_height, ex_ymin, ex_ymax, rows)) continue;

      LidarImageBox st_BBox;
      st_BBox.min_x = static_cast<float32>(rect_x);
      st_BBox.min_y = static_cast<float32>(rect_y);
      st_BBox.max_x = static_cast<float32>(rect_x + rect_width);
      st_BBox.max_y = static_cast<float32>(rect_y + rect_height);
      st_BBox.c_idx = static_cast<int>(i);
      boxes->push_back(st_BBox);
    }
  }
  return true;
}

// is_new_yolo_frame is always true at Main's only call site (:995) and is
// dropped; vec_FusionIdx (debug image only) is not kept.
// Ported from Main FusionCamera.cpp:357-493 (8f93c22e)
void IOUFusion(std::vector<TrackedObject>& objects, const std::vector<YoloBox>& detected,
               const std::vector<LidarImageBox>& lidar_boxes, std::uint64_t image_stamp_ns,
               std::uint64_t epoch, const FusionConfig& config, FusionCounters* counters) {
  const float32 fusion_x_offset = 0;

  // Main loads these once and clamps them (:366-384).
  const int INITIAL_CONFIRMATIONS = std::max(1, config.initial_confirmations);
  const int CHANGE_CONFIRMATIONS = std::max(1, config.change_confirmations);
  const int CLASS_MISS_GRACE = std::max(0, config.class_miss_grace);
  const double FUSION_IOU_THRESHOLD = config.iou_threshold;

  struct MatchCandidate {
    double iou;
    std::size_t camera_idx;
    std::size_t lidar_bbox_idx;
  };
  std::vector<MatchCandidate> candidates;

  // Build all valid pairs first, then greedily select highest-IoU one-to-one
  // matches, so one cluster cannot gain several confirmations from duplicates.
  for (std::size_t i = 0; i < detected.size(); ++i) {
    const auto& cam = detected[i];
    for (std::size_t j = 0; j < lidar_boxes.size(); ++j) {
      const auto& lid_bbox = lidar_boxes[j];

      float32 center_x = (lid_bbox.min_x + lid_bbox.max_x) / 2.0f;
      float32 center_y = (lid_bbox.min_y + lid_bbox.max_y) / 2.0f;
      float32 width = (lid_bbox.max_x - lid_bbox.min_x) * 1.3f;
      float32 height = (lid_bbox.max_y - lid_bbox.min_y) * 1.5f;

      LidarImageBox scaled_lid_box;
      scaled_lid_box.min_x = center_x - width / 2.0f + fusion_x_offset;
      scaled_lid_box.min_y = center_y - height / 2.0f;
      scaled_lid_box.max_x = center_x + width / 2.0f + fusion_x_offset;
      scaled_lid_box.max_y = center_y + height / 2.0f;

      const float32 x_left = std::max(cam.min_x, scaled_lid_box.min_x);
      const float32 y_top = std::max(cam.min_y, scaled_lid_box.min_y);
      const float32 x_right = std::min(cam.max_x, scaled_lid_box.max_x);
      const float32 y_bottom = std::min(cam.max_y, scaled_lid_box.max_y);
      const float32 inter_area = std::max(0.0f, x_right - x_left) * std::max(0.0f, y_bottom - y_top);
      const float32 cam_area =
          std::max(0.0f, cam.max_x - cam.min_x) * std::max(0.0f, cam.max_y - cam.min_y);
      const float32 lid_area = std::max(0.0f, scaled_lid_box.max_x - scaled_lid_box.min_x) *
                               std::max(0.0f, scaled_lid_box.max_y - scaled_lid_box.min_y);
      const float32 union_area = cam_area + lid_area - inter_area;
      const double iou = union_area > 1e-6f ? inter_area / union_area : 0.0;
      if (iou > FUSION_IOU_THRESHOLD) {
        candidates.push_back({iou, i, j});
      }
    }
  }

  // std::sort (not stable), as Main: equal-IoU order is the library's.
  std::sort(candidates.begin(), candidates.end(),
            [](const MatchCandidate& a, const MatchCandidate& b) { return a.iou > b.iou; });

  std::unordered_set<std::size_t> matched_lidar_indices_this_frame;
  std::unordered_set<std::size_t> matched_camera_indices;

  for (const auto& candidate : candidates) {
    const auto& lid_bbox = lidar_boxes[candidate.lidar_bbox_idx];
    const std::size_t lidar_idx = lid_bbox.c_idx;
    if (lidar_idx >= objects.size() || matched_camera_indices.count(candidate.camera_idx) != 0 ||
        matched_lidar_indices_this_frame.count(lidar_idx) != 0) {
      continue;
    }

    matched_camera_indices.insert(candidate.camera_idx);
    matched_lidar_indices_this_frame.insert(lidar_idx);

    auto& lid_object = objects[lidar_idx];
    const auto& cam = detected[candidate.camera_idx];

    // A false BeginObservation still counts as matched (no Miss below).
    if (fusion_class::BeginObservation(lid_object, image_stamp_ns, epoch)) {
      const std::string before = lid_object.str_ObjectClass;
      fusion_class::Observe(lid_object, cam.str_class, cam.str_raw_class, INITIAL_CONFIRMATIONS,
                            CHANGE_CONFIRMATIONS, config.mission_profile);
      if (counters) {
        ++counters->observations;
        if (lid_object.str_ObjectClass != before &&
            fusion_class::IsConfirmed(lid_object.str_ObjectClass, config.mission_profile))
          ++counters->confirmations_applied;
      }
    }
  }

  // Unmatched observed objects (including those outside the image or behind
  // the camera) receive a miss. Prediction-only tracks keep their class.
  for (std::size_t i = 0; i < objects.size(); ++i) {
    auto& object = objects[i];
    if (object.s32_MissCount > 0 || fusion_class::IsGroundNoise(object)) continue;
    if (matched_lidar_indices_this_frame.find(i) == matched_lidar_indices_this_frame.end() &&
        fusion_class::BeginObservation(object, image_stamp_ns, epoch)) {
      fusion_class::Miss(object, CLASS_MISS_GRACE);
      if (counters) ++counters->misses_applied;
    }
  }
}

FrontCameraFusion::FrontCameraFusion(const FusionConfig& config, const camera::Calibration& front)
    : config_(config), front_(front) {
  // Main FusionCamera.cpp:784-790 (policy read once, anything else throws).
  if (config_.yolo_policy != "unique" && config_.yolo_policy != "bounded_reuse")
    throw std::runtime_error("invalid fusion_yolo_policy");
  reuse_ = config_.yolo_policy == "bounded_reuse";
  // Main FusionCamera.cpp:798-803 (clamps and ns conversion).
  config_.max_time_offset_s = std::max(0.001, config_.max_time_offset_s);
  config_.sync_wait_timeout_s = std::max(config_.max_time_offset_s, config_.sync_wait_timeout_s);
  if (!std::isfinite(config_.max_time_offset_s) || !std::isfinite(config_.sync_wait_timeout_s))
    throw std::runtime_error("fusion time limits must be finite");
  max_offset_ns_ = static_cast<std::int64_t>(std::llround(config_.max_time_offset_s * 1e9));
  wait_timeout_ns_ = static_cast<std::int64_t>(std::llround(config_.sync_wait_timeout_s * 1e9));
  // Main FusionCamera.cpp:381-383.
  config_.initial_confirmations = std::max(1, config_.initial_confirmations);
  config_.change_confirmations = std::max(1, config_.change_confirmations);
  config_.class_miss_grace = std::max(0, config_.class_miss_grace);
  // Adapter validate_threshold() and profile defaults (adapter :14-24, :60-68).
  if (!fusion_adapter::ValidThreshold(config_.confidence_threshold))
    throw std::runtime_error("adapter confidence must be finite and in [0,1]");
  allowed_ = !config_.allowed_names.empty() ? config_.allowed_names
             : config_.mission_profile      ? fusion_adapter::MissionNames()
                                            : fusion_adapter::LegacyNames();
  if (!std::isfinite(config_.camera_read_window_s) ||
      config_.camera_read_window_s < config_.max_time_offset_s)
    throw std::runtime_error("camera_read_window_s must be finite and cover max_time_offset_s");
  read_window_ns_ = static_cast<std::int64_t>(std::llround(config_.camera_read_window_s * 1e9));
  if (front_.frame_id.empty() || front_.width == 0 || front_.height == 0 ||
      front_.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      front_.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
    throw std::runtime_error("invalid front camera calibration");
}

// Ported from Main FusionCamera.cpp:684-1048 (8f93c22e)
FusionReport FrontCameraFusion::Process(std::int64_t decision_ns, std::int64_t scan_stamp_ns,
                                        const asmc::lidar::Context& context,
                                        std::vector<TrackedObject>& live,
                                        std::vector<TrackedObject>* history, CameraSource& camera,
                                        std::int64_t ros_now_ns, std::int64_t steady_now_ns) {
  FusionReport report;
  ++counters_.calls;

  // One non-consuming read per call (Main swaps the callback queues once).
  const std::int64_t reference_ns = scan_stamp_ns != 0 ? scan_stamp_ns : last_queued_stamp_ns_;
  const CameraSnapshot snapshot = camera.Read(reference_ns, read_window_ns_, ros_now_ns, steady_now_ns);
  report.camera_active = snapshot.active;
  report.camera_reason = snapshot.reason;

  // Main CameraControl and the control-stale / YOLO-silence gates
  // (FusionCamera.cpp:69-85, 712-722) mapped onto the source state: every
  // active flip and every change of the receiver epoch of the returned frames
  // (session/generation restart) starts a new consumer epoch.
  // A "missing" front view after frames were seen means the receiver cleared
  // every view (status session/generation transition or a heartbeat after a
  // closed gate): Main bumps its epoch at that moment, not at the next frame.
  const bool source_cleared = snapshot.active && snapshot.frames.empty() &&
                              snapshot.reason == "missing" && source_epoch_ != 0;
  const bool source_restarted = source_cleared ||
                                (snapshot.active && snapshot.epoch != 0 && source_epoch_ != 0 &&
                                 snapshot.epoch != source_epoch_);
  if (source_cleared) source_epoch_ = 0;
  if (snapshot.active != was_active_ || source_restarted) ++consumer_epoch_;
  if (snapshot.active && snapshot.epoch != 0) source_epoch_ = snapshot.epoch;
  was_active_ = snapshot.active;

  // Ported from Main FusionCamera.cpp:723-740 (8f93c22e)
  if (admitted_epoch_ != consumer_epoch_) {
    ++counters_.epoch_changes;
    pending_.clear();
    consumed_.clear();
    prune_before_ns_ = 0;
    received_any_frame_ = false;
    admitted_epoch_ = consumer_epoch_;
    for (auto& object : live) fusion_class::InvalidateObservation(object, consumer_epoch_);
    // A missed/reappearing track must not reacquire a class from an older
    // history snapshot. Main invalidates every Prev_Objects_Queue snapshot;
    // only back() (this `history`) is ever read again.
    if (history)
      for (auto& object : *history) fusion_class::InvalidateObservation(object, consumer_epoch_);
  }
  // Ported from Main FusionCamera.cpp:741-748 (8f93c22e)
  if (!snapshot.active) {
    ++counters_.inactive_calls;
    for (auto& object : live) fusion_class::InvalidateObservation(object, consumer_epoch_);
    if (history) fusion_state::Persist(live, *history);
    report.pending_scans = pending_.size();
    return report;
  }

  // Admission (Main FusionCamera.cpp:749-765 + BufferFusionYoloFrame :209-243).
  // The source keeps the frames, so yolo_history is rebuilt every call from
  // the returned frames minus unique-consumed and pruned ones. Boxes pass the
  // adapter filter, so a frame whose boxes are all filtered is a normal empty frame.
  std::vector<TimedYoloFrame> yolo_history;
  yolo_history.reserve(snapshot.frames.size());
  for (const auto& frame : snapshot.frames) {
    // Adapter whole-frame drop (zero stamp/size) and BufferFusionYoloFrame zero stamp.
    if (frame.stamp_ns <= 0 || frame.width <= 0 || frame.height <= 0) continue;
    if (frame.stamp_ns < prune_before_ns_) continue;
    const auto used = consumed_.find(frame.identity);
    if (used != consumed_.end() && !reuse_) continue;  // unique: Main erased it
    TimedYoloFrame admitted;
    admitted.stamp_ns = frame.stamp_ns;
    admitted.boxes = fusion_adapter::ConvertBoxes(frame.boxes, allowed_, config_.confidence_threshold);
    admitted.identity = frame.identity;
    admitted.epoch = consumer_epoch_;  // Main frame.epoch = g_CameraEpoch at admission
    admitted.consumed = used != consumed_.end();
    admitted.source = &frame;
    yolo_history.push_back(std::move(admitted));
  }
  // InsertByStamp order: ascending stamp, equal stamps keep arrival order. Ties
  // in DecideMatch therefore go to the earlier stamp (the receiver returns the
  // newer one first).
  std::stable_sort(yolo_history.begin(), yolo_history.end(),
                   [](const TimedYoloFrame& a, const TimedYoloFrame& b) { return a.stamp_ns < b.stamp_ns; });
  std::int64_t first_arriving_yolo_stamp_ns = 0;
  if (!received_any_frame_ && !yolo_history.empty()) {
    first_arriving_yolo_stamp_ns = yolo_history.front().stamp_ns;
    received_any_frame_ = true;
  }

  // Ported from Main FusionCamera.cpp:767-782 (8f93c22e)
  if (scan_stamp_ns != 0 && scan_stamp_ns != last_queued_stamp_ns_) {
    pending_.push_back(PendingScan{scan_stamp_ns, decision_ns, context, live});
    last_queued_stamp_ns_ = scan_stamp_ns;
    ++counters_.queued;
  }

  // LiDAR frames retained while the model was loading predate the first
  // possible match: discard those provably outside the matching window.
  // Ported from Main FusionCamera.cpp:805-818 (8f93c22e)
  if (first_arriving_yolo_stamp_ns != 0) {
    const std::int64_t earliest_matchable_yolo =
        first_arriving_yolo_stamp_ns - RosDurationNs(config_.max_time_offset_s);
    while (!pending_.empty() && pending_.front().stamp_ns < earliest_matchable_yolo) {
      pending_.pop_front();
      ++counters_.startup_discarded;
    }
  }

  // Until the first frame arrives keep only the newest scans.
  // Ported from Main FusionCamera.cpp:820-833 (8f93c22e)
  if (!received_any_frame_) {
    while (pending_.size() > config_.max_pending_before_first_frame) {
      pending_.pop_front();
      ++counters_.startup_discarded;
    }
    report.pending_scans = pending_.size();
    return report;
  }

  // FIFO with head-of-line blocking.
  // Ported from Main FusionCamera.cpp:835-1030 (8f93c22e)
  while (!pending_.empty()) {
    const PendingScan& lidar_frame = pending_.front();
    std::vector<std::int64_t> yolo_stamps_ns;
    yolo_stamps_ns.reserve(yolo_history.size());
    const std::uint64_t empty_barrier = reuse_ ? fusion_sync::LatestEmptyStamp(yolo_history) : 0;
    for (const auto& frame : yolo_history) {
      yolo_stamps_ns.push_back(fusion_sync::EligibleObservationStamp(frame, empty_barrier));
    }
    // Main: llround((decision_time - queued_at).toSec() * 1e9) on WallTime.
    const std::int64_t waited_ns = decision_ns - lidar_frame.queued_ns;
    const auto decision = fusion_sync::DecideMatch(lidar_frame.stamp_ns, yolo_stamps_ns, waited_ns,
                                                   max_offset_ns_, wait_timeout_ns_);

    if (decision.action == fusion_sync::MatchAction::Wait) {
      ++counters_.waits;
      break;
    }
    // Receiver contract: re-check the record right before applying it. A
    // record superseded since Read keeps the scan pending for the next call.
    if (decision.action == fusion_sync::MatchAction::Match &&
        !camera.Current(*yolo_history[decision.yolo_index].source, ros_now_ns, steady_now_ns)) {
      ++counters_.not_current;
      break;
    }

    PendingScan synchronized_lidar_frame = std::move(pending_.front());
    pending_.pop_front();

    if (decision.action == fusion_sync::MatchAction::Drop) {
      ++counters_.dropped;
      ++report.dropped_this_call;
      PruneInputsBefore(yolo_history, synchronized_lidar_frame.stamp_ns, config_.max_time_offset_s,
                        &prune_before_ns_);
      continue;
    }

    TimedYoloFrame yolo_frame = yolo_history[decision.yolo_index];
    const std::uint64_t processing_camera_epoch = yolo_frame.epoch;  // Main g_ProcessingCameraEpoch

    // Main FusionCamera.cpp:892-935 (JPEG lookup within 2 ms, decode, decode
    // drop) not ported: no image is used; the size comes from the calibration.

    const bool reused = fusion_sync::ConsumeMatchedFrame(yolo_history, decision.yolo_index, reuse_);
    consumed_[yolo_frame.identity] = yolo_frame.stamp_ns;

    // Geometry from the synchronized historical frame; class counters
    // continue from the current track.
    std::vector<TrackedObject>& synchronized_objects = synchronized_lidar_frame.objects;
    for (auto& historical_object : synchronized_objects) {
      if (TrackedObject* current = fusion_state::Find(live, historical_object.s32_Id)) {
        fusion_state::Copy(*current, historical_object, true);
      }
    }

    std::vector<YoloBox> detected_objects;
    {
      // Main: decoded image cols/rows.
      const float32 image_width = static_cast<float32>(front_.width);
      const float32 image_height = static_cast<float32>(front_.height);

      // Reject the ego hood false positive at the bottom of the front camera.
      for (const auto& box : yolo_frame.boxes) {
        const float32 width = box.max_x - box.min_x;
        const float32 center_y = (box.min_y + box.max_y) * 0.5f;
        const std::string normalized_class = fusion_class::Normalize(box.str_class, config_.mission_profile);
        const bool is_ego_hood = normalized_class == "car" && width >= image_width * 0.60f &&
                                 center_y >= image_height * 0.85f &&
                                 box.max_y >= image_height * 0.95f;
        if (is_ego_hood) continue;
        YoloBox normalized_box = box;
        normalized_box.str_class = normalized_class;
        detected_objects.push_back(std::move(normalized_box));
      }
    }

    std::vector<LidarImageBox> lidar_boxes;
    LidarToImage(synchronized_objects, synchronized_lidar_frame.context, front_, config_, &lidar_boxes);

    IOUFusion(synchronized_objects, detected_objects, lidar_boxes,
              static_cast<std::uint64_t>(yolo_frame.stamp_ns), processing_camera_epoch, config_,
              &counters_);

    for (const auto& historical_object : synchronized_objects) {
      if (TrackedObject* current = fusion_state::Find(live, historical_object.s32_Id)) {
        fusion_state::Copy(historical_object, *current, true);
      }
    }

    // The tracker committed this scan before fusion: persist only the class
    // fields into that frame (no geometry leaks into tracking history).
    if (history) fusion_state::Persist(live, *history);

    // Main PublishFusionDebugImage (:1021) not ported.

    ++counters_.matched;
    ++report.matched_this_call;
    if (reused) ++counters_.reused; else ++counters_.fresh;
    report.match_offsets_ns.push_back(decision.offset_ns);
    PruneInputsBefore(yolo_history, synchronized_lidar_frame.stamp_ns, config_.max_time_offset_s,
                      &prune_before_ns_);
  }

  // Hard bound against a stopped camera or malformed stamps.
  // Ported from Main FusionCamera.cpp:1032-1038 (8f93c22e)
  while (pending_.size() > config_.max_pending_scans) {
    pending_.pop_front();
    ++counters_.bound_discarded;
  }
  // Main :1039-1044 (yolo <= 128, JPEG <= 80): the source bounds the retained
  // frames (receiver capacity per view); yolo_history lives for one call.

  // Consumed identities are kept while the source still returns the frame or
  // its stamp is >= (oldest pending scan, else this reference) - read window.
  // Later reads use a reference >= this one (non-decreasing scan stamps), so
  // their window never returns an erased identity again.
  std::int64_t oldest_ns = reference_ns;
  if (!pending_.empty()) oldest_ns = std::min(oldest_ns, pending_.front().stamp_ns);
  const std::int64_t keep_from_ns = oldest_ns - read_window_ns_;
  std::set<std::string> retained;
  for (const auto& frame : snapshot.frames) retained.insert(frame.identity);
  for (auto it = consumed_.begin(); it != consumed_.end();) {
    if (it->second < keep_from_ns && retained.count(it->first) == 0) it = consumed_.erase(it);
    else ++it;
  }

  report.pending_scans = pending_.size();
  return report;
}

}  // namespace fusion
}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
