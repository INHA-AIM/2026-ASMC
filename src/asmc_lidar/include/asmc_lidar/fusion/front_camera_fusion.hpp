#pragma once
// Front-camera class fusion for tracked LiDAR objects.
// Ported from Main SensorFusion/FusionCamera.cpp (runCameraFusion, LidarToImage,
// IOUFusion), FusionClassPolicy.hpp, FusionClassState.hpp and
// FusionTimeMatcher.hpp at 8f93c22e, front camera only (camera_front), with the
// common-geometry projection (per-scan Context, stored leveling inverted once).
// The policy headers are ported in fusion_policy.hpp.
//
// Input is the canonical asmc_msgs/CameraObjectObservation stream through the
// asmc_lidar camera::Receiver buffer (CameraSource below), not Main's /YOLO/BBox
// adapter. Deliberate differences from Main are listed in docs/pipeline.md
// (decision clock injected instead of WallTime, receiver gate/stale reasons
// instead of WallTime silence gates, calibration image size instead of decoded
// JPEG size, no debug image).

#include <asmc_lidar/fusion/camera_observations.hpp>
#include <asmc_lidar/fusion/fusion_policy.hpp>
#include <asmc_lidar/tracking/track.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace fusion {

struct FusionConfig {
  // Bounded, observation-only diagnostics. Disabling must not change fusion.
  bool capture_diagnostics = true;
  std::string views = "all"; // three-view default; front is internal rollback, no extra launch flag
  // Main ~fusion_* parameters (main.launch values).
  double max_time_offset_s = 0.05;    // Match only within +-50 ms (inclusive)
  double sync_wait_timeout_s = 0.20;  // stop waiting for a later frame after 200 ms
  std::string yolo_policy = "unique";  // or "bounded_reuse"
  int initial_confirmations = 5;
  int change_confirmations = 3;
  int class_miss_grace = 3;
  double iou_threshold = 0.25;  // Main static double FUSION_IOU_THRESHOLD
  bool mission_profile = true;  // ~camera_class_profile == mission16_v1
  // Main adapter defaults (camera_objects_yolo_bbox_adapter.py).
  double confidence_threshold = 0.0;
  // Adapter ~allowed_names; empty = profile default (mission16_v1: asmc_camera
  // mission CLASSES, legacy: the 7 COCO names).
  std::vector<std::string> allowed_names;
  bool exclude_hood_y_band = false;
  int exclude_ymin_px = 637;
  int exclude_ymax_px = 720;
  // Main queue bounds.
  std::size_t max_pending_scans = 32;
  std::size_t max_pending_before_first_frame = 2;
  // Window used to read every retained camera frame around a pending scan
  // (must cover the receiver retention; it is not a match tolerance).
  double camera_read_window_s = 1.0;
};

// One original-resolution camera observation, shared by front/all consumers.
struct CameraFrame {
  std::int64_t stamp_ns = 0;
  std::uint64_t epoch = 0;     // receiver epoch of the record
  std::string frame_id = "camera_front";
  std::string identity;        // session|generation|frame_id|stamp|calibration_id
  int width = 0, height = 0;
  std::vector<camera::Box> boxes;
  // Receiver record (BufferCameraSource only; null for other sources).
  std::shared_ptr<const camera::Stored> record;
};

// Camera state at one decision. active=false means the camera is not usable
// (gate closed, STANDBY/PREPARE, stale, invalid, missing...).
struct CameraSnapshot {
  bool active = false;
  bool gate_active = false;
  std::string gate_reason;
  std::uint64_t gate_epoch = 0;
  std::map<std::string,std::vector<CameraFrame>> by_view;
  std::string reason;
  std::uint64_t epoch = 0;          // epoch of the returned frames; 0 when none
  std::vector<CameraFrame> frames;  // retained front frames, any order
  struct View {
    std::string frame_id, reason, session, calibration_id, model_id, config_id;
    bool matching_implemented=false;
    std::uint64_t generation = 0;
    std::uint32_t wire_sequence = 0;
    std::int64_t stamp_ns = 0, image_minus_scan_ns = 0;
    std::size_t candidates = 0;
  };
  // Receiver summaries. FrontCameraFusion marks only front as implemented;
  // MultiCameraFusion marks the independently configured views.
  std::vector<View> views;
};

class CameraSource {
 public:
  virtual ~CameraSource() = default;
  virtual CameraSnapshot Read(std::int64_t reference_stamp_ns, std::int64_t window_ns,
                              std::int64_t ros_now_ns, std::int64_t steady_now_ns) = 0;
  // Re-checked immediately before a match is applied (receiver current()).
  virtual bool Current(const CameraFrame& /*frame*/, std::int64_t /*ros_now_ns*/,
                       std::int64_t /*steady_now_ns*/) {
    return true;
  }
};

struct FusionCounters {
  std::uint64_t calls = 0, queued = 0, matched = 0, dropped = 0, waits = 0;
  std::uint64_t reused = 0, inactive_calls = 0, epoch_changes = 0;
  std::uint64_t startup_discarded = 0, bound_discarded = 0;
  std::uint64_t observations = 0, confirmations_applied = 0, misses_applied = 0;
  std::uint64_t fresh = 0;        // Main g_TotalFresh (matches of a first-use frame)
  std::uint64_t not_current = 0;  // match deferred: CameraSource::Current() was false
  std::uint64_t projection_invalid = 0;
};

struct AssociationDiagnostic {
  int track_id = -1, source_box_index = -1;
  double iou = 0.;
  bool applied = false, projected = false;
  std::string reason, raw_class, group, class_before, class_after;
  std::array<float, 4> projected_box{}, camera_box{};
};

struct MatchDiagnostic {
  std::uint64_t scan_sequence = 0;
  std::int64_t scan_stamp_ns = 0, wait_ns = 0, image_stamp_ns = 0, image_minus_scan_ns = 0;
  std::string action, reason, observation_id, camera_id, session, calibration_id, model_id, config_id;
  std::uint64_t generation = 0;
  std::uint32_t wire_sequence = 0;
  bool reused = false, projection_valid = false;
  std::size_t boxes_received = 0, boxes_admitted = 0, associations_omitted = 0;
  std::vector<AssociationDiagnostic> associations;
};

struct FusionReport {
  bool camera_active = false;
  std::string camera_reason;
  std::size_t pending_scans = 0;
  std::size_t matched_this_call = 0;
  std::size_t dropped_this_call = 0;
  // |image stamp - scan stamp| of each match in this call. No pose
  // compensation is applied for this offset (documented, not validated).
  std::vector<std::int64_t> match_offsets_ns;
  bool diagnostics_enabled = false;
  std::uint64_t emitting_scan_sequence = 0;
  std::int64_t emitting_scan_stamp_ns = 0;
  std::vector<CameraSnapshot::View> views;
  // At most 64 scan decisions and 128 object associations per decision.
  // No point clouds or receiver shared_ptr are retained by diagnostics.
  std::vector<MatchDiagnostic> decisions;
  std::size_t decisions_omitted = 0;
};

// Main BBox of a projected LiDAR object (st_Camera.fusion_data.vec_Lidar_BBoxes):
// Min = boundingRect tl(), Max = br() (exclusive corner), c_idx = object index.
struct LidarImageBox {
  float32 min_x = 0.f, min_y = 0.f, max_x = 0.f, max_y = 0.f;
  int c_idx = 0;
};

// Main LidarToImage (common-geometry branch): projects each observed object's
// cluster points with camera_projection(context, base_from_camera, intrinsic)
// and the calibration image size. Returns false (no boxes) when the context
// cannot be projected.
bool LidarToImage(const std::vector<TrackedObject>& objects, const asmc::lidar::Context& context,
                  const camera::Calibration& camera, const FusionConfig& config,
                  std::vector<LidarImageBox>* boxes);

// Same expanded rectangle arithmetic shared by legacy front and multi-view.
double ProjectionIoU(const fusion_adapter::YoloBox&, const LidarImageBox&);
// Main IOUFusion: greedy one-to-one IoU association, Observe on matches and
// Miss on every other observed object. `detected` holds normalized classes.
void IOUFusion(std::vector<TrackedObject>& objects,
               const std::vector<fusion_adapter::YoloBox>& detected,
               const std::vector<LidarImageBox>& lidar_boxes, std::uint64_t image_stamp_ns,
               std::uint64_t epoch, const FusionConfig& config, FusionCounters* counters,
               MatchDiagnostic* diagnostic = nullptr);

class FrontCameraFusion {
 public:
  // Throws std::runtime_error on an invalid policy/threshold/calibration. The
  // stored config holds Main's clamped values.
  FrontCameraFusion(const FusionConfig& config, const camera::Calibration& front);
  const FusionConfig& config() const { return config_; }

  // Called once per processed scan with that scan's tracks (Main
  // runCameraFusion). `live` are the scan's objects (classes updated in place);
  // `history` is the tracker's last committed frame (Main
  // Prev_Objects_Queue.back()), class fields are persisted into it. Geometry is
  // never modified. decision_ns is the injected decision clock.
  FusionReport Process(std::int64_t decision_ns, std::int64_t scan_stamp_ns,
                       const asmc::lidar::Context& context, std::vector<TrackedObject>& live,
                       std::vector<TrackedObject>* history, CameraSource& camera,
                       std::int64_t ros_now_ns, std::int64_t steady_now_ns);

  const FusionCounters& counters() const { return counters_; }
  std::uint64_t consumer_epoch() const { return consumer_epoch_; }
  std::size_t consumed_size() const { return consumed_.size(); }

 private:
  struct PendingScan {
    std::int64_t stamp_ns = 0;
    std::int64_t queued_ns = 0;
    asmc::lidar::Context context;
    std::vector<TrackedObject> objects;
  };
  FusionConfig config_;
  camera::Calibration front_;
  FusionCounters counters_;
  std::deque<PendingScan> pending_;
  // Identities already used (both policies) -> source stamp. unique excludes
  // them from matching; bounded_reuse counts their reuse. Pruned in Process().
  std::map<std::string, std::int64_t> consumed_;
  std::uint64_t admitted_epoch_ = 0;
  std::uint64_t consumer_epoch_ = 1;  // Main g_CameraEpoch analogue (class invalidation epoch)
  bool was_active_ = false;
  bool received_any_frame_ = false;
  std::int64_t last_queued_stamp_ns_ = 0;
  std::uint64_t source_epoch_ = 0;     // receiver epoch of the last active frames
  std::int64_t prune_before_ns_ = 0;   // Main PruneInputsBefore watermark (yolo part)
  std::int64_t max_offset_ns_ = 0, wait_timeout_ns_ = 0, read_window_ns_ = 0;
  bool reuse_ = false;
  std::vector<std::string> allowed_;
};

// Adapter over the asmc_lidar camera::Buffer (Receiver). Not consuming; the
// fusion keeps its own consumed set.
class BufferCameraSource : public CameraSource {
 public:
  explicit BufferCameraSource(std::shared_ptr<camera::Buffer> buffer,
                              std::string front_frame_id = "camera_front");
  CameraSnapshot Read(std::int64_t reference_stamp_ns, std::int64_t window_ns,
                      std::int64_t ros_now_ns, std::int64_t steady_now_ns) override;
  bool Current(const CameraFrame& frame, std::int64_t ros_now_ns,
               std::int64_t steady_now_ns) override;

 private:
  std::shared_ptr<camera::Buffer> buffer_;
  std::string front_;
};

}  // namespace fusion
}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
