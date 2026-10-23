#pragma once
// Detection records, same-scan consolidation and the CA Kalman/Hungarian tracker.
// Ported from Main Tracking.cpp (Objectinfo_for_km, ConsolidateCurrentDetections,
// CompareCentroids, obb_util), FrameFilter.cpp, Kalman.cpp and HungarianAlg.cpp at
// 8f93c22e, common-geometry path only (local association, no map-centre cost).

#include <asmc_lidar/geometry/scan_frame.hpp>
#include <asmc_lidar/tracking/track.hpp>

#include <pcl/PointIndices.h>

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

// Main LiDARTracking (setting.yaml LiDAR.Tracking + Global.cpp initializer).
// b_UseMapAssociation and f32_COST_W_DIR are not used by the common path; they
// are kept only so the same setting file loads.
struct TrackingConfig {
  float32 f32_COST_W_DIST = 1.0f;
  float32 f32_COST_W_IOU = 1.0f;
  float32 f32_COST_W_DIR = 0.5f;
  float32 f32_COST_THRESHOLD = 3.0f;
  float32 f32_COST_INF = 9999.f;
  bool b_UseMapAssociation = true;
  float32 f32_ContinuityBonus = 0.20f;
  float32 f32_DuplicateCenterDistance = 0.60f;
  float32 f32_DuplicateIouThreshold = 0.35f;
  float32 f32_MergedTrackRetireDistance = 0.45f;
};

// Main ConsolidateCurrentDetections(): repeated pairwise merge while the OBB
// centre distance <= DuplicateCenterDistance and the body AABB IoU >=
// DuplicateIouThreshold; the object with more points (lower index on a tie)
// owns the merged cloud and is recomputed (RecomputeGeometry).
void ConsolidateCurrentDetections(std::vector<TrackedObject>& detections,
                                  const TrackingConfig& config);

// Main Tracking.cpp namespace tracking_association (common path; the map-centre
// helpers FiniteMapCenter/MapCenterDistance are legacy-only and not ported).
namespace tracking_association {
bool IsGroundNoiseClass(const std::string& object_class);  // "noise" or "지면노이즈"
// Carry the previous class unless the previous class was ground noise and the
// current observation is not.
bool ShouldCarryPreviousClass(const TrackedObject& previous, const TrackedObject& current);
// Body-frame AABB IoU (f32_Min/Max X/Y); 0 when the union <= 1e-6.
float AabbIou(const TrackedObject& left, const TrackedObject& right);
// (long/long) x (short/short) side ratios of the OBB W/H, sides floored at 0.05,
// invariant to an axis swap. This is the "IOU" term of the association cost.
float ShapeAgreement(const TrackedObject& left, const TrackedObject& right);
// Mean (double accumulation), AABB, f32_Dist and L-shape OBB from the finite
// cluster points; unchanged when there is none.
void RecomputeGeometry(TrackedObject& object);
// Main fusion_state::Copy(source, target, unlock=false): the 8 class fields.
void CopyClassState(const TrackedObject& source, TrackedObject& target);
}  // namespace tracking_association

class Tracker {
 public:
  // Main hard-coded tracker constants (Tracking.cpp:920, FrameFilter.cpp:55/142).
  static constexpr int32 kMaxMissCount = 25;     // deleted at the 25th consecutive miss
  static constexpr double kBirthInitDt = 0.1;    // dt passed to the KF init of first-frame births
  static constexpr std::size_t kMaxHistory = 3;  // Prev_Objects_Queue length

  explicit Tracker(const TrackingConfig& config = {});
  const TrackingConfig& config() const { return config_; }

  // Main FrameFilter() + CompareCentroids() (common path). On input `objects`
  // holds this scan's detections; on return the matched, born and carried
  // (missed) tracks. dt_s is (current - previous source stamp) as ros::Duration
  // toSec(). `frame` must already be Prepare()d and hold this scan's leveling.
  // Not done here (Main LidarProcess does it after FrameFilter, the pipeline
  // must too, in this order): sort by f32_Dist, RecordTrackingFrame for every
  // MissCount==0 object, then Commit().
  void Update(std::vector<TrackedObject>& objects, double dt_s, const LocalState& local,
              const ScanFrame& frame);

  // Main LidarProcess tail: push the finished frame, keep at most 3 frames.
  void Commit(const std::vector<TrackedObject>& frame);
  // Last committed frame (Main st_LiDAR.Prev_Objects_Queue.back()); nullptr if none.
  std::vector<TrackedObject>* LastFrame();
  const std::deque<std::vector<TrackedObject>>& History() const { return history_; }
  int32 NextTrackId() const { return next_track_id_; }

 private:
  TrackingConfig config_;
  std::deque<std::vector<TrackedObject>> history_;
  int32 next_track_id_ = 0;
};

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
