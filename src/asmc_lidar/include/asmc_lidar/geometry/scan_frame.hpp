#pragma once
// Per-scan coordinate context and track re-expression between scan frames.
// Ported from Main Geometry/FrameBoundary.cpp at 8f93c22e (the tracker fix
// "keep tracked OBB geometry in one frame"). Main keeps this state in
// namespace-scope globals; here one ScanFrame instance belongs to one pipeline.
// Only the common-geometry behaviour exists (Main's enable_common_geometry=true).

#include <asmc_lidar/tracking/track.hpp>

#include <cstdint>
#include <string>

namespace asmc {
namespace lidar {
namespace pipeline {

class ScanFrame {
 public:
  // Main SensorMount(): translation = LiDAR position, rotation = rpy of the
  // mount angles converted with Main's float Deg2Rad.
  static asmc::lidar::Pose SensorMount(const VehicleSpec& vehicle);
  // Main MapRotation(): rpy(IMU roll, pitch, yaw).
  static asmc::lidar::Rot MapRotation(const LocalState& local);

  // calibration_id must be non-empty (asmc::lidar::base_from_input requires it).
  void Configure(const VehicleSpec& vehicle, const std::string& calibration_id);

  // Main PrepareScan(Observation, ...): resets the Context, stores the scan
  // attitude/yaw of `local`, frame=LeveledSensor, mount, calibration,
  // leveling_source, map pose and ego_linear_base=(CAN v,0,0).
  void Prepare(const asmc::lidar::Observation& time, const LocalState& local,
               const asmc::lidar::PoseSelection& map_pose, std::uint64_t scan_sequence);
  // Main StoreLeveling(): the matrix CorrectingPoints actually applied.
  void StoreLeveling(const asmc::lidar::Rot& rotation);

  const asmc::lidar::Context& Current() const { return current_; }

  // Main RecordTrackingFrame / AlignTrackingState / AlignTrackingFrame.
  // AlignTrackingState re-expresses only the copied filter state (matched
  // tracks keep this scan's OBB). AlignTrackingFrame re-expresses a whole
  // previous-scan track: KF state plus OBB center, Z range, 8 corners and yaw,
  // with the same single frame change (no motion prediction of the box).
  // RecordTrackingFrame stores height=f32_CoreZ and this scan's frame; the caller
  // runs it for every MissCount==0 object after Tracker::Update (LidarProcess
  // order). Both Align calls return unchanged when !tracking_frame_valid.
  void RecordTrackingFrame(TrackedObject& object) const;
  void AlignTrackingState(TrackedObject& object, double yaw_now_rad) const;
  void AlignTrackingFrame(TrackedObject& object, double yaw_now_rad) const;
  std::optional<asmc::lidar::Pose> PreviousTrackingTransform(
      const TrackedObject& object,double yaw_now_rad) const;

  // Diagnostics of AlignTrackingFrame's corner-edge yaw follow (Main applies it
  // only for |correction| < 0.1 rad). Counts every call so a corner-order
  // violation is visible instead of silently skipped.
  // calls = not_recorded + degenerate_edge + applied + rejected. not_recorded:
  // the object had no tracking frame (Main returns before any change).
  // degenerate_edge: edge 0->1 xy length <= 1e-6 (every matched-track yaw prior,
  // whose value-initialized copy has all corners at the origin, lands here).
  struct EdgeFollowStats {
    std::uint64_t calls = 0, degenerate_edge = 0, applied = 0, rejected = 0;
    double max_applied_abs_rad = 0.;
    double max_rejected_abs_rad = 0.;
    std::uint64_t not_recorded = 0;
  };
  const EdgeFollowStats& edge_follow_stats() const { return edge_stats_; }
  void ResetEdgeFollowStats() { edge_stats_ = EdgeFollowStats{}; }

 private:
  // Main FrameBoundary.cpp TrackingFrameChange: previous tracking frame -> current.
  struct FrameChange {
    asmc::lidar::Rot rotation;
    asmc::lidar::Vec translation;
  };
  asmc::lidar::Rot YawFromInput() const;
  asmc::lidar::Vec OriginInYaw() const;
  FrameChange ChangeSincePrevious(const TrackedObject& object, double yaw_now_rad) const;
  void AlignState(TrackedObject& object, const FrameChange& change, double yaw_now_rad) const;

  asmc::lidar::Pose mount_ = asmc::lidar::Pose::Identity();
  std::string calibration_id_;
  asmc::lidar::Context current_;
  asmc::lidar::Rot scan_attitude_ = asmc::lidar::Rot::Identity();
  double scan_yaw_ = 0.;
  mutable EdgeFollowStats edge_stats_;
};

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
