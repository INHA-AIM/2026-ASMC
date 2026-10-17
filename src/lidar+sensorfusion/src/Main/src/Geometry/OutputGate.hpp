#pragma once
// B0 final-output selection for one LiDAR scan. Pure: no ROS, no globals, no I/O,
// standard headers only, so every row is unit-testable without a ROS master.
//
// enable_common_geometry=true (main_integration.launch). Local is the primary
// scan output and map is an additional output only while the map pose is valid:
//   map pose valid for this scan (gps_observed / gps_bounded_prediction)
//       -> /lidar/local_object AND /lidar/map_object. Legacy (/last_lidar, /lidar/tracked_objects_map,
//          /lidar/motion_debug, planner UDP) only when LegacyMotionReady holds for the whole frame.
//   gps_missing / gps_invalid / gps_stale / gps_recovering (fewer than 3 consecutive good samples)
//       -> /lidar/local_object only. No map_v2, no legacy, map markers DELETEALL.
//   LiDAR input, scan time, IMU/CAN (pose lookup), leveling, map transform or object geometry
//   invalid, or a map pose invalid for a non-GPS reason (map_speed_invalid, local_motion_invalid, ...)
//       -> kNone: NOTHING on either V2 topic (no empty or dummy array), no legacy, DELETEALL;
//          the reason is reported only through /perception/lidar/diagnostics.
// An empty V2 array (OBSERVED_EMPTY) is a real observation of zero objects on the active topic
// with valid inputs, never a mode signal. The local_v2/map_v2 choice never depends on subscribers.
// The 3-consecutive-good GPS recovery latch stays in asmc::lidar::BoundedPoseHistory
// (reason "gps_recovering"); this gate is stateless, so the first scan after the latch is map_v2.
//
// enable_common_geometry=false (production main.launch default): no V2 at all and the legacy
// outputs follow the pre-B0 rule exactly (publish iff lidar_processed, no SuppressMap, no
// SetLegacyReason, LegacyMotionReady not applied). Known gap left for B1: in this mode warm-up
// objects still reach /last_lidar and planner UDP as speed-0 objects, as before B0.
#include <cstdint>
#include <string>

namespace output_gate {

enum class Mode : std::uint8_t { kNone = 0, kLocal = 1, kMap = 2, kBoth = 3 };
enum class GpsState : std::uint8_t {
  kNotEvaluated = 0,   // legacy mode or pose not ready (no map pose computed for this scan)
  kHealthy = 1,        // map pose valid
  kDenied = 2,         // gps_missing / gps_invalid / gps_stale
  kRecovering = 3,     // gps_recovering (fewer than 3 consecutive good samples)
  kNonGpsInvalid = 4,  // map pose invalid for a non-GPS reason
};
const char* ModeName(Mode mode);
const char* GpsStateName(GpsState state);
// Map-pose reason strings: asmc_lidar/src/bounded_pose.cpp and Parser/SensorPoseHistory.cpp MapAt.
GpsState ClassifyGps(bool map_pose_valid, const std::string& map_reason);

struct Inputs {
  bool common_geometry = false;        // frame_boundary::Enabled()
  bool pose_ready = false;             // local IMU/CAN lookup for this scan succeeded
  std::string pose_reason;             // lookup reason when !pose_ready
  bool lidar_processed = false;        // LidarProcess() accepted this scan
  // Meaningful only when pose_ready (time/map) or lidar_processed (generation/leveling); all of
  // them come from ONE context of this scan (frame_boundary::GateInputs). Otherwise false/empty.
  bool context_current = false;        // context.input_generation == this scan's generation
  std::string time_error;              // asmc::lidar::validate_time(context.time), empty == valid
  bool leveling_valid = false;         // asmc::lidar::base_from_input(context).has_value()
  bool map_pose_valid = false;         // context.map_pose.valid
  bool map_pose_matches_scan = false;  // query stamp and clock equal the scan, map_from_base valid
  std::string map_reason;              // context.map_pose.reason
};

struct Decision {
  Mode v2 = Mode::kNone;                  // kNone: no V2 message at all for this scan
  bool legacy_permitted = false;          // frame-level only; LegacyMotionReady applied in PlanLegacy
  bool suppress_legacy_markers = false;   // SuppressMap(scan_stamp): map + /last_lidar_markers DELETEALL
  GpsState gps = GpsState::kNotEvaluated;
  std::string reason = "no_decision";     // diagnostic reason, never empty after Decide
};
Decision Decide(const Inputs& in);
// After the V2 builder or stamp guard rejects the frame: nothing on V2, legacy withheld.
Decision Downgrade(const Decision& decision, const std::string& reason);

struct LegacyPlan {
  bool publish = false;            // PlannerObjectRosPublisher::Publish AND PlannerObjectSender::Send
  bool suppress_markers = false;   // PlannerObjectRosPublisher::SuppressMap
  bool report_reason = false;      // frame_boundary::Publisher::SetLegacyReason (common geometry only)
  std::string reason;              // legacy_output diagnostic value
};
// legacy_motion_ready = frame_boundary::LegacyMotionReady(st_LiDAR): frame-wide warm-up gate,
// unchanged. Ignored when !common_geometry (pre-B0 production behaviour).
LegacyPlan PlanLegacy(const Decision& effective, const Inputs& in, bool legacy_motion_ready);

}  // namespace output_gate
