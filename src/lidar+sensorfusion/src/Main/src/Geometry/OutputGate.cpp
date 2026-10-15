#include <Geometry/OutputGate.hpp>

#include <utility>

namespace output_gate {

const char* ModeName(Mode mode) {
  switch (mode) {
    case Mode::kLocal: return "local";
    case Mode::kMap: return "map";
    case Mode::kNone: default: return "none";
  }
}

const char* GpsStateName(GpsState state) {
  switch (state) {
    case GpsState::kHealthy: return "healthy";
    case GpsState::kDenied: return "denied";
    case GpsState::kRecovering: return "recovering";
    case GpsState::kNonGpsInvalid: return "non_gps_invalid";
    case GpsState::kNotEvaluated: default: return "not_evaluated";
  }
}

// Only the four GPS reasons select local. map_speed_invalid (CAN), local_motion_invalid (IMU/CAN),
// time errors and unknown reasons are an invalid local input, never a GPS denial.
GpsState ClassifyGps(bool map_pose_valid, const std::string& reason) {
  if (map_pose_valid) return GpsState::kHealthy;
  if (reason == "gps_missing" || reason == "gps_invalid" || reason == "gps_stale")
    return GpsState::kDenied;
  if (reason == "gps_recovering") return GpsState::kRecovering;
  return GpsState::kNonGpsInvalid;
}

namespace {
Decision None(GpsState gps, std::string reason) {
  Decision d;
  d.v2 = Mode::kNone;
  d.legacy_permitted = false;
  d.suppress_legacy_markers = true;
  d.gps = gps;
  d.reason = reason.empty() ? std::string("unknown") : std::move(reason);
  return d;
}
std::string OrUnknown(const std::string& value) { return value.empty() ? "unknown" : value; }
}  // namespace

Decision Decide(const Inputs& in) {
  if (!in.common_geometry) {
    // Pre-B0 rule with Enabled()==false: map_ready=true, legacy_ready=true.
    Decision d;
    d.v2 = Mode::kNone;
    d.legacy_permitted = in.lidar_processed;
    d.suppress_legacy_markers = false;   // SuppressMap was called only when Enabled()
    d.gps = GpsState::kNotEvaluated;
    d.reason = "legacy_mode";
    return d;
  }
  if (!in.pose_ready) return None(GpsState::kNotEvaluated, "local_input_invalid:" + OrUnknown(in.pose_reason));
  const GpsState gps = ClassifyGps(in.map_pose_valid, in.map_reason);
  if (!in.lidar_processed) return None(gps, "lidar_input_invalid");
  if (!in.context_current) return None(gps, "context_stale");
  if (!in.time_error.empty()) return None(gps, "time_invalid:" + in.time_error);
  if (!in.leveling_valid) return None(gps, "leveling_transform_invalid");
  switch (gps) {
    case GpsState::kHealthy: {
      if (!in.map_pose_matches_scan) return None(gps, "map_pose_time_mismatch");
      Decision d;
      d.v2 = Mode::kMap;
      d.legacy_permitted = true;
      d.suppress_legacy_markers = false;
      d.gps = gps;
      d.reason = "map:" + OrUnknown(in.map_reason);
      return d;
    }
    case GpsState::kDenied:
    case GpsState::kRecovering: {
      Decision d;
      d.v2 = Mode::kLocal;
      d.legacy_permitted = false;
      d.suppress_legacy_markers = true;
      d.gps = gps;
      d.reason = "local_only:" + in.map_reason;
      return d;
    }
    case GpsState::kNonGpsInvalid:
    case GpsState::kNotEvaluated:
    default:
      return None(gps, "map_pose_invalid:" + OrUnknown(in.map_reason));
  }
}

Decision Downgrade(const Decision& decision, const std::string& reason) {
  return None(decision.gps, reason);
}

LegacyPlan PlanLegacy(const Decision& effective, const Inputs& in, bool legacy_motion_ready) {
  LegacyPlan plan;
  if (!in.common_geometry) {
    // Pre-B0: publish iff lidar_processed; no SuppressMap; SetLegacyReason not called.
    plan.publish = effective.legacy_permitted;
    plan.suppress_markers = false;
    plan.report_reason = false;
    plan.reason.clear();
    return plan;
  }
  plan.publish = effective.v2 == Mode::kMap && effective.legacy_permitted && legacy_motion_ready;
  plan.suppress_markers = !plan.publish;     // DELETEALL whenever the legacy frame is withheld
  plan.report_reason = true;
  // Pre-B0 legacy_output strings are kept; "frame_invalid" is new for B0-only rejections.
  if (!in.pose_ready) plan.reason = "input_invalid";
  else if (!in.map_pose_valid || !in.time_error.empty()) plan.reason = "pose_unavailable";
  else if (!effective.legacy_permitted || effective.v2 != Mode::kMap) plan.reason = "frame_invalid";
  else if (!legacy_motion_ready) plan.reason = "motion_state_unrepresentable";
  else plan.reason = "available";
  return plan;
}

}  // namespace output_gate
