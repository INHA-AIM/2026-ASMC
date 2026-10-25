#pragma once
// Decision-only post-fusion audit. No ROS, PCL, track mutation or exclusion API.
#include <array>
#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <vector>
namespace asmc::lidar::filtering {
using Point = std::array<double, 3>;
struct AuditFrame {
  std::string frame_id, map_id;
  std::array<double, 3> map_origin{};
  int status = 0;
  std::int64_t stamp_ns = 0, pose_stamp_ns = 0;
  bool pose_valid = false, position_error_verified = false, mission_scope_verified = false;
  double position_error_bound_m = 0, source_age_ms = -1;
};
struct AuditObject {
  int track_id = -1;
  std::string observation_state = "unknown";
  std::vector<Point> corners;
};
struct SpatialRelation {
  bool available = false, boundary_protected = false, unknown_cells = false;
  double road_ratio_grid = 0, raster_resolution_m = 0;
  std::string reason;
  std::vector<std::string> exception_types;
};
struct PointEvidence;
struct AuditDecision {
  int track_id = -1;
  std::string input_state = "unknown", decision = "undetermined", reason = "uninitialized";
  bool has_spatial = false;
  SpatialRelation spatial;
  std::vector<std::string> exception_types;
  double comparison_ms = 0;
  bool preview_enabled=false;
  std::string preview_decision="undetermined", preview_reason="not_evaluated";
  std::string point_preview="undetermined",obb_preview="undetermined",height_preview="undetermined";
  std::string obb_reference_decision, obb_reference_reason;
  std::size_t support_points=0, road_points=0, interior_road_points=0, guard_points=0, unknown_points=0;
  double road_point_ratio=0, interior_road_ratio=0, min_base_z=0, obb_bottom_base_z=0, roof_limit_z=0;
  bool height_available=false, camera_dynamic_protected=false, camera_vehicle_protected=false, camera_known=false;
  std::string preview_protection_policy, camera_observed_group="unknown", camera_observed_raw="unknown";

};
struct AuditReport {
  bool enabled = false, available = false;
  std::string reason = "off", map_id, map_sha256, bundle_sha256, policy_sha256, preview_config_id;
  double calculation_ms = 0;
  double pose_source_age_ms = -1;
  std::string pose_method;
  std::size_t minimum_support_points=0,minimum_road_points=0;
  double minimum_road_ratio=0,road_inset_m=0,roof_limit_base_z=0;
  std::vector<AuditDecision> objects;
};
class HdMapAudit {
 public:
  // Load and verify once. All files are bounded raw arrays next to the manifest.
  // Identity/hash failure throws here; runtime catches it and preserves outputs.
  HdMapAudit(const std::string& manifest, const std::string& expected_manifest_sha256,
             double road_inset_m = 0);
  AuditDecision Evaluate(const AuditFrame&, const AuditObject&) const;
  // Constant-time road/guard lookup; caller transforms each support point once.
  void Accumulate(PointEvidence&, const Point& map_xyz, double base_z) const;
  const std::string& map_id() const;
  const std::string& map_sha256() const;
  const std::string& bundle_sha256() const;
  const std::string& policy_sha256() const;
  double road_inset_m() const;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
std::string FileSha256(const std::string& path);
// Optional diagnostics serialization only; caller supplies scan/session identity.
void WriteAuditDecision(std::ostream&, const AuditDecision&);
void WriteAuditReport(std::ostream&, const AuditReport&);
}  // namespace asmc::lidar::filtering
