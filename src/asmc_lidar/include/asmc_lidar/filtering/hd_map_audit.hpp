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
struct AuditDecision {
  int track_id = -1;
  std::string input_state = "unknown", decision = "undetermined", reason = "uninitialized";
  bool has_spatial = false;
  SpatialRelation spatial;
  std::vector<std::string> exception_types;
  double comparison_ms = 0;
};
struct AuditReport {
  bool enabled = false, available = false;
  std::string reason = "off", map_id, map_sha256, bundle_sha256, policy_sha256;
  double calculation_ms = 0;
  std::vector<AuditDecision> objects;
};
class HdMapAudit {
 public:
  // Load and verify once. All files are bounded raw arrays next to the manifest.
  // Identity/hash failure throws here; runtime catches it and preserves outputs.
  HdMapAudit(const std::string& manifest, const std::string& expected_manifest_sha256);
  AuditDecision Evaluate(const AuditFrame&, const AuditObject&) const;
  const std::string& map_id() const;
  const std::string& map_sha256() const;
  const std::string& bundle_sha256() const;
  const std::string& policy_sha256() const;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
std::string FileSha256(const std::string& path);
// Optional diagnostics serialization only; caller supplies scan/session identity.
void WriteAuditDecision(std::ostream&, const AuditDecision&);
void WriteAuditReport(std::ostream&, const AuditReport&);
}  // namespace asmc::lidar::filtering
