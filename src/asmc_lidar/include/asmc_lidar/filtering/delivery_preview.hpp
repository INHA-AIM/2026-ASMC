#pragma once
#include <asmc_lidar/filtering/hd_map_audit.hpp>
#include <limits>
namespace asmc::lidar::filtering {
inline constexpr const char* kDeliveryProtectionPolicy = "yolo_vehicle_dynamic_interior_v3";
// Evidence comes from the full voxel/filter/cluster support, never GUI sampling.
struct PointEvidence {
  std::size_t total=0, finite=0, road=0, interior_road=0, guard=0, unknown=0;
  double min_base_z=std::numeric_limits<double>::infinity();
};
struct DeliveryPolicy {
  bool enabled=false;
  double roof_base_z_m=1.284;  // mesh height 1.644m + nominal road z=-.36m
  double roof_margin_m=0.60;  // conservative review candidate, not deployment approval
  std::size_t minimum_points=10, minimum_road_points=3;
  // Both minimum count and fraction must be met in the inset road mask.
  // The original road count remains available for comparison; no map is edited.
  double minimum_road_ratio=0.05, road_inset_m=0.20, camera_age_ms=500;
};
bool PositiveCameraEvidence(std::int64_t scan_ns, std::uint64_t observation_ns,
                            std::uint64_t active_epoch, std::uint64_t observation_epoch,
                            int misses, double max_age_ms, double max_future_ms);
// No tracker mutation or publisher. Provisional preview can differ from the
// uncertainty-gated final audit decision; neither removes a ROS object.
void ApplyDeliveryPreview(AuditDecision&, const PointEvidence&, const DeliveryPolicy&,
                          bool map_valid, const std::string& matched_camera_group,
                          double obb_bottom_base_z);
}  // namespace asmc::lidar::filtering
