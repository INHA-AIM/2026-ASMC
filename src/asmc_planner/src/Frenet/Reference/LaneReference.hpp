#pragma once

#include "Map/HdLaneMap.hpp"
#include "Frenet/Reference/Reference.hpp"
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <functional>

namespace asmc_planner::lane_reference {

// Opt-in preparation of a connected HD-map chain before smoothing. Existing
// consumers retain their input contract; highway execution requests a grid
// that is independent of the source link's vertex density.
enum class LaneReferenceSampling { AS_PROVIDED, UNIFORM_ARC_LENGTH };

struct LaneReferenceOptions {
  ReferenceConfig reference{};
  FootprintShape vehicle{};
  double max_curvature = 0.0;
  double max_lateral_accel = 0.0;
  double max_join_gap_m = 0.0;
  double sample_distance_m = 0.0;
  double sample_yaw_rad = 0.0;
  double footprint_margin_m = 0.0;
  std::size_t max_samples = 0;
  // Optional authoritative runtime road query. Production's pinned 0.2 m
  // raster has source-seam semantics different from exact polygon overlays.
  // It must check the same full vehicle footprint; never a center-point test.
  // Standalone/vector-map consumers retain exact polygon containment.
  std::function<bool(double,double,double)> runtime_road_footprint;
};

class LaneReference;
struct LaneReferenceBuildResult {
  std::shared_ptr<const LaneReference> candidate;
  std::string detail;
  std::optional<RefPoint> rejected_point;
  explicit operator bool() const { return bool(candidate); }
};

// Read-only reference candidate. Construction checks sampled static constraints;
// it does not prove dynamic gap safety, trajectory continuity, or authorize LC.
class LaneReference {
 public:
  const RefLine& line() const { return line_; }
  const std::vector<std::string>& lanes() const { return lanes_; }
  const MapIdentity& identity() const { return map_->identity(); }
  std::shared_ptr<const HdLaneMap> map() const { return map_; }
  const LaneReferenceOptions& options() const { return options_; }
  double usableBegin() const { return usable_begin_; }
  double usableEnd() const { return usable_end_; }
  double maximumCurvature() const { return maximum_curvature_; }
  double curveSpeedLimit() const { return curve_speed_limit_; }
  bool coversCenter(double x,double y) const { return corridor_->coversPoint(x,y); }
  // Strict target-lane occupancy for handover. Static reference construction
  // uses the lane polygon for centerline identity and DrivableArea for the
  // whole vehicle, as required by the HD-map consumer contract.
  bool coversFootprint(double x,double y,double yaw) const;
  bool coversRoadFootprint(double x,double y,double yaw) const;
 private:
  RefLine line_;
  std::vector<std::string> lanes_;
  std::shared_ptr<const HdLaneMap> map_;
  std::shared_ptr<const LaneRegion> corridor_;
  LaneReferenceOptions options_;
  double usable_begin_=0.0, usable_end_=0.0;
  double maximum_curvature_=0.0, curve_speed_limit_=0.0;
  friend LaneReferenceBuildResult BuildLaneReference(std::shared_ptr<const HdLaneMap>,
      const MapIdentity&,const std::vector<std::string>&,const LaneReferenceOptions&);
  friend LaneReferenceBuildResult BuildLaneReference(std::shared_ptr<const HdLaneMap>,
      const MapIdentity&,const std::vector<std::string>&,const LaneReferenceOptions&,
      LaneReferenceSampling);
};

// The caller selects the mission-compatible sequence explicitly. In a fork,
// successors.front() is not a lane policy. Loops and disconnected seams fail.
// This is initialization/preparation work: cache the immutable result. Never
// call this builder from the 20 Hz trajectory-publication timer.
LaneReferenceBuildResult BuildLaneReference(std::shared_ptr<const HdLaneMap> map,
    const MapIdentity& expected,const std::vector<std::string>& lane_sequence,
    const LaneReferenceOptions& options);
LaneReferenceBuildResult BuildLaneReference(std::shared_ptr<const HdLaneMap> map,
    const MapIdentity& expected,const std::vector<std::string>& lane_sequence,
    const LaneReferenceOptions& options,LaneReferenceSampling sampling);

struct ReferenceProjection {
  bool valid=false;
  double s=0.0, d=0.0, distance_m=0.0, heading_error_rad=0.0, lateral_speed_mps=0.0;
};
// No old s/d or old s_hint crosses this interface. Ego and object projections
// are reconstructed from the same physical map coordinates on the new line.
// Projection validity is not lane occupancy; callers must also check distance,
// usable station range and object/vehicle footprint against the lane corridor.
// The speed overload assumes velocity along yaw (ego kinematics). Objects with
// independent motion must use map vx/vy, not their body heading as velocity.
ReferenceProjection ProjectOnLaneReference(const LaneReference& reference,
    double x,double y,double yaw,double speed_mps);
ReferenceProjection ProjectOnLaneReference(const LaneReference& reference,
    double x,double y,double yaw,double velocity_x_mps,double velocity_y_mps);

}  // namespace asmc_planner::lane_reference
