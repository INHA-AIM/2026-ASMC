#pragma once

#include "Frenet/Reference/LaneReference.hpp"
#include "Frenet/Geometry/CartesianTypes.hpp"
#include <cstdint>

namespace asmc_planner::lane_reference {

struct HandoverOptions {
  double lateral_tolerance_m=0.0, heading_tolerance_rad=0.0, lateral_speed_tolerance_mps=0.0;
  double settle_duration_s=0.0, ego_max_age_s=0.0, objects_max_age_s=0.0, max_pose_gap_s=0.0;
};
struct HandoverObservation {
  double now_s=0.0, ego_stamp_s=0.0, objects_stamp_s=0.0;
  bool ego_valid=false, objects_valid=false;
  CartesianState ego{};
};
enum class HandoverGeometryState { INVALID, DUPLICATE_POSE, ALIGNING, GEOMETRY_ELIGIBLE };
struct HandoverGeometryResult {
  HandoverGeometryState state=HandoverGeometryState::INVALID;
  ReferenceProjection projection;
  std::string detail;
};

// Preparation certificate only. GEOMETRY_ELIGIBLE is not public READY or
// COMPLETED. The runtime still needs a newly validated continuation trajectory,
// dynamic collision/gap check and one atomic ego/object/leader/ref swap.
class ReferenceHandoverGate {
 public:
  explicit ReferenceHandoverGate(HandoverOptions options) : options_(options) {}
  bool begin(std::uint32_t command_id,std::uint64_t source_epoch,
             std::shared_ptr<const LaneReference> target,double minimum_arrival_s);
  HandoverGeometryResult observe(std::uint32_t command_id,std::uint64_t source_epoch,
      const MapIdentity& current_map,const HandoverObservation& observation);
 private:
  HandoverOptions options_;
  std::shared_ptr<const LaneReference> target_;
  std::uint32_t command_id_=0;
  std::uint64_t source_epoch_=0;
  double minimum_arrival_s_=0.0, last_stamp_=-1.0, stable_since_=-1.0;
};

}  // namespace asmc_planner::lane_reference
