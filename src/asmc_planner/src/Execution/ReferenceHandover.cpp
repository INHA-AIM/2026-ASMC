#include "Execution/ReferenceHandover.hpp"
#include <cmath>

namespace asmc_planner::lane_reference {
namespace {
bool positive(double x) { return std::isfinite(x)&&x>0.0; }
bool fresh(double now,double stamp,double maximum_age) {
  return positive(stamp)&&std::isfinite(now)&&now>=stamp&&now-stamp<=maximum_age;
}
}
bool ReferenceHandoverGate::begin(std::uint32_t id,std::uint64_t epoch,
    std::shared_ptr<const LaneReference> target,double arrival) {
  const auto& o=options_;
  if (!id||!epoch||!target||!positive(o.lateral_tolerance_m)||!positive(o.heading_tolerance_rad)||
      o.heading_tolerance_rad>=1.5707963267948966||!positive(o.lateral_speed_tolerance_mps)||
      !positive(o.settle_duration_s)||!positive(o.ego_max_age_s)||!positive(o.objects_max_age_s)||
      !positive(o.max_pose_gap_s)||!std::isfinite(arrival)||arrival<target->usableBegin()||arrival>target->usableEnd()) return false;
  if (command_id_==id) {
    // A same-ID refresh must not reset the settling timer or silently change
    // the target candidate/arrival/epoch fingerprint.
    return source_epoch_==epoch&&target_.get()==target.get()&&minimum_arrival_s_==arrival;
  }
  command_id_=id; source_epoch_=epoch; target_=std::move(target); minimum_arrival_s_=arrival;
  last_stamp_=-1.0; stable_since_=-1.0;
  return true;
}
HandoverGeometryResult ReferenceHandoverGate::observe(std::uint32_t id,std::uint64_t epoch,
    const MapIdentity& current_map,const HandoverObservation& obs) {
  auto invalid=[&](const std::string& s) { stable_since_=-1.0; return HandoverGeometryResult{HandoverGeometryState::INVALID,{},s}; };
  if (!target_||id!=command_id_||epoch!=source_epoch_||current_map!=target_->identity()) return invalid("handover_context_mismatch");
  if (!obs.ego_valid||!obs.objects_valid||!fresh(obs.now_s,obs.ego_stamp_s,options_.ego_max_age_s)||
      !fresh(obs.now_s,obs.objects_stamp_s,options_.objects_max_age_s)) return invalid("stale_or_invalid_input");
  const auto& e=obs.ego;
  if (!std::isfinite(e.x)||!std::isfinite(e.y)||!std::isfinite(e.yaw)||!std::isfinite(e.v)||
      e.v<0.0||!std::isfinite(e.kappa)||!std::isfinite(e.a)) return invalid("invalid_vehicle_state");
  if (obs.ego_stamp_s<last_stamp_) { last_stamp_=-1.0; return invalid("pose_time_reversed"); }
  if (obs.ego_stamp_s==last_stamp_) return {HandoverGeometryState::DUPLICATE_POSE,{},"duplicate_source_pose_does_not_advance_settling"};
  if (last_stamp_>0.0 && obs.ego_stamp_s-last_stamp_>options_.max_pose_gap_s) stable_since_=-1.0;
  last_stamp_=obs.ego_stamp_s;
  auto p=ProjectOnLaneReference(*target_,e.x,e.y,e.yaw,e.v);
  const bool aligned=p.valid&&p.s>=minimum_arrival_s_&&p.s<=target_->usableEnd()&&
    std::abs(p.d)<=options_.lateral_tolerance_m&&std::abs(p.heading_error_rad)<=options_.heading_tolerance_rad&&
    std::abs(p.lateral_speed_mps)<=options_.lateral_speed_tolerance_mps&&target_->coversFootprint(e.x,e.y,e.yaw)&&
    !target_->map()->touchesSolid(e.x,e.y,e.yaw,target_->options().vehicle,target_->options().footprint_margin_m);
  if (!aligned) { stable_since_=-1.0; return {HandoverGeometryState::ALIGNING,p,"target_lane_alignment_or_footprint_not_ready"}; }
  if (stable_since_<0.0) stable_since_=obs.ego_stamp_s;
  if (obs.ego_stamp_s-stable_since_+1e-9<options_.settle_duration_s)
    return {HandoverGeometryState::ALIGNING,p,"target_lane_settling"};
  return {HandoverGeometryState::GEOMETRY_ELIGIBLE,p,"geometry_only_continuation_and_dynamic_safety_still_required"};
}
}  // namespace asmc_planner::lane_reference
