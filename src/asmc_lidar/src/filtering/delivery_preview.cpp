#include <asmc_lidar/filtering/delivery_preview.hpp>
#include <cmath>
namespace asmc::lidar::filtering {
bool PositiveCameraEvidence(std::int64_t scan,std::uint64_t observation,std::uint64_t epoch,
                            std::uint64_t observed_epoch,int misses,double age_limit,double future_limit) {
  if(scan<=0||!observation||!epoch||epoch!=observed_epoch||misses!=0)return false;
  const double age=(double(scan)-double(observation))/1e6;
  return age>=-future_limit&&age<=age_limit;
}
void ApplyDeliveryPreview(AuditDecision& d,const PointEvidence& e,const DeliveryPolicy& p,
                          bool map_valid,bool dynamic,bool known,double obb_bottom) {
  if(!p.enabled)return;
  d.obb_reference_decision=d.decision;d.obb_reference_reason=d.reason;
  d.preview_enabled=true;d.support_points=e.total;d.road_points=e.road;
  d.guard_points=e.guard;d.unknown_points=e.unknown;
  d.road_point_ratio=e.total?double(e.road)/e.total:0;
  d.height_available=e.finite==e.total&&e.total>0&&std::isfinite(e.min_base_z);
  if(d.height_available)d.min_base_z=e.min_base_z;
  d.obb_bottom_base_z=obb_bottom;d.roof_limit_z=p.roof_base_z_m+p.roof_margin_m;
  d.camera_dynamic_protected=dynamic;d.camera_known=known;
  const auto preview=[&](const char* v,const char* why){d.preview_decision=v;d.preview_reason=why;};
  const auto keep=[&](const char* why){preview("keep",why);d.decision="keep";d.reason=why;};
  // Independent geometric facets reuse the already computed footprint/support.
  // They are provisional display evidence, not final exclusion authorization.
  if(map_valid&&d.has_spatial&&d.spatial.available&&!d.spatial.unknown_cells)
    d.obb_preview=d.spatial.road_ratio_grid>0?"keep":"exclude_candidate";
  // A mobility group is a protection policy, never a measured moving flag.
  if(dynamic){d.point_preview=d.obb_preview=d.height_preview="keep";keep("camera_dynamic_protected");return;}
  if(!d.exception_types.empty())d.point_preview=d.obb_preview="keep";
  if(e.total>=p.minimum_points&&e.finite==e.total) {
    if(d.height_available&&std::isfinite(obb_bottom))
      d.height_preview=!known&&std::min(e.min_base_z,obb_bottom)>d.roof_limit_z?"exclude_candidate":"keep";
    if(map_valid&&d.has_spatial&&d.spatial.available&&!e.unknown&&d.exception_types.empty())
      d.point_preview=(e.road>=p.minimum_road_points||d.road_point_ratio>=p.minimum_road_ratio)?"keep":"exclude_candidate";
  }
  if(e.total<p.minimum_points||e.finite!=e.total){
    preview(d.obb_preview=="exclude_candidate"?"exclude_candidate":"undetermined",
            d.obb_preview=="exclude_candidate"?"obb_support_outside":"insufficient_cluster_support");return;
  }
  // Both the lowest support point AND the OBB must clear the body + margin.
  // Never discard a recognized car/person/obstacle solely because only its top
  // was scanned. Sparse/occluded unknown structures remain review candidates.
  if(!known&&d.height_available&&std::isfinite(obb_bottom)&&
      std::min(e.min_base_z,obb_bottom)>d.roof_limit_z) {
    d.height_preview="exclude_candidate";
    preview("exclude_candidate","overhead_clearance_candidate");
    // Position is irrelevant here, but completeness of the lower surface is
    // unproven. Do not promote a preview to final exclusion authorization.
    d.decision="undetermined";d.reason="overhead_lower_surface_unverified";return;
  }
  d.height_preview="keep";
  if(!map_valid||!d.has_spatial||!d.spatial.available){preview("undetermined","map_or_pose_unavailable");return;}
  if(!d.exception_types.empty()){d.point_preview="keep";keep("exception_space_protected");return;}
  if(e.unknown){preview("undetermined","unknown_support_map_cells");d.decision="undetermined";d.reason="unknown_support_map_cells";return;}
  if(e.road>=p.minimum_road_points||d.road_point_ratio>=p.minimum_road_ratio){d.point_preview="keep";keep("cluster_drivable_support");return;}
  d.point_preview="exclude_candidate";
  if(e.guard){
    // A road-exterior rail entirely inside the uncertainty band is still an
    // outside-support PREVIEW. Retain the uncertainty in the final decision;
    // a 1m guard must not silently turn 0 road points into measured road support.
    preview("exclude_candidate","cluster_outside_boundary_uncertain");
    d.decision="undetermined";d.reason="boundary_pose_uncertainty_unverified";return;
  }
  // Large OBBs can enter the road while the underlying guardrail points do not.
  // Keep the old OBB relation as diagnostics, use the support for the preview.
  preview("exclude_candidate","cluster_support_outside");
  if(d.decision!="exclude_candidate"){
    d.decision="undetermined";d.reason="point_policy_pose_and_scope_unverified";
  }
}
}  // namespace asmc::lidar::filtering
