#include "Frenet/Reference/LaneReference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace asmc_planner::lane_reference {
namespace {
bool positive(double v) { return std::isfinite(v) && v>0.0; }
bool optionsValid(const LaneReferenceOptions& o) {
  const auto& r=o.reference;
  return ValidFootprint(o.vehicle) && positive(o.max_curvature) && positive(o.max_lateral_accel) &&
    std::isfinite(o.max_join_gap_m) && o.max_join_gap_m>=0.0 &&
    positive(o.sample_distance_m) && positive(o.sample_yaw_rad) &&
    std::isfinite(o.footprint_margin_m) && o.footprint_margin_m>=0.0 &&
    o.max_samples>=3 && std::isfinite(r.closure_tolerance_m) && r.closure_tolerance_m>=0.0 &&
    positive(r.smoothing_window_m) && positive(r.resample_ds) &&
    r.relax_max_iter>=0 && positive(r.relax_step) && r.relax_step<=1.0 &&
    positive(r.ego_projection_search_window);
}
double angle(double v) { return std::atan2(std::sin(v),std::cos(v)); }

bool uniformInput(const std::vector<double>& x,const std::vector<double>& y,
    double maximum_step,std::size_t budget,std::vector<double>& out_x,std::vector<double>& out_y) {
  std::vector<double> stations(x.size(),0.0);
  for(std::size_t i=1;i<x.size();++i)
    stations[i]=stations[i-1]+std::hypot(x[i]-x[i-1],y[i]-y[i-1]);
  const double length=stations.back();
  if(!positive(length))return false;
  const double requested=std::ceil(length/maximum_step);
  if(!std::isfinite(requested)||requested>double(budget-1))return false;
  const auto segments=static_cast<std::size_t>(requested);
  // A single grid over the WHOLE connected chain. Resetting at each link
  // would recreate the short endpoint interval that biases vertex averaging.
  // Spread the final remainder across the grid to retain both physical ends.
  out_x.reserve(segments+1);out_y.reserve(segments+1);
  std::size_t source=0;
  for(std::size_t i=0;i<=segments;++i) {
    const double s=i==segments?length:length*double(i)/double(segments);
    while(source+2<stations.size()&&stations[source+1]<s)++source;
    const double ratio=(s-stations[source])/(stations[source+1]-stations[source]);
    out_x.push_back(x[source]+ratio*(x[source+1]-x[source]));
    out_y.push_back(y[source]+ratio*(y[source+1]-y[source]));
  }
  return true;
}
}  // namespace

bool LaneReference::coversFootprint(double x,double y,double yaw) const {
  return corridor_->coversFootprint(x,y,yaw,options_.vehicle,options_.footprint_margin_m) &&
    coversRoadFootprint(x,y,yaw);
}
bool LaneReference::coversRoadFootprint(double x,double y,double yaw) const {
  return options_.runtime_road_footprint?options_.runtime_road_footprint(x,y,yaw):
      map_->road()->coversFootprint(x,y,yaw,options_.vehicle,options_.footprint_margin_m);
}

LaneReferenceBuildResult BuildLaneReference(std::shared_ptr<const HdLaneMap> map,
    const MapIdentity& expected,const std::vector<std::string>& sequence,const LaneReferenceOptions& options) {
  return BuildLaneReference(std::move(map),expected,sequence,options,LaneReferenceSampling::AS_PROVIDED);
}

LaneReferenceBuildResult BuildLaneReference(std::shared_ptr<const HdLaneMap> map,
    const MapIdentity& expected,const std::vector<std::string>& sequence,const LaneReferenceOptions& options,
    LaneReferenceSampling sampling) {
  auto fail=[](const std::string& s,std::optional<RefPoint> p=std::nullopt) { return LaneReferenceBuildResult{{},s,p}; };
  if (!map || map->identity()!=expected) return fail("map_identity_mismatch");
  if (!optionsValid(options) || sequence.empty() || sequence.size()>options.max_samples) return fail("invalid_reference_options_or_sequence");
  std::vector<double> x,y;
  std::unordered_set<std::string> seen;
  const LaneData* previous=nullptr;
  double raw_length=0.0;
  for (const auto& id:sequence) {
    const auto* lane=map->lane(id);
    if (!lane || !lane->traversable || !seen.insert(id).second) return fail("unknown_unusable_or_repeated_lane:"+id);
    if (previous) {
      if (std::find(previous->successors.begin(),previous->successors.end(),id)==previous->successors.end())
        return fail("disconnected_lane_sequence:"+previous->id+":"+id);
      const auto& a=previous->centerline.back(); const auto& b=lane->centerline.front();
      if (std::hypot(a.x-b.x,a.y-b.y)>options.max_join_gap_m) return fail("endpoint_gap:"+id);
    }
    for (const auto& p:lane->centerline) {
      if (!x.empty()) {
        const double ds=std::hypot(p.x-x.back(),p.y-y.back());
        if (ds<=1e-9) continue;
        raw_length+=ds;
      }
      x.push_back(p.x); y.push_back(p.y);
      if (x.size()>options.max_samples) return fail("raw_reference_sample_budget_exceeded");
    }
    previous=lane;
  }
  const double sample_step=std::min(options.sample_distance_m,options.sample_yaw_rad/options.max_curvature);
  if (raw_length/options.reference.resample_ds>double(options.max_samples)-2.0 ||
      raw_length/sample_step>double(options.max_samples)-2.0) return fail("reference_sample_budget_exceeded");
  if(sampling==LaneReferenceSampling::UNIFORM_ARC_LENGTH) {
    std::vector<double> uniform_x,uniform_y;
    if(!uniformInput(x,y,options.reference.resample_ds,options.max_samples,uniform_x,uniform_y))
      return fail("uniform_input_sample_budget_or_length_invalid");
    x=std::move(uniform_x);y=std::move(uniform_y);
  } else if(sampling!=LaneReferenceSampling::AS_PROVIDED) return fail("invalid_reference_sampling_policy");
  auto candidate=std::make_shared<LaneReference>();
  candidate->map_=std::move(map); candidate->lanes_=sequence; candidate->options_=options;
  std::string error;
  candidate->corridor_=candidate->map_->corridor(sequence,error);
  if (!candidate->corridor_) return fail("invalid_lane_corridor:"+error);
  try { candidate->line_=BuildRefLine(x,y,options.reference,options.max_curvature); }
  catch (const std::exception& e) { return fail(std::string("reference_build_failed:")+e.what()); }
  const auto& line=candidate->line_;
  if (line.closed_loop || line.points.size()<3 || line.points.size()>options.max_samples || !positive(line.length))
    return fail("invalid_open_reference");
  for (std::size_t i=0;i<line.points.size();++i) {
    const auto& p=line.points[i];
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.s)||!std::isfinite(p.theta)||
        !std::isfinite(p.kappa)||!std::isfinite(p.d_kappa)||(i && p.s<=line.points[i-1].s)) return fail("invalid_reference_sample");
    candidate->maximum_curvature_=std::max(candidate->maximum_curvature_,std::abs(p.kappa));
    if (i) candidate->maximum_curvature_=std::max(candidate->maximum_curvature_,
      std::abs(angle(p.theta-line.points[i-1].theta))/(p.s-line.points[i-1].s));
  }
  if (candidate->maximum_curvature_>options.max_curvature+1e-9) return fail("reference_curvature_exceeded");
  candidate->usable_begin_=options.vehicle.rear_m+options.footprint_margin_m;
  candidate->usable_end_=line.length-options.vehicle.front_m-options.footprint_margin_m;
  if (candidate->usable_end_<=candidate->usable_begin_) return fail("insufficient_footprint_horizon");
  // Only the reported usable interval is certified: the car extends beyond a
  // local reference's first/last station. Interpolate must not hide that cap.
  const double validation_step=candidate->maximum_curvature_>1e-12
    ?std::min(options.sample_distance_m,options.sample_yaw_rad/candidate->maximum_curvature_)
    :options.sample_distance_m;
  const auto count=std::size_t(std::ceil((candidate->usable_end_-candidate->usable_begin_)/validation_step));
  for (std::size_t i=0;i<=count;++i) {
    const double s=candidate->usable_begin_+(candidate->usable_end_-candidate->usable_begin_)*double(i)/double(count);
    const auto p=Interpolate(line,s);
    if (!candidate->coversRoadFootprint(p.x,p.y,p.theta))
      return fail("reference_footprint_outside_drivable_union",p);
    // Lane polygons identify the requested lane sequence. DrivableArea is the
    // authoritative physical containment layer, including convergence fill.
    // Requiring the whole car inside each lane union here would turn a tiny
    // classification seam into a wall, even on validated drivable pavement.
    if (!candidate->corridor_->coversPoint(p.x,p.y))
      return fail("reference_center_outside_lane_corridor",p);
    std::string marking;
    if (candidate->map_->touchesSolid(p.x,p.y,p.theta,options.vehicle,options.footprint_margin_m,&marking))
      return fail("reference_touches_solid:"+marking,p);
  }
  candidate->curve_speed_limit_=candidate->maximum_curvature_>1e-12
    ? std::sqrt(options.max_lateral_accel/candidate->maximum_curvature_)
    : std::numeric_limits<double>::infinity();
  return {std::move(candidate),"static_reference_valid_dynamic_safety_not_evaluated",std::nullopt};
}

ReferenceProjection ProjectOnLaneReference(const LaneReference& r,double x,double y,double yaw,double speed) {
  if (!std::isfinite(speed)||speed<0.0) return {};
  return ProjectOnLaneReference(r,x,y,yaw,speed*std::cos(yaw),speed*std::sin(yaw));
}

ReferenceProjection ProjectOnLaneReference(const LaneReference& r,double x,double y,double yaw,double vx,double vy) {
  ReferenceProjection result;
  if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||!std::isfinite(vx)||!std::isfinite(vy)) return result;
  const auto& line=r.line();
  const double s=FindClosestS(line,x,y,nullptr,r.options().reference.ego_projection_search_window);
  const auto p=Interpolate(line,s);
  result.s=s; result.d=-(x-p.x)*std::sin(p.theta)+(y-p.y)*std::cos(p.theta);
  result.distance_m=std::hypot(x-p.x,y-p.y);
  result.heading_error_rad=angle(yaw-p.theta);
  result.lateral_speed_mps=-vx*std::sin(p.theta)+vy*std::cos(p.theta);
  result.valid=std::isfinite(s)&&std::isfinite(result.d)&&std::isfinite(result.distance_m)&&
    std::isfinite(result.heading_error_rad)&&std::isfinite(result.lateral_speed_mps);
  return result;
}
}  // namespace asmc_planner::lane_reference
