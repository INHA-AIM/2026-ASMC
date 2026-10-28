#include "Execution/LaneReferenceExecution.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include "Frenet/Geometry/PolygonUtils.hpp"

namespace asmc_planner::lane_reference {
namespace {
bool positive(double x) { return std::isfinite(x)&&x>0; }
double angle(double x) { return std::atan2(std::sin(x),std::cos(x)); }
bool pathShape(const CartesianPath& p) {
  const auto n=p.x.size();
  if(n<2||p.y.size()!=n||p.yaw.size()!=n||p.kappa.size()!=n||p.v.size()!=n||p.a.size()!=n)return false;
  for(const auto* values:{&p.x,&p.y,&p.yaw,&p.kappa,&p.v,&p.a})
    for(double x:*values)if(!std::isfinite(x))return false;
  return std::all_of(p.v.begin(),p.v.end(),[](double v){return v>=-1e-6;});
}
bool fresh(double now,double stamp,double age) {
  return std::isfinite(now)&&positive(stamp)&&now>=stamp&&now-stamp<=age;
}
ReferenceProjection project(const RefLine& line,double x,double y,double yaw,double vx,double vy) {
  if(line.points.size()<3||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||
      !std::isfinite(vx)||!std::isfinite(vy))return {};
  const double s=FindClosestS(line,x,y,nullptr,100);const auto p=Interpolate(line,s);
  const double d=-(x-p.x)*std::sin(p.theta)+(y-p.y)*std::cos(p.theta);
  return {true,s,d,std::hypot(x-p.x,y-p.y),angle(yaw-p.theta),-vx*std::sin(p.theta)+vy*std::cos(p.theta)};
}
}
bool HandoverPathsContinuous(const CartesianPath& a,const CartesianPath& b,double dt,
    const HandoverContinuity& c) {
  if(!pathShape(a)||!pathShape(b)||!positive(dt)||!positive(c.overlap_s)||
      !positive(c.position_m)||!positive(c.heading_rad)||!positive(c.curvature_inv_m)||
      !positive(c.speed_mps)||!positive(c.acceleration_mps2))return false;
  const auto count=std::size_t(std::ceil(c.overlap_s/dt))+1;
  if(a.x.size()<count||b.x.size()<count)return false;
  for(std::size_t i=0;i<count;++i)
    if(std::hypot(a.x[i]-b.x[i],a.y[i]-b.y[i])>c.position_m||
       std::abs(angle(a.yaw[i]-b.yaw[i]))>c.heading_rad||
       std::abs(a.kappa[i]-b.kappa[i])>c.curvature_inv_m||
       std::abs(a.v[i]-b.v[i])>c.speed_mps||std::abs(a.a[i]-b.a[i])>c.acceleration_mps2)return false;
  return true;
}
bool HandoverPathsGeometricallyContinuous(const CartesianPath& a,const CartesianPath& b,
    double dt,const HandoverContinuity& c) {
  if(!pathShape(a)||!pathShape(b)||!positive(dt)||!positive(c.overlap_s)||
      !positive(c.position_m)||!positive(c.heading_rad)||!positive(c.curvature_inv_m)||
      !positive(c.speed_mps)||!positive(c.acceleration_mps2))return false;
  const auto count=std::size_t(std::ceil(c.overlap_s/dt))+1;
  if(a.x.size()<count||b.x.size()<count||std::abs(a.v.front()-b.v.front())>c.speed_mps||
      std::abs(a.a.front()-b.a.front())>c.acceleration_mps2)return false;
  auto arc=[&](const CartesianPath& p) {
    std::vector<double> s(count,0);
    for(std::size_t n=1;n<count;++n)s[n]=s[n-1]+std::hypot(p.x[n]-p.x[n-1],p.y[n]-p.y[n-1]);
    return s;
  };
  const auto sa=arc(a),sb=arc(b);
  const double length=std::min(sa.back(),sb.back());
  auto sample=[](const CartesianPath& p,const std::vector<double>& s,double at) {
    auto end=std::lower_bound(s.begin()+1,s.end(),at);
    const auto n=std::min(std::size_t(end-s.begin()),s.size()-1);
    const double w=s[n]>s[n-1]?(at-s[n-1])/(s[n]-s[n-1]):0;
    return CartesianState{p.x[n-1]+w*(p.x[n]-p.x[n-1]),p.y[n-1]+w*(p.y[n]-p.y[n-1]),
        p.yaw[n-1]+w*angle(p.yaw[n]-p.yaw[n-1]),p.kappa[n-1]+w*(p.kappa[n]-p.kappa[n-1]),0,0};
  };
  for(std::size_t n=0;n<count;++n) {
    const double at=length*n/(count-1);
    const auto pa=sample(a,sa,at),pb=sample(b,sb,at);
    if(std::hypot(pa.x-pb.x,pa.y-pb.y)>c.position_m||
       std::abs(angle(pa.yaw-pb.yaw))>c.heading_rad||
       std::abs(pa.kappa-pb.kappa)>c.curvature_inv_m)return false;
  }
  return true;
}
LaneReferenceExecution::LaneReferenceExecution(std::unique_ptr<FrenetPlanner> source,
    std::unique_ptr<FrenetPlanner> target,std::shared_ptr<const LaneReference> lane,
    HandoverOptions alignment,HandoverContinuity continuity,double end_guard,double arrival)
    :owned_source_(std::move(source)),owned_target_(std::move(target)),
     current_(owned_source_.get()),staged_(owned_target_.get()),lane_(std::move(lane)),
     gate_(alignment),alignment_(alignment),continuity_(continuity),end_guard_s_(end_guard),arrival_s_(arrival) {
  initialize(true);
}
LaneReferenceExecution::LaneReferenceExecution(FrenetPlanner& source,FrenetPlanner& target,
    std::shared_ptr<const LaneReference> lane,HandoverOptions alignment,
    HandoverContinuity continuity,double end_guard,double arrival,bool install)
    :current_(&source),staged_(&target),lane_(std::move(lane)),gate_(alignment),
     alignment_(alignment),continuity_(continuity),end_guard_s_(end_guard),arrival_s_(arrival) {
  initialize(install);
}
void LaneReferenceExecution::initialize(bool install) {
  finite_target_=install;
  if(!current_||!staged_||!lane_||!positive(end_guard_s_)||
     !positive(current_->sample_dt())||std::abs(current_->sample_dt()-staged_->sample_dt())>1e-9||
     !positive(continuity_.overlap_s)||!positive(continuity_.position_m)||!positive(continuity_.heading_rad)||
     !positive(continuity_.curvature_inv_m)||!positive(continuity_.speed_mps)||!positive(continuity_.acceleration_mps2)||
     (install&&!staged_->SetPreparedReference(lane_->line())))throw std::invalid_argument("invalid prepared reference execution");
  source_=current_;target_=staged_;
  ReferenceHandoverGate validation(alignment_);
  if(!validation.begin(1,1,lane_,arrival_s_))throw std::invalid_argument("invalid reference alignment policy");
  mission_=current_->reference();
}
void LaneReferenceExecution::resetDormant() {
  current_=source_;staged_=target_;frame_.reset();pending_.reset();command_id_=0;++epoch_;
  gate_=ReferenceHandoverGate(alignment_);
  target_->ResetProjectionHistory();
}
bool LaneReferenceExecution::begin(std::uint32_t id,std::uint64_t source_epoch) {
  if(source_epoch&&source_epoch!=epoch_) {
    if(command_id_==id||adopted())return false;
    epoch_=source_epoch;
  }
  if(adopted()||!gate_.begin(id,epoch_,lane_,arrival_s_))return false;
  command_id_=id; pending_.reset(); return true;
}
bool LaneReferenceExecution::validInput(const ReferenceExecutionInput& i) const {
  const auto& o=i.observation;
  if(i.sequence==0||i.map!=lane_->identity()||!o.ego_valid||!o.objects_valid||
     !fresh(o.now_s,o.ego_stamp_s,alignment_.ego_max_age_s)||
     !fresh(o.now_s,o.objects_stamp_s,alignment_.objects_max_age_s))return false;
  if(frame_&&(i.sequence<frame_->input_sequence||o.ego_stamp_s<frame_->ego_stamp_s||
      o.objects_stamp_s<frame_->objects_stamp_s))return false;
  const auto& e=o.ego;
  if(!std::isfinite(e.x)||!std::isfinite(e.y)||!std::isfinite(e.yaw)||
     !std::isfinite(e.v)||e.v<0||!std::isfinite(e.a)||!std::isfinite(e.kappa))return false;
  std::unordered_set<int> ids;
  for(const auto& object:i.objects) {
    if(object.id<0||!ids.insert(object.id).second||!object.geometry_valid||!object.z_valid||
       !std::isfinite(object.x)||!std::isfinite(object.y)||!std::isfinite(object.heading)||
       !std::isfinite(object.vx)||!std::isfinite(object.vy))return false;
    for(const auto& p:object.corners)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
    if(!geometry::ValidateCorners8(object.corners).valid||!geometry::ComputeZRange(object.corners).valid)return false;
  }
  return true;
}
bool LaneReferenceExecution::build(FrenetPlanner& core,const ReferenceExecutionInput& i,
    double desired,ActiveReferenceFrame& f,std::string& detail) {
  if(!validInput(i)||!positive(desired)){detail="invalid_or_stale_reference_input";return false;}
  const auto& ego=i.observation.ego;
  const auto projection=[&](double x,double y,double yaw,double vx,double vy) {
    return finite_target_?ProjectOnLaneReference(*lane_,x,y,yaw,vx,vy):
        project(core.reference(),x,y,yaw,vx,vy);
  };
  f.ego=projection(ego.x,ego.y,ego.yaw,ego.v*std::cos(ego.yaw),ego.v*std::sin(ego.yaw));
  if(!f.ego.valid||(finite_target_&&(f.ego.s<lane_->usableBegin()||f.ego.s>lane_->usableEnd()))){
    detail="ego_outside_reference_horizon";return false;
  }
  double nearest=1e100;
  for(const auto& o:i.objects) {
    const auto p=projection(o.x,o.y,o.heading,o.vx,o.vy);
    if(!p.valid){detail="object_projection_failed";return false;}
    f.objects.push_back({o.id,p});
    double d_min=1e100,d_max=-1e100,s_min=1e100;
    for(const auto& corner:o.corners) {
      const auto q=projection(corner.x,corner.y,o.heading,0,0);
      d_min=std::min(d_min,q.d);d_max=std::max(d_max,q.d);s_min=std::min(s_min,q.s);
    }
    // OBB overlap with ego's lane corridor; collision testing still sees ALL
    // objects, including unknown/static objects and adjacent-lane vehicles.
    const double half=lane_->options().vehicle.half_width_m+lane_->options().footprint_margin_m;
    const double gap=s_min-f.ego.s-lane_->options().vehicle.front_m;
    const bool following_actor=o.type==int(asmc_planner::ObjectType::VEHICLE)||
        (o.type==int(asmc_planner::ObjectType::UNKNOWN)&&
         (o.motion_state==ObjectMotionState::DYNAMIC||o.motion_state==ObjectMotionState::DYNAMIC_PENDING));
    if(following_actor&&(!finite_target_||p.s<=lane_->usableEnd())&&d_min<=half&&d_max>=-half&&gap>0&&gap<nearest){
      nearest=gap;f.leader_id=o.id;
    }
  }
  PlannerCommand command;command.mode=f.leader_id>=0?FOLLOWING:LANE_KEEPING;
  command.target_object_id=f.leader_id;command.has_target_speed=true;
  command.target_speed=desired;
  // The dormant core needs the same road identity as the future active core.
  // Select from this validated lane chain, never the old reference's station.
  double nearest_lane=INFINITY;
  for(const auto& id:lane_->lanes()) {
    const auto* road=lane_->map()->lane(id);
    if(!road)continue;
    for(const auto& point:road->centerline) {
      const double distance=std::hypot(point.x-ego.x,point.y-ego.y);
      if(distance<nearest_lane){nearest_lane=distance;command.source_link_id=road->link_id;}
    }
  }
  // A finite trial reference is never extrapolated beyond its end. Reuse the
  // existing spatial STOP planner early enough for its full planning horizon.
  if(finite_target_&&lane_->usableEnd()-f.ego.s<=ego.v*end_guard_s_+lane_->options().vehicle.front_m){
    command.mode=STOP;command.target_object_id=-1;command.stop_position=lane_->usableEnd();
  }
  core.SetObjectObservationTime(i.observation.objects_stamp_s);
  if(!core.Plan(ego,i.objects,command,f.path)||!pathShape(f.path)){
    detail="target_continuation_generation_or_collision_failed";return false;
  }
  const auto& shape=lane_->options().vehicle;
  for(std::size_t n=0;n<f.path.x.size();++n) {
    const auto p=projection(f.path.x[n],f.path.y[n],f.path.yaw[n],
        f.path.v[n]*std::cos(f.path.yaw[n]),f.path.v[n]*std::sin(f.path.yaw[n]));
    if(!p.valid||(finite_target_&&(p.s<lane_->usableBegin()-1e-6||p.s>lane_->usableEnd()+1e-6))||
       !lane_->coversRoadFootprint(f.path.x[n],f.path.y[n],f.path.yaw[n])||
       lane_->map()->touchesSolid(f.path.x[n],f.path.y[n],f.path.yaw[n],shape,lane_->options().footprint_margin_m)){
      detail="target_continuation_outside_road_or_solid";return false;
    }
  }
  f.mode=command.mode;f.route_s=finite_target_?FindClosestS(mission_,ego.x,ego.y,nullptr,
      lane_->options().reference.ego_projection_search_window):f.ego.s;
  f.input_sequence=i.sequence;f.ego_stamp_s=i.observation.ego_stamp_s;
  f.objects_stamp_s=i.observation.objects_stamp_s;return true;
}
bool LaneReferenceExecution::prepare(const ReferenceExecutionInput& i,const CartesianPath& executing,
    double speed,std::string& detail) {
  pending_.reset();
  if(adopted()||!command_id_){detail="handover_not_bound";return false;}
  if(!validInput(i)){
    auto invalid=i.observation;invalid.objects_valid=false;
    gate_.observe(command_id_,epoch_,i.map,invalid);
    detail="invalid_handover_input";return false;
  }
  const auto geometry=gate_.observe(command_id_,epoch_,i.map,i.observation);
  detail=geometry.detail;
  if(geometry.state!=HandoverGeometryState::GEOMETRY_ELIGIBLE)return false;
  auto candidate=std::make_unique<ActiveReferenceFrame>();
  if(!executing.v.empty())staged_->SeedPreparedReferenceContinuation(executing.v.back());
  if(!build(*staged_,i,speed,*candidate,detail))return false;
  const bool continuous=HandoverPathsContinuous(executing,candidate->path,staged_->sample_dt(),continuity_)||
      (finite_target_&&HandoverPathsGeometricallyContinuous(executing,candidate->path,staged_->sample_dt(),continuity_));
  if(!continuous){
    detail="continuation_not_continuous_with_executing_path";return false;
  }
  candidate->ref_epoch=epoch_+1;pending_=std::move(candidate);detail="handover_prepared";return true;
}
bool LaneReferenceExecution::commit(const ReferenceExecutionInput& latest,
    const std::function<bool()>& complete,std::string& detail) {
  if(adopted()||!pending_||!validInput(latest)||pending_->input_sequence!=latest.sequence||
     pending_->ego_stamp_s!=latest.observation.ego_stamp_s||
     pending_->objects_stamp_s!=latest.observation.objects_stamp_s){detail="handover_input_changed";pending_.reset();return false;}
  detail="reference_and_continuation_adopted";
  if(!complete||!complete()){detail="completion_transition_rejected";pending_.reset();return false;}
  std::swap(current_,staged_);frame_.swap(pending_);++epoch_;
  return true;
}
bool LaneReferenceExecution::continueLane(const ReferenceExecutionInput& i,double speed,std::string& detail) {
  if(!adopted()){detail="no_adopted_reference";return false;}
  auto next=std::make_unique<ActiveReferenceFrame>();
  if(!build(*current_,i,speed,*next,detail))return false;
  next->ref_epoch=epoch_;frame_.swap(next);detail="active_lane_replanned_from_current_snapshot";return true;
}
}  // namespace asmc_planner::lane_reference
