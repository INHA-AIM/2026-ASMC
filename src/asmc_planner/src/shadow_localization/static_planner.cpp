#include "shadow_localization/static_planner.hpp"
#include "shadow_localization/following_policy.hpp"
#include <chrono>
#include <stdexcept>
#include <set>
#include <yaml-cpp/yaml.h>

namespace shadow_localization {
using namespace local_planning;
StaticPlanner::StaticPlanner(const std::string& parameters,bool sequential_targets)
    :parameters_(parameters),sequential_targets_(sequential_targets){reset();}
void StaticPlanner::reset(){
  previous_path_.clear();motion_evidence_.clear();
  core_=std::make_unique<FrenetPlanner>();tracks_.clear();phase_="unavailable";retargets_=0;
  if(!core_->Init(parameters_))throw std::runtime_error("localization core init failed");
  if(!core_->LimitAvoidSearchBudget(.025))
    throw std::runtime_error("localization cooperative search budget unavailable");
  const Config physical;
  if(!core_->SetConservativeVehicleEnvelope({physical.width,physical.length,physical.rear,
      std::max(physical.width,core_->vehicle_shape().road_outer_width)}))
    throw std::runtime_error("localization physical envelope cannot shrink native geometry");
  if(!wall_overlay_.empty()&&!core_->SetShadowWallOverlay(wall_overlay_))throw std::runtime_error("wall overlay reset refused");
  core_->SetShadowWallPermission(wall_permission_);
  set_paint_exemption(paint_exempt_);
}
void StaticPlanner::remember_path(const Result& result,const Motion& origin,const Transform& map){
  if(!continuity_enabled_||!result.valid||!maneuver_active()||result.chosen_cap<=.1||result.path.size()<3)return;
  CartesianPath path;const double c=std::cos(origin.yaw),s=std::sin(origin.yaw);
  for(const auto& p:result.path){
    const auto xy=map.apply(origin.x+c*p.x-s*p.y,origin.y+s*p.x+c*p.y);
    path.x.push_back(xy[0]);path.y.push_back(xy[1]);path.yaw.push_back(p.yaw+origin.yaw+map.yaw);
    path.kappa.push_back(p.k);path.v.push_back(result.chosen_cap);path.a.push_back(0.);
  }
  // A stored verified suffix is a finite stopping-space boundary. Never
  // extrapolate the cached end or turn an old terminal stop into cruise.
  path.v.back()=0.;
  previous_path_.remember(std::move(path),origin.t);
}
bool StaticPlanner::stationary(const Obstacle& ob,const Motion& origin){
  auto& h=tracks_[ob.id];
  const auto& o=ob.observation;
  if(!h.empty()&&ob.stamp<h.back().t)h.clear();
  if(h.empty()||ob.stamp>h.back().t){
    // Partial faces change fitted centre, axis order and size while occupying
    // the same fixed volume. Use the already leveled authoritative hull;
    // rotate into odometry only once per distinct scan, with no second tilt.
    const double cs=std::cos(origin.yaw),sn=std::sin(origin.yaw);
    Sample q{ob.stamp,INFINITY,-INFINITY,INFINITY,-INFINITY};
    for(const auto& p:ob.polygon){
      const double x=origin.x+cs*p.x-sn*p.y,y=origin.y+sn*p.x+cs*p.y;
      q.xmin=std::min(q.xmin,x);q.xmax=std::max(q.xmax,x);
      q.ymin=std::min(q.ymin,y);q.ymax=std::max(q.ymax,y);
    }
    if(!std::isfinite(q.xmin+q.xmax+q.ymin+q.ymax))return false;
    h.push_back(q);
    while(h.size()>1&&q.t-h.front().t>1.)h.pop_front();
  }
  std::vector<asmc_geometry::motion_evidence::Point> shape;
  const double ca=std::cos(origin.yaw),sa=std::sin(origin.yaw);
  for(auto p:ob.polygon)shape.push_back({origin.x+ca*p.x-sa*p.y,origin.y+sa*p.x+ca*p.y});
  const auto& q=h.back();
  const auto evidence=motion_evidence_.update("localization_odom",ob.id,ob.stamp,
      {(.5*(q.xmin+q.xmax)),(.5*(q.ymin+q.ymax))},0,0,true,nullptr,&shape);
  // Rest is evidence from distinct, ego-compensated observations, never a
  // consequence of unavailable absolute velocity or a static-looking class.
  if(h.size()<4||h.back().t-h.front().t<.5)return false;
  double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY,max_x=0,max_y=0;
  for(const auto& p:h){
    xmin=std::min(xmin,p.xmin);xmax=std::max(xmax,p.xmax);
    ymin=std::min(ymin,p.ymin);ymax=std::max(ymax,p.ymax);
    max_x=std::max(max_x,p.xmax-p.xmin);max_y=std::max(max_y,p.ymax-p.ymin);
  }
  // Uniform translation expands the union beyond every observed extent and
  // is rejected. Revealing another face inside the observed extent is not
  // motion. Keep the former 15cm support-displacement tolerance.
  if(std::hypot(std::max(0.,xmax-xmin-max_x),std::max(0.,ymax-ymin-max_y))>.15)return false;
  const bool vulnerable=ob.camera_class=="person"||ob.camera_class=="dog";
  return !vulnerable||(evidence.valid&&evidence.stationary);
}
bool StaticPlanner::observed_velocity(int id,double& vx,double& vy) const {
  const auto found=tracks_.find(id);if(found==tracks_.end())return false;
  const auto& h=found->second;if(h.size()<4)return false;
  const auto& first=h.front();const auto& last=h.back();const double dt=last.t-first.t;
  if(dt<.3)return false;
  if(std::abs((last.xmax-last.xmin)-(first.xmax-first.xmin))>.15||
     std::abs((last.ymax-last.ymin)-(first.ymax-first.ymin))>.15)return false;
  vx=((last.xmin+last.xmax)-(first.xmin+first.xmax))/(2*dt);
  vy=((last.ymin+last.ymax)-(first.ymin+first.ymax))/(2*dt);
  if(!std::isfinite(vx+vy)||std::hypot(vx,vy)>15.)return false;
  for(const auto& p:h){const double elapsed=p.t-first.t;
    if(std::hypot((p.xmin+p.xmax-first.xmin-first.xmax)/2-vx*elapsed,
                  (p.ymin+p.ymax-first.ymin-first.ymax)/2-vy*elapsed)>.10)return false;}
  return true;
}
// One swept-footprint boundary for reference tracking, native bypass and return.
// Semantic coverage is checked only when supplied by an actual caller. The
// map/OBB route never manufactures a free semantic raster from missing input.
bool StaticPlanner::footprint(const Scene& scene,const Config& cfg,const Pose& p,
                             Result& out,const Motion& origin,const Transform& map) const {
  if(scene.grid.rows>0 && !CheckFootprint(scene,cfg,p,out))return false;
  const VehicleShape shape{cfg.width,cfg.length,cfg.rear};
  CartesianState local{p.x,p.y,p.yaw,p.k,0,0};
  const auto physical=MakeEgoFootprint(local,shape,0.);
  for(const auto& ob:scene.obstacles){
    // A trusted moving hull is checked at trajectory time below. Treating
    // every future pose against its present hull would reject a crossing
    // that clears before arrival. Unknown motion retains the static bound.
    if(in_plan_&&dynamic_enabled_&&moving_ids_.count(ob.id))continue;
    const double distance=geometry::ConvexPolygonDistance(physical,ob.polygon);
    out.minimum_clearance=std::min(out.minimum_clearance,distance);
    if(distance<=cfg.margin){out.blocking_id=ob.id;out.reason="localization_obb";return false;}
  }
  const double cs=std::cos(origin.yaw),sn=std::sin(origin.yaw);
  const auto xy=map.apply(origin.x+cs*p.x-sn*p.y,origin.y+sn*p.x+cs*p.y);
  if(!core_->road_map().FootprintInside(xy[0],xy[1],p.yaw+origin.yaw+map.yaw,shape)){
    out.blocked_x=p.x;out.blocked_y=p.y;out.reason="localization_map_boundary";return false;
  }
  return true;
}
bool StaticPlanner::sweep(const Scene& scene,const Config& cfg,const Pose& from,
                         const Pose& to,Result& out,const Motion& origin,const Transform& map) const {
  const int n=std::max(1,int(std::ceil((std::hypot(to.x-from.x,to.y-from.y)+
                                     cfg.length*std::abs(to.yaw-from.yaw))/.1)));
  for(int i=1;i<=n;++i){const double q=double(i)/n;
    if(!footprint(scene,cfg,Pose{from.x+q*(to.x-from.x),from.y+q*(to.y-from.y),
            from.yaw+q*(to.yaw-from.yaw),to.k,0},out,origin,map))return false;
  }
  return true;
}
Result StaticPlanner::verify(const CartesianPath& path,const Scene& scene,
                            const Config& cfg,const Motion& origin,const Transform& map){
  Result out;
  std::vector<double> limits;
  bool reached_end=true;
  if(path.x.size()<2||path.y.size()!=path.x.size()||path.yaw.size()!=path.x.size()||
     path.kappa.size()!=path.x.size()||path.v.size()!=path.x.size()){
    out.reason="shadow_native_path_invalid";return out;
  }
  const double cs=std::cos(origin.yaw),sn=std::sin(origin.yaw);
  // Validate the exact map candidate in the current observed local road/OBBs.
  // Stop at the end of observable free space, then require stopping distance.
  for(size_t i=0;i<path.x.size();++i){
    auto xy=map.inverse(path.x[i],path.y[i]);double dx=xy[0]-origin.x,dy=xy[1]-origin.y;
    Pose p{cs*dx+sn*dy,-sn*dx+cs*dy,
      std::remainder(path.yaw[i]-map.yaw-origin.yaw,2*M_PI),path.kappa[i],0};
    if(!std::isfinite(p.x+p.y+p.yaw+p.k+path.v[i])||path.v[i]<0||std::abs(p.k)>cfg.max_k){
      out.reason="shadow_native_nonfinite";return out;
    }
    if(!out.path.empty()){
      p.yaw=out.path.back().yaw+std::remainder(p.yaw-out.path.back().yaw,2*M_PI);
      const double ds=std::hypot(p.x-out.path.back().x,p.y-out.path.back().y);
      if(ds<1e-5)continue;
      p.s=out.path.back().s+ds;
      if(!sweep(scene,cfg,out.path.back(),p,out,origin,map)){reached_end=false;break;}
    }else{
      // Never join a remotely starting candidate with an unchecked segment.
      if(std::hypot(p.x,p.y)>.35||!footprint(scene,cfg,p,out,origin,map)){
        out.reason="shadow_start_unverified";return out;
      }
    }
    out.path.push_back(p);
    limits.push_back(path.v[i]);
  }
  // A native moving horizon is not a stopping-space boundary. Extend only
  // after its entire suffix was checked, never through a rejected sample or
  // past an explicit native stop. The extension passes BOTH native map road
  // containment and the same physical local sweep, and ends in a new stop.
  if(reached_end&&!out.path.empty()&&path.v.back()>.1){
    const VehicleShape shape{cfg.width,cfg.length,cfg.rear};
    while(out.path.back().s<cfg.max_range){
      const auto& last=out.path.back();Pose p=last;const double ds=.20;
      p.x+=ds*std::cos(p.yaw+.5*p.k*ds);p.y+=ds*std::sin(p.yaw+.5*p.k*ds);
      p.yaw+=p.k*ds;p.s+=ds;
      auto xy=map.apply(origin.x+cs*p.x-sn*p.y,origin.y+sn*p.x+cs*p.y);
      if(!core_->road_map().FootprintInside(xy[0],xy[1],p.yaw+origin.yaw+map.yaw,shape)||
         !sweep(scene,cfg,last,p,out,origin,map))break;
      out.path.push_back(p);
      limits.push_back(path.v.back());
    }
  }
  if(out.path.size()<3){out.reason="shadow_no_verified_prefix";return out;}
  Config capped=cfg;
  capped.speed_cap=std::min(cfg.speed_cap,*std::max_element(path.v.begin(),path.v.end()));
  if(!std::isfinite(capped.speed_cap)||capped.speed_cap<0){out.reason="shadow_speed_invalid";return out;}
  if(out.reason!="no_path")out.rejected[out.reason]++;
  ++speed_profiles_;return TimeParameterize(scene,capped,std::move(out),
      moving_objects_?limits:std::vector<double>{});
}
Result StaticPlanner::plan(const Scene& scene,const Config& cfg,const Motion& origin,const Transform& map){
  struct Scope {bool& flag;Scope(bool& f):flag(f){flag=true;}~Scope(){flag=false;}} scope(in_plan_);
  const auto start=std::chrono::steady_clock::now();
  native_calls_=speed_profiles_=0;
  confirmed_rest_=generated_=feasible_=0;candidate_reason_="not_requested";
  transition_distance_=return_distance_=0;
  moving_objects_=pending_motion_objects_=0;moving_ids_.clear();
  rest_motion_disagreements_=0;
  reused_path_=false;continuity_reason_=continuity_enabled_?"not_needed":"disabled";
  following_target_=-1;following_gap_=-1;
  Result failure;
  if(!map.valid){failure.reason="shadow_anchor_unavailable";return failure;}
  auto xy=map.apply(origin.x,origin.y);
  CartesianState ego{xy[0],xy[1],origin.yaw+map.yaw,0,scene.speed,scene.acceleration};
  double s=0;
  if(!core_->ProjectEgoToS(ego,s)){failure.reason="shadow_route_unavailable";return failure;}
  std::vector<TrackedObject> objects;
  std::set<int> object_ids;
  double nearest=40.;int target=-1;
  double leader_distance=INFINITY,leader_reference_gap=0.;int leader=-1;
  const int active_target=core_->active_avoid_target_id();
  const std::string active_phase=core_->avoid_phase_name();
  bool active_target_behind_body=false;
  const double yaw=ego.yaw,cs=std::cos(yaw),sn=std::sin(yaw);
  const double observation_time=scene.obstacles.empty()?origin.t:scene.obstacles.front().stamp;
  for(const auto& ob:scene.obstacles){
    auto o=ob.observation;
    if(std::abs(ob.stamp-observation_time)>1e-6){failure.reason="shadow_mixed_observation_times";return failure;}
    if(origin.t-ob.stamp>.3||ob.stamp-origin.t>.05){failure.reason="shadow_object_time_unaligned";return failure;}
    if(ob.id<0||!object_ids.insert(ob.id).second||!std::isfinite(ob.stamp)||ob.stamp<=0||
       !o.geometry_valid||!o.z_valid||!std::isfinite(o.x+o.y+o.z+o.width+o.length+o.height)||
       o.width<=0||o.length<=0||o.height<=0){failure.reason="shadow_object_geometry";return failure;}
    for(const auto& p:o.corners)if(!std::isfinite(p.x+p.y+p.z)){
      failure.reason="shadow_object_geometry";return failure;
    }
    const bool support_rest=stationary(ob,origin);
    const bool producer_dynamic=o.motion_state==ObjectMotionState::DYNAMIC;
    double vx=0,vy=0;
    const bool fit=observed_velocity(ob.id,vx,vy);
    // A motion bit/applied KF velocity is an estimate, not an immutable
    // semantic class. Contradict it only with the existing >=0.5s rest
    // support AND a stable finite hull fit, never one stopped observation.
    const bool rest=support_rest&&((!fit&&!producer_dynamic)||(fit&&std::hypot(vx,vy)<=.1));
    rest_motion_disagreements_+=rest&&producer_dynamic;
    const bool velocity=dynamic_enabled_&&fit;
    const bool dynamic_hint=producer_dynamic || ob.camera_class=="person" || ob.camera_class=="dog" ||
        (fit&&std::hypot(vx,vy)>.1);
    if(dynamic_hint&&!rest&&!velocity&&o.x>-cfg.rear&&o.x<cfg.max_range&&std::abs(o.y)<6.){
      if(!dynamic_enabled_){failure.reason="shadow_dynamic_not_supported";return failure;}
      ++pending_motion_objects_; // Continue only along a verified stopping prefix.
    }
    const bool moving=velocity&&!rest&&(dynamic_hint||std::hypot(vx,vy)>.1);
    confirmed_rest_+=rest&&!moving;
    if(ob.id==active_target&&!ob.polygon.empty()){
      const double forward=std::max_element(ob.polygon.begin(),ob.polygon.end(),
          [](const auto& a,const auto& b){return a.x<b.x;})->x;
      // Current observed hull wholly behind the rear bumper, plus a metre
      // and the local clearance margin. Missing observations never retire it.
      active_target_behind_body=forward < -cfg.rear-1.-cfg.margin;
    }
    for(auto& corner:o.corners){double x=corner.x,y=corner.y;corner.x=ego.x+cs*x-sn*y;corner.y=ego.y+sn*x+cs*y;}
    double x=o.x,y=o.y;o.x=ego.x+cs*x-sn*y;o.y=ego.y+sn*x+cs*y;o.heading+=yaw;
    o.velocity_unavailable=!moving;o.vx=o.vy=o.speed=0;
    o.stationary_observed=rest&&!moving;
    o.motion_state=moving?ObjectMotionState::DYNAMIC:rest?ObjectMotionState::LOW_THREAT:ObjectMotionState::UNKNOWN;
    if(moving){
      const double c=std::cos(map.yaw),sine=std::sin(map.yaw);
      o.vx=c*vx-sine*vy;o.vy=sine*vx+c*vy;o.speed=std::hypot(vx,vy);++moving_objects_;
      moving_ids_.insert(ob.id);
    }
    objects.push_back(o);
    double os=0,od=0;
    const double lateral=-sn*o.vx+cs*o.vy,along=cs*o.vx+sn*o.vy;
    const bool slow_vehicle=moving&&o.speed<=1.&&along>=-.05&&std::abs(lateral)<.2;
    if(moving&&along>.1&&std::abs(lateral)<.2){
      double rear=INFINITY,left=INFINITY,right=-INFINITY;
      for(const auto& p:o.corners){const double dx=p.x-ego.x,dy=p.y-ego.y;
        rear=std::min(rear,cs*dx+sn*dy);left=std::min(left,-sn*dx+cs*dy);right=std::max(right,-sn*dx+cs*dy);}
      const double center=cs*(o.x-ego.x)+sn*(o.y-ego.y),gap=rear-(cfg.length-cfg.rear);
      const auto desired=FollowingReferenceGap(cfg.length-cfg.rear,center,rear,following_bumper_gap_);
      if(desired&&gap>0&&gap<cfg.max_range&&gap<leader_distance&&
         left<cfg.width*.5+cfg.margin&&right>-cfg.width*.5-cfg.margin){
        leader=ob.id;leader_distance=gap;leader_reference_gap=*desired;
      }
    }
    if(((rest&&!moving)||slow_vehicle)&&core_->ProjectPointToFrenet(o.x,o.y,s,60.,6.,os,od)&&
       os-s>0&&os-s<nearest&&std::abs(od)<.5*(cfg.width+o.width)+cfg.margin){
      nearest=os-s;target=o.id;
    }
  }
  for(auto i=tracks_.begin();i!=tracks_.end();)
    if(i->second.empty()||origin.t-i->second.back().t>2.)i=tracks_.erase(i);else ++i;
  // Preserve original observation positions/history. The native derived
  // collision vector advances once to the ego timestamp, never by scene.age
  // again after already compensating the observation-to-ego interval.
  core_->SetObjectObservationTime(observation_time,origin.t);
  auto temporal_check=[&](Result result,bool brake_cruise=false){
    if(!result.valid||!moving_objects_)return result;
    CartesianPath timed;
    const double local_cs=std::cos(origin.yaw),local_sn=std::sin(origin.yaw);
    for(const auto& p:result.trajectory){const auto xy=map.apply(origin.x+local_cs*p[0]-local_sn*p[1],origin.y+local_sn*p[0]+local_cs*p[1]);
      timed.x.push_back(xy[0]);timed.y.push_back(xy[1]);timed.yaw.push_back(yaw+p[2]);
      timed.kappa.push_back(p[3]);timed.v.push_back(p[4]);timed.a.push_back(p[5]);}
    auto aged=objects;for(auto& object:aged)if(!object.velocity_unavailable)
      object=AdvanceObjectObservation(object,std::max(0.,origin.t-observation_time));
    auto check=EvaluateCartesianTrajectorySafety(timed,aged,core_->vehicle_shape(),core_->collision_config(),.05);
    const bool road=core_->road_map().EvaluatePath(timed,core_->vehicle_shape(),.05,true).drivable;
    // A conflict late in a cruise prediction is not permission to creep into
    // it, nor does it erase the already checked free prefix. Try ONE terminal
    // braking profile on that prefix, then check the entire new timed body
    // against the same predictions/margins again. No second lattice/search.
    if(!check.clear&&road&&check.blocking_sample_index>1&&
       check.blocking_sample_index<int(result.trajectory.size())&&result.path.size()>3){
      const auto& hit=result.trajectory[check.blocking_sample_index];
      const auto nearest=std::min_element(result.path.begin(),result.path.end(),[&](const auto& a,const auto& b){
        return std::hypot(a.x-hit[0],a.y-hit[1])<std::hypot(b.x-hit[0],b.y-hit[1]);});
      const double stop_s=nearest->s-std::max(1.5,.15*scene.speed);
      Result prefix=result;prefix.path.erase(std::remove_if(prefix.path.begin(),prefix.path.end(),
          [&](const auto& p){return p.s>stop_s;}),prefix.path.end());
      prefix.trajectory.clear();prefix.valid=false;
      Config braking=cfg;
      if(brake_cruise)braking.speed_cap=0.;
      ++speed_profiles_;auto yield=TimeParameterize(scene,braking,std::move(prefix));
      if(yield.valid){
        CartesianPath stopped;
        for(const auto& p:yield.trajectory){const auto xy=map.apply(origin.x+local_cs*p[0]-local_sn*p[1],origin.y+local_sn*p[0]+local_cs*p[1]);
          stopped.x.push_back(xy[0]);stopped.y.push_back(xy[1]);stopped.yaw.push_back(yaw+p[2]);
          stopped.kappa.push_back(p[3]);stopped.v.push_back(p[4]);stopped.a.push_back(p[5]);}
        const auto safety=EvaluateCartesianTrajectorySafety(stopped,aged,core_->vehicle_shape(),core_->collision_config(),.05);
        if(safety.clear&&core_->road_map().EvaluatePath(stopped,core_->vehicle_shape(),.05,true).drivable){
          yield.reason="verified_dynamic_stopping_prefix";
          yield.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
          return yield;
        }
      }
    }
    if(!check.clear||!road){result.valid=false;result.reason=!road?"shadow_dynamic_road_conflict":"shadow_dynamic_temporal_conflict";result.blocking_id=check.blocking_object_id;result.trajectory.clear();}
    return result;
  };
  if(sequential_targets_&&target>=0&&target!=active_target&&active_target_behind_body&&
      core_->avoid_committed()&&!core_->avoid_completed()&&
      (active_phase=="PASS"||active_phase=="RETURN")){
    // Do not force a centreline return into a second observed static object.
    // Retire only the completely passed target and PREPARE from the measured
    // current state. Both old and new OBBs remain in every native/local check;
    // no previous path is relabelled, and only one native search runs.
    core_->ResetExternalAvoidance();
    ++retargets_;
  }
  PlannerCommand command{};command.has_target_speed=true;command.target_speed=cfg.speed_cap;
  CartesianPath path;FrenetPlanner::AvoidPrepareResult prepared;
  bool ok=false;
  if(core_->avoid_committed()&&!core_->avoid_completed()){
    command.mode=AVOID;command.target_object_id=core_->active_avoid_target_id();
    ++native_calls_;
    ok=core_->PlanCommittedAvoid(ego,objects,command,path,prepared);
    generated_=prepared.generated_candidate_count;feasible_=prepared.valid_candidate_count;
    transition_distance_=prepared.map_transition_distance;
    return_distance_=prepared.map_return_schedule_valid?std::max(0.,prepared.map_return_end_s-prepared.map_return_start_s):0.;
    phase_=core_->avoid_phase_name();
  }else if(target>=0){
    if(core_->avoid_completed())core_->ResetExternalAvoidance();
    command.mode=AVOID;command.target_object_id=target;
    // Native search already evaluates both sides with its cooperative budget.
    // Use the same conservative envelope in generation and final verification.
    ++native_calls_;
    ok=core_->PrepareAvoid(ego,objects,command,path,prepared);phase_="PREPARE";
    transition_distance_=prepared.map_transition_distance;
    return_distance_=prepared.map_return_schedule_valid?std::max(0.,prepared.map_return_end_s-prepared.map_return_start_s):0.;
    generated_=prepared.generated_candidate_count;feasible_=prepared.valid_candidate_count;
    if(prepared.candidate_valid){
      auto candidate=temporal_check(verify(prepared.candidate_path,scene,cfg,origin,map));
      candidate_reason_=candidate.reason;
      // Do not bind a maneuver whose FULL retimed candidate failed. A shorter
      // stopping witness can approach safely while PREPARE searches, but is
      // not evidence that the bypass itself can be committed.
      if(candidate.valid&&candidate.reason!="verified_dynamic_stopping_prefix"&&core_->CommitAvoid(prepared)){
        phase_=core_->avoid_phase_name();
        remember_path(candidate,origin,map);
        candidate.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        return candidate;
      }
      failure.rejected["native_candidate_"+candidate.reason]++;
      failure.rejected.insert(candidate.rejected.begin(),candidate.rejected.end());
    }else {failure.rejected["native_candidate_missing"]++;candidate_reason_="native_candidate_missing";}

  }else if(moving_objects_){
    // A clear timed reference needs no duplicate KEEP lattice. Reuse native
    // KEEP/FOLLOW only when the current occupancy actually constrains it.
    if(leader<0){
      auto cruise=temporal_check(reference(scene,cfg,origin,map),true);
      if(cruise.valid){phase_="DYNAMIC_REFERENCE";
        cruise.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        return cruise;}
    }
    command.mode=leader>=0?FOLLOWING:LANE_KEEPING;
    if(leader>=0){command.target_object_id=leader;command.min_gap=leader_reference_gap;
      command.time_gap=.01;following_target_=leader;following_gap_=leader_distance;}
    ++native_calls_;ok=core_->Plan(ego,objects,command,path);phase_=leader>=0?"DYNAMIC_FOLLOW":"DYNAMIC_KEEP";
    generated_=core_->last_candidate_paths().size();
    feasible_=std::count(core_->last_candidate_validity().begin(),core_->last_candidate_validity().end(),true);
  }else{
    if(core_->avoid_completed())core_->ResetExternalAvoidance();
    // No active maneuver needs a second KEEP lattice. Reuse the same fixed
    // reference and the common map/OBB sweep, with one time profile.
    auto result=reference(scene,cfg,origin,map);phase_=pending_motion_objects_?"MOTION_PENDING_REFERENCE":"KEEP";
    if(pending_motion_objects_&&result.valid)result.reason="verified_motion_pending_reference";
    return result;
  }
  Result result;
  transition_distance_=prepared.map_transition_distance;
  return_distance_=prepared.map_return_schedule_valid?std::max(0.,prepared.map_return_end_s-prepared.map_return_start_s):0.;
  if(ok)result=verify(path,scene,cfg,origin,map);
  else result.reason=moving_objects_?"shadow_dynamic_yield":"shadow_native_blocked";
  if(!ok){
    switch(core_->last_failure_reason()){
      case FrenetPlanner::PlanFailureReason::TARGET_OBJECT_LOST:candidate_reason_="native_target_observation_lost";break;
      case FrenetPlanner::PlanFailureReason::ROAD_BOUNDARY_BLOCKED:candidate_reason_="native_road_boundary_blocked";break;
      case FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED:candidate_reason_="native_collision_blocked";break;
      case FrenetPlanner::PlanFailureReason::TRANSIENT_GENERATION_FAILURE:candidate_reason_="native_generation_failed";break;
      case FrenetPlanner::PlanFailureReason::INVALID_COMMAND:candidate_reason_="native_command_invalid";break;
      default:candidate_reason_="native_unavailable";break;
    }
  }
  if(!ok&&moving_objects_&&(phase_=="DYNAMIC_KEEP"||phase_=="DYNAMIC_FOLLOW")){
    const auto failure=core_->last_failure_reason();
    if(failure==FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED||
       failure==FrenetPlanner::PlanFailureReason::TRANSIENT_GENERATION_FAILURE){
      if(!path.x.empty()){
        auto braking=verify(path,scene,cfg,origin,map);
        if(braking.valid){braking.reason="verified_dynamic_yield";result=std::move(braking);}
      }
      if(!result.valid&&std::abs(scene.speed)<.15){
        Result hold;
        if(footprint(scene,cfg,Pose{},hold,origin,map)){
          hold.valid=true;hold.reason="verified_dynamic_wait";hold.path.push_back(Pose{});
          for(int i=0;i<41;++i)hold.trajectory.push_back({0,0,0,0,0,0,i*.05});
          result=std::move(hold);
        }
      }
    }
  }
  // Native PREPARE may expose only a short hold while its cooperatively
  // sliced search continues. A collision-checked stopping prefix of the fixed
  // reference can retain a speed justified by its verified stopping distance.
  // A fixed 3km/h cap can invalidate a feasible paint-time-bounded bypass by
  // braking before COMMIT. The same body/map/OBBs, ages and stopping bounds
  // apply. Never substitute this for a failed committed maneuver or dynamics.
  if(phase_=="PREPARE"&&(!result.valid||result.chosen_cap<.1||result.path.empty()||
      result.path.back().s<std::min(10.,cfg.max_range*.5))){
    const auto native=native_calls_,timing=speed_profiles_;
    auto approach=reference(scene,cfg,origin,map);
    native_calls_+=native;speed_profiles_+=timing;phase_="WAITING_REFERENCE";
    if(approach.valid&&!approach.path.empty()&&
       std::hypot(approach.path.front().x,approach.path.front().y)<=.35&&
       approach.chosen_cap>.05&&
       (!result.valid||result.chosen_cap<.1||result.path.empty()||
        approach.path.back().s>result.path.back().s+1.)){
      approach.reason="verified_waiting_reference";result=std::move(approach);
    }
  }
  result.rejected.insert(failure.rejected.begin(),failure.rejected.end());

  result.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  const bool braking_reference=phase_=="DYNAMIC_KEEP"||phase_=="DYNAMIC_FOLLOW";
  auto checked=temporal_check(std::move(result),braking_reference);
  if(checked.valid)remember_path(checked,origin,map);
  else if(continuity_enabled_&&native_calls_&&maneuver_active()){
    auto remaining=previous_path_.remaining(ego,origin.t,continuity_reason_);
    if(remaining){
      auto previous=temporal_check(verify(*remaining,scene,cfg,origin,map));
      if(previous.valid){
        reused_path_=true;continuity_reason_="current_scene_revalidated";
        previous.reason="verified_previous_path";checked=std::move(previous);
      }else continuity_reason_="current_scene_"+previous.reason;
    }
  }
  checked.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  return checked;
}
} // namespace shadow_localization

namespace shadow_localization {
void StaticPlanner::load_wall_corridor(const std::string& path){
  const auto file=YAML::LoadFile(path);
  if(file["schema"].as<std::string>()!="asmc.localization.wall_corridor/v1" ||
     file["frame_id"].as<std::string>()!="map" || file["simulator"].as<std::string>()!="25.S4.MolitComp03" ||
     file["base_product_sha256"].as<std::string>()!=core_->road_map().product_sha256() ||
     file["wall_reserve_m"].as<double>()<.65)
    throw std::runtime_error("wall corridor map/frame/simulator/reserve mismatch");
  std::vector<geometry::Polygon2D> areas;double previous=-INFINITY;
  for(const auto& segment:file["segments"]){
    const double a=segment["station_start"].as<double>(),b=segment["station_end"].as<double>();
    if(!std::isfinite(a+b)||a<1877.744798966||b>1996.730791531||a<previous||b<=a)throw std::runtime_error("wall station scope mismatch");
    previous=b;geometry::Polygon2D polygon;
    for(const auto& xy:segment["polygon"])polygon.push_back({xy[0].as<double>(),xy[1].as<double>()});
    areas.push_back(std::move(polygon));
  }
  if(areas.empty()||!core_->SetShadowWallOverlay(areas))throw std::runtime_error("wall corridor geometry refused");
  wall_overlay_=std::move(areas);
}
}
