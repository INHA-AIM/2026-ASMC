#include "Frenet/frenet_planner.hpp"
#include "Frenet/Generator/StaticAvoidPath.hpp"
#include "Frenet/Geometry/AvoidanceGeometry.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include <algorithm>
#include <cmath>

bool FrenetPlanner::AcquireStaticAvoidGroup(double s,
    const std::vector<TrackedObject>& objects,const PlannerCommand& command) {
    if(!avoid_road_map_.enabled()||!avoid_cfg_.static_margin_recovery_enabled||
       command.avoid_in_lane_only||v2_avoidance_.dynamic_target)return false;
    const auto primary=std::find_if(objects.begin(),objects.end(),[&](const auto& o){
        return o.id==command.target_object_id;
    });
    if(primary!=objects.end()&&!IsStaticAvoidProgressObject(*primary))return false;
    const TrackedObject* nearest=nullptr;double best=INFINITY;
    for(const auto& o:objects) {
        if(!IsStaticAvoidProgressObject(o))continue;
        double os=0,od=0,ov=0;
        ProjectObjectToFrenet(ref_,o,os,od,ov,ref_config_.ego_projection_search_window);
        const auto bounds=ProjectAvoidFootprint(o.corners,o.x,o.y,Interpolate(ref_,os).theta);
        const double gap=ReferenceArcDistance(ref_,s,os);
        const double corridor=vehicle_shape_.width*.5+collision_cfg_.safety_margin;
        if(!bounds.valid||gap+bounds.far_s<-vehicle_shape_.rear_overhang||
           gap+bounds.near_s>EffectiveAvoidDetectionDistance()||
           od+bounds.min_d>corridor||od+bounds.max_d<-corridor)continue;
        const double cost=std::max(0.0,gap+bounds.near_s)+.1*std::fabs(od);
        if(cost<best){nearest=&o;best=cost;}
    }
    if(!nearest)return false;
    // The command remains the execution owner. An observation ID only supplies
    // current geometry; it is never the permission to enter static recovery.
    execution_.bindAvoidCommandTarget(command.target_object_id);
    RefreshAvoidObservation(s,*nearest,v2_avoidance_,false);
    execution_.observeAvoidVisible();v2_avoidance_.static_group_recovery=true;
    return true;
}

bool FrenetPlanner::PlanStaticAvoidCreep(const CartesianState& ego,double s,double d,
    double speed,double initial_a,const std::vector<TrackedObject>& objects,
    const PlannerCommand& command,AvoidPrepareResult& result,
    std::chrono::steady_clock::time_point deadline,const StaticAvoidPolicy& policy) {
    auto& context=v2_avoidance_;
    if(ref_.points.empty()||speed<=.05||policy.mode!=StaticAvoidPolicy::CONTACT_PROGRESS)return false;
    const double cap=std::min({speed,avoid_cfg_.bypass_speed_cap,avoid_cfg_.static_contact_speed_cap});
    double es=s,sd=0,sdd=0,ed=d,prime=0,second=0;
    CartesianToFrenet(ref_,ego,es,sd,sdd,ed,prime,second,&s,ref_config_.ego_projection_search_window);
    if(!std::isfinite(prime)||!std::isfinite(second)||std::fabs(prime)>2)return false;
    // This is a rolling execution horizon, with enough distance for a complete
    // jerk-limited stop from the observed state. No historical path is needed.
    const double horizon=6.0+ego.v*ego.v/(limits_.max_longitudinal_accel*1.6)+ego.v;
    std::vector<double> offsets;
    if(context.static_progress_fresh)offsets.push_back(context.map_recovery_path_target_d);
    offsets.insert(offsets.end(),{d,0.0,d+.25,d-.25,d+.5,d-.5,d+1,d-1});
    if(context.has_requested_target_d)offsets.insert(offsets.begin(),context.requested_target_d);
    std::vector<double> unique;
    for(double offset:offsets)if(std::isfinite(offset)&&std::fabs(offset)<=4&&
        std::none_of(unique.begin(),unique.end(),[&](double v){return std::fabs(v-offset)<1e-6;}))unique.push_back(offset);
    VehicleShape road_shape=vehicle_shape_;
    road_shape.width=std::max(road_shape.width,road_shape.road_outer_width);
    auto limits=limits_;
    limits.max_forward_accel=std::min(limits.max_forward_accel,avoid_cfg_.bypass_accel_cap);
    bool found=false;double best=INFINITY;
    AvoidPrepareResult selected;
    const double primary_gap=ReferenceArcDistance(ref_,s,context.obstacle_s);
    bool attempted=false;
    const auto resume=context.static_creep_cursor;
    for(std::size_t choice=0;choice<unique.size();++choice) {
        if(std::chrono::steady_clock::now()>=deadline&&attempted)break;
        attempted=true;
        const auto index=(resume+choice)%unique.size();
        const double offset=unique[index];
        // Keep enumeration resumable even when the first connector is invalid.
        if(!found)context.static_creep_cursor=(index+1)%unique.size();
        const auto polynomial=MakeQuintic(d,prime,second,offset,0,0,horizon);
        if(context.has_requested_target_d) {
            const double goal=ReferenceArcDistance(ref_,s,context.requested_target_s);
            if(goal>=0&&goal<=horizon&&
               std::fabs(EvalPos(polynomial,goal)-context.requested_target_d)>avoid_cfg_.lateral_tolerance) {
                result.rejection_stage="contact.target_pose";continue;
            }
        }
        if(primary_gap>=0&&primary_gap<=horizon) {
            const double at_primary=EvalPos(polynomial,primary_gap);
            const auto direction=execution_->avoid.requested_direction;
            if((direction==LateralDirection::LEFT&&at_primary<=context.obstacle_d)||
               (direction==LateralDirection::RIGHT&&at_primary>=context.obstacle_d))continue;
        }
        ++result.generated_candidate_count;
        CartesianPath geometry;
        bool valid=true;
        for(double u=0;u<horizon+.1;u+=.1) {
            const double z=std::min(u,horizon);
            const auto pose=FrenetToCartesian(Interpolate(ref_,s+z),s+z,1,0,
                EvalPos(polynomial,z),EvalVel(polynomial,z),EvalAcc(polynomial,z));
            if(!std::isfinite(pose.kappa)||std::fabs(pose.kappa)>limits.max_curvature){valid=false;break;}
            geometry.x.push_back(pose.x);geometry.y.push_back(pose.y);
            geometry.yaw.push_back(pose.yaw);geometry.kappa.push_back(pose.kappa);
        }
        if(!valid){result.rejection_stage="contact.geometry";continue;}
        CartesianPath candidate;std::string rejected;
        if(!RetimeStaticAvoidGeometry(geometry,ego.v,initial_a,cap,path_cfg_.dt,wheelbase_,limits,candidate,&rejected)) {
            result.rejection_stage="contact."+rejected;continue;
        }
        const auto road=avoid_road_map_.EvaluateRecoveryPath(candidate,road_shape,path_cfg_.dt,
            std::max(0.0,6.0-context.map_recovery_travel_m),context.map_recovery_overlap_limit_m,
            context.map_recovery_active,context.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_);
        int blocker=-1;bool recovery=false;
        const bool clear=road.drivable&&StaticAvoidTrajectoryClear(candidate,objects,vehicle_shape_,
            collision_cfg_,path_cfg_.dt,&policy,&blocker,&recovery,&context.margin_recovery_ids);
        last_candidate_paths_.push_back(candidate);last_candidate_validity_.push_back(clear);
        if(!clear) {
            result.rejection_stage=road.drivable?"contact.collision":"contact.road";
            if(blocker>=0)result.blocking_object_id=blocker;
            continue;
        }
        if(command.recovery_clear) {
            double end_s=0;
            if(!ProjectPointToS(candidate.x.back(),candidate.y.back(),command.recovery_min_end_s,end_s)||
               end_s<command.recovery_min_end_s)continue;
        }
        ++result.valid_candidate_count;
        const auto contact=MeasureStaticAvoidContact(geometry,objects,vehicle_shape_,policy);
        const double cost=contact.cost+2*std::fabs(offset-d)+AvoidRoadMap::PaintCost(road);
        if(found&&cost>=best-.1)continue;
        selected=result;selected.candidate_valid=selected.spatial_candidate_selected=true;
        selected.retained_geometry=false;selected.static_progress_fresh=true;
        selected.candidate_path=std::move(candidate);selected.spatial_geometry=std::move(geometry);
        selected.static_contact_cost=contact.cost;selected.static_minimum_clearance=contact.minimum_clearance;
        selected.selected_target_d=offset;selected.selected_cost=cost;
        selected.escape_recovery_required=recovery;selected.blocking_object_id=-1;
        selected.map_entry_s=s;selected.map_transition_distance=horizon;
        // Preserve a finite occupied group independently of the primary ID.
        const bool retained_group=context.static_group_recovery&&context.map_group_valid;
        double group_end=retained_group?ReferenceArcDistance(ref_,s,context.map_group_end_s):0;
        struct Occupancy {double entry,exit;};std::vector<Occupancy> group;
        for(const auto& o:objects)if(IsStaticAvoidProgressObject(o)) {
            double os=0,od=0,ov=0;
            ProjectObjectToFrenet(ref_,o,os,od,ov,ref_config_.ego_projection_search_window);
            const auto bounds=ProjectAvoidFootprint(o.corners,o.x,o.y,Interpolate(ref_,os).theta);
            const double gap=ReferenceArcDistance(ref_,s,os);
            const double corridor=std::max(std::fabs(d),std::fabs(offset))+
                vehicle_shape_.width*.5+collision_cfg_.safety_margin;
            if(!bounds.valid||gap>80||od+bounds.min_d>corridor||od+bounds.max_d<-corridor)continue;
            const double exit=gap+bounds.far_s+vehicle_shape_.rear_overhang+avoid_cfg_.pass_clearance;
            if(exit>0)group.push_back({gap+bounds.near_s-(vehicle_shape_.length-vehicle_shape_.rear_overhang),exit});
        }
        std::sort(group.begin(),group.end(),[](const auto& a,const auto& b){return a.entry<b.entry;});
        if(!retained_group||group_end>0) {
            const double reconnect=std::max(6.0,2*LowSpeedLateralTransitionDistance(offset,0)+ego.v*.5);
            bool have_group=retained_group;
            for(const auto& o:group) {
                if(have_group&&o.entry-group_end>reconnect)break;
                group_end=std::max(group_end,o.exit);have_group=true;
            }
        }
        selected.map_group_valid=retained_group||group_end>0;
        selected.map_group_end_s=s+group_end;
        selected.map_return_schedule_valid=true;
        selected.map_return_start_s=s+group_end;
        selected.map_return_end_s=s+group_end+std::max(6.0,LowSpeedLateralTransitionDistance(offset,0));
        selected.rejection_stage.clear();selected.search_pending=false;
        best=cost;found=true;
        if(contact.cost==0&&std::fabs(offset-d)<1e-6)break;
    }
    if(!found)return false;
    context.static_creep_cursor=0;
    const auto generated=result.generated_candidate_count,valid=result.valid_candidate_count;
    result=std::move(selected);result.generated_candidate_count=generated;result.valid_candidate_count=valid;
    return true;
}
