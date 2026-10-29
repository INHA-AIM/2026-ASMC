#include "Frenet/frenet_planner.hpp"
#include "Frenet/Generator/StaticAvoidPath.hpp"
#include "Frenet/Geometry/AvoidanceGeometry.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include "Frenet/Math/Utils.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <numeric>

namespace {
void Append(CartesianPath& path,const CartesianState& p) {
    path.x.push_back(p.x);path.y.push_back(p.y);path.yaw.push_back(p.yaw);path.kappa.push_back(p.kappa);
}
bool StaticOccupancy(const TrackedObject& o) {
    return o.geometry_valid&&!o.velocity_unavailable&&
        o.motion_state==ObjectMotionState::LOW_THREAT&&
        (o.type==0||o.type==3);
}
struct Gate {
    int id=-1;
    double entry=0,exit=0,left=0,right=0;
};
}

bool FrenetPlanner::PlanStaticAvoidCorridor(const CartesianState& ego,double s,double d,
    double speed,double initial_a,const std::vector<TrackedObject>& objects,
    const PlannerCommand& command,bool committed,AvoidPrepareResult& result,
    std::chrono::steady_clock::time_point deadline,const StaticAvoidPolicy* policy) {
    if(v2_avoidance_.dynamic_target||ref_.points.empty()||speed<=.05)return false;
    const double length=ref_.points.back().s,dt=path_cfg_.dt;
    const double cap=std::min(speed,avoid_cfg_.bypass_speed_cap);
    const double lateral_limit=avoid_cfg_.static_search_lateral_limit;
    auto& context=v2_avoidance_;
    const int level=policy?policy->search_level:0;
    auto& progress=context.static_search_cursors[std::clamp(level,0,5)];
    const auto body=MakeEgoFootprint(ego,vehicle_shape_,0);
    auto clearance=[&](const TrackedObject& o) {
        return geometry::ConvexPolygonDistance(body,PredictObstacleFootprint(o,0));
    };
    if(!policy||policy->mode!=StaticAvoidPolicy::CONTACT_PROGRESS) {
        for(const auto& o:objects)if(StaticOccupancy(o)&&clearance(o)<=1e-9) {
            progress.center_done=progress.corridor_exhausted=progress.reconnection_exhausted=true;
            result.blocking_object_id=o.id;result.rejection_stage="initial_body_overlap";
            return false;
        }
    }
    if(!context.margin_recovery_initialized) {
        context.margin_recovery_initialized=true;
        context.margin_axis_x=std::cos(ego.yaw);context.margin_axis_y=std::sin(ego.yaw);
        for(const auto& o:objects)if(StaticOccupancy(o)) {
            const double gap=clearance(o);
            if(gap<=1e-9||gap>collision_cfg_.safety_margin+1e-9)continue;
            const double dx=o.x-ego.x,dy=o.y-ego.y;
            const bool physical=dx*context.margin_axis_x+dy*context.margin_axis_y<=0;
            const double progress=physical?gap:std::fabs(-dx*context.margin_axis_y+dy*context.margin_axis_x);
            context.margin_anchors.push_back({o.id,o.x,o.y,progress,physical,false});
        }
    }
    context.margin_recovery_ids.clear();
    for(auto& a:context.margin_anchors) {
        if(a.exited)continue;
        const TrackedObject* observation=nullptr;
        for(const auto& o:objects)if(o.id==a.id){observation=&o;break;}
        if(!observation) {
            for(const auto& o:objects)if(StaticOccupancy(o)&&std::hypot(o.x-a.x,o.y-a.y)<.4) {
                if(observation){observation=nullptr;break;}
                observation=&o;
            }
            if(observation)a.id=observation->id;
        }
        if(!observation)continue;
        if(!StaticOccupancy(*observation)){a.exited=true;continue;}
        const double gap=clearance(*observation);
        a.x=observation->x;a.y=observation->y;
        if(gap>collision_cfg_.safety_margin+1e-9){a.exited=true;continue;}
        const double dx=a.x-ego.x,dy=a.y-ego.y;
        const double progress=a.physical?gap:std::fabs(-dx*context.margin_axis_y+dy*context.margin_axis_x);
        if(gap>1e-9&&progress+.1*collision_cfg_.safety_margin>=a.maximum) {
            a.maximum=std::max(a.maximum,progress);context.margin_recovery_ids.push_back(a.id);
        }
    }

    VehicleShape road_shape=vehicle_shape_;
    road_shape.width=std::max(road_shape.width,road_shape.road_outer_width);
    const double recovery_budget=std::max(0.0,6.0-context.map_recovery_travel_m);
    std::vector<TrackedObject> fixed_objects;
    for(const auto& o:objects)if(o.motion_state==ObjectMotionState::LOW_THREAT&&
        o.geometry_valid&&!o.velocity_unavailable)fixed_objects.push_back(o);
    auto validate=[&](const CartesianPath& full_geometry,bool retained) {
        if(retained&&(context.has_requested_target_d||
           execution_->avoid.requested_direction!=LateralDirection::NONE||MapAvoidSideLocked(committed))) {
            double nearest_station=INFINITY,offset=0,last_s=s,nearest_primary=INFINITY,primary_offset=0;
            for(std::size_t i=0;i<full_geometry.x.size();++i) {
                double ps=s,pd=0;
                if(!ProjectPointToFrenet(full_geometry.x[i],full_geometry.y[i],s,100,
                    std::max(20.0,lateral_limit+1),ps,pd))continue;
                last_s=ps;
                const double delta=std::fabs(ReferenceArcDistance(ref_,context.requested_target_s,ps));
                if(delta<nearest_station){nearest_station=delta;offset=pd;}
                const double primary_delta=std::fabs(ReferenceArcDistance(ref_,context.obstacle_s,ps));
                if(primary_delta<nearest_primary){nearest_primary=primary_delta;primary_offset=pd;}
            }
            const double goal=ReferenceArcDistance(ref_,s,context.requested_target_s);
            if(context.has_requested_target_d&&goal>=0&&goal<=ReferenceArcDistance(ref_,s,last_s)&&
               (nearest_station>.25||std::fabs(offset-context.requested_target_d)>avoid_cfg_.lateral_tolerance)) {
                result.rejection_stage="retained.target_pose";return false;
            }
            const double primary_gap=ReferenceArcDistance(ref_,s,context.obstacle_s);
            if(primary_gap>=0&&primary_gap<=ReferenceArcDistance(ref_,s,last_s)&&
               !MapAvoidSideAllowed(primary_offset,committed)) {
                result.rejection_stage="retained.direction";return false;
            }
        }
        CartesianPath prefix;
        const bool contact=policy&&policy->mode==StaticAvoidPolicy::CONTACT_PROGRESS;
        if(contact) {
            // A rolling, stop-capable prefix is retimed from the measured
            // state. Keep the full nominal geometry separately for continuity.
            const double stop_reserve=6.0+ego.v*ego.v/(limits_.max_longitudinal_accel*1.6)+ego.v;
            double traveled=0;
            for(std::size_t i=0;i<full_geometry.x.size();++i) {
                if(i)traveled+=std::hypot(full_geometry.x[i]-full_geometry.x[i-1],full_geometry.y[i]-full_geometry.y[i-1]);
                CartesianState p{};p.x=full_geometry.x[i];p.y=full_geometry.y[i];
                p.yaw=full_geometry.yaw[i];p.kappa=full_geometry.kappa[i];Append(prefix,p);
                if(traveled>=stop_reserve)break;
            }
        }
        const auto& geometry=contact?prefix:full_geometry;
        ++result.generated_candidate_count;
        int fixed_blocker=-1;
        // Spatial samples remain dense even when a time sample travels past a
        // thin bollard. This zero-time check applies only to confirmed fixed
        // occupancy; uncertain/moving objects retain the timed hypotheses below.
        if(!StaticAvoidTrajectoryClear(geometry,fixed_objects,vehicle_shape_,collision_cfg_,
                0,policy,&fixed_blocker,nullptr,&context.margin_recovery_ids)) {
            result.blocking_object_id=fixed_blocker;result.rejection_stage="spatial_collision";return false;
        }
        CartesianPath candidate;
        auto executable_limits=limits_;
        executable_limits.max_forward_accel=std::min(executable_limits.max_forward_accel,avoid_cfg_.bypass_accel_cap);
        AvoidRoadMap::PathEvaluation road;
        bool recovery=false;
        const auto execute=[&](double target) {
            std::string rejection;
            if(!RetimeStaticAvoidGeometry(geometry,ego.v,initial_a,target,dt,wheelbase_,executable_limits,candidate,&rejection)) {
                result.rejection_stage="actuation."+rejection;return false;
            }
            road=avoid_road_map_.EvaluateRecoveryPath(candidate,road_shape,dt,
                recovery_budget,context.map_recovery_overlap_limit_m,context.map_recovery_active,
                context.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_);
            int blocker=-1;recovery=false;
            const bool clear=road.drivable&&StaticAvoidTrajectoryClear(candidate,objects,
                vehicle_shape_,collision_cfg_,dt,policy,&blocker,&recovery,&context.margin_recovery_ids);
            last_candidate_paths_.push_back(candidate);last_candidate_validity_.push_back(clear);
            if(!clear) {
                result.rejection_stage=road.drivable?"collision":"road";
                if(blocker>=0)result.blocking_object_id=blocker;
            }
            return clear;
        };
        double accepted_cap=cap;
        if(!execute(cap)) {
            // An optional speed upgrade must not discard a still executable
            // incumbent. Rebuild its timing from the measured state and check
            // every current object/road/actuation constraint at the old cap.
            const double previous=context.map_speed_profile_cap;
            if(!retained||!context.static_recovery_engaged||context.static_progress_level!=0||
               (policy&&policy->mode!=StaticAvoidPolicy::NORMAL)||!context.map_speed_profile_valid||
               previous<=.05||previous>=cap-1e-6)return false;
            ++result.generated_candidate_count;
            if(!execute(previous))return false;
            accepted_cap=previous;
        }
        if(command.recovery_clear) {
            double end_s=0;
            if(!ProjectPointToS(candidate.x.back(),candidate.y.back(),command.recovery_min_end_s,end_s)||
               end_s<command.recovery_min_end_s)return false;
        }
        if(accepted_cap<cap-1e-6) {
            context.static_speed.Enter(accepted_cap);
            result.static_speed_retry=true;
        }
        ++result.valid_candidate_count;result.blocking_object_id=-1;
        result.candidate_valid=result.spatial_candidate_selected=true;
        result.static_execution_speed_cap=accepted_cap;
        result.selected_cost=AvoidRoadMap::PaintCost(road);
        result.escape_recovery_required=recovery;result.retained_geometry=retained;
        result.static_progress_fresh=retained&&context.static_progress_fresh;
        result.spatial_geometry=full_geometry;result.candidate_path=std::move(candidate);
        if(policy&&policy->mode!=StaticAvoidPolicy::NORMAL) {
            const auto contact=MeasureStaticAvoidContact(geometry,objects,vehicle_shape_,*policy);
            result.static_contact_cost=contact.cost;
            result.static_minimum_clearance=contact.minimum_clearance;
        }
        result.rejection_stage.clear();result.search_pending=false;
        context.map_corridor_cursor=0;
        return true;
    };

    auto derivatives=[&](const CartesianState& pose,double hint,double& station,double& offset,double& prime,double& second) {
        double sd=0,sdd=0;
        CartesianToFrenet(ref_,pose,station,sd,sdd,offset,prime,second,&hint,ref_config_.ego_projection_search_window);
        return std::isfinite(prime)&&std::isfinite(second)&&std::fabs(prime)<2;
    };
    double es=s,ed=d,ep=0,epp=0;
    if(!derivatives(ego,s,es,ed,ep,epp))return false;
    auto segment=[&](CartesianPath& path,double from,double from_d,double from_p,double from_pp,
        double to,double to_d,double to_p,double to_pp) {
        if(to<=from+.05)return false;
        const auto polynomial=MakeQuintic(from_d,from_p,from_pp,to_d,to_p,to_pp,to-from);
        for(double u=path.x.empty()?0:.1;u<to-from+.1;u+=.1) {
            const double z=std::min(u,to-from);
            const auto p=FrenetToCartesian(Interpolate(ref_,from+z),from+z,1,0,
                EvalPos(polynomial,z),EvalVel(polynomial,z),EvalAcc(polynomial,z));
            if(!std::isfinite(p.kappa)||std::fabs(p.kappa)>limits_.max_curvature+1e-9)return false;
            Append(path,p);
        }
        return true;
    };

    // Spatial execution is the incumbent. Update timing and a C2 connection
    // from the measured pose before considering a different corridor.
    if(context.map_spatial_geometry.x.size()>2&&(!context.static_progress_fresh||
       (policy&&policy->mode==StaticAvoidPolicy::CONTACT_PROGRESS))) {
        const auto& old=context.map_spatial_geometry;
        std::size_t nearest=0;double distance=INFINITY;
        for(std::size_t i=0;i<old.x.size();++i) {
            const double q=std::hypot(old.x[i]-ego.x,old.y[i]-ego.y);
            if(q<distance){distance=q;nearest=i;}
        }
        double remaining=0;
        for(std::size_t i=nearest+1;i<old.x.size();++i)
            remaining+=std::hypot(old.x[i]-old.x[i-1],old.y[i]-old.y[i-1]);
        if(context.static_progress_fresh&&remaining<3.0+ego.v)distance=INFINITY;
        if(distance<1.5)for(double join:{3.0,5.0,8.0}) {
            std::size_t index=nearest;double traveled=0;
            while(index+1<old.x.size()&&traveled<join) {
                traveled+=std::hypot(old.x[index+1]-old.x[index],old.y[index+1]-old.y[index]);++index;
            }
            if(index+2>=old.x.size())continue;
            CartesianState endpoint{};endpoint.x=old.x[index];endpoint.y=old.y[index];
            endpoint.yaw=old.yaw[index];endpoint.kappa=old.kappa[index];
            double end_s=s,end_d=0,end_p=0,end_pp=0;
            if(!derivatives(endpoint,s,end_s,end_d,end_p,end_pp))continue;
            end_s=s+ReferenceArcDistance(ref_, s, end_s);
            CartesianPath connected;
            if(!segment(connected,s,d,ep,epp,end_s,end_d,end_p,end_pp))continue;
            for(std::size_t i=index+1;i<old.x.size();++i) {
                CartesianState p{};p.x=old.x[i];p.y=old.y[i];p.yaw=old.yaw[i];p.kappa=old.kappa[i];Append(connected,p);
            }
            if(validate(connected,true)) {
                result.selected_target_d=committed?context.target_d:context.map_recovery_path_target_d;
                result.selected_cost=0;result.map_entry_s=context.map_entry_s;
                result.map_transition_distance=context.map_transition_distance;
                result.map_return_schedule_valid=context.map_return_schedule_valid;
                result.map_return_start_s=context.map_return_start_s;result.map_return_end_s=context.map_return_end_s;
                result.map_group_valid=context.map_group_valid;result.map_group_end_s=context.map_group_end_s;
                result.static_corridor_kind="retained";
                return true;
            }
        }
    }
    const bool center_probe_only=progress.corridor_exhausted;
    if(std::chrono::steady_clock::now()>=deadline){result.search_pending=true;return false;}

    std::vector<Gate> gates,occupancy;
    const double front=vehicle_shape_.length-vehicle_shape_.rear_overhang;
    for(const auto& o:objects) {
        if(!StaticOccupancy(o))continue;
        double os=0,od=0,velocity=0;
        ProjectObjectToFrenet(ref_,o,os,od,velocity,ref_config_.ego_projection_search_window);
        const double gap=ReferenceArcDistance(ref_, s, os);
        const auto rp=Interpolate(ref_,os);
        const auto bounds=ProjectAvoidFootprint(o.corners,o.x,o.y,rp.theta);
        if(!bounds.valid)continue;
        const bool reduced=policy&&policy->mode!=StaticAvoidPolicy::NORMAL&&
            std::find(policy->object_ids.begin(),policy->object_ids.end(),o.id)!=policy->object_ids.end();
        const double safety=reduced?std::min(collision_cfg_.safety_margin,policy->margin):collision_cfg_.safety_margin;
        const double margin=.5*vehicle_shape_.width+safety;
        const double left=od+bounds.max_d+margin,right=od+bounds.min_d-margin;
        const double exit=gap+bounds.far_s+vehicle_shape_.rear_overhang+avoid_cfg_.pass_clearance;
        if(exit<=0||gap>std::min(80.0,EffectiveAvoidDetectionDistance()))continue;
        const Gate gate{o.id,gap+bounds.near_s-front-safety,exit,left,right};
        // Side objects still constrain the free-space interval even when they
        // do not intrude into the original reference corridor.
        occupancy.push_back(gate);
        if(o.id!=execution_->avoid.obstacle_id &&
           (od+bounds.min_d>.5*vehicle_shape_.width+collision_cfg_.safety_margin ||
            od+bounds.max_d<-.5*vehicle_shape_.width-collision_cfg_.safety_margin))continue;
        gates.push_back(gate);
    }
    std::sort(gates.begin(),gates.end(),[](const Gate& a,const Gate& b){return std::tie(a.entry,a.id)<std::tie(b.entry,b.id);});
    if(gates.empty()){progress.center_done=progress.corridor_exhausted=true;return false;}
    // Bound the group by the space needed to return and start another shift.
    // A more distant target is handed back to Behavior after this group.
    std::size_t count=1;
    double end=gates.front().exit;
    bool contains_primary=gates.front().id==execution_->avoid.obstacle_id;
    for(;count<gates.size()&&count<8;++count) {
        const double amplitude=std::min(std::fabs(gates[count-1].left),std::fabs(gates[count-1].right));
        const double next=std::min(std::fabs(gates[count].left),std::fabs(gates[count].right));
        const double reconnect=1.25*(LowSpeedLateralTransitionDistance(amplitude+avoid_cfg_.direction_extra_margin,0)+
            LowSpeedLateralTransitionDistance(0,next+avoid_cfg_.direction_extra_margin))+std::max(ego.v,cap)*.5;
        if(contains_primary&&gates[count].entry-end>reconnect)break;
        end=std::max(end,gates[count].exit);
        contains_primary|=gates[count].id==execution_->avoid.obstacle_id;
    }
    gates.resize(count);
    // A fixed footprint spans the entire bounded lateral search strip ahead.
    // No route in this bounded family can pass it; do not spend many cycles
    // enumerating the same wall before the explicitly enabled recovery tiers.
    if(std::any_of(gates.begin(),gates.end(),[&](const Gate& g){
        return g.entry>0&&g.right<=-lateral_limit&&g.left>=lateral_limit;
    })) {
        progress.center_done=progress.corridor_exhausted=progress.reconnection_exhausted=true;
        context.map_corridor_cursor=0;result.rejection_stage="bounded_lateral_closed";
        return false;
    }
    if(std::none_of(gates.begin(),gates.end(),[&](const Gate& g){return g.id==execution_->avoid.obstacle_id;})&&
       !(committed&&ReferenceArcDistance(ref_, s, context.obstacle_s)<0)&&
       !context.static_group_recovery&&!context.static_recovery_engaged&&
       !(policy&&policy->mode!=StaticAvoidPolicy::NORMAL)) {
        // The command still owns its identity. Earlier blockers can constrain
        // its path, but cannot silently replace an out-of-horizon primary.
        return false;
    }

    // First form the center of a genuinely free interval between BOTH sides.
    // Inflated occupancy only proposes candidates; full turning-body and timed
    // collision checks below remain authoritative.
    std::vector<double> centers;
    bool center_available=true;
    for(const auto& g:gates) {
        std::vector<std::pair<double,double>> blocked;
        for(const auto& o:occupancy)if(o.entry<=g.exit&&o.exit>=g.entry)
            blocked.push_back({o.right,o.left});
        std::sort(blocked.begin(),blocked.end());
        double lower=-lateral_limit,best_center=0,best_distance=INFINITY;
        const auto consider=[&](double upper) {
            if(upper-lower<=1e-6)return;
            const bool reference_clear=lower<-1e-6&&upper>1e-6;
            if(!reference_clear&&(lower<=-lateral_limit+1e-6||upper>=lateral_limit-1e-6))return;
            const double target=reference_clear?0.0:.5*(lower+upper);
            if(std::fabs(target)<best_distance){best_distance=std::fabs(target);best_center=target;}
        };
        for(const auto& interval:blocked) {
            if(interval.second<=lower)continue;
            consider(std::min(lateral_limit,interval.first));
            lower=std::max(lower,interval.second);
            if(lower>=lateral_limit)break;
        }
        consider(lateral_limit);
        if(!std::isfinite(best_distance)){center_available=false;break;}
        centers.push_back(best_center);
    }
    // The remaining layouts are ranked by lateral effort, with right only a
    // tie-break. A widened search must not make an outside detour win over an
    // already certified center corridor merely because its bit mask is first.
    const std::size_t variants=std::size_t(1)<<gates.size();
    std::vector<std::size_t> layouts(variants);std::iota(layouts.begin(),layouts.end(),0);
    const auto effort=[&](std::size_t bits) {
        double cost=0;bool previous=false;
        for(std::size_t i=0;i<gates.size();++i) {
            const bool left=(((bits>>i)&1)!=0)!=(!avoid_cfg_.prefer_right_when_equal);
            cost+=std::fabs(left?gates[i].left:gates[i].right);
            if(i&&left!=previous)cost+=2;
            previous=left;
        }
        return cost;
    };
    std::stable_sort(layouts.begin(),layouts.end(),[&](auto a,auto b){return effort(a)<effort(b);});
    const std::size_t center_variants=center_available?5:0;
    if(!center_available)progress.center_done=true;
    if(center_probe_only&&!center_available)return false;
    const std::size_t total=center_variants+(center_probe_only?0:variants*15);
    const std::size_t resume=std::min(context.map_corridor_cursor,total);
    bool has_best=false;AvoidPrepareResult best_result;double best_paint=INFINITY;
    auto accept_best=[&]() {
        if(!has_best)return false;
        const auto generated=result.generated_candidate_count,valid=result.valid_candidate_count;
        result=best_result;result.generated_candidate_count=generated;result.valid_candidate_count=valid;
        context.map_corridor_cursor=0;return true;
    };
    // Recheck a newly opened center interval even while an outer sweep is
    // continuing on fresh observations. Never carry an unchecked geometry.
    std::vector<std::size_t> work;
    if(center_variants&&resume>=center_variants)work.push_back(0);
    for(std::size_t i=resume;i<total;++i)work.push_back(i);
    for(const std::size_t ordinal:work) {
        if(std::chrono::steady_clock::now()>=deadline) {
            if(accept_best())return true;
            context.map_corridor_cursor=ordinal;result.search_pending=true;return false;
        }
        const bool central=ordinal<center_variants;
        if(!central)progress.center_done=true;
        const std::size_t local=central?ordinal:ordinal-center_variants;
        const std::size_t bits=central?0:layouts[local%variants],variant=central?local:local/variants;
        const double extra=std::max(.02,avoid_cfg_.direction_extra_margin*(1-.5*double((variant/5)%3)));
        std::vector<double> targets;
        std::vector<bool> sides;
        bool admissible=true;
        for(std::size_t i=0;i<gates.size();++i) {
            bool left=central?centers[i]>=context.obstacle_d:((bits>>i)&1)!=0;
            if(!central&&!avoid_cfg_.prefer_right_when_equal)left=!left;
            double target=central?centers[i]:(left?gates[i].left+extra:gates[i].right-extra);
            if(gates[i].id==execution_->avoid.obstacle_id) {
                if(!MapAvoidSideAllowed(context.obstacle_d+(left?1:-1),committed))admissible=false;
                if(context.has_requested_target_d)target=context.requested_target_d;
                if(!MapAvoidSideAllowed(target,committed))admissible=false;
            }
            targets.push_back(target);sides.push_back(left);
        }
        if(!admissible)continue;
        // Overlapping same-side bodies need their joint width before entering
        // the first one. Waiting until its rear edge can make the second shift
        // impossible, even though a wider continuous corridor is available.
        for(std::size_t pass=0;!central&&pass<gates.size();++pass)
            for(std::size_t i=1;i<gates.size();++i)if(sides[i]==sides[i-1]) {
                const double room=gates[i].entry-gates[i-1].exit;
                if(room<1.25*LowSpeedLateralTransitionDistance(targets[i-1],targets[i])) {
                    const double common=sides[i]?std::max(targets[i-1],targets[i]):std::min(targets[i-1],targets[i]);
                    if(context.has_requested_target_d&&
                       ((gates[i].id==execution_->avoid.obstacle_id&&std::fabs(common-targets[i])>1e-9)||
                        (gates[i-1].id==execution_->avoid.obstacle_id&&std::fabs(common-targets[i-1])>1e-9)))admissible=false;
                    targets[i-1]=targets[i]=common;
                }
            }
        if(!admissible)continue;
        CartesianPath geometry;double previous=0,offset=d,prime=ep,second=epp;
        double primary_target=context.target_d,entry_s=s,first_transition=0;
        bool good=true;double group_end=0;
        const double delay_fractions[]={0,.25,.5,.65,.8};
        const double delay=std::max(0.0,gates.front().entry)*delay_fractions[variant%5];
        if(delay>.5) {
            if(!segment(geometry,s,d,ep,epp,s+delay,0,0,0))continue;
            previous=delay;offset=prime=second=0;entry_s=std::fmod(s+delay+length,length);
        }
        for(std::size_t i=0;i<gates.size();++i) {
            const auto& g=gates[i];
            const double target=targets[i];
            const bool primary=g.id==execution_->avoid.obstacle_id;
            if(primary) {
                primary_target=target;
            }
            if(std::fabs(target)>lateral_limit){good=false;break;}
            double entry=g.entry;
            // Already alongside an initially close object: continuous escape
            // toward its clear rear edge, with the per-object recovery proof.
            if(entry<=previous+.3)entry=std::max(previous+LowSpeedLateralTransitionDistance(offset,target),g.exit);
            if(!segment(geometry,s+previous,offset,prime,second,s+entry,target,0,0)){good=false;break;}
            if(i==0)first_transition=entry-previous;
            const double leave=std::max(entry+.3,g.exit);
            if(!segment(geometry,s+entry,target,0,0,s+leave,target,0,0)){good=false;break;}
            previous=leave;offset=target;prime=second=0;group_end=std::max(group_end,leave);
        }
        if(!good)continue;
        const double return_distance=std::max(6.0,LowSpeedLateralTransitionDistance(offset,0)*1.25);
        const double return_start=previous,return_end=previous+return_distance;
        if(!segment(geometry,s+previous,offset,0,0,s+return_end,0,0,0))continue;
        const double tail=std::max(8.0,cap*cap/(limits_.max_longitudinal_accel*.8)+cap);
        const double total_distance=command.recovery_clear?
            std::max(return_end+tail,command.recovery_min_end_s-s+.5):return_end+tail;
        if(!segment(geometry,s+return_end,0,0,0,s+total_distance,0,0,0))continue;
        if(validate(geometry,false)) {
            const double paint_cost=result.selected_cost;
            result.selected_target_d=primary_target;
            result.selected_cost=total_distance+extra+paint_cost;
            result.map_entry_s=entry_s;result.map_transition_distance=first_transition;
            result.map_return_schedule_valid=true;
            result.map_return_start_s=std::fmod(s+return_start+length,length);
            result.map_return_end_s=std::fmod(s+return_end+length,length);
            result.map_group_valid=gates.size()>1;
            result.map_group_end_s=std::fmod(s+group_end+length,length);
            result.static_corridor_kind=central?"center":
                (std::all_of(sides.begin(),sides.end(),[](bool left){return left;})?"left":
                 std::none_of(sides.begin(),sides.end(),[](bool left){return left;})?"right":"mixed");
            if(central||paint_cost<=1e-9)return true;
            if(!has_best||paint_cost<best_paint) {
                best_paint=paint_cost;best_result=result;has_best=true;
            }
            result.candidate_valid=false;
        }
    }
    if(accept_best())return true;
    progress.center_done=progress.corridor_exhausted=true;
    context.map_corridor_cursor=0;
    return false;
}
