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
    auto& context=v2_avoidance_;
    const auto body=MakeEgoFootprint(ego,vehicle_shape_,0);
    auto clearance=[&](const TrackedObject& o) {
        return geometry::ConvexPolygonDistance(body,PredictObstacleFootprint(o,0));
    };
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
    if(context.map_spatial_geometry.x.size()>2&&!context.static_progress_fresh) {
        const auto& old=context.map_spatial_geometry;
        std::size_t nearest=0;double distance=INFINITY;
        for(std::size_t i=0;i<old.x.size();++i) {
            const double q=std::hypot(old.x[i]-ego.x,old.y[i]-ego.y);
            if(q<distance){distance=q;nearest=i;}
        }
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
                return true;
            }
        }
    }
    if(std::chrono::steady_clock::now()>=deadline){result.search_pending=true;return false;}

    std::vector<Gate> gates;
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
        if(o.id!=execution_->avoid.obstacle_id &&
           (od+bounds.min_d>.5*vehicle_shape_.width+collision_cfg_.safety_margin ||
            od+bounds.max_d<-.5*vehicle_shape_.width-collision_cfg_.safety_margin))continue;
        gates.push_back({o.id,gap+bounds.near_s-front-safety,exit,left,right});
    }
    std::sort(gates.begin(),gates.end(),[](const Gate& a,const Gate& b){return std::tie(a.entry,a.id)<std::tie(b.entry,b.id);});
    if(gates.empty())return false;
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
    if(std::none_of(gates.begin(),gates.end(),[&](const Gate& g){return g.id==execution_->avoid.obstacle_id;})&&
       !(committed&&ReferenceArcDistance(ref_, s, context.obstacle_s)<0)&&
       !context.static_group_recovery&&!context.static_recovery_engaged&&
       !(policy&&policy->mode!=StaticAvoidPolicy::NORMAL)) {
        // The command still owns its identity. Earlier blockers can constrain
        // its path, but cannot silently replace an out-of-horizon primary.
        return false;
    }

    // Enumerate bounded side sequences. Changing side between distinct gates
    // is allowed; direction/pose ownership applies to the commanded object.
    const std::size_t variants=std::size_t(1)<<gates.size();
    const std::size_t total=variants*15;
    const std::size_t resume=std::min(context.map_corridor_cursor,total);
    bool has_best=false;AvoidPrepareResult best_result;double best_paint=INFINITY;
    auto accept_best=[&]() {
        if(!has_best)return false;
        const auto generated=result.generated_candidate_count,valid=result.valid_candidate_count;
        result=best_result;result.generated_candidate_count=generated;result.valid_candidate_count=valid;
        context.map_corridor_cursor=0;return true;
    };
    for(std::size_t ordinal=resume;ordinal<total;++ordinal) {
        if(std::chrono::steady_clock::now()>=deadline) {
            if(accept_best())return true;
            context.map_corridor_cursor=ordinal;result.search_pending=true;return false;
        }
        const std::size_t bits=ordinal%variants,variant=ordinal/variants;
        const double extra=std::max(.02,avoid_cfg_.direction_extra_margin*(1-.5*double((variant/5)%3)));
        std::vector<double> targets;
        std::vector<bool> sides;
        bool admissible=true;
        for(std::size_t i=0;i<gates.size();++i) {
            bool left=((bits>>i)&1)!=0;
            if(!avoid_cfg_.prefer_right_when_equal)left=!left;
            double target=left?gates[i].left+extra:gates[i].right-extra;
            if(gates[i].id==execution_->avoid.obstacle_id) {
                const auto direction=execution_->avoid.requested_direction;
                if((direction==LateralDirection::LEFT&&!left)||(direction==LateralDirection::RIGHT&&left))admissible=false;
                if(context.has_requested_target_d)target=context.requested_target_d;
                if(committed&&(context.map_lateral_started||context.map_side_reselected||std::fabs(ep)>.10)&&
                   ((context.target_d>=context.obstacle_d)!=left))admissible=false;
            }
            targets.push_back(target);sides.push_back(left);
        }
        if(!admissible)continue;
        // Overlapping same-side bodies need their joint width before entering
        // the first one. Waiting until its rear edge can make the second shift
        // impossible, even though a wider continuous corridor is available.
        for(std::size_t pass=0;pass<gates.size();++pass)
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
            if(std::fabs(target)>4.0){good=false;break;}
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
            if(paint_cost<=1e-9)return true;
            if(!has_best||paint_cost<best_paint) {
                best_paint=paint_cost;best_result=result;has_best=true;
            }
            result.candidate_valid=false;
        }
    }
    if(accept_best())return true;
    context.map_corridor_cursor=0;
    return false;
}
