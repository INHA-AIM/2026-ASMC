#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/Math/Polynomial.hpp"
#include "frenet_planner/Maneuver/ManeuverUtils.hpp"
#include "frenet_planner/Math/Utils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

bool FrenetPlanner::MapAvoidProcess(const CartesianState& ego,const FrenetState& start,
    double s,double d,double speed,const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command,bool /*prepare*/,bool committed,CartesianPath& output,
    AvoidPrepareResult& result) {
    const double length=ref_.points.back().s;
    const double gap=ForwardArcDistance(s,v2_avoidance_.obstacle_s,length);
    const bool returning=committed&&v2_avoidance_.phase==AvoidPhase::RETURN;
    const auto target=std::find_if(obstacles.begin(),obstacles.end(),[&](const TrackedObject& o){
        return o.id==v2_avoidance_.obstacle_id;
    });
    // UNKNOWN/PENDING velocities still participate in every collision test.
    // They cannot commit a static bypass based on a transient predicted corridor.
    const bool static_confirmed=target!=obstacles.end()&&target->motion_state==ObjectMotionState::LOW_THREAT;
    const double front=vehicle_shape_.length-vehicle_shape_.rear_overhang;
    const double clearance=front+v2_avoidance_.obstacle_longitudinal_half_extent+collision_cfg_.safety_margin;
    const double crawl=std::min(speed,limits_.low_speed_crawl_speed_cap);
    const double dt=path_cfg_.dt;
    const bool initial_inside=avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,vehicle_shape_);
    VehicleShape settled_shape=vehicle_shape_;
    settled_shape.width+=.10;settled_shape.length+=.10;settled_shape.rear_overhang+=.05;
    const bool initial_settled=avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,settled_shape);
    if(v2_avoidance_.map_recovery_active){
        const double movement=std::fabs(ForwardArcDistance(v2_avoidance_.map_recovery_last_s,s,length));
        v2_avoidance_.map_recovery_travel_m+=movement;
        v2_avoidance_.map_recovery_last_s=s;
        v2_avoidance_.map_recovery_settled_travel_m=initial_settled
            ? v2_avoidance_.map_recovery_settled_travel_m+movement : 0.0;
        if(v2_avoidance_.map_recovery_settled_travel_m>=.5){
            v2_avoidance_.map_recovery_active=false;
            v2_avoidance_.map_recovery_travel_m=0;
        }
    }
    if(!initial_inside&&!v2_avoidance_.map_recovery_active){
        v2_avoidance_.map_recovery_active=true;
        v2_avoidance_.map_recovery_last_s=s;
        v2_avoidance_.map_recovery_travel_m=0;
        v2_avoidance_.map_recovery_settled_travel_m=0;
        v2_avoidance_.map_recovery_overlap_limit_m=std::min(.35,
            avoid_road_map_.FootprintOverlapDepth(ego.x,ego.y,ego.yaw,vehicle_shape_)+.10);
    }
    const double recovery_budget=std::max(0.0,6.0-v2_avoidance_.map_recovery_travel_m);
    const double decel=limits_.max_longitudinal_accel*.8;
    const double jerk=limits_.max_longitudinal_jerk;
    const double initial_slope=std::tan(std::remainder(ego.yaw-Interpolate(ref_,s).theta,2*M_PI))
        *(1-Interpolate(ref_,s).kappa*d);
    const double bypass_side=v2_avoidance_.target_d>=v2_avoidance_.obstacle_d?1.0:-1.0;
    if(committed&&bypass_side*(d-v2_avoidance_.map_lateral_origin_d)>=.20)
        v2_avoidance_.map_lateral_started=true;
    const bool may_retime=committed&&!returning&&!v2_avoidance_.map_lateral_started&&gap>clearance+4.0;

    // Generate in distance, then time-sample with observed initial v/a. Approach,
    // shift, pass and return are validated together, including the rear-axle body.
    auto generate=[&](double target_d,double delay,double transition,bool maintenance,CartesianPath& path){
        // A short remaining entry delay cannot accommodate a separate return to
        // the centerline. Start the bypass from the measured d/yaw instead.
        // The full trajectory still passes the same road and collision checks.
        const double center_distance=std::max(2.0,LowSpeedLateralTransitionDistance(d,0.0)*1.25);
        const double shift_start=maintenance?0.0:(delay<center_distance?0.0:delay);
        const double settle=maintenance ? std::max(10.0,std::min(25.0,ego.v*2.5)) : std::max(1.0,std::min(20.0,shift_start));
        const auto approach=MakeQuintic(d,initial_slope,0,0,0,0,settle);
        const double shift_initial=shift_start>0?0.0:d;
        const auto shift=MakeQuintic(shift_initial,shift_start>0?0.0:initial_slope,0,target_d,0,0,transition);
        const double pass_end=std::max(shift_start+transition,
            gap+v2_avoidance_.obstacle_longitudinal_half_extent+vehicle_shape_.rear_overhang+avoid_cfg_.pass_clearance);
        const double return_distance=std::max(transition,LowSpeedLateralTransitionDistance(target_d,0.0)*1.25);
        const auto back=MakeQuintic(target_d,0,0,0,0,0,return_distance);
        const double total=maintenance?std::max(1.0,gap-clearance):
            (returning?transition:pass_end+return_distance);
        double u=0,v=std::max(0.0,ego.v),a=clip(ego.a,-decel,limits_.max_longitudinal_accel);
        for(int n=0;n<1000;++n){
            double offset,prime,second;
            if(maintenance){const double z=std::min(u,settle);offset=EvalPos(approach,z);prime=EvalVel(approach,z);second=EvalAcc(approach,z);}
            else if(u<shift_start){const double z=std::min(u,settle);offset=EvalPos(approach,z);prime=EvalVel(approach,z);second=EvalAcc(approach,z);}
            else if(u<shift_start+transition){const double z=u-shift_start;offset=EvalPos(shift,z);prime=EvalVel(shift,z);second=EvalAcc(shift,z);}
            else if(u<pass_end){offset=target_d;prime=second=0;}
            else {const double z=std::min(return_distance,u-pass_end);offset=EvalPos(back,z);prime=EvalVel(back,z);second=EvalAcc(back,z);}
            const auto rp=Interpolate(ref_,s+u);
            // Cartesian v is authoritative; translate its magnitude to s_dot.
            const double metric=std::hypot(1-rp.kappa*offset,prime);
            if(metric<.1)return false;
            const auto point=FrenetToCartesian(rp,s+u,v/metric,a/metric,offset,prime,second);
            if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits_.max_curvature+1e-6||
                v*v*std::fabs(point.kappa)>limits_.max_lateral_accel+1e-6)return false;
            path.x.push_back(point.x);path.y.push_back(point.y);path.yaw.push_back(point.yaw);
            path.kappa.push_back(point.kappa);path.v.push_back(v);path.a.push_back(a);
            if(u>=total-.05 || (maintenance&&(n*dt>=3.0||v<.02)&&n>0))return path.x.size()>1;
            // Jerk ramp distance reserve begins deceleration before the lateral
            // transition. No instantaneous replacement of measured speed by cap.
            const double remaining=std::max(0.0,(maintenance?total:shift_start)-u);
            const double ramp=std::max(0.0,v)*decel/std::max(.1,jerk);
            const double terminal=maintenance?0.0:crawl;
            double cap=std::min(speed,std::sqrt(terminal*terminal+2*decel*std::max(0.0,remaining-ramp)));
            if(maintenance){
                cap=std::min(cap,crawl); // Static classification is pending: brake immediately.
            }
            const double desired=clip((cap-v)/.8,-decel,limits_.max_longitudinal_accel);
            const double next_a=clip(desired,a-jerk*dt,a+jerk*dt);
            const double next_v=std::max(0.0,v+(a+next_a)*.5*dt);
            u+=(v+next_v)*.5*dt/metric;v=next_v;a=next_a;
            if(!maintenance&&v<.02&&speed<.05)return false;
        }
        return false;
    };

    // Keep the already accepted recovery/bypass curve while tracking it closely.
    // Re-fitting the same tiny boundary overlap each cycle can change the curve
    // enough to lose every candidate. This is never an unchecked stale-path fallback.
    if(committed&&!v2_avoidance_.map_recovery_path.x.empty()){
        const auto& cached=v2_avoidance_.map_recovery_path;
        std::size_t nearest=0;double distance=INFINITY;
        for(std::size_t i=0;i<cached.x.size();++i){
            const double delta=std::hypot(cached.x[i]-ego.x,cached.y[i]-ego.y);
            if(delta<distance){distance=delta;nearest=i;}
        }
        if(nearest+1<cached.x.size()&&distance<.05&&
           std::fabs(std::remainder(cached.yaw[nearest]-ego.yaw,2*M_PI))<.02&&
           std::fabs(cached.v[nearest]-ego.v)<.10&&std::fabs(cached.a[nearest]-ego.a)<.5){
            CartesianPath continuation=cached;
            for(auto* values:{&continuation.x,&continuation.y,&continuation.yaw,&continuation.kappa,&continuation.v,&continuation.a})
                values->erase(values->begin(),values->begin()+nearest);
            continuation.x[0]=ego.x;continuation.y[0]=ego.y;continuation.yaw[0]=ego.yaw;
            continuation.kappa[0]=ego.kappa;continuation.v[0]=ego.v;continuation.a[0]=ego.a;
            bool kinematic=true;
            for(std::size_t i=0;i<continuation.x.size();++i){
                if(!std::isfinite(continuation.kappa[i])||std::fabs(continuation.kappa[i])>limits_.max_curvature||
                   continuation.v[i]*continuation.v[i]*std::fabs(continuation.kappa[i])>limits_.max_lateral_accel)
                    kinematic=false;
                if(i&&(std::fabs(continuation.v[i]-continuation.v[i-1])/dt>limits_.max_longitudinal_accel+1e-6||
                       std::fabs(continuation.a[i]-continuation.a[i-1])/dt>limits_.max_longitudinal_jerk+1e-6))kinematic=false;
            }
            const auto road=avoid_road_map_.EvaluateRecoveryPath(continuation,vehicle_shape_,dt,
                recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m);
            int blocking=-1;bool margin_recovery=false;
            const bool clear=kinematic&&road.drivable&&CartesianTrajectoryClear(continuation,obstacles,
                vehicle_shape_,collision_cfg_,dt,&blocking,v2_avoidance_.obstacle_id,&margin_recovery);
            if(clear){
                ++result.generated_candidate_count;++result.valid_candidate_count;
                last_candidate_paths_.push_back(continuation);last_candidate_validity_.push_back(true);
                result.candidate_valid=true;result.spatial_candidate_selected=true;
                result.selected_target_d=v2_avoidance_.map_recovery_path_target_d;
                result.selected_cost=AvoidRoadMap::PaintCost(road);
                result.candidate_path=continuation;result.escape_recovery_required=margin_recovery;
                result.map_entry_s=v2_avoidance_.map_entry_s;result.map_transition_distance=v2_avoidance_.map_transition_distance;
                v2_avoidance_.map_recovery_path=continuation;
                output=continuation;UpdateCommittedAvoidLifecycle(s,d,ego.v,result);
                last_failure_reason_=PlanFailureReason::NONE;return true;
            }
        }
    }

    double best=std::numeric_limits<double>::infinity();
    if(!result.direction_conflict&&!result.target_behind_ego&&(committed||static_confirmed)) {
        std::vector<double> targets;
        if(committed) targets.push_back(returning?0.0:v2_avoidance_.target_d);
        else for(int i=0;i<v2_avoid_targets_.count;++i)targets.push_back(v2_avoid_targets_.candidates[i]);
        if(!committed)std::stable_sort(targets.begin(),targets.end(),[&](double a,double b){
            const bool a_preferred=((a<v2_avoidance_.obstacle_d)==avoid_cfg_.prefer_right_when_equal);
            const bool b_preferred=((b<v2_avoidance_.obstacle_d)==avoid_cfg_.prefer_right_when_equal);
            return a_preferred&&!b_preferred;
        });
        auto evaluate_target=[&](double target_d,const std::vector<double>& delays){
            // A near-zero offset error can still have a substantial heading
            // error. Reserve distance for that slope to settle; the sampled
            // curvature, acceleration, road and collision checks remain final.
            const double heading_distance=6.0*std::fabs(initial_slope)/
                std::max(.01,limits_.max_curvature);
            const double natural=std::max(
                LowSpeedLateralTransitionDistance(d,target_d),heading_distance);
            for(double delay:delays){
                for(double scale=1.0;scale<=3.0+1e-6;scale+=.25){
                    double transition=natural*scale;
                    if(committed&&!returning)transition=std::max(natural,
                        v2_avoidance_.map_transition_distance-std::max(0.0,-ForwardArcDistance(s,v2_avoidance_.map_entry_s,length)))*scale;

                    double cost=delay+transition;
                    if(!committed && ((target_d<v2_avoidance_.obstacle_d)!=avoid_cfg_.prefer_right_when_equal))
                        cost+=avoid_cfg_.direction_tiebreak_cost;
                    if(cost>=best)continue;
                    CartesianPath candidate;++result.generated_candidate_count;
                    int blocking=-1;bool margin_recovery=false;
                    const bool generated=generate(target_d,delay,transition,false,candidate);
                    const auto road_evaluation=generated
                        ? avoid_road_map_.EvaluateRecoveryPath(candidate,vehicle_shape_,dt,recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m)
                        : AvoidRoadMap::PathEvaluation{};
                    const bool road=road_evaluation.drivable;
                    const bool clear=generated&&road&&
                        CartesianTrajectoryClear(candidate,obstacles,vehicle_shape_,collision_cfg_,dt,&blocking,v2_avoidance_.obstacle_id,&margin_recovery);
                    if(!candidate.x.empty()){last_candidate_paths_.push_back(candidate);last_candidate_validity_.push_back(clear);}
                    if(!clear){if(blocking>=0)result.blocking_object_id=blocking;continue;}
                    ++result.valid_candidate_count;
                    cost+=AvoidRoadMap::PaintCost(road_evaluation);
                    if(cost>=best)continue;
                    best=cost;result.blocking_object_id=-1;result.escape_recovery_required=margin_recovery;result.candidate_valid=true;result.spatial_candidate_selected=true;
                    result.selected_target_d=target_d;result.selected_cost=cost;result.candidate_path=candidate;
                    // Match the station to the actual generated shift. A short
                    // delay is folded into an immediate measured-pose transition.
                    const double center_distance=std::max(2.0,LowSpeedLateralTransitionDistance(d,0.0)*1.25);
                    result.map_entry_s=std::fmod(s+(delay<center_distance?0.0:delay),length);
                    result.map_transition_distance=transition;
                }
            }
        };
        const double locked_delay=committed
            ? std::max(0.0,ForwardArcDistance(s,v2_avoidance_.map_entry_s,length)):0.0;
        std::vector<double> free_delays;
        const double latest_entry=std::max(0.0,std::min(40.0,gap-clearance-1.0));
        for(double delay=0;delay<=latest_entry+1e-6;delay+=2.0)free_delays.push_back(delay);
        const std::vector<double> nominal_delays=committed
            ? std::vector<double>{locked_delay}:free_delays;
        // Preserve any valid nominal solution. Observation noise must not cause
        // an opportunistic change of entry station, side or bypass width.
        for(double target_d:targets)evaluate_target(target_d,nominal_delays);
        if(!result.candidate_valid&&may_retime){
            evaluate_target(targets.front(),free_delays);
        }
        // All fresh objects are checked, regardless of target identity. Nearby
        // obstacles and split clusters may require a wider same-side bypass.
        // An explicit target pose remains authoritative; RETURN keeps d=0.
        if(!result.candidate_valid&&!returning&&!v2_avoidance_.has_requested_target_d){
            const double step=std::max(.25,avoid_cfg_.committed_recovery_step);
            const double expansion=std::min(2.0,std::max(0.0,avoid_cfg_.committed_recovery_max_expansion));
            for(double extra=step;extra<=expansion+1e-6&&!result.candidate_valid;extra+=step){
                for(double base:targets){
                    const double side=base>=v2_avoidance_.obstacle_d?1.0:-1.0;
                    const double widened=base+side*extra;
                    if(std::fabs(widened)>4.0)continue;
                    evaluate_target(widened,may_retime?free_delays:nominal_delays);
                }
            }
        }
    }
    if(result.candidate_valid)result.blocking_object_id=-1;
    if(committed){
        if(!result.candidate_valid){last_failure_reason_=result.blocking_object_id>=0?
            PlanFailureReason::COLLISION_BLOCKED:PlanFailureReason::TRANSIENT_GENERATION_FAILURE;return false;}
        if(!returning){
            v2_avoidance_.target_d=result.selected_target_d;
            if(may_retime){
                const double entry_gap=ForwardArcDistance(s,result.map_entry_s,length);
                v2_avoidance_.map_entry_s=result.map_entry_s;
                v2_avoidance_.map_transition_distance=result.map_transition_distance;
                v2_avoidance_.shift_entry_distance=gap-std::max(0.0,entry_gap);
                v2_avoidance_.phase=entry_gap>0.0?AvoidPhase::TRACKING:AvoidPhase::SHIFT;
                v2_avoidance_.start_d=d;
            } else if(v2_avoidance_.phase==AvoidPhase::PASS&&
                      std::fabs(d-v2_avoidance_.target_d)>avoid_cfg_.lateral_tolerance){
                v2_avoidance_.phase=AvoidPhase::SHIFT;
            }
        }
        v2_avoidance_.map_recovery_path=result.candidate_path;
        v2_avoidance_.map_recovery_path_target_d=result.selected_target_d;
        output=result.candidate_path;UpdateCommittedAvoidLifecycle(s,d,ego.v,result);
        last_failure_reason_=PlanFailureReason::NONE;return true;
    }
    // PREPARE never publishes its bypass. Brake along the reference until static
    // classification and a complete road-constrained bypass are both available.
    CartesianPath approach;
    const bool approach_generated=generate(0,0,1,true,approach);
    const bool approach_road=approach_generated&&avoid_road_map_.EvaluateRecoveryPath(approach,vehicle_shape_,dt,recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m).drivable;
    if(approach_generated&&approach_road&&
       CartesianTrajectoryClear(approach,obstacles,vehicle_shape_,collision_cfg_,dt))output=std::move(approach);
    else {
        PlannerCommand base=command;base.target_speed=crawl;
        if(!BuildAvoidMaintenancePath(ego,start,base,obstacles,output))return false;
        if(output.x.size()>1&&!avoid_road_map_.EvaluateRecoveryPath(output,vehicle_shape_,dt,recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m).drivable)BuildZeroSpeedHoldPath(ego,s,output);
    }
    last_failure_reason_=PlanFailureReason::NONE;return true;
}
