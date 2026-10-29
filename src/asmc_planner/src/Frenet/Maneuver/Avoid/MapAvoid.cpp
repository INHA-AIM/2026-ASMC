#include "Frenet/frenet_planner.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Math/Utils.hpp"
#include "Frenet/Speed/AvoidSpeedMemory.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>
#include <cstdio>
#include <cstdlib>

bool FrenetPlanner::MapAvoidProcess(const CartesianState& ego,const FrenetState& start,
    double s,double d,double speed,const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command,bool /*prepare*/,bool committed,CartesianPath& output,
    AvoidPrepareResult& result) {
    // Offline diagnosis uses the existing evaluations, never another search or
    // collision sweep. Normal/live runs leave this bounded summary disabled.
    struct Diagnostic {
        bool enabled=std::getenv("ASMC_AVOID_DIAGNOSTICS")!=nullptr;
        unsigned generated=0,kinematic_rejected=0,road_rejected=0,collision_rejected=0,clear=0;
        unsigned paint_only=0,boundary=0;
        double least_overlap=INFINITY,least_paint=INFINITY;
        ~Diagnostic(){if(enabled)std::fprintf(stderr,
            "[AVOID_DIAGNOSTIC] generated=%u kinematic=%u road=%u collision=%u clear=%u paint_only=%u boundary=%u least_overlap=%.6f least_paint=%.6f\n",
            generated,kinematic_rejected,road_rejected,collision_rejected,clear,paint_only,boundary,least_overlap,least_paint);}
    } diagnostic;
    if(v2_avoidance_.static_recovery_engaged)
        speed=std::min(speed,v2_avoidance_.static_contact_used?
            avoid_cfg_.static_contact_speed_cap:avoid_cfg_.static_recovery_speed_cap);
    const auto retain_static_policy=[&]() {
        auto& context=v2_avoidance_;
        context.static_recovery_engaged|=result.static_progress_mode!=StaticAvoidPolicy::NORMAL;
        context.static_contact_used|=result.static_progress_mode==StaticAvoidPolicy::CONTACT_PROGRESS;
        context.static_progress_level=result.static_progress_level;
        context.static_progress_fresh=result.static_progress_fresh;
        context.static_group_recovery|=result.static_progress_fresh;
        if(result.static_progress_level==0&&!result.static_progress_fresh) {
            context.last_nominal=result;
            context.last_nominal.static_progress_mode=StaticAvoidPolicy::NORMAL;
            if(context.last_nominal.spatial_geometry.x.empty())
                context.last_nominal.spatial_geometry=result.candidate_path;
            result.static_progress_margin=collision_cfg_.safety_margin;
        }
        if(context.static_recovery_engaged&&result.static_progress_level>0) {
            result.static_progress_mode=context.static_contact_used?
                StaticAvoidPolicy::CONTACT_PROGRESS:StaticAvoidPolicy::REDUCED_MARGIN;
        }
    };
    const auto search_started=std::chrono::steady_clock::now();
    const double slice=v2_avoidance_.dynamic_target?avoid_cfg_.map_search_budget_s:
        std::min(avoid_cfg_.map_search_budget_s,replanning_period_s_*.7);
    auto search_deadline=search_started+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(slice));
    const auto recovery_deadline=search_deadline;
    if(avoid_cfg_.static_margin_recovery_enabled&&!v2_avoidance_.dynamic_target)
        search_deadline=search_started+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(slice*.55));
    bool search_expired=false;
    std::size_t candidate_index=0;
    const std::size_t resume_at=v2_avoidance_.map_search_cursor;
    std::size_t selected_search_index=0;
    bool indexed_candidate_selected=false;
    std::set<std::tuple<double,double,double,double,double,bool,bool>> evaluated;
    const double length=ref_.points.back().s;
    const double gap=ReferenceArcDistance(ref_, s, v2_avoidance_.obstacle_s);
    if(committed&&v2_avoidance_.map_return_schedule_valid&&
       (!v2_avoidance_.map_group_valid||ReferenceArcDistance(ref_, s, v2_avoidance_.map_group_end_s)<=0)&&
       (execution_->avoid.phase==AvoidPhase::SHIFT||execution_->avoid.phase==AvoidPhase::PASS)&&
       ReferenceArcDistance(ref_, s, v2_avoidance_.map_return_start_s)<=0)
        execution_.observeEarlyReturnCandidate(true);
    const bool returning=committed&&execution_->avoid.phase==AvoidPhase::RETURN;
    const auto target=std::find_if(obstacles.begin(),obstacles.end(),[&](const TrackedObject& o){
        return o.id==execution_->avoid.obstacle_id;
    });
    // UNKNOWN/PENDING velocities still participate in every collision test.
    // They cannot commit a static bypass based on a transient predicted corridor.
    const bool slow_dynamic=target!=obstacles.end() && v2_avoidance_.dynamic_target &&
        target->motion_state==ObjectMotionState::DYNAMIC && std::hypot(target->vx,target->vy)<=1.0;
    const bool static_confirmed=target!=obstacles.end() &&
        (target->motion_state==ObjectMotionState::LOW_THREAT || slow_dynamic);
    const auto lane_clear=[&](const CartesianPath& path) {
        if(!v2_avoidance_.dynamic_target || shadow_dynamic_bypass_) return true;
        double hint=s;
        const double bound=.5*lane_width_;
        for(std::size_t i=0;i<path.x.size();++i) {
            CartesianState pose{};pose.x=path.x[i];pose.y=path.y[i];pose.yaw=path.yaw[i];
            // Check the complete turning body, not just the rear axle's d.
            // A curved in-lane candidate can otherwise put its front corner
            // over the lane edge while its centre remains inside the bound.
            const auto body=MakeEgoFootprint(pose,vehicle_shape_,collision_cfg_.safety_margin);
            for(const auto& point:body) {
                double ps=hint,pd=0;
                if(!ProjectPointToFrenet(point.x,point.y,hint,100,5,ps,pd) || std::fabs(pd)>bound) return false;
            }
            double pd=0;ProjectPointToFrenet(path.x[i],path.y[i],hint,100,5,hint,pd);
        }
        return !path.x.empty();
    };
    const auto transition_distance=[&](double from,double to) {
        const double native=LowSpeedLateralTransitionDistance(from,to);
        if(!v2_avoidance_.dynamic_target || shadow_dynamic_bypass_)return native;
        // A rest-to-rest quintic's maximum slope is 1.875*|delta|/L.
        // Reserve room for the vehicle's overhang during that turn. This is
        // a candidate-search hint; full corner projections above remain final.
        const double room=.5*(lane_width_-vehicle_shape_.width)-collision_cfg_.safety_margin-
            std::max(std::fabs(from),std::fabs(to));
        const double overhang=std::max(vehicle_shape_.rear_overhang,
            vehicle_shape_.length-vehicle_shape_.rear_overhang)+collision_cfg_.safety_margin;
        return std::max(native,1.875*std::fabs(to-from)*overhang/std::max(.01,room));
    };
    const double front=vehicle_shape_.length-vehicle_shape_.rear_overhang;
    const double clearance=front+v2_avoidance_.obstacle_longitudinal_half_extent+collision_cfg_.safety_margin;
    const double crawl=std::min(speed,avoid_cfg_.bypass_speed_cap);
    const double dt=path_cfg_.dt;
    VehicleShape map_shape=vehicle_shape_;
    map_shape.width=std::max(map_shape.width,map_shape.road_outer_width);
    const bool initial_inside=avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,map_shape);
    VehicleShape settled_shape=map_shape;
    settled_shape.width+=.10;settled_shape.length+=.10;settled_shape.rear_overhang+=.05;
    const bool initial_settled=avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,settled_shape);
    if(v2_avoidance_.map_recovery_active){
        const double movement=std::fabs(ReferenceArcDistance(ref_, v2_avoidance_.map_recovery_last_s, s));
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
            avoid_road_map_.FootprintOverlapDepth(ego.x,ego.y,ego.yaw,map_shape)+.10);
    }
    const double recovery_budget=std::max(0.0,6.0-v2_avoidance_.map_recovery_travel_m);
    const double decel=limits_.max_longitudinal_accel*.8;
    const double jerk=limits_.max_longitudinal_jerk;
    const double initial_slope=std::tan(std::remainder(ego.yaw-Interpolate(ref_,s).theta,2*M_PI))
        *(1-Interpolate(ref_,s).kappa*d);
    const auto initial_ref=Interpolate(ref_,s);
    const double initial_heading=std::remainder(ego.yaw-initial_ref.theta,2*M_PI);
    const double cosine=std::cos(initial_heading),metric0=1-initial_ref.kappa*d;
    const double initial_second=(ego.kappa-initial_ref.kappa*cosine/metric0)*metric0*metric0/
        (cosine*cosine*cosine)-(initial_ref.d_kappa*d+initial_ref.kappa*initial_slope)*std::tan(initial_heading);
    if(committed&&std::fabs(d-v2_avoidance_.map_lateral_origin_d)>=.20)
        v2_avoidance_.map_lateral_started=true;
    const bool may_retime=committed&&!returning&&!v2_avoidance_.static_recovery_engaged&&
        !v2_avoidance_.map_lateral_started&&gap>clearance+4.0;
    const bool may_reselect_side=may_retime&&!v2_avoidance_.map_side_reselected&&static_confirmed&&
        (execution_->avoid.phase==AvoidPhase::TRACKING||execution_->avoid.phase==AvoidPhase::SHIFT)&&
        !v2_avoidance_.has_requested_target_d&&
        execution_->avoid.requested_direction==LateralDirection::NONE&&
        std::fabs(initial_slope)<.10;

    const auto profile_now=std::chrono::steady_clock::now();
    const double profile_age=std::chrono::duration<double>(
        profile_now-v2_avoidance_.map_speed_profile_at).count();
    // The configured planning period is also used by KEEP's continuity policy.
    // Actual elapsed time rejects stale history after a runtime pause.
    const double elapsed=std::max(replanning_period_s_,profile_age);
    const bool bypass_active=committed&&v2_avoidance_.map_speed_profile_valid&&
        (execution_->avoid.phase==AvoidPhase::SHIFT ||
         execution_->avoid.phase==AvoidPhase::PASS || returning);
    // A newly reduced cap must not borrow a release from the faster profile.
    const bool release_cap_unchanged=!v2_avoidance_.map_recovery_path.v.empty()&&
        v2_avoidance_.map_recovery_path.v.back()<=crawl+.1;
    const double initial_a=AvoidAccelerationSeed(ego.a,ego.v,crawl,
        bypass_active,v2_avoidance_.map_recovery_path.a,dt,elapsed,
        decel,limits_.max_longitudinal_accel,release_cap_unchanged,
        release_cap_unchanged);
    const int recovery_object_id=v2_avoidance_.dynamic_target?execution_->avoid.obstacle_id:-1;
    const auto* recovery_ids=v2_avoidance_.dynamic_target?nullptr:&v2_avoidance_.margin_recovery_ids;
    const double forward_accel=v2_avoidance_.dynamic_target?limits_.max_longitudinal_accel:
        std::min({limits_.max_longitudinal_accel,limits_.max_forward_accel,avoid_cfg_.bypass_accel_cap});
    const auto reaches_recovery_exit=[&](const CartesianPath& path) {
        if(!command.recovery_clear)return true;
        if(path.x.empty() || path.y.empty())return false;
        double end_s=0;
        return ProjectPointToS(path.x.back(),path.y.back(),
                   command.recovery_min_end_s,end_s) &&
               end_s>=command.recovery_min_end_s;
    };

    // Generate in distance, then time-sample with observed initial v/a. Approach,
    // shift, pass and return are validated together, including the rear-axle body.
    bool keep_return_schedule=true;
    auto generate=[&](double target_d,double delay,double transition,double return_distance,double return_advance,bool direct_return,bool maintenance,CartesianPath& path,double* return_start=nullptr,double* return_end=nullptr){
        // A short remaining entry delay cannot accommodate a separate return to
        // the centerline. Start the bypass from the measured d/yaw instead.
        // The full trajectory still passes the same road and collision checks.
        const double center_distance=std::max(2.0,LowSpeedLateralTransitionDistance(d,0.0)*1.25);
        const double shift_start=(maintenance||direct_return)?0.0:(delay<center_distance?0.0:delay);
        const double settle=maintenance ? std::max(10.0,std::min(25.0,ego.v*2.5)) : std::max(1.0,std::min(20.0,shift_start));
        const auto approach=MakeQuintic(d,initial_slope,initial_second,0,0,0,settle);
        const double shift_initial=shift_start>0?0.0:d;
        const auto shift=MakeQuintic(shift_initial,shift_start>0?0.0:initial_slope,
            shift_start>0?0.0:initial_second,target_d,0,0,transition);
        double nominal_pass=gap+v2_avoidance_.obstacle_longitudinal_half_extent+
            vehicle_shape_.rear_overhang+avoid_cfg_.pass_clearance-return_advance;
        if(shadow_dynamic_bypass_&&slow_dynamic&&target->length>2.) {
            // A long slow leader moves while we shift/pass. A return schedule
            // built only around its observed station returns into its future
            // body. Extend this private research candidate by relative speed;
            // every timed prediction/road check below remains authoritative.
            const auto tangent=Interpolate(ref_,s).theta;
            const double leader_speed=target->vx*std::cos(tangent)+target->vy*std::sin(tangent);
            if(leader_speed>.1&&leader_speed<crawl-.2)
                nominal_pass+=leader_speed*std::max(0.,nominal_pass)/(crawl-leader_speed);
        }
        const double scheduled_pass=committed&&keep_return_schedule&&v2_avoidance_.map_return_schedule_valid
            ? ReferenceArcDistance(ref_, s, v2_avoidance_.map_return_start_s) : nominal_pass;
        const double pass_end=std::max(shift_start+transition,scheduled_pass);
        if(return_start)*return_start=std::fmod(s+(direct_return?0.0:pass_end)+length,length);
        if(return_end)*return_end=std::fmod(s+(direct_return?transition:pass_end+return_distance)+length,length);
        const auto back=MakeQuintic(target_d,0,0,0,0,0,return_distance);
        const double ordinary_total=maintenance?std::max(1.0,gap-clearance):
            (direct_return?transition:pass_end+return_distance);
        // Recovery may only release the signal brake with a single checked
        // path beyond the conflict exit. After the normal bypass returns to
        // d=0, continue on the same reference with every road/collision and
        // dynamics check still active.
        const double total=!maintenance&&command.recovery_clear?
            std::max(ordinary_total,
                     std::max(0.0,command.recovery_min_end_s-s)+.5):
            ordinary_total;
        // Never accept a short return that ends before the target can be
        // passed; completion must not hand KEEP an obstacle still ahead.
        if(direct_return&&total<gap+v2_avoidance_.obstacle_longitudinal_half_extent+
            vehicle_shape_.rear_overhang+avoid_cfg_.pass_clearance)return false;
        // Pending/blocked-path maintenance retains observed braking. Only a
        // moving bypass that passes the full checks may use the release seed.
        double u=0,v=std::max(0.0,ego.v),a=maintenance
            ? std::min(initial_a,clip(ego.a,-decel,limits_.max_longitudinal_accel))
            : initial_a;
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
            if(u>=total-.05 || (maintenance&&(n*dt>=3.0||v<.02)&&n>0))
                return path.x.size()>1&&PathObeysSteeringRate(path,dt,wheelbase_,limits_.steering);
            // Jerk ramp distance reserve begins deceleration before the lateral
            // transition. No instantaneous replacement of measured speed by cap.
            const double remaining=std::max(0.0,(maintenance?total:shift_start)-u);
            const double ramp=std::max(0.0,v)*decel/std::max(.1,jerk);
            const double terminal=maintenance?0.0:crawl;
            double cap=std::min(speed,std::sqrt(terminal*terminal+2*decel*std::max(0.0,remaining-ramp)));
            if(maintenance){
                cap=std::min(cap,crawl); // Static classification is pending: brake immediately.
            }
            const double desired=clip((cap-v)/.8,-decel,forward_accel);
            const double next_a=clip(desired,a-jerk*dt,a+jerk*dt);
            const double next_v=std::max(0.0,v+(a+next_a)*.5*dt);
            u+=(v+next_v)*.5*dt/metric;v=next_v;a=next_a;
            if(!maintenance&&v<.02&&speed<.05)return false;
        }
        return false;
    };

    if(v2_avoidance_.static_progress_level>0)
        PlanStaticAvoidRecovery(ego,s,d,speed,initial_a,obstacles,command,committed,result,recovery_deadline);
    if(!result.candidate_valid&&!v2_avoidance_.dynamic_target&&(committed||static_confirmed)&&
       !result.direction_conflict&&!result.target_behind_ego) {
        const bool reconnect_turn=v2_avoidance_.map_search_turn%2==1&&v2_avoidance_.map_corridor_cursor==0;
        const auto corridor_deadline=reconnect_turn?search_started:search_deadline;
        PlanStaticAvoidCorridor(ego,s,d,speed,initial_a,obstacles,command,committed,result,corridor_deadline);
        if(reconnect_turn&&!result.candidate_valid)result.search_pending=false;
    }

    // Keep the already accepted recovery/bypass curve while tracking it closely.
    // Re-fitting the same tiny boundary overlap each cycle can change the curve
    // enough to lose every candidate. This is never an unchecked stale-path fallback.
    if(!result.candidate_valid&&committed&&v2_avoidance_.static_progress_level==0&&
       !v2_avoidance_.map_recovery_path.x.empty()&&
       std::fabs(initial_a-clip(ego.a,-decel,limits_.max_longitudinal_accel))<1e-6){
        const auto& cached=v2_avoidance_.map_recovery_path;
        std::size_t nearest=0;double fraction=0,distance=INFINITY;
        for(std::size_t i=0;i+1<cached.x.size();++i){
            const double dx=cached.x[i+1]-cached.x[i],dy=cached.y[i+1]-cached.y[i];
            const double norm=dx*dx+dy*dy;
            const double f=norm>1e-12?clip(((ego.x-cached.x[i])*dx+(ego.y-cached.y[i])*dy)/norm,0.0,1.0):0.0;
            const double delta=std::hypot(cached.x[i]+f*dx-ego.x,cached.y[i]+f*dy-ego.y);
            if(delta<distance){distance=delta;nearest=i;fraction=f;}
        }
        auto value=[&](const std::vector<double>& values,double index,bool angle=false){
            const std::size_t i=std::min(static_cast<std::size_t>(index),values.size()-1);
            const std::size_t j=std::min(i+1,values.size()-1);
            const double f=index-i;
            return values[i]+f*(angle?std::remainder(values[j]-values[i],2*M_PI):values[j]-values[i]);
        };
        const double projected=nearest+fraction;
        if(projected+1<cached.x.size()&&distance<.05&&
           std::fabs(std::remainder(value(cached.yaw,projected,true)-ego.yaw,2*M_PI))<.02&&
           std::fabs(value(cached.v,projected)-ego.v)<.10&&std::fabs(value(cached.a,projected)-ego.a)<.5){
            CartesianPath continuation;
            for(double offset=0;offset<std::ceil(cached.x.size()-1-projected)+1;offset+=1.0){
                const double index=std::min(projected+offset,double(cached.x.size()-1));
                continuation.x.push_back(value(cached.x,index));continuation.y.push_back(value(cached.y,index));
                continuation.yaw.push_back(value(cached.yaw,index,true));continuation.kappa.push_back(value(cached.kappa,index));
                continuation.v.push_back(value(cached.v,index));continuation.a.push_back(value(cached.a,index));
            }
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
            const auto road=avoid_road_map_.EvaluateRecoveryPath(continuation,map_shape,dt,
                recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_);
            int blocking=-1;bool margin_recovery=false;
            kinematic=kinematic&&PathObeysSteeringRate(continuation,dt,wheelbase_,limits_.steering);
            const bool clear=kinematic&&road.drivable&&lane_clear(continuation)&&CartesianAvoidTrajectoryClear(continuation,obstacles,
                vehicle_shape_,collision_cfg_,dt,&blocking,recovery_object_id,&margin_recovery,recovery_ids);
            if(clear && reaches_recovery_exit(continuation)){
                ++result.generated_candidate_count;++result.valid_candidate_count;
                last_candidate_paths_.push_back(continuation);last_candidate_validity_.push_back(true);
                result.candidate_valid=true;result.spatial_candidate_selected=true;
                v2_avoidance_.map_search_cursor=0;
                v2_avoidance_.map_reconnection_cursor=0;
                result.selected_target_d=v2_avoidance_.map_recovery_path_target_d;
                result.selected_cost=AvoidRoadMap::PaintCost(road);
                result.candidate_path=continuation;result.escape_recovery_required=margin_recovery;
                result.map_entry_s=v2_avoidance_.map_entry_s;result.map_transition_distance=v2_avoidance_.map_transition_distance;
                result.map_return_schedule_valid=v2_avoidance_.map_return_schedule_valid;
                result.map_return_start_s=v2_avoidance_.map_return_start_s;
                result.map_return_end_s=v2_avoidance_.map_return_end_s;
                v2_avoidance_.map_recovery_path=continuation;
                v2_avoidance_.map_speed_profile_valid=true;
                v2_avoidance_.map_speed_profile_at=std::chrono::steady_clock::now();
                retain_static_policy();
                output=continuation;UpdateCommittedAvoidLifecycle(s,d,ego.v,result,
                    CanHandoffAvoidToKeep(ego,s,d,obstacles));
                last_failure_reason_=PlanFailureReason::NONE;return true;
            }
        }
    }

    const auto reconnection_deadline=search_deadline;
    bool selected_fresh_entry=false;
    bool selected_direct_return=returning;
    const bool pending_corridor=!committed&&result.search_pending&&v2_avoidance_.map_corridor_cursor>0;
    if(!pending_corridor&&!result.candidate_valid&&!v2_avoidance_.dynamic_target&&
       v2_avoidance_.map_search_turn%2==1) {
        CartesianPath connected;
        if(PlanAvoidReconnection(ego,s,d,speed,initial_a,obstacles,committed,connected,result,reconnection_deadline)) {
            selected_fresh_entry=true;
            selected_direct_return=returning||execution_->avoid.phase==AvoidPhase::PASS;
        }
    }
    double best=std::numeric_limits<double>::infinity();
    bool best_touches_paint=true;
    if(!pending_corridor&&!result.candidate_valid&&!result.direction_conflict&&!result.target_behind_ego&&(committed||static_confirmed)) {
        std::vector<double> targets;
        if(committed) targets.push_back(returning?0.0:v2_avoidance_.target_d);
        else for(int i=0;i<v2_avoid_targets_.count;++i)targets.push_back(v2_avoid_targets_.candidates[i]);
        if(!committed && v2_avoidance_.dynamic_target && !shadow_dynamic_bypass_) {
            targets.clear();
            const double bound=.5*(lane_width_-vehicle_shape_.width)-collision_cfg_.safety_margin;
            for(double offset=.1;offset<=bound+1e-6;offset+=.1) {
                targets.push_back(-offset);targets.push_back(offset);
            }
            if(bound>0) {targets.push_back(-bound);targets.push_back(bound);}
        }
        if(!committed)std::stable_sort(targets.begin(),targets.end(),[&](double a,double b){
            // The nearer lateral escape avoids searching wide, off-road
            // shifts first on a narrow approach. Full-body road and obstacle
            // checks remain mandatory for the selected candidate.
            const double da=std::fabs(a-d),db=std::fabs(b-d);
            if(std::fabs(da-db)>1e-6)return da<db;
            const bool a_preferred=((a<v2_avoidance_.obstacle_d)==avoid_cfg_.prefer_right_when_equal);
            const bool b_preferred=((b<v2_avoidance_.obstacle_d)==avoid_cfg_.prefer_right_when_equal);
            return a_preferred&&!b_preferred;
        });
        auto evaluate_target=[&](double target_d,const std::vector<double>& delays,double return_advance,bool direct_return=false,bool fresh_entry=false){
            if(search_expired)return;
            // A near-zero offset error can still have a substantial heading
            // error. Reserve distance for that slope to settle; the sampled
            // curvature, acceleration, road and collision checks remain final.
            const double heading_distance=6.0*std::fabs(initial_slope)/
                std::max(.01,limits_.max_curvature);
            const double natural=std::max(
                transition_distance(d,target_d),heading_distance);
            const double scheduled_remaining=committed&&!fresh_entry&&v2_avoidance_.map_return_schedule_valid
                ? ReferenceArcDistance(ref_, s, v2_avoidance_.map_return_end_s) : natural;
            const double base_transition=direct_return&&keep_return_schedule?std::max(natural,scheduled_remaining):natural;
            for(double delay:delays){
                // The same shorter-distance option used by full bypass returns
                // is also available when reconnecting a blocked RETURN.
                const double first_scale=direct_return&&!keep_return_schedule?.75:1.0;
                // Refine the existing quarter-scale grid for measured-pose
                // RETURN reconnection; narrow feasible intervals must not be
                // skipped between curvature and downstream-collision failures.
                const double scale_step=direct_return&&!keep_return_schedule?.125:.25;
                for(double scale=first_scale;scale<=3.0+1e-6;scale+=scale_step){
                    double transition=base_transition*scale;
                    if(committed&&!direct_return&&!fresh_entry)transition=std::max(natural,
                        v2_avoidance_.map_transition_distance-std::max(0.0,-ReferenceArcDistance(ref_, s, v2_avoidance_.map_entry_s)))*scale;

                    // Search return distance independently of entry length. The
                    // former long return remains a fallback; every shorter
                    // candidate passes the same kinematic, road and OBB checks.
                    const double return_base=transition_distance(target_d,0.0);
                    std::vector<double> returns=direct_return
                        ? std::vector<double>{1.0}
                        : std::vector<double>{std::max(1.0,return_base*.75),return_base,
                            return_base*1.25,std::max(transition,return_base*1.25)};
                    if(!direct_return&&!fresh_entry&&committed&&keep_return_schedule&&v2_avoidance_.map_return_schedule_valid){
                        const double center_distance=std::max(2.0,LowSpeedLateralTransitionDistance(d,0.0)*1.25);
                        const double entry=delay<center_distance?0.0:delay;
                        const double pass=std::max(entry+transition,
                            ReferenceArcDistance(ref_, s, v2_avoidance_.map_return_start_s));
                        returns={std::max(1.0,ReferenceArcDistance(ref_, s, v2_avoidance_.map_return_end_s)-pass)};
                    }
                    std::sort(returns.begin(),returns.end());
                    returns.erase(std::unique(returns.begin(),returns.end(),
                        [](double a,double b){return std::fabs(a-b)<1e-6;}),returns.end());
                    for(double return_distance:returns){
                        const double effective_advance=keep_return_schedule&&v2_avoidance_.map_return_schedule_valid?0.0:return_advance;
                        if(!evaluated.emplace(target_d,delay,transition,return_distance,effective_advance,direct_return,keep_return_schedule).second)continue;
                        const std::size_t index=candidate_index++;
                        if(index<resume_at)continue;
                        if(std::chrono::steady_clock::now()>=search_deadline){
                            search_expired=true;v2_avoidance_.map_search_cursor=index;return;
                        }
                        double cost=delay+transition+(direct_return?0.0:return_distance);
                        if(!committed && ((target_d<v2_avoidance_.obstacle_d)!=avoid_cfg_.prefer_right_when_equal))
                            cost+=avoid_cfg_.direction_tiebreak_cost;
                        if(!best_touches_paint&&cost>=best)continue;
                        CartesianPath candidate;++result.generated_candidate_count;
                        int blocking=-1;bool margin_recovery=false;
                        double return_start=0,return_end=0;
                        const bool generated=generate(target_d,delay,transition,return_distance,return_advance,direct_return,false,candidate,&return_start,&return_end);
                        ++diagnostic.generated;
                        if(!generated)++diagnostic.kinematic_rejected;
                        const auto road_evaluation=generated
                            ? avoid_road_map_.EvaluateRecoveryPath(candidate,map_shape,dt,recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_)
                            : AvoidRoadMap::PathEvaluation{};
                        const bool road=road_evaluation.drivable;
                        if(generated){
                            diagnostic.least_overlap=std::min(diagnostic.least_overlap,road_evaluation.maximum_overlap_m);
                            diagnostic.least_paint=std::min(diagnostic.least_paint,road_evaluation.longest_paint_contact_seconds);
                            if(!road)++diagnostic.road_rejected;
                            if(diagnostic.enabled&&!road){
                                // Counterfactual audit only; it cannot authorize
                                // a candidate or run in the normal control path.
                                const auto physical=avoid_road_map_.EvaluateRecoveryPath(candidate,map_shape,dt,
                                    recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,
                                    v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m,
                                    false,solid_paint_contact_elapsed_s_);
                                if(physical.drivable)++diagnostic.paint_only;else ++diagnostic.boundary;
                            }
                        }
                        const bool clear=generated&&road&&lane_clear(candidate)&&
                            CartesianAvoidTrajectoryClear(candidate,obstacles,vehicle_shape_,collision_cfg_,dt,&blocking,recovery_object_id,&margin_recovery,recovery_ids);
                        if(clear)++diagnostic.clear;
                        else if(generated&&road)++diagnostic.collision_rejected;
                        if(!candidate.x.empty()){last_candidate_paths_.push_back(candidate);last_candidate_validity_.push_back(clear);}
                        if(!clear){if(blocking>=0)result.blocking_object_id=blocking;continue;}
                        ++result.valid_candidate_count;
                        cost+=AvoidRoadMap::PaintCost(road_evaluation);
                        const bool touches_paint=road_evaluation.paint_contact_seconds>0.0;
                        if(result.candidate_valid&&
                           ((touches_paint&&!best_touches_paint)||
                            (touches_paint==best_touches_paint&&cost>=best)))continue;
                        best_touches_paint=touches_paint;
                        selected_search_index=index;
                        indexed_candidate_selected=true;
                        selected_direct_return=direct_return;
                        selected_fresh_entry=fresh_entry;
                        result.map_return_schedule_valid=true;
                        result.map_return_start_s=return_start;result.map_return_end_s=return_end;
                        best=cost;result.blocking_object_id=-1;result.escape_recovery_required=margin_recovery;result.candidate_valid=true;result.spatial_candidate_selected=true;
                        result.selected_target_d=target_d;result.selected_cost=cost;result.candidate_path=candidate;
                        // Match the station to the actual generated shift. A short
                        // delay is folded into an immediate measured-pose transition.
                        const double center_distance=std::max(2.0,LowSpeedLateralTransitionDistance(d,0.0)*1.25);
                        result.map_entry_s=std::fmod(s+(delay<center_distance?0.0:delay),length);
                        result.map_transition_distance=transition;
                    }
                }
            }
        };
        const double locked_delay=committed
            ? std::max(0.0,ReferenceArcDistance(ref_, s, v2_avoidance_.map_entry_s)):0.0;
        std::vector<double> free_delays;
        const double latest_entry=std::max(0.0,std::min(40.0,gap-clearance-1.0));
        for(double delay=0;delay<=latest_entry+1e-6;delay+=2.0)free_delays.push_back(delay);
        const std::vector<double> nominal_delays=committed
            ? std::vector<double>{locked_delay}:free_delays;
        // A valid nominal/cached path wins over opportunistic side or return
        // changes. Broader blocked-path searches are cooperatively sliced.
        auto search_return_start=[&](double return_advance,bool broaden){
            // Preserve any valid nominal solution. Observation noise must not cause
            // an opportunistic change of entry station, side or bypass width.
            for(double target_d:targets)evaluate_target(target_d,nominal_delays,return_advance,returning);
            // A blocked outward reservation is not a minimum bypass width.
            // Reconnect from measured pose to narrower targets on the SAME side,
            // never inward past current target geometry. Full swept-body checks
            // include every object and protect against premature return.
            if(!result.candidate_valid&&!search_expired&&committed&&!returning&&
               !v2_avoidance_.has_requested_target_d){
                const bool left=v2_avoidance_.target_d>=v2_avoidance_.obstacle_d;
                const auto same=DecideAvoidTargets(v2_avoidance_,
                    left?LateralDirection::LEFT:LateralDirection::RIGHT);
                if(same.count){
                    const double minimum=same.candidates[0];
                    const double sign=left?1.0:-1.0;
                    const double width=sign*(v2_avoidance_.target_d-minimum);
                    if(width>1e-6){
                        const bool saved_schedule=keep_return_schedule;
                        keep_return_schedule=false;
                        const double step=std::max(.25,avoid_cfg_.committed_recovery_step);
                        for(double extra=0;extra<width-1e-6&&!result.candidate_valid&&!search_expired;extra+=step)
                            evaluate_target(minimum+sign*extra,std::vector<double>{0.0},return_advance,false,true);
                        keep_return_schedule=saved_schedule;
                    }
                }
            }
            // An accepted side is an execution commitment only after measured
            // lateral motion. A blocked, still-straight approach may choose the
            // other side, unless the FSM explicitly owns the direction/pose.
            if(!result.candidate_valid&&!search_expired&&may_reselect_side){
                const auto alternatives=DecideAvoidTargets(v2_avoidance_,LateralDirection::NONE);
                const bool locked_left=v2_avoidance_.target_d>=v2_avoidance_.obstacle_d;
                for(int i=0;i<alternatives.count&&!result.candidate_valid&&!search_expired;++i){
                    const double alternative=alternatives.candidates[i];
                    if((alternative>=v2_avoidance_.obstacle_d)==locked_left)continue;
                    const bool saved_schedule=keep_return_schedule;
                    keep_return_schedule=false;
                    evaluate_target(alternative,free_delays,return_advance,false,true);
                    const double step=std::max(.25,avoid_cfg_.committed_recovery_step);
                    const double expansion=std::min(2.0,std::max(0.0,avoid_cfg_.committed_recovery_max_expansion));
                    for(double extra=step;extra<=expansion+1e-6&&!result.candidate_valid&&!search_expired;extra+=step){
                        const double widened=alternative+(locked_left?-extra:extra);
                        if(std::fabs(widened)<=4.0)evaluate_target(widened,free_delays,return_advance,false,true);
                    }
                    keep_return_schedule=saved_schedule;
                }
            }
            if(!broaden)return;
            if(!result.candidate_valid&&may_retime){
                evaluate_target(targets.front(),free_delays,return_advance);
            }
            if(!result.candidate_valid&&committed&&!returning&&
               execution_->avoid.phase==AvoidPhase::PASS){
                evaluate_target(0.0,std::vector<double>{0.0},0.0,true);
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
                        evaluate_target(widened,may_retime?free_delays:nominal_delays,return_advance);
                    }
                }
            }
        };
        // Test nominal geometry and nearby return starts before expensive
        // widening/entry grids. Preserve a valid nominal or cached solution.
        search_return_start(0.0,false);
        if(!returning)for(double advance=.5;advance<=1.5+1e-6&&!result.candidate_valid&&!search_expired;advance+=.5)
            search_return_start(advance,false);
        if(!result.candidate_valid&&!search_expired&&committed){
            keep_return_schedule=false;
            search_return_start(0.0,false);
            if(!returning)for(double advance=.5;advance<=1.5+1e-6&&!result.candidate_valid&&!search_expired;advance+=.5)
                search_return_start(advance,false);
        }
        if(!result.candidate_valid&&!search_expired){
            search_return_start(0.0,true);
            if(!returning)for(double advance=.5;advance<=1.5+1e-6&&!result.candidate_valid&&!search_expired;advance+=.5)
                search_return_start(advance,true);
        }

    }
    if(!pending_corridor&&!result.candidate_valid&&!result.direction_conflict&&!result.target_behind_ego&&
       (committed||static_confirmed)){
        CartesianPath reconnected;
        if(PlanAvoidReconnection(ego,s,d,speed,initial_a,obstacles,committed,reconnected,result,reconnection_deadline)){
            selected_fresh_entry=true;
            selected_direct_return=returning||execution_->avoid.phase==AvoidPhase::PASS||gap<-(v2_avoidance_.obstacle_longitudinal_half_extent+
                vehicle_shape_.rear_overhang+avoid_cfg_.pass_clearance);
        }
    }
    if(result.candidate_valid && !lane_clear(result.candidate_path)) result.candidate_valid=false;
    if(!result.candidate_valid)
        PlanStaticAvoidRecovery(ego,s,d,speed,initial_a,obstacles,command,committed,result,recovery_deadline);
    if(result.candidate_valid){
        retain_static_policy();
        result.blocking_object_id=-1;
        // Keep the successful search position for PREPARE -> EXECUTE
        // revalidation, including static obstacles. Restarting at zero can
        // alternate READY/WAITING forever when the checked candidate lies
        // beyond one search slice. Only the position is retained; geometry,
        // road and all current objects are checked again on every cycle.
        v2_avoidance_.map_search_cursor=!committed&&indexed_candidate_selected
            ? selected_search_index : 0;
        v2_avoidance_.map_reconnection_cursor=0;
    }
    else if(!search_expired)v2_avoidance_.map_search_cursor=0;
    if(committed){
        if(!result.candidate_valid){
            ++v2_avoidance_.map_search_turn;
            v2_avoidance_.map_speed_profile_valid=false;
            last_failure_reason_=result.blocking_object_id>=0?
                PlanFailureReason::COLLISION_BLOCKED:PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            // A fresh checked braking trajectory, never the stale bypass, can
            // still be published while the next search slice remains pending.
            output=CartesianPath{};
            const auto braking_clear=[&](const CartesianPath& path) {
                return PathObeysSteeringRate(path,dt,wheelbase_,limits_.steering)&&
                    avoid_road_map_.EvaluateRecoveryPath(path,map_shape,dt,recovery_budget,
                        v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,
                        v2_avoidance_.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_).drivable&&
                    CartesianAvoidTrajectoryClear(path,obstacles,vehicle_shape_,collision_cfg_,dt,
                        nullptr,v2_avoidance_.dynamic_target?execution_->avoid.obstacle_id:-1,
                        nullptr,&v2_avoidance_.margin_recovery_ids);
            };
            PlanCollisionBrakingPathWithSafety(ego,s,d,obstacles,output,
                limits_.max_longitudinal_accel*.8,0,0,false,braking_clear);
            return false;
        }
        if(!result.map_group_valid&&v2_avoidance_.map_group_valid&&
           ReferenceArcDistance(ref_, s, v2_avoidance_.map_group_end_s)>0) {
            result.map_group_valid=true;result.map_group_end_s=v2_avoidance_.map_group_end_s;
        }
        if(selected_direct_return&&(!result.map_group_valid||ReferenceArcDistance(ref_, s, result.map_group_end_s)<=0)){
            execution_.observeEarlyReturnCandidate(true);
        } else {
            if ((result.selected_target_d >= v2_avoidance_.obstacle_d) !=
                (v2_avoidance_.target_d >= v2_avoidance_.obstacle_d)) {
                v2_avoidance_.map_side_reselected = true;
            }
            v2_avoidance_.target_d=result.selected_target_d;
            if((may_retime||selected_fresh_entry)&&!v2_avoidance_.static_recovery_engaged){
                const double entry_gap=ReferenceArcDistance(ref_, s, result.map_entry_s);
                v2_avoidance_.map_entry_s=result.map_entry_s;
                v2_avoidance_.map_transition_distance=result.map_transition_distance;
                v2_avoidance_.shift_entry_distance=gap-std::max(0.0,entry_gap);
                execution_.retimeAvoidEntry(entry_gap>0.0);
                v2_avoidance_.start_d=d;
            } else {
                execution_.revalidateAvoidOffset(
                    std::fabs(d-v2_avoidance_.target_d)>avoid_cfg_.lateral_tolerance);
            }
        }
        v2_avoidance_.map_return_schedule_valid=result.map_return_schedule_valid;
        v2_avoidance_.map_return_start_s=result.map_return_start_s;
        v2_avoidance_.map_return_end_s=result.map_return_end_s;
        v2_avoidance_.map_group_valid=result.map_group_valid;
        v2_avoidance_.map_group_end_s=result.map_group_end_s;
        v2_avoidance_.static_progress_level=result.static_progress_level;
        v2_avoidance_.map_spatial_geometry=result.spatial_geometry.x.empty()?
            result.candidate_path:result.spatial_geometry;
        v2_avoidance_.map_recovery_path=result.candidate_path;
        v2_avoidance_.map_recovery_path_target_d=result.selected_target_d;
        v2_avoidance_.map_speed_profile_valid=true;
        v2_avoidance_.map_speed_profile_at=std::chrono::steady_clock::now();
        v2_avoidance_.map_search_turn=0;
        if(v2_avoidance_.static_recovery_engaged&&execution_->avoid.phase==AvoidPhase::TRACKING)
            execution_.beginAvoidShiftProbe();
        output=result.candidate_path;UpdateCommittedAvoidLifecycle(s,d,ego.v,result,
                    CanHandoffAvoidToKeep(ego,s,d,obstacles));
        last_failure_reason_=PlanFailureReason::NONE;return true;
    }
    // PREPARE never publishes its bypass. Brake along the reference until static
    // classification and a complete road-constrained bypass are both available.
    if(result.candidate_valid) {
        // A PREPARE hint is revalidated, never executed before COMMIT. Retain
        // its geometry so a late search result does not fall back to WAITING
        // on each subsequent slice while approaching the same command.
        v2_avoidance_.map_spatial_geometry=result.spatial_geometry.x.empty()?result.candidate_path:result.spatial_geometry;
        v2_avoidance_.map_recovery_path_target_d=result.selected_target_d;
        v2_avoidance_.map_entry_s=result.map_entry_s;
        v2_avoidance_.map_transition_distance=result.map_transition_distance;
        v2_avoidance_.map_return_schedule_valid=result.map_return_schedule_valid;
        v2_avoidance_.map_return_start_s=result.map_return_start_s;
        v2_avoidance_.map_return_end_s=result.map_return_end_s;
        v2_avoidance_.map_group_valid=result.map_group_valid;
        v2_avoidance_.map_group_end_s=result.map_group_end_s;
        v2_avoidance_.static_progress_level=result.static_progress_level;
        v2_avoidance_.map_search_turn=0;
    } else if(!pending_corridor)++v2_avoidance_.map_search_turn;
    CartesianPath approach;
    const bool approach_generated=generate(0,0,1,1,0,false,true,approach);
    // The Controller can follow a WAITING approach only if the same checked
    // trajectory includes its stop. Complete the usual three-second prefix
    // with the common jerk-limited braking generator; then recheck the entire
    // combined path against current objects at their original time origin.
    // An incomplete prefix retains the existing fail-closed Controller hold.
    if(approach_generated && !approach.v.empty() && approach.v.back()>.1) {
        const std::size_t n=approach.x.size()-1;
        CartesianState tail_ego{approach.x[n],approach.y[n],approach.yaw[n],
            approach.kappa[n],approach.v[n],approach.a[n]};
        double tail_s=s,tail_d=d;
        CartesianPath tail;
        if(ProjectPointToFrenet(tail_ego.x,tail_ego.y,s,100,5,tail_s,tail_d) &&
           PlanCollisionBrakingPath(tail_ego,tail_s,tail_d,{},tail,decel) &&
           tail.x.size()>1) {
            auto append=[](std::vector<double>& into,const std::vector<double>& from) {
                into.insert(into.end(),from.begin()+1,from.end());
            };
            CartesianPath complete=approach;
            append(complete.x,tail.x);append(complete.y,tail.y);append(complete.yaw,tail.yaw);
            append(complete.kappa,tail.kappa);append(complete.v,tail.v);append(complete.a,tail.a);
            if(avoid_road_map_.EvaluateRecoveryPath(complete,map_shape,dt,recovery_budget,
                   v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,
                   v2_avoidance_.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_).drivable &&
               CartesianAvoidTrajectoryClear(complete,obstacles,vehicle_shape_,collision_cfg_,dt))
                approach=std::move(complete);
        }
    }
    // The gradual prefix may already consume the available stopping distance.
    // In that case start the common braking profile at the measured state;
    // never certify a moving endpoint as a complete waiting trajectory.
    if(approach_generated && !approach.v.empty() && approach.v.back()>.01) {
        CartesianPath immediate;
        if(PlanCollisionBrakingPath(ego,s,d,obstacles,immediate,decel) &&
           avoid_road_map_.EvaluateRecoveryPath(immediate,map_shape,dt,recovery_budget,
               v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,
               v2_avoidance_.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_).drivable)
            approach=std::move(immediate);
    }
    const bool approach_road=approach_generated&&avoid_road_map_.EvaluateRecoveryPath(approach,map_shape,dt,recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_).drivable;
    if(approach_generated&&approach_road&&
       CartesianAvoidTrajectoryClear(approach,obstacles,vehicle_shape_,collision_cfg_,dt))output=std::move(approach);
    else {
        PlannerCommand base=command;base.target_speed=crawl;
        if(!BuildAvoidMaintenancePath(ego,start,base,obstacles,output))return false;
        if(output.x.size()>1&&!avoid_road_map_.EvaluateRecoveryPath(output,map_shape,dt,recovery_budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,v2_avoidance_.map_recovery_settled_travel_m,true,solid_paint_contact_elapsed_s_).drivable)BuildZeroSpeedHoldPath(ego,s,output);
        if(output.x.size()>1&&!CartesianAvoidTrajectoryClear(output,obstacles,
            vehicle_shape_,collision_cfg_,dt)){
            output=CartesianPath{};last_failure_reason_=PlanFailureReason::COLLISION_BLOCKED;return false;
        }
    }
    last_failure_reason_=PlanFailureReason::NONE;return true;
}

