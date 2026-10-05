#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/Generator/BoundaryConditionPolicy.hpp"
#include "frenet_planner/Speed/CurveSpeed.hpp"
#include "frenet_planner/Math/Polynomial.hpp"
#include "frenet_planner/Maneuver/ManeuverUtils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// STOP's braking fallback retains the current d. During a long
// approach that repeatedly freezes a tracking offset instead of correcting it.
// Rebuild only that constant-offset fallback, retaining its longitudinal stop
// schedule and the observed initial heading. Short stops stay unchanged.
bool RecenterWaitingFallback(const RefLine& ref,const CartesianState& ego,
    double s,double d,const KinematicLimits& limits,double service_decel,
    const CartesianPath& original,CartesianPath& centered) {
    if(original.x.size()<2||std::fabs(d)<.2)return false;
    std::vector<double> stations;double hint=s;
    for(size_t i=0;i<original.x.size();++i) {
        hint=FindClosestS(ref,original.x[i],original.y[i],&hint,5);
        const auto rp=Interpolate(ref,hint);
        double offset=(original.y[i]-rp.y)*std::cos(rp.theta)-
            (original.x[i]-rp.x)*std::sin(rp.theta);
        if(std::fabs(offset-d)>.005)return false;
        stations.push_back(hint);
    }
    const double settle=std::max(10.0,std::min(25.0,ego.v*2.5));
    if(stations.back()-s<settle)return false;
    const auto origin=Interpolate(ref,s);
    const double slope=(1-origin.kappa*d)*std::tan(std::remainder(ego.yaw-origin.theta,2*M_PI));
    if(!std::isfinite(slope))return false;
    const auto lateral=MakeQuintic(d,slope,0,0,0,0,settle);
    for(size_t i=0;i<stations.size();++i) {
        const auto rp=Interpolate(ref,stations[i]);
        const double metric=std::fabs(1-rp.kappa*d);
        if(metric<.1)return false;
        const double sd=original.v[i]/metric;
        const auto base=FrenetToCartesian(rp,stations[i],sd,0,d,0,0);
        const double sdd=(original.a[i]-base.a)/metric;
        const double q=std::min(settle,std::max(0.0,stations[i]-s));
        const auto point=FrenetToCartesian(rp,stations[i],sd,sdd,
            EvalPos(lateral,q),EvalVel(lateral,q),EvalAcc(lateral,q));
        if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits.max_curvature||
           point.v*point.v*std::fabs(point.kappa)>limits.max_lateral_accel||
           std::fabs(point.a)>std::max(service_decel,limits.max_longitudinal_accel))return false;
        centered.x.push_back(point.x);centered.y.push_back(point.y);centered.yaw.push_back(point.yaw);
        centered.kappa.push_back(point.kappa);centered.v.push_back(point.v);centered.a.push_back(point.a);
    }
    return !centered.x.empty();
}
// Fixed-reference spatial centering plus a bounded longitudinal profile. A
// long quartic from rest spends the whole horizon accelerating; it cannot
// certify the downstream exit at a modest end speed. This profile reaches
// cruise speed, follows the curve envelope, and checks the complete exit.
bool MakeMergeProfile(const RefLine& ref, const CartesianState& ego,
    const FrenetState& start, double cap, double duration, double dt,
    const CurveSpeedConfig& curve, const KinematicLimits& limits,
    double settle, double target_d, bool release_positive_accel, CartesianPath& path) {
    const auto origin=Interpolate(ref,start.s);
    const double slope=(1-origin.kappa*start.d)*
        std::tan(std::remainder(ego.yaw-origin.theta,2*M_PI));
    if(!std::isfinite(slope)||std::fabs(start.d)>1.25)return false;
    auto lateral=MakeQuintic(start.d,slope,0,target_d,0,0,settle);
    // Preview the actual offset geometry, rather than only reference curvature.
    // The shared envelope reserves distance for unwinding positive acceleration
    // under the jerk limit before the curve's braking boundary.
    RefLine geometry;
    double arc=0,previous_metric=1;
    const double span=std::max(80.0,std::max(cap,ego.v)*duration+30.0);
    for(double ds=0;ds<=span;ds+=.25) {
        auto rp=Interpolate(ref,start.s+ds);double q=std::min(ds,settle);
        double d=EvalPos(lateral,q),dp=EvalVel(lateral,q),dpp=EvalAcc(lateral,q);
        double m=std::hypot(1-rp.kappa*d,dp);
        auto point=FrenetToCartesian(rp,start.s+ds,0,0,d,dp,dpp);
        if(!std::isfinite(point.kappa)||m<.1)return false;
        if(ds>0)arc+=.125*(previous_metric+m);
        geometry.points.push_back({point.x,point.y,arc,point.yaw,point.kappa,0});
        previous_metric=m;
    }
    geometry.length=arc;
    auto envelope=BuildKeepSpeedEnvelope(geometry,0,ego.v,release_positive_accel?std::min(0.0,ego.a):ego.a,duration,dt,
        cap,curve,limits.max_longitudinal_accel,limits.max_lateral_accel,
        limits.max_longitudinal_jerk);
    if(envelope.empty())return false;
    auto target_at=[&](double position) {
        auto it=std::lower_bound(envelope.begin(),envelope.end(),position,
            [](const SpeedEnvelopePoint& p,double s){return p.s<s;});
        if(it==envelope.begin())return std::sqrt(it->speed_squared);
        if(it==envelope.end())return std::sqrt(envelope.back().speed_squared);
        const auto& before=*(it-1);
        double f=(position-before.s)/(it->s-before.s);
        return std::sqrt(std::max(0.0,before.speed_squared+f*(it->speed_squared-before.speed_squared)));
    };
    double travelled=0;
    double distance=0, speed=std::max(0.0,ego.v);
    double accel=std::clamp(ego.a,-limits.max_longitudinal_accel,limits.max_longitudinal_accel);
    // Try continuous profiles first. If none is kinematically feasible, drop
    // positive acceleration memory before escalating to a complete safe stop.
    if(release_positive_accel)accel=std::min(0.0,accel);
    double metric=std::hypot(1-origin.kappa*start.d,slope);
    if(metric<.1)return false;
    if(speed<.01&&accel<0)accel=0;
    for(int i=0;i<=static_cast<int>(std::ceil(duration/dt));++i) {
        auto rp=Interpolate(ref,start.s+distance);
        double q=std::min(distance,settle);
        double d=EvalPos(lateral,q),dp=EvalVel(lateral,q),dpp=EvalAcc(lateral,q);
        metric=std::hypot(1-rp.kappa*d,dp);
        if(metric<.1)return false;
        double sd=speed/metric;
        auto base=FrenetToCartesian(rp,start.s+distance,sd,0,d,dp,dpp);
        auto point=FrenetToCartesian(rp,start.s+distance,sd,(accel-base.a)/metric,d,dp,dpp);
        if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits.max_curvature ||
           speed*speed*std::fabs(point.kappa)>limits.max_lateral_accel+1e-6)return false;
        path.x.push_back(point.x);path.y.push_back(point.y);path.yaw.push_back(point.yaw);
        path.kappa.push_back(point.kappa);path.v.push_back(speed);path.a.push_back(accel);
        double target=std::min(cap,target_at(travelled));
        double requested=std::clamp(2*(target-speed),-limits.max_longitudinal_accel,limits.max_longitudinal_accel);
        double next_accel=std::clamp(requested,accel-limits.max_longitudinal_jerk*dt,accel+limits.max_longitudinal_jerk*dt);
        double next_speed=speed+.5*(accel+next_accel)*dt;
        // Braking to zero is a valid boundary, not a reverse trajectory.
        // Once stationary, negative acceleration is no longer a moving state.
        if(next_speed<0){next_speed=0;next_accel=0;}
        double step=.5*(speed+next_speed)*dt;
        travelled+=step;
        distance+=step/metric;
        speed=next_speed;accel=next_accel;
    }
    return !path.x.empty();
}
}

void FrenetPlanner::ResetRoundaboutMerge() {
    roundabout_merge_committed_=false;
    roundabout_preferred_d_=0;
    roundabout_bridge_active_=roundabout_bridge_used_=false;
    roundabout_bridge_travel_=roundabout_bridge_settled_=0;
    roundabout_safe_observations_=0;
    roundabout_last_observation_=roundabout_last_entry_=-1;
    ++roundabout_epoch_;
    last_roundabout_result_=MergePrepareResult{};
}
void FrenetPlanner::RevokeRoundaboutReadiness() {
    if (roundabout_merge_committed_) return;
    ++roundabout_epoch_;
    roundabout_safe_observations_=0;
    roundabout_last_observation_=roundabout_last_entry_=-1;
    last_roundabout_result_=MergePrepareResult{};
}
bool FrenetPlanner::CanCommitRoundaboutMerge(const MergePrepareResult& result) const {
    return roundabout_merge_enabled() && !roundabout_merge_committed_ &&
        result.epoch==roundabout_epoch_ && result.candidate_valid && result.ready &&
        last_roundabout_result_.ready && !result.candidate_path.x.empty();
}
void FrenetPlanner::ApplyRoundaboutMergeCommit(const MergePrepareResult&) {
    roundabout_merge_committed_=true;
    stop_=StopContext{};
}
bool FrenetPlanner::PlanRoundaboutMerge(const CartesianState& ego,
    const std::vector<TrackedObject>& obstacles,const PlannerCommand& command,
    double planning_time,double observation_stamp,CartesianPath& out_path,
    MergePrepareResult& result) {
    result=MergePrepareResult{};result.epoch=++roundabout_epoch_;
    last_roundabout_result_=result;
    last_failure_reason_=PlanFailureReason::NONE;
    if(!roundabout_merge_enabled() || command.mode!=MERGE ||
       command.merge_type!=MergeType::ROUNDABOUT ||
       command.source_link_id!=roundabout_merge_cfg_.source_link ||
       command.target_link_id!=roundabout_merge_cfg_.target_link ||
       command.merge_sa_id!=-1 || command.merge_sb_id!=-1 ||
       !std::isfinite(planning_time)||!std::isfinite(observation_stamp)||
       !std::isfinite(ego.x)||!std::isfinite(ego.y)||!std::isfinite(ego.yaw)||
       !std::isfinite(ego.v)||!std::isfinite(ego.a)||ego.v<0) {
        last_failure_reason_=PlanFailureReason::INVALID_COMMAND;return false;
    }
    speed_profiler_.Reset();
    last_profiled_acceleration_=0.0;
    auto ctx=PreparePlanningContext(ego,obstacles,command);
    last_s_=ctx.s;last_d_=ctx.d;has_last_s_=true;
    double dd,ddd;ArcDerivToTimeDeriv(ctx.s_dot,ctx.s_ddot,ctx.d_prime,ctx.d_pprime,dd,ddd);
    auto start=ClampInitialBoundaryState(ctx.s,ctx.s_dot,ctx.s_ddot,ctx.d,dd,ddd,limits_);
    result.progress=std::clamp((ctx.s-roundabout_zone_.stop_s)/
        (roundabout_zone_.rear_clear_s-roundabout_zone_.stop_s),0.0,1.0);
    const double cap=std::min(ctx.resolved_target_speed,roundabout_merge_cfg_.speed_cap);
    last_target_speed_=cap;
    if(roundabout_merge_committed_ && ctx.s>=roundabout_zone_.rear_clear_s &&
       !roundabout_zone_.Overlaps(ego,vehicle_shape_,collision_cfg_.safety_margin)) {
        PlannerCommand keep=command;keep.mode=LANE_KEEPING;keep.target_speed=cap;keep.has_target_speed=true;
        bool ok=KeepProcess(ego,start,ctx.s,ctx.d,start.s_d,start.s_dd,start.d_d,start.d_dd,
            obstacles,ctx.predicted_obstacles,cap,keep,out_path);
        result.maneuver_completed=ok;result.progress=ok?1:result.progress;
        last_roundabout_result_=result;return ok;
    }
    // Consume the one bridge episode from actual boundary contact, not from a
    // predicted corner metres ahead. Subsequent plans retain the same budget.
    if(roundabout_merge_committed_&&!roundabout_bridge_used_ &&
       !avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,vehicle_shape_)) {
        roundabout_bridge_active_=roundabout_bridge_used_=true;
        roundabout_bridge_start_time_=planning_time;
        roundabout_bridge_x_=ego.x;roundabout_bridge_y_=ego.y;
    }
    if(roundabout_bridge_active_) {
        double movement=std::hypot(ego.x-roundabout_bridge_x_,ego.y-roundabout_bridge_y_);
        roundabout_bridge_travel_+=movement;
        roundabout_bridge_x_=ego.x;roundabout_bridge_y_=ego.y;
        VehicleShape settled=vehicle_shape_;
        settled.width+=.10;settled.length+=.10;settled.rear_overhang+=.05;
        if(avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,settled))
            roundabout_bridge_settled_+=movement;
        else roundabout_bridge_settled_=0;
        if(roundabout_bridge_settled_>=.5)roundabout_bridge_active_=false;
    }
    double best_cost=std::numeric_limits<double>::infinity();
    // A departure certificate must also be obtainable from a slightly displaced
    // waiting pose. It does not execute movement during PREPARE: the stop path
    // remains authoritative, and the complete departure still needs gap and
    // physical collision checks. Never authorize a fresh boundary excursion
    // from an already contained pre-commit pose.
    const bool waiting_boundary_recovery = !roundabout_merge_committed_ &&
        ctx.s >= roundabout_zone_.stop_s-1.0 &&
        !avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,vehicle_shape_);
    bool selected_bridge=false, profile_failed=false;
    double selected_paint=0, selected_d=roundabout_preferred_d_;
    last_candidate_paths_.clear();last_candidate_validity_.clear();
    const double ordinary_settle=std::max(10.0,std::min(25.0,ego.v*2.5));
    // Keep the same mission reference and prefer the last selected local
    // offset, avoiding repeated left/right target changes at a raster corner.
    // Strict grid containment is preferred; an execution/waiting recovery bridge is bounded
    // by 15 cm, 10% of the footprint, 6 m and 5 s before settled re-entry.
    for(bool allow_bridge:{false,true}) {
    if(allow_bridge&&((!roundabout_merge_committed_&&!waiting_boundary_recovery)||result.candidate_valid))break;
    for(bool release_positive_accel:{false,true}) {
    if(release_positive_accel&&(ego.a<=0||!profile_failed||result.candidate_valid))break;
    std::vector<std::pair<double,double>> geometries{
        {ordinary_settle,roundabout_preferred_d_},{6,roundabout_preferred_d_},
        {10,.35},{10,-.35},{6,.35},{6,-.35}};
    if(roundabout_preferred_d_!=0){geometries.push_back({ordinary_settle,0});geometries.push_back({6,0});}
    for(const auto geometry:geometries) {
    for(double fraction:{1.0,.8,.6,.4}) {
        ++result.generated;
        CartesianPath path;
        bool valid=MakeMergeProfile(ref_,ego,start,cap*fraction,
            roundabout_merge_cfg_.max_horizon,path_cfg_.dt,curve_speed_cfg_,limits_,
            geometry.first,geometry.second,release_positive_accel,path);
        profile_failed|=!valid;
        AvoidRoadMap::PathEvaluation road;bool bridge=false;
        if(valid) {
            road=avoid_road_map_.EvaluatePath(path,vehicle_shape_,path_cfg_.dt);
            if(!road.drivable && allow_bridge && (roundabout_merge_committed_||waiting_boundary_recovery) &&
               (roundabout_bridge_active_||!roundabout_bridge_used_)) {
                road=avoid_road_map_.EvaluateMergeBoundaryBridge(path,vehicle_shape_,path_cfg_.dt,
                    6.0-roundabout_bridge_travel_,
                    roundabout_bridge_active_?5.0-(planning_time-roundabout_bridge_start_time_):5.0,
                    roundabout_bridge_settled_);
                bridge=road.drivable;
            }
            valid=road.drivable;
        }
        MergeCandidateCheck check;
        if(valid) {
            check=CheckRoundaboutCandidate(path,roundabout_zone_,ctx.predicted_obstacles,
                vehicle_shape_,collision_cfg_,path_cfg_.dt,merge_cfg_.min_front_time_gap,
                merge_cfg_.min_rear_time_gap,!roundabout_merge_committed_);
            valid=check.safe;
            if(check.blocking_object_id>=0)result.blocking_object_id=check.blocking_object_id;
        }
        last_candidate_paths_.push_back(path);last_candidate_validity_.push_back(valid);
        if(!valid)continue;
        ++result.valid;
        double cost=check.exit_time+AvoidRoadMap::PaintCost(road)+std::fabs(geometry.second);
        if(cost<best_cost){best_cost=cost;selected_d=geometry.second;selected_bridge=bridge;selected_paint=road.paint_contact_seconds;result.candidate_valid=true;
            result.entry_time=check.entry_time;result.exit_time=check.exit_time;
            result.candidate_path=std::move(path);}
    }
    // Prefer the original reference geometry when it is feasible. Alternate
    // local shapes rescue geometry or prolonged paint contact, with bounded work.
    if(result.candidate_valid&&selected_paint<.3)break;
    }
    if(result.candidate_valid)break;
    }
    if(result.candidate_valid)break;
    }
    if(result.candidate_valid)result.blocking_object_id=-1;
    if(roundabout_merge_committed_) {
        if(result.candidate_valid){
            if(selected_bridge&&!roundabout_bridge_active_&&
               !avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,vehicle_shape_)) {
                roundabout_bridge_active_=roundabout_bridge_used_=true;
                roundabout_bridge_start_time_=planning_time;
                roundabout_bridge_x_=ego.x;roundabout_bridge_y_=ego.y;
            }
            roundabout_preferred_d_=selected_d;
            out_path=result.candidate_path;last_roundabout_result_=result;return true;
        }
        last_roundabout_result_=result;
        bool braking=PlanCollisionBrakingPath(ego,ctx.s,ctx.d,ctx.predicted_obstacles,out_path);
        last_failure_reason_=braking?PlanFailureReason::NONE:PlanFailureReason::COLLISION_BLOCKED;
        return braking;
    }
    if(!result.candidate_valid) {
        roundabout_safe_observations_=0;roundabout_last_entry_=-1;
    } else if(observation_stamp>roundabout_last_observation_+1e-6) {
        double absolute_entry=planning_time+result.entry_time;
        if(roundabout_last_entry_>=0 && std::fabs(absolute_entry-roundabout_last_entry_)>
            roundabout_merge_cfg_.confirmation_time_tolerance)roundabout_safe_observations_=0;
        ++roundabout_safe_observations_;roundabout_last_entry_=absolute_entry;
    } else if(observation_stamp<roundabout_last_observation_-1e-6) {
        roundabout_safe_observations_=0;roundabout_last_entry_=-1;
    }
    roundabout_last_observation_=observation_stamp;
    result.ready=result.candidate_valid && roundabout_safe_observations_>=merge_cfg_.safe_confirm_cycles &&
        ctx.s>=roundabout_zone_.stop_s-1.0;
    // The PREPARE output remains a stop-capable approach/hold trajectory.
    // Its target is the rear axle at the map stopline, independent of the
    // normal traffic-light bumper offset. STOP's small buffer is compensated.
    PlannerCommand waiting=command;waiting.mode=STOP;waiting.has_target_speed=true;
    waiting.target_speed=cap;waiting.stop_position=roundabout_zone_.stop_s+stop_cfg_.stop_buffer;
    bool ok=StopProcess(ego,start,ctx.s,ctx.d,start.s_d,start.s_dd,start.d_d,start.d_dd,
        obstacles,ctx.predicted_obstacles,cap,waiting,out_path);
    if(ok) {
        CartesianPath centered;
        if(RecenterWaitingFallback(ref_,ego,ctx.s,ctx.d,limits_,
                stop_cfg_.max_service_decel,out_path,centered)&&
           avoid_road_map_.EvaluatePath(centered,vehicle_shape_,path_cfg_.dt).drivable&&
           CartesianTrajectoryClear(centered,ctx.predicted_obstacles,vehicle_shape_,
                collision_cfg_,path_cfg_.dt))out_path=std::move(centered);
    }
    if(!ok)result.ready=false;
    last_roundabout_result_=result;
    return ok;
}
