#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/Generator/BoundaryConditionPolicy.hpp"
#include "frenet_planner/Speed/CurveSpeed.hpp"
#include "frenet_planner/Math/Polynomial.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// Fixed-reference spatial centering plus a bounded longitudinal profile. A
// long quartic from rest spends the whole horizon accelerating; it cannot
// certify the downstream exit at a modest end speed. This profile reaches
// cruise speed, follows the curve envelope, and checks the complete exit.
bool MakeMergeProfile(const RefLine& ref, const CartesianState& ego,
    const FrenetState& start, double cap, double duration, double dt,
    const CurveSpeedConfig& curve, const KinematicLimits& limits,
    CartesianPath& path) {
    const auto origin=Interpolate(ref,start.s);
    const double slope=(1-origin.kappa*start.d)*
        std::tan(std::remainder(ego.yaw-origin.theta,2*M_PI));
    if(!std::isfinite(slope)||std::fabs(start.d)>1.25)return false;
    const double settle=std::max(10.0,std::min(25.0,ego.v*2.5));
    auto lateral=MakeQuintic(start.d,slope,0,0,0,0,settle);
    double distance=0, speed=std::max(0.0,ego.v);
    double accel=std::clamp(ego.a,-limits.max_longitudinal_accel,limits.max_longitudinal_accel);
    double metric=std::hypot(1-origin.kappa*start.d,slope);
    if(metric<.1)return false;
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
        double target=std::min(cap,LookaheadTargetSpeed(ref,start.s+distance,speed,
            curve,limits.max_longitudinal_accel,limits.max_lateral_accel,limits.max_curvature));
        double requested=std::clamp(2*(target-speed),-limits.max_longitudinal_accel,limits.max_longitudinal_accel);
        double next_accel=std::clamp(requested,accel-limits.max_longitudinal_jerk*dt,accel+limits.max_longitudinal_jerk*dt);
        double next_speed=speed+.5*(accel+next_accel)*dt;
        if(next_speed<-.001)return false;
        next_speed=std::max(0.0,next_speed);
        distance+=.5*(speed+next_speed)*dt/metric;
        speed=next_speed;accel=next_accel;
    }
    return !path.x.empty();
}
}

void FrenetPlanner::ResetRoundaboutMerge() {
    roundabout_merge_committed_=false;
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
    double best_cost=std::numeric_limits<double>::infinity();
    last_candidate_paths_.clear();last_candidate_validity_.clear();
    for(double fraction:{1.0,.8,.6,.4}) {
        ++result.generated;
        CartesianPath path;
        bool valid=MakeMergeProfile(ref_,ego,start,cap*fraction,
            roundabout_merge_cfg_.max_horizon,path_cfg_.dt,curve_speed_cfg_,limits_,path);
        if(valid) valid=avoid_road_map_.EvaluatePath(path,vehicle_shape_,path_cfg_.dt).drivable;
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
        double cost=check.exit_time;
        if(cost<best_cost){best_cost=cost;result.candidate_valid=true;
            result.entry_time=check.entry_time;result.exit_time=check.exit_time;
            result.candidate_path=std::move(path);}
    }
    if(roundabout_merge_committed_) {
        if(result.candidate_valid){out_path=result.candidate_path;last_roundabout_result_=result;return true;}
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
        ctx.s>=roundabout_zone_.stop_s-merge_cfg_.commit_distance;
    // The PREPARE output remains a stop-capable approach/hold trajectory.
    // Its target is the rear axle at the map stopline, independent of the
    // normal traffic-light bumper offset. STOP's small buffer is compensated.
    PlannerCommand waiting=command;waiting.mode=STOP;waiting.has_target_speed=true;
    waiting.target_speed=cap;waiting.stop_position=roundabout_zone_.stop_s+stop_cfg_.stop_buffer;
    bool ok=StopProcess(ego,start,ctx.s,ctx.d,start.s_d,start.s_dd,start.d_d,start.d_dd,
        obstacles,ctx.predicted_obstacles,cap,waiting,out_path);
    if(!ok)result.ready=false;
    last_roundabout_result_=result;
    return ok;
}
