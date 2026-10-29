#include <unordered_set>
#include "Frenet/Maneuver/Merge/HighwayVelocityRecovery.hpp"
#include "Frenet/frenet_planner.hpp"
#include "Frenet/Generator/SpatialStop.hpp"
#include "Frenet/Maneuver/Merge/ExecutionResponse.hpp"
#include "Frenet/Generator/BoundaryConditionPolicy.hpp"
#include "Frenet/Generator/LateralGenerator.hpp"
#include "Frenet/Generator/CartesianRetimer.hpp"
#include "Frenet/Evaluation/Cost.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include "Frenet/Speed/CurveSpeed.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Maneuver/Merge/HighwayPrediction.hpp"
#include "Frenet/Geometry/HighwayLaneMotion.hpp"
#include "Frenet/Maneuver/Merge/HighwayTransitionPath.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <filesystem>

namespace {
bool CartesianCandidate(const RefLine& ref,const FrenetPath& motion,
    double d,double first,double second,double settle,const KinematicLimits& /*limits*/,
    CartesianPath& out,double terminal_d=0) {
    if(motion.s.empty()||settle<.1)return false;
    const SpatialLateralProfile geometry{true,motion.s.front(),settle,
        MakeQuintic(d,first,second,terminal_d,0,0,settle)};
    const auto spatial=SampleSpatialLateralMotion(motion,geometry);
    if(!spatial.valid)return false;
    out=ConvertToCartesianPath(spatial,ref);
    // Safety/kinematics belong to the certified executable span, not the
    // unissued continuation used to discover a complete lane handover.
    return out.x.size()>1;
}
}

bool FrenetPlanner::EnableHighwayMerge(std::string& error) {
    if(highway_route_)return true;
    if(!ref_loaded_||!avoid_road_map_.enabled()||highway_areas_path_.empty()) {
        error="highway requires an initialized official route and validated HD map";return false;
    }
    highway_route_=PrepareHighwayMergeRoute(ref_,ref_config_,highway_merge_cfg_,
        vehicle_shape_,limits_.max_curvature,highway_areas_path_,stop_cfg_.comfortable_decel,error,
        (std::filesystem::path(highway_areas_path_).parent_path()/"lines.json").string());
    return bool(highway_route_);
}
void FrenetPlanner::ResetHighwayMerge() {
    highway_committed_=false;highway_zone_index_=-1;highway_readiness_.Reset();
    highway_lane_entry_times_.clear();
    highway_returning_to_source_=false;
    highway_unresolved_return_object_=-1;highway_unresolved_return_stamp_=-1;
    highway_preferred_target_=-1;
    highway_transition_terminal_s_=0;highway_transition_geometry_={};
    highway_last_observation_=highway_last_plan_=highway_settled_since_=-1;
    ++highway_epoch_;highway_last_result_={};highway_acceleration_memory_.Reset();
}
bool FrenetPlanner::HighwayReadyWasIssued(double now) const {
    return highway_readiness_.WasIssued(now,
        std::min(highway_merge_cfg_.confirmation_gap_s,highway_merge_cfg_.source_max_age_s));
}
bool FrenetPlanner::CanCommitHighwayMerge(const HighwayMergeResult& r) const {
    return !highway_committed_&&r.epoch==highway_epoch_&&
        r.epoch==highway_last_result_.epoch&&r.ready&&r.candidate_valid&&
        highway_last_result_.ready&&!r.candidate_path.x.empty();
}
void FrenetPlanner::ApplyHighwayMergeCommit(const HighwayMergeResult& r) {
    if(CanCommitHighwayMerge(r)) {
        highway_committed_=true;highway_transition_terminal_s_=r.transition_terminal_s;
        highway_transition_geometry_=r.transition_geometry;
    }
}

bool FrenetPlanner::PlanHighwayMerge(const CartesianState& ego,
    const std::vector<TrackedObject>& objects,const PlannerCommand& command,
    double now,double observation,CartesianPath& output,HighwayMergeResult& result,double ego_stamp,
    const RefLine* transition_target,const RefLine* source_reference,bool execute_requested) {
    return PlanHighwayTransitionImpl(ego,objects,command,now,observation,output,result,
        ego_stamp,transition_target,source_reference,highway_route_.get(),nullptr,false,execute_requested);
}

bool FrenetPlanner::PlanHighwayTransition(const HighwayMergeRoute& route,
    const HighwayMergeRoute::Zone& zone,bool optional,const CartesianState& ego,
    const std::vector<TrackedObject>& objects,const PlannerCommand& command,
    double now,double observation,CartesianPath& output,HighwayMergeResult& result,
    double ego_stamp,const RefLine& target,bool execute_requested) {
    return PlanHighwayTransitionImpl(ego,objects,command,now,observation,output,result,
        ego_stamp,&target,&ref_,&route,&zone,optional,execute_requested);
}

bool FrenetPlanner::PlanHighwayLaneContinuation(const CartesianState& ego,
    const std::vector<TrackedObject>& objects,double speed,const std::string& source,CartesianPath& output) {
    const double ego_s=FindClosestS(ref_,ego.x,ego.y,nullptr,100);
    PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;
    keep.target_speed=speed;keep.source_link_id=source;
    double nearest=INFINITY;
    for(const auto& object:objects) {
        if(object.type!=int(asmc_planner::ObjectType::VEHICLE)&&
           !(object.type==int(asmc_planner::ObjectType::UNKNOWN)&&
             (object.motion_state==ObjectMotionState::DYNAMIC||
              object.motion_state==ObjectMotionState::DYNAMIC_PENDING)))continue;
        double lo=INFINITY,hi=-INFINITY,front=INFINITY;
        for(const auto& corner:object.corners) {
            const double station=FindClosestS(ref_,corner.x,corner.y,nullptr,100);
            const auto p=Interpolate(ref_,station);
            const double lateral=-(corner.x-p.x)*std::sin(p.theta)+(corner.y-p.y)*std::cos(p.theta);
            lo=std::min(lo,lateral);hi=std::max(hi,lateral);front=std::min(front,station);
        }
        const double gap=front-ego_s-(vehicle_shape_.length-vehicle_shape_.rear_overhang);
        if(lo<=vehicle_shape_.width*.5&&hi>=-vehicle_shape_.width*.5&&gap>0&&
           gap<=following_cfg_.max_leader_search_s&&gap<nearest) {
            nearest=gap;keep.mode=FOLLOWING;keep.target_object_id=object.id;
        }
    }
    return Plan(ego,objects,keep,output);

}

bool FrenetPlanner::PlanHighwayTransitionImpl(const CartesianState& ego,
    const std::vector<TrackedObject>& objects,const PlannerCommand& command,
    double now,double observation,CartesianPath& output,HighwayMergeResult& result,double ego_stamp,
    const RefLine* transition_target,const RefLine* source_reference,
    const HighwayMergeRoute* route,const HighwayMergeRoute::Zone* requested_zone,bool optional,bool execute_requested) {
    const auto operation_start=std::chrono::steady_clock::now();
    result={};output={};result.epoch=++highway_epoch_;
    last_failure_reason_=PlanFailureReason::INVALID_COMMAND;
    const bool lane_change=command.mode==LANE_CHANGE_LEFT||command.mode==LANE_CHANGE_RIGHT;
    if(!route||(!optional&&(command.mode!=MERGE||command.merge_type!=MergeType::HIGHWAY))||
       (optional&&!lane_change)||
       command.merge_sa_id!=-1||command.merge_sb_id!=-1||
       !std::isfinite(now)||!std::isfinite(observation)||observation<=0||
       !std::isfinite(ego.v)||ego.v<0||!std::isfinite(ego.a)) {
        RevokeHighwayMergeReadiness();highway_preferred_target_=-1;
        result.detail="invalid_or_unavailable_highway_request";return false;
    }
    const auto& cfg=route->config;
    auto highway_limits=limits_;
    highway_limits.max_forward_accel=std::min(limits_.max_forward_accel,cfg.forward_accel_mps2);
    auto curve_command=command;curve_command.mode=MERGE;curve_command.merge_type=MergeType::HIGHWAY;
    const auto curve_policy=ResolveHighwayCurveSpeed(curve_speed_cfg_,cfg,true,curve_command);
    if(ego_stamp<0)ego_stamp=now;
    if(now<highway_last_plan_||observation<highway_last_observation_||
       now-observation<0||now-observation>cfg.source_max_age_s||
       !std::isfinite(ego_stamp)||now-ego_stamp<0||now-ego_stamp>cfg.source_max_age_s) {
        RevokeHighwayMergeReadiness();highway_preferred_target_=-1;highway_acceleration_memory_.Reset();
        result.detail="stale_or_reversed_source_time";
        last_failure_reason_=PlanFailureReason::TRANSIENT_GENERATION_FAILURE;return false;
    }
    SetObjectObservationTime(observation);
    auto ctx=PreparePlanningContext(ego,objects,command);
    const auto approach=HighwayApproachAt(*route,ego,&ctx.s);
    if(!requested_zone&&highway_zone_index_<0&&approach.valid)highway_zone_index_=static_cast<int>(approach.zone_index);
    if(!requested_zone&&(highway_zone_index_<0||static_cast<std::size_t>(highway_zone_index_)>=route->zones.size())) {
        RevokeHighwayMergeReadiness();highway_preferred_target_=-1;
        result.detail="ego_outside_requested_highway_zone";return false;
    }
    const auto& zone=requested_zone?*requested_zone:route->zones[highway_zone_index_];
    if(command.source_link_id!=zone.config.source_link||
       command.target_link_id!=(optional?zone.config.target_lanes.front().substr(5):
           route->map->lane(zone.config.target_lanes.back())->link_id)) {
        RevokeHighwayMergeReadiness();highway_preferred_target_=-1;
        result.detail="highway_source_or_target_mismatch";return false;
    }
    auto execution_zone=zone;
    for(auto& lane:execution_zone.following_lanes) {
        const auto prepared=std::find_if(highway_traffic_lanes_.begin(),highway_traffic_lanes_.end(),
            [&](const auto& p){return p&&lane.reference.get()==&p->line();});
        if(prepared==highway_traffic_lanes_.end())continue;
        lane.response_lane_end_s=(*prepared)->line().length;
        for(const auto& id:(*prepared)->lanes()) {
            const auto* road=(*prepared)->map()->lane(id);
            if(road&&road->successors.size()>1) {
                const auto& end=road->centerline.back();
                lane.response_lane_end_s=FindClosestS((*prepared)->line(),end.x,end.y,nullptr,100);
                break;
            }
        }
    }
    if(highway_committed_)for(auto& lane:execution_zone.following_lanes) {
        if(!lane.reference||!lane.region)continue;
        const auto body=MakeEgoFootprint(ego,vehicle_shape_,0);
        if(lane.region->intersectsPolygon(body))
            highway_lane_entry_times_.emplace(lane.reference.get(),now);
        const auto entered=highway_lane_entry_times_.find(lane.reference.get());
        if(entered!=highway_lane_entry_times_.end())
            lane.reaction_remaining_s=std::max(0.0,highway_rear_response::reaction_s-
                std::max(0.0,now-entered->second));
    }
    auto candidate_obstacles=ctx.predicted_obstacles;
    std::unordered_set<int> missing_velocity_ids;
    for(auto& object:candidate_obstacles) {
        if(object.velocity_unavailable)missing_velocity_ids.insert(object.id);
        const auto track=object_tracks_.find(object.id);
        const auto heading=route->map->laneHeading(object.x,object.y,object.vx,object.vy);
        object=RecoverHighwayVelocity(object,track==object_tracks_.end()?nullptr:&track->second,cfg);
        object=PrepareHighwayObjectPrediction(object,
            track==object_tracks_.end()?nullptr:&track->second,cfg,nullptr,heading.value_or(NAN),
            object_tracking_cfg_.min_motion_direction_change_rad);
        object=PrepareHighwayLaneMotion(object,track==object_tracks_.end()?nullptr:&track->second,
            highway_traffic_lanes_,cfg.motion_fit_min_span_s,cfg.min_heading_alignment);
        object=PrepareHighwayUnresolvedMotion(object,track==object_tracks_.end()?nullptr:&track->second,
            cfg,heading.value_or(NAN),ego,vehicle_shape_);
    }
    // Source-corridor maintenance uses the ordinary collision policy, not
    // the additional full-merge speed interval. Preserve the fitted motion.
    ctx.predicted_obstacles=candidate_obstacles;
    for(auto& object:ctx.predicted_obstacles) {
        if(!missing_velocity_ids.count(object.id))
            object.prediction_min_speed=object.prediction_max_speed=NAN;
        object=AdvanceObjectObservation(object,std::max(0.0,ego_stamp-observation));
    }
    if(highway_last_observation_>=0&&observation-highway_last_observation_>cfg.confirmation_gap_s) {
        highway_readiness_.Reset();highway_settled_since_=-1;
    }
    const double evidence_gap=std::min(cfg.confirmation_gap_s,cfg.source_max_age_s);
    highway_readiness_.Expire(now,evidence_gap);
    const bool execute_authorized=execute_requested&&highway_readiness_.WasIssued(now,evidence_gap);
    highway_last_observation_=observation;highway_last_plan_=now;
    const auto deadline=operation_start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double,std::milli>(cfg.planning_budget_ms));
    const double cap=std::min(cfg.speed_cap_mps,ctx.resolved_target_speed);
    if(!std::isfinite(cap)||cap<=0){RevokeHighwayMergeReadiness();result.detail="invalid_highway_speed_cap";return false;}
    // All request/identity checks have passed. A missing trajectory below is
    // a recoverable planning failure, never an invalid command. Leaving the
    // entry default here terminally latched EXECUTING and restarted PREPARE.
    last_failure_reason_=PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
    FrenetState start{};start.s=ctx.s;start.s_d=std::max(0.0,ctx.s_dot);
    start.s_dd=std::clamp(highway_acceleration_memory_.Seed(ctx.s_ddot,now),
        -limits_.max_longitudinal_accel,limits_.max_longitudinal_accel);
    if(start.s_dd<0)start.s_dd=std::max(start.s_dd,
        -std::sqrt(2*limits_.max_longitudinal_jerk*start.s_d));
    if(ego.v<.1&&start.s_dd<0)start.s_dd=0;
    const auto origin=Interpolate(ref_,ctx.s);
    const double slope=(1-origin.kappa*ctx.d)*std::tan(std::remainder(ego.yaw-origin.theta,2*M_PI));
    const double settle=std::max(8.0,ego.v*3.0);
    const auto& source=source_reference?*source_reference:zone.source_reference;
    double source_s,source_v,source_a,source_d,source_first,source_second;
    CartesianToFrenet(source,ego,source_s,source_v,source_a,source_d,source_first,source_second,nullptr,100);
    const auto source_stop=Interpolate(zone.source_reference,zone.source_stop_local_s);
    const double source_stop_s=FindClosestS(source,source_stop.x,source_stop.y,nullptr,100);
    const auto source_clear=[&](const CartesianPath& path) {
        if(path.x.size()<2)return false;
        double hint=FindClosestS(zone.source_reference,path.x.front(),path.y.front(),nullptr,100);
        for(std::size_t i=0;i<path.x.size();++i) {
            const CartesianState pose{path.x[i],path.y[i],path.yaw[i],0,0,0};
            if(!zone.source_region->coversPoint(pose.x,pose.y))return false;
            // Source/target polygons intentionally overlap in a taper. Their
            // overlap is not a new lane-change command. Stay on the source
            // reference and stop before its validated full-footprint end.
            // As with LaneReference, road containment uses the runtime raster;
            // exact joined polygons have known microscopic seam gaps.
            hint=FindClosestS(zone.source_reference,pose.x,pose.y,&hint,15);
            if(hint>zone.source_stop_local_s+1e-3)return false;
        }
        return true;
    };
    const auto source_aligned=[&](const CartesianState& pose) {
        const auto p=Interpolate(source,FindClosestS(source,pose.x,pose.y,nullptr,100));
        const double d=-(pose.x-p.x)*std::sin(p.theta)+(pose.y-p.y)*std::cos(p.theta);
        const double heading=std::remainder(pose.yaw-p.theta,2*M_PI);
        return zone.source_region->coversPolygon(MakeEgoFootprint(pose,vehicle_shape_,0))&&
            std::abs(d)<=cfg.completion_lateral_m&&std::abs(heading)<=cfg.completion_heading_rad&&
            std::abs(pose.v*std::sin(heading))<=lane_change_cfg_.lateral_speed_tolerance;
    };
    auto grid=path_cfg_;grid.time_horizon={cfg.max_horizon_s,cfg.max_horizon_s,1};
    grid.delta_s_dot={0,0,1};
    // A bounded speed grid changes the actual longitudinal schedule. No gap
    // helper invents a second ego trajectory, and no candidate runs in KEEP.
    CartesianPath maintenance_path;
    bool maintenance_ok=false;
    auto recovery_deadline=std::chrono::steady_clock::time_point::min();
    const auto recovery_budget=[&]() {
        if(recovery_deadline==std::chrono::steady_clock::time_point::min())
            recovery_deadline=std::chrono::steady_clock::now()+
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double,std::milli>(cfg.planning_budget_ms));
        return recovery_deadline;
    };
    const auto maintenance_safety=[&](const CartesianPath& path) {
        if(!highway_committed_)return CartesianTrajectoryClear(path,ctx.predicted_obstacles,
            vehicle_shape_,collision_cfg_,path_cfg_.dt);
        // Keep the ordinary maintenance certificate for every non-blocking
        // actor. Only a blocker is reconsidered under the SAME execution rear
        // policy as moving schedules; do this before the generator rejects it.
        auto remaining=ctx.predicted_obstacles;
        for(std::size_t attempt=0;attempt<=ctx.predicted_obstacles.size();++attempt) {
            const auto ordinary=EvaluateCartesianTrajectorySafety(path,remaining,
                vehicle_shape_,collision_cfg_,path_cfg_.dt);
            if(ordinary.clear)return true;
            const auto actor=std::find_if(candidate_obstacles.begin(),candidate_obstacles.end(),
                [&](const auto& o){return o.id==ordinary.blocking_object_id;});
            if(actor==candidate_obstacles.end())return false;
            const auto check=EvaluateHighwayMergeCandidate(path,execution_zone,ref_,{*actor},
                cfg,vehicle_shape_,collision_cfg_,path_cfg_.dt,false,ego_stamp-observation,
                recovery_budget(),transition_target!=nullptr);
            if(!check.safe)return false; // mitigation never certifies a STOP
            const auto before=remaining.size();
            remaining.erase(std::remove_if(remaining.begin(),remaining.end(),
                [&](const auto& o){return o.id==actor->id;}),remaining.end());
            if(remaining.size()+1!=before)return false;
        }
        return false;
    };
    const auto build_maintenance=[&]() {
        // Maintenance must remain available when the transition certificate
        // exhausts its budget. Bound its own source search as before; never
        // publish a cached source path or an uncertified transition instead.
        const auto deadline=recovery_budget();
        if(optional&&!highway_committed_) {
            // Refusing an optional change never creates a ramp stop. Use the
            // existing source-lane planner, including its normal leader policy.
            maintenance_ok=PlanHighwayLaneContinuation(ego,objects,cap,command.source_link_id,maintenance_path);
            return;
        }
        if(!highway_committed_&&source_s<source_stop_s-1e-4) {
            // A rolling source-lane approach retains stopping ability at the
            // last mapped source-only frontier. It does not aim at an
            // intermediate wait point; the next cycle searches from the new
            // actual position. Reuse ordinary STOP's spatial profile, with a
            // single bounded grid. Calling the entire STOP candidate search here
            // would duplicate its many lateral/time alternatives on every MERGE tick.
            auto stopping_limits=limits_;
            stopping_limits.max_forward_accel=highway_limits.max_forward_accel;
            stopping_limits.max_longitudinal_accel=std::max(limits_.max_longitudinal_accel,
                stop_cfg_.max_service_decel);
            stopping_limits.boundary_condition_accel_fraction=limits_.boundary_condition_accel_fraction;
            FrenetState boundary{};
            boundary.s=source_s;boundary.s_d=std::max(0.0,source_v);
            // Use the same physical boundary headroom as ordinary planning.
            // The shared spatial STOP generator itself is unchanged.
            boundary.s_dd=std::clamp(source_a,
                -stopping_limits.max_longitudinal_accel*limits_.boundary_condition_accel_fraction,
                limits_.max_longitudinal_accel*limits_.boundary_condition_accel_fraction);
            if(boundary.s_dd<0)boundary.s_dd=std::max(boundary.s_dd,
                -std::sqrt(2*limits_.max_longitudinal_jerk*
                    stopping_limits.boundary_condition_accel_fraction*boundary.s_d));
            boundary.d=source_d;
            auto stopping_grid=path_cfg_;
            stopping_grid.delta_s_dot={0,0,0};
            const double horizon_step=std::max(path_cfg_.dt,path_cfg_.time_horizon.step);
            stopping_grid.time_horizon.step=horizon_step*std::max(1.0,
                std::floor((path_cfg_.time_horizon.max-path_cfg_.time_horizon.min)/
                    (2*horizon_step)));
            const double remaining=source_stop_s-source_s;
            stopping_grid.delta_s={-remaining,0,remaining/3.0};
            // Progress toward the admissible road/traffic cap is preferred.
            // Braking feasibility, not an arbitrary ramp cruise, limits speed.
            const double preferred=cap;
            std::vector<FrenetPath> candidates;
            // The witness cannot assume stronger braking than the existing
            // collision-braking fallback. Keep the STOP generator's jerk
            // headroom as well, so a normal stop remains reachable next cycle.
            const StoppingContinuationPolicy stopping{limits_.max_longitudinal_accel,
                limits_.max_longitudinal_jerk*limits_.boundary_condition_accel_fraction,
                limits_.max_curvature,limits_.max_lateral_accel};
            // Intent becomes an ordinary trajectory request. Generation,
            // kinematic/collision filtering, cost and selection are shared;
            // only source-road containment and its endpoint are maneuver facts.
            std::vector<double> speeds;
            // Removing a fixed low-speed preference must not remove braking
            // alternatives. Keep a bounded grid down to crawl; the common
            // progress cost still favors the fastest safe admissible motion.
            for(double speed:{cap,std::max(cfg.min_travel_speed,cap*.5),
                              std::max(cfg.min_travel_speed,cap*.25),cfg.min_travel_speed}) {
                if(std::none_of(speeds.begin(),speeds.end(),[&](double v){return std::fabs(v-speed)<1e-3;}))
                    speeds.push_back(speed);
            }
            for(double speed:speeds) {
                for(double terminal_d:{0.0,source_d}) {
                    if(std::chrono::steady_clock::now()>=deadline)break;
                    if(terminal_d!=0&&std::fabs(source_d)<1e-3)continue;
                    TrajectoryRequest request;
                    request.lateral.target_d=terminal_d;
                    // A nearby longitudinal endpoint must not squeeze the
                    // steering correction into the last centimetres. The body
                    // and source-frontier certificate still covers the stop.
                    request.lateral.settle_distance=settle;
                    request.lateral.initial_slope=source_first;
                    request.lateral.initial_second=source_second;
                    request.longitudinal_mode=LongitudinalMode::STOPPING;
                    request.target_speed=speed;
                    request.stop.spatial_profile=true;
                    request.stop.stop_s=source_stop_s;
                    request.stop.cruise_speed=speed;
                    request.stop.comfortable_decel=stop_cfg_.comfortable_decel;
                    // Physical Cartesian filtering still uses the actual
                    // forward limit. The generator's separate achievable
                    // response handles longitudinal timing and jerk release.
                    request.stop.acceleration_limit=limits_.max_longitudinal_accel;
                    request.stop.allow_partial_horizons=true;
                    request.stop.terminal_offset=stopping_grid.delta_s;
                    request.collision.stopping=stopping;
                    // A stop profile neutralizes its boundary acceleration
                    // before changing cruise speed. Repeatedly using only that
                    // profile for a distant rolling approach makes acceleration
                    // alternate with release. Offer ordinary velocity/envelope
                    // motion as well; the SAME stopping witness below certifies
                    // that its prefix can still stop before the source frontier.
                    auto rolling=request;
                    rolling.longitudinal_mode=LongitudinalMode::VELOCITY_KEEPING;
                    rolling.speed_envelope=BuildKeepSpeedEnvelope(source,source_s,boundary.s_d,
                        boundary.s_dd,stopping_grid.time_horizon.max,path_cfg_.dt,speed,
                        curve_policy,limits_.max_longitudinal_accel,limits_.max_lateral_accel,
                        limits_.max_longitudinal_jerk);
                    if(!rolling.speed_envelope.empty()) {
                        auto generated=GenerateRequestedCandidates(boundary,rolling,source,stopping_grid,
                            highway_limits,{},vehicle_shape_,collision_cfg_);
                        candidates.insert(candidates.end(),std::make_move_iterator(generated.begin()),
                            std::make_move_iterator(generated.end()));
                    }
                    auto generated=GenerateRequestedCandidates(boundary,request,source,stopping_grid,
                        stopping_limits,{},vehicle_shape_,collision_cfg_);
                    candidates.insert(candidates.end(),std::make_move_iterator(generated.begin()),
                        std::make_move_iterator(generated.end()));
                }
            }
            EvaluateApproachCosts(candidates,preferred,limits_.max_longitudinal_accel,
                limits_.max_longitudinal_jerk,0.0);
            // Road checks cover both the executable prefix and its full safe
            // braking witness. A shorter prefix cannot hide the final frontier.
            while(const auto* best=SelectBestPath(candidates)) {
                FrenetPath witness;
                std::vector<FrenetPath> checked{*best};
                FilterByCollision(checked,source,ctx.predicted_obstacles,vehicle_shape_,collision_cfg_,-1,-1,lane_width_*.5,&stopping);
                if(checked.front().valid&&BuildStoppingSafetyPath(*best,source,stopping,collision_cfg_.reactive_lookahead,witness)) {
                    const auto safety=ConvertToCartesianPath(witness,source);
                    if(source_clear(safety)&&avoid_road_map_.EvaluatePath(safety,vehicle_shape_,path_cfg_.dt).drivable) {
                        maintenance_path=ConvertToCartesianPath(*best,source);maintenance_ok=true;break;
                    }
                }
                candidates[best-candidates.data()].valid=false;
                if(std::chrono::steady_clock::now()>=deadline)break;
            }
        }
        if(!maintenance_ok&&ego.v<=.1) {
            maintenance_path={};
            const int count=std::max(2,static_cast<int>(std::ceil(collision_cfg_.reactive_lookahead/path_cfg_.dt)));
            for(int i=0;i<=count;++i) {
                maintenance_path.x.push_back(ego.x);maintenance_path.y.push_back(ego.y);
                maintenance_path.yaw.push_back(ego.yaw);maintenance_path.kappa.push_back(ego.kappa);
                maintenance_path.v.push_back(0);maintenance_path.a.push_back(0);
            }
            maintenance_ok=(highway_committed_||source_clear(maintenance_path))&&
                avoid_road_map_.EvaluatePath(maintenance_path,vehicle_shape_,path_cfg_.dt).drivable&&
                maintenance_safety(maintenance_path);
        }
        if(!maintenance_ok) {
            maintenance_ok=PlanCollisionBrakingPathWithSafety(ego,ctx.s,ctx.d,
                ctx.predicted_obstacles,maintenance_path,0.0,0.0,0.0,false,
                highway_committed_?std::function<bool(const CartesianPath&)>(maintenance_safety):
                    std::function<bool(const CartesianPath&)>{});
            if(!maintenance_ok&&!highway_committed_)
                maintenance_ok=PlanCollisionBrakingPath(ego,ctx.s,ctx.d,
                    ctx.predicted_obstacles,maintenance_path,0.0,ctx.d,0.0,true);
            if(maintenance_ok&&!highway_committed_)maintenance_ok=source_clear(maintenance_path);
        }
    };
    // Inspect the actual transition before source maintenance can consume its
    // deadline. EXECUTE can publish a newly certified path in this same tick.
    result.detail="no_kinematically_valid_candidate";
    double best=INFINITY;
    HighwayMergeResult mitigation;
    double mitigation_cost=INFINITY,mitigation_progress=-1;
    // Inspect faster admissible schedules before a slow current-speed hold.
    // Every schedule still receives the complete safety certificate.
    bool rejected_candidate=false;
    std::vector<double> targets;
    for(double target:{cap,std::min(cap,std::max(cfg.min_travel_speed,ego.v)),cap*.8,cap*.6,cap*.4}) {
        if(std::none_of(targets.begin(),targets.end(),[&](double x){return std::fabs(x-target)<1e-3;}))
            targets.push_back(target);
    }
    std::sort(targets.begin(),targets.end(),std::greater<double>());
    if(!highway_committed_&&highway_preferred_target_>0) {
        // Cache only the search intent, never timed samples or approval. The
        // connector, timing, OBB/gaps, kinematics and road certificate below
        // are rebuilt from this tick's actual inputs.
        const double preferred=std::min(cap,highway_preferred_target_);
        targets.erase(std::remove_if(targets.begin(),targets.end(),[&](double v){
            return std::fabs(v-preferred)<1e-3;}),targets.end());
        targets.insert(targets.begin(),preferred);
    }
    // Complete the accepted change before considering an abort. A selected,
    // still-safe abort retains ownership instead of following a flickering gap.
    // A safe return remains authoritative while velocity is unresolved. A
    // NEW valid velocity can reopen target search once, with full fresh checks.
    // It grants neither clearance nor reuse of an earlier approval.
    bool reconsider_target=false;
    if(optional&&highway_committed_&&highway_unresolved_return_object_>=0) {
        const auto fresh=std::find_if(objects.begin(),objects.end(),[&](const auto& o){
            return o.id==highway_unresolved_return_object_;});
        if(observation>highway_unresolved_return_stamp_&&fresh!=objects.end()&&
           !fresh->velocity_unavailable) {
            reconsider_target=true;highway_unresolved_return_object_=-1;
        } else if(observation-highway_unresolved_return_stamp_>cfg.source_max_age_s) {
            highway_unresolved_return_object_=-1;
        }
    }
    const bool was_returning=highway_returning_to_source_&&!reconsider_target;
    int unresolved_forward_blocker=-1;bool definite_forward_block=false;
    const bool abort_allowed=optional&&highway_committed_&&transition_target&&
        (was_returning||!zone.target_region->coversPolygon(MakeEgoFootprint(ego,vehicle_shape_,0)));
    const bool approaching_transition=transition_target&&!highway_committed_&&ctx.s<zone.config.start_s;
    if(approaching_transition)result.detail="approaching_reference_transition";
    bool primary_blocked=false;auto search_deadline=deadline;
    for(int family=0;family<(abort_allowed?2:1)&&!result.candidate_valid&&!approaching_transition;++family) {
      if(family>0) {
          if(mitigation.forward_mitigation&&!primary_blocked)break;
          if(result.detail=="planning_time_budget"&&!primary_blocked&&!reconsider_target)break;
          // Share the existing maintenance budget with recovery. A known
          // collision must not starve a safe return just because inspecting
          // other forward alternatives exhausted the first search budget.
          // Timeout without physical rejection never authorizes this switch.
          search_deadline=recovery_budget();
      }
      const bool returning=abort_allowed&&(family==0?was_returning:!was_returning);
      const auto* destination=returning?&source:transition_target;
      auto check_zone=execution_zone;
      if(returning) {
          check_zone.target_reference=source;check_zone.target_region=zone.source_region;
          check_zone.config.completion_s=source_stop_s;
      }
      double transition_distance=0;
      if(destination) {
          const auto p=Interpolate(*destination,FindClosestS(*destination,ego.x,ego.y,nullptr,100));
          transition_distance=std::hypot(p.x-ego.x,p.y-ego.y);
      }
      double tracking_guard=0;
      const bool same_destination=(!reconsider_target&&returning==was_returning)||
          (reconsider_target&&returning&&highway_returning_to_source_);
      const bool retained=destination&&highway_committed_&&same_destination&&
          std::abs(ego.kappa)<=limits_.max_curvature&&
          ego.v*ego.v*std::abs(ego.kappa)<=limits_.max_lateral_accel&&
          HighwayTransitionTrackingGuard(ref_,*destination,highway_transition_geometry_,
              ctx.s,ego,vehicle_shape_,cfg.transition_tracking_position_m,
              cfg.transition_tracking_heading_rad,cfg.transition_tracking_curvature_inv_m,tracking_guard);
      const int stages=destination&&highway_committed_?4:1;
      for(int stage=0;stage<stages&&!result.candidate_valid&&!approaching_transition;++stage) {
        if(stages>1&&stage==0&&!retained)continue;
        for(double target:targets) {
          if(std::chrono::steady_clock::now()>=search_deadline) {result.detail="planning_time_budget";break;}
          const auto envelope=BuildKeepSpeedEnvelope(ref_,ctx.s,ego.v,start.s_dd,
              cfg.max_horizon_s,path_cfg_.dt,target,curve_policy,limits_.max_longitudinal_accel,
              limits_.max_lateral_accel,limits_.max_longitudinal_jerk);
          auto motions=GenerateEnvelopeVelocityCandidates(start,target,envelope,grid,highway_limits);
          if(motions.empty()&&!highway_committed_)rejected_candidate=true;
          for(auto& motion:motions) {
              CartesianPath candidate;HighwayTransitionGeometry geometry;
              std::vector<CartesianFollowingCandidate> schedules;
              CartesianPath physical_geometry;double physical_acceleration=0;
              const bool reuse=stages>1&&stage==0;
              const bool recover=stages>1&&stage>=2;
              // Recovery lengths respect CURRENT as well as intended speed. A
              // slower target cannot shorten a connector before the car brakes.
              const double design_speed=recover?std::max(target,ego.v):target;
              const double connector=destination?std::max({settle,
                  std::sqrt(6*transition_distance/limits_.max_curvature),
                  std::sqrt(6*transition_distance*design_speed*design_speed/limits_.max_lateral_accel)}):settle;
              double terminal=ctx.s+connector;
              if(optional&&!highway_committed_&&terminal>check_zone.config.completion_s) {
                  rejected_candidate=true;
                  result.detail="optional_change_past_latest_completion";continue;
              }
              if(stages>1) {
                  if(reuse)terminal=highway_transition_geometry_.terminal_s;
                  else if(stage==1)terminal=highway_transition_terminal_s_;
                  else terminal=ctx.s+connector*(stage==2?1.0:1.5);
                  // The map's mandatory merge end is a hard recovery horizon.
                  // Never shift the target beyond the validated maneuver zone.
                  if(recover) {
                      terminal=std::min(std::max(terminal,highway_transition_terminal_s_),check_zone.config.completion_s);
                      if(terminal<=highway_transition_terminal_s_+.5)continue;
                  }
                  if(!reuse&&terminal-ctx.s<.5)continue;
              }
              if(destination) {
                  if(ctx.s<zone.config.start_s) {result.detail="approaching_reference_transition";continue;}
                  geometry=reuse?highway_transition_geometry_:MakeHighwayTransitionGeometry(
                      ref_,*destination,ctx.s,ctx.d,slope,ctx.d_pprime,terminal);
                  if(!SampleHighwayTransitionPath(ref_,*destination,motion,geometry,candidate)) {
                      if(!highway_committed_)rejected_candidate=true;
                      continue;
                  }
                  // A connector changes reference geometry. Resolve its speed
                  // envelope in actual metres on that curve, not mission s.
                  physical_geometry=candidate;FrenetPath physical_motion;
                  physical_acceleration=std::clamp(highway_acceleration_memory_.Seed(ego.a,now),
                      -limits_.max_longitudinal_accel,limits_.max_longitudinal_accel);
                  if(physical_acceleration<0)physical_acceleration=std::max(physical_acceleration,
                      -std::sqrt(2*limits_.max_longitudinal_jerk*ego.v));
                  if(ego.v<.1&&physical_acceleration<0)physical_acceleration=0;
                  if(!RetimeCartesianGeometry(physical_geometry,ego.v,physical_acceleration,target,
                      grid,highway_limits,curve_policy,candidate,physical_motion)) {
                      if(!highway_committed_)rejected_candidate=true;
                      continue;
                  }
                  schedules.push_back({std::move(candidate),std::move(physical_motion),-1});
              } else if(!CartesianCandidate(ref_,motion,ctx.d,slope,ctx.d_pprime,settle,limits_,candidate)) {
                  if(!highway_committed_)rejected_candidate=true;
                  continue;
              }
              if(!destination)schedules.push_back({std::move(candidate),motion,-1});
              bool following_loaded=false;
              for(std::size_t schedule_index=0;;++schedule_index) {
                  if(schedule_index==schedules.size()) {
                      // A certified free-flow path needs no extra FOLLOW search.
                      // Only a rejected schedule opens the bounded alternatives, for
                      // mandatory merges as well as optional lane changes.
                      if(!result.candidate_valid&&!following_loaded&&highway_committed_&&
                         std::abs(target-targets.front())<1e-3&&destination&&
                         std::chrono::steady_clock::now()<search_deadline) {
                          following_loaded=true;
                          auto following_grid=grid;
                          following_grid.time_horizon.min=std::min(path_cfg_.time_horizon.max,cfg.max_horizon_s);
                          following_grid.time_horizon.step=std::max(path_cfg_.dt,
                              cfg.max_horizon_s-following_grid.time_horizon.min);
                          auto following=RetimeCartesianFollowingGeometry(physical_geometry,ego.v,
                              physical_acceleration,target,candidate_obstacles,vehicle_shape_,following_cfg_,
                              following_grid,highway_limits,curve_policy,ego_stamp-observation);
                          for(auto& alternative:following)schedules.push_back(std::move(alternative));
                      }
                      if(schedule_index==schedules.size())break;
                  }
                  auto& schedule=schedules[schedule_index];
                  candidate=std::move(schedule.path);
                  auto& physical_motion=schedule.motion;
                  ++result.generated;
                  auto body=vehicle_shape_;
                  if(reuse) {
                      body.length+=2*tracking_guard;body.width+=2*tracking_guard;
                      body.rear_overhang=(vehicle_shape_.rear_overhang<0?vehicle_shape_.length*.5:
                          vehicle_shape_.rear_overhang)+tracking_guard;
                  }
                  const auto check=EvaluateHighwayMergeCandidate(candidate,check_zone,ref_,candidate_obstacles,
                      cfg,body,collision_cfg_,path_cfg_.dt,!highway_committed_,ego_stamp-observation,search_deadline,
                      destination!=nullptr);
                  const bool rear_mitigation=highway_committed_&&check.rear_response_unverified&&
                      check.certified_path_samples>1;
                  if(!check.safe&&!rear_mitigation) {
                      if(family==0&&check.reason=="candidate_collision") {
                          primary_blocked=true;
                          const auto blocked=std::find_if(objects.begin(),objects.end(),[&](const auto& o){
                              return o.id==check.blocking_object_id;});
                          if(optional&&!was_returning&&blocked!=objects.end()&&blocked->velocity_unavailable)
                              unresolved_forward_blocker=blocked->id;
                          else definite_forward_block=true;
                      }
                      if(!highway_committed_&&check.reason!="planning_time_budget")
                          rejected_candidate=true;
                      if(!result.candidate_valid){result.check=check;result.detail=check.reason;}continue;
                  }
                  if(returning&&(!check.covers_completion||
                      FindClosestS(source,candidate.x[check.certified_path_samples-1],
                          candidate.y[check.certified_path_samples-1],nullptr,100)>source_stop_s))continue;
                  if(optional&&!highway_committed_&&zone.admission_rear_observation_m>0) {
                      const auto first=FindClosestS(zone.target_reference,ego.x,ego.y,nullptr,100);
                      const auto index=std::min(candidate.x.size()-1,
                          std::size_t(std::ceil(check.clear_time_s/path_cfg_.dt)));
                      const auto last=FindClosestS(zone.target_reference,candidate.x[index],candidate.y[index],nullptr,100);
                      const double rear_gap=zone.admission_rear_observation_m+last-first-
                          zone.admission_unseen_speed_mps*(check.clear_time_s+std::max(0.0,now-observation))-
                          vehicle_shape_.rear_overhang;
                      // Filter THIS schedule, then inspect other generated schedules.
                      // A post-selection veto would wrongly discard safe alternatives.
                      if(check.clear_time_s<0||!std::isfinite(rear_gap)||rear_gap<cfg.min_rear_gap) {
                          rejected_candidate=true;
                          result.detail="rear_observation_window_too_short_for_candidate_arrival";continue;
                      }
                  }
                  // The merge checker owns the complete handover plus its full
                  // post-merge tail. Check exactly that SAME issued span; distant
                  // unissued geometry cannot invalidate an otherwise usable handover.
                  if(!HighwayTransitionKinematicsValid(candidate,physical_motion,limits_,check.certified_path_samples)) {
                      if(!highway_committed_)rejected_candidate=true;
                      continue;
                  }
                  for(auto* values:{&candidate.x,&candidate.y,&candidate.yaw,&candidate.kappa,&candidate.v,&candidate.a})
                      values->resize(check.certified_path_samples);
                  const auto road=avoid_road_map_.EvaluatePath(candidate,body,path_cfg_.dt);
                  if(!road.drivable||(destination&&road.paint_contact_seconds>0)){
                      if(!highway_committed_)rejected_candidate=true;
                      if(family==0)primary_blocked=true;
                    result.detail=road.drivable?"transition_crosses_protected_paint":"candidate_outside_drivable_map";continue;
                  }
                  if(std::chrono::steady_clock::now()>=search_deadline) {result.detail="planning_time_budget";break;}
                  const double cost=(check.covers_completion?check.clear_time_s:
                      2*cfg.max_horizon_s+(check.entry_time_s<0?cfg.max_horizon_s:check.entry_time_s))+
                      .1*(cap-target);
                  if(rear_mitigation) {
                      // Reuse this fully front/side/road/kinematic-checked
                      // schedule only if no fully certified forward candidate
                      // is found. It cannot issue READY or complete handover.
                      double travel=0;
                      for(std::size_t i=1;i<candidate.x.size();++i)
                          travel+=std::hypot(candidate.x[i]-candidate.x[i-1],candidate.y[i]-candidate.y[i-1]);
                      const double progress=travel/((candidate.x.size()-1)*path_cfg_.dt);
                      if(!returning&&travel>1e-6&&(progress>mitigation_progress+1e-6||
                          (std::fabs(progress-mitigation_progress)<=1e-6&&cost<mitigation_cost))) {
                          mitigation_cost=cost;mitigation_progress=progress;mitigation.forward_mitigation=true;
                          mitigation.candidate_path=std::move(candidate);mitigation.check=check;
                          mitigation.transition_terminal_s=destination?terminal:0;
                          mitigation.transition_geometry=geometry;
                          mitigation.transition_reused=reuse;mitigation.transition_recovered=recover;
                          mitigation.following_object_id=schedule.leader_id;
                      }
                      continue;
                  }
                  ++result.valid;
                  if(cost>=best)continue;
                  best=cost;result.candidate_valid=true;result.candidate_path=std::move(candidate);result.check=check;
                  if(!highway_committed_)highway_preferred_target_=target;
                  result.transition_terminal_s=destination?terminal:0;
                  result.transition_geometry=geometry;result.transition_reused=reuse;result.transition_recovered=recover;
                  result.following_object_id=schedule.leader_id;result.returning_to_source=returning;
                  if(result.check.covers_completion)break;
              }
          }
          if(result.candidate_valid&&result.check.covers_completion)break;
        }
      }
    }
    if(!result.candidate_valid&&mitigation.forward_mitigation) {
        const auto generated=result.generated,valid=result.valid;
        mitigation.epoch=result.epoch;result=std::move(mitigation);
        result.generated=generated;result.valid=valid;
        result.detail="highway_forward_mitigation; rear_response_unverified="+
            std::to_string(result.check.unverified_rear_id);
    }
    if(result.candidate_valid) {
        if(result.returning_to_source&&!was_returning&&unresolved_forward_blocker>=0&&
           !definite_forward_block) {
            highway_unresolved_return_object_=unresolved_forward_blocker;
            highway_unresolved_return_stamp_=observation;
        } else if(!result.returning_to_source)highway_unresolved_return_object_=-1;
        highway_returning_to_source_=result.returning_to_source;
        result.ready=highway_readiness_.ObserveSafe(now,observation,cfg.safe_confirm_cycles,evidence_gap);
        result.detail=highway_committed_?"highway_executing":
            (result.ready?"highway_ready":"confirming_fresh_highway_observations");
    } else {
        // UNKNOWN is not SAFE: no ready flag/token survives a timeout. Retain
        // only still-fresh past observations so the NEXT complete certificate
        // need not restart confirmation. A real rejection discards evidence.
        if(rejected_candidate||result.detail!="planning_time_budget")highway_readiness_.Reset();
        highway_settled_since_=-1;
    }
    result.progress=std::clamp((ctx.s-zone.config.start_s)/
        (zone.config.completion_s-zone.config.start_s),0.0,1.0);
    if(highway_committed_&&(result.candidate_valid||result.forward_mitigation)) {
        highway_returning_to_source_=result.returning_to_source;
        const auto road_body=asmc_planner::lane_reference::FootprintShape{
            vehicle_shape_.length-vehicle_shape_.rear_overhang,vehicle_shape_.rear_overhang,vehicle_shape_.width*.5};
        const bool aligned=result.returning_to_source?source_aligned(ego):
            !transition_target&&ctx.s-vehicle_shape_.rear_overhang>=zone.config.completion_s&&
            std::fabs(ctx.d)<=cfg.completion_lateral_m&&
            std::fabs(std::remainder(ego.yaw-origin.theta,2*M_PI))<=cfg.completion_heading_rad&&
            zone.target_region->coversFootprint(ego.x,ego.y,ego.yaw,road_body,0);
        if(!aligned)highway_settled_since_=-1;
        else if(highway_settled_since_<0)highway_settled_since_=now;
        result.source_return_completed=result.candidate_valid&&result.returning_to_source&&aligned&&
            now-highway_settled_since_>=cfg.completion_settle_s;
        result.maneuver_completed=result.candidate_valid&&!result.returning_to_source&&aligned&&now-highway_settled_since_>=cfg.completion_settle_s;
        if(!result.forward_mitigation)
            result.detail=result.maneuver_completed?"highway_completed_on_existing_target_reference":"highway_executing";
        if(result.returning_to_source)result.detail=result.source_return_completed?
            "optional_change_source_lane_recovered":"optional_change_returning_to_source";
        if(highway_unresolved_return_object_>=0)
            result.detail+="; awaiting_motion_observation="+std::to_string(highway_unresolved_return_object_);
        output=result.candidate_path;
        if(transition_target) {
            highway_transition_geometry_=result.transition_geometry;
            highway_transition_terminal_s_=result.transition_terminal_s;
            if(result.transition_reused)result.detail+="; retained_transition";
            if(result.transition_recovered)result.detail+="; recovered_transition";
            if(result.following_object_id>=0)result.detail+="; following_object="+
                std::to_string(result.following_object_id);
        }
        highway_acceleration_memory_.Accept(output.a,now,path_cfg_.dt);
    } else if(!highway_committed_&&execute_authorized&&result.ready&&result.candidate_valid&&
              highway_readiness_.WasIssued(now,evidence_gap)) {
        // The ROS queue MUST commit before publishing this output. If its
        // conversion, final freshness or command gate fails it sends safe STOP.
        output=result.candidate_path;result.output_requires_commit=true;
    } else {
        // PREPARE rolls along the source lane with a stop-safe spatial speed
        // envelope ending at its final source-only frontier.
        // After commitment a blocked path brakes in place along this route;
        // it never restarts a wait at a frontier now behind the vehicle.
        build_maintenance();
        if(!maintenance_ok) {
            result.ready=false;highway_readiness_.Reset();
            if(result.check.blocking_object_id>=0)
                last_failure_reason_=PlanFailureReason::COLLISION_BLOCKED;
            result.detail+="; no_safe_corridor_maintenance; emergency_command_not_certified_stop";highway_last_result_=result;return false;
        }
        if(highway_committed_)result.detail="highway_braking: "+result.detail;
        output=std::move(maintenance_path);
        highway_acceleration_memory_.Reset();
    }
    last_mode_=optional?command.mode:MERGE;last_failure_reason_=PlanFailureReason::NONE;
    highway_last_result_=result;return !output.x.empty();
}
