#include "Frenet/frenet_planner.hpp"
#include "Frenet/Generator/BoundaryConditionPolicy.hpp"
#include "Frenet/Evaluation/Cost.hpp"
#include "Frenet/Math/FrenetConverter.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Speed/CurveSpeed.hpp"
#include "Frenet/Generator/SpatialStop.hpp"
#include <algorithm>
#include <cmath>

bool FrenetPlanner::CanHandoffAvoidToKeep(
    const CartesianState& ego, double s, double d,
    const std::vector<TrackedObject>& obstacles) const {
    if (!execution_->avoid.active || !execution_->avoid.committed ||
        ref_.points.empty() || !std::isfinite(d) ||
        std::fabs(d) > avoid_cfg_.lateral_tolerance) return false;
    const double length = ref_.points.back().s;
    const double gap = ReferenceArcDistance(ref_, s, v2_avoidance_.obstacle_s);
    const double clearance = avoid_cfg_.rear_overhang +
        v2_avoidance_.obstacle_longitudinal_half_extent + avoid_cfg_.pass_clearance;
    if (!std::isfinite(gap) || gap >= -clearance) return false;
    if(v2_avoidance_.map_group_valid &&
       ReferenceArcDistance(ref_, s, v2_avoidance_.map_group_end_s)>0.0)return false;
    const double heading_error = std::remainder(
        ego.yaw - Interpolate(ref_,s).theta,2.0*M_PI);
    // Heading must already be settled, including at standstill. Bound the
    // heading-induced body displacement by the existing lateral tolerance.
    const double heading_limit = std::atan2(avoid_cfg_.lateral_tolerance,
                                            vehicle_shape_.length);
    if (!std::isfinite(heading_error) || std::fabs(heading_error)>heading_limit)
        return false;

    // A short, slow KEEP candidate can stop before a downstream obstacle. That
    // is not evidence that the remaining static avoidance is finished. Check
    // every static/uncertain OBB against the finite response-and-stop body
    // corridor. No ID or width approximation.
    std::vector<TrackedObject> stationary;
    for (auto o : obstacles) {
        if (o.motion_state == ObjectMotionState::DYNAMIC) continue;
        o.vx=o.vy=o.speed=o.yaw_rate=0.0;
        o.motion_state=ObjectMotionState::LOW_THREAT;
        stationary.push_back(o);
    }
    if (!stationary.empty()) {
        CartesianPath corridor;
        // Certify the finite handoff and its full stop, not an obstacle-free
        // acquisition horizon. A distant next target belongs to a new command.
        FrenetState motion{};motion.s_d=std::max(0.0,ego.v);motion.s_dd=ego.a;
        const auto stop=GenerateBrakingContinuation(motion,
            limits_.max_longitudinal_accel*.8,limits_.max_longitudinal_jerk,path_cfg_.dt);
        if(stop.s.empty())return false;
        const double response=ego.v*(.15+2*replanning_period_s_+.2);
        const double distance=std::min(length*.5,std::max(1.0,stop.s.back()+response));
        constexpr double step=.25;
        for (double traveled=0.0;traveled<distance+step;traveled+=step) {
            const auto p=Interpolate(ref_,std::fmod(s+std::min(traveled,distance),length));
            corridor.x.push_back(p.x);corridor.y.push_back(p.y);
            corridor.yaw.push_back(p.theta);corridor.kappa.push_back(p.kappa);
            corridor.v.push_back(0.0);corridor.a.push_back(0.0);
        }
        auto geometry_only=collision_cfg_;
        geometry_only.margin_growth_rate=0.0;
        geometry_only.reactive_lookahead=0.0;
        if (!CartesianAvoidTrajectoryClear(corridor,stationary,vehicle_shape_,
                                           geometry_only,path_cfg_.dt)) return false;
    }

    // Use KEEP's pure generator/filter component: no mode, speed-memory,
    // candidate-debug, lifecycle or controller mutation during this preview.
    double projected_s=s,s_dot=0,s_ddot=0,projected_d=d,d_prime=0,d_pprime=0;
    CartesianToFrenet(ref_,ego,projected_s,s_dot,s_ddot,projected_d,d_prime,d_pprime,
                      &s,ref_config_.ego_projection_search_window);
    double d_dot=0,d_ddot=0;
    ArcDerivToTimeDeriv(s_dot,s_ddot,d_prime,d_pprime,d_dot,d_ddot);
    auto start=ClampInitialBoundaryState(projected_s,s_dot,s_ddot,projected_d,
                                         d_dot,d_ddot,limits_);
    const auto cfg=MakePipelineConfigBundle();
    const double raw_cap=LookaheadTargetSpeed(ref_,s,ego.v,cfg.curve_speed,
        cfg.limits.max_longitudinal_accel,cfg.limits.max_lateral_accel,
        cfg.limits.max_curvature);
    const SpeedProfileLimits profile_limits{max_target_speed_accel_,
        std::max(cfg.curve_speed.min_usable_decel,
                 cfg.limits.max_longitudinal_accel*cfg.curve_speed.comfortable_decel_factor),
        cfg.limits.max_longitudinal_jerk};
    // Normal KEEP resets its profiler when entering from another maneuver.
    const SpeedProfiler fresh_profiler;
    const auto profile=fresh_profiler.Preview({raw_cap,ego.v,ego.a,replanning_period_s_},
                                               profile_limits);
    TrajectoryRequest request;
    request.longitudinal_mode=LongitudinalMode::VELOCITY_KEEPING;
    request.lateral.target_d=0.0;request.lateral.search_tolerance=0.0;
    request.target_speed=profile.target_speed;
    auto envelope_cfg=cfg.curve_speed;
    const double headroom=cfg.limits.boundary_condition_accel_fraction;
    envelope_cfg.comfortable_decel_factor=std::min(envelope_cfg.comfortable_decel_factor,headroom);
    envelope_cfg.min_usable_decel=std::min(envelope_cfg.min_usable_decel,
                                          cfg.limits.max_longitudinal_accel*headroom);
    request.speed_envelope=BuildKeepSpeedEnvelope(ref_,s,s_dot,start.s_dd,
        cfg.path.time_horizon.max,cfg.path.dt,request.target_speed,envelope_cfg,
        cfg.limits.max_longitudinal_accel,cfg.limits.max_lateral_accel,
        cfg.limits.max_longitudinal_jerk*headroom);
    if (request.speed_envelope.empty()) return false;
    start.s_dd=ResolveEnvelopeBoundaryAcceleration(start.s_dd,s_dot,raw_cap,
                    request.speed_envelope,cfg.limits.accel_intent_deadband_mps);
    const TrajectoryPipeline pipeline(cfg,ref_);
    auto candidates=pipeline.GenerateCandidates(start,request,cfg.limits,
                                                 obstacles,true,nullptr);
    EvaluateCosts(candidates,cfg.cost_weights,0.0);
    const auto* best=SelectBestPath(candidates);
    if (!best) return false;
    const auto path=ConvertToCartesianPath(*best,ref_);
    if (path.x.size()<2) return false;
    const VehicleShape map_shape=vehicle_shape_;
    if (avoid_road_map_.enabled() &&
        !avoid_road_map_.EvaluateRecoveryPath(path,map_shape,path_cfg_.dt,
                                              0.0,0.0,false,0.0,true,
                                              solid_paint_contact_elapsed_s_).drivable) return false;
    // Preserve the AVOID dual stationary/moving hypothesis even though
    // normal KEEP's pipeline has already applied its collision filter.
    return CartesianAvoidTrajectoryClear(path,obstacles,vehicle_shape_,
                                          collision_cfg_,path_cfg_.dt);
}
