/**
 * @file Stop.cpp
 * @brief 외부 STOP(APPROACH/BRAKING/HOLD)과 EMERGENCY_STOP 실행부, 그리고
 *        StopProcess/EmergencyStopProcess 진입점.
 *
 * KEEP/FOLLOW/STOP이 공유하는 candidate 파이프라인 본체(ResolveManeuver/
 * EvaluateCosts를 포함한 ExecuteManeuverPipeline)는 Keep.cpp에 있다 -
 * STOP 전용 분기(external_stop_mode)가 그 파이프라인 뒷부분 전체에 걸쳐
 * 있어서 여기로 옮기면 같은 조건을 두 곳에서 유지해야 한다. 이 파일은
 * STOP/EMERGENCY 전용 gate가 이미 참으로 확정된 뒤의 실행부(APPROACH/
 * BRAKING/HOLD phase 전이, EMERGENCY 직접 감속)와 그 진입점만 담당한다 -
 * 호출 순서/gate 자체는 PlanInternal이 소유한다.
 */

#include "Frenet/frenet_planner.hpp"
#include "Frenet/Generator/StoppingLateralGeometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Frenet/Generator/SpatialStop.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Math/FrenetConverter.hpp"
#include "Frenet/Math/Polynomial.hpp"


bool FrenetPlanner::PlanCollisionBrakingPath(
    const CartesianState& ego, double s, double d,
    const std::vector<TrackedObject>& obstacles, CartesianPath& out_path,
    double service_decel, double target_d, double lateral_distance) const {
    const double dt = path_cfg_.dt;
    const double accel = std::max(limits_.max_longitudinal_accel, service_decel);
    const double jerk = limits_.max_longitudinal_jerk;
    if (!std::isfinite(ego.v) || !std::isfinite(ego.a) || ego.v <= .1 ||
        accel <= 0 || jerk <= 0 || dt <= 0 || std::fabs(ego.a) > accel)
        return false;

    // Exact three-stage jerk-limited stop: ramp to negative acceleration,
    // hold it if needed, then ramp back to zero at zero velocity.
    const double peak = std::min(accel, std::sqrt(jerk*ego.v + .5*ego.a*ego.a));
    const double down = (ego.a + peak)/jerk;
    const double v1 = ego.v + (ego.a*ego.a - peak*peak)/(2*jerk);
    const double raw_hold = (v1 - peak*peak/(2*jerk))/peak;
    if (down < 0 || raw_hold < -1e-8) return false;
    const double hold = std::max(0.0, raw_hold);
    const double up = peak/jerk;
    const double end = down + hold + up;
    const double d1 = ego.v*down + .5*ego.a*down*down - jerk*down*down*down/6;
    const double d2 = d1 + v1*hold - .5*peak*hold*hold;
    const double v2 = v1 - peak*hold;
    const double stop_distance = d2 + v2*up - .5*peak*up*up + jerk*up*up*up/6;
    const auto motion = [&](double t) -> std::array<double,3> {
        if (t < down)
            return {ego.v*t + .5*ego.a*t*t - jerk*t*t*t/6,
                    ego.v + ego.a*t - .5*jerk*t*t, ego.a - jerk*t};
        t -= down;
        if (t < hold) return {d1 + v1*t - .5*peak*t*t, v1 - peak*t, -peak};
        t -= hold;
        if (t < up)
            return {d2 + v2*t - .5*peak*t*t + jerk*t*t*t/6,
                    v2 - peak*t + .5*jerk*t*t, -peak + jerk*t};
        return {stop_distance, 0.0, 0.0};
    };

    const auto rp0 = Interpolate(ref_, s);
    const double slope = (1-rp0.kappa*d)*
        std::tan(std::remainder(ego.yaw-rp0.theta, 2*M_PI));
    if (!std::isfinite(slope)) return false;
    // Same gradual centering distance as AVOID's braking maintenance path.
    const double settle = lateral_distance > 0.0 ? lateral_distance
        : std::max(10.0, std::min(25.0, ego.v*2.5));
    double initial_second = 0.0;
    if (service_decel > 0.0) {
        double ps, pv, pa, pd, dp, dpp;
        CartesianToFrenet(ref_, ego, ps, pv, pa, pd, dp, dpp,
                         &s, ref_config_.ego_projection_search_window);
        initial_second = dpp;
    }
    if (!std::isfinite(initial_second)) return false;
    const auto lateral = MakeQuintic(d, slope, initial_second,
                                     target_d, 0, 0, settle);
    const double duration = std::max(end+dt, collision_cfg_.reactive_lookahead);
    if (!std::isfinite(duration) || duration/dt > 1000) return false;
    const int count = static_cast<int>(std::ceil(duration/dt));
    CartesianPath path;
    double u = 0, previous_distance = 0, previous_metric = 1;
    for (int n=0; n<=count; ++n) {
        const auto state = motion(n*dt);
        if (n) u += (state[0]-previous_distance)/previous_metric;
        previous_distance = state[0];
        const auto rp = Interpolate(ref_, s+u);
        const double q = std::min(u, settle);
        const double offset = EvalPos(lateral,q), prime = EvalVel(lateral,q);
        const double second = EvalAcc(lateral,q);
        const double metric = std::hypot(1-rp.kappa*offset,prime);
        if (!std::isfinite(metric) || metric < .1) return false;
        previous_metric = metric;
        const double v = std::max(0.0,state[1]), a = state[2];
        const auto point = FrenetToCartesian(rp,s+u,v/metric,a/metric,offset,prime,second);
        if (!std::isfinite(point.kappa) || std::fabs(point.kappa)>limits_.max_curvature ||
            v*v*std::fabs(point.kappa)>limits_.max_lateral_accel) return false;
        path.x.push_back(point.x);path.y.push_back(point.y);path.yaw.push_back(point.yaw);
        path.kappa.push_back(point.kappa);path.v.push_back(v);path.a.push_back(a);
    }
    path.x[0]=ego.x;path.y[0]=ego.y;path.yaw[0]=ego.yaw;path.kappa[0]=ego.kappa;
    const bool road_clear =
        avoid_road_map_.EvaluateRecoveryPath(path,vehicle_shape_,dt).drivable;
    const auto collision = EvaluateCartesianTrajectorySafety(
        path, obstacles, vehicle_shape_, collision_cfg_, dt);
    if (!road_clear || !collision.clear) {
        if (service_decel > 0.0)
            std::printf("[FrenetPlanner-STOP] braking rejected: road=%d blocker=%d time=%.2f\n",
                        road_clear ? 1 : 0, collision.blocking_object_id,
                        collision.blocking_sample_index * dt);
        return false;
    }
    out_path=std::move(path);
    return true;
}

bool FrenetPlanner::EmergencyStopProcess(const CartesianState& ego, double s,
                                               double d, double s_dot,
                                               CartesianPath& out_path) {
    last_mode_ = EMERGENCY;
    last_target_speed_ = 0.0;
    if (!GenerateDecelToZeroPath(s, d, s_dot, out_path)) {
        last_failure_reason_ = PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
        return false;
    }
    static int emergency_log_tick = 0;
    if (++emergency_log_tick % 5 == 0) {
        std::printf("[FrenetPlanner-STATE] mode=EMERGENCY_STOP s=%.2f d=%.3f "
                    "speed=%.2f samples=%zu\n",
                    s, d, ego.v, out_path.v.size());
    }
    last_failure_reason_ = PlanFailureReason::NONE;
    return true;
}

bool FrenetPlanner::SignalStopFootprintBeforeLine(const CartesianState& ego,
    const asmc_planner::signal::StopConstraint& signal) const {
    asmc_planner::signal::Geometry physical;
    physical.key=signal.key;physical.reviewed=signal.valid();
    physical.line=signal.physical_line;physical.approach=signal.approach;
    asmc_planner::signal::Pose pose;
    pose.valid=true;pose.x=ego.x;pose.y=ego.y;pose.yaw=ego.yaw;
    pose.speed=std::max(0.,ego.v);pose.acceleration=ego.a;
    asmc_planner::signal::Config shape;
    shape.front_extent=vehicle_shape_.length-avoid_cfg_.rear_overhang;
    shape.rear_extent=avoid_cfg_.rear_overhang;
    shape.half_width=std::max(vehicle_shape_.width,vehicle_shape_.road_outer_width)/2;
    shape.position_error=signal.position_error;shape.clearance=signal.clearance;
    return asmc_planner::signal::BodyBeforeLine(pose,physical,shape);
}

void FrenetPlanner::UpdateExternalStopExecution(const CartesianState& ego,double s, double s_dot,
                                                 PlannerCommand& cmd,
                                                 double& out_stop_distance) {
    const double s_max = ref_.points.empty() ? 0.0 : ref_.points.back().s;
    out_stop_distance = ForwardArcDistance(s, cmd.stop_position, s_max);
    double stop_distance = out_stop_distance;

    // 목표가 바뀌면 새 STOP 명령으로 보고 실행 단계를 리셋한다. command_id는
    // publish마다 증가할 수 있어 phase 유지 판단 기준으로 쓸 수 없다.
    const bool signal_stop=cmd.signal_stop.valid();
    if (signal_stop) {
        const bool new_signal_target = !stop_.target_valid ||
            stop_.signal.key != cmd.signal_stop.key;
        if (new_signal_target) {
            // A completed stop belongs to one physical line and movement.
            // A later light must never inherit its HOLD phase, even when KEEP
            // commands were executed between the two signal requests.
            execution_.resetStopTarget();
            stop_.goal_failed=false;
        }
        // A revision is an explicit update even below the legacy 2m threshold.
        // Preserve BRAKING/HOLD only within the same signal key; updating its
        // goal never grants forward motion.
        if (stop_.signal.key!=cmd.signal_stop.key ||
            stop_.signal.target_revision!=cmd.signal_stop.target_revision ||
            stop_.target_s!=cmd.stop_position) stop_.goal_reached=false;
        stop_.signal=cmd.signal_stop;
        stop_.target_s=cmd.stop_position;stop_.target_valid=true;
        stop_.reapproaching=false;
        out_stop_distance=cmd.stop_position-s;
        stop_distance=out_stop_distance;
    } else if (!stop_.target_valid ||
        !std::isfinite(stop_.target_s) ||
        std::fabs(cmd.stop_position - stop_.target_s) >
            stop_cfg_.target_change_tolerance) {
        execution_.resetStopTarget();
        stop_.target_s = cmd.stop_position;
        stop_.target_valid = true;
        stop_.reapproaching = false;
        stop_.signal={};
    }

    const double comfortable_decel = std::max(0.1, stop_cfg_.comfortable_decel);
    const double max_service_decel =
        std::max(comfortable_decel, stop_cfg_.max_service_decel);
    const double speed = std::max(0.0, s_dot);
    const double effective_distance =
        std::max(0.0, stop_distance - stop_cfg_.stop_buffer);
    // 늦게 들어온 STOP도 comfortable_decel에 고정하지 않는다. 현재 속도로
    // 남은 유효거리 안에 정지하는 데 필요한 평균 감속도를 계산하고 일반
    // 제동 한계까지 올린다. 거리 0은 나눗셈 대신 곧바로 최대 일반 제동이다.
    const double required_decel =
        effective_distance > 1e-3
            ? speed * speed / (2.0 * effective_distance)
            : max_service_decel;
    stop_.commanded_decel = std::min(
        max_service_decel, std::max(comfortable_decel, required_decel));

    const double braking_distance =
        speed * speed / (2.0 * comfortable_decel);
    const double trigger_distance = braking_distance + stop_cfg_.braking_margin;

    // APPROACH -> BRAKING은 단방향(latch). 경계에서 측정 노이즈로 왕복하면
    // 후보 생성기가 매 사이클 바뀌어 속도 프로파일이 흔들린다.
    const double final_distance=cmd.stop_position-s-stop_cfg_.stop_buffer;
    // The 0.5 m endpoint buffer is a planning margin, not a zero-tolerance
    // physical line. A small measured tracking/projection error inside the
    // configured target tolerance is still checked against the full body and
    // physical line below; it must not become a latched execution failure.
    if(signal_stop && final_distance<-stop_cfg_.position_tolerance)
        stop_.goal_failed=true;
    const bool low_speed=std::fabs(s_dot)<=stop_cfg_.speed_tolerance;
    const bool at_target = signal_stop ?
        std::abs(final_distance)<=stop_cfg_.position_tolerance &&
        SignalStopFootprintBeforeLine(ego,cmd.signal_stop) :
        stop_distance <= stop_cfg_.position_tolerance;
    if(signal_stop) {
        stop_.goal_reached=low_speed && at_target;
        if(low_speed && execution_->stop_phase!=StopExecutionPhase::APPROACH)
            execution_.holdStop();
    }
    execution_.observeStop({(signal_stop && cmd.signal_stop.braking_only) ||
        stop_distance <= trigger_distance, at_target,low_speed});

    if (execution_->stop_phase == StopExecutionPhase::APPROACH) {
        // 목표점이 아니라 buffer만큼 앞을 향해 감속한다.
        const double effective =
            std::max(0.0, stop_distance - stop_cfg_.stop_buffer);
        const double v_safe = std::sqrt(2.0 * comfortable_decel * effective);
        // 순항 목표속도와 비교해 더 낮은 쪽. 멀면 기존 속도 유지, 가까워질
        // 수록 자연스럽게 낮아진다.
        cmd.target_speed = std::min(cmd.target_speed, v_safe);
        // cmd.mode는 여기서부터 "외부 behavior mode"가 아니라 후보 생성기
        // 선택용 내부 scratch 값이다(PlannerCommand.hpp 알려진 예외 참고).
        // LANE_KEEPING으로 바꾸는 것은 ResolveManeuver에 velocity keeping
        // 생성기를 쓰라고 지시하는 것뿐이고, 실제 STOP 여부는 last_mode_와
        // execution_->stop_phase가 그대로 STOP으로 보고한다 - 외부에서 관측 가능한 어떤
        // 값도 STOP에서 KEEP으로 바뀌지 않는다.
        cmd.mode = LANE_KEEPING;
    } else if (execution_->stop_phase == StopExecutionPhase::BRAKING) {
        // 목표를 이미 지나쳤다면 되돌아가는 정지 다항식을 만들지 않는다.
        if (stop_distance < 0.0) {
            cmd.stop_position = s;
        }
        // BRAKING의 종방향 프로파일은 stopping quintic이 정하므로
        // cmd.target_speed는 후보 생성에 쓰이지 않는다. 다만 진단값
        // (last_target_speed_)이 순항속도로 남으면 실제로는 0을 향해
        // 감속 중인데 10m/s를 지령한 것처럼 보이므로 정지 목표에 맞춘
        // 감속 프로파일 값으로 보고한다.
        const double effective =
            std::max(0.0, stop_distance - stop_cfg_.stop_buffer);
        cmd.target_speed = std::min(cmd.target_speed,
                                    std::sqrt(2.0 * stop_.commanded_decel * effective));
    }
}

void FrenetPlanner::PlanExternalStopHold(const CartesianState& ego, double s,
                                         CartesianPath& out_path) const {
    const RefPoint rp = Interpolate(ref_, s);
    out_path.x.assign(1, ego.x);
    out_path.y.assign(1, ego.y);
    out_path.yaw.assign(1, std::isfinite(ego.yaw) ? ego.yaw : rp.theta);
    out_path.kappa.assign(1, 0.0);
    out_path.v.assign(1, 0.0);
    out_path.a.assign(1, 0.0);
}

bool FrenetPlanner::PlanExternalStopBrakingFallback(
    const CartesianState& ego, double s, double d, double stop_distance,
    double target_d, const std::vector<TrackedObject>& obstacles,
    CartesianPath& out_path, bool preserve_service_braking) {
    // This is the final fallback, after target-preserving spatial STOP
    // candidates. Do not publish an unchecked constant-d path or pretend that
    // stopping early means the requested target has been reached (no HOLD latch).
    // A signal STOP can reach its terminal longitudinal buffer while still
    // moving slowly. Forcing the remaining lateral error to zero within that
    // last metre can require impossible curvature and reject an otherwise
    // safe braking path. Preserve the current steering trend over a normal
    // lateral settling distance; the produced short path is still checked
    // against the road, obstacles and physical stop line before publication.
    const double settle = stop_.signal.valid()
        ? std::max(10.0, ego.v * 2.5)
        : std::max(1.0, std::min(
            std::max(1.0, stop_distance), std::max(10.0, ego.v * 2.5)));
    CartesianState braking_ego = ego;
    // Ordinary distance-native STOP retains achieved service braking. Legacy
    // roundabout waiting keeps its existing boundary policy.
    // At very low speed a stronger negative boundary cannot be released to
    // a=0 with the jerk limit before v=0, and would demand reverse motion.
    // Limit only that infeasible boundary; keep the measured state untouched.
    const double boundary_limit = limits_.max_longitudinal_accel *
        limits_.boundary_condition_accel_fraction;
    const double release_limit = std::sqrt(2.0 *
        limits_.max_longitudinal_jerk * std::max(0.0, ego.v));
    const double braking_limit = preserve_service_braking
        ? std::max(limits_.max_longitudinal_accel, stop_cfg_.max_service_decel)
        : boundary_limit;
    braking_ego.a = std::clamp(ego.a,
        -std::min(braking_limit, release_limit), boundary_limit);
    // This fallback is a braking request, not a request to repeat observed
    // throttle. Keep the achieved negative boundary; release positive memory.
    if (preserve_service_braking) braking_ego.a = std::min(0.0,braking_ego.a);
    if (!PlanCollisionBrakingPath(braking_ego, s, d, obstacles, out_path,
                                  stop_cfg_.max_service_decel, target_d, settle)) {
        out_path = {};
        // Explicit safety failure forbids runtime reuse of an older path.
        last_failure_reason_ = PlanFailureReason::COLLISION_BLOCKED;
        return false;
    }
    if (preserve_service_braking && stop_distance >= 0.0) {
        double hint = s;
        for (std::size_t i=0; i<out_path.x.size(); ++i) {
            hint = FindClosestS(ref_,out_path.x[i],out_path.y[i],&hint,5);
            const double final_s=s+stop_distance-(stop_.signal.valid() ? stop_cfg_.stop_buffer : 0.0);
            if (hint > final_s + stop_cfg_.position_tolerance) {
                if(stop_.signal.valid()) stop_.goal_failed=true;
                // The strongest checked service stop cannot meet this late
                // target. Keep its braking/steering rather than replacing it
                // with an unchecked straight stop. Never report goal arrival.
                std::printf("[FrenetPlanner-STOP] unavoidable service-stop overrun: "
                    "target_s=%.2f brake_s=%.2f\n",s+stop_distance,hint);
                break;
            }
        }
    }
    last_candidate_paths_.push_back(out_path);
    last_candidate_validity_.push_back(true);
    std::printf("[FrenetPlanner-STOP] checked steerable braking: "
                "s=%.2f v=%.2f target_distance=%.2f samples=%zu\n",
                s, ego.v, stop_distance, out_path.v.size());
    last_failure_reason_ = PlanFailureReason::NONE;
    return true;
}

void FrenetPlanner::AppendExternalStopSpatialCandidates(
    const CartesianState& ego, const FrenetState& start,
    const TrajectoryRequest& request, const KinematicLimits& limits,
    const std::vector<TrackedObject>& obstacles,
    std::vector<FrenetPath>& candidates) const {
    PathGeneratorConfig generator = path_cfg_;
    std::vector<FrenetPath> longitudinal;
    if (request.longitudinal_mode == LongitudinalMode::STOPPING) {
        generator.delta_s = request.stop.terminal_offset;
        longitudinal = request.stop.spatial_profile
            ? GenerateSpatialStoppingCandidates(start, request.stop.stop_s,
                request.stop.cruise_speed, request.stop.comfortable_decel,
                generator, limits, request.stop.acceleration_limit,request.stop.braking_only)
            : GenerateStoppingCandidates(start, request.stop.stop_s, generator);
    } else {
        longitudinal = GenerateVelocityKeepingCandidates(start, request.target_speed, generator);
    }
    FilterLongitudinalByKinematicLimits(longitudinal, limits);
    double ps, pv, pa, pd, slope, second;
    CartesianToFrenet(ref_, ego, ps, pv, pa, pd, slope, second,
                     &start.s, ref_config_.ego_projection_search_window);
    if (!std::isfinite(slope) || !std::isfinite(second)) return;

    std::vector<FrenetPath> spatial;
    for (const auto& lon : longitudinal) {
        if (!lon.valid || lon.s.size() < 2) continue;
        const double span = lon.s.back() - start.s;
        if (!std::isfinite(span) || span <= 1e-3) continue;
        // Preserve the ordinary STOP candidate order and values while sharing
        // the same bounded geometry generation with MERGE waiting.
        for (const auto& geometry : MakeStoppingLateralGeometries(start.d,
                slope, second, request.lateral.target_d, span, ego.v, limits)) {
            const auto& lateral = geometry.polynomial;
            const double distance = geometry.distance;
            FrenetPath path = lon;
            path.d.clear(); path.d_d.clear(); path.d_dd.clear();
            path.jerk_cost_lat = 0.0;
            for (std::size_t i = 0; i < lon.s.size(); ++i) {
                const double u = std::max(0.0, std::min(distance, lon.s[i] - start.s));
                const double dp = EvalVel(lateral, u), dpp = EvalAcc(lateral, u);
                path.d.push_back(EvalPos(lateral, u));
                path.d_d.push_back(dp * lon.s_d[i]);
                path.d_dd.push_back(dpp * lon.s_d[i] * lon.s_d[i] + dp * lon.s_dd[i]);
                if (i) {
                    const double dt = lon.t[i] - lon.t[i-1];
                    const double jerk = (path.d_dd[i] - path.d_dd[i-1]) / dt;
                    path.jerk_cost_lat += jerk * jerk * dt;
                }
            }
            spatial.push_back(std::move(path));
        }
    }
    FilterLateralByAcceleration(spatial, limits);
    FilterByCurvature(spatial, ref_, limits, CartesianGeometryMode::GEOMETRIC_YAW_CURVATURE);
    for (auto& path : spatial) {
        if (!path.valid) continue;
        const auto cart = ConvertToCartesianPath(
            path, ref_, CartesianGeometryMode::GEOMETRIC_YAW_CURVATURE);
        for (std::size_t i=0; i<cart.v.size(); ++i) {
            if (!std::isfinite(cart.v[i]) || !std::isfinite(cart.a[i]) ||
                cart.v[i] < -1e-6 ||
                std::fabs(cart.a[i]) > limits.max_longitudinal_accel + 1e-6 ||
                cart.v[i]*cart.v[i]*std::fabs(cart.kappa[i]) > limits.max_lateral_accel + 1e-6) {
                path.valid = false;
                path.rejection_reason = RejectionReason::CURVATURE;
                break;
            }
        }
    }
    FilterByCollision(spatial, ref_, obstacles, vehicle_shape_, collision_cfg_);
    candidates.insert(candidates.end(), std::make_move_iterator(spatial.begin()),
                       std::make_move_iterator(spatial.end()));
}

void FrenetPlanner::TrimExternalStopReverseTrajectory(
    CartesianPath& selected_cartesian) const {
    std::size_t stop_index = selected_cartesian.v.size();
    for (std::size_t i = 0; i < selected_cartesian.v.size(); ++i) {
        if (selected_cartesian.v[i] < 0.0) { stop_index = i; break; }
    }
    if (stop_index < selected_cartesian.v.size()) {
        // 최소 한 점은 남겨 MPC가 추종할 자세 정보를 잃지 않게 한다.
        const std::size_t keep = std::max<std::size_t>(1, stop_index);
        selected_cartesian.x.resize(keep);
        selected_cartesian.y.resize(keep);
        selected_cartesian.yaw.resize(keep);
        selected_cartesian.kappa.resize(keep);
        selected_cartesian.v.resize(keep);
        selected_cartesian.a.resize(keep);
        selected_cartesian.v.back() = 0.0;
        selected_cartesian.a.back() = 0.0;
    }
}

bool FrenetPlanner::StopProcess(
    const CartesianState& ego, const FrenetState& start,
    double s, double d, double s_dot, double s_ddot,
    double d_dot, double d_ddot,
    const std::vector<TrackedObject>& obstacles,
    const std::vector<TrackedObject>& predicted_obstacles,
    double resolved_target_speed, const PlannerCommand& external_command,
    CartesianPath& out_path) {
    // STOP의 APPROACH/BRAKING/HOLD phase 갱신과 HOLD 조기반환은 공용
    // pipeline(Keep.cpp의 ExecuteManeuverPipeline) 안에서
    // external_stop_mode 분기로 처리한다 - SelectBestStoppingPath/
    // TrimExternalStopReverseTrajectory/BRAKING fail-safe 등 STOP 전용
    // 로직이 파이프라인 뒷부분 전체에 걸쳐 있어서, 그 부분만 분리하면
    // 같은 조건을 두 곳에서 유지해야 한다. 여기서는 그 진입점만 명확히
    // 이름 붙인다.
    return ExecuteManeuverPipeline(
        ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
        predicted_obstacles, resolved_target_speed, external_command,
        out_path);
}


