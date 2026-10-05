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

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "frenet_planner/Maneuver/ManeuverUtils.hpp"
#include "frenet_planner/Math/Polynomial.hpp"


bool FrenetPlanner::PlanCollisionBrakingPath(
    const CartesianState& ego, double s, double d,
    const std::vector<TrackedObject>& obstacles, CartesianPath& out_path) const {
    const double dt = path_cfg_.dt;
    const double accel = limits_.max_longitudinal_accel;
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
    const double settle = std::max(10.0, std::min(25.0, ego.v*2.5));
    const auto lateral = MakeQuintic(d, slope, 0, 0, 0, 0, settle);
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
    if (!avoid_road_map_.EvaluateRecoveryPath(path,vehicle_shape_,dt).drivable ||
        !CartesianTrajectoryClear(path,obstacles,vehicle_shape_,collision_cfg_,dt)) return false;
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

void FrenetPlanner::UpdateExternalStopExecution(double s, double s_dot,
                                                 PlannerCommand& cmd,
                                                 double& out_stop_distance) {
    const double s_max = ref_.points.empty() ? 0.0 : ref_.points.back().s;
    out_stop_distance = ForwardArcDistance(s, cmd.stop_position, s_max);
    const double stop_distance = out_stop_distance;

    // 목표가 바뀌면 새 STOP 명령으로 보고 실행 단계를 리셋한다. command_id는
    // publish마다 증가할 수 있어 phase 유지 판단 기준으로 쓸 수 없다.
    if (!stop_.target_valid ||
        !std::isfinite(stop_.target_s) ||
        std::fabs(cmd.stop_position - stop_.target_s) >
            stop_cfg_.target_change_tolerance) {
        stop_.phase = StopExecutionPhase::APPROACH;
        stop_.target_s = cmd.stop_position;
        stop_.target_valid = true;
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
    if (stop_.phase == StopExecutionPhase::APPROACH &&
        stop_distance <= trigger_distance) {
        stop_.phase = StopExecutionPhase::BRAKING;
    }

    // 목표에 도달했고 충분히 느리면 HOLD. 목표를 지나쳤어도(음수 거리)
    // 후진하지 않고 그 자리에서 정지를 유지한다.
    const bool at_target = stop_distance <= stop_cfg_.position_tolerance;
    if (at_target && std::fabs(s_dot) <= stop_cfg_.speed_tolerance) {
        stop_.phase = StopExecutionPhase::HOLD;
    }

    if (stop_.phase == StopExecutionPhase::APPROACH) {
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
        // stop_.phase가 그대로 STOP으로 보고한다 - 외부에서 관측 가능한 어떤
        // 값도 STOP에서 KEEP으로 바뀌지 않는다.
        cmd.mode = LANE_KEEPING;
    } else if (stop_.phase == StopExecutionPhase::BRAKING) {
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

void FrenetPlanner::PlanExternalStopBrakingFallback(double s, double d,
                                                     double s_dot,
                                                     double stop_distance,
                                                     CartesianPath& out_path) {
    GenerateDecelToZeroPath(s, d, s_dot, out_path);
    const double v0 = std::max(0.0, s_dot);
    std::printf("[FrenetPlanner-STOP] BRAKING fail-safe: no candidate, "
                "emitting decel-to-zero path (s=%.2f v=%.2f dist=%.2f)\n",
                s, v0, stop_distance);
    last_failure_reason_ = PlanFailureReason::NONE;
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

