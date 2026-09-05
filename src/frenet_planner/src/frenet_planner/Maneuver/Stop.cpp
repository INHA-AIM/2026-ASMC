/**
 * @file Stop.cpp
 * @brief 외부 STOP(APPROACH/BRAKING/HOLD)과 EMERGENCY_STOP 실행부.
 *
 * PlanInternal의 공용 candidate 파이프라인(ResolveManeuver/EvaluateCosts),
 * GenerateDecelToZeroPath/BuildZeroSpeedHoldPath 자체(AVOID와 공유),
 * common no-valid failure 분류는 이 파일로 옮기지 않는다. 이 파일은 STOP/
 * EMERGENCY 전용 gate가 이미 참으로 확정된 뒤의 실행부와, STOP phase
 * lifecycle 갱신만 담당한다 - 호출 순서/gate 자체는 PlanInternal이 소유한다.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "frenet_planner/Maneuver/ManeuverUtils.hpp"

bool FrenetPlanner::PlanExternalEmergencyStop(const CartesianState& ego, double s,
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

    const double decel = std::max(0.1, stop_cfg_.comfortable_decel);
    const double speed = std::max(0.0, s_dot);
    const double braking_distance = speed * speed / (2.0 * decel);
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
        const double v_safe = std::sqrt(2.0 * decel * effective);
        // 순항 목표속도와 비교해 더 낮은 쪽. 멀면 기존 속도 유지, 가까워질
        // 수록 자연스럽게 낮아진다.
        cmd.target_speed = std::min(cmd.target_speed, v_safe);
        // 후보 생성만 velocity keeping으로 위임한다. 외부에 보고되는
        // last_mode_는 아래에서 STOP으로 유지한다.
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
                                    std::sqrt(2.0 * decel * effective));
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
