#include "MPC/MPC.hpp"

#include <algorithm>
#include <cmath>

#include "MPC/Cost/cost_function.hpp"
#include "MPC/Math/utils.hpp"
#include "MPC/Model/vehicle_model.hpp"
#include "MPC/Solver/mpc_solver.hpp"
#include "MPC/Reference/braking_reference.hpp"
#include "MPC/Reference/reference_validator.hpp"

namespace {

int CyclesFromSeconds(double seconds, double frequency_hz) {
    return std::max(1, static_cast<int>(std::ceil(seconds * frequency_hz)));
}

void RecordPublishedSteering(double steering) {
    g_mpc.steering_history.push_back(steering);
    const std::size_t history_size = static_cast<std::size_t>(std::ceil(
        g_mpc.params.steering_response_delay_s / g_mpc.params.dt)) + 1;
    if (g_mpc.steering_history.size() > history_size) {
        g_mpc.steering_history.erase(g_mpc.steering_history.begin(),
            g_mpc.steering_history.end() - history_size);
    }
}

}  // namespace

void InitMPC(const MPCParams& params) {
    g_mpc = MPCContext{};
    g_mpc.params = params;
    const double frequency =
        g_mpc.params.control_frequency > 0.0 ? g_mpc.params.control_frequency : 20.0;
    g_mpc.stuck_timeout_cycles =
        CyclesFromSeconds(g_mpc.params.stuck_timeout, frequency);
    g_mpc.recovery_duration_cycles =
        CyclesFromSeconds(g_mpc.params.stuck_recovery_duration, frequency);
}

void ResetMPCOnFailSafe() {
    g_mpc.braking_geometry.clear();
    g_mpc.warm_start.clear();
    g_mpc.steering_history.clear();
    g_mpc.last_control = MPCControl{};
    g_mpc.stuck_cycles = 0;
    g_mpc.recovery_cycles_left = 0;
}

void ResetMPCForPlannerStop(double published_steering) {
    if (!std::isfinite(published_steering)) {
        ResetMPCOnFailSafe();
        return;
    }
    // A valid planner STOP invalidates the optimization seed, not commands
    // still travelling through the steering actuator. Record this published
    // command so a brief STOP -> KEEP transition predicts the real delay.
    g_mpc.warm_start.clear();
    g_mpc.last_control = MPCControl{};
    g_mpc.last_control.delta = std::clamp(published_steering,
        -g_mpc.params.steering_max, g_mpc.params.steering_max);
    g_mpc.stuck_cycles = 0;
    g_mpc.recovery_cycles_left = 0;
    RecordPublishedSteering(g_mpc.last_control.delta);
}

void ResetMPCForHandoff(double steering,double acceleration) {
    ResetMPCOnFailSafe();
    if(!std::isfinite(steering+acceleration))return;
    g_mpc.last_control.delta=std::clamp(steering,-g_mpc.params.steering_max,g_mpc.params.steering_max);
    g_mpc.last_control.accel=std::clamp(acceleration,g_mpc.params.accel_min,g_mpc.params.accel_max);
    RecordPublishedSteering(g_mpc.last_control.delta);
}

MPCOutput MPCProcess(
    const MPCState& ego_state, const ReferencePath& reference, double acceleration_limit, double acceleration_rise_rate) {
    MPCOutput output;

    // UDP MPC와 같은 정지 데드락 복구. 정상 STOP은 호출자가 이미 걸러낸
    // 뒤이며, 전진 속도와 실제 경로 진행량이 함께 확인될 때만 launch
    // assist를 허용한다.
    double max_reference_speed = 0.0;
    for (double speed : reference.v_ref) {
        max_reference_speed = std::max(max_reference_speed, speed);
    }
    double path_progress = 0.0;
    if (reference.size() >= 2) {
        path_progress = std::hypot(
            reference.x_ref.back() - reference.x_ref.front(),
            reference.y_ref.back() - reference.y_ref.front());
    }
    const bool forward_intent =
        max_reference_speed > g_mpc.params.stuck_target_speed &&
        path_progress > g_mpc.params.stuck_min_path_progress;
    const bool ego_stopped =
        std::fabs(ego_state.vx) < g_mpc.params.stuck_ego_speed;

    if (!forward_intent || !ego_stopped) {
        g_mpc.stuck_cycles = 0;
        g_mpc.recovery_cycles_left = 0;
    } else if (g_mpc.recovery_cycles_left == 0 &&
               ++g_mpc.stuck_cycles >= g_mpc.stuck_timeout_cycles) {
        g_mpc.warm_start.clear();
        g_mpc.last_control = MPCControl{};
        g_mpc.recovery_cycles_left = g_mpc.recovery_duration_cycles;
        g_mpc.stuck_cycles = 0;
        output.stuck_event = StuckEvent::RECOVERY_STARTED;
        output.stuck_event_ego_v = ego_state.vx;
        output.stuck_event_max_ref_v = max_reference_speed;
        output.stuck_event_path_progress = path_progress;
    }

    auto solve_params = g_mpc.params;
    solve_params.acceleration_rise_rate = acceleration_rise_rate;
    if (std::isfinite(acceleration_limit))
        solve_params.accel_max = std::clamp(acceleration_limit,solve_params.accel_min,solve_params.accel_max);
    const MPCControl optimization_previous_control = g_mpc.last_control;
    const MPCResult result = solveMPC(
        ego_state, reference, optimization_previous_control, g_mpc.warm_start,
        solve_params, g_mpc.steering_history);
    if (!result.success ||
        result.controls.size() != static_cast<std::size_t>(g_mpc.params.horizon) ||
        !std::isfinite(result.control.delta) ||
        !std::isfinite(result.control.accel)) {
        output.success = false;
        output.failure_reason = "solver failure";
        return output;
    }

    g_mpc.warm_start = result.controls;
    g_mpc.last_control = result.control;
    output.cost = computeCostBreakdown(
        result.predicted_states, result.controls, reference,
        optimization_previous_control, solve_params);
    output.first_reference_curvature = reference.k_ref.empty()
        ? 0.0 : reference.k_ref.front();
    output.first_feedforward_steering = SteeringFeedforward(
        output.first_reference_curvature, g_mpc.params.wheelbase,
        g_mpc.params.steering_max, g_mpc.params.steering_curvature_gain);
    output.first_reference_speed = reference.v_ref.empty()
        ? 0.0 : reference.v_ref.front();
    output.first_reference_acceleration = reference.a_ref.empty()
        ? 0.0 : reference.a_ref.front();
    output.first_used_reference_acceleration = clip(
        output.first_reference_acceleration,
        solve_params.accel_min, solve_params.accel_max);
    output.current_speed = ego_state.vx;
    output.first_speed_error =
        output.current_speed - output.first_reference_speed;
    MPCControl output_control = result.control;
    if (g_mpc.recovery_cycles_left > 0 && forward_intent && ego_stopped) {
        output_control.accel =
            std::min(solve_params.accel_max,
                     std::max(output_control.accel, g_mpc.params.stuck_launch_accel));
        // 다음 cycle의 rate cost가 실제 출력한 가속과 이어지도록 동기화한다.
        g_mpc.last_control.accel = output_control.accel;
        --g_mpc.recovery_cycles_left;
        if (g_mpc.recovery_cycles_left == 0) {
            output.stuck_event = StuckEvent::RECOVERY_COMPLETE;
        }
    }

    if (ValidateReference(reference, g_mpc.params).ok())
        g_mpc.braking_geometry = reference;
    output.success = true;
    output.control = output_control;
    RecordPublishedSteering(output_control.delta);
    output.first_acceleration_residual = output.control.accel -
        output.first_used_reference_acceleration;
    return output;
}


const MPCParams& GetMPCParams() { return g_mpc.params; }

bool MPCPlannerStopSteering(const MPCState& ego, double brake_pedal,
                            double& steering) {
    if (ego.vx < g_mpc.params.stuck_ego_speed) {
        g_mpc.braking_geometry.clear();
        return false;
    }
    const double decel = BrakePedalDeceleration(brake_pedal, g_mpc.params);
    const auto reference = MakeBrakingReference(g_mpc.braking_geometry,
        ego, decel, g_mpc.params);
    if (reference.empty()) return false;
    MPCParams braking_params = g_mpc.params;
    // The runtime publishes the existing fixed brake pedal, so optimization
    // must predict that same deceleration and may never ask for acceleration.
    braking_params.accel_min = -decel;
    braking_params.accel_max = -decel;
    const auto result = solveMPC(ego, reference, g_mpc.last_control, {},
        braking_params, g_mpc.steering_history);
    if (!result.success || !std::isfinite(result.control.delta)) return false;
    steering = result.control.delta;
    return true;
}
