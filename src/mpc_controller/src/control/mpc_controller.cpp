#include "control/mpc_controller.hpp"

#include <algorithm>
#include <cmath>

#include "solver/mpc_solver.hpp"

namespace {

int CyclesFromSeconds(double seconds, double frequency_hz) {
    return std::max(1, static_cast<int>(std::ceil(seconds * frequency_hz)));
}

}  // namespace

MpcController::MpcController(const MPCParams& params) : params_(params) {
    const double frequency =
        params_.control_frequency > 0.0 ? params_.control_frequency : 20.0;
    stuck_timeout_cycles_ =
        CyclesFromSeconds(params_.stuck_timeout, frequency);
    recovery_duration_cycles_ =
        CyclesFromSeconds(params_.stuck_recovery_duration, frequency);
}

void MpcController::ResetOnFailSafe() {
    warm_start_.clear();
    previous_steering_feedforward_.clear();
    last_control_ = MPCControl{};
    stuck_cycles_ = 0;
    recovery_cycles_left_ = 0;
}

MpcController::Output MpcController::ControlProcess(
    const MPCState& ego_state, const ReferencePath& reference) {
    Output output;

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
        max_reference_speed > params_.stuck_target_speed &&
        path_progress > params_.stuck_min_path_progress;
    const bool ego_stopped =
        std::fabs(ego_state.vx) < params_.stuck_ego_speed;

    if (!forward_intent || !ego_stopped) {
        stuck_cycles_ = 0;
        recovery_cycles_left_ = 0;
    } else if (recovery_cycles_left_ == 0 &&
               ++stuck_cycles_ >= stuck_timeout_cycles_) {
        warm_start_.clear();
        previous_steering_feedforward_.clear();
        last_control_ = MPCControl{};
        recovery_cycles_left_ = recovery_duration_cycles_;
        stuck_cycles_ = 0;
        output.stuck_event = StuckEvent::RECOVERY_STARTED;
        output.stuck_event_ego_v = ego_state.vx;
        output.stuck_event_max_ref_v = max_reference_speed;
        output.stuck_event_path_progress = path_progress;
    }

    // curve-tracking stability iteration 3의 [MPC-WARM] 진단 - solveMPC
    // 호출 전 "이번 rebase가 실제로 쓸 이전 상태"를 미리 캡처한다. solveMPC
    // 내부에서 같은 조건으로 rebase 여부를 다시 판정하므로, 여기서는
    // 진단만 재현하고 rebase 로직 자체를 중복 구현하지 않는다(단순 stage-0
    // 스냅샷일 뿐, 결과 delta에 영향을 주지 않는다).
    const bool warm_start_ready =
        static_cast<int>(warm_start_.size()) >= params_.horizon;
    const bool feedforward_ready =
        previous_steering_feedforward_.size() == warm_start_.size();
    if (warm_start_ready && feedforward_ready && !warm_start_.empty()) {
        const std::size_t shifted_index =
            std::min<std::size_t>(1, warm_start_.size() - 1);
        output.warm_raw0 = warm_start_[shifted_index].delta;
        output.prev_ff_shift0 = previous_steering_feedforward_[shifted_index];
        output.warm_residual0 = output.warm_raw0 - output.prev_ff_shift0;
        output.used_curvature_rebase =
            std::isfinite(output.warm_raw0) && std::isfinite(output.prev_ff_shift0);
    }

    const MPCResult result = solveMPC(
        ego_state, reference, last_control_, warm_start_,
        previous_steering_feedforward_, params_);
    if (!result.success ||
        result.controls.size() != static_cast<std::size_t>(params_.horizon) ||
        !std::isfinite(result.control.delta) ||
        !std::isfinite(result.control.accel)) {
        output.success = false;
        output.failure_reason = "solver failure";
        return output;
    }

    if (output.used_curvature_rebase && !result.steering_feedforward.empty()) {
        output.current_ff0 = result.steering_feedforward[0];
        output.warm_rebased0 = output.current_ff0 + output.warm_residual0;
    }
    output.warm_projected0 = result.initial_guess_delta0;

    warm_start_ = result.controls;
    previous_steering_feedforward_ = result.steering_feedforward;
    last_control_ = result.control;
    MPCControl output_control = result.control;
    if (recovery_cycles_left_ > 0 && forward_intent && ego_stopped) {
        output_control.accel =
            std::max(output_control.accel, params_.stuck_launch_accel);
        // 다음 cycle의 rate cost가 실제 출력한 가속과 이어지도록 동기화한다.
        last_control_.accel = output_control.accel;
        --recovery_cycles_left_;
        if (recovery_cycles_left_ == 0) {
            output.stuck_event = StuckEvent::RECOVERY_COMPLETE;
        }
    }

    output.success = true;
    output.control = output_control;
    return output;
}
