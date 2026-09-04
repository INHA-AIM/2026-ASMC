#include "ros/command_execution_manager.hpp"

namespace frenet_planner_ros {

namespace {

using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;

// handshake(PREPARE/EXECUTE/CANCEL)를 쓰는 maneuver. 나머지는 항상 EXECUTE로
// 들어오며 준비 단계가 없다.
bool UsesHandshake(uint8_t maneuver_type) {
    return maneuver_type == Command::MERGE ||
           maneuver_type == Command::LANE_CHANGE ||
           maneuver_type == Command::AVOID;
}

}  // namespace

const char* RuntimeFailureName(RuntimeFailure failure) {
    switch (failure) {
        case RuntimeFailure::NONE: return "NONE";
        case RuntimeFailure::INPUT_DATA_STALE: return "INPUT_DATA_STALE";
        case RuntimeFailure::MANEUVER_NOT_IMPLEMENTED:
            return "MANEUVER_NOT_IMPLEMENTED";
    }
    return "UNKNOWN_RUNTIME_FAILURE";
}

void CommandExecutionManager::SetImplementedManeuver(uint8_t maneuver_type,
                                                     bool implemented) {
    if (maneuver_type > Command::EMERGENCY_STOP) return;
    const uint8_t bit = static_cast<uint8_t>(1u << maneuver_type);
    if (implemented) implemented_mask_ |= bit;
    else implemented_mask_ = static_cast<uint8_t>(implemented_mask_ & ~bit);
}

bool CommandExecutionManager::IsManeuverImplemented(uint8_t maneuver_type) const {
    if (maneuver_type > Command::EMERGENCY_STOP) return false;
    return (implemented_mask_ & static_cast<uint8_t>(1u << maneuver_type)) != 0;
}

bool CommandExecutionManager::IsNonPreemptible() const {
    // KEEP / FOLLOW / STOP / EMERGENCY_STOP은 준비 단계가 없어 transaction
    // 시작과 동시에 EXECUTING이다. 이 상태를 잠그면 FSM이 정상적인 모드
    // 전환(KEEP -> STOP, FOLLOW -> AVOID 등)조차 할 수 없다.
    //
    // 실제로 보호해야 하는 것은 횡이동·합류가 COMMIT된 이후다.
    if (!UsesHandshake(maneuver_type_)) return false;
    return execution_state_ == Feedback::COMMITTED ||
           execution_state_ == Feedback::EXECUTING;
}

void CommandExecutionManager::BeginTransaction(const Command& message) {
    has_active_command_ = true;
    command_id_ = message.command_id;
    fingerprint_ = MakeCommandFingerprint(message);
    maneuver_type_ = message.maneuver_type;
    terminal_latched_ = false;
    terminal_result_code_ = Feedback::RESULT_NONE;
    terminal_result_detail_.clear();
    runtime_failure_ = RuntimeFailure::NONE;
    // handshake를 쓰는 maneuver는 준비 단계에서 시작한다. 나머지는 바로
    // 실행으로 들어간다(계약 5.5절).
    execution_state_ = UsesHandshake(message.maneuver_type)
        ? Feedback::PREPARING : Feedback::EXECUTING;
}

CommandAcceptance CommandExecutionManager::Accept(const Command& message) {
    CommandAcceptance acceptance;

    const bool is_emergency = message.maneuver_type == Command::EMERGENCY_STOP;
    const bool same_id = has_active_command_ && command_id_ == message.command_id;

    // ---- EMERGENCY_STOP은 언제든 preempt한다 (계약 7.3절) ----------------
    // execution_state와 무관하다. COMMITTED된 MERGE 궤적도 폐기한다.
    if (is_emergency && !same_id) {
        BeginTransaction(message);
        acceptance.command_accepted = true;
        acceptance.detail = "emergency stop preempted the active command";
        // 아래 구현 상태 확인으로 이어진다.
    } else if (!has_active_command_) {
        BeginTransaction(message);
        acceptance.command_accepted = true;
    } else if (!same_id) {
        // ---- 다른 command_id ---------------------------------------------
        if (IsNonPreemptible()) {
            // 교차로 한가운데서 기동을 바꾸는 것이 가장 위험하다. 기존
            // transaction을 그대로 유지한다(계약 7.2절).
            acceptance.command_accepted = true;
            acceptance.should_plan = true;
            acceptance.execution_state = execution_state_;
            acceptance.result_code = Feedback::TOO_LATE_TO_CANCEL;
            acceptance.detail =
                "committed maneuver cannot be replaced by a new command";
            return acceptance;
        }
        // terminal latch는 새 ID에서만 해제된다(계약 6.3절).
        BeginTransaction(message);
        acceptance.command_accepted = true;
    } else {
        // ---- 같은 command_id --------------------------------------------
        const CommandFingerprint incoming = MakeCommandFingerprint(message);
        if (!SameCommandFingerprint(fingerprint_, incoming)) {
            // 같은 ID로 목표가 바뀌면 이 transaction이 무엇이었는지가
            // 무너진다. 거부하되 기존 상태는 보존한다(계약 4.1절).
            acceptance.command_accepted = false;
            acceptance.should_plan = IsNonPreemptible();
            acceptance.execution_state = execution_state_;
            acceptance.result_code = Feedback::INVALID_COMMAND;
            acceptance.detail =
                "command payload changed without a new command_id";
            return acceptance;
        }

        // terminal latch 중이면 같은 ID 재발행에도 상태를 유지한다.
        if (terminal_latched_) {
            acceptance.command_accepted = true;
            acceptance.should_plan = false;
            acceptance.execution_state = execution_state_;
            // 상태와 함께 사유도 유지한다. 그러지 않으면 두 번째 사이클부터
            // result_code가 RESULT_NONE으로 떨어져 FSM이 원인을 잃는다.
            acceptance.result_code = terminal_result_code_;
            acceptance.detail = terminal_result_detail_.empty()
                ? std::string("terminal state latched until a new command_id")
                : terminal_result_detail_;
            return acceptance;
        }

        // CANCEL 처리 (계약 7.1절).
        if (message.execution_request == Command::CANCEL) {
            if (IsNonPreemptible()) {
                acceptance.command_accepted = true;
                acceptance.should_plan = true;
                acceptance.execution_state = execution_state_;
                acceptance.result_code = Feedback::TOO_LATE_TO_CANCEL;
                acceptance.detail = "cancel rejected after commit";
                return acceptance;
            }
            // PREPARING / WAITING / READY에서는 취소가 성립한다.
            has_active_command_ = false;
            command_id_ = 0;
            fingerprint_ = CommandFingerprint{};
            maneuver_type_ = 0;
            execution_state_ = Feedback::IDLE;
            terminal_latched_ = false;
            terminal_result_code_ = Feedback::RESULT_NONE;
            terminal_result_detail_.clear();
            runtime_failure_ = RuntimeFailure::NONE;
            acceptance.command_accepted = true;
            acceptance.should_plan = false;
            acceptance.execution_state = Feedback::IDLE;
            acceptance.detail = "command cancelled";
            return acceptance;
        }

        // EXECUTE는 COMMIT 허가다. 실제 COMMIT 승격은 planner의 재검증
        // 결과에 달려 있으므로(계약 5.3절) 여기서 상태를 바꾸지 않는다.
        // D6~D8에서 planner 결과를 받아 READY -> COMMITTED로 올린다.
        acceptance.command_accepted = true;
    }

    // ---- 구현 상태 확인 (계약 11.4절) ------------------------------------
    // 여기까지 왔으면 명령 자체는 받아들였다. 다만 아직 열리지 않은
    // maneuver라면 planner를 호출하지 않는다.
    if (!IsManeuverImplemented(message.maneuver_type)) {
        runtime_failure_ = RuntimeFailure::MANEUVER_NOT_IMPLEMENTED;
        execution_state_ = Feedback::FAILED;
        terminal_latched_ = true;
        // FrenetFeedback.msg에 MANEUVER_NOT_IMPLEMENTED가 없으므로
        // result_code는 RESULT_NONE으로 두고 사유는 detail로만 남긴다.
        terminal_result_code_ = Feedback::RESULT_NONE;
        terminal_result_detail_ = "maneuver not implemented yet";
        acceptance.command_accepted = true;
        acceptance.should_plan = false;
        acceptance.execution_state = Feedback::FAILED;
        // result_code는 RESULT_NONE으로 남는다. FrenetFeedback.msg에
        // MANEUVER_NOT_IMPLEMENTED가 없고, 기존 코드로 거짓 매핑하지
        // 않는다(RuntimeFailure 주석 참고). 사유는 detail로만 전달한다.
        acceptance.detail = "maneuver not implemented yet";
        return acceptance;
    }

    runtime_failure_ = RuntimeFailure::NONE;
    acceptance.should_plan = true;
    acceptance.execution_state = execution_state_;
    return acceptance;
}

void CommandExecutionManager::NotifyInputDataStale() {
    // 명령은 정상이므로 transaction을 버리지 않는다. 이 사이클의 planning만
    // 막히고, 입력이 회복되면 같은 transaction으로 계속한다.
    runtime_failure_ = RuntimeFailure::INPUT_DATA_STALE;
}

void CommandExecutionManager::NotifyPlanSucceeded() {
    if (!has_active_command_ || terminal_latched_) return;
    runtime_failure_ = RuntimeFailure::NONE;
    // PREPARING에서 유효 후보가 나왔으면 EXECUTE를 받을 준비가 된 것이다.
    // 실제 gap/안전성 판정은 D6~D8에서 planner 결과로 세분화한다.
    if (execution_state_ == Feedback::PREPARING ||
        execution_state_ == Feedback::WAITING) {
        execution_state_ = Feedback::READY;
    }
}

void CommandExecutionManager::NotifyPlanFailed(bool terminal,
                                               uint8_t result_code,
                                               const std::string& detail) {
    if (!has_active_command_ || terminal_latched_) return;
    if (terminal) {
        execution_state_ = Feedback::FAILED;
        terminal_latched_ = true;
        terminal_result_code_ = result_code;
        terminal_result_detail_ = detail;
        return;
    }
    // 일시적 실패는 준비 단계로 되돌린다. COMMITTED 이후에는 잠긴 궤적을
    // 계속 실행해야 하므로 상태를 낮추지 않는다.
    if (!IsNonPreemptible() && UsesHandshake(maneuver_type_)) {
        execution_state_ = Feedback::WAITING;
    }
}

bool CommandExecutionManager::NotifyCommitted() {
    if (!has_active_command_ || terminal_latched_) return false;
    // handshake를 쓰지 않는 maneuver는 COMMIT 개념이 없다. 이미 EXECUTING이다.
    if (!UsesHandshake(maneuver_type_)) return false;
    // 계약 5절의 정상 경로는 READY -> COMMITTED뿐이다(헤더 주석 참고).
    if (execution_state_ != Feedback::READY) return false;
    execution_state_ = Feedback::COMMITTED;
    runtime_failure_ = RuntimeFailure::NONE;
    return true;
}

bool CommandExecutionManager::NotifyExecutionStarted() {
    if (!has_active_command_ || terminal_latched_) return false;
    if (!UsesHandshake(maneuver_type_)) return false;
    if (execution_state_ != Feedback::COMMITTED) return false;
    execution_state_ = Feedback::EXECUTING;
    runtime_failure_ = RuntimeFailure::NONE;
    return true;
}

void CommandExecutionManager::NotifyManeuverCompleted() {
    if (!has_active_command_) return;
    execution_state_ = Feedback::COMPLETED;
    terminal_latched_ = true;
    // 정상 완료는 오류가 아니므로 사유를 남기지 않는다. completed_normally()가
    // true가 되어 호출자가 궤적 출력 정책을 구분할 수 있다(계약 6.2절).
    terminal_result_code_ = Feedback::RESULT_NONE;
    terminal_result_detail_.clear();
    runtime_failure_ = RuntimeFailure::NONE;
}

}  // namespace frenet_planner_ros
