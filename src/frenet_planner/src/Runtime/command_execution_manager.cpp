#include "command_execution_manager.hpp"

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

const char* PlanningTargetName(PlanningTarget target) {
    switch (target) {
        case PlanningTarget::NONE: return "NONE";
        case PlanningTarget::INCOMING_COMMAND: return "INCOMING_COMMAND";
        case PlanningTarget::ACTIVE_TRANSACTION: return "ACTIVE_TRANSACTION";
    }
    return "UNKNOWN_PLANNING_TARGET";
}

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
    // 이 명령이 이 transaction의 원본이다. 거부된 명령으로는 갱신하지 않으므로
    // planning_target == ACTIVE_TRANSACTION인 사이클에 호출자가 그대로 쓴다.
    active_command_ = message;
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

CommandAcceptance CommandExecutionManager::Accept(
    const Command& message, bool allow_committed_tracking_preemption) {
    CommandAcceptance acceptance;

    const bool is_emergency = message.maneuver_type == Command::EMERGENCY_STOP;
    const bool same_id = has_active_command_ && command_id_ == message.command_id;
    // Behavior는 STOP과 AVOID가 동시에 활성화되면 global-path s를 비교해
    // 더 앞선 제약만 새 command_id로 보낸다. 그 STOP까지 COMMITTED AVOID
    // 잠금으로 거부하면 빨간불 정지선을 지나칠 수 있으므로, 횡기동 중에도
    // 계획 정지는 새 transaction으로 전환할 수 있게 한다. EMERGENCY_STOP의
    // 무조건 선점과 달리 이 경로는 upstream 거리 중재를 거친 STOP에 한한다.
    const bool planned_stop_preemption =
        !same_id && has_active_command_ &&
        message.maneuver_type == Command::STOP &&
        maneuver_type_ == Command::AVOID;

    // ---- EMERGENCY_STOP은 언제든 preempt한다 (계약 7.3절) ----------------
    // execution_state와 무관하다. COMMITTED된 MERGE 궤적도 폐기한다.
    if ((is_emergency && !same_id) || planned_stop_preemption) {
        BeginTransaction(message);
        acceptance.transaction_started = true;
        acceptance.command_accepted = true;
        acceptance.detail = is_emergency
            ? "emergency stop preempted the active command"
            : "planned stop preempted the committed avoid command";
        // 아래 구현 상태 확인으로 이어진다.
    } else if (!has_active_command_ &&
               message.execution_request == Command::CANCEL) {
        // ---- 취소할 transaction이 없다 (D6-4b) --------------------------
        //
        // CANCEL은 **기존 transaction을 끝내는 요청**이지 새 기동을 여는
        // 요청이 아니다. 그런데 여기서 BeginTransaction을 하면 CANCEL이
        // handshake maneuver의 PREPARING을 새로 만들고, node의 AVOID 경로는
        // PREPARE/EXECUTE만 받으므로 그 명령이 일반 Plan()으로 흘러
        // INVALID_COMMAND + FAILED latch가 된다.
        //
        // 실측(D6-4b): READY에서 CANCEL이 성공해 IDLE이 된 뒤 FSM이 같은 ID의
        // CANCEL을 한 번 더 발행하자
        //   cycle 1 : IDLE      (정상 취소)
        //   cycle 2+: FAILED / INVALID_COMMAND
        // 가 나왔다. 이미 취소된 것을 다시 취소했을 뿐인데 terminal 실패가
        // latch되어, FSM은 새 command_id를 쓰기 전까지 회복할 수 없다.
        //
        // 계약 7.1절은 COMPLETED/FAILED의 CANCEL을 "무시(이미 종료)"로 두고
        // 있다. 활성 transaction이 아예 없는 경우도 같은 성격이므로 상태를
        // 만들지 않고 그대로 IDLE을 회신한다. 명령 자체는 정상이므로
        // command_accepted=true다(7.1절 마지막 문단과 같은 이유).
        acceptance.command_accepted = true;
        acceptance.planning_target = PlanningTarget::NONE;
        acceptance.execution_state = Feedback::IDLE;
        acceptance.detail = "no active transaction to cancel";
        return acceptance;
    } else if (!has_active_command_) {
        BeginTransaction(message);
        acceptance.transaction_started = true;
        acceptance.command_accepted = true;
    } else if (!same_id) {
        // ---- 다른 command_id ---------------------------------------------
        if (IsNonPreemptible() && !allow_committed_tracking_preemption) {
            // 교차로 한가운데서 기동을 바꾸는 것이 가장 위험하다. 기존
            // transaction을 그대로 유지한다(계약 7.2절).
            //
            // 계획 대상은 **거부한 새 명령이 아니라 활성 transaction**이다.
            // 여기서 새 명령을 계획하면 피드백은 회피 중이라고 하면서 MPC로는
            // 차선 복귀 궤적이 나간다(PlanningTarget 주석 참고).
            acceptance.command_accepted = true;
            acceptance.planning_target = PlanningTarget::ACTIVE_TRANSACTION;
            acceptance.execution_state = execution_state_;
            acceptance.result_code = Feedback::TOO_LATE_TO_CANCEL;
            acceptance.detail =
                "committed maneuver cannot be replaced by a new command";
            return acceptance;
        }
        // terminal latch는 새 ID에서만 해제된다(계약 6.3절). committed
        // TRACKING은 node가 core phase까지 확인한 경우에만 여기로 들어온다.
        // 아직 횡이동 전이므로 FSM의 최신 transaction으로 교체해도 기동을
        // 중간에서 끊는 문제가 없다.
        BeginTransaction(message);
        acceptance.transaction_started = true;
        acceptance.command_accepted = true;
    } else {
        // ---- 같은 command_id --------------------------------------------
        const CommandFingerprint incoming = MakeCommandFingerprint(message);
        if (!SameCommandFingerprint(fingerprint_, incoming)) {
            // 같은 ID로 목표가 바뀌면 이 transaction이 무엇이었는지가
            // 무너진다. 거부하되 기존 상태는 보존한다(계약 4.1절).
            acceptance.command_accepted = false;
            // 거부했으므로 이 명령을 계획하지 않는다. 진행 중인 기동이
            // 있으면 그것을 계속하고, 없으면 planner를 호출하지 않는다.
            acceptance.planning_target = IsNonPreemptible()
                ? PlanningTarget::ACTIVE_TRANSACTION : PlanningTarget::NONE;
            acceptance.execution_state = execution_state_;
            acceptance.result_code = Feedback::INVALID_COMMAND;
            acceptance.detail =
                "command payload changed without a new command_id";
            return acceptance;
        }

        // terminal latch 중이면 같은 ID 재발행에도 상태를 유지한다.
        if (terminal_latched_) {
            acceptance.command_accepted = true;
            acceptance.planning_target = PlanningTarget::NONE;
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
                // 취소를 거부했으므로 잠긴 기동을 계속한다. CANCEL 메시지
                // 자체를 계획 대상으로 쓰면 안 된다.
                acceptance.command_accepted = true;
                acceptance.planning_target =
                    PlanningTarget::ACTIVE_TRANSACTION;
                acceptance.execution_state = execution_state_;
                acceptance.result_code = Feedback::TOO_LATE_TO_CANCEL;
                acceptance.detail = "cancel rejected after commit";
                return acceptance;
            }
            // PREPARING / WAITING / READY에서는 취소가 성립한다.
            has_active_command_ = false;
            active_command_ = Command{};
            command_id_ = 0;
            fingerprint_ = CommandFingerprint{};
            maneuver_type_ = 0;
            execution_state_ = Feedback::IDLE;
            terminal_latched_ = false;
            terminal_result_code_ = Feedback::RESULT_NONE;
            terminal_result_detail_.clear();
            runtime_failure_ = RuntimeFailure::NONE;
            acceptance.command_accepted = true;
            acceptance.planning_target = PlanningTarget::NONE;
            acceptance.execution_state = Feedback::IDLE;
            acceptance.detail = "command cancelled";
            // 호출자가 core의 maneuver context도 비울 수 있게 알린다
            // (CommandAcceptance::transaction_cancelled 주석 참고).
            acceptance.transaction_cancelled = true;
            return acceptance;
        }

        // EXECUTE는 COMMIT 허가다. 실제 COMMIT 승격은 planner의 재검증
        // 결과에 달려 있으므로(계약 5.3절) 여기서 상태를 바꾸지 않는다.
        // D6~D8에서 planner 결과를 받아 READY -> COMMITTED로 올린다.
        //
        // 정상 refresh이므로 active_command_를 갱신한다. fingerprint가 같아도
        // execution_request(PREPARE -> EXECUTE)와 valid_until은 mutable
        // 필드이므로 최신 값을 들고 있어야 한다(계약 4.3절).
        active_command_ = message;
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
        acceptance.planning_target = PlanningTarget::NONE;
        acceptance.execution_state = Feedback::FAILED;
        // result_code는 RESULT_NONE으로 남는다. FrenetFeedback.msg에
        // MANEUVER_NOT_IMPLEMENTED가 없고, 기존 코드로 거짓 매핑하지
        // 않는다(RuntimeFailure 주석 참고). 사유는 detail로만 전달한다.
        acceptance.detail = "maneuver not implemented yet";
        return acceptance;
    }

    runtime_failure_ = RuntimeFailure::NONE;
    acceptance.planning_target = PlanningTarget::INCOMING_COMMAND;
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

bool CommandExecutionManager::NotifyManeuverCompleted() {
    if (!has_active_command_ || terminal_latched_) return false;
    // handshake든 아니든 완료 직전 상태는 EXECUTING이다(헤더 주석 참고).
    if (execution_state_ != Feedback::EXECUTING) return false;
    execution_state_ = Feedback::COMPLETED;
    terminal_latched_ = true;
    // 정상 완료는 오류가 아니므로 사유를 남기지 않는다. completed_normally()가
    // true가 되어 호출자가 궤적 출력 정책을 구분할 수 있다(계약 6.2절).
    terminal_result_code_ = Feedback::RESULT_NONE;
    terminal_result_detail_.clear();
    runtime_failure_ = RuntimeFailure::NONE;
    return true;
}

}  // namespace frenet_planner_ros
