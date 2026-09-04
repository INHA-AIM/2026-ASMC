#pragma once

// BehaviorCommand 한 건을 하나의 maneuver transaction으로 관리한다.
//
// ===== behavior_command_adapter와의 경계 =====
//
//   behavior_command_adapter   stateless. 메시지 하나를 보고 판단한다
//                              (enum 매핑 / validation / fingerprint / 변환)
//   command_execution_manager  stateful. 명령들 사이의 관계를 관리한다
//                              (transaction / terminal latch / preemption)
//
// 이 경계를 지키지 않으면 adapter가 곧 state machine이 되어버린다. 상태를
// 갖는 판단은 전부 이쪽에 둔다.
// 계약: docs/fsm-frenet-interface-v2.md 12.8절
//
// ===== 왜 transaction인가 =====
//
// command_id는 피드백용 일련번호가 아니라 maneuver transaction의 identity다.
// 한 기동이 시작해서 완료·실패·취소로 끝날 때까지 같은 ID를 유지하며,
// PREPARE -> EXECUTE는 같은 transaction 안의 정상 전이다(계약 4절, 5절).

#include <cstdint>
#include <string>

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <ros/time.h>

#include "ros/behavior_command_adapter.hpp"

namespace frenet_planner_ros {

// runtime 상태에서 오는 실패. BehaviorCommand의 결함(CommandValidationCode)과
// 성격이 다르므로 분리한다.
//
// ===== ROS 회신은 아직 보류 =====
//
// FrenetFeedback.msg에 대응 result_code가 없다.
//   INPUT_DATA_STALE=10
//   MANEUVER_NOT_IMPLEMENTED=11
// 두 상수는 D3-0에서 정책으로 확정했지만(계약 11.1절) 메시지는 FSM 담당자
// 소유이므로(계약 1.4절) 아직 원본에 없다.
//
// 기존 코드로 거짓 매핑하지 않는다:
//   RESULT_NONE      "오류 없음"인데 FAILED와 같이 나가면 모순이다
//   INVALID_COMMAND  명령은 정상인데 FSM 잘못이라고 거짓 보고가 된다
//   STALE_COMMAND    valid_until 만료가 아니므로 FSM이 재발행으로 회복 시도
//
// 따라서 이 enum은 내부 판단용으로만 쓰고, ROS 직렬화는 메시지가 갱신된 뒤
// 별도로 붙인다. 그때까지 result_code는 RESULT_NONE으로 남고 사유는
// result_detail 문자열에만 나타난다.
enum class RuntimeFailure {
    NONE = 0,
    // Ego / Object 입력이 freshness 검사를 통과하지 못했다. 명령은 정상이다.
    INPUT_DATA_STALE,
    // 명령은 정상이지만 해당 maneuver가 아직 구현되지 않았다. 개발 기간
    // 한정이며 D10 cutover 이후 발생하면 그 자체가 버그다(계약 11.4절).
    MANEUVER_NOT_IMPLEMENTED,
};

const char* RuntimeFailureName(RuntimeFailure failure);

// 명령 하나를 받아들인 결과.
struct CommandAcceptance {
    // 명령의 형식과 필수 필드가 정상인지. 궤적 생성 성공 여부와 무관하다.
    // TOO_LATE_TO_CANCEL을 회신할 때도 true다 - 명령은 정상적으로
    // 수신·해석되었고 취소가 불가능한 시점이라는 뜻이다(계약 7.1절).
    bool command_accepted = false;

    // planner를 호출해도 되는지. false이면 이 사이클에 planner를 호출하지
    // 않는다. 미구현 maneuver를 ResolveManeuver에 넣으면 default 분기로
    // velocity keeping 궤적이 나오는데, 이는 FSM이 요청하지 않은 기동이
    // 성공처럼 보이는 위험한 상태다(계약 11.4절).
    bool should_plan = false;

    // FrenetFeedback.execution_state로 그대로 보고할 값.
    uint8_t execution_state = asmc_msgs::FrenetFeedback::IDLE;

    // FrenetFeedback.result_code. RuntimeFailure는 위 주석대로 아직
    // 직렬화하지 않으므로 RESULT_NONE으로 남는다.
    uint8_t result_code = asmc_msgs::FrenetFeedback::RESULT_NONE;

    // 사람이 읽는 사유. FrenetFeedback.result_detail로 전달한다.
    std::string detail;
};

class CommandExecutionManager {
 public:
    // 어떤 maneuver가 구현되어 있는지 알려주는 콜백 형태 대신, 단계 진행에
    // 따라 열리는 목록을 명시적으로 설정한다. D4는 KEEP/STOP/EMERGENCY_STOP,
    // D5부터 FOLLOW가 추가되는 식이다(계약 11.4절).
    //
    // 기본값은 아무것도 열지 않은 상태다 - 열지 않은 maneuver를 조용히
    // 실행하는 것보다 MANEUVER_NOT_IMPLEMENTED로 거부하는 편이 안전하다.
    void SetImplementedManeuver(uint8_t maneuver_type, bool implemented);
    bool IsManeuverImplemented(uint8_t maneuver_type) const;

    // 검증을 통과한 명령을 transaction 규칙에 따라 받아들인다.
    //
    // validation은 호출자가 먼저 수행한다(ValidateBehaviorCommand). 이
    // 함수는 명령들 사이의 관계만 판단한다:
    //   새 command_id            -> 새 transaction
    //   같은 ID + 같은 payload   -> 같은 transaction 계속
    //   같은 ID + 다른 payload   -> 거부, 기존 transaction 보존
    //   CANCEL                   -> 상태에 따라 수락 또는 TOO_LATE_TO_CANCEL
    //   COMMITTED 중 다른 ID     -> 거부 (EMERGENCY_STOP만 예외)
    //   terminal latch 중        -> 새 ID까지 유지
    CommandAcceptance Accept(const asmc_msgs::BehaviorCommand& message);

    // 입력 데이터 stale을 기록한다. 명령 자체는 정상이므로 transaction을
    // 버리지 않고, 이 사이클의 planning만 막는다.
    void NotifyInputDataStale();

    // planner 실행 결과를 반영한다. D3-3에서는 상태 전이만 다루고,
    // 궤적 캐시 소유권은 D4에서 추가한다(계약 8.4절).
    void NotifyPlanSucceeded();

    // planner 실행 실패를 알린다.
    //
    // terminal=true이면 상태와 함께 **실패 사유도 latch**한다. 사유를 latch
    // 하지 않으면 다음 사이클에 같은 command_id가 다시 들어올 때 latch
    // 분기가 result_code 기본값(RESULT_NONE)으로 회신해, FSM 입장에서는
    //   cycle 1: FAILED + NOT_INITIALIZED
    //   cycle 2: FAILED + RESULT_NONE   <- 사유가 사라진다
    // 가 된다. 계약 6절은 새 command_id까지 terminal 상태를 유지하라고
    // 하므로 사유도 함께 유지해야 한다.
    void NotifyPlanFailed(bool terminal,
                          uint8_t result_code =
                              asmc_msgs::FrenetFeedback::RESULT_NONE,
                          const std::string& detail = std::string());

    // EXECUTE 허가를 받은 뒤 planner가 현재 상황으로 재검증에 성공해 궤적을
    // COMMIT했다. READY -> COMMITTED 전이다.
    //
    // 이 시점부터 일반 명령으로는 전환할 수 없다(IsNonPreemptible). 실제
    // COMMIT 판정은 planner의 재검증 결과이므로(계약 5.3절) manager가 스스로
    // 올리지 않고 호출자가 알려준다. D6~D8에서 planner 결과에 연결한다.
    void NotifyCommitted();

    // maneuver를 기하학적으로 완료했다. terminal latch가 걸린다.
    void NotifyManeuverCompleted();

    // 현재 상태 조회 (피드백 작성용).
    bool has_active_command() const { return has_active_command_; }
    uint32_t command_id() const { return command_id_; }
    uint8_t maneuver_type() const { return maneuver_type_; }
    uint8_t execution_state() const { return execution_state_; }
    RuntimeFailure runtime_failure() const { return runtime_failure_; }
    bool terminal_latched() const { return terminal_latched_; }
    const CommandFingerprint& fingerprint() const { return fingerprint_; }

    // latch된 실패 사유. terminal_latched()가 false이면 RESULT_NONE / "".
    uint8_t terminal_result_code() const { return terminal_result_code_; }
    const std::string& terminal_result_detail() const {
        return terminal_result_detail_;
    }

    // 이 transaction이 정상 완료(COMPLETED)로 끝났는지. 실패 latch와 구분해야
    // 하는 이유는 궤적 출력 정책이 다르기 때문이다 - 계약 6.2절에 따라 latch
    // 중에도 궤적은 계속 나가야 하고, STOP COMPLETED이면 HOLD 궤적을
    // 유지해야 한다(미구현 maneuver의 SAFE_STOP과 다르다).
    bool completed_normally() const {
        return terminal_latched_ &&
               execution_state_ == asmc_msgs::FrenetFeedback::COMPLETED;
    }

 private:
    // 지금 다른 명령으로 전환하면 위험한 시점인지.
    //
    // 보호 대상은 **handshake maneuver가 실제로 COMMIT된 이후**다.
    // KEEP/FOLLOW/STOP은 transaction 시작과 동시에 EXECUTING이 되지만, 이들
    // 사이의 전환은 정상 주행의 일부다(KEEP -> STOP, FOLLOW -> AVOID 등).
    // 모든 EXECUTING을 잠그면 FSM이 어떤 모드 전환도 할 수 없게 된다.
    //
    // 계약 7.2절이 막으려는 것은 "교차로 한가운데서 기동을 바꾸는 것",
    // 즉 횡이동·합류가 이미 시작된 상태의 중단이다.
    bool IsNonPreemptible() const;

    // CANCEL을 거부해야 하는 시점인지. CANCEL은 handshake maneuver만 보내므로
    // IsNonPreemptible()과 같은 판정을 쓴다.
    bool CancelRejected() const { return IsNonPreemptible(); }
    // 새 transaction을 시작한다.
    void BeginTransaction(const asmc_msgs::BehaviorCommand& message);

    bool has_active_command_ = false;
    uint32_t command_id_ = 0;
    CommandFingerprint fingerprint_;
    uint8_t maneuver_type_ = 0;
    uint8_t execution_state_ = asmc_msgs::FrenetFeedback::IDLE;

    // COMPLETED / FAILED에 도달하면 새 command_id가 올 때까지 유지한다.
    // FSM이 종료 피드백 한 프레임을 놓치는 것을 막는다(계약 6절).
    bool terminal_latched_ = false;

    RuntimeFailure runtime_failure_ = RuntimeFailure::NONE;

    // terminal 상태와 함께 latch되는 실패 사유(위 NotifyPlanFailed 주석 참고).
    uint8_t terminal_result_code_ = asmc_msgs::FrenetFeedback::RESULT_NONE;
    std::string terminal_result_detail_;

    // 구현된 maneuver 비트마스크. maneuver_type은 0~6이다.
    uint8_t implemented_mask_ = 0;
};

}  // namespace frenet_planner_ros
