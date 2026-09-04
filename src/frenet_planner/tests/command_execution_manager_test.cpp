// D3-3: transaction / terminal latch / preemption 회귀.
//
// 이 테스트가 지키는 계약:
//   4절  command_id는 transaction identity이며 같은 ID의 payload 변경은 거부
//   5절  PREPARE -> EXECUTE는 같은 transaction
//   6절  COMPLETED / FAILED는 새 command_id까지 유지 (terminal latch)
//   7절  CANCEL은 COMMITTED 이후 거부, EMERGENCY_STOP만 언제든 preempt
//   11.4절 미구현 maneuver는 planner를 호출하지 않는다
//
// D6-0에서 handshake 상태 전이 강제를 추가했다:
//   PREPARING/WAITING -> READY -> COMMITTED -> EXECUTING만 허용하고,
//   그 밖의 승격 시도는 상태를 바꾸지 않고 false를 반환한다.

#include "ros/command_execution_manager.hpp"

#include <iostream>
#include <string>

namespace {

using frenet_planner_ros::CommandExecutionManager;
using frenet_planner_ros::RuntimeFailure;
using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

Command MakeKeep(uint32_t id) {
    Command message;
    message.header.frame_id = "map";
    message.command_id = id;
    message.maneuver_type = Command::KEEP;
    message.execution_request = Command::EXECUTE;
    message.direction = Command::DIRECTION_NONE;
    message.merge_type = Command::MERGE_NONE;
    message.desired_speed_mps = -1.0;
    message.primary_object_id = -1;
    message.preferred_front_object_id = -1;
    message.preferred_rear_object_id = -1;
    message.use_custom_follow_gap = false;
    message.desired_time_gap_s = -1.0;
    message.minimum_gap_m = -1.0;
    message.valid_until = ros::Time(1001.0);
    return message;
}

Command MakeLaneChange(uint32_t id, uint8_t request) {
    Command message = MakeKeep(id);
    message.maneuver_type = Command::LANE_CHANGE;
    message.execution_request = request;
    message.direction = Command::DIRECTION_LEFT;
    message.has_target_pose = true;
    message.target_pose_map.position.x = 100.0;
    message.target_pose_map.orientation.w = 1.0;
    return message;
}

Command MakeEmergencyStop(uint32_t id) {
    Command message = MakeKeep(id);
    message.maneuver_type = Command::EMERGENCY_STOP;
    return message;
}

// D4 시점의 구현 상태: KEEP / STOP / EMERGENCY_STOP만 열려 있다.
CommandExecutionManager MakeD4Manager() {
    CommandExecutionManager manager;
    manager.SetImplementedManeuver(Command::KEEP, true);
    manager.SetImplementedManeuver(Command::STOP, true);
    manager.SetImplementedManeuver(Command::EMERGENCY_STOP, true);
    return manager;
}

// 모든 maneuver가 열린 상태 (D8 이후).
CommandExecutionManager MakeFullManager() {
    CommandExecutionManager manager = MakeD4Manager();
    manager.SetImplementedManeuver(Command::FOLLOW, true);
    manager.SetImplementedManeuver(Command::AVOID, true);
    manager.SetImplementedManeuver(Command::LANE_CHANGE, true);
    manager.SetImplementedManeuver(Command::MERGE, true);
    return manager;
}

}  // namespace

int main() {
    // ================= 새 transaction =================
    {
        CommandExecutionManager manager = MakeD4Manager();
        const auto acceptance = manager.Accept(MakeKeep(10));
        Expect(acceptance.command_accepted && acceptance.should_plan &&
                   manager.command_id() == 10 &&
                   manager.execution_state() == Feedback::EXECUTING,
               "new command_id starts a transaction");
    }
    {
        // handshake maneuver는 PREPARING으로 시작한다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(11, Command::PREPARE));
        Expect(manager.execution_state() == Feedback::PREPARING,
               "handshake maneuver starts in PREPARING");
    }

    // ================= 같은 ID + 같은 payload =================
    {
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(10));
        const auto again = manager.Accept(MakeKeep(10));
        Expect(again.command_accepted && again.should_plan &&
                   manager.command_id() == 10,
               "same id with same payload continues the transaction");
    }

    // ================= 같은 ID + payload 변경 =================
    {
        CommandExecutionManager manager = MakeFullManager();
        Command prepare = MakeLaneChange(12, Command::PREPARE);
        manager.Accept(prepare);
        Command mutated = prepare;
        mutated.target_pose_map.position.x = 300.0;  // 목표 변경
        const auto acceptance = manager.Accept(mutated);
        Expect(!acceptance.command_accepted &&
                   acceptance.result_code == Feedback::INVALID_COMMAND &&
                   manager.command_id() == 12 &&
                   manager.execution_state() == Feedback::PREPARING,
               "same id with changed payload is rejected and state preserved");
    }

    // ================= PREPARE -> EXECUTE =================
    {
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(13, Command::PREPARE));
        const auto execute = manager.Accept(MakeLaneChange(13, Command::EXECUTE));
        Expect(execute.command_accepted && execute.should_plan &&
                   manager.command_id() == 13,
               "PREPARE -> EXECUTE stays in the same transaction");
    }

    // ================= CANCEL =================
    {
        // PREPARING에서는 취소가 성립한다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(14, Command::PREPARE));
        const auto cancel = manager.Accept(MakeLaneChange(14, Command::CANCEL));
        Expect(cancel.command_accepted && !cancel.should_plan &&
                   cancel.execution_state == Feedback::IDLE &&
                   !manager.has_active_command(),
               "CANCEL in PREPARING returns to IDLE");
    }
    {
        // READY에서도 취소가 성립한다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(15, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        Expect(manager.execution_state() == Feedback::READY,
               "successful PREPARE reaches READY");
        const auto cancel = manager.Accept(MakeLaneChange(15, Command::CANCEL));
        Expect(cancel.command_accepted &&
                   cancel.execution_state == Feedback::IDLE,
               "CANCEL in READY returns to IDLE");
    }
    {
        // COMMIT된 handshake maneuver의 CANCEL은 거부한다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(16, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyCommitted();
        const auto result =
            manager.Accept(MakeLaneChange(16, Command::CANCEL));
        Expect(result.command_accepted &&
                   result.result_code == Feedback::TOO_LATE_TO_CANCEL &&
                   result.should_plan &&
                   manager.execution_state() == Feedback::COMMITTED,
               "CANCEL after commit is TOO_LATE_TO_CANCEL");
    }

    // ================= 일반 모드 사이의 전환은 허용된다 =================
    // KEEP/FOLLOW/STOP은 준비 단계가 없어 transaction 시작과 동시에
    // EXECUTING이다. 이 상태를 잠그면 FSM이 정상 주행 중 어떤 모드 전환도
    // 할 수 없다. 계약 7.2절이 막으려는 것은 횡이동·합류가 COMMIT된
    // 이후의 중단이지, 모든 EXECUTING이 아니다.
    {
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(20));
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "KEEP is EXECUTING right away");
        Command stop = MakeKeep(21);
        stop.maneuver_type = Command::STOP;
        stop.has_target_pose = true;
        stop.target_pose_map.orientation.w = 1.0;
        const auto acceptance = manager.Accept(stop);
        Expect(acceptance.command_accepted && acceptance.should_plan &&
                   acceptance.result_code == Feedback::RESULT_NONE &&
                   manager.command_id() == 21 &&
                   manager.maneuver_type() == Command::STOP,
               "KEEP -> STOP transition is allowed");
    }
    {
        // 역방향도 허용된다.
        CommandExecutionManager manager = MakeD4Manager();
        Command stop = MakeKeep(22);
        stop.maneuver_type = Command::STOP;
        stop.has_target_pose = true;
        stop.target_pose_map.orientation.w = 1.0;
        manager.Accept(stop);
        const auto acceptance = manager.Accept(MakeKeep(23));
        Expect(acceptance.command_accepted && acceptance.should_plan &&
                   manager.command_id() == 23 &&
                   manager.maneuver_type() == Command::KEEP,
               "STOP -> KEEP transition is allowed");
    }
    {
        // FOLLOW -> AVOID 도 정상 전환이다.
        CommandExecutionManager manager = MakeFullManager();
        Command follow = MakeKeep(24);
        follow.maneuver_type = Command::FOLLOW;
        follow.primary_object_id = 7;
        manager.Accept(follow);
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "FOLLOW is EXECUTING right away");
        Command avoid = MakeKeep(25);
        avoid.maneuver_type = Command::AVOID;
        avoid.execution_request = Command::PREPARE;
        avoid.primary_object_id = 21;
        const auto acceptance = manager.Accept(avoid);
        Expect(acceptance.command_accepted && acceptance.should_plan &&
                   manager.command_id() == 25 &&
                   manager.execution_state() == Feedback::PREPARING,
               "FOLLOW -> AVOID transition is allowed");
    }

    // ================= COMMIT된 handshake maneuver는 보호된다 ============
    {
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(26, Command::PREPARE));
        manager.NotifyPlanSucceeded();               // -> READY
        // READY에서는 아직 전환이 허용된다.
        Expect(manager.execution_state() == Feedback::READY,
               "LANE_CHANGE reaches READY");
        const auto switch_at_ready = manager.Accept(MakeKeep(27));
        Expect(switch_at_ready.command_accepted && switch_at_ready.should_plan &&
                   manager.command_id() == 27,
               "LANE_CHANGE in READY can still be replaced");
    }
    {
        // COMMITTED 이후에는 일반 명령을 거부한다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(28, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyCommitted();
        Expect(manager.execution_state() == Feedback::COMMITTED,
               "EXECUTE approval commits the maneuver");
        const auto acceptance = manager.Accept(MakeKeep(29));
        Expect(acceptance.command_accepted &&
                   acceptance.result_code == Feedback::TOO_LATE_TO_CANCEL &&
                   acceptance.should_plan &&
                   manager.command_id() == 28 &&
                   manager.maneuver_type() == Command::LANE_CHANGE,
               "committed LANE_CHANGE rejects a normal new command");
    }
    {
        // COMMITTED 중에도 EMERGENCY_STOP은 통과한다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(30, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyCommitted();
        const auto acceptance = manager.Accept(MakeEmergencyStop(31));
        Expect(acceptance.command_accepted && acceptance.should_plan &&
                   manager.command_id() == 31 &&
                   manager.maneuver_type() == Command::EMERGENCY_STOP,
               "committed LANE_CHANGE yields to EMERGENCY_STOP");
    }

    // ================= EMERGENCY_STOP preemption =================
    {
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(30));  // -> EXECUTING
        const auto acceptance = manager.Accept(MakeEmergencyStop(31));
        Expect(acceptance.command_accepted && acceptance.should_plan &&
                   manager.command_id() == 31 &&
                   manager.maneuver_type() == Command::EMERGENCY_STOP,
               "EMERGENCY_STOP preempts an executing command");
    }
    {
        // terminal latch도 뚫는다.
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(32));
        manager.NotifyManeuverCompleted();
        Expect(manager.terminal_latched(), "completed command is latched");
        const auto acceptance = manager.Accept(MakeEmergencyStop(33));
        Expect(acceptance.command_accepted && manager.command_id() == 33 &&
                   !manager.terminal_latched(),
               "EMERGENCY_STOP clears a terminal latch");
    }

    // ================= terminal latch =================
    {
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(40));
        manager.NotifyManeuverCompleted();
        Expect(manager.execution_state() == Feedback::COMPLETED,
               "completion sets COMPLETED");
        // 같은 ID 재발행에도 유지된다.
        for (int i = 0; i < 3; ++i) {
            const auto acceptance = manager.Accept(MakeKeep(40));
            Expect(acceptance.command_accepted && !acceptance.should_plan &&
                       acceptance.execution_state == Feedback::COMPLETED,
                   "COMPLETED is latched on same-id refresh");
        }
        // 새 ID에서만 해제된다.
        const auto next = manager.Accept(MakeKeep(41));
        Expect(next.command_accepted && next.should_plan &&
                   manager.command_id() == 41 &&
                   manager.execution_state() == Feedback::EXECUTING &&
                   !manager.terminal_latched(),
               "new command_id clears the COMPLETED latch");
    }
    {
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(42));
        manager.NotifyPlanFailed(/*terminal=*/true);
        Expect(manager.execution_state() == Feedback::FAILED &&
                   manager.terminal_latched(),
               "terminal plan failure sets FAILED and latches");
        const auto again = manager.Accept(MakeKeep(42));
        Expect(again.command_accepted && !again.should_plan &&
                   again.execution_state == Feedback::FAILED,
               "FAILED is latched on same-id refresh");
        const auto next = manager.Accept(MakeKeep(43));
        Expect(next.should_plan && manager.execution_state() == Feedback::EXECUTING,
               "new command_id clears the FAILED latch");
    }
    {
        // 일시적 실패는 latch하지 않는다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(44, Command::PREPARE));
        manager.NotifyPlanFailed(/*terminal=*/false);
        Expect(!manager.terminal_latched() &&
                   manager.execution_state() == Feedback::WAITING,
               "transient failure moves to WAITING without latching");
        manager.NotifyPlanSucceeded();
        Expect(manager.execution_state() == Feedback::READY,
               "recovery from WAITING reaches READY");
    }

    // ================= 미구현 maneuver =================
    {
        // D4 시점에 FOLLOW가 오면 planner를 호출하지 않는다.
        CommandExecutionManager manager = MakeD4Manager();
        Command follow = MakeKeep(50);
        follow.maneuver_type = Command::FOLLOW;
        follow.primary_object_id = 7;
        const auto acceptance = manager.Accept(follow);
        Expect(acceptance.command_accepted && !acceptance.should_plan &&
                   acceptance.execution_state == Feedback::FAILED &&
                   manager.runtime_failure() ==
                       RuntimeFailure::MANEUVER_NOT_IMPLEMENTED,
               "unimplemented maneuver is accepted but not planned");
        // 거짓 result_code를 쓰지 않는다. FrenetFeedback.msg에
        // MANEUVER_NOT_IMPLEMENTED가 없으므로 RESULT_NONE으로 남기고 사유는
        // detail에만 담는다.
        Expect(acceptance.result_code == Feedback::RESULT_NONE &&
                   !acceptance.detail.empty(),
               "unimplemented maneuver does not fake a result_code");
    }
    {
        // 구현 상태를 열면 정상 실행된다.
        CommandExecutionManager manager = MakeFullManager();
        Command follow = MakeKeep(51);
        follow.maneuver_type = Command::FOLLOW;
        follow.primary_object_id = 7;
        const auto acceptance = manager.Accept(follow);
        Expect(acceptance.should_plan &&
                   manager.runtime_failure() == RuntimeFailure::NONE,
               "implemented maneuver is planned");
    }
    {
        // 기본값은 아무것도 열지 않은 상태다.
        CommandExecutionManager manager;
        Expect(!manager.IsManeuverImplemented(Command::KEEP),
               "no maneuver is implemented by default");
    }

    // ================= 입력 데이터 stale =================
    {
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(60));
        manager.NotifyInputDataStale();
        Expect(manager.runtime_failure() == RuntimeFailure::INPUT_DATA_STALE &&
                   manager.has_active_command() &&
                   manager.command_id() == 60,
               "input stale keeps the transaction alive");
        // 입력이 회복되면 같은 transaction으로 계속한다.
        const auto acceptance = manager.Accept(MakeKeep(60));
        Expect(acceptance.should_plan &&
                   manager.runtime_failure() == RuntimeFailure::NONE,
               "transaction resumes after input recovers");
    }

    // ================= terminal 실패 사유도 latch되는가 ===================
    // 실측에서 발견된 문제: 상태만 latch하고 사유를 버리면 두 번째 사이클부터
    // result_code가 RESULT_NONE으로 떨어져 FSM이 원인을 잃는다.
    //   cycle 1: FAILED + NOT_INITIALIZED
    //   cycle 2: FAILED + RESULT_NONE     <- 사유 소실
    // 계약 6절은 새 command_id까지 terminal 상태를 유지하라고 하므로 사유도
    // 함께 유지해야 한다.
    {
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(90));
        manager.NotifyPlanFailed(/*terminal=*/true,
                                 Feedback::NOT_INITIALIZED,
                                 "planner is not initialized");
        Expect(manager.terminal_latched() &&
                   manager.terminal_result_code() == Feedback::NOT_INITIALIZED,
               "terminal failure latches its result_code");

        // 같은 ID가 여러 사이클 반복되어도 사유가 유지된다.
        for (int i = 0; i < 3; ++i) {
            const auto again = manager.Accept(MakeKeep(90));
            Expect(again.result_code == Feedback::NOT_INITIALIZED &&
                       again.detail == "planner is not initialized",
                   "latched result_code survives same-id refresh");
        }

        // 새 command_id에서 초기화된다.
        manager.Accept(MakeKeep(91));
        Expect(manager.terminal_result_code() == Feedback::RESULT_NONE &&
                   manager.terminal_result_detail().empty(),
               "new command_id clears the latched result_code");
    }
    {
        // TARGET_PROJECTION_FAILED도 같은 방식으로 유지된다(D4-2 STOP 경로).
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(92));
        manager.NotifyPlanFailed(/*terminal=*/true,
                                 Feedback::TARGET_PROJECTION_FAILED,
                                 "stop target is not on the reference line");
        const auto again = manager.Accept(MakeKeep(92));
        Expect(again.result_code == Feedback::TARGET_PROJECTION_FAILED,
               "projection failure reason is latched too");
    }
    {
        // 정상 완료는 오류가 아니므로 사유가 없고, completed_normally()로
        // 실패 latch와 구분된다. 호출자가 궤적 출력 정책을 나누는 근거다.
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(93));
        manager.NotifyManeuverCompleted();
        Expect(manager.completed_normally() &&
                   manager.terminal_result_code() == Feedback::RESULT_NONE,
               "normal completion latches without a failure reason");

        CommandExecutionManager failed_manager = MakeD4Manager();
        failed_manager.Accept(MakeKeep(94));
        failed_manager.NotifyPlanFailed(/*terminal=*/true,
                                        Feedback::COLLISION_BLOCKED, "blocked");
        Expect(failed_manager.terminal_latched() &&
                   !failed_manager.completed_normally(),
               "failure latch is not reported as normal completion");
    }
    {
        // 미구현 maneuver도 사유를 유지한다(detail 기반).
        CommandExecutionManager manager = MakeD4Manager();
        Command follow = MakeKeep(95);
        follow.maneuver_type = Command::FOLLOW;
        follow.primary_object_id = 7;
        manager.Accept(follow);
        const auto again = manager.Accept(follow);
        Expect(again.detail == "maneuver not implemented yet" &&
                   again.result_code == Feedback::RESULT_NONE,
               "unimplemented reason survives same-id refresh");
    }

    // ================= active_maneuver 소유권 =============================
    // 피드백의 active_maneuver는 **활성 transaction**의 maneuver여야 한다.
    // 방금 들어온(거부된) 명령의 것이 아니다. node의 publishFeedbackV2가
    // manager.maneuver_type()을 그대로 echo하므로, 여기서는 거부 후에도
    // 그 값이 유지되는지를 검증한다.
    {
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(80, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyCommitted();

        // 일반 KEEP이 오면 거부되고 LANE_CHANGE가 계속된다.
        const auto rejected = manager.Accept(MakeKeep(81));
        Expect(rejected.result_code == Feedback::TOO_LATE_TO_CANCEL,
               "normal command is rejected during committed LANE_CHANGE");
        Expect(manager.maneuver_type() == Command::LANE_CHANGE &&
                   manager.has_active_command(),
               "active maneuver stays LANE_CHANGE after rejecting KEEP");
        // 거부된 명령의 direction(NONE)을 쓰면 LANE_CHANGE 변환이 실패해
        // MANEUVER_NONE이 나간다. manager 값을 쓰면 그 문제가 없다.
        Expect(manager.maneuver_type() != Command::KEEP,
               "rejected command's maneuver does not leak into active state");
    }

    // ================= 거부된 명령이 상태를 오염시키지 않는가 =============
    // validation 실패는 manager를 거치지 않으므로(node가 먼저 걸러낸다) 여기서
    // 검증하는 것은 "manager 상태가 그대로 유지되는가"다. 실측에서 거부 회신에
    // FAILED가 실려 나가는 것을 봤는데, 그것은 직전 transaction이 정당하게
    // latch된 결과였다(계약 6절). 그 구분을 코드로 고정한다.
    {
        CommandExecutionManager manager = MakeD4Manager();
        // 활성 transaction이 없으면 IDLE이다.
        Expect(!manager.has_active_command() &&
                   manager.execution_state() == Feedback::IDLE,
               "fresh manager reports IDLE");

        // 미구현 maneuver로 FAILED latch를 만든다.
        Command follow = MakeKeep(70);
        follow.maneuver_type = Command::FOLLOW;
        follow.primary_object_id = 7;
        manager.Accept(follow);
        Expect(manager.execution_state() == Feedback::FAILED &&
                   manager.terminal_latched(),
               "unimplemented maneuver latches FAILED");

        // latch 중에는 그 상태가 계속 보고되는 것이 정상이다. 이것을 "거부
        // 회신이 상태를 오염시켰다"고 오해하면 안 된다.
        const auto refresh = manager.Accept(follow);
        Expect(refresh.execution_state == Feedback::FAILED,
               "latched FAILED keeps being reported (not contamination)");

        // 새 ID로 구현된 maneuver가 오면 정상 복귀한다.
        const auto recovered = manager.Accept(MakeKeep(71));
        Expect(recovered.should_plan &&
                   manager.execution_state() == Feedback::EXECUTING &&
                   !manager.terminal_latched(),
               "new command_id recovers from an unimplemented-maneuver latch");
    }

    // ============ D6-0: handshake 상태 전이 강제 ========================
    //
    // D6부터 handshake가 실제로 동작한다. 계약 5절의 정상 경로는 단 하나다:
    //   PREPARING/WAITING -> READY -> COMMITTED -> EXECUTING
    // 그 밖의 전이를 허용하면 검증되지 않은 궤적이 COMMIT되어 MPC로 나간다.
    {
        // PREPARING에서 바로 COMMIT하려는 시도는 거부한다. 준비 중인 후보는
        // 아직 충돌 검사를 통과했다는 보장이 없다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(200, Command::PREPARE));
        Expect(manager.execution_state() == Feedback::PREPARING,
               "handshake maneuver starts in PREPARING");
        Expect(!manager.NotifyCommitted(),
               "COMMIT from PREPARING is rejected");
        Expect(manager.execution_state() == Feedback::PREPARING,
               "rejected COMMIT leaves PREPARING untouched");
    }
    {
        // WAITING에서의 COMMIT도 거부한다. WAITING은 "안전 후보가 없다"는
        // 뜻이므로 잠글 궤적 자체가 없다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(201, Command::PREPARE));
        manager.NotifyPlanFailed(/*terminal=*/false);
        Expect(manager.execution_state() == Feedback::WAITING,
               "blocked PREPARE moves to WAITING");
        Expect(!manager.NotifyCommitted(), "COMMIT from WAITING is rejected");
        Expect(manager.execution_state() == Feedback::WAITING,
               "rejected COMMIT leaves WAITING untouched");
    }
    {
        // 정상 경로: READY -> COMMITTED -> EXECUTING.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(202, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        Expect(manager.execution_state() == Feedback::READY,
               "safe candidate reaches READY");

        Expect(manager.NotifyCommitted(), "COMMIT from READY succeeds");
        Expect(manager.execution_state() == Feedback::COMMITTED,
               "successful COMMIT reaches COMMITTED");

        Expect(manager.NotifyExecutionStarted(),
               "execution start from COMMITTED succeeds");
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "committed maneuver becomes EXECUTING");

        // 이미 EXECUTING이면 더 올릴 것이 없다. 반복 호출이 상태를 되돌리거나
        // 중복 전이를 만들지 않아야 한다.
        Expect(!manager.NotifyExecutionStarted(),
               "execution start is not repeatable");
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "repeated execution start leaves EXECUTING untouched");
        Expect(!manager.NotifyCommitted(),
               "COMMIT from EXECUTING is rejected (no going backwards)");
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "rejected COMMIT does not roll EXECUTING back to COMMITTED");
    }
    {
        // COMMITTED를 건너뛴 EXECUTING 승격은 없다. READY에서 바로
        // EXECUTING으로 가면 EXECUTE 재검증을 통과하지 않은 기동이
        // "실행 중"으로 보고된다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(203, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        Expect(!manager.NotifyExecutionStarted(),
               "execution start from READY is rejected");
        Expect(manager.execution_state() == Feedback::READY,
               "rejected execution start leaves READY untouched");
    }
    {
        // handshake를 쓰지 않는 maneuver에는 COMMIT/EXECUTING 승격 개념이
        // 없다. KEEP은 transaction 시작과 동시에 EXECUTING이며, 두 API는
        // 아무 일도 하지 않아야 한다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeKeep(204));
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "KEEP starts in EXECUTING without a handshake");
        Expect(!manager.NotifyCommitted(),
               "KEEP does not go through COMMIT");
        Expect(!manager.NotifyExecutionStarted(),
               "KEEP does not need an execution-start transition");
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "KEEP stays EXECUTING after the no-op transitions");
    }
    {
        // terminal latch 중에는 어떤 승격도 통과하지 않는다. FAILED로 끝난
        // transaction이 COMMITTED로 살아나면 계약 6절이 무너진다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(205, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyPlanFailed(/*terminal=*/true,
                                 Feedback::COLLISION_BLOCKED, "blocked");
        Expect(manager.terminal_latched() &&
                   manager.execution_state() == Feedback::FAILED,
               "terminal failure latches FAILED");
        Expect(!manager.NotifyCommitted(),
               "COMMIT is rejected while a terminal state is latched");
        Expect(!manager.NotifyExecutionStarted(),
               "execution start is rejected while latched");
        Expect(manager.execution_state() == Feedback::FAILED,
               "latched FAILED survives the rejected transitions");
    }
    {
        // 활성 transaction이 없으면 아무 승격도 없다.
        CommandExecutionManager manager = MakeFullManager();
        Expect(!manager.NotifyCommitted(),
               "COMMIT without an active command is rejected");
        Expect(!manager.NotifyExecutionStarted(),
               "execution start without an active command is rejected");
        Expect(manager.execution_state() == Feedback::IDLE,
               "idle manager stays IDLE");
    }
    {
        // COMMIT 이후 EXECUTING까지 갔어도 EMERGENCY_STOP은 통과한다.
        // D6-0에서 새로 생긴 EXECUTING 상태가 preemption을 막지 않는지
        // 확인한다 - 막히면 회피 중 급정지가 불가능해진다(계약 7.3절).
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(206, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyCommitted();
        manager.NotifyExecutionStarted();
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "maneuver is EXECUTING before the emergency");
        const auto emergency = manager.Accept(MakeEmergencyStop(207));
        Expect(emergency.command_accepted && emergency.should_plan &&
                   manager.command_id() == 207 &&
                   manager.maneuver_type() == Command::EMERGENCY_STOP,
               "EXECUTING maneuver still yields to EMERGENCY_STOP");
    }
    {
        // EXECUTING 중 일반 명령은 계속 거부된다(계약 7.2절).
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(208, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyCommitted();
        manager.NotifyExecutionStarted();
        const auto rejected = manager.Accept(MakeKeep(209));
        Expect(rejected.command_accepted && rejected.should_plan &&
                   rejected.result_code == Feedback::TOO_LATE_TO_CANCEL &&
                   manager.command_id() == 208,
               "EXECUTING maneuver rejects a normal new command");
        const auto cancel =
            manager.Accept(MakeLaneChange(208, Command::CANCEL));
        Expect(cancel.result_code == Feedback::TOO_LATE_TO_CANCEL &&
                   manager.execution_state() == Feedback::EXECUTING,
               "CANCEL during EXECUTING is TOO_LATE_TO_CANCEL");
    }
    {
        // EXECUTING에서 정상 완료하면 COMPLETED로 latch된다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(210, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyCommitted();
        manager.NotifyExecutionStarted();
        manager.NotifyManeuverCompleted();
        Expect(manager.execution_state() == Feedback::COMPLETED &&
                   manager.terminal_latched() && manager.completed_normally(),
               "EXECUTING maneuver completes normally");
    }
    {
        // COMMITTED 이후의 일시적 실패는 상태를 낮추지 않는다. 이미 잠긴
        // 궤적을 계속 실행해야 하기 때문이다. D6-0에서 EXECUTING이 실제로
        // 쓰이기 시작하므로 EXECUTING에서도 같은지 확인한다.
        CommandExecutionManager manager = MakeFullManager();
        manager.Accept(MakeLaneChange(211, Command::PREPARE));
        manager.NotifyPlanSucceeded();
        manager.NotifyCommitted();
        manager.NotifyExecutionStarted();
        manager.NotifyPlanFailed(/*terminal=*/false);
        Expect(!manager.terminal_latched() &&
                   manager.execution_state() == Feedback::EXECUTING,
               "transient failure during EXECUTING does not drop to WAITING");
    }

    if (g_failures != 0) {
        std::cerr << g_failures << " execution manager check(s) failed\n";
        return 1;
    }
    std::cout << "all command execution manager checks passed\n";
    return 0;
}
