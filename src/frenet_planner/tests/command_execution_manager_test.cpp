// D3-3: transaction / terminal latch / preemption 회귀.
//
// 이 테스트가 지키는 계약:
//   4절  command_id는 transaction identity이며 같은 ID의 payload 변경은 거부
//   5절  PREPARE -> EXECUTE는 같은 transaction
//   6절  COMPLETED / FAILED는 새 command_id까지 유지 (terminal latch)
//   7절  CANCEL은 COMMITTED 이후 거부, EMERGENCY_STOP만 언제든 preempt
//   11.4절 미구현 maneuver는 planner를 호출하지 않는다

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
        // EXECUTING(=COMMITTED phase)에서는 거부한다.
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(16));
        Expect(manager.execution_state() == Feedback::EXECUTING,
               "non-handshake maneuver is EXECUTING");
        Command cancel = MakeKeep(16);
        cancel.execution_request = Command::CANCEL;
        // KEEP은 CANCEL을 쓰지 않지만 manager 규칙 자체를 검증한다.
        const auto result = manager.Accept(cancel);
        Expect(result.command_accepted &&
                   result.result_code == Feedback::TOO_LATE_TO_CANCEL &&
                   result.should_plan &&
                   manager.execution_state() == Feedback::EXECUTING,
               "CANCEL while EXECUTING is TOO_LATE_TO_CANCEL");
    }

    // ================= COMMITTED 중 새 일반 명령 =================
    {
        CommandExecutionManager manager = MakeD4Manager();
        manager.Accept(MakeKeep(20));  // -> EXECUTING
        const auto acceptance = manager.Accept(MakeKeep(21));
        Expect(acceptance.command_accepted &&
                   acceptance.result_code == Feedback::TOO_LATE_TO_CANCEL &&
                   acceptance.should_plan &&
                   manager.command_id() == 20,
               "new normal command during EXECUTING is rejected, current preserved");
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

    if (g_failures != 0) {
        std::cerr << g_failures << " execution manager check(s) failed\n";
        return 1;
    }
    std::cout << "all command execution manager checks passed\n";
    return 0;
}
