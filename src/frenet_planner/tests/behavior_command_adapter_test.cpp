// D3-2: BehaviorCommand validation + transaction fingerprint + 입력 변환 회귀.
//
// 이 테스트가 지키는 두 가지 계약:
//
//   1. validation matrix (계약 문서 3.1~3.4절)
//      형식 오류는 INVALID_COMMAND, valid_until 만료는 STALE_COMMAND로
//      구분되어야 한다. 섞이면 FSM이 잘못된 회복 동작을 한다.
//
//   2. transaction semantics (계약 문서 4절)
//      같은 command_id에서 PREPARE -> EXECUTE는 동일 transaction이고,
//      payload가 바뀌면 다른 명령이다. execution_request를 fingerprint에
//      넣으면 정상 handshake가 깨지므로 그것을 명시적으로 검증한다.

#include "ros/behavior_command_adapter.hpp"

#include <iostream>
#include <string>

namespace {

using frenet_planner_ros::CommandFingerprint;
using frenet_planner_ros::CommandValidationCode;
using frenet_planner_ros::ConvertBehaviorCommand;
using frenet_planner_ros::MakeCommandFingerprint;
using frenet_planner_ros::SameCommandFingerprint;
using frenet_planner_ros::ValidateBehaviorCommand;
using Command = asmc_msgs::BehaviorCommand;

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

const ros::Time kNow(1000.0);
const ros::Time kFuture(1001.0);

// 공통 필드가 모두 유효한 KEEP 명령. 각 케이스는 여기서 필요한 필드만 바꾼다.
Command MakeValidKeep() {
    Command message;
    message.header.frame_id = "map";
    message.header.stamp = kNow;
    message.command_id = 42;
    message.maneuver_type = Command::KEEP;
    message.execution_request = Command::EXECUTE;
    message.direction = Command::DIRECTION_NONE;
    message.merge_type = Command::MERGE_NONE;
    message.has_target_pose = false;
    message.desired_speed_mps = -1.0;
    message.primary_object_id = -1;
    message.preferred_front_object_id = -1;
    message.preferred_rear_object_id = -1;
    message.use_custom_follow_gap = false;
    message.desired_time_gap_s = -1.0;
    message.minimum_gap_m = -1.0;
    message.valid_until = kFuture;
    return message;
}

void SetUnitPose(Command& message, double x, double y) {
    message.has_target_pose = true;
    message.target_pose_map.position.x = x;
    message.target_pose_map.position.y = y;
    message.target_pose_map.position.z = 0.0;
    message.target_pose_map.orientation.x = 0.0;
    message.target_pose_map.orientation.y = 0.0;
    message.target_pose_map.orientation.z = 0.0;
    message.target_pose_map.orientation.w = 1.0;
}

Command MakeValidFollow() {
    Command message = MakeValidKeep();
    message.maneuver_type = Command::FOLLOW;
    message.primary_object_id = 7;
    return message;
}

Command MakeValidStop() {
    Command message = MakeValidKeep();
    message.maneuver_type = Command::STOP;
    SetUnitPose(message, 120.0, 34.0);
    return message;
}

Command MakeValidLaneChange(uint8_t direction) {
    Command message = MakeValidKeep();
    message.maneuver_type = Command::LANE_CHANGE;
    message.execution_request = Command::PREPARE;
    message.direction = direction;
    SetUnitPose(message, 200.0, 50.0);
    return message;
}

Command MakeValidAvoid() {
    Command message = MakeValidKeep();
    message.maneuver_type = Command::AVOID;
    message.execution_request = Command::PREPARE;
    message.primary_object_id = 21;
    return message;
}

Command MakeValidMerge() {
    Command message = MakeValidKeep();
    message.maneuver_type = Command::MERGE;
    message.execution_request = Command::PREPARE;
    message.merge_type = Command::MERGE_ROUNDABOUT;
    return message;
}

void ExpectValid(const Command& message, const std::string& name) {
    const auto result = ValidateBehaviorCommand(message, kNow);
    if (!result.ok()) {
        std::cerr << "    (detail: " << result.detail << ")\n";
    }
    Expect(result.ok(), name);
}

void ExpectCode(const Command& message, CommandValidationCode expected,
                const std::string& name) {
    const auto result = ValidateBehaviorCommand(message, kNow);
    Expect(result.code == expected && !result.detail.empty(), name);
}

void ExpectInvalid(const Command& message, const std::string& name) {
    ExpectCode(message, CommandValidationCode::INVALID_COMMAND, name);
}

}  // namespace

int main() {
    // ================= 정상 명령 =================
    ExpectValid(MakeValidKeep(), "valid KEEP");
    ExpectValid(MakeValidFollow(), "valid FOLLOW");
    ExpectValid(MakeValidStop(), "valid STOP");
    {
        Command estop = MakeValidKeep();
        estop.maneuver_type = Command::EMERGENCY_STOP;
        ExpectValid(estop, "valid EMERGENCY_STOP");
    }
    ExpectValid(MakeValidLaneChange(Command::DIRECTION_LEFT),
                "valid LANE_CHANGE LEFT");
    ExpectValid(MakeValidLaneChange(Command::DIRECTION_RIGHT),
                "valid LANE_CHANGE RIGHT");
    ExpectValid(MakeValidAvoid(), "valid AVOID");
    ExpectValid(MakeValidMerge(), "valid MERGE");

    // handshake maneuver는 세 요청을 모두 받는다.
    for (uint8_t request : {Command::PREPARE, Command::EXECUTE, Command::CANCEL}) {
        Command lane_change = MakeValidLaneChange(Command::DIRECTION_LEFT);
        lane_change.execution_request = request;
        ExpectValid(lane_change, "LANE_CHANGE accepts execution_request "
                                     + std::to_string(static_cast<int>(request)));
    }

    // ================= 공통 필드 오류 =================
    {
        Command message = MakeValidKeep();
        message.header.frame_id = "odom";
        ExpectInvalid(message, "non-map frame_id is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.command_id = 0;
        ExpectInvalid(message, "command_id 0 is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.valid_until = ros::Time(0.0);
        ExpectInvalid(message, "zero valid_until is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.maneuver_type = 7;
        ExpectInvalid(message, "unknown maneuver_type is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.execution_request = 3;
        ExpectInvalid(message, "unknown execution_request is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.direction = 3;
        ExpectInvalid(message, "unknown direction is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.merge_type = 3;
        ExpectInvalid(message, "unknown merge_type is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.desired_speed_mps = -2.0;
        ExpectInvalid(message, "negative desired_speed (not -1) is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.desired_speed_mps = std::nan("");
        ExpectInvalid(message, "NaN desired_speed is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.primary_object_id = -2;
        ExpectInvalid(message, "object id < -1 is INVALID");
    }
    // sentinel은 정확히 비교한다. 계약이 "== -1.0 또는 >= 0.0"이므로
    // -1 < value < 0 구간은 전부 거부되어야 한다. 허용오차 비교(1e-6)를
    // 쓰면 -0.9999995가 통과하므로 그것을 막는다.
    {
        Command message = MakeValidKeep();
        message.desired_speed_mps = -0.9999995;
        ExpectInvalid(message, "near-sentinel desired_speed (-0.9999995) is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.desired_speed_mps = -0.5;
        ExpectInvalid(message, "desired_speed -0.5 is INVALID");
    }
    {
        Command message = MakeValidFollow();
        message.desired_time_gap_s = -0.9999995;  // custom gap off
        ExpectInvalid(message, "near-sentinel time_gap is INVALID when custom is off");
    }
    {
        Command message = MakeValidFollow();
        message.minimum_gap_m = -0.9999995;
        ExpectInvalid(message, "near-sentinel min_gap is INVALID when custom is off");
    }

    // desired_speed_mps = 0.0은 "정지 요청"으로 유효하다.
    {
        Command message = MakeValidKeep();
        message.desired_speed_mps = 0.0;
        ExpectValid(message, "desired_speed 0.0 is a valid stop request");
    }

    // ================= STALE =================
    {
        Command message = MakeValidKeep();
        message.valid_until = ros::Time(999.0);  // now=1000
        ExpectCode(message, CommandValidationCode::STALE_COMMAND,
                   "expired valid_until is STALE");
    }
    // 형식 오류가 STALE로 가려지지 않아야 한다. 형식이 잘못된 명령이 늦게
    // 도착했다면 FSM이 알아야 하는 것은 형식 오류다.
    {
        Command message = MakeValidKeep();
        message.command_id = 0;
        message.valid_until = ros::Time(999.0);
        ExpectInvalid(message, "format error outranks expiry (not masked as STALE)");
    }

    // ================= FOLLOW gap =================
    {
        Command message = MakeValidFollow();
        message.use_custom_follow_gap = true;
        message.desired_time_gap_s = 1.5;
        message.minimum_gap_m = 6.0;
        ExpectValid(message, "custom follow gap with positive values");
    }
    {
        Command message = MakeValidFollow();
        message.use_custom_follow_gap = true;
        message.desired_time_gap_s = 0.0;
        message.minimum_gap_m = 6.0;
        ExpectInvalid(message, "custom gap with time_gap 0 is INVALID");
    }
    {
        Command message = MakeValidFollow();
        message.use_custom_follow_gap = true;
        message.desired_time_gap_s = 1.5;
        message.minimum_gap_m = -1.0;
        ExpectInvalid(message, "custom gap with negative min_gap is INVALID");
    }
    {
        Command message = MakeValidFollow();
        message.use_custom_follow_gap = false;
        message.desired_time_gap_s = 1.5;  // -1.0이어야 한다
        ExpectInvalid(message, "gap value set while custom gap is off is INVALID");
    }

    // ================= maneuver별 =================
    {
        Command message = MakeValidFollow();
        message.primary_object_id = -1;
        ExpectInvalid(message, "FOLLOW without primary_object_id is INVALID");
    }
    {
        Command message = MakeValidLaneChange(Command::DIRECTION_NONE);
        ExpectInvalid(message, "LANE_CHANGE with DIRECTION_NONE is INVALID");
    }
    {
        Command message = MakeValidStop();
        message.has_target_pose = false;
        ExpectInvalid(message, "STOP without target pose is INVALID");
    }
    {
        Command message = MakeValidKeep();
        SetUnitPose(message, 10.0, 10.0);
        ExpectInvalid(message, "KEEP carrying a target pose is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.direction = Command::DIRECTION_LEFT;
        ExpectInvalid(message, "KEEP with a direction is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.primary_object_id = 5;
        ExpectInvalid(message, "KEEP with an object id is INVALID");
    }
    {
        Command message = MakeValidMerge();
        message.merge_type = Command::MERGE_NONE;
        ExpectInvalid(message, "MERGE without merge_type is INVALID");
    }
    {
        Command message = MakeValidKeep();
        message.merge_type = Command::MERGE_ROUNDABOUT;
        ExpectInvalid(message, "non-MERGE with a merge_type is INVALID");
    }
    {
        Command message = MakeValidMerge();
        message.primary_object_id = 3;
        ExpectInvalid(message, "MERGE using primary_object_id is INVALID");
    }
    {
        Command message = MakeValidAvoid();
        message.primary_object_id = -1;
        ExpectInvalid(message, "AVOID without primary_object_id is INVALID");
    }
    // handshake를 쓰지 않는 maneuver는 EXECUTE만 받는다.
    {
        Command message = MakeValidKeep();
        message.execution_request = Command::PREPARE;
        ExpectInvalid(message, "KEEP with PREPARE is INVALID");
    }
    // AVOID의 direction은 선택이며 세 값 모두 허용된다.
    for (uint8_t direction : {Command::DIRECTION_NONE, Command::DIRECTION_LEFT,
                              Command::DIRECTION_RIGHT}) {
        Command message = MakeValidAvoid();
        message.direction = direction;
        ExpectValid(message, "AVOID accepts direction "
                                 + std::to_string(static_cast<int>(direction)));
    }

    // ================= target pose 유효성 =================
    {
        Command message = MakeValidStop();
        message.target_pose_map.position.x = std::nan("");
        ExpectInvalid(message, "non-finite target position is INVALID");
    }
    {
        Command message = MakeValidStop();
        // 영벡터 quaternion은 회전을 나타내지 않는다.
        message.target_pose_map.orientation.w = 0.0;
        ExpectInvalid(message, "non-unit quaternion is INVALID");
    }

    // ================= Fingerprint =================
    // 핵심: PREPARE -> EXECUTE가 같은 transaction으로 인정되어야 한다.
    {
        Command prepare = MakeValidLaneChange(Command::DIRECTION_LEFT);
        Command execute = prepare;
        execute.execution_request = Command::EXECUTE;
        Expect(SameCommandFingerprint(MakeCommandFingerprint(prepare),
                                      MakeCommandFingerprint(execute)),
               "PREPARE -> EXECUTE is the same fingerprint");
    }
    {
        Command first = MakeValidLaneChange(Command::DIRECTION_LEFT);
        Command second = first;
        second.execution_request = Command::CANCEL;
        Expect(SameCommandFingerprint(MakeCommandFingerprint(first),
                                      MakeCommandFingerprint(second)),
               "CANCEL keeps the same fingerprint");
    }
    // 매 발행마다 바뀌는 필드는 identity에 영향이 없어야 한다.
    {
        Command first = MakeValidKeep();
        Command second = first;
        second.header.stamp = ros::Time(1234.5);
        second.header.seq = 99;
        Expect(SameCommandFingerprint(MakeCommandFingerprint(first),
                                      MakeCommandFingerprint(second)),
               "header.stamp change keeps the same fingerprint");
    }
    {
        Command first = MakeValidKeep();
        Command second = first;
        second.valid_until = ros::Time(2000.0);
        Expect(SameCommandFingerprint(MakeCommandFingerprint(first),
                                      MakeCommandFingerprint(second)),
               "valid_until change keeps the same fingerprint");
    }
    {
        Command first = MakeValidKeep();
        Command second = first;
        second.reason = "slow_leader";
        Expect(SameCommandFingerprint(MakeCommandFingerprint(first),
                                      MakeCommandFingerprint(second)),
               "reason change keeps the same fingerprint");
    }

    // payload가 바뀌면 다른 명령이다.
    {
        Command first = MakeValidFollow();
        Command second = first;
        second.primary_object_id = 20;
        Expect(!SameCommandFingerprint(MakeCommandFingerprint(first),
                                       MakeCommandFingerprint(second)),
               "target object change is a DIFFERENT fingerprint");
    }
    {
        Command first = MakeValidStop();
        Command second = first;
        second.target_pose_map.position.x += 5.0;
        Expect(!SameCommandFingerprint(MakeCommandFingerprint(first),
                                       MakeCommandFingerprint(second)),
               "target pose change is a DIFFERENT fingerprint");
    }
    {
        Command first = MakeValidKeep();
        Command second = first;
        second.maneuver_type = Command::EMERGENCY_STOP;
        Expect(!SameCommandFingerprint(MakeCommandFingerprint(first),
                                       MakeCommandFingerprint(second)),
               "maneuver change is a DIFFERENT fingerprint");
    }
    {
        Command first = MakeValidLaneChange(Command::DIRECTION_LEFT);
        Command second = MakeValidLaneChange(Command::DIRECTION_RIGHT);
        Expect(!SameCommandFingerprint(MakeCommandFingerprint(first),
                                       MakeCommandFingerprint(second)),
               "direction change is a DIFFERENT fingerprint");
    }
    {
        Command first = MakeValidKeep();
        Command second = first;
        second.desired_speed_mps = 8.0;
        Expect(!SameCommandFingerprint(MakeCommandFingerprint(first),
                                       MakeCommandFingerprint(second)),
               "desired_speed change is a DIFFERENT fingerprint");
    }
    {
        Command first = MakeValidMerge();
        Command second = first;
        second.merge_type = Command::MERGE_HIGHWAY;
        Expect(!SameCommandFingerprint(MakeCommandFingerprint(first),
                                       MakeCommandFingerprint(second)),
               "merge_type change is a DIFFERENT fingerprint");
    }
    {
        Command first = MakeValidMerge();
        Command second = first;
        second.target_link_id = "link_42";
        Expect(!SameCommandFingerprint(MakeCommandFingerprint(first),
                                       MakeCommandFingerprint(second)),
               "target_link_id change is a DIFFERENT fingerprint");
    }
    {
        Command first = MakeValidMerge();
        Command second = first;
        second.preferred_front_object_id = 11;
        Expect(!SameCommandFingerprint(MakeCommandFingerprint(first),
                                       MakeCommandFingerprint(second)),
               "preferred_front change is a DIFFERENT fingerprint");
    }
    // 부동소수 허용오차: 표현 오차 수준의 차이는 같다고 본다.
    {
        Command first = MakeValidKeep();
        first.desired_speed_mps = 8.0;
        Command second = first;
        second.desired_speed_mps = 8.0 + 1e-9;
        Expect(SameCommandFingerprint(MakeCommandFingerprint(first),
                                      MakeCommandFingerprint(second)),
               "float representation noise keeps the same fingerprint");
    }

    // ================= 입력 필드 변환 =================
    {
        Command message = MakeValidFollow();
        message.desired_speed_mps = 7.5;
        PlannerCommand command{};
        Expect(ConvertBehaviorCommand(message, command) &&
                   command.mode == FOLLOWING &&
                   command.target_object_id == 7 &&
                   command.has_target_speed && command.target_speed == 7.5,
               "FOLLOW converts mode / target / speed");
    }
    {
        // -1.0은 "planner 기본 목표속도 사용"이다.
        Command message = MakeValidKeep();
        message.desired_speed_mps = -1.0;
        PlannerCommand command{};
        Expect(ConvertBehaviorCommand(message, command) &&
                   !command.has_target_speed,
               "desired_speed -1.0 means use planner default");
    }
    {
        // v2 경로는 상한을 쓰지 않는다.
        PlannerCommand command{};
        Expect(ConvertBehaviorCommand(MakeValidKeep(), command) &&
                   !command.has_max_speed,
               "v2 conversion never sets has_max_speed");
    }
    {
        // 계약 11.8절: custom gap이 꺼져 있으면 0.0 sentinel.
        PlannerCommand command{};
        Expect(ConvertBehaviorCommand(MakeValidFollow(), command) &&
                   command.time_gap == 0.0 && command.min_gap == 0.0,
               "custom gap off maps to 0.0 sentinel (not -1.0)");
    }
    {
        Command message = MakeValidFollow();
        message.use_custom_follow_gap = true;
        message.desired_time_gap_s = 1.8;
        message.minimum_gap_m = 6.5;
        PlannerCommand command{};
        Expect(ConvertBehaviorCommand(message, command) &&
                   command.time_gap == 1.8 && command.min_gap == 6.5,
               "custom gap on passes values through");
    }
    {
        Command message = MakeValidMerge();
        message.merge_type = Command::MERGE_HIGHWAY;
        message.preferred_front_object_id = 11;
        message.preferred_rear_object_id = 12;
        PlannerCommand command{};
        Expect(ConvertBehaviorCommand(message, command) &&
                   command.mode == MERGE &&
                   command.merge_type == MergeType::HIGHWAY &&
                   command.merge_sa_id == 11 && command.merge_sb_id == 12,
               "MERGE converts merge_type and gap object ids");
    }
    {
        // stop_position은 RefLine 투영이 필요하므로 변환 단계에서 채우지
        // 않는다. NaN이 유지되어야 하며, 그 상태로 STOP을 실행하면 planner가
        // INVALID_COMMAND로 거부한다.
        PlannerCommand command{};
        Expect(ConvertBehaviorCommand(MakeValidStop(), command) &&
                   command.mode == STOP &&
                   !std::isfinite(command.stop_position),
               "STOP conversion leaves stop_position unset for the caller");
    }
    {
        Command message = MakeValidLaneChange(Command::DIRECTION_RIGHT);
        PlannerCommand command{};
        Expect(ConvertBehaviorCommand(message, command) &&
                   command.mode == LANE_CHANGE_RIGHT,
               "LANE_CHANGE RIGHT converts to LANE_CHANGE_RIGHT");
    }
    {
        // enum 매핑이 불가능한 명령은 변환도 실패한다.
        Command message = MakeValidLaneChange(Command::DIRECTION_LEFT);
        message.direction = Command::DIRECTION_NONE;
        PlannerCommand command{};
        Expect(!ConvertBehaviorCommand(message, command),
               "LANE_CHANGE without direction fails conversion");
    }

    if (g_failures != 0) {
        std::cerr << g_failures << " adapter check(s) failed\n";
        return 1;
    }
    std::cout << "all v2 command adapter checks passed\n";
    return 0;
}
