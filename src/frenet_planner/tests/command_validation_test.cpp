// C1 regression: 외부 command 진입점의 mode whitelist와 필드 검증.
//
// 검증 대상은 "무엇을 거부하는가"가 아니라 "왜 거부했는가"까지다. ROS node가
// 이전 궤적 재사용(TRANSIENT_GENERATION_FAILURE)과 즉시 정지(INVALID_COMMAND /
// UNSUPPORTED_BEHAVIOR)를 구분하려면 reason이 정확해야 한다.

#include "frenet_planner/frenet_planner.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using Reason = FrenetPlanner::PlanFailureReason;

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

CartesianState LoadPoseNearArcLength(const std::string& path,
                                     double target_s,
                                     double speed) {
    std::ifstream input(path);
    std::vector<double> xs;
    std::vector<double> ys;
    double x = 0.0;
    double y = 0.0;
    double ignored = 0.0;
    while (input >> x >> y >> ignored) {
        xs.push_back(x);
        ys.push_back(y);
    }

    std::size_t index = 1;
    double s = 0.0;
    for (; index < xs.size(); ++index) {
        s += std::hypot(xs[index] - xs[index - 1],
                        ys[index] - ys[index - 1]);
        if (s >= target_s) break;
    }
    if (index >= xs.size()) index = xs.size() - 1;

    CartesianState pose{};
    pose.x = xs[index];
    pose.y = ys[index];
    pose.yaw = std::atan2(ys[index] - ys[index - 1],
                          xs[index] - xs[index - 1]);
    pose.v = speed;
    return pose;
}

// authoritative 8-corner OBB(perception의 obb_corners와 같은 형태)를
// 직접 채운 객체.
TrackedObject MakeAuthoritativeBox(int id, double x, double y) {
    TrackedObject obstacle;
    obstacle.id = id;
    obstacle.x = x;
    obstacle.y = y;
    obstacle.z = 1.0;
    obstacle.width = 2.0;
    obstacle.length = 4.0;
    obstacle.height = 2.0;
    obstacle.geometry_valid = true;
    obstacle.z_valid = true;
    const double xy[4][2] = {
        {x - 2.0, y - 1.0}, {x + 2.0, y - 1.0},
        {x + 2.0, y + 1.0}, {x - 2.0, y + 1.0},
    };
    for (std::size_t i = 0; i < 4; ++i) {
        obstacle.corners[i] = {xy[i][0], xy[i][1], 0.0};
        obstacle.corners[i + 4] = {xy[i][0], xy[i][1], 2.0};
    }
    return obstacle;
}

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string config = package_src + "/../config/params.yaml";
    const std::string waypoints = FRENET_TEST_WAYPOINT_FILE;

    FrenetPlanner planner;
    if (!planner.Init(config)) {
        std::cerr << "[FAIL] planner initialization\n";
        return 1;
    }

    // MERGE approach window 밖의 평범한 구간에서 검증한다. 여기서는 legacy
    // behavior가 개입할 여지가 없어 command 검증 결과만 관찰된다.
    const CartesianState ego = LoadPoseNearArcLength(waypoints, 200.0, 5.0);
    const std::vector<TrackedObject> no_objects;
    CartesianPath path;

    // --- 허용된 모드 -------------------------------------------------------
    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    Expect(planner.Plan(ego, no_objects, keep, path) &&
               planner.last_failure_reason() == Reason::NONE,
           "KEEP command passes validation");

    // 유효한 STOP은 validation을 통과해 PlanInternal까지 진입해야 한다.
    // (실제 정지 궤적 품질은 C4의 command_stop_plan_test가 담당한다.)
    PlannerCommand stop{};
    stop.mode = STOP;
    stop.stop_position = 230.0;  // ego(s=200m) 기준 전방
    planner.Plan(ego, no_objects, stop, path);
    Expect(planner.last_failure_reason() != Reason::UNSUPPORTED_BEHAVIOR &&
               planner.last_failure_reason() != Reason::INVALID_COMMAND,
           "valid STOP command is accepted by the external gate");

    // --- STOP 필드 검증 ----------------------------------------------------
    PlannerCommand stop_unset{};
    stop_unset.mode = STOP;  // stop_position은 기본값 NaN 그대로
    Expect(!planner.Plan(ego, no_objects, stop_unset, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "STOP without stop_position is INVALID_COMMAND");

    PlannerCommand stop_nan{};
    stop_nan.mode = STOP;
    stop_nan.stop_position = kNaN;
    Expect(!planner.Plan(ego, no_objects, stop_nan, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "STOP with NaN stop_position is INVALID_COMMAND");

    PlannerCommand stop_inf{};
    stop_inf.mode = STOP;
    stop_inf.stop_position = kInf;
    Expect(!planner.Plan(ego, no_objects, stop_inf, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "STOP with infinite stop_position is INVALID_COMMAND");

    PlannerCommand stop_negative{};
    stop_negative.mode = STOP;
    stop_negative.stop_position = -1.0;
    Expect(!planner.Plan(ego, no_objects, stop_negative, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "STOP with negative stop_position is INVALID_COMMAND");

    // --- 공통 속도 검증 ----------------------------------------------------
    PlannerCommand negative_speed{};
    negative_speed.mode = LANE_KEEPING;
    negative_speed.has_target_speed = true;
    negative_speed.target_speed = -1.0;
    Expect(!planner.Plan(ego, no_objects, negative_speed, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "negative target speed is INVALID_COMMAND");

    PlannerCommand nan_speed{};
    nan_speed.mode = LANE_KEEPING;
    nan_speed.has_target_speed = true;
    nan_speed.target_speed = kNaN;
    Expect(!planner.Plan(ego, no_objects, nan_speed, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "NaN target speed is INVALID_COMMAND");

    PlannerCommand negative_max{};
    negative_max.mode = LANE_KEEPING;
    negative_max.has_max_speed = true;
    negative_max.max_speed = -0.1;
    Expect(!planner.Plan(ego, no_objects, negative_max, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "negative max speed is INVALID_COMMAND");

    // has_*_speed가 false면 값은 사용되지 않으므로 검증 대상이 아니다.
    PlannerCommand unused_speed{};
    unused_speed.mode = LANE_KEEPING;
    unused_speed.has_target_speed = false;
    unused_speed.target_speed = kNaN;
    Expect(planner.Plan(ego, no_objects, unused_speed, path) &&
               planner.last_failure_reason() == Reason::NONE,
           "unused target_speed value is not validated");

    // --- 아직 열지 않은 모드 -----------------------------------------------
    // D5-1에서 FOLLOWING을 external whitelist에 추가했다. 이제 거부되지
    // 않으며, 대상 객체가 관측되지 않으면 TARGET_OBJECT_LOST가 된다
    // (whitelist 거부가 아니다). 실제 확보 규칙은
    // command_follow_plan_test가 검증한다.
    PlannerCommand follow{};
    follow.mode = FOLLOWING;
    follow.target_object_id = 7;
    Expect(!planner.Plan(ego, no_objects, follow, path) &&
               planner.last_failure_reason() == Reason::TARGET_OBJECT_LOST,
           "external FOLLOWING with no visible target is TARGET_OBJECT_LOST");

    // FOLLOW는 대상 ID가 필수다.
    PlannerCommand follow_no_target{};
    follow_no_target.mode = FOLLOWING;
    Expect(!planner.Plan(ego, no_objects, follow_no_target, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "external FOLLOWING without a target id is INVALID_COMMAND");

    PlannerCommand merge{};
    merge.mode = MERGE;
    merge.merge_type = MergeType::ROUNDABOUT;
    Expect(!planner.Plan(ego, no_objects, merge, path) &&
               planner.last_failure_reason() == Reason::UNSUPPORTED_BEHAVIOR,
           "external MERGE is UNSUPPORTED_BEHAVIOR");

    // D6-1에서 AVOID를 core whitelist에 추가했다. 이제 UNSUPPORTED_BEHAVIOR로
    // 거부되지 않고 필드 검증으로 넘어간다 - 대상이 없으면 INVALID_COMMAND다.
    // (FOLLOWING과 같은 이유: 대상 선택 권한이 FSM에 있으므로 planner가
    // 스스로 고르지 않는다.)
    //
    // 대상 확보에 성공한 뒤의 동작은 command_avoid_target_test가 담당한다.
    PlannerCommand avoid_without_target{};
    avoid_without_target.mode = AVOID;
    avoid_without_target.avoidance_d_offset = 1.5;
    Expect(!planner.Plan(ego, no_objects, avoid_without_target, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "external AVOID without a target is INVALID_COMMAND");

    // D7에서 LANE_CHANGE_LEFT/RIGHT를 core whitelist에 추가했다(AVOID와 동일한
    // 이유 - 대상 확보/투영까지만 열렸다). has_lane_change_target_d가
    // false이면(FSM target pose 투영 실패에 대응) INVALID_COMMAND다.
    //
    // 목표 투영에 성공한 뒤의 동작은 command_lane_change_plan_test가 담당한다.
    PlannerCommand lane_change{};
    lane_change.mode = LANE_CHANGE_LEFT;
    Expect(!planner.Plan(ego, no_objects, lane_change, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "external LANE_CHANGE_LEFT without a target is INVALID_COMMAND");

    // D4-3에서 EMERGENCY를 external whitelist에 추가했다. 이제 거부되지
    // 않으며, 목표점 없이도(stop_position=NaN) 동작해야 한다 - 정지 위치
    // 계산 소유권이 Frenet에 있다(계약 8.1절).
    //
    // 감속 궤적의 성질(단조 감소, v>=0, 마지막 v=0)은
    // command_emergency_plan_test가 검증한다. 여기서는 whitelist와 필드
    // 검증만 본다.
    PlannerCommand emergency{};
    emergency.mode = EMERGENCY;
    // stop_position을 일부러 지정하지 않는다.
    Expect(planner.Plan(ego, no_objects, emergency, path) &&
               planner.last_failure_reason() == Reason::NONE,
           "external EMERGENCY is accepted without a stop target");

    PlannerCommand emergency_with_target{};
    emergency_with_target.mode = EMERGENCY;
    emergency_with_target.stop_position = 210.0;
    Expect(planner.Plan(ego, no_objects, emergency_with_target, path) &&
               planner.last_failure_reason() == Reason::NONE,
           "external EMERGENCY ignores a stop target if one is given");

    // 모드 whitelist는 필드 검증보다 먼저다. 지원하지 않는 모드는 필드가
    // 무엇이든 UNSUPPORTED_BEHAVIOR로 보고돼야 한다.
    //
    // 닫힌 모드 예시는 단계가 진행되며 교체된다. D5까지는 AVOID를,
    // D6-1~D7 사이에는 LANE_CHANGE_LEFT를 썼지만 D7에서 LANE_CHANGE도
    // 열렸으므로 이제 MERGE가 유일하게 남은 닫힌 예시다.
    PlannerCommand unsupported_and_invalid{};
    unsupported_and_invalid.mode = MERGE;
    unsupported_and_invalid.has_target_speed = true;
    unsupported_and_invalid.target_speed = -5.0;
    Expect(!planner.Plan(ego, no_objects, unsupported_and_invalid, path) &&
               planner.last_failure_reason() == Reason::UNSUPPORTED_BEHAVIOR,
           "mode whitelist is checked before field validation");

    // --- authoritative OBB objects ------------------------------------------
    // 단일 objects collection이 authoritative corners를 이미 갖고 있을 때도
    // command validation 자체는 장애물 유무와 무관하게 동일하게 동작해야
    // 한다.
    const std::vector<TrackedObject> objects = {
        MakeAuthoritativeBox(31, ego.x + 100.0, ego.y + 100.0)};

    Expect(planner.Plan(ego, objects, keep, path) &&
               planner.last_failure_reason() == Reason::NONE,
           "authoritative OBB object accepts a matched KEEP command");

    PlannerCommand still_closed{};
    // D7에서 LANE_CHANGE도 열렸으므로 여기도 아직 닫힌 모드로 바꾼다(위와 동일).
    still_closed.mode = MERGE;
    Expect(!planner.Plan(ego, objects, still_closed, path) &&
               planner.last_failure_reason() == Reason::UNSUPPORTED_BEHAVIOR,
           "authoritative OBB object rejects unsupported mode identically");

    Expect(!planner.Plan(ego, objects, stop_unset, path) &&
               planner.last_failure_reason() == Reason::INVALID_COMMAND,
           "authoritative OBB object rejects invalid STOP identically");

    // --- 실패 reason이 재사용 정책과 섞이지 않는지 ------------------------
    // 어떤 command 결함도 TRANSIENT_GENERATION_FAILURE로 보고되면 안 된다.
    // 그 값은 ROS node에서 이전 궤적 재사용을 뜻하기 때문이다.
    const PlannerCommand rejected[] = {
        stop_unset, stop_nan, stop_negative, negative_speed,
        nan_speed, negative_max, follow, merge, avoid_without_target,
        lane_change,
    };
    bool never_transient = true;
    for (const PlannerCommand& command : rejected) {
        planner.Plan(ego, no_objects, command, path);
        never_transient = never_transient &&
            planner.last_failure_reason() != Reason::TRANSIENT_GENERATION_FAILURE;
    }
    Expect(never_transient,
           "command rejections are never reported as transient failures");

    if (g_failures == 0) {
        std::cout << "All command validation tests passed\n";
        return 0;
    }
    return 1;
}
