// D4-3: 외부 EMERGENCY_STOP의 직접 감속 궤적 회귀.
//
// 이 테스트의 핵심은 "EMERGENCY 전용 분기가 왜 필요한가"를 코드로 고정하는
// 것이다. BehaviorCommand는 목표점 없이(has_target_pose=false) 긴급정지를
// 지시하므로 stop_position이 NaN이다. 그 상태로 후보 파이프라인
// (ResolveManeuver -> GenerateStoppingCandidates)에 내려보내면 NaN 경계조건
// 으로 후보를 만들고, 전부 탈락하면 "긴급정지인데 아무 궤적도 못 보냄"이
// 된다. 긴급정지에서 그 실패 방향 자체를 제거하는 것이 목적이다.
//
// 계약: docs/contracts/planning/fsm-frenet-v2.md 8.1절

#include "Frenet/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

CartesianState LoadPoseNearArcLength(const std::string& path, double target_s,
                                     double speed) {
    std::ifstream input(path);
    std::vector<double> xs, ys;
    double x = 0.0, y = 0.0, ignored = 0.0;
    while (input >> x >> y >> ignored) { xs.push_back(x); ys.push_back(y); }
    std::size_t index = 1;
    double s = 0.0;
    for (; index < xs.size(); ++index) {
        s += std::hypot(xs[index] - xs[index - 1], ys[index] - ys[index - 1]);
        if (s >= target_s) break;
    }
    if (index >= xs.size()) index = xs.size() - 1;
    CartesianState pose{};
    pose.x = xs[index];
    pose.y = ys[index];
    pose.yaw = std::atan2(ys[index] - ys[index - 1], xs[index] - xs[index - 1]);
    pose.v = speed;
    return pose;
}

// params.yaml의 waypoint_file은 배포 환경 절대경로다. 테스트가 환경에
// 의존하지 않도록 사본을 만들어 쓴다(legacy_behavior_regression_test와 동일).
std::string MakeLocalConfig(const std::string& source, const std::string& waypoints,
                            const std::string& out_path) {
    std::ifstream in(source);
    if (!in) return {};
    std::ofstream out(out_path);
    if (!out) return {};
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("waypoint_file:", 0) == 0) {
            out << "waypoint_file: \"" << waypoints << "\"\n";
        } else {
            out << line << '\n';
        }
    }
    return out_path;
}

PlannerCommand MakeEmergencyCommand() {
    PlannerCommand command{};
    command.mode = EMERGENCY;
    // 목표점을 주지 않는다. stop_position은 기본값 NaN으로 남는다 - 이것이
    // BehaviorCommand의 EMERGENCY_STOP 계약이다.
    return command;
}

// 궤적이 긴급정지 계약을 만족하는지: 모든 v >= 0, 단조 감소, 마지막 v = 0.
bool IsValidStopPath(const CartesianPath& path) {
    if (path.v.empty()) return false;
    for (std::size_t i = 0; i < path.v.size(); ++i) {
        if (!std::isfinite(path.v[i]) || path.v[i] < 0.0) return false;
        if (!std::isfinite(path.x[i]) || !std::isfinite(path.y[i])) return false;
        if (i > 0 && path.v[i] > path.v[i - 1] + 1e-9) return false;
    }
    return path.v.back() == 0.0;
}

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoints = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/../config/frenet/params.yaml", waypoints,
        "command_emergency_plan_test_params.yaml");
    if (config.empty()) {
        std::cerr << "[FAIL] could not prepare test config\n";
        return 1;
    }

    const std::vector<TrackedObject> no_objects;

    // --- 주행 중 긴급정지 ------------------------------------------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        const CartesianState ego = LoadPoseNearArcLength(waypoints, 200.0, 8.0);
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects,
                                     MakeEmergencyCommand(), path);
        Expect(ok, "EMERGENCY with NaN stop_position still produces a path");
        Expect(planner.last_failure_reason() ==
                   FrenetPlanner::PlanFailureReason::NONE,
               "EMERGENCY does not report a failure reason");
        Expect(planner.last_mode() == EMERGENCY,
               "EMERGENCY is reported as the active mode");
        Expect(IsValidStopPath(path),
               "EMERGENCY path decelerates monotonically, v>=0, ends at 0");
        Expect(path.v.size() > 1 && path.v.front() > 0.0,
               "EMERGENCY path starts at the current speed");
        Expect(planner.last_target_speed() == 0.0,
               "EMERGENCY reports a zero target speed");
    }

    // --- 이미 정지한 상태 -------------------------------------------------
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        const CartesianState ego = LoadPoseNearArcLength(waypoints, 200.0, 0.0);
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects,
                                     MakeEmergencyCommand(), path);
        Expect(ok && !path.v.empty(),
               "EMERGENCY at standstill still emits a path");
        Expect(path.v.back() == 0.0,
               "EMERGENCY at standstill emits a zero-speed path");
        // 저속 fallback으로 새면 kMinCreepSpeed로 기어서 전진한다. 긴급정지
        // 중에 차량이 움직이면 안 되므로 이 검사가 중요하다.
        bool any_motion = false;
        for (double v : path.v) if (v > 1e-6) any_motion = true;
        Expect(!any_motion, "EMERGENCY at standstill never creeps forward");
    }

    // --- 여러 초기 속도 ---------------------------------------------------
    for (double speed : {1.0, 3.0, 8.0, 15.0}) {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        const CartesianState ego = LoadPoseNearArcLength(waypoints, 200.0, speed);
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects,
                                     MakeEmergencyCommand(), path);
        Expect(ok && IsValidStopPath(path),
               "EMERGENCY from " + std::to_string(static_cast<int>(speed)) +
                   " m/s decelerates to a stop");
    }

    // --- 장애물이 있어도 긴급정지는 궤적을 낸다 ----------------------------
    // 충돌 필터를 타지 않으므로 "충돌 차단으로 긴급정지 실패"가 발생하지
    // 않는다. 이것이 후보 파이프라인을 건너뛰는 이유다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        const CartesianState ego = LoadPoseNearArcLength(waypoints, 200.0, 8.0);
        const CartesianState blocker = LoadPoseNearArcLength(waypoints, 210.0, 0.0);
        TrackedObject object;
        object.id = 1;
        object.x = blocker.x;
        object.y = blocker.y;
        object.heading = blocker.yaw;
        object.width = 2.0;
        object.length = 4.5;
        object.height = 1.5;
        const std::vector<TrackedObject> objects{object};
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, MakeEmergencyCommand(), path);
        Expect(ok && IsValidStopPath(path),
               "EMERGENCY emits a path even with an obstacle straight ahead");
    }

    // --- STOP은 여전히 목표점을 요구한다 ----------------------------------
    // EMERGENCY에 목표점 검증을 면제한 것이 STOP까지 느슨하게 만들지
    // 않았는지 확인한다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) return 1;
        const CartesianState ego = LoadPoseNearArcLength(waypoints, 200.0, 8.0);
        PlannerCommand stop{};
        stop.mode = STOP;  // stop_position은 NaN
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects,
                                     stop, path);
        Expect(!ok && planner.last_failure_reason() ==
                          FrenetPlanner::PlanFailureReason::INVALID_COMMAND,
               "STOP without a target is still INVALID_COMMAND");
    }

    std::remove(config.c_str());
    if (g_failures != 0) {
        std::cerr << g_failures << " emergency stop check(s) failed\n";
        return 1;
    }
    std::cout << "all emergency stop checks passed\n";
    return 0;
}
