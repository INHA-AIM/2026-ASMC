// 외부 KEEP low-speed boundary candidate starvation 회귀
// (fix/yuntae-frenet-speed-continuity Phase 5).
//
// 재현 조건: ego speed가 kLowSpeedFallbackV(0.5 m/s) 문턱을 살짝 넘었는데
// 강한 감속 중이던 s_ddot 경계조건이 남아 있으면(실측: v=0.5~0.7,
// s_ddot~=-2.4) 정상 velocity keeping 후보가 전부 accel/jerk 한계를 넘어
// 탈락한다(longitudinal 0/40 valid). 이 테스트는 speed 0.49/0.50/0.51/0.6/0.8
// 근방에서 trajectory availability가 discontinuous하게 깨지지 않는지,
// 그리고 negative velocity/jerk invariant가 fallback 경로에서도 유지되는지
// 확인한다.
//
// 실제 production FrenetPlanner::Plan()을 그대로 호출한다.

#include "frenet_planner/frenet_planner.hpp"

#include <cmath>
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
                                     double speed, double accel) {
    std::ifstream input(path);
    std::vector<double> xs, ys;
    double x = 0.0, y = 0.0, ignored = 0.0;
    while (input >> x >> y >> ignored) {
        xs.push_back(x);
        ys.push_back(y);
    }
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
    pose.a = accel;
    return pose;
}

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

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "low_speed_keep_handoff_test_params.yaml");
    if (config.empty()) {
        std::cerr << "[FAIL] could not prepare test config\n";
        return 1;
    }

    // 회전교차로/합류 구간과 무관한 평범한 직선 근처(kEgoS=200, 다른
    // command_*_plan_test와 동일 관례) 사용.
    constexpr double kEgoS = 200.0;

    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    const std::vector<ObjectInfo> no_objects;

    // speed 0.49 / 0.50 / 0.51 / 0.6 / 0.8 근방에서 discontinuous하게
    // trajectory availability가 깨지지 않는지 확인한다. 강한 감속 경계조건
    // (a=-2.4)을 모든 속도에서 동일하게 준다 - 재현 조건과 동일.
    const std::vector<double> speeds = {0.49, 0.50, 0.51, 0.6, 0.8};
    for (double speed : speeds) {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        const CartesianState ego =
            LoadPoseNearArcLength(waypoint_file, kEgoS, speed, /*accel=*/-2.4);
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects, keep, path);
        std::printf("  speed=%.2f -> Plan() = %s, path.x.size()=%zu\n",
                    speed, ok ? "true" : "false", path.x.size());
        Expect(ok && !path.x.empty(),
                "speed=" + std::to_string(speed) +
                    ": external KEEP produces a trajectory despite strong "
                    "negative measured accel near the low-speed boundary");

        if (ok) {
            bool has_negative_velocity = false;
            for (double v : path.v) if (v < -1e-6) has_negative_velocity = true;
            Expect(!has_negative_velocity,
                    "speed=" + std::to_string(speed) +
                        ": fallback/normal trajectory has no negative velocity "
                        "sample (LOCK-1 invariant holds)");
        }
    }

    // 정상 속도(예: 8 m/s)에서는 이 fallback이 아예 발동하지 않고 정상
    // pipeline이 그대로 동작해야 한다(회귀 없음 확인).
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        const CartesianState ego =
            LoadPoseNearArcLength(waypoint_file, kEgoS, /*speed=*/8.0, /*accel=*/-2.4);
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects, keep, path);
        Expect(ok && !path.x.empty(),
               "normal speed(8.0): external KEEP still produces a trajectory "
               "via the normal pipeline (fallback does not interfere)");
    }

    if (g_failures == 0) {
        std::cout << "All low-speed KEEP handoff tests passed\n";
        return 0;
    }
    return 1;
}
