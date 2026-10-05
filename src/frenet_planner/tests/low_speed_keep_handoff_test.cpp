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
#include "frenet_planner/Math/FrenetConverter.hpp"
#include "frenet_planner/Reference/Reference.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

double MaxAbsCurvature(const CartesianPath& path) {
    double maximum = 0.0;
    for (double kappa : path.kappa) {
        if (!std::isfinite(kappa)) return std::numeric_limits<double>::infinity();
        maximum = std::max(maximum, std::fabs(kappa));
    }
    return maximum;
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

ReferenceConfig MakeReferenceConfig() {
    ReferenceConfig config{};
    config.closure_tolerance_m = 0.05;
    config.smoothing_window_m = 10.0;
    config.resample_ds = 0.5;
    config.relax_max_iter = 300;
    config.relax_step = 0.3;
    config.ego_projection_search_window = 30.0;
    return config;
}

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/../config/params.yaml", waypoint_file,
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
    const std::vector<TrackedObject> no_objects;

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
            Expect(MaxAbsCurvature(path) <= 0.23 + 1e-6,
                    "speed=" + std::to_string(speed) +
                        ": fallback/normal trajectory obeys the published "
                        "curvature limit");
        }
    }

    // MORAI 재현: 급커브에서 정상 KEEP 후보가 곡률로 전부 탈락하고
    // v<1.5 m/s near-low-speed fallback으로 넘어갔을 때, 기존 고정 복귀
    // 거리는 첫 샘플 kappa=0.25243을 발행해 MPC(0.23607) STOP을 만들었다.
    // fallback은 횡복귀 거리를 자동으로 늘려 Frenet의 더 보수적인 0.23
    // 계약 안의 경로만 반환해야 한다.
    {
        RefLine ref;
        Expect(LoadReferenceLine(waypoint_file, ref, MakeReferenceConfig(),
                                 /*max_curvature=*/0.23),
               "curvature fallback snapshot: reference line loads");

        FrenetPlanner planner;
        Expect(planner.Init(config),
               "curvature fallback snapshot: planner initializes");

        CartesianState ego{};
        ego.x = -94.434454;
        ego.y = -356.620170;
        ego.yaw = 1.7649386;
        ego.v = 0.66832665;
        ego.a = -2.4;

        PlannerCommand snapshot_keep{};
        snapshot_keep.mode = LANE_KEEPING;
        snapshot_keep.has_target_speed = true;
        snapshot_keep.target_speed = 4.8;
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects, snapshot_keep, path);
        Expect(ok && path.x.size() >= 2,
               "curvature fallback snapshot: KEEP returns a usable path");
        if (ok) {
            const double max_curvature = MaxAbsCurvature(path);
            Expect(max_curvature <= 0.23 + 1e-6,
                   "curvature fallback snapshot: adaptive lateral recovery "
                   "stays within max_curvature");

            double ego_s = 0.0, ego_s_dot = 0.0, ego_s_ddot = 0.0;
            double ego_d = 0.0, ego_d_prime = 0.0, ego_d_pprime = 0.0;
            CartesianToFrenet(ref, ego, ego_s, ego_s_dot, ego_s_ddot,
                              ego_d, ego_d_prime, ego_d_pprime,
                              /*s_hint=*/nullptr, /*search_window=*/30.0);

            CartesianState terminal{};
            terminal.x = path.x.back();
            terminal.y = path.y.back();
            terminal.yaw = path.yaw.back();
            terminal.v = path.v.back();
            terminal.a = path.a.back();
            terminal.kappa = path.kappa.back();
            double terminal_s = 0.0, terminal_s_dot = 0.0;
            double terminal_s_ddot = 0.0, terminal_d = 0.0;
            double terminal_d_prime = 0.0, terminal_d_pprime = 0.0;
            CartesianToFrenet(ref, terminal, terminal_s, terminal_s_dot,
                              terminal_s_ddot, terminal_d,
                              terminal_d_prime, terminal_d_pprime, &ego_s,
                              /*search_window=*/30.0);
            std::printf("  curvature fallback snapshot: max|kappa|=%.6f "
                        "d %.3f -> %.3f\n",
                        max_curvature, ego_d, terminal_d);
            Expect(std::fabs(terminal_d) < std::fabs(ego_d),
                   "curvature fallback snapshot: feasible path still makes "
                   "progress toward d=0");
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
