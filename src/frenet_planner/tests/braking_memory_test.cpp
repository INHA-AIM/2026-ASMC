// KEEP replanning braking-memory 회귀
// (fix/yuntae-frenet-speed-continuity Phase A).
//
// 재현 조건: CartesianToFrenet이 실측 IMU 가속도를 역산한 s_ddot을
// PlanInternal이 ±max_longitudinal_accel*0.8(=2.4)로 clamp해 새 velocity
// keeping quartic의 초기 가속도 경계조건으로 그대로 사용한다
// (frenet_planner.cpp 1548-1550행). target_speed가 현재 속도보다 계속
// 높아져도(가속 의도) 직전 강한 제동의 s_ddot=-2.4가 매 cycle 다시 주입되면
// 선택된 후보의 근거리 속도가 계속 깎이고, 반복 replanning에서는 monotonic
// collapse로 이어진다(실제 MORAI: target 7.67->9.72인데 실제 속도
// 6.05->1.11).
//
// 이 테스트는 실제 production FrenetPlanner::Plan()을 그대로 호출한다 -
// 로직을 복사하지 않는다. 완화 로직(ResolveVelocityKeepingBoundaryAcceleration)
// 자체의 단위 계약은 curve_speed.hpp에서 직접 검증한다.

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/Speed/CurveSpeed.hpp"

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

constexpr double kEgoS = 200.0;  // 다른 command_*_plan_test와 동일 관례

}  // namespace

// CASE A: current=5, target=9~10, raw accel=-2.4.
// before: selected near-term velocity decreases.
// after: 이유 없이 감소하지 않는다(v[0]에서 이미 current 이상 또는 target
// 방향으로 유지되어야 한다 - 최소한 raw accel 그대로 주입되던 이전처럼
// 매 cycle 5~8% 씩 깎이지 않아야 한다).
void TestCaseA_NoUnwarrantedDecelerationWhenAccelerating() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "braking_memory_test_case_a_params.yaml");

    for (double target : {9.0, 10.0}) {
        FrenetPlanner planner;
        planner.Init(config);
        const CartesianState ego =
            LoadPoseNearArcLength(waypoint_file, kEgoS, /*speed=*/5.0, /*accel=*/-2.4);
        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        keep.has_target_speed = true;
        keep.target_speed = target;
        const std::vector<ObjectInfo> no_objects;
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects, keep, path);
        Expect(ok && !path.v.empty(),
               "CASE A target=" + std::to_string(target) +
                   ": external KEEP produces a trajectory");
        if (ok && path.v.size() > 1) {
            std::printf("  CASE A target=%.1f: v[0]=%.4f v[1]=%.4f (current=5.00)\n",
                        target, path.v[0], path.v[1]);
            Expect(path.v[1] >= 5.0 - 1e-6,
                   "CASE A target=" + std::to_string(target) +
                       ": v[1] does not decrease below current speed when "
                       "target > current (braking-memory relaxed)");
        }
    }
}

// CASE B: repeated 20Hz replanning with persistent raw negative accel.
// before: velocity can monotonically collapse.
// after: target > current인 동안 speed reference가 계속 collapse하지 않는다.
void TestCaseB_RepeatedReplanningDoesNotCollapse() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "braking_memory_test_case_b_params.yaml");

    FrenetPlanner planner;
    planner.Init(config);
    const std::vector<ObjectInfo> no_objects;
    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 9.0;

    double speed = 5.0;
    constexpr double kCyclePeriod = 0.05;
    constexpr int kNumCycles = 40;
    double min_seen = speed;

    std::printf("  CASE B chain: target=9.0 (constant, > current), raw accel=-2.40 "
                "re-injected every cycle\n");
    for (int cycle = 0; cycle < kNumCycles; ++cycle) {
        const CartesianState ego =
            LoadPoseNearArcLength(waypoint_file, kEgoS, speed, /*accel=*/-2.4);
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects, keep, path);
        if (!ok || path.v.size() < 2) {
            std::printf("  cycle %2d: Plan() failed or too few samples\n", cycle);
            break;
        }
        // sample_dt는 production dt(0.1s)와 같으므로 0.05s 지점은 v[0]/v[1]의
        // 중점으로 근사한다.
        const double next_speed = 0.5 * (path.v[0] + path.v[1]);
        std::printf("  cycle %2d: speed %.4f -> %.4f\n", cycle, speed, next_speed);
        speed = next_speed;
        min_seen = std::min(min_seen, speed);
    }
    (void)kCyclePeriod;
    std::printf("  final speed = %.4f, min_seen = %.4f, initial = 5.0000\n",
                speed, min_seen);
    Expect(speed >= 5.0 - 1e-3,
           "CASE B: after repeated replanning with persistent negative raw "
           "accel, speed reference does not collapse below the initial "
           "speed while target > current");
}

// CASE C: target < current (real deceleration) - existing negative s_ddot
// boundary must NOT be forced to 0.
void TestCaseC_RealDecelerationBoundaryPreserved() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "braking_memory_test_case_c_params.yaml");

    FrenetPlanner planner;
    planner.Init(config);
    const CartesianState ego =
        LoadPoseNearArcLength(waypoint_file, kEgoS, /*speed=*/9.0, /*accel=*/-2.4);
    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 5.0;  // target < current: real deceleration intent
    const std::vector<ObjectInfo> no_objects;
    CartesianPath path;
    const bool ok = planner.Plan(ego, no_objects, keep, path);
    Expect(ok && !path.v.empty(), "CASE C: external KEEP produces a trajectory");
    if (ok && path.v.size() > 1) {
        std::printf("  CASE C: v[0]=%.4f v[1]=%.4f (current=9.00, target=5.00)\n",
                    path.v[0], path.v[1]);
        Expect(path.v[1] < 9.0 - 1e-6,
               "CASE C: when target < current, deceleration still happens "
               "(boundary was not forced to 0)");
    }
}

// CASE D: negative velocity guard preserved (LOCK).
void TestCaseD_NegativeVelocityGuardPreserved() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "braking_memory_test_case_d_params.yaml");

    FrenetPlanner planner;
    planner.Init(config);
    const CartesianState ego =
        LoadPoseNearArcLength(waypoint_file, kEgoS, /*speed=*/5.0, /*accel=*/-2.4);
    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 10.0;
    const std::vector<ObjectInfo> no_objects;
    CartesianPath path;
    const bool ok = planner.Plan(ego, no_objects, keep, path);
    Expect(ok, "CASE D: plan succeeds");
    bool has_negative = false;
    for (double v : path.v) if (v < -1e-6) has_negative = true;
    Expect(!has_negative, "CASE D: no negative velocity sample (LOCK-1 preserved)");
}

// CASE E/F: jerk<=6.0 and max longitudinal accel preserved (LOCK) across the
// relaxed-boundary path.
void TestCaseEF_JerkAndAccelLimitsPreserved() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "braking_memory_test_case_ef_params.yaml");

    FrenetPlanner planner;
    planner.Init(config);
    const CartesianState ego =
        LoadPoseNearArcLength(waypoint_file, kEgoS, /*speed=*/2.5, /*accel=*/-2.4);
    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 9.0;
    const std::vector<ObjectInfo> no_objects;
    CartesianPath path;
    const bool ok = planner.Plan(ego, no_objects, keep, path);
    Expect(ok, "CASE E/F: plan succeeds");
    (void)ok;
    // CartesianPath에는 jerk가 없으므로 accel(a)만 직접 확인한다. jerk 자체는
    // longitudinal_kinematics_test/FilterLongitudinalByKinematicLimits가 이미
    // 검증하며, 여기서는 이 완화 경로가 accel 한계를 넘지 않는지만 재확인한다.
    double max_abs_accel = 0.0;
    for (double a : path.a) max_abs_accel = std::max(max_abs_accel, std::fabs(a));
    std::printf("  CASE E/F: max|a|=%.4f (limit 3.0)\n", max_abs_accel);
    Expect(max_abs_accel <= 3.0 + 1e-6,
           "CASE E/F: max longitudinal accel limit preserved on the relaxed "
           "boundary path");
}

// CASE G: no new normal-KEEP candidate starvation - normal-speed KEEP with
// zero/positive accel still produces a trajectory (regression check).
void TestCaseG_NoNormalKeepStarvation() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/frenet_planner/config/params.yaml", waypoint_file,
        "braking_memory_test_case_g_params.yaml");

    FrenetPlanner planner;
    planner.Init(config);
    const CartesianState ego =
        LoadPoseNearArcLength(waypoint_file, kEgoS, /*speed=*/8.0, /*accel=*/0.0);
    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 8.0;
    const std::vector<ObjectInfo> no_objects;
    CartesianPath path;
    const bool ok = planner.Plan(ego, no_objects, keep, path);
    Expect(ok && !path.v.empty(),
           "CASE G: normal-speed KEEP (accel=0, target==current) still "
           "produces a trajectory (no new starvation)");
}

// Unit-level contract check for ResolveVelocityKeepingBoundaryAcceleration
// itself (production function, not reimplemented).
void TestHelperContract() {
    // accelerating intent + negative raw accel -> relaxed to 0.
    Expect(ResolveVelocityKeepingBoundaryAcceleration(-2.4, 5.0, 9.0, 0.3) == 0.0,
           "helper: relaxes to 0 when target > current+deadband and raw accel < 0");
    // target <= current -> raw accel preserved unchanged.
    Expect(ResolveVelocityKeepingBoundaryAcceleration(-2.4, 9.0, 5.0, 0.3) == -2.4,
           "helper: preserves raw accel unchanged when target <= current "
           "(real deceleration)");
    // target within deadband of current -> not treated as accelerating intent.
    Expect(ResolveVelocityKeepingBoundaryAcceleration(-2.4, 8.9, 9.0, 0.3) == -2.4,
           "helper: preserves raw accel when target is within the deadband "
           "of current (not a clear accelerating intent)");
    // already non-negative raw accel -> unchanged regardless of intent.
    Expect(ResolveVelocityKeepingBoundaryAcceleration(1.0, 5.0, 9.0, 0.3) == 1.0,
           "helper: leaves non-negative raw accel unchanged");
}

int main() {
    TestHelperContract();
    TestCaseA_NoUnwarrantedDecelerationWhenAccelerating();
    TestCaseB_RepeatedReplanningDoesNotCollapse();
    TestCaseC_RealDecelerationBoundaryPreserved();
    TestCaseD_NegativeVelocityGuardPreserved();
    TestCaseEF_JerkAndAccelLimitsPreserved();
    TestCaseG_NoNormalKeepStarvation();

    if (g_failures == 0) {
        std::cout << "All braking-memory regression tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
