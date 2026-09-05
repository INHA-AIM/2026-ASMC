// Distance-aware curve speed envelope 회귀 (fix/yuntae-frenet-speed-continuity
// Phase 1).
//
// 이 테스트가 지키는 계약:
//   TEST 1: 30m 앞 sharp curve가 있어도 현재 target이 바로 final corner
//           speed로 떨어지지 않는다.
//   TEST 2: curve에 가까워질수록 target speed가 점진적으로 감소한다.
//   TEST 3: curve 진입점 근처에서는 local curve-safe speed까지 충분히
//           감소한다.
//   TEST 4: straight road에서는 target speed가 planner max speed(target_vel)를
//           유지한다.
//   TEST 5: closed-loop seam(wrap-around)에서도 동일하게 동작한다.
//   TEST 6: future curve가 더 급하면 더 낮은 current allowed speed가
//           선택된다.
//
// 실제 production LookaheadTargetSpeed/VelocityFromCurvature
// (frenet/curve_speed.hpp)를 그대로 호출한다 - 로직을 복사하지 않는다.
// RefLine은 waypoint smoothing/resampling을 우회해 s/kappa를 직접
// 통제하기 위해 손으로 구성한다.

#include "frenet_planner/frenet/curve_speed.hpp"

#include <cmath>
#include <functional>
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

CurveSpeedConfig MakeCurveCfg() {
    CurveSpeedConfig cfg;
    cfg.target_vel = 10.0;
    cfg.curve_lookahead_m = 15.0;
    return cfg;
}

// s=0..total_s, dt=1m 간격 균일 점. kappa는 kappa_fn(s)로 지정.
RefLine MakeRefLine(double total_s, double step,
                     const std::function<double(double)>& kappa_fn) {
    RefLine ref;
    for (double s = 0.0; s <= total_s + 1e-9; s += step) {
        RefPoint p{};
        p.x = s;
        p.y = 0.0;
        p.s = s;
        p.theta = 0.0;
        p.kappa = kappa_fn(s);
        p.d_kappa = 0.0;
        ref.points.push_back(p);
    }
    return ref;
}

}  // namespace

// TEST 1 + 3 - 30m 앞 sharp curve, 즉시 corner speed로 떨어지지 않고
// 진입점 근처에서는 local curve-safe speed까지 충분히 감소.
void TestFarSharpCurveDoesNotForceImmediateCornerSpeed() {
    CurveSpeedConfig cfg = MakeCurveCfg();
    const double max_lateral_accel = 3.0;
    const double max_longitudinal_accel = 3.0;
    const double max_curvature = 0.1704;

    // sharp curve: kappa=0.15 (radius ~6.7m) 구간이 s=30~40에 있고
    // 그 외에는 직선(kappa=0).
    auto kappa_fn = [](double s) { return (s >= 30.0 && s <= 40.0) ? 0.15 : 0.0; };
    RefLine ref = MakeRefLine(120.0, 1.0, kappa_fn);

    const double corner_speed = VelocityFromCurvature(0.15, cfg, max_lateral_accel);
    std::cout << "  corner_speed (kappa=0.15) = " << corner_speed << '\n';

    // ego가 커브에서 30m 떨어진 지점(s=0)에서 8 m/s로 주행 중.
    const double far_target = LookaheadTargetSpeed(
        ref, /*s_start=*/0.0, /*ego_speed=*/8.0, cfg,
        max_longitudinal_accel, max_lateral_accel, max_curvature);
    std::cout << "  far_target (30m before curve) = " << far_target << '\n';
    Expect(far_target > corner_speed + 0.5,
           "TEST1: 30m 앞 sharp curve가 있어도 즉시 corner speed로 "
           "떨어지지 않는다");

    // 커브 진입점(s=29, 커브 시작 바로 앞)에서는 local curve-safe speed에
    // 충분히 근접해야 한다.
    const double near_target = LookaheadTargetSpeed(
        ref, /*s_start=*/29.0, /*ego_speed=*/8.0, cfg,
        max_longitudinal_accel, max_lateral_accel, max_curvature);
    std::cout << "  near_target (1m before curve) = " << near_target << '\n';
    Expect(near_target < far_target,
           "TEST1: 커브에 더 가까운 지점의 target이 더 멀리 있을 때보다 낮다");
    Expect(near_target <= corner_speed + 1.0,
           "TEST3: 커브 진입점 근처에서는 local curve-safe speed까지 "
           "충분히 감소한다");
}

// TEST 2 - curve에 가까워질수록 target speed가 점진적으로(단조) 감소한다.
void TestTargetSpeedDecreasesMonotonicallyApproachingCurve() {
    CurveSpeedConfig cfg = MakeCurveCfg();
    const double max_lateral_accel = 3.0;
    const double max_longitudinal_accel = 3.0;
    const double max_curvature = 0.1704;

    auto kappa_fn = [](double s) { return (s >= 60.0 && s <= 70.0) ? 0.15 : 0.0; };
    RefLine ref = MakeRefLine(150.0, 1.0, kappa_fn);

    double prev = std::numeric_limits<double>::infinity();
    bool monotonic = true;
    std::vector<double> samples;
    for (double s_start = 0.0; s_start <= 59.0; s_start += 5.0) {
        const double v = LookaheadTargetSpeed(ref, s_start, 8.0, cfg,
                                               max_longitudinal_accel,
                                               max_lateral_accel, max_curvature);
        samples.push_back(v);
        if (v > prev + 1e-6) monotonic = false;
        prev = v;
    }
    std::cout << "  progressive samples (s=0..59, step 5):";
    for (double v : samples) std::cout << ' ' << v;
    std::cout << '\n';
    Expect(monotonic,
           "TEST2: curve에 가까워질수록(s_start 증가) target speed가 "
           "단조 감소한다");
}

// TEST 4 - straight road: target speed가 planner max speed를 유지한다.
void TestStraightRoadKeepsMaxSpeed() {
    CurveSpeedConfig cfg = MakeCurveCfg();
    const double max_lateral_accel = 3.0;
    const double max_longitudinal_accel = 3.0;
    const double max_curvature = 0.1704;

    RefLine ref = MakeRefLine(200.0, 1.0, [](double) { return 0.0; });
    const double v = LookaheadTargetSpeed(ref, 50.0, 8.0, cfg,
                                          max_longitudinal_accel,
                                          max_lateral_accel, max_curvature);
    std::cout << "  straight road target = " << v << '\n';
    Expect(std::fabs(v - cfg.target_vel) < 1e-6,
           "TEST4: straight road에서는 target speed == target_vel");
}

// TEST 5 - closed-loop seam(wrap-around)에서도 동일하게 동작한다.
void TestClosedLoopSeamWrapsCorrectly() {
    CurveSpeedConfig cfg = MakeCurveCfg();
    const double max_lateral_accel = 3.0;
    const double max_longitudinal_accel = 3.0;
    const double max_curvature = 0.1704;

    // 트랙 총 길이 100m, sharp curve가 이음새(s=0 부근, 즉 마지막 근처에서
    // 감아 이어짐) 바로 너머(s=5~10)에 있다.
    const double total_s = 100.0;
    auto kappa_fn = [](double s) { return (s >= 5.0 && s <= 10.0) ? 0.15 : 0.0; };
    RefLine ref = MakeRefLine(total_s, 1.0, kappa_fn);

    const double corner_speed = VelocityFromCurvature(0.15, cfg, max_lateral_accel);

    // ego가 피니시라인 근처(s=95, 즉 이음새 너머 5m 앞의 커브까지 10m)에서 주행.
    const double v = LookaheadTargetSpeed(ref, /*s_start=*/95.0, /*ego_speed=*/8.0,
                                          cfg, max_longitudinal_accel,
                                          max_lateral_accel, max_curvature);
    std::cout << "  seam-wrapped target (s_start=95, curve at wrapped s=5..10) = "
              << v << '\n';
    Expect(v < cfg.target_vel - 0.5,
           "TEST5: 폐루프 이음새 너머의 커브도 감지되어 감속이 시작된다");
    Expect(v >= corner_speed - 1e-6,
           "TEST5: 이음새 wrap-around 후에도 결과가 물리적으로 타당한 "
           "범위(corner_speed 이상) 안에 있다");
}

// TEST 6 - future curve가 더 급하면(kappa가 더 크면) 더 낮은 current
// allowed speed가 선택된다.
void TestSharperFutureCurveYieldsLowerAllowedSpeed() {
    CurveSpeedConfig cfg = MakeCurveCfg();
    const double max_lateral_accel = 3.0;
    const double max_longitudinal_accel = 3.0;
    const double max_curvature = 0.1704;

    // 두 curve 모두 동일 위치(s=15..20)에 두어 lookahead window(약 21.6m,
    // ego_speed=8.0 기준) 안에 반드시 들어오게 한다. kappa 크기만 다르다.
    RefLine mild_ref = MakeRefLine(
        100.0, 1.0, [](double s) { return (s >= 15.0 && s <= 20.0) ? 0.05 : 0.0; });
    RefLine sharp_ref = MakeRefLine(
        100.0, 1.0, [](double s) { return (s >= 15.0 && s <= 20.0) ? 0.15 : 0.0; });

    const double v_mild = LookaheadTargetSpeed(mild_ref, 0.0, 8.0, cfg,
                                               max_longitudinal_accel,
                                               max_lateral_accel, max_curvature);
    const double v_sharp = LookaheadTargetSpeed(sharp_ref, 0.0, 8.0, cfg,
                                                max_longitudinal_accel,
                                                max_lateral_accel, max_curvature);
    std::cout << "  v_mild(kappa=0.05) = " << v_mild
              << ", v_sharp(kappa=0.15) = " << v_sharp << '\n';
    Expect(v_sharp < v_mild,
           "TEST6: 더 급한 미래 커브(kappa 큼)가 더 낮은 current allowed "
           "speed를 만든다");
}

int main() {
    TestFarSharpCurveDoesNotForceImmediateCornerSpeed();
    TestTargetSpeedDecreasesMonotonicallyApproachingCurve();
    TestStraightRoadKeepsMaxSpeed();
    TestClosedLoopSeamWrapsCorrectly();
    TestSharperFutureCurveYieldsLowerAllowedSpeed();

    if (g_failures == 0) {
        std::cout << "[ALL PASS]\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
