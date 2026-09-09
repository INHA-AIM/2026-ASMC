// Commit F: 폐루프(닫힌 트랙) reference topology 회귀.
//
// 배경: 대회 global path는 첫점/끝점이 물리적으로 같은 닫힌 트랙이다. 그런데
// BuildRefLine의 전처리(smoothing/resampling/derivative)는 열린 경로처럼
// 동작했고, Interpolate/FindClosestS/CurveSpeed의 조회만 closed-loop wrap을
// 가정했다 - 전처리와 조회 topology가 서로 달랐다. 그 결과 실제 MORAI
// 완주 후 피니시라인을 통과하는 순간 참조선 자체가 불연속(수 m 순간이동,
// heading/curvature 스파이크)이었고, 그 위에서 계획된 trajectory도 같이
// 깨졌다.
//
// 이 테스트는 RefLine이 명시적 topology(closed_loop/length)를 갖고,
// 폐루프 전처리/조회가 이음새에서 실제로 연속인지 검증한다. production
// BuildRefLine/LoadReferenceLine/Interpolate/FindClosestS/
// LookaheadTargetSpeed/FrenetPlanner::Plan을 그대로 호출한다 - 로직을
// 복사하지 않는다.

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/Reference/Reference.hpp"
#include "frenet_planner/Speed/CurveSpeed.hpp"

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

constexpr double kMaxCurvature = 0.1704;  // production params.yaml과 동일

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

double NormalizedAngleDiff(double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

double Dist(const RefPoint& a, const RefPoint& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

// Test 18.1 - 폐루프 감지: 실제 대회 waypoint는 closed_loop=true,
// 열린 synthetic path는 closed_loop=false.
void TestClosedLoopDetection() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine closed_ref;
    Expect(LoadReferenceLine(waypoint_file, closed_ref, kMaxCurvature),
           "setup: real competition waypoint loads");
    Expect(closed_ref.closed_loop,
           "18.1: real competition waypoint is detected as closed_loop=true");

    // 열린 synthetic path: 직선 100m, 첫점/끝점이 멀리 떨어져 있다.
    const std::vector<double> open_x = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};
    const std::vector<double> open_y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const RefLine open_ref = BuildRefLine(open_x, open_y, kMaxCurvature);
    Expect(!open_ref.closed_loop,
           "18.1: open synthetic path is detected as closed_loop=false");
}

// Test 18.2 - 전처리 후 물리적 폐합: front()/back() 위치가 거의 같아야 한다.
void TestPhysicalClosureAfterPreprocessing() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    Expect(LoadReferenceLine(waypoint_file, ref, kMaxCurvature),
           "setup: reference loads for closure check");
    Expect(ref.closed_loop, "18.2 setup: reference is closed_loop");

    const double gap = Dist(ref.points.front(), ref.points.back());
    std::printf("  seam physical gap after preprocessing = %.6f m\n", gap);
    // 재샘플링 간격(0.5m)보다 훨씬 작아야 한다 - 몇 m대 seam gap이 있었다면
    // (수정 전 버그) 이 검사가 실패한다.
    Expect(gap < 0.05,
           "18.2: front()/back() are physically the same point after "
           "preprocessing (no multi-meter seam gap)");
}

// Test 18.3 - loop length: length>0, front().s==0, back().s ~= length.
void TestLoopLength() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);

    Expect(ref.length > 0.0, "18.3: ref.length is positive");
    Expect(std::fabs(ref.points.front().s) < 1e-9,
           "18.3: front().s == 0");
    Expect(std::fabs(ref.points.back().s - ref.length) < 1e-6,
           "18.3: back().s approximately equals ref.length");
}

// Test 18.4 - 이음새 공간 연속성: length 근방과 0 근방 사이에 텔레포트가
// 없다. 임계값은 국소 재샘플링 간격(0.5m) 기준으로 정한다 - 임의의 큰
// 수가 아니다.
void TestSeamSpatialContinuity() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);
    const double length = ref.length;
    constexpr double kEps = 0.3;
    // 재샘플링 간격은 production kResampleDs(0.5m)와 같은 자릿수 - 정상적인
    // 인접 샘플 이동 거리의 몇 배를 넘는 점프는 seam teleport로 본다.
    constexpr double kMaxAdjacentJump = 1.5;  // [m] ~= 3 * kResampleDs

    const RefPoint p_before = Interpolate(ref, length - kEps);
    const RefPoint p_at = Interpolate(ref, length);
    const RefPoint p_zero = Interpolate(ref, 0.0);
    const RefPoint p_after = Interpolate(ref, kEps);

    std::printf("  seam positions: (%.3f,%.3f) -> (%.3f,%.3f) -> (%.3f,%.3f) -> (%.3f,%.3f)\n",
               p_before.x, p_before.y, p_at.x, p_at.y, p_zero.x, p_zero.y,
               p_after.x, p_after.y);

    Expect(Dist(p_before, p_at) < kMaxAdjacentJump,
           "18.4: length-eps -> length has no teleport");
    Expect(Dist(p_at, p_zero) < 1e-6,
           "18.4: length and 0 are the exact same physical point");
    Expect(Dist(p_zero, p_after) < kMaxAdjacentJump,
           "18.4: 0 -> eps has no teleport");
}

// Test 18.5 - heading 연속성: theta(length-eps)와 theta(eps)를 정규화된
// 각도 차이로 비교한다(+pi/-pi 원시값 비교 금지).
void TestHeadingContinuity() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);
    const double length = ref.length;
    constexpr double kEps = 0.3;

    const double theta_before = Interpolate(ref, length - kEps).theta;
    const double theta_after = Interpolate(ref, kEps).theta;
    const double diff = std::fabs(NormalizedAngleDiff(theta_before, theta_after));
    std::printf("  seam heading: before=%.4f after=%.4f normalized diff=%.4f rad\n",
               theta_before, theta_after, diff);
    // 2*kEps 구간에서 급격한 트랙 형상이 아니라면 heading 변화가 커도
    // 0.3 rad(~17도)를 넘지 않아야 한다 - production 트랙의 국소 곡률
    // 한계(kMaxCurvature=0.1704) 하에서 0.6m 구간이 만들 수 있는 최대
    // 회전은 0.1704*0.6 ~= 0.1 rad 수준이다. 여유를 둬 0.3 rad로 검사한다.
    Expect(diff < 0.3,
           "18.5: heading is geometrically continuous across the seam");
}

// Test 18.6 - 곡률 연속성: seam 양쪽 kappa가 인위적으로 튀지 않고, 여전히
// max_curvature 안에 있다.
void TestCurvatureContinuity() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);
    const double length = ref.length;
    constexpr double kEps = 0.3;

    const double kappa_before = Interpolate(ref, length - kEps).kappa;
    const double kappa_after = Interpolate(ref, kEps).kappa;
    std::printf("  seam curvature: before=%.5f after=%.5f\n", kappa_before, kappa_after);

    Expect(std::isfinite(kappa_before) && std::isfinite(kappa_after),
           "18.6: seam curvature values are finite");
    Expect(std::fabs(kappa_before) <= kMaxCurvature + 1e-6,
           "18.6: curvature just before the seam respects max_curvature");
    Expect(std::fabs(kappa_after) <= kMaxCurvature + 1e-6,
           "18.6: curvature just after the seam respects max_curvature");
    Expect(std::fabs(kappa_before - kappa_after) < 0.05,
           "18.6: no artificial curvature spike across the seam");
}

// Test 18.7 - d_kappa가 seam 근방에서 finite하고 endpoint 전용 인공물이
// 없다.
void TestDKappaContinuity() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);
    const double length = ref.length;

    bool all_finite = true;
    for (double ds = -2.0; ds <= 2.0; ds += 0.5) {
        const RefPoint rp = Interpolate(ref, length + ds);
        if (!std::isfinite(rp.d_kappa)) all_finite = false;
    }
    Expect(all_finite, "18.7: d_kappa is finite throughout the seam neighborhood");

    // 이음점(복제 endpoint) 자체와 그 이웃의 d_kappa가 크게 어긋나지
    // 않아야 한다(편측 endpoint 처리 인공물이 없다는 뜻).
    const double dk_at_seam = ref.points.back().d_kappa;
    const double dk_at_front = ref.points.front().d_kappa;
    Expect(std::fabs(dk_at_seam - dk_at_front) < 1e-9,
           "18.7: duplicated closing point's d_kappa matches the front point exactly");
}

// Test 18.8 - Interpolate wrap 회귀. 폐루프: length+delta == delta,
// -delta == length-delta. 열린 경로: wrap하지 않는다(clamp).
void TestInterpolateWrapRegression() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);
    const double length = ref.length;
    constexpr double kDelta = 3.0;

    const RefPoint a = Interpolate(ref, length + kDelta);
    const RefPoint b = Interpolate(ref, kDelta);
    Expect(Dist(a, b) < 1e-6,
           "18.8: closed loop Interpolate(length+delta) matches Interpolate(delta)");

    const RefPoint c = Interpolate(ref, -kDelta);
    const RefPoint d = Interpolate(ref, length - kDelta);
    Expect(Dist(c, d) < 1e-6,
           "18.8: closed loop Interpolate(-delta) matches Interpolate(length-delta)");

    // 열린 경로: length를 넘어가도 clamp될 뿐 wrap하지 않는다.
    const std::vector<double> open_x = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};
    const std::vector<double> open_y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const RefLine open_ref = BuildRefLine(open_x, open_y, kMaxCurvature);
    const RefPoint beyond = Interpolate(open_ref, open_ref.length + 100.0);
    Expect(Dist(beyond, open_ref.points.back()) < 1e-6,
           "18.8: open path clamps beyond the end instead of wrapping");
    const RefPoint before_start = Interpolate(open_ref, -100.0);
    Expect(Dist(before_start, open_ref.points.front()) < 1e-6,
           "18.8: open path clamps before the start instead of wrapping");
}

// Test 18.9 - FindClosestS seam 회귀. length 근방 ~ 0 근방을 지나가는
// 실제 Cartesian pose를 만들어 투영한다. 순환 거리로 연속성을 판단하고,
// 전역 단조 wrapped-s는 요구하지 않는다(length -> 0은 수학적으로 유효).
void TestProjectionSeamRegression() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);
    const double length = ref.length;

    const std::vector<double> offsets = {
        length - 3.0, length - 2.0, length - 1.0, length - 0.5,
        0.0, 0.5, 1.0, 2.0, 3.0};

    double prev_s = length - 3.5;  // 첫 hint: 트랙 위 임의의 근방 값
    bool any_large_jump = false;
    for (double s_query : offsets) {
        const RefPoint rp = Interpolate(ref, s_query);
        Expect(std::isfinite(rp.x) && std::isfinite(rp.y),
               "18.9: seam sample pose is finite");

        const double projected = FindClosestS(ref, rp.x, rp.y, &prev_s);
        Expect(std::isfinite(projected), "18.9: FindClosestS returns a finite s");

        // 순환 거리로 판단: 투영된 s와 실제 알고 있는 s_query(모듈로 length)
        // 사이의 최短 원형 거리가 재샘플링 간격 수준이어야 한다.
        double expected = std::fmod(s_query, length);
        if (expected < 0.0) expected += length;
        double diff = std::fabs(projected - expected);
        double circ_diff = std::min(diff, length - diff);
        if (circ_diff > 1.0) any_large_jump = true;

        std::printf("  project s_query=%.2f -> projected=%.4f (expected~=%.4f, circ_diff=%.4f)\n",
                   s_query, projected, expected, circ_diff);
        prev_s = projected;
    }
    Expect(!any_large_jump,
           "18.9: no large jump to an unrelated branch when projecting across the seam");
}

// Test 18.10 - production KEEP trajectory가 seam을 가로지를 때도 물리적으로
// 타당해야 한다. 이것이 이 회귀의 핵심이다.
void TestProductionKeepTrajectoryCrossesSeam() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/../config/params.yaml", waypoint_file,
        "closed_loop_reference_test_params.yaml");

    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);
    Expect(ref.closed_loop, "18.10 setup: production reference is closed_loop");

    FrenetPlanner planner;
    Expect(planner.Init(config), "18.10: planner initializes");

    // s = length - 5m, d~=0, speed~=8-10 m/s, KEEP.
    const double start_s = ref.length - 5.0;
    const RefPoint rp = Interpolate(ref, start_s);
    CartesianState ego{};
    ego.x = rp.x;
    ego.y = rp.y;
    ego.yaw = rp.theta;
    ego.kappa = 0.0;
    ego.v = 9.0;
    ego.a = 0.0;

    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 9.0;
    const std::vector<TrackedObject> no_objects;
    CartesianPath path;
    const bool ok = planner.Plan(ego, no_objects, keep, path);
    std::printf("  18.10: Plan() = %s, path.x.size()=%zu\n", ok ? "true" : "false",
               path.x.size());
    Expect(ok && !path.x.empty(),
           "18.10: KEEP plan succeeds when the trajectory crosses the seam");

    if (!ok || path.x.empty()) return;

    bool all_finite = true;
    bool jump_ok = true;
    bool speed_ok = true;
    bool curvature_ok = true;
    for (std::size_t i = 0; i < path.x.size(); ++i) {
        if (!std::isfinite(path.x[i]) || !std::isfinite(path.y[i]) ||
            !std::isfinite(path.yaw[i]) || !std::isfinite(path.kappa[i]) ||
            !std::isfinite(path.v[i]) || !std::isfinite(path.a[i])) {
            all_finite = false;
        }
        if (path.v[i] > 20.0) speed_ok = false;  // production target 9 m/s 대비 명백한 스파이크
        if (std::fabs(path.kappa[i]) > kMaxCurvature + 1e-3) curvature_ok = false;
        if (i > 0) {
            const double adjacent_dist =
                std::hypot(path.x[i] - path.x[i-1], path.y[i] - path.y[i-1]);
            // 물리적으로 기대되는 이동 거리: v * dt + 보간/이산화 여유.
            const double dt = 0.1;  // production sample_dt와 동일 크기
            const double expected = std::max(path.v[i-1], path.v[i]) * dt;
            if (adjacent_dist > expected + 1.5) jump_ok = false;
        }
    }
    Expect(all_finite, "18.10: all trajectory samples are finite");
    Expect(jump_ok,
           "18.10: no multi-meter adjacent Cartesian jump across the seam");
    Expect(speed_ok, "18.10: no artificial velocity spike across the seam");
    Expect(curvature_ok, "18.10: no artificial curvature spike across the seam");

    // KEEP 종단 d가 중앙 지향이어야 한다(Commit A의 KEEP centerline 계약이
    // seam에서도 유지된다) - 최종 pose를 다시 투영해 |d|가 크지 않은지 본다.
    CartesianState final_pose{};
    final_pose.x = path.x.back();
    final_pose.y = path.y.back();
    final_pose.yaw = path.yaw.back();
    final_pose.v = path.v.back();
    final_pose.a = path.a.back();
    double fs=0, fs_dot=0, fs_ddot=0, fd=0, fd_prime=0, fd_pprime=0;
    CartesianToFrenet(ref, final_pose, fs, fs_dot, fs_ddot, fd, fd_prime, fd_pprime);
    std::printf("  18.10: terminal d=%.4f\n", fd);
    Expect(std::fabs(fd) < 1.0,
           "18.10: KEEP selected terminal d remains centerline-oriented across the seam");
}

// Test 18.11 - CurveSpeed seam 회귀. closed_loop=true면 lookahead가 loop
// 시작부까지 wrap해서 본다. 열린 참조는 wrap하지 않는다.
void TestCurveSpeedSeamRegression() {
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    RefLine ref;
    LoadReferenceLine(waypoint_file, ref, kMaxCurvature);

    CurveSpeedConfig cfg{};
    cfg.target_vel = 10.0;
    cfg.curve_lookahead_m = 15.0;
    const double max_lateral_accel = 3.0;
    const double max_longitudinal_accel = 3.0;

    // 트랙 끝 바로 앞에서 조회 - production 트랙의 실제 곡률 분포로는 항상
    // 급커브가 있으리라는 보장이 없으므로, wrap 자체가 크래시 없이 동작하고
    // 물리적으로 타당한 범위(0 <= v <= target_vel) 안에 있는지만 검증한다.
    const double v = LookaheadTargetSpeed(ref, ref.length - 3.0, 9.0, cfg,
                                          max_longitudinal_accel,
                                          max_lateral_accel, kMaxCurvature);
    std::printf("  18.11: closed-loop seam lookahead speed = %.3f\n", v);
    Expect(std::isfinite(v) && v >= 0.0 && v <= cfg.target_vel + 1e-6,
           "18.11: closed-loop LookaheadTargetSpeed near the seam stays in a "
           "physically valid range");

    // 열린 참조: 끝에서 lookahead를 멈춘다(wrap하지 않는다) - straight
    // 합성 경로라 항상 target_vel이어야 한다.
    const std::vector<double> open_x = {0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0};
    const std::vector<double> open_y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const RefLine open_ref = BuildRefLine(open_x, open_y, kMaxCurvature);
    const double v_open = LookaheadTargetSpeed(open_ref, open_ref.length - 1.0, 9.0,
                                               cfg, max_longitudinal_accel,
                                               max_lateral_accel, kMaxCurvature);
    std::printf("  18.11: open reference near-end lookahead speed = %.3f\n", v_open);
    Expect(std::fabs(v_open - cfg.target_vel) < 1e-6,
           "18.11: open reference does not wrap past its end");
}

int main() {
    TestClosedLoopDetection();
    TestPhysicalClosureAfterPreprocessing();
    TestLoopLength();
    TestSeamSpatialContinuity();
    TestHeadingContinuity();
    TestCurvatureContinuity();
    TestDKappaContinuity();
    TestInterpolateWrapRegression();
    TestProjectionSeamRegression();
    TestProductionKeepTrajectoryCrossesSeam();
    TestCurveSpeedSeamRegression();

    if (g_failures == 0) {
        std::cout << "All closed-loop reference regression tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
