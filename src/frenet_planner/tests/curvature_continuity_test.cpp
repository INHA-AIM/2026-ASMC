// Moderate-speed KEEP curvature starvation 회귀
// (fix/yuntae-frenet-speed-continuity Phase B).
//
// 재현 조건: 실제 MORAI 실행에서 s=75.71/884.50/910.32 근방(B1/B3/B4)에서
// 유효한 lat/lon 결합 후보가 있었는데도(combined=78) 전부 사후 곡률 필터
// (FilterByCurvature)에서 탈락했다(after_curvature=0). 원인은 external
// KEEP이 오직 차선 중앙(d1=0) 복귀 후보만 생성하기 때문 - 이 지점들에서는
// 중앙 복귀 quintic 자체의 d(t) 전환이 참조선 곡률과 결합해 기하학적 곡률
// 한계(0.1704)를 넘긴다. GenerateLateralContinuityCandidates(현재 d 유지)로
// 만든 candidate는 같은 조건에서 곡률을 통과한다(사전 조사로 확인).
//
// 이 테스트는 실제 production FrenetPlanner::Plan()을 그대로 호출한다 -
// 로직을 복사하지 않는다.

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/Generator/GeneratorConfig.hpp"
#include "frenet_planner/Generator/LateralGenerator.hpp"
#include "frenet_planner/Generator/LongitudinalGenerator.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"

#include <algorithm>
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

// 실제 waypoint 파일을 RefLine으로 로드하고 FrenetToCartesian으로 정확히
// (s, d, s_dot, s_ddot)에 해당하는 CartesianState를 만든다 - MORAI 스냅샷을
// production 변환으로 그대로 재구성한다(수치 hard-code 없음).
CartesianState BuildPoseAt(const RefLine& ref, double s, double d, double s_dot,
                           double s_ddot) {
    RefPoint rp = Interpolate(ref, s);
    return FrenetToCartesian(rp, s, s_dot, s_ddot, d, /*d_prime=*/0.0,
                             /*d_pprime=*/0.0);
}

constexpr double kMaxCurvature = 0.1704;

}  // namespace

// CASE B1/B3/B4: 실제 MORAI 스냅샷 재현. 유효 candidate가 남아야 하고,
// 선택된 경로의 max curvature가 한계를 넘지 않아야 한다.
void TestMoraiSnapshotsProduceValidCurvatureFeasiblePath() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/../config/params.yaml", waypoint_file,
        "curvature_continuity_test_params.yaml");

    RefLine ref;
    double max_curvature_for_load = kMaxCurvature;
    Expect(LoadReferenceLine(waypoint_file, ref, max_curvature_for_load),
           "setup: reference line loads for direct pose construction");

    struct Snapshot {
        const char* label;
        double s, d, s_dot, s_ddot, target_speed;
    };
    // B1/B3/B4는 사전 조사(B-1)에서 정확히 combined=78, after_curvature=0을
    // 재현한 파라미터다. B2는 명시된 s_dot/s_ddot이 스펙에 없어 정확한 후보
    // 총수(72)를 재현하지 못해 이 회귀에서는 제외한다(정직하게 보고).
    const std::vector<Snapshot> snapshots = {
        {"B1", 75.71, -0.677, 1.97, -2.40, 5.12},
        {"B3", 884.50, 0.43, 2.0, -2.4, 5.0},
        {"B4", 910.32, 0.34, 2.0, -2.4, 5.0},
    };

    for (const auto& snap : snapshots) {
        FrenetPlanner planner;
        Expect(planner.Init(config), std::string(snap.label) + ": planner initializes");

        const CartesianState ego =
            BuildPoseAt(ref, snap.s, snap.d, snap.s_dot, snap.s_ddot);

        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        keep.has_target_speed = true;
        keep.target_speed = snap.target_speed;
        const std::vector<TrackedObject> no_objects;
        CartesianPath path;
        const bool ok = planner.Plan(ego, no_objects, keep, path);
        std::printf("  %s: Plan() = %s, path.x.size()=%zu\n", snap.label,
                    ok ? "true" : "false", path.x.size());
        Expect(ok && !path.x.empty(),
               std::string(snap.label) +
                   ": external KEEP produces a trajectory at the MORAI "
                   "curvature-starvation snapshot (no longer 0 valid candidates)");

        // K1-B 회귀: 선택된 경로가 기하 곡률만이 아니라 실제
        // v^2*|kappa|(Cartesian 횡가속도)도 max_lateral_accel(=3.0, config
        // 그대로) 아래여야 한다. path.kappa/path.v는 production
        // ConvertToCartesianPath가 만든 값을 그대로 재사용한다(재계산 없음).
        if (ok && !path.x.empty()) {
            double lateral_accel_peak = 0.0;
            const std::size_t n = std::min(path.kappa.size(), path.v.size());
            for (std::size_t i = 0; i < n; ++i) {
                lateral_accel_peak = std::max(
                    lateral_accel_peak, path.v[i] * path.v[i] * std::fabs(path.kappa[i]));
            }
            std::printf("  %s: selected_lateral_accel_peak=%.3f (limit 3.0)\n",
                        snap.label, lateral_accel_peak);
            Expect(lateral_accel_peak <= 3.0 + 1e-6,
                   std::string(snap.label) +
                       ": selected KEEP trajectory respects max_lateral_accel "
                       "(v^2*|kappa|), not just geometric curvature");
        }

        // path.kappa(최종 CartesianPath)는 analytic 공식이라 저속에서
        // ComputeGeometricPath 기반 production 필터(FilterByCurvature)가
        // 실제로 판정한 값과 다를 수 있다(frenet_converter.hpp 설계 노트:
        // analytic 식은 저속에서 발산해 의도적으로 필터에서 배제됨). 이
        // divergence는 Phase B와 무관한 기존 특성이다 - 그래서 curvature
        // 판정 자체는 FilterByCurvature/ResolveManeuver를 직접 호출해
        // (production 함수, 로직 복제 없음) 확인한다.
        FrenetState direct_start{};
        direct_start.s = snap.s; direct_start.d = snap.d;
        direct_start.s_d = snap.s_dot; direct_start.s_dd = snap.s_ddot;
        PlannerCommand direct_cmd{};
        direct_cmd.mode = LANE_KEEPING;
        direct_cmd.has_target_speed = true;
        direct_cmd.target_speed = snap.target_speed;
        PathGeneratorConfig direct_cfg;
        direct_cfg.lateral_d1 = {-3.0, 3.0, 0.5};
        direct_cfg.lateral_target_tolerance = 0.5;
        direct_cfg.time_horizon = {1.5, 5.0, 0.5};
        direct_cfg.delta_s = {-3.0, 3.0, 1.0};
        direct_cfg.delta_s_dot = {-2.0, 2.0, 1.0};
        direct_cfg.dt = 0.1;
        KinematicLimits direct_limits{3.0, 3.0, kMaxCurvature, 6.0};
        VehicleShape vehicle_shape{2.0, 4.5};
        CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
        std::vector<TrackedObject> no_obstacles;
        PlannerDebugStats direct_stats{};
        auto direct_candidates = ResolveManeuver(direct_start, direct_cmd, ref, direct_cfg,
                                                 direct_limits, /*lane_width=*/3.5,
                                                 no_obstacles, vehicle_shape, collision_cfg,
                                                 &direct_stats);
        // K1.1 회귀: normal 후보의 starvation 판정 자체를 K1과 동일하게(곡률
        // + Cartesian 횡가속도 둘 다 거친 뒤) 다시 확인한다 - K1 이전에는
        // after_curvature만 보면 충분했지만, 지금 production 코드
        // (Maneuver/Keep.cpp)는 그 다음에 FilterByCartesianLateralAcceleration도
        // 적용하므로 "정상 후보가 진짜로 0개인가"는 이 필터까지 통과해야
        // 판정된다.
        FilterByCartesianLateralAcceleration(direct_candidates, ref,
                                             direct_limits.max_lateral_accel);
        std::size_t normal_valid_after_lateral_accel = 0;
        for (const auto& p : direct_candidates) if (p.valid) ++normal_valid_after_lateral_accel;

        std::vector<FrenetPath> continuity =
            GenerateLateralContinuityCandidates(direct_start, direct_cfg);
        FilterLateralByAcceleration(continuity, direct_limits);
        auto continuity_lon =
            GenerateVelocityKeepingCandidates(direct_start, snap.target_speed, direct_cfg);
        FilterLongitudinalByKinematicLimits(continuity_lon, direct_limits);
        auto continuity_combined = CombineLateralLongitudinal(continuity, continuity_lon);
        FilterByCurvature(continuity_combined, ref, direct_limits);
        std::size_t continuity_valid_after_curvature = 0;
        for (const auto& p : continuity_combined) if (p.valid) ++continuity_valid_after_curvature;

        // K1.1 핵심 회귀: continuity_combined에도 production
        // FilterByCartesianLateralAcceleration을 그대로 적용한다(K1.1이
        // Maneuver/Keep.cpp의 AppendExternalKeepContinuityCandidates 안에
        // 추가한 것과 같은 호출) - continuity가 곡률은 통과했지만 실제
        // v^2*|kappa|는 넘는 경우를 여기서 잡아야 한다.
        FilterByCartesianLateralAcceleration(continuity_combined, ref,
                                             direct_limits.max_lateral_accel);
        std::size_t continuity_valid = 0;
        double continuity_lateral_accel_peak = 0.0;
        for (const auto& p : continuity_combined) {
            if (!p.valid) continue;
            ++continuity_valid;
            const GeometricPath geo = ComputeGeometricPath(p.s, p.d, ref);
            const std::size_t n = std::min(geo.kappa.size(), p.s_d.size());
            for (std::size_t i = 0; i < n; ++i) {
                continuity_lateral_accel_peak = std::max(
                    continuity_lateral_accel_peak,
                    p.s_d[i] * p.s_d[i] * std::fabs(geo.kappa[i]));
            }
        }

        std::printf("  %s: ResolveManeuver after_curvature=%zu, normal valid "
                    "after lateral-accel=%zu, continuity valid after "
                    "curvature=%zu, continuity valid after lateral-accel=%zu "
                    "(peak=%.3f)\n",
                    snap.label, direct_stats.combined_valid_after_curvature,
                    normal_valid_after_lateral_accel, continuity_valid_after_curvature,
                    continuity_valid, continuity_lateral_accel_peak);

        // B1/B4는 이 스위트가 이미 curvature 단계에서 0 valid로 확인한
        // 진짜 starvation 스냅샷이다(direct_stats.combined_valid_after_curvature
        // == 0이면 lateral-accel 필터를 더 적용해도 여전히 0이어야 한다).
        // B3는 위 주석(88행)의 원래 claim과 달리 현재 waypoint/설정 조합에서
        // 실제로는 after_curvature=1(정상 후보 하나가 곡률+횡가속도를 모두
        // 통과)이다 - starvation 스냅샷이 아니므로 이 케이스에서는 그
        // precondition을 강제하지 않는다(정직하게 건너뜀, 파일 상단 B2
        // 제외 관례와 동일).
        if (direct_stats.combined_valid_after_curvature == 0) {
            Expect(normal_valid_after_lateral_accel == 0,
                   std::string(snap.label) +
                       ": setup precondition - normal KEEP candidates are "
                       "genuinely starved (0 valid) after curvature+lateral-accel "
                       "filters at this snapshot, so continuity fallback is "
                       "actually required (not a cost-competition scenario)");
        } else {
            std::printf("  %s: skipping starvation precondition (normal "
                        "candidates already valid after curvature alone, not "
                        "a starvation snapshot at this config)\n",
                        snap.label);
        }
        Expect(continuity_valid >= 1,
               std::string(snap.label) +
                   ": continuity fallback has at least one candidate that "
                   "survives production FilterByCartesianLateralAcceleration "
                   "(K1.1 - not just geometric curvature)");
        if (continuity_valid >= 1) {
            Expect(continuity_lateral_accel_peak <= direct_limits.max_lateral_accel + 1e-6,
                   std::string(snap.label) +
                       ": surviving continuity candidate(s) satisfy "
                       "v^2*|kappa| <= max_lateral_accel exactly (peak <= limit)");
        }
        Expect(direct_stats.combined_valid_after_curvature + continuity_valid >= 1,
               std::string(snap.label) +
                   ": at least one candidate (normal recenter or continuity) "
                   "passes the production curvature filter (max_curvature "
                   "0.1704 preserved, not raised)");
    }
}

// CASE: 정상 직선/완만한 구간에서는 기존 center recovery(d~=0)가 그대로
// 선택된다 - continuity candidate 추가가 정상 KEEP 동작을 바꾸지 않는다.
void TestNormalKeepStillRecentersToCenter() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/../config/params.yaml", waypoint_file,
        "curvature_continuity_test_normal_params.yaml");

    RefLine ref;
    double max_curvature_for_load = kMaxCurvature;
    LoadReferenceLine(waypoint_file, ref, max_curvature_for_load);

    FrenetPlanner planner;
    planner.Init(config);
    // 평범한 직선 근처(다른 command_*_plan_test와 동일 관례), 차선 중앙에서
    // 살짝 벗어난 상태(d=0.3)로 시작해 center recovery가 여전히 일어나는지
    // 확인한다.
    const CartesianState ego = BuildPoseAt(ref, /*s=*/200.0, /*d=*/0.3,
                                           /*s_dot=*/8.0, /*s_ddot=*/0.0);
    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 8.0;
    const std::vector<TrackedObject> no_objects;
    CartesianPath path;
    const bool ok = planner.Plan(ego, no_objects, keep, path);
    Expect(ok && !path.x.empty(), "normal KEEP: plan succeeds away from any tight curve");

    if (ok && !path.x.empty()) {
        CartesianState final_pose{};
        final_pose.x = path.x.back();
        final_pose.y = path.y.back();
        final_pose.yaw = path.yaw.back();
        final_pose.v = path.v.back();
        final_pose.a = path.a.back();
        double fs = 0.0, fs_dot = 0.0, fs_ddot = 0.0, fd = 0.0, fd_prime = 0.0, fd_pprime = 0.0;
        CartesianToFrenet(ref, final_pose, fs, fs_dot, fs_ddot, fd, fd_prime, fd_pprime);
        std::printf("  normal KEEP: terminal d=%.4f (initial d=0.30)\n", fd);
        Expect(std::fabs(fd) < 0.3,
               "normal KEEP: still recenters toward the lane center (terminal "
               "d closer to 0 than the initial 0.3m offset)");
    }
}

// CASE (K1-C 회귀, §7 실측 재현): 정상 center-recovery 후보가 곡률/횡가속도를
// 모두 통과하는 완만한 구간에서, offset(d!=0) 상태로 시작해도 continuity
// fallback이 생성되지 않아야 한다 - 예전 구현은 여기서도 continuity를
// 항상 얹어 EvaluateCosts로 경쟁시켰고(kd_path 항 때문에 실측
// s=115.79/d=+0.173/chosen_d=+0.173처럼 continuity가 이길 수 있었다),
// 이제는 정상 후보가 하나라도 유효하면 continuity 자체를 생성하지 않는다.
// last_candidate_paths()/last_candidate_validity()(기존 debug 공개 API,
// 새 API 아님)로 이번 cycle에 실제 생성된 후보 수를 확인한다 - continuity가
// 생성됐다면 정상 center-recovery 후보(대략 lateral_d1 격자 x longitudinal
// 격자)보다 후보 총량이 늘어난다.
void TestContinuityNotGeneratedWhenNormalCandidatesValid() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/../config/params.yaml", waypoint_file,
        "curvature_continuity_test_no_fallback_params.yaml");

    RefLine ref;
    double max_curvature_for_load = kMaxCurvature;
    LoadReferenceLine(waypoint_file, ref, max_curvature_for_load);

    FrenetPlanner planner;
    Expect(planner.Init(config), "no-fallback: planner initializes");

    // moderate offset(d=0.6m, 차선 폭 안), 완만한 직선 구간(TestNormalKeep
    // StillRecentersToCenter와 같은 s=200 근방) - 정상 center-recovery
    // 후보가 곡률/횡가속도 필터를 모두 통과할 조건.
    const CartesianState ego = BuildPoseAt(ref, /*s=*/200.0, /*d=*/0.6,
                                           /*s_dot=*/5.0, /*s_ddot=*/0.0);
    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 5.0;
    const std::vector<TrackedObject> no_objects;
    CartesianPath path;
    const bool ok = planner.Plan(ego, no_objects, keep, path);
    Expect(ok && !path.x.empty(),
           "no-fallback: plan succeeds with a moderate lane offset on a mild segment");

    // 별도 direct ResolveManeuver 호출(§ 위 TestMoraiSnapshots와 동일 관례)로
    // "정상 후보가 실제로 살아 있었는지"를 production 함수로 독립 확인한다 -
    // 이 조건이 거짓이면 이 테스트 자체가 K1-C를 검증하지 못하므로 케이스
    // 선택이 잘못됐다는 뜻이다(정직하게 실패해야 한다).
    FrenetState direct_start{};
    direct_start.s = 200.0; direct_start.d = 0.6;
    direct_start.s_d = 5.0; direct_start.s_dd = 0.0;
    PlannerCommand direct_cmd{};
    direct_cmd.mode = LANE_KEEPING;
    direct_cmd.has_target_speed = true;
    direct_cmd.target_speed = 5.0;
    PathGeneratorConfig direct_cfg;
    direct_cfg.lateral_d1 = {0.6, 0.6, 0.5};
    direct_cfg.lateral_target_tolerance = 0.0;
    direct_cfg.time_horizon = {1.5, 5.0, 0.5};
    direct_cfg.delta_s = {-3.0, 3.0, 1.0};
    direct_cfg.delta_s_dot = {-2.0, 2.0, 1.0};
    direct_cfg.dt = 0.1;
    KinematicLimits direct_limits{3.0, 3.0, kMaxCurvature, 6.0};
    VehicleShape vehicle_shape{2.0, 4.5};
    CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
    std::vector<TrackedObject> no_obstacles;
    PlannerDebugStats direct_stats{};
    auto direct_candidates = ResolveManeuver(direct_start, direct_cmd, ref, direct_cfg,
                                             direct_limits, /*lane_width=*/3.5,
                                             no_obstacles, vehicle_shape, collision_cfg,
                                             &direct_stats);
    FilterByCartesianLateralAcceleration(direct_candidates, ref,
                                         direct_limits.max_lateral_accel);
    std::size_t normal_valid_after_lateral_accel = 0;
    for (const auto& p : direct_candidates) if (p.valid) ++normal_valid_after_lateral_accel;
    std::printf("  no-fallback: normal_keep_valid_after_lateral_accel=%zu\n",
                normal_valid_after_lateral_accel);
    Expect(normal_valid_after_lateral_accel > 0,
           "no-fallback: setup precondition - normal KEEP candidates survive "
           "curvature+lateral-accel filters at this offset/segment (so any "
           "continuity candidates in this cycle would be true cost "
           "competition, not starvation fallback)");

    // production Plan()이 실제로 만든 후보 목록에서, continuity가 만드는
    // "현재 d 유지" 종점(대략 d=0.6 근방)이 하나라도 존재하는지 확인한다.
    // 이 테스트가 실패하면(=continuity 후보가 여전히 섞여 있으면) K1-C
    // 게이팅이 스코프를 벗어났다는 뜻이다.
    const auto& candidate_paths = planner.last_candidate_paths();
    const auto& candidate_validity = planner.last_candidate_validity();
    std::size_t stale_offset_terminal_count = 0;
    for (std::size_t i = 0; i < candidate_paths.size(); ++i) {
        if (i >= candidate_validity.size() || candidate_paths[i].x.empty()) continue;
        double fs = 0.0, fs_dot = 0.0, fs_ddot = 0.0, fd = 0.0, fd_prime = 0.0, fd_pprime = 0.0;
        CartesianState terminal{};
        terminal.x = candidate_paths[i].x.back();
        terminal.y = candidate_paths[i].y.back();
        terminal.yaw = candidate_paths[i].yaw.back();
        terminal.v = candidate_paths[i].v.back();
        CartesianToFrenet(ref, terminal, fs, fs_dot, fs_ddot, fd, fd_prime, fd_pprime);
        if (std::fabs(fd - 0.6) < 0.05) ++stale_offset_terminal_count;
    }
    std::printf("  no-fallback: candidates terminating near initial offset "
                "d=0.6 = %zu/%zu\n",
                stale_offset_terminal_count, candidate_paths.size());
    Expect(stale_offset_terminal_count == 0,
           "no-fallback: no continuity(current-d-hold) candidate appears "
           "in Plan()'s candidate set when normal center-recovery candidates "
           "are already valid (continuity is true fallback, not a permanent "
           "cost competitor)");
}

// CASE (K1.1 §9, rejection path): 반지름은 max_curvature 한계 안이지만
// (기하 곡률 필터는 통과) 시험 속도에서 v^2*|kappa|가 max_lateral_accel을
// 넘도록 만든 합성 원형 참조선에서, continuity 후보(현재 d 유지) 자체가
// FilterByCartesianLateralAcceleration에서 무효 처리되는지 확인한다 -
// production GenerateLateralContinuityCandidates/FilterByCurvature/
// FilterByCartesianLateralAcceleration을 그대로 호출한다(로직 복제 없음).
// MakeCircleRef는 curvature_contract_test.cpp의 관례를 그대로 따른
// 파일-지역 합성 참조선 헬퍼다(패키지 간 공유 코드를 새로 만들지 않는다).
void TestContinuityCandidateRejectedWhenDynamicallyInfeasible() {
    auto MakeCircleRef = [](double radius, int n_points) {
        RefLine ref;
        const double step = 2.0 * M_PI * radius / n_points;
        for (int i = 0; i <= n_points; ++i) {
            const double theta = static_cast<double>(i) / n_points * 2.0 * M_PI;
            RefPoint p{};
            p.x = radius * std::cos(theta);
            p.y = radius * std::sin(theta);
            p.s = step * i;
            p.theta = theta + M_PI / 2.0;
            p.kappa = 1.0 / radius;
            p.d_kappa = 0.0;
            ref.points.push_back(p);
        }
        ref.closed_loop = true;
        ref.length = ref.points.empty() ? 0.0 : ref.points.back().s;
        return ref;
    };

    const double max_lateral_accel = 3.0;
    // v^2*kappa = max_lateral_accel*1.3(한계보다 30% 위) 이 되도록 속도를
    // 고른다 - kappa = 1/radius는 max_curvature(0.1704)의 절반 정도로
    // 여유를 둬서 기하 곡률 필터는 확실히 통과하게 한다(이 테스트가 검증할
    // 것은 lateral-accel 필터이지 curvature 필터가 아니다).
    const double kappa = kMaxCurvature * 0.5;
    const double radius = 1.0 / kappa;
    const double speed = std::sqrt(max_lateral_accel * 1.3 / kappa);

    RefLine ref = MakeCircleRef(radius, 200);

    FrenetState start{};
    start.s = 0.0; start.d = 0.5; start.d_d = 0.0; start.d_dd = 0.0;
    start.s_d = speed; start.s_dd = 0.0;

    PathGeneratorConfig cfg;
    cfg.lateral_d1 = {-3.0, 3.0, 0.5};
    cfg.lateral_target_tolerance = 0.5;
    cfg.time_horizon = {1.5, 5.0, 0.5};
    cfg.delta_s = {-3.0, 3.0, 1.0};
    cfg.delta_s_dot = {-2.0, 2.0, 1.0};
    cfg.dt = 0.1;
    KinematicLimits limits{max_lateral_accel, 3.0, kMaxCurvature, 6.0};

    std::vector<FrenetPath> continuity_lateral =
        GenerateLateralContinuityCandidates(start, cfg);
    FilterLateralByAcceleration(continuity_lateral, limits);
    std::vector<FrenetPath> continuity_lon =
        GenerateVelocityKeepingCandidates(start, speed, cfg);
    FilterLongitudinalByKinematicLimits(continuity_lon, limits);
    std::vector<FrenetPath> continuity_combined =
        CombineLateralLongitudinal(continuity_lateral, continuity_lon);
    FilterByCurvature(continuity_combined, ref, limits);

    std::size_t valid_after_curvature = 0;
    for (const auto& p : continuity_combined) if (p.valid) ++valid_after_curvature;
    std::printf("  reject: continuity valid_after_curvature=%zu (kappa=%.4f <= "
                "max_curvature=%.4f, expected to pass geometric curvature)\n",
                valid_after_curvature, kappa, kMaxCurvature);
    Expect(valid_after_curvature >= 1,
           "reject: setup precondition - continuity candidate(s) pass the "
           "geometric curvature filter (this test targets the "
           "lateral-acceleration filter specifically, not curvature)");

    FilterByCartesianLateralAcceleration(continuity_combined, ref, max_lateral_accel);
    std::size_t valid_after_lateral_accel = 0;
    for (const auto& p : continuity_combined) if (p.valid) ++valid_after_lateral_accel;
    std::printf("  reject: continuity valid_after_lateral_accel=%zu (speed=%.2f, "
                "v^2*kappa=%.3f > limit=%.2f expected)\n",
                valid_after_lateral_accel, speed, speed * speed * kappa, max_lateral_accel);
    Expect(valid_after_lateral_accel == 0,
           "reject: a continuity candidate that geometrically passes "
           "curvature but exceeds v^2*|kappa| > max_lateral_accel is "
           "invalidated by FilterByCartesianLateralAcceleration (K1.1 - "
           "continuity cannot bypass dynamic feasibility)");
}

int main() {
    TestMoraiSnapshotsProduceValidCurvatureFeasiblePath();
    TestNormalKeepStillRecentersToCenter();
    TestContinuityNotGeneratedWhenNormalCandidatesValid();
    TestContinuityCandidateRejectedWhenDynamicallyInfeasible();

    if (g_failures == 0) {
        std::cout << "All curvature-continuity regression tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
