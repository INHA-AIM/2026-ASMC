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
        std::vector<FrenetPath> continuity =
            GenerateLateralContinuityCandidates(direct_start, direct_cfg);
        FilterLateralByAcceleration(continuity, direct_limits);
        auto continuity_lon =
            GenerateVelocityKeepingCandidates(direct_start, snap.target_speed, direct_cfg);
        FilterLongitudinalByKinematicLimits(continuity_lon, direct_limits);
        auto continuity_combined = CombineLateralLongitudinal(continuity, continuity_lon);
        FilterByCurvature(continuity_combined, ref, direct_limits);
        std::size_t continuity_valid = 0;
        for (const auto& p : continuity_combined) if (p.valid) ++continuity_valid;

        std::printf("  %s: ResolveManeuver after_curvature=%zu, continuity candidates "
                    "valid after curvature=%zu\n",
                    snap.label, direct_stats.combined_valid_after_curvature, continuity_valid);
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

int main() {
    TestMoraiSnapshotsProduceValidCurvatureFeasiblePath();
    TestNormalKeepStillRecentersToCenter();
    TestContinuityNotGeneratedWhenNormalCandidatesValid();

    if (g_failures == 0) {
        std::cout << "All curvature-continuity regression tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
