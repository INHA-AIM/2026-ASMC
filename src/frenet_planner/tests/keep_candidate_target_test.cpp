// Commit A regression: KEEP centerline candidate stabilization.
//
// 배경: 코너 구간에서 LANE_KEEPING이 전역 lateral_target_tolerance(0.5m)를
// 그대로 써서 d1={-0.5, 0, +0.5} 후보를 만들었고, cost 경쟁 결과로 cycle마다
// 선택된 terminal d가 바뀌는 조향 oscillation의 원인이 됐다. 이 테스트는
// LANE_KEEPING nominal 후보가 d=0 하나로 고정됐는지, 그리고 기존
// continuity fallback(GenerateLateralContinuityCandidates)과 다른
// maneuver(AVOID/LANE_CHANGE)의 target semantics는 그대로인지 확인한다.
//
// production 함수(ResolveManeuver, GenerateLateralContinuityCandidates)를
// 직접 호출한다 - 로직을 복사하지 않는다.

#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/PlannerCommand.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Generator/GeneratorConfig.hpp"
#include "frenet_planner/Generator/LateralGenerator.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"
#include "frenet_planner/Reference/Reference.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

constexpr double kEpsilon = 1e-9;
constexpr double kMaxCurvature = 0.1704;
constexpr double kLaneWidth = 3.5;

PathGeneratorConfig MakeConfig() {
    PathGeneratorConfig cfg;
    cfg.lateral_d1 = {-3.0, 3.0, 0.5};
    cfg.lateral_target_tolerance = 0.5;
    cfg.time_horizon = {1.5, 5.0, 0.5};
    cfg.delta_s = {-3.0, 3.0, 1.0};
    cfg.delta_s_dot = {-2.0, 2.0, 1.0};
    cfg.dt = 0.1;
    return cfg;
}

KinematicLimits MakeLimits() {
    return KinematicLimits{3.0, 3.0, kMaxCurvature, 6.0};
}

RefLine MakeStraightRef() {
    // 곡률 0인 직선 참조선. lateral 후보 자체(narrowed grid)를 확인하는
    // 목적이므로 곡률 필터의 영향을 배제한다.
    RefLine ref;
    for (int i = 0; i <= 200; ++i) {
        RefPoint rp{};
        rp.s = static_cast<double>(i) * 2.0;
        rp.x = rp.s;
        rp.y = 0.0;
        rp.theta = 0.0;
        rp.kappa = 0.0;
        rp.d_kappa = 0.0;
        ref.points.push_back(rp);
    }
    return ref;
}

}  // namespace

// Test A-1: LANE_KEEPING nominal candidate의 terminal d가 모두 0.0 근처.
void TestKeepNominalCandidatesTargetCenter() {
    const RefLine ref = MakeStraightRef();
    const PathGeneratorConfig cfg = MakeConfig();
    const KinematicLimits limits = MakeLimits();
    const VehicleShape vehicle_shape{2.0, 4.5, -1.0};
    const CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
    const std::vector<TrackedObject> no_obstacles;

    FrenetState start{};
    start.s = 50.0;
    start.d = 0.0;
    start.s_d = 5.0;
    start.s_dd = 0.0;

    PlannerCommand keep{};
    keep.mode = LANE_KEEPING;
    keep.has_target_speed = true;
    keep.target_speed = 5.0;

    PlannerDebugStats stats{};
    auto candidates = ResolveManeuver(start, keep, ref, cfg, limits, kLaneWidth,
                                      no_obstacles, vehicle_shape, collision_cfg, &stats);

    std::size_t valid_count = 0;
    bool all_centered = true;
    for (const auto& p : candidates) {
        if (!p.valid || p.d.empty()) continue;
        ++valid_count;
        if (std::fabs(p.d.back()) >= kEpsilon) all_centered = false;
    }
    std::printf("  KEEP nominal: valid candidates=%zu\n", valid_count);
    Expect(valid_count > 0, "KEEP nominal: at least one valid candidate exists");
    Expect(all_centered,
           "KEEP nominal: every valid candidate's terminal d is 0.0 "
           "(no more -0.5/0/+0.5 spread)");
}

// Test A-2: current d != 0일 때 continuity candidate(현재 d 유지)가 여전히
// fallback 대안으로 생성된다 - Commit A가 continuity 생성 자체를 건드리지
// 않았는지 확인한다.
void TestKeepContinuityFallbackStillExists() {
    const PathGeneratorConfig cfg = MakeConfig();
    const KinematicLimits limits = MakeLimits();

    FrenetState start{};
    start.s = 50.0;
    start.d = 1.2;  // 차선 중앙이 아닌 상태
    start.s_d = 5.0;
    start.s_dd = 0.0;

    auto continuity = GenerateLateralContinuityCandidates(start, cfg);
    FilterLateralByAcceleration(continuity, limits);

    bool found_current_d = false;
    for (const auto& p : continuity) {
        if (!p.valid || p.d.empty()) continue;
        if (std::fabs(p.d.back() - start.d) < kEpsilon) found_current_d = true;
    }
    std::printf("  KEEP continuity: candidates=%zu\n", continuity.size());
    Expect(!continuity.empty(),
           "KEEP continuity: GenerateLateralContinuityCandidates still "
           "produces candidates");
    Expect(found_current_d,
           "KEEP continuity: fallback candidate still preserves current d "
           "(1.2m), not forced back to 0");
}

// Test A-3: AVOID/LANE_CHANGE의 lateral target semantics는 그대로 - 전역
// lateral_target_tolerance가 적용되어 목표 d 주변 스프레드가 유지된다.
void TestOtherManeuversKeepTargetSpread() {
    const RefLine ref = MakeStraightRef();
    const PathGeneratorConfig cfg = MakeConfig();
    const KinematicLimits limits = MakeLimits();
    const VehicleShape vehicle_shape{2.0, 4.5, -1.0};
    const CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
    const std::vector<TrackedObject> no_obstacles;

    FrenetState start{};
    start.s = 50.0;
    start.d = 0.0;
    start.s_d = 5.0;
    start.s_dd = 0.0;

    {
        PlannerCommand avoid{};
        avoid.mode = AVOID;
        avoid.has_target_speed = true;
        avoid.target_speed = 5.0;
        avoid.avoidance_d_offset = 1.0;

        PlannerDebugStats stats{};
        auto candidates = ResolveManeuver(start, avoid, ref, cfg, limits, kLaneWidth,
                                          no_obstacles, vehicle_shape, collision_cfg,
                                          &stats);
        bool found_offset = false;
        bool found_spread = false;
        for (const auto& p : candidates) {
            if (!p.valid || p.d.empty()) continue;
            if (std::fabs(p.d.back() - 1.0) < kEpsilon) found_offset = true;
            if (std::fabs(p.d.back() - 1.0) >= kEpsilon &&
                std::fabs(p.d.back() - 1.0) <= 0.5 + kEpsilon) found_spread = true;
        }
        Expect(found_offset,
               "AVOID: candidate set still contains the exact target d "
               "(avoidance_d_offset=1.0)");
        Expect(found_spread,
               "AVOID: candidate set still spreads +-0.5m around the "
               "target (tolerance not narrowed for AVOID)");
    }

    {
        // D7: LANE_CHANGE_LEFT/RIGHT의 목표 중심은 더 이상 고정 ±lane_width가
        // 아니라 PlannerCommand.lane_change_d_offset(FSM target_pose_map을
        // Frenet으로 투영한 값)이다(LateralGenerator.cpp ResolveLateralOffset
        // 참고). 이 테스트는 그 목표 주변 스프레드 정책(전역
        // lateral_target_tolerance, 좁히지 않음)이 유지되는지만 확인한다 -
        // 목표를 lane_change_d_offset으로 명시해야 한다.
        PlannerCommand lane_change{};
        lane_change.mode = LANE_CHANGE_LEFT;
        lane_change.lane_change_d_offset = kLaneWidth;
        lane_change.has_target_speed = true;
        lane_change.target_speed = 5.0;

        PlannerDebugStats stats{};
        auto candidates = ResolveManeuver(start, lane_change, ref, cfg, limits, kLaneWidth,
                                          no_obstacles, vehicle_shape, collision_cfg,
                                          &stats);
        bool found_spread = false;
        for (const auto& p : candidates) {
            if (!p.valid || p.d.empty()) continue;
            if (std::fabs(p.d.back() - kLaneWidth) >= kEpsilon &&
                std::fabs(p.d.back() - kLaneWidth) <= 0.5 + kEpsilon) found_spread = true;
        }
        Expect(found_spread,
               "LANE_CHANGE_LEFT: candidate set still spreads +-0.5m around "
               "lane_change_d_offset target (tolerance not narrowed)");
    }
}

int main() {
    TestKeepNominalCandidatesTargetCenter();
    TestKeepContinuityFallbackStillExists();
    TestOtherManeuversKeepTargetSpread();

    if (g_failures == 0) {
        std::cout << "All KEEP candidate target regression tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
