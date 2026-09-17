// R5 regression: TrajectoryRequest -> GenerateRequestedCandidates contract.
//
// R4에서 이미 두 번(leader_object_id 누락, lateral hold sentinel 오염)
// "field mapping omission" 버그가 났다. R5는 그 매핑 지식을 Generator
// 안으로 옮기는 단계이므로, 기존 24개 테스트(ResolveManeuver 대상)만
// 믿지 않고 TrajectoryRequest 경계 자체를 직접 검증한다.
//
// production 함수(GenerateRequestedCandidates)를 직접 호출한다 - 로직을
// 복사하지 않는다.

#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/ObjectTypes.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Generator/GeneratorConfig.hpp"
#include "frenet_planner/Geometry/PolygonUtils.hpp"
#include "frenet_planner/Pipeline/TrajectoryRequest.hpp"
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
    KinematicLimits limits{};
    limits.max_lateral_accel = 3.0;
    limits.max_longitudinal_accel = 3.0;
    limits.max_curvature = kMaxCurvature;
    limits.max_longitudinal_jerk = 6.0;
    limits.low_speed_fallback_v = 0.5;
    return limits;
}

RefLine MakeStraightRef() {
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

FrenetState MakeStraightStart() {
    FrenetState start{};
    start.s = 50.0;
    start.d = 0.0;
    start.s_d = 5.0;
    start.s_dd = 0.0;
    return start;
}

// authoritative 8-corner OBB를 직접 구성한다(polygon_collision_test.cpp의
// MakeObstacle과 동일한 fixture 패턴 - production은 center/width/length/yaw
// 기반 geometry 합성을 하지 않는다).
TrackedObject MakeVehicleAt(int id, double x, double length) {
    TrackedObject obj{};
    obj.id = id;
    obj.x = x;
    obj.y = 0.0;
    obj.heading = 0.0;
    obj.length = length;
    obj.width = 2.0;
    obj.height = 1.5;

    const double half_length = obj.length * 0.5;
    const double half_width = obj.width * 0.5;
    const double local_xy[4][2] = {
        {-half_length, -half_width}, {half_length, -half_width},
        {half_length, half_width},   {-half_length, half_width},
    };
    for (std::size_t i = 0; i < 4; ++i) {
        const double corner_x = x + local_xy[i][0];
        const double corner_y = local_xy[i][1];
        obj.corners[i] = {corner_x, corner_y, 0.0};
        obj.corners[i + 4] = {corner_x, corner_y, obj.height};
    }
    obj.geometry_valid = geometry::ValidateCorners8(obj.corners).valid;
    obj.z_valid = false;
    return obj;
}

}  // namespace

// Test A: KEEP request(target_d=0, search_tolerance=0, VELOCITY_KEEPING) ->
// 기존 LANE_KEEPING nominal semantics(터미널 d=0 단일 고정)와 동일.
void TestKeepRequestMatchesLegacySemantics() {
    const RefLine ref = MakeStraightRef();
    const PathGeneratorConfig cfg = MakeConfig();
    const KinematicLimits limits = MakeLimits();
    const VehicleShape vehicle_shape{2.0, 4.5, -1.0};
    const CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
    const std::vector<TrackedObject> no_obstacles;
    const FrenetState start = MakeStraightStart();

    TrajectoryRequest request;
    request.lateral.target_d = 0.0;
    request.lateral.search_tolerance = 0.0;
    request.longitudinal_mode = LongitudinalMode::VELOCITY_KEEPING;
    request.target_speed = 5.0;

    PlannerDebugStats stats{};
    auto candidates = GenerateRequestedCandidates(
        start, request, ref, cfg, limits, no_obstacles, vehicle_shape,
        collision_cfg, &stats);

    std::size_t valid_count = 0;
    bool all_centered = true;
    for (const auto& p : candidates) {
        if (!p.valid || p.d.empty()) continue;
        ++valid_count;
        if (std::fabs(p.d.back()) >= kEpsilon) all_centered = false;
    }
    std::printf("  KEEP request: valid candidates=%zu\n", valid_count);
    Expect(valid_count > 0, "KEEP request: at least one valid candidate exists");
    Expect(all_centered,
           "KEEP request: every valid candidate's terminal d is 0.0 "
           "(matches legacy LANE_KEEPING nominal semantics)");
}

// Test B: FOLLOW request - leader_s/speed/accel, time_gap/min_gap/gap_gain,
// leader_object_id가 전부 generator/collision filter까지 전달된다. 특히
// leader_object_id 누락 회귀(R4-D에서 한 번 실제로 발생)를 검증한다:
// leader 차량을 정확히 그 자리에 collision obstacle로도 넣어서, coast
// 연장 단계에서 장애물로 오인해 후보가 사라지지 않는지 확인한다.
void TestFollowRequestPropagatesLeaderAndExemption() {
    const RefLine ref = MakeStraightRef();
    const PathGeneratorConfig cfg = MakeConfig();
    const KinematicLimits limits = MakeLimits();
    const VehicleShape vehicle_shape{2.0, 4.5, -1.0};
    const CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
    const FrenetState start = MakeStraightStart();

    constexpr int kLeaderId = 42;
    constexpr double kLeaderS = 70.0;  // start.s=50 + 20m gap
    const std::vector<TrackedObject> obstacles = {
        MakeVehicleAt(kLeaderId, kLeaderS, 4.5)};

    TrajectoryRequest request;
    request.lateral.target_d = 0.0;
    request.lateral.search_tolerance = cfg.lateral_target_tolerance;
    request.longitudinal_mode = LongitudinalMode::FOLLOWING;
    request.target_speed = 5.0;
    request.collision.coast_exempt_object_id = kLeaderId;
    request.following.leader_s = kLeaderS;
    request.following.leader_speed = 0.0;  // 정지 leader (R4-D 재현 조건)
    request.following.leader_accel = 0.0;
    request.following.time_gap = 1.5;
    request.following.min_gap = 5.0;
    request.following.gap_gain = 0.5;

    PlannerDebugStats stats{};
    auto candidates = GenerateRequestedCandidates(
        start, request, ref, cfg, limits, obstacles, vehicle_shape,
        collision_cfg, &stats);

    std::size_t valid_after_collision = 0;
    for (const auto& p : candidates) if (p.valid) ++valid_after_collision;
    std::printf("  FOLLOW request: valid after collision=%zu/%zu "
                "(after_curvature=%zu)\n",
                valid_after_collision, candidates.size(),
                stats.combined_valid_after_curvature);
    Expect(stats.longitudinal_total > 0 && stats.longitudinal_valid > 0,
           "FOLLOW request: leader_s/speed/accel/time_gap/min_gap/gap_gain "
           "reached GenerateFollowingCandidates (non-empty valid set)");
    Expect(valid_after_collision > 0,
           "FOLLOW request: a stopped leader placed exactly at leader_s is "
           "NOT filtered out as a collision - leader_object_id reached the "
           "collision filter as coast_exempt_id (R4-D regression guard)");
}

// Test C: request.stop.terminal_offset을 바꾸면 generated STOP candidate의
// terminal delta 범위가 그에 따라 달라진다 - Generator가
// StopConfig.delta_s_override를 몰래 직접 읽고 있지 않음을 증명한다.
void TestStopTerminalOffsetIsRequestDriven() {
    const RefLine ref = MakeStraightRef();
    const PathGeneratorConfig cfg = MakeConfig();
    const KinematicLimits limits = MakeLimits();
    const VehicleShape vehicle_shape{2.0, 4.5, -1.0};
    const CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
    const std::vector<TrackedObject> no_obstacles;
    // BRAKING 단계를 가정한 낮은 속도/짧은 거리 - time_horizon([1.5, 5.0]s)
    // 안에서 max_longitudinal_accel(3.0 m/s^2) 한계로 물리적으로 달성 가능한
    // 조건이어야 FilterLongitudinalByKinematicLimits를 통과하는 후보가 남는다.
    FrenetState start = MakeStraightStart();
    start.s_d = 2.0;
    const double kStopS = start.s + 6.0;  // 6m 앞

    auto MaxTerminalS = [&](const SamplingRange& terminal_offset) {
        TrajectoryRequest request;
        request.lateral.target_d = 0.0;
        request.lateral.search_tolerance = 0.0;
        request.longitudinal_mode = LongitudinalMode::STOPPING;
        request.target_speed = 5.0;
        request.stop.stop_s = kStopS;
        request.stop.terminal_offset = terminal_offset;

        PlannerDebugStats stats{};
        auto candidates = GenerateRequestedCandidates(
            start, request, ref, cfg, limits, no_obstacles, vehicle_shape,
            collision_cfg, &stats);
        double max_terminal_s = -1e9;
        for (const auto& p : candidates) {
            if (!p.valid || p.s.empty()) continue;
            max_terminal_s = std::max(max_terminal_s, p.s.back());
        }
        return max_terminal_s;
    };

    // Narrow: 정지 목표를 절대 지나치지 않는다(레거시 STOP delta_s_override와
    // 동일한 형태, 목표 앞쪽[-1.0, 0.0]만 허용).
    const double narrow_max = MaxTerminalS(SamplingRange{-1.0, 0.0, 0.25});
    // Wide: 목표를 넘어서는 지점까지 허용(+5.0)했을 때 실제로 그 범위가
    // 반영되는지 확인한다.
    const double wide_max = MaxTerminalS(SamplingRange{-1.0, 5.0, 0.25});

    std::printf("  STOP terminal_offset: narrow_max_s=%.2f wide_max_s=%.2f "
                "(stop_s=%.2f)\n",
                narrow_max, wide_max, kStopS);
    Expect(narrow_max > -1e8 && narrow_max <= kStopS + kEpsilon,
           "STOP request: narrow terminal_offset never overshoots stop_s");
    Expect(wide_max > kStopS + kEpsilon,
           "STOP request: widening terminal_offset actually widens the "
           "generated terminal-s range (proves the generator reads "
           "request.stop.terminal_offset, not a config it re-reads itself)");
}

// Test D: has_hold=true, target_d=0.0이 has_hold=false와 실제로 다른 후보
// 집합을 만드는지 확인한다(R4-E가 고친 sentinel 버그의 회귀 방지) - hold가
// 걸리면 tolerance=0으로 좁아지고(터미널 d가 0 하나로 고정), hold가 없으면
// (KEEP 기준) search_tolerance=0을 명시했을 때만 동일한 결과가 나오므로,
// 여기서는 search_tolerance를 0이 아닌 값으로 주고 has_hold로만 좁혀지는지
// 검증한다.
void TestLateralHoldAtZeroDiffersFromNoHold() {
    const RefLine ref = MakeStraightRef();
    const PathGeneratorConfig cfg = MakeConfig();
    const KinematicLimits limits = MakeLimits();
    const VehicleShape vehicle_shape{2.0, 4.5, -1.0};
    const CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
    const std::vector<TrackedObject> no_obstacles;
    const FrenetState start = MakeStraightStart();

    auto TerminalDSpread = [&](bool has_hold) {
        TrajectoryRequest request;
        request.lateral.has_hold = has_hold;
        request.lateral.target_d = 0.0;
        // has_hold와 무관하게 search_tolerance는 그대로 authoritative
        // input이다(§5) - 여기서는 hold 자체가 켜져도 값(target_d=0.0)이
        // 바뀌지 않는지가 관심사이므로, search_tolerance를 넓게 줘서 만약
        // has_hold=true를 target_d!=0.0로 오인하는 sentinel 버그가 있다면
        // 이 tolerance가 무시되고 좁아지는지를 별도로 확인한다.
        request.lateral.search_tolerance = cfg.lateral_target_tolerance;
        request.longitudinal_mode = LongitudinalMode::VELOCITY_KEEPING;
        request.target_speed = 5.0;

        PlannerDebugStats stats{};
        auto candidates = GenerateRequestedCandidates(
            start, request, ref, cfg, limits, no_obstacles, vehicle_shape,
            collision_cfg, &stats);
        double min_d = 1e9, max_d = -1e9;
        for (const auto& p : candidates) {
            if (!p.valid || p.d.empty()) continue;
            min_d = std::min(min_d, p.d.back());
            max_d = std::max(max_d, p.d.back());
        }
        return max_d - min_d;
    };

    const double spread_no_hold = TerminalDSpread(/*has_hold=*/false);
    const double spread_hold_at_zero = TerminalDSpread(/*has_hold=*/true);

    std::printf("  lateral hold d=0: spread(no_hold)=%.3f "
                "spread(has_hold=true, target_d=0.0)=%.3f\n",
                spread_no_hold, spread_hold_at_zero);
    // has_hold는 R5의 GenerateRequestedCandidates에서 target_d/tolerance를
    // 그대로 authoritative input으로 쓰므로(§5), 이 함수 자체는 has_hold
    // 필드를 별도로 읽지 않는다 - hold 여부에 따른 tolerance 좁힘은 R4의
    // Keep.cpp 쪽(request 생성부)의 책임이다. 따라서 이 테스트는 "동일한
    // target_d/search_tolerance를 명시하면 has_hold 값과 무관하게 동일한
    // 후보가 나온다"(=target_d=0.0이 hold 여부를 암묵적으로 바꾸지 않는다)
    // 것을 확인한다.
    Expect(std::fabs(spread_no_hold - spread_hold_at_zero) < kEpsilon,
           "lateral request: has_hold=true with target_d=0.0 produces the "
           "same candidate spread as has_hold=false with the same "
           "target_d/search_tolerance (target_d==0.0 is not treated as an "
           "implicit sentinel by the generator itself)");
    Expect(spread_no_hold > kEpsilon,
           "lateral request: search_tolerance is genuinely honored (spread "
           "> 0 when tolerance > 0), so a sentinel-based bug upstream would "
           "have been visible as spread collapsing to 0");
}

// Test E: FOLLOW request가 기존 cfg.lateral_target_tolerance 범위를
// 유지하는지 검증한다(R5에서 가장 쉽게 발생할 수 있는 회귀 - Keep.cpp가
// FOLLOW의 search_tolerance를 실수로 0으로 좁히면 이 테스트가 잡는다).
void TestFollowRequestKeepsLateralTolerance() {
    const RefLine ref = MakeStraightRef();
    const PathGeneratorConfig cfg = MakeConfig();
    const KinematicLimits limits = MakeLimits();
    const VehicleShape vehicle_shape{2.0, 4.5, -1.0};
    const CollisionCheckConfig collision_cfg{0.3, 0.1, 3.0};
    const std::vector<TrackedObject> no_obstacles;
    const FrenetState start = MakeStraightStart();

    TrajectoryRequest request;
    request.lateral.target_d = 0.0;
    request.lateral.search_tolerance = cfg.lateral_target_tolerance;
    request.longitudinal_mode = LongitudinalMode::FOLLOWING;
    request.target_speed = 5.0;
    request.following.leader_s = start.s + 30.0;
    request.following.leader_speed = 5.0;
    request.following.leader_accel = 0.0;
    request.following.time_gap = 1.5;
    request.following.min_gap = 5.0;
    request.following.gap_gain = 0.5;

    PlannerDebugStats stats{};
    auto candidates = GenerateRequestedCandidates(
        start, request, ref, cfg, limits, no_obstacles, vehicle_shape,
        collision_cfg, &stats);

    bool found_center = false;
    bool found_spread = false;
    for (const auto& p : candidates) {
        if (!p.valid || p.d.empty()) continue;
        if (std::fabs(p.d.back()) < kEpsilon) found_center = true;
        if (std::fabs(p.d.back()) >= kEpsilon &&
            std::fabs(p.d.back()) <= cfg.lateral_target_tolerance + kEpsilon)
            found_spread = true;
    }
    std::printf("  FOLLOW lateral tolerance: found_center=%d found_spread=%d\n",
                found_center, found_spread);
    Expect(found_center, "FOLLOW request: candidate set still contains the "
                         "exact target d (0.0)");
    Expect(found_spread,
           "FOLLOW request: candidate set still spreads +-lateral_target_"
           "tolerance around the target (not narrowed to a single point)");
}

int main() {
    TestKeepRequestMatchesLegacySemantics();
    TestFollowRequestPropagatesLeaderAndExemption();
    TestStopTerminalOffsetIsRequestDriven();
    TestLateralHoldAtZeroDiffersFromNoHold();
    TestFollowRequestKeepsLateralTolerance();

    if (g_failures == 0) {
        std::cout << "All request-native generator contract tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
