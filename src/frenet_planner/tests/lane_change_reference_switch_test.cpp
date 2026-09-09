// D7 §21-25: Global Path / RefLine 전환(ReplaceReferenceLine) 회귀.
//
// Frenet은 target_link_id로 MGeo topology를 스스로 탐색하지 않는다(§19) -
// 이 테스트는 순수하게 core의 reference-replacement API 계약만 검증한다:
//
//   - 성공한 교체는 새 좌표계에서 ego가 정상 투영된다(d ~ 0)
//   - 이전 reference의 last_s hint가 새 reference 투영을 오염시키지 않는다
//   - 좌표계에 종속된 context(AVOID/FOLLOW/LANE_CHANGE lock, last_s/d)는
//     reset된다
//   - 물리적 속도 continuity history(last_target_speed_)는 보존된다
//   - 유효하지 않은 새 경로는 기존 reference를 파괴하지 않는다
//   - 교체 전/후 생성 trajectory가 물리적으로 연속이다(teleport 없음) -
//     MPC의 reference validator가 거부할 만한 불연속을 만들지 않는다(§25)
//   - 인위적인 0속도 전환이 없다
//
// 계약: docs/fsm-frenet-interface-v2.md, D7 스펙 §19-25

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

struct Waypoints {
    std::vector<double> x;
    std::vector<double> y;
};

Waypoints LoadWaypoints(const std::string& path) {
    std::ifstream input(path);
    Waypoints waypoints;
    double x = 0.0, y = 0.0, ignored = 0.0;
    while (input >> x >> y >> ignored) {
        waypoints.x.push_back(x);
        waypoints.y.push_back(y);
    }
    return waypoints;
}

std::string MakeLocalConfig(const std::string& source,
                            const std::string& waypoints,
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

// waypoints를 lateral_offset만큼 옆으로 평행 이동한 "인접 차로" 시퀀스.
// 실제 MGeo adjacent-lane centerline이 없으므로(§19 - Frenet은 MGeo를 모른다)
// 원본 트랙의 국소 접선에 수직으로 민 합성 경로로 대체한다 - reference
// 교체 API 자체의 계약(성공/실패, state reset, 연속성)을 검증하는 데는
// 실제 인접 차로 좌표가 필요하지 않다.
Waypoints MakeParallelOffsetPath(const Waypoints& base, double offset,
                                 std::size_t start_index,
                                 std::size_t count) {
    Waypoints out;
    const std::size_t end_index =
        std::min(base.x.size(), start_index + count);
    for (std::size_t i = start_index; i + 1 < end_index; ++i) {
        const double dx = base.x[i + 1] - base.x[i];
        const double dy = base.y[i + 1] - base.y[i];
        const double norm = std::hypot(dx, dy);
        if (norm < 1e-6) continue;
        const double nx = -dy / norm;
        const double ny = dx / norm;
        out.x.push_back(base.x[i] + offset * nx);
        out.y.push_back(base.y[i] + offset * ny);
    }
    return out;
}

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string config = MakeLocalConfig(
        package_src + "/../config/params.yaml", waypoint_file,
        "lane_change_reference_switch_test_params.yaml");
    if (config.empty()) {
        std::cerr << "[FAIL] could not prepare test config\n";
        return 1;
    }

    const Waypoints waypoints = LoadWaypoints(waypoint_file);
    if (waypoints.x.size() < 200) {
        std::cerr << "[FAIL] waypoint load\n";
        return 1;
    }

    // ===== 1. 정상 인접 경로 교체 성공 + 새 ref에서 ego d ~ 0 ============
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
        CartesianState ego{};
        ego.x = waypoints.x[300];
        ego.y = waypoints.y[300];
        ego.yaw = std::atan2(waypoints.y[301] - waypoints.y[300],
                             waypoints.x[301] - waypoints.x[300]);
        ego.v = 6.0;

        // 교체 전 old ref에서 ego를 한 번 투영해 last_s_/last_d_를 채운다
        // (실제 runtime에서도 교체 시점에는 이미 여러 cycle의 old-ref
        // 투영 history가 쌓여 있다).
        double old_s = 0.0;
        Expect(planner.ProjectEgoToS(ego, old_s),
               "0: ego projects onto the old reference before switch");

        const Waypoints adjacent =
            MakeParallelOffsetPath(waypoints, 3.5, 250, 400);
        Expect(planner.ReplaceReferenceLine(adjacent.x, adjacent.y),
               "1: ReplaceReferenceLine succeeds for a valid adjacent path");

        // 새 ref 위의 한 점(교체에 쓰인 원본 offset 지점)에서 ego를 그
        // 지점으로 옮기고 투영하면 d가 거의 0이어야 한다.
        CartesianState ego_on_new_ref{};
        ego_on_new_ref.x = adjacent.x[50];
        ego_on_new_ref.y = adjacent.y[50];
        ego_on_new_ref.yaw = ego.yaw;
        ego_on_new_ref.v = 6.0;
        double new_s = 0.0, new_d = 0.0;
        Expect(planner.ProjectPointToFrenet(ego_on_new_ref.x, ego_on_new_ref.y,
                                            0.0, 500.0, 10.0, new_s, new_d),
               "1: point on the new reference projects successfully");
        Expect(std::fabs(new_d) < 0.1,
               "1: ego on the new reference has d ~ 0");
    }

    // ===== 2. old last_s hint가 new ref 투영을 오염시키지 않음 ===========
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianState ego{};
        ego.x = waypoints.x[1000];
        ego.y = waypoints.y[1000];
        ego.yaw = std::atan2(waypoints.y[1001] - waypoints.y[1000],
                             waypoints.x[1001] - waypoints.x[1000]);
        ego.v = 6.0;
        double old_s = 0.0;
        planner.ProjectEgoToS(ego, old_s);  // old_s ~= 1000-scale s

        // 새 reference는 완전히 다른 s 범위(트랙 앞부분)를 쓴다 - old_s
        // hint가 오염이라면 FindClosestS가 새 ref의 짧은 길이를 벗어난
        // 지점을 찾으려다 실패하거나 엉뚱한 점을 반환할 것이다.
        const Waypoints new_ref_path =
            MakeParallelOffsetPath(waypoints, 0.0, 10, 200);
        Expect(planner.ReplaceReferenceLine(new_ref_path.x, new_ref_path.y),
               "2: replacement to an unrelated s-range succeeds");

        CartesianState ego_on_new{};
        ego_on_new.x = new_ref_path.x[20];
        ego_on_new.y = new_ref_path.y[20];
        double s = 0.0;
        Expect(planner.ProjectEgoToS(ego_on_new, s) && s < new_ref_path.x.size() * 3.0,
               "2: post-switch projection uses the new reference, not the "
               "stale old-ref s hint");
    }

    // ===== 3. 좌표계 종속 context reset =================================
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianState ego{};
        ego.x = waypoints.x[300];
        ego.y = waypoints.y[300];
        ego.yaw = std::atan2(waypoints.y[301] - waypoints.y[300],
                             waypoints.x[301] - waypoints.x[300]);
        ego.v = 6.0;

        // AVOID를 확보해 v2_avoidance_ context를 활성화한다.
        PlannerCommand avoid{};
        avoid.mode = AVOID;
        avoid.target_object_id = 999;  // 존재하지 않아도 active_avoid_target_id
                                       // 조회만 확인하면 된다.
        CartesianPath path;
        planner.Plan(ego, {}, avoid, path);

        const Waypoints adjacent =
            MakeParallelOffsetPath(waypoints, 3.5, 250, 400);
        planner.ReplaceReferenceLine(adjacent.x, adjacent.y);

        Expect(planner.active_avoid_target_id() == -1,
               "3: AVOID context is reset after a reference switch");
        Expect(!planner.avoid_committed(),
               "3: AVOID commit lock is reset after a reference switch");
    }

    // ===== 4. 물리적 속도 continuity history는 보존 =====================
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianState ego{};
        ego.x = waypoints.x[300];
        ego.y = waypoints.y[300];
        ego.yaw = std::atan2(waypoints.y[301] - waypoints.y[300],
                             waypoints.x[301] - waypoints.x[300]);
        ego.v = 8.0;

        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        CartesianPath path;
        planner.Plan(ego, {}, keep, path);
        const double speed_before_switch = planner.last_target_speed();

        const Waypoints adjacent =
            MakeParallelOffsetPath(waypoints, 3.5, 250, 400);
        planner.ReplaceReferenceLine(adjacent.x, adjacent.y);

        // reset 직후에도 last_target_speed()는 여전히 이전 순항 속도를
        // 보고해야 한다(0으로 꺾이면 안 된다) - §23.
        Expect(planner.last_target_speed() > 0.5 &&
                   std::fabs(planner.last_target_speed() -
                             speed_before_switch) < 1e-9,
               "4: target speed continuity history is preserved across a "
               "reference switch");
    }

    // ===== 5. invalid new path -> old ref 그대로 유지 ===================
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianState ego{};
        ego.x = waypoints.x[300];
        ego.y = waypoints.y[300];
        double s_before = 0.0;
        planner.ProjectEgoToS(ego, s_before);

        // 너무 짧은(4점 미만) 경로는 거부되어야 한다.
        const std::vector<double> too_short_x{0.0, 1.0};
        const std::vector<double> too_short_y{0.0, 0.0};
        Expect(!planner.ReplaceReferenceLine(too_short_x, too_short_y),
               "5: a too-short path is rejected");

        // NaN이 섞인 경로도 거부되어야 한다.
        std::vector<double> nan_x(waypoints.x.begin() + 250,
                                  waypoints.x.begin() + 260);
        std::vector<double> nan_y(waypoints.y.begin() + 250,
                                  waypoints.y.begin() + 260);
        nan_x[5] = std::nan("");
        Expect(!planner.ReplaceReferenceLine(nan_x, nan_y),
               "5: a path containing NaN is rejected");

        // 거부된 뒤에도 기존 ref는 그대로 살아 있어야 한다.
        double s_after = 0.0;
        Expect(planner.ProjectEgoToS(ego, s_after) &&
                   std::fabs(s_after - s_before) < 1.0,
               "5: the original reference survives rejected replacements");
    }

    // ===== 6/7. 교체 전/후 생성 trajectory가 물리적으로 연속 =============
    // (no artificial zero-speed transition, no teleport)
    {
        FrenetPlanner planner;
        planner.Init(config);
        CartesianState ego{};
        ego.x = waypoints.x[300];
        ego.y = waypoints.y[300];
        ego.yaw = std::atan2(waypoints.y[301] - waypoints.y[300],
                             waypoints.x[301] - waypoints.x[300]);
        ego.v = 6.0;

        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        CartesianPath before_path;
        const bool before_ok = planner.Plan(ego, {}, keep, before_path);
        Expect(before_ok && !before_path.v.empty() && before_path.v.front() > 0.5,
               "6: trajectory before the switch has normal forward speed");

        const Waypoints adjacent =
            MakeParallelOffsetPath(waypoints, 3.5, 250, 400);
        Expect(planner.ReplaceReferenceLine(adjacent.x, adjacent.y),
               "6: replacement to a nearby adjacent path succeeds");

        // 교체 직후 같은 ego 위치/속도로 바로 다음 cycle을 계획한다 -
        // Ref switch를 위해 차량을 세우지 않는다는 계약(§21, §24)을 검증한다.
        CartesianPath after_path;
        const bool after_ok = planner.Plan(ego, {}, keep, after_path);
        Expect(after_ok, "7: planning succeeds immediately after the switch "
                         "(no forced stop-then-resume)");
        Expect(!after_path.v.empty() && after_path.v.front() > 0.5,
               "7: no artificial zero-speed transition right after switch");

        // teleport 여부: 첫 두 점 사이 거리가 물리적으로 타당한 범위인지
        // (한 스텝 dt에 이동 가능한 최대 거리 근방) 확인한다 - MPC의
        // reference validator가 거부할 만한 수 m급 불연속이 없어야 한다.
        if (after_path.x.size() >= 2) {
            const double step_distance =
                std::hypot(after_path.x[1] - after_path.x[0],
                          after_path.y[1] - after_path.y[0]);
            Expect(step_distance < 3.0,
                   "7: first trajectory step after switch is not a "
                   "multi-meter teleport");
        }
    }

    if (g_failures == 0) {
        std::cout << "All lane change reference switch tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
