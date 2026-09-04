// D1-3: legacy(간이 FSM) 행동 결정의 회귀 커버리지.
//
// 왜 필요한가:
//   기존 테스트는 legacy 경로의 행동 결정을 검증하지 않는다.
//   command_keep_plan_test는 legacy Plan()을 한 번 호출하지만 "merge zone에서
//   MERGE가 활성화된다"만 확인하고, AVOID/FOLLOW 결정과 우선순위 체인
//   (AVOID > MERGE > FOLLOW > KEEP)은 어떤 테스트도 커버하지 않았다.
//
//   D1-2에서 그 결정 블록을 SelectLegacyBehavior()로 추출했고, D5/D6/D8에서
//   모드별로 legacy 조각을 제거할 예정이다. 그 작업들이 검증된 회전교차로/
//   고속도로 주행(커밋 711d536)을 깨지 않았는지 확인할 안전망이 이 테스트다.
//
// 무엇을 검증하는가:
//   행동 결정의 관측 가능한 결과(last_mode)와 그 결정이 만든 궤적의 성질.
//   특정 좌표값을 고정하지 않는다 - 튜닝 파라미터가 바뀌면 깨지는 테스트는
//   회귀 안전망으로 쓸 수 없다. 대신 "무엇이 반드시 성립해야 하는가"를 본다.
//
// 참고: docs/d1-legacy-behavior-boundary.md

#include "frenet_planner/frenet_planner.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

// params.yaml의 waypoint_file은 배포 환경의 절대경로(/root/ws/...)로 되어 있어
// 그 경로가 없는 환경에서는 Init이 실패한다. 테스트는 환경에 의존하면 안 되므로
// waypoint_file만 FRENET_TEST_WAYPOINT_FILE로 바꾼 사본을 만들어 사용한다.
// (production 설정은 건드리지 않는다.)
std::string MakeLocalConfig(const std::string& source_config,
                            const std::string& waypoint_file,
                            const std::string& out_path) {
    std::ifstream in(source_config);
    if (!in) return {};
    std::ofstream out(out_path);
    if (!out) return {};
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("waypoint_file:", 0) == 0) {
            out << "waypoint_file: \"" << waypoint_file << "\"\n";
        } else {
            out << line << '\n';
        }
    }
    return out_path;
}

struct Waypoints {
    std::vector<double> x;
    std::vector<double> y;
};

Waypoints LoadWaypoints(const std::string& path) {
    std::ifstream in(path);
    Waypoints wp;
    double x = 0.0, y = 0.0, z = 0.0;
    while (in >> x >> y >> z) { wp.x.push_back(x); wp.y.push_back(y); }
    return wp;
}

// 누적 호길이가 target_s에 도달하는 waypoint의 pose.
CartesianState PoseAt(const Waypoints& wp, double target_s, double speed) {
    double s = 0.0;
    std::size_t i = 1;
    for (; i < wp.x.size(); ++i) {
        s += std::hypot(wp.x[i] - wp.x[i - 1], wp.y[i] - wp.y[i - 1]);
        if (s >= target_s) break;
    }
    if (i >= wp.x.size()) i = wp.x.size() - 1;
    CartesianState c{};
    c.x = wp.x[i];
    c.y = wp.y[i];
    c.yaw = std::atan2(wp.y[i] - wp.y[i - 1], wp.x[i] - wp.x[i - 1]);
    c.v = speed;
    return c;
}

ObjectInfo MakeObject(int id, const CartesianState& pose, double speed) {
    ObjectInfo o;
    o.id = id;
    o.x = pose.x;
    o.y = pose.y;
    o.heading = pose.yaw;
    o.speed = speed;
    o.vx = speed * std::cos(pose.yaw);
    o.vy = speed * std::sin(pose.yaw);
    o.width = 2.0;
    o.length = 4.5;
    o.height = 1.5;
    return o;
}

const char* ModeName(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING: return "KEEP";
        case FOLLOWING: return "FOLLOW";
        case AVOID: return "AVOID";
        case MERGE: return "MERGE";
        case STOP: return "STOP";
        case EMERGENCY: return "EMERGENCY";
        default: return "OTHER";
    }
}

// 시나리오를 정해진 사이클만큼 돌리고 관측된 mode 목록을 반환한다.
// ego는 매 사이클 자기 궤적을 따라 전진해 실제 주행에 가깝게 만든다.
std::vector<BehaviorState> RunScenario(const std::string& config,
                                       const Waypoints& wp,
                                       double start_s, double start_v,
                                       const std::vector<ObjectInfo>& objects,
                                       int cycles,
                                       bool* all_paths_valid = nullptr) {
    std::vector<BehaviorState> modes;
    if (all_paths_valid) *all_paths_valid = true;

    // 시나리오마다 새 planner를 써서 context(avoidance_/following_/merge_)가
    // 케이스 사이에 새지 않게 한다.
    FrenetPlanner planner;
    if (!planner.Init(config)) return modes;

    CartesianState ego = PoseAt(wp, start_s, start_v);
    for (int c = 0; c < cycles; ++c) {
        CartesianPath path;
        const bool ok = planner.Plan(ego, objects, path);
        modes.push_back(planner.last_mode());
        if (!ok && all_paths_valid) *all_paths_valid = false;
        if (ok && path.x.size() > 1) {
            const std::size_t k = std::min<std::size_t>(2, path.x.size() - 1);
            ego.x = path.x[k];
            ego.y = path.y[k];
            ego.yaw = path.yaw[k];
            ego.v = path.v[k];
        }
    }
    return modes;
}

bool Contains(const std::vector<BehaviorState>& modes, BehaviorState mode) {
    for (BehaviorState m : modes) if (m == mode) return true;
    return false;
}

bool AllAre(const std::vector<BehaviorState>& modes, BehaviorState mode) {
    if (modes.empty()) return false;
    for (BehaviorState m : modes) if (m != mode) return false;
    return true;
}

std::string Describe(const std::vector<BehaviorState>& modes) {
    std::string s;
    for (BehaviorState m : modes) { s += ModeName(m); s += " "; }
    return s;
}

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string source_config =
        package_src + "/frenet_planner/config/params.yaml";

    const Waypoints wp = LoadWaypoints(waypoint_file);
    if (wp.x.size() < 100) {
        std::cerr << "[FAIL] waypoint load (" << waypoint_file << ")\n";
        return 1;
    }

    const std::string config = MakeLocalConfig(
        source_config, waypoint_file,
        "legacy_behavior_regression_test_params.yaml");
    if (config.empty()) {
        std::cerr << "[FAIL] could not prepare test config\n";
        return 1;
    }

    {
        FrenetPlanner probe;
        if (!probe.Init(config)) {
            std::cerr << "[FAIL] planner initialization\n";
            return 1;
        }
    }

    // params.yaml의 회전교차로 conflict_s. 아래 MERGE 시나리오가 이 값을
    // 기준으로 접근 구간을 잡는다.
    constexpr double kRoundaboutConflictS = 885.76;
    // 회전교차로/저속 fallback과 무관한 평범한 직선 주행 구간.
    constexpr double kPlainS = 200.0;

    // --- 1. 빈 도로: KEEP을 유지해야 한다 -------------------------------
    {
        bool paths_ok = false;
        const auto modes = RunScenario(config, wp, kPlainS, 8.0, {}, 5, &paths_ok);
        Expect(AllAre(modes, LANE_KEEPING),
               "empty road stays in KEEP (observed: " + Describe(modes) + ")");
        Expect(paths_ok, "empty road produces a valid trajectory every cycle");
    }

    // --- 2. 같은 차선 전방 저속 차량: FOLLOW를 획득해야 한다 --------------
    {
        const CartesianState leader_pose = PoseAt(wp, kPlainS + 35.0, 4.0);
        const std::vector<ObjectInfo> objects{MakeObject(11, leader_pose, 4.0)};
        bool paths_ok = false;
        const auto modes = RunScenario(config, wp, kPlainS, 8.0, objects, 5, &paths_ok);
        Expect(Contains(modes, FOLLOWING),
               "slower leader ahead triggers FOLLOW (observed: " + Describe(modes) + ")");
        Expect(!Contains(modes, AVOID),
               "moving leader does not trigger AVOID");
        Expect(paths_ok, "FOLLOW produces a valid trajectory every cycle");
    }

    // --- 3. 같은 차선 전방 정지 장애물: AVOID로 전환해야 한다 -------------
    {
        const CartesianState obstacle_pose = PoseAt(wp, kPlainS + 25.0, 0.0);
        const std::vector<ObjectInfo> objects{MakeObject(21, obstacle_pose, 0.0)};
        const auto modes = RunScenario(config, wp, kPlainS, 6.0, objects, 8);
        Expect(Contains(modes, AVOID),
               "static obstacle ahead triggers AVOID (observed: " + Describe(modes) + ")");
    }

    // --- 4. 우선순위: AVOID가 FOLLOW를 이겨야 한다 ------------------------
    // 정적 장애물과 선행 차량이 동시에 있을 때, SelectLegacyBehavior의
    // if-else 순서(AVOID > MERGE > FOLLOW > KEEP)가 실제로 지켜지는지 본다.
    // D5/D6에서 legacy 조각을 제거할 때 이 우선순위가 깨지기 쉽다.
    {
        const CartesianState obstacle_pose = PoseAt(wp, kPlainS + 25.0, 0.0);
        const CartesianState leader_pose = PoseAt(wp, kPlainS + 50.0, 4.0);
        const std::vector<ObjectInfo> objects{
            MakeObject(21, obstacle_pose, 0.0),
            MakeObject(11, leader_pose, 4.0)};
        const auto modes = RunScenario(config, wp, kPlainS, 6.0, objects, 8);
        Expect(Contains(modes, AVOID),
               "obstacle + leader still reaches AVOID (observed: " + Describe(modes) + ")");
        // AVOID가 한 번 잠기면 그 뒤로 FOLLOW로 되돌아가지 않아야 한다.
        bool follow_after_avoid = false;
        bool seen_avoid = false;
        for (BehaviorState m : modes) {
            if (m == AVOID) seen_avoid = true;
            else if (seen_avoid && m == FOLLOWING) follow_after_avoid = true;
        }
        Expect(!follow_after_avoid,
               "AVOID outranks FOLLOW once locked (observed: " + Describe(modes) + ")");
    }

    // --- 5. 회전교차로 접근: MERGE가 활성화되어야 한다 --------------------
    {
        const auto modes = RunScenario(
            config, wp, kRoundaboutConflictS - 30.0, 5.0, {}, 6);
        Expect(AllAre(modes, MERGE),
               "roundabout approach activates MERGE (observed: " + Describe(modes) + ")");
    }

    // --- 6. 저속 fallback: merge zone 밖에서는 KEEP ----------------------
    {
        const auto modes = RunScenario(config, wp, kPlainS, 0.2, {}, 3);
        Expect(AllAre(modes, LANE_KEEPING),
               "low speed outside merge zone stays KEEP (observed: "
               + Describe(modes) + ")");
    }

    // --- 7. 저속 + merge zone: fallback을 우회해 MERGE를 유지해야 한다 ----
    // near_any_merge 가드가 사라지면 저속 fallback이 먼저 return해서
    // conflict point 앞에서 데드락이 발생한다(d1 문서 2.2절).
    {
        const auto modes = RunScenario(
            config, wp, kRoundaboutConflictS - 15.0, 0.2, {}, 3);
        Expect(AllAre(modes, MERGE),
               "low speed inside merge zone bypasses the fallback and keeps MERGE "
               "(observed: " + Describe(modes) + ")");
    }

    // --- 8. 외부 KEEP 명령은 legacy 결정을 타지 않아야 한다 ---------------
    // v2 경로가 legacy 행동 선택권과 완전히 분리되어 있음을 확인한다.
    // merge zone 안에서도 외부 KEEP은 KEEP으로 남아야 한다.
    {
        FrenetPlanner planner;
        if (!planner.Init(config)) {
            std::cerr << "[FAIL] planner initialization (external case)\n";
            return 1;
        }
        const CartesianState ego =
            PoseAt(wp, kRoundaboutConflictS - 30.0, 5.0);
        CartesianPath path;

        // 먼저 legacy 호출로 내부 MERGE context를 활성화한다.
        planner.Plan(ego, {}, path);
        Expect(planner.last_mode() == MERGE,
               "legacy call activates MERGE inside the zone");

        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        const bool keep_ok = planner.Plan(ego, {}, keep, path);
        Expect(keep_ok && planner.last_mode() == LANE_KEEPING,
               "external KEEP ignores legacy merge-zone selection");
    }

    std::remove(config.c_str());

    if (g_failures != 0) {
        std::cerr << g_failures << " legacy behavior regression check(s) failed\n";
        return 1;
    }
    std::cout << "all legacy behavior regression checks passed\n";
    return 0;
}
