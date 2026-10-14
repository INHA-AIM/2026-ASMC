// C4 regression: 외부 STOP 명령의 실행 통합.
//
// 핵심은 "STOP을 연결했다"가 아니라 receding horizon(time_horizon 상한 5초)
// 안에서 STOP이 실제로 실행 가능한가이다. GenerateStoppingCandidates는
// s(T)=stop_s, v(T)=0, a(T)=0을 강제하므로 목표가 멀면 모든 후보가 가속도
// 필터에서 탈락한다(실측: 5m/s에서 30m 앞 -> longitudinal 0/56 valid).
// 그래서 STOP을 APPROACH / BRAKING / HOLD로 나누고, 이 파일이 그 경계를 고정한다.

#include "Frenet/frenet_planner.hpp"

#include "Frenet/Reference/Reference.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using Reason = FrenetPlanner::PlanFailureReason;

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

struct Waypoints {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
};

Waypoints LoadWaypoints(const std::string& path) {
    std::ifstream input(path);
    Waypoints wp;
    double x = 0.0;
    double y = 0.0;
    double ignored = 0.0;
    while (input >> x >> y >> ignored) {
        wp.x.push_back(x);
        wp.y.push_back(y);
    }
    wp.s.assign(wp.x.size(), 0.0);
    for (std::size_t i = 1; i < wp.x.size(); ++i) {
        wp.s[i] = wp.s[i - 1] +
            std::hypot(wp.x[i] - wp.x[i - 1], wp.y[i] - wp.y[i - 1]);
    }
    return wp;
}

std::size_t IndexAtArcLength(const Waypoints& wp, double target_s) {
    for (std::size_t i = 1; i < wp.s.size(); ++i) {
        if (wp.s[i] >= target_s) return i;
    }
    return wp.s.empty() ? 0 : wp.s.size() - 1;
}

CartesianState PoseAtArcLength(const Waypoints& wp, double target_s, double speed) {
    const std::size_t i = IndexAtArcLength(wp, target_s);
    CartesianState pose{};
    pose.x = wp.x[i];
    pose.y = wp.y[i];
    pose.yaw = std::atan2(wp.y[i] - wp.y[i - 1], wp.x[i] - wp.x[i - 1]);
    pose.v = speed;
    return pose;
}

// 궤적 전체가 유한한지. NaN 경계조건이 새어 나오면 여기서 걸린다.
bool AllFinite(const CartesianPath& path) {
    for (std::size_t i = 0; i < path.x.size(); ++i) {
        if (!std::isfinite(path.x[i]) || !std::isfinite(path.y[i]) ||
            !std::isfinite(path.yaw[i]) || !std::isfinite(path.kappa[i]) ||
            !std::isfinite(path.v[i]) || !std::isfinite(path.a[i])) {
            return false;
        }
    }
    return true;
}

bool NoReverse(const CartesianPath& path) {
    for (double v : path.v) {
        if (v < -1e-6) return false;
    }
    return true;
}

double MaxSpeed(const CartesianPath& path) {
    double m = 0.0;
    for (double v : path.v) m = std::max(m, v);
    return m;
}

double MinAcceleration(const CartesianPath& path) {
    double minimum = std::numeric_limits<double>::infinity();
    for (double acceleration : path.a) minimum = std::min(minimum, acceleration);
    return minimum;
}

double MaxAbsCurvature(const CartesianPath& path) {
    double maximum = 0.0;
    for (double curvature : path.kappa) {
        maximum = std::max(maximum, std::fabs(curvature));
    }
    return maximum;
}

double LateralOffsetAt(const RefLine& reference, double x, double y,
                       double near_s) {
    const double projected_s = FindClosestS(reference, x, y, &near_s, 30.0);
    const RefPoint point = Interpolate(reference, projected_s);
    return -std::sin(point.theta) * (x - point.x) +
            std::cos(point.theta) * (y - point.y);
}

constexpr double kEgoS = 400.0;  // MERGE 구간과 무관한 평범한 주행 구간

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string config = package_src + "/../config/frenet/params.yaml";
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;

    const Waypoints wp = LoadWaypoints(waypoint_file);
    if (wp.x.size() < 100) {
        std::cerr << "[FAIL] waypoint load\n";
        return 1;
    }
    const double track_length = wp.s.back();

    // planner가 내부적으로 쓰는 RefLine. BuildRefLine이 균일 재샘플링을 하므로
    // 호길이 파라미터화가 waypoint 파일의 누적 s와 다르다. 투영 검증은 반드시
    // 이 좌표계에서 해야 한다.
    ReferenceConfig reference_config{};
    reference_config.closure_tolerance_m = 0.05;
    reference_config.smoothing_window_m = 10.0;
    reference_config.resample_ds = 0.5;
    reference_config.relax_max_iter = 300;
    reference_config.relax_step = 0.3;
    reference_config.ego_projection_search_window = 30.0;

    RefLine reference;
    if (!LoadReferenceLine(waypoint_file, reference, reference_config, /*max_curvature=*/0.2) ||
        reference.points.empty()) {
        std::cerr << "[FAIL] reference line load\n";
        return 1;
    }
    const double reference_length = reference.points.back().s;

    const std::vector<TrackedObject> no_objects;
    CartesianPath path;

    // planner를 매번 새로 만들어 StopContext(phase latch)가 케이스 사이에
    // 새지 않게 한다.
    auto make_planner = []() {
        auto planner = std::make_unique<FrenetPlanner>();
        return planner;
    };
    const std::string cfg = config;
    auto fresh = [&](std::unique_ptr<FrenetPlanner>& p) {
        p = make_planner();
        return p->Init(cfg);
    };

    std::unique_ptr<FrenetPlanner> planner;
    if (!fresh(planner)) {
        std::cerr << "[FAIL] planner initialization\n";
        return 1;
    }

    // --- Cartesian target -> Frenet s 투영 --------------------------------
    {
        // RefLine의 호길이 파라미터화는 waypoint 누적거리와 상수 오프셋이 있다
        // (BuildRefLine이 자체적으로 s를 다시 계산한다). 따라서 절대값이 아니라
        // ego 대비 상대 전방거리로 검증한다 - STOP이 실제로 쓰는 값도 이쪽이다.
        const std::size_t target_index = IndexAtArcLength(wp, kEgoS + 30.0);
        const CartesianState ego_pose = PoseAtArcLength(wp, kEgoS, 5.0);
        double ego_projected = 0.0;
        double projected = 0.0;
        const bool ego_ok = planner->ProjectEgoToS(ego_pose, ego_projected);
        const bool ok = planner->ProjectPointToS(
            wp.x[target_index], wp.y[target_index], ego_projected, projected);
        const double forward =
            projected - ego_projected;
        const double expected_forward = wp.s[target_index] - wp.s[IndexAtArcLength(wp, kEgoS)];
        Expect(ego_ok && ok && std::fabs(forward - expected_forward) < 2.0,
               "Cartesian stop target projects to the matching forward distance");

        double unused = 0.0;
        Expect(!planner->ProjectPointToS(
                   std::numeric_limits<double>::quiet_NaN(), 0.0, kEgoS, unused),
               "NaN stop target is rejected by projection");

        // 경로에서 크게 벗어난 점은 최근접 s가 나오더라도 이 경로의 정지
        // 지점이 아니다 (max_lateral_offset 초과).
        Expect(!planner->ProjectPointToS(wp.x[target_index] + 500.0,
                                         wp.y[target_index] + 500.0,
                                         kEgoS, unused),
               "stop target far from the reference line is rejected");
    }

    // --- 원거리 정지 목표 투영: 탐색창 회귀 --------------------------------
    // FindClosestS의 기본 탐색창은 30m(ego localization용)다. 정지 목표에 그
    // 창을 쓰면 목표가 창 밖으로 나가고, 잘린 초기 추정값에서 출발한 Newton이
    // 창 경계에 머무는 게 아니라 트랙의 다른 구간으로 발산한다. 실측 기준
    // 전방 80m에서 트랙의 2.7%, 100m에서 4.0% 지점이 이 문제로 정상 STOP을
    // INVALID_COMMAND로 거부했다(최악 오차 735m).
    //
    // 특정 지점 하나만 검사하면 그 소수 구간을 그대로 놓치므로 트랙 전체를
    // 훑는다. ego/목표 모두 planner가 실제로 쓰는 RefLine 좌표에서 만든다 -
    // waypoint 파일의 누적 s는 RefLine 재샘플링 때문에 값이 다르다.
    {
        if (!fresh(planner)) return 1;
        const double lookaheads[] = {50.0, 80.0, 100.0};
        int failures = 0;
        int checked = 0;
        for (double lookahead : lookaheads) {
            for (double sample_s = 0.0; sample_s < reference_length; sample_s += 2.0) {
                const RefPoint target = Interpolate(
                    reference, std::fmod(sample_s + lookahead, reference_length));
                double projected = 0.0;
                ++checked;
                if (!planner->ProjectPointToS(target.x, target.y,
                                              sample_s, projected)) {
                    ++failures;
                    continue;
                }
                double forward = projected - sample_s;
                if (forward < -0.5 * reference_length) forward += reference_length;
                if (forward > 0.5 * reference_length) forward -= reference_length;
                if (std::fabs(forward - lookahead) > 1.0) ++failures;
            }
        }
        Expect(checked > 1500, "projection sweep covered the whole track");
        Expect(failures == 0,
               "StopTargetProjectionSurvivesTheDefaultSearchWindowAcrossTheTrack");
    }

    // --- 80m STOP은 정상 명령으로 계획되어야 한다 --------------------------
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 5.0);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        const std::size_t target_index = IndexAtArcLength(wp, kEgoS + 80.0);
        double stop_s = 0.0;
        const bool projected = planner->ProjectPointToS(
            wp.x[target_index], wp.y[target_index], ego_s, stop_s);
        Expect(projected && std::fabs((stop_s - ego_s) - 80.0) < 5.0,
               "80m stop target projects to the matching forward distance");

        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = stop_s;
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok && !path.x.empty() && AllFinite(path),
               "80m STOP produces a valid trajectory");
        Expect(ok || planner->last_failure_reason() != Reason::INVALID_COMMAND,
               "80m STOP is never rejected as INVALID_COMMAND");
    }

    // --- 30m STOP: 짧은 horizon 때문에 실패하면 안 된다 --------------------
    // 이 테스트가 C4의 핵심이다. 수정 전에는 longitudinal 0/56 valid로
    // 후보가 하나도 남지 않아 Plan()이 false였다.
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 5.0);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = ego_s + 30.0;

        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok, "StopThirtyMetersAheadDoesNotFailBecauseOfShortHorizon");
        Expect(ok && !path.x.empty() && AllFinite(path),
               "30m STOP produces a finite trajectory");
        Expect(ok && planner->last_mode() == STOP,
               "30m STOP still reports STOP mode while approaching");
        Expect(ok && NoReverse(path), "30m STOP never commands reverse motion");
    }

    // --- APPROACH 속도 프로파일: 가까워질수록 단조 감소 ---------------------
    // 30m/5m/s에서는 아직 감속할 필요가 없다(제동거리 6.25m). APPROACH가
    // 실제로 작동하는지는 거리를 좁혀가며 목표속도가 낮아지는지로 검증한다.
    {
        const double distances[] = {25.0, 20.0, 15.0, 10.0};
        double previous = std::numeric_limits<double>::infinity();
        bool monotonic = true;
        bool all_ok = true;
        for (double distance : distances) {
            if (!fresh(planner)) return 1;
            const CartesianState ego = PoseAtArcLength(wp, kEgoS, 5.0);
            double ego_s = 0.0;
            planner->ProjectEgoToS(ego, ego_s);
            PlannerCommand stop{};
            stop.mode = STOP;
            stop.stop_position = ego_s + distance;
            const bool ok = planner->Plan(ego, no_objects, stop, path);
            all_ok = all_ok && ok && !path.x.empty() && AllFinite(path);
            const double target = planner->last_target_speed();
            monotonic = monotonic && target <= previous + 1e-6;
            previous = target;
        }
        Expect(all_ok, "APPROACH produces a valid trajectory at every distance");
        Expect(monotonic,
               "APPROACH target speed decreases monotonically as the stop nears");
        Expect(previous < 10.0,
               "APPROACH target speed is below free cruise at 10m");
    }

    // --- 원거리 STOP은 순항 속도를 과하게 깎지 않는다 ----------------------
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 5.0);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand far_stop{};
        far_stop.mode = STOP;
        far_stop.stop_position = ego_s + 200.0;
        const bool ok = planner->Plan(ego, no_objects, far_stop, path);
        Expect(ok && !path.x.empty() && AllFinite(path),
               "distant STOP keeps producing a valid trajectory");
        Expect(ok && MaxSpeed(path) > 0.5,
               "distant STOP does not brake to a crawl immediately");
    }

    // --- 근거리 STOP: 실제 정지 궤적 ---------------------------------------
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 3.0);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = kEgoS + 5.0;
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        stop.stop_position = ego_s + 5.0;
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok && !path.x.empty() && AllFinite(path),
               "5m STOP produces a finite trajectory");
        Expect(ok && NoReverse(path), "5m STOP never commands reverse motion");
        Expect(ok && path.v.back() <= 1e-6,
               "5m STOP trajectory ends at zero speed");
    }

    // --- 아주 가까운 STOP: 후보가 없어도 정지해야 한다 ---------------------
    // 5m/s에서 3m 앞 정지는 exact-stop quintic이 가속도 한계를 넘어 후보가
    // 0개가 된다. 여기서 Plan()이 false를 반환하면 노드가 이전 궤적을 재사용해
    // 정지 목표를 지나쳐 계속 주행한다 - BRAKING fail-safe가 이를 막는다.
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 5.0);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = ego_s + 3.0;
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok, "very close STOP still returns a trajectory (BRAKING fail-safe)");
        Expect(ok && !path.x.empty() && AllFinite(path),
               "BRAKING fail-safe trajectory is finite");
        Expect(ok && NoReverse(path),
               "BRAKING fail-safe never commands reverse motion");
        Expect(ok && path.v.back() <= 1e-6,
               "BRAKING fail-safe decelerates to zero speed");
    }

    // --- 늦게 발견한 STOP은 comfortable 감속보다 강하게 제동 -------------
    // 50km/h에서 15m는 1.5m/s²로 멈출 수 없다. 정상 후보가 없더라도
    // fallback은 남은 거리 기반으로 서비스 제동 한계까지 강화되어야 한다.
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 13.898889);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = ego_s + 15.0;
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok && !path.x.empty() && AllFinite(path),
               "late high-speed STOP produces a finite trajectory");
        Expect(ok && MinAcceleration(path) < -3.0,
               "late high-speed STOP escalates beyond comfortable braking");
        Expect(ok && MinAcceleration(path) >= -5.0 - 1e-6,
               "normal STOP stays within the configured service brake limit");
    }

    // --- 이미 정지해 있으면 HOLD (creep 금지) ------------------------------
    // C1에서 STOP을 저속 fallback에서 제외한 이유가 이것이다. fallback은
    // kMinCreepSpeed로 계속 전진하므로 정지 목표를 넘어가 버린다.
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 0.0);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = ego_s + 0.2;  // position_tolerance 안
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok && !path.x.empty() && AllFinite(path),
               "stopped vehicle at the target produces a HOLD trajectory");
        Expect(ok && MaxSpeed(path) <= 1e-9,
               "HOLD never creeps forward");
        Expect(ok && planner->last_mode() == STOP,
               "HOLD still reports STOP mode");
    }

    // --- stop_buffer 경계의 투영 잡음에도 HOLD를 latch ---------------------
    // 실측에서는 stop_buffer=0.5m 지점에 정지한 뒤 s 투영이 경계 양쪽으로
    // 흔들리며 0속도/재출발 궤적을 반복했다. 0.2m의 진입 여유 안에서 한 번
    // HOLD하면 같은 STOP target 동안 다시 BRAKING으로 내려가지 않아야 한다.
    {
        if (!fresh(planner)) return 1;
        CartesianState ego = PoseAtArcLength(wp, kEgoS, 0.1);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = ego_s + 0.65;
        const bool entered = planner->Plan(ego, no_objects, stop, path);
        Expect(entered && planner->stop_holding() && MaxSpeed(path) <= 1e-9,
               "stopped vehicle inside HOLD entry margin never creeps");

        // localization이 뒤로 0.2m 흔들려도 target이 같으면 HOLD 유지.
        ego = PoseAtArcLength(wp, kEgoS - 0.2, 0.0);
        const bool held = planner->Plan(ego, no_objects, stop, path);
        Expect(held && planner->stop_holding() && MaxSpeed(path) <= 1e-9,
               "HOLD remains latched across stop-distance noise");
    }

    // --- 저속 STOP 출력 곡률은 후보 필터와 동일한 기하값 사용 ------------
    // d_dot/s_dot 기반 analytic 변환은 정지 직전 s_dot->0에서 실측
    // kappa=8.46/m까지 증폭됐다. 위치 기반 곡률로 출력하면 MPC의 차량 한계
    // (tan(40deg)/3m ~= 0.2797/m)를 넘지 않아야 한다.
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 0.3);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = ego_s + 1.0;
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok && !path.x.empty() && AllFinite(path),
               "low-speed STOP produces a finite trajectory");
        Expect(ok && MaxAbsCurvature(path) <= 0.2797 + 1e-6,
               "low-speed STOP output respects the MPC curvature limit");
    }

    // --- 저속 STOP은 안전한 현재 횡위치에서 차체만 정렬 -------------------
    {
        if (!fresh(planner)) return 1;
        CartesianState ego = PoseAtArcLength(wp, kEgoS, 0.5);
        const double initial_d = 0.20;
        ego.x += -std::sin(ego.yaw) * initial_d;
        ego.y +=  std::cos(ego.yaw) * initial_d;
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = ego_s + 2.0;
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        const double end_d = ok && !path.x.empty()
            ? LateralOffsetAt(reference, path.x.back(), path.y.back(), ego_s)
            : 1e9;
        Expect(ok && std::fabs(end_d - initial_d) < 0.05,
               "low-speed STOP preserves a safe in-lane lateral offset");
    }

    // --- 목표를 살짝 지나쳤을 때 후진하지 않는다 ---------------------------
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 0.3);
        double ego_s = 0.0;
        planner->ProjectEgoToS(ego, ego_s);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = ego_s - 1.0;  // 이미 1m 지나침
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok && !path.x.empty() && AllFinite(path),
               "overshot STOP still produces a trajectory");
        Expect(ok && NoReverse(path),
               "overshot STOP never commands reverse motion");
    }

    // --- 폐루프 이음새 너머의 정지 목표 ------------------------------------
    // ego가 트랙 끝 근처, 목표는 이음새를 넘어간 s. 단순 (target - ego)는
    // 크게 음수가 되어 "이미 지나쳤다"로 오판한다.
    {
        if (!fresh(planner)) return 1;
        const double ego_s = track_length - 15.0;
        const CartesianState ego = PoseAtArcLength(wp, ego_s, 5.0);
        PlannerCommand stop{};
        stop.mode = STOP;
        stop.stop_position = 15.0;  // 이음새 너머 = 실제로는 전방 약 30m
        const bool ok = planner->Plan(ego, no_objects, stop, path);
        Expect(ok && !path.x.empty() && AllFinite(path),
               "closed-loop seam STOP target is treated as ahead, not behind");
        Expect(ok && NoReverse(path),
               "closed-loop seam STOP never commands reverse motion");
    }

    // --- 잘못된 STOP은 궤적을 만들지 않는다 --------------------------------
    {
        if (!fresh(planner)) return 1;
        const CartesianState ego = PoseAtArcLength(wp, kEgoS, 5.0);

        PlannerCommand keep{};
        keep.mode = LANE_KEEPING;
        Expect(planner->Plan(ego, no_objects, keep, path) && !path.x.empty(),
               "KEEP succeeds before the invalid STOP transition");
        const CartesianPath keep_path = path;

        PlannerCommand bad_stop{};
        bad_stop.mode = STOP;  // stop_position 미지정(NaN)
        const bool rejected = !planner->Plan(ego, no_objects, bad_stop, path);
        Expect(rejected &&
                   planner->last_failure_reason() == Reason::INVALID_COMMAND,
               "KEEP -> invalid STOP is rejected as INVALID_COMMAND");
        // 노드는 이 reason에서 이전 궤적을 재사용하지 않고 즉시 정지한다(C2).
        Expect(rejected &&
                   planner->last_failure_reason() !=
                       Reason::TRANSIENT_GENERATION_FAILURE,
               "invalid STOP is never reported as a transient failure");
        Expect(!keep_path.x.empty(),
               "previous KEEP trajectory existed and must not be reused by the node");
    }

    // A stopped vehicle before the same target uses bounded reapproach,
    // independently of the normal cruise speed and service brake strength.
    {
        if (!fresh(planner)) return 1;
        auto ego = PoseAtArcLength(wp, kEgoS, 0.0);
        double station=0;planner->ProjectEgoToS(ego,station);
        PlannerCommand stop{};stop.mode=STOP;stop.has_target_speed=true;
        stop.target_speed=13.8889;stop.stop_position=station+2.56;
        const bool ok=planner->Plan(ego,no_objects,stop,path);
        Expect(ok&&path.v.size()>1,"early STOP restart produces a moving stop-capable trajectory");
        double peak_v=0,peak_a=0;
        for(double v:path.v)peak_v=std::max(peak_v,v);
        for(double a:path.a)peak_a=std::max(peak_a,a);
        Expect(ok&&peak_v<=1.05,"STOP restart does not inherit the normal cruise speed");
        Expect(ok&&peak_a<=.51,"STOP restart limits positive acceleration independently of braking");
        double end=0;
        const bool projected=ok&&planner->ProjectPointToS(path.x.back(),path.y.back(),station,end);
        Expect(projected&&end<=stop.stop_position+.02&&path.v.back()<1e-6,
               "restart keeps a zero-speed endpoint before the unchanged target");
    }
    // A late braking fallback must not request another throttle pulse merely
    // because the measured starting acceleration was positive.
    {
        if (!fresh(planner)) return 1;
        auto ego=PoseAtArcLength(wp,kEgoS,2.35);ego.a=3.17;
        double station=0;planner->ProjectEgoToS(ego,station);
        PlannerCommand stop{};stop.mode=STOP;stop.has_target_speed=true;
        stop.target_speed=13.8889;stop.stop_position=station+1.4;
        const bool ok=planner->Plan(ego,no_objects,stop,path);
        Expect(ok&&path.v.size()>1,"late accelerating STOP retains checked steerable braking");
        double peak_v=0;for(double v:path.v)peak_v=std::max(peak_v,v);
        Expect(ok&&peak_v<=ego.v+.01,"braking fallback does not repeat measured throttle acceleration");
    }

    if (g_failures == 0) {
        std::cout << "All command STOP plan tests passed\n";
        return 0;
    }
    return 1;
}

