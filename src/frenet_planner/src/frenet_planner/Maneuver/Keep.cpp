/**
 * @file Keep.cpp
 * @brief external KEEP(LANE_KEEPING) 전용 candidate 보강과, KEEP/FOLLOW/
 *        STOP이 공유하는 candidate 파이프라인(ExecuteManeuverPipeline).
 *
 * ExecuteManeuverPipeline은 candidate 생성 -> 곡률/충돌 필터 -> cost 평가
 * -> 최적 후보 선택 -> Cartesian 변환까지, KEEP/FOLLOW/STOP이 실제로 동일하게
 * 거치는 pipeline이다. STOP 전용 분기(external_stop_mode)가 이 pipeline
 * 뒷부분 전체에 걸쳐 있어서 모드별로 쪼개면 같은 조건을 여러 곳에서
 * 유지해야 하므로, KEEP 파일에 pipeline 본체를 두고 KeepProcess는 그 얇은
 * 진입점 역할만 한다(frenet_planner.hpp 클래스 선언부 주석 참고). Follow.cpp/
 * Stop.cpp의 FollowProcess/StopProcess는 각자 모드별 준비만 하고 이
 * pipeline을 호출한다.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <utility>

#include "frenet_planner/Evaluation/Cost.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Generator/LateralGenerator.hpp"
#include "frenet_planner/Generator/LongitudinalGenerator.hpp"
#include "frenet_planner/Maneuver/ManeuverUtils.hpp"
#include "frenet_planner/Speed/SpeedPolicy.hpp"

namespace {

const char* ModeName(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING: return "LANE_KEEPING";
        case FOLLOWING: return "FOLLOWING";
        case AVOID: return "AVOID";
        case MERGE: return "MERGE";
        default: return "OTHER";
    }
}

}  // namespace

void FrenetPlanner::AppendExternalKeepContinuityCandidates(
    const FrenetState& maneuver_start, const PlannerCommand& cmd,
    const std::vector<TrackedObject>& planning_collision_obstacles,
    std::vector<FrenetPath>& candidates) const {
    std::vector<FrenetPath> continuity_lateral =
        GenerateLateralContinuityCandidates(maneuver_start, path_cfg_,
                                            limits_.low_speed_fallback_v);
    FilterLateralByAcceleration(continuity_lateral, limits_);

    std::vector<FrenetPath> longitudinal_for_continuity =
        GenerateVelocityKeepingCandidates(maneuver_start, cmd.target_speed,
                                          path_cfg_);
    FilterLongitudinalByKinematicLimits(longitudinal_for_continuity, limits_);

    std::vector<FrenetPath> continuity_combined = CombineLateralLongitudinal(
        continuity_lateral, longitudinal_for_continuity);
    FilterByCurvature(continuity_combined, ref_, limits_);
    if (!planning_collision_obstacles.empty()) {
        FilterByCollision(continuity_combined, ref_,
                          planning_collision_obstacles, vehicle_shape_,
                          collision_cfg_, /*coast_exempt_id=*/-1);
    }
    candidates.insert(candidates.end(),
                      std::make_move_iterator(continuity_combined.begin()),
                      std::make_move_iterator(continuity_combined.end()));
}

bool FrenetPlanner::PlanExternalKeepLowSpeedFallback(const CartesianState& ego,
                                                      double s, double d,
                                                      double resolved_target_speed,
                                                      CartesianPath& out_path) {
    const bool fallback_ok = PlanLowSpeedFallback(
        ego, s, d, /*target_d=*/0.0, out_path, limits_.low_speed_crawl_duration_s,
        resolved_target_speed);
    std::printf("[FrenetPlanner-KEEP] near-low-speed fail-safe: no "
                "candidate (v=%.2f), emitting crawl fallback path\n",
                ego.v);
    last_failure_reason_ = PlanFailureReason::NONE;
    return fallback_ok;
}

bool FrenetPlanner::ExecuteManeuverPipeline(
    const CartesianState& ego, const FrenetState& start,
    double s, double d, double s_dot, double s_ddot,
    double d_dot, double d_ddot,
    const std::vector<TrackedObject>& obstacles,
    const std::vector<TrackedObject>& predicted_obstacles,
    double resolved_target_speed, const PlannerCommand& command,
    CartesianPath& out_path) {
    const PlannerCommand* external_command = &command;
    // 외부 command 경로는 FSM이 지정한 mode를 그대로 받아 실행한다. 진입
    // 가능한 모드는 ValidateExternalCommand의 whitelist가 이미 걸러낸 상태다.
    // FOLLOWING의 leader_s/leader_speed/leader_accel/time_gap/min_gap/
    // gap_gain은 FollowProcess가 이미 채운 뒤 이 함수를 불렀다 - 여기서
    // 다시 ResolveExternalFollowTarget을 부르면 v2_following_ 상태(grace
    // 창/missing_cycles)가 사이클당 두 번 갱신되므로 절대 다시 부르지 않는다.
    PlannerCommand cmd = *external_command;
    const double leader_s = cmd.leader_s;
    const double leader_speed = cmd.leader_speed;

    // 외부 AVOID의 대상 확보는 **저속 fallback보다 위**에서 이미 끝났다
    // (D6-2a). 여기서 하면 ego < 0.5 m/s일 때 도달하지 않는다.
    // AVOID/FOLLOW가 아닌 명령의 context reset도 위쪽에서 처리했다.

    cmd.target_speed = resolved_target_speed;

    // =====================================================================
    // 외부 STOP 실행 (APPROACH / BRAKING / HOLD)
    //
    // GenerateStoppingCandidates는 종료조건을 s(T)=stop_s, v(T)=0, a(T)=0으로
    // 강제한다. time_horizon 상한이 5초이므로 정지 목표가 멀면(예: 5m/s에서
    // 30m) 그 안에 도달하려는 후보가 전부 비현실적 가감속이 되어 가속도
    // 필터에서 탈락한다. horizon을 늘리는 대신, receding horizon 구조에 맞게
    // 먼 구간은 속도만 낮추며 접근하고(APPROACH) 제동거리 안에 들어왔을 때
    // 정지 다항식을 만든다(BRAKING).
    // =====================================================================
    const bool external_stop_mode = cmd.mode == STOP;
    double stop_distance = 0.0;
    if (external_stop_mode) {
        UpdateExternalStopExecution(s, s_dot, cmd, stop_distance);
        // 정지 직전에 이미 차선 중심 허용 범위 안이라면 d=0까지 억지로
        // 붙이지 않는다. 현재 d를 종단 목표로 고정하되 lateral quintic의
        // 종단 d_dot/d_ddot=0 조건은 유지되므로 차체는 경로 방향으로 곧게
        // 정렬된다. 허용 범위 밖이거나 속도가 높으면 기존 d=0 복귀 그대로다.
        if (std::fabs(s_dot) <= stop_cfg_.lateral_hold_speed &&
            std::fabs(d) <= stop_cfg_.lateral_error_tolerance) {
            cmd.has_stop_lateral_hold = true;
            cmd.stop_lateral_hold_d = d;
        }
    }

    last_mode_ = external_stop_mode ? STOP : cmd.mode;
    last_target_speed_ = cmd.target_speed;

    // HOLD는 후보 생성/충돌 필터 파이프라인을 거치지 않는다. 이미 정지한
    // 상태이므로 0속도 궤적을 그대로 유지한다.
    if (external_stop_mode && stop_.phase == StopExecutionPhase::HOLD) {
        PlanExternalStopHold(ego, s, out_path);
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    PlannerDebugStats stats;

    // STOP이 AVOID보다 앞선 목표로 선택됐으면, 차량은 그 정지 목표를 넘어
    // 진행하지 않는다. 하지만 APPROACH는 내부적으로 velocity-keeping 후보를
    // 사용하므로 5초 horizon이 정지 목표를 지나 뒤쪽 장애물까지 닿을 수
    // 있다. 그 장애물이 모든 후보를 제거하면 실제 정지선보다 훨씬 앞에서
    // SAFE_STOP에 갇힌다.
    //
    // 정지 자세에서도 앞범퍼가 닿을 수 있는 장애물은 반드시 유지한다.
    // 후륜축 기준 stop_position 뒤로 front extent+safety margin보다 더 먼
    // 객체만 제외하므로, 정지 목표 전/인접 장애물의 안전성은 그대로다.
    std::vector<TrackedObject> stop_scoped_collision_obstacles;
    const std::vector<TrackedObject>* planning_collision_obstacles_ptr =
        &predicted_obstacles;
    if (external_stop_mode && std::isfinite(stop_distance) &&
        stop_distance >= 0.0) {
        const double rear_overhang = std::max(
            0.0, std::min(avoid_cfg_.rear_overhang, vehicle_shape_.length));
        const double front_extent = vehicle_shape_.length - rear_overhang;
        const double protected_distance =
            stop_distance + front_extent + collision_cfg_.safety_margin;
        const double track_length =
            ref_.points.empty() ? 0.0 : ref_.points.back().s;
        stop_scoped_collision_obstacles.reserve(predicted_obstacles.size());
        for (const TrackedObject& object : predicted_obstacles) {
            double object_s = 0.0, object_d = 0.0, object_s_dot = 0.0;
            ProjectObjectToFrenet(ref_, object, object_s, object_d,
                                  object_s_dot,
                                  ref_config_.ego_projection_search_window);
            double object_gap = object_s - s;
            if (object_gap <= 0.0 && track_length > 0.0) {
                object_gap += track_length;
            }
            const double object_front_gap =
                object_gap - 0.5 * std::max(0.0, object.length);
            if (!std::isfinite(object_front_gap) ||
                object_front_gap <= protected_distance) {
                stop_scoped_collision_obstacles.push_back(object);
            }
        }
        planning_collision_obstacles_ptr =
            &stop_scoped_collision_obstacles;
    }
    const std::vector<TrackedObject>& planning_collision_obstacles =
        *planning_collision_obstacles_ptr;

    // braking-memory 완화(fix/yuntae-frenet-speed-continuity Phase A):
    // external KEEP/LANE_KEEPING에서만, 그리고 cmd.mode가
    // UpdateExternalStopExecution의 STOP-APPROACH 위임(Maneuver/Stop.cpp)이
    // 아니라 실제로 external_command->mode==LANE_KEEPING일 때만 적용한다 -
    // STOP/FOLLOW의 경계조건 semantics는 start.s_dd(raw/진단용, 이 함수
    // 앞부분의 Frenet boundary-state 구성 지점)를 그대로 유지해야 한다.
    FrenetState maneuver_start = start;
    const bool external_keep_candidates = external_command->mode == LANE_KEEPING;
    if (external_keep_candidates) {
        constexpr double kAccelIntentDeadband = 0.3;  // [m/s]
        maneuver_start.s_dd = ResolveVelocityKeepingBoundaryAcceleration(
            start.s_dd, s_dot, resolved_target_speed, kAccelIntentDeadband);
    }
    KinematicLimits maneuver_limits = limits_;
    if (external_stop_mode) {
        // 일반 주행의 가속도 한계는 그대로 두고 STOP 후보에만 서비스 제동
        // 한계를 허용한다. MPC의 accel_min보다 큰 감속 궤적을 보내지 않도록
        // params.yaml에서 두 값을 같은 크기로 유지한다.
        maneuver_limits.max_longitudinal_accel = std::max(
            maneuver_limits.max_longitudinal_accel,
            stop_cfg_.max_service_decel);
    }
    std::vector<FrenetPath> candidates = ResolveManeuver(
        maneuver_start, cmd, ref_, path_cfg_, maneuver_limits, lane_width_,
        planning_collision_obstacles, vehicle_shape_, collision_cfg_,
        stop_cfg_, merge_cfg_, &stats);

    // moderate-speed curvature starvation 완화(Phase B): 실측 MORAI에서
    // 급커브 근접 중 유효한 결합 후보가 있는데도 전부 중앙(d1=0) 복귀였던
    // 탓에 사후 곡률 필터에서 통째로 탈락하는 사례가 있었다(예:
    // combined=78, after_curvature=0). external KEEP에만, 현재 d가 차선
    // 폭 안일 때만, 현재 오프셋을 유지하는 continuity candidate를
    // 추가한다 - d=0 강제 클램프가 아니라 대안 후보 하나를 더 얹는
    // 것뿐이며, 아래 EvaluateCosts가 여전히 target_d=0을 기준으로 비용을
    // 매기므로 중앙 복귀 후보가 곡률을 통과하면 그쪽이 그대로 선택된다
    // (kd/kd_path 항이 더 낮은 비용을 준다). 중앙 복귀가 전부 곡률에서
    // 탈락했을 때만 이 candidate가 유일하게 남아 선택된다.
    if (external_keep_candidates &&
        std::fabs(maneuver_start.d) <= lane_width_ * 0.5) {
        AppendExternalKeepContinuityCandidates(
            maneuver_start, cmd, planning_collision_obstacles, candidates);
    }

    const double lateral_cost_target = cmd.has_stop_lateral_hold
        ? cmd.stop_lateral_hold_d : 0.0;
    EvaluateCosts(candidates, cost_weights_, lateral_cost_target);

    if (cmd.mode != prev_mode_) {
        std::printf("[FrenetPlanner] mode: %s -> %s (s=%.2f d=%.3f)\n",
                    ModeName(prev_mode_), ModeName(cmd.mode), s, d);
        if (cmd.mode == FOLLOWING) {
            const double gap = leader_s - s;
            const double desired_gap = following_cfg_.min_gap
                                     + following_cfg_.time_gap * leader_speed;
            std::printf("[FrenetPlanner]   leader_id=%d leader_s=%.2f leader_speed=%.2f "
                        "gap=%.2f desired_gap=%.2f\n",
                        cmd.target_object_id, leader_s, leader_speed, gap, desired_gap);
        }
        prev_mode_ = cmd.mode;
    }

    // 안전성 필터를 통과한 후보만 valid로 남아 있다. 외부 STOP에서는 그
    // 안전 집합 안에서 목표점에 가장 가까운 종점을 우선하고, 같은 종점의
    // 시간/jerk 조합은 기존 cost로 결정한다.
    const FrenetPath* best = external_stop_mode
        ? SelectBestStoppingPath(candidates)
        : SelectBestPath(candidates);
    const int selected_index = best
        ? static_cast<int>(best - candidates.data())
        : -1;

    last_candidate_paths_.reserve(candidates.size());
    last_candidate_validity_.reserve(candidates.size());
    for (const FrenetPath& candidate : candidates) {
        if (!candidate.s.empty() && candidate.s.size() == candidate.d.size()) {
            last_candidate_paths_.push_back(ConvertToCartesianPath(candidate, ref_));
            last_candidate_validity_.push_back(candidate.valid);
        }
    }

    if (debug_writer_) {
        debug_writer_->Publish(ref_, ego, start, cmd.mode, "NONE",
                               cmd.target_speed,
                               planning_collision_obstacles, candidates,
                               selected_index, stats, cmd);
    }

    if (!best) {
        // 후보가 곡률 단계까지는 살아 있었는데 충돌 검사에서 전부 제거된 것은
        // 일시적인 생성 실패가 아니라 명시적인 안전 차단이다. main.cpp가 이
        // 값을 보고 과거 경로 재전송을 금지한다.
        last_failure_reason_ =
            stats.combined_valid_after_curvature > 0 &&
            stats.combined_valid_after_collision == 0
                ? PlanFailureReason::COLLISION_BLOCKED
                : PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
        std::printf("[FrenetPlanner] No valid candidate this cycle "
                    "(mode=%s, %zu generated)\n",
                    ModeName(cmd.mode), candidates.size());
        std::printf("[FrenetPlanner-DEBUG] ego: x=%.3f y=%.3f yaw=%.3f v=%.3f a=%.3f\n",
                     ego.x, ego.y, ego.yaw, ego.v, ego.a);
        std::printf("[FrenetPlanner-DEBUG] start: s=%.2f d=%.3f s_dot=%.2f s_ddot=%.2f d_dot=%.3f d_ddot=%.3f "
                     "target_speed=%.2f\n",
                     s, d, s_dot, s_ddot, d_dot, d_ddot, cmd.target_speed);
        std::printf("[FrenetPlanner-DEBUG] lateral %zu/%zu valid | longitudinal %zu/%zu valid | "
                     "combined %zu -> after curvature %zu -> after collision %zu valid\n",
                     stats.lateral_valid, stats.lateral_total,
                     stats.longitudinal_valid, stats.longitudinal_total,
                     stats.combined_total, stats.combined_valid_after_curvature,
                     stats.combined_valid_after_collision);
        if (stats.combined_valid_after_curvature > 0 &&
            stats.combined_valid_after_collision == 0) {
            std::unordered_map<int, int> blocker_counts;
            std::unordered_map<int, double> blocker_first_times;
            for (const auto& candidate : candidates) {
                if (candidate.rejection_reason != RejectionReason::COLLISION ||
                    candidate.collision_object_id < 0) continue;
                const int blocker_id = candidate.collision_object_id;
                ++blocker_counts[blocker_id];
                double collision_t = -1.0;
                if (candidate.collision_sample_index >= 0 &&
                    static_cast<size_t>(candidate.collision_sample_index) < candidate.t.size()) {
                    collision_t = candidate.t[static_cast<size_t>(candidate.collision_sample_index)];
                }
                auto time_it = blocker_first_times.find(blocker_id);
                if (collision_t >= 0.0 &&
                    (time_it == blocker_first_times.end() || collision_t < time_it->second))
                    blocker_first_times[blocker_id] = collision_t;
            }
            std::vector<std::pair<int, int>> ranked_blockers(
                blocker_counts.begin(), blocker_counts.end());
            std::sort(ranked_blockers.begin(), ranked_blockers.end(),
                      [](const auto& lhs, const auto& rhs) {
                          return lhs.second > rhs.second;
                      });
            const size_t report_count = std::min<size_t>(3, ranked_blockers.size());
            for (size_t rank = 0; rank < report_count; ++rank) {
                const int blocker_id = ranked_blockers[rank].first;
                const int blocked_paths = ranked_blockers[rank].second;
                const auto obj_it = std::find_if(
                    predicted_obstacles.begin(), predicted_obstacles.end(),
                    [blocker_id](const TrackedObject& obj) { return obj.id == blocker_id; });
                if (obj_it == predicted_obstacles.end()) {
                    std::printf("[FrenetPlanner-COLLISION] rank=%zu blocking_id=%d "
                                "paths=%d object=missing leader_id=%d\n",
                                rank + 1, blocker_id, blocked_paths,
                                cmd.mode == FOLLOWING ? cmd.target_object_id : -1);
                    continue;
                }
                double obj_s, obj_d, obj_s_dot;
                ProjectObjectToFrenet(ref_, *obj_it, obj_s, obj_d, obj_s_dot,
                                      ref_config_.ego_projection_search_window);
                double obj_gap = obj_s - s;
                const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
                if (obj_gap <= 0.0 && track_length > 0.0) obj_gap += track_length;
                const auto time_it = blocker_first_times.find(blocker_id);
                const double first_t = time_it == blocker_first_times.end()
                    ? -1.0 : time_it->second;
                std::printf("[FrenetPlanner-COLLISION] rank=%zu blocking_id=%d paths=%d "
                            "first_t=%.2f obj_s=%.2f obj_d=%.2f gap=%.2f speed_s=%.2f "
                            "same_lane=%d leader_id=%d leader_match=%d\n",
                            rank + 1, blocker_id, blocked_paths, first_t,
                            obj_s, obj_d, obj_gap, obj_s_dot,
                            std::fabs(obj_d) <= lane_width_ * 0.5 ? 1 : 0,
                            cmd.mode == FOLLOWING ? cmd.target_object_id : -1,
                            cmd.mode == FOLLOWING &&
                                cmd.target_object_id == blocker_id ? 1 : 0);
            }
        }
        // STOP BRAKING에서 후보가 0개인 것은 흔한 상황이다. 정지 목표가
        // 아주 가까우면(예: 5m/s에서 3m 앞) exact-stop quintic이 가속도
        // 한계를 넘어 전부 탈락한다. 여기서 false를 반환하면 노드가 이전
        // 궤적을 재사용해 정지 목표를 지나쳐 계속 주행하게 된다 - STOP에서
        // 가장 위험한 실패 방향이다. 충돌 차단이 아닌 경우에 한해 그 자리에서
        // 감속해 정지하는 fail-safe 궤적을 직접 만들어 반환한다.
        if (external_stop_mode &&
            stop_.phase == StopExecutionPhase::BRAKING &&
            last_failure_reason_ != PlanFailureReason::COLLISION_BLOCKED) {
            PlanExternalStopBrakingFallback(s, d, s_dot, stop_distance, out_path);
            return true;
        }

        // 외부 KEEP의 low-speed boundary candidate starvation
        // (fix/yuntae-frenet-speed-continuity Phase 5). kLowSpeedFallbackV
        // (0.5 m/s) 문턱을 살짝 넘은 직후, 강한 감속 중이던 s_ddot 경계조건이
        // 그대로 남아 있으면(예: ego.v=0.6, s_ddot=-2.4) 정상 velocity
        // keeping 후보가 전부 accel/jerk 한계를 넘어 탈락할 수 있다(실측
        // 재현: longitudinal 0/40 valid). 이 경계에서만, 그리고 실제로 후보가
        // 전부 탈락했을 때만(=이 분기), 이미 검증된 저속 fallback 경로로
        // 안전하게 이어간다. 정상 후보가 하나라도 있으면 이 분기 자체에
        // 도달하지 않으므로 정상 pipeline을 절대 우회하지 않는다.
        //
        // STOP/FOLLOW/AVOID는 각자 별도 정책(위 STOP fail-safe,
        // GenerateFollowingCandidates의 정지-리더 목표, AVOID의 전용 저속
        // 처리)이 있으므로 여기서는 건드리지 않는다 - 순수 외부 KEEP만
        // 대상이다.
        const bool external_keep = cmd.mode == LANE_KEEPING;
        if (external_keep &&
            std::fabs(ego.v) < limits_.near_low_speed_fallback_v &&
            last_failure_reason_ != PlanFailureReason::COLLISION_BLOCKED) {
            return PlanExternalKeepLowSpeedFallback(ego, s, d,
                                                     resolved_target_speed, out_path);
        }
        return false;
    }

    CartesianPath selected_cartesian = ConvertToCartesianPath(*best, ref_);

    // STOP 종단부에서는 s_dot이 0으로 수렴한다. 일반 변환식은 d_dot/s_dot,
    // d_ddot/s_dot^2를 사용하므로 0.1~0.6 m/s의 아주 낮은 속도에서도 작은
    // 위치/미분 오차가 kappa를 수 배~수십 배로 증폭할 수 있다. 후보 안전성
    // 필터는 위치로 계산한 ComputeGeometricPath의 곡률을 이미 사용하므로,
    // STOP 출력도 동일한 기하 곡률/heading을 사용해야 필터와 MPC 사이의
    // 계약이 일치한다. x/y와 종방향 v/a는 기존 정지 다항식을 그대로 둔다.
    if (external_stop_mode) {
        const GeometricPath stop_geometry =
            ComputeGeometricPath(best->s, best->d, ref_);
        if (stop_geometry.yaw.size() == selected_cartesian.yaw.size() &&
            stop_geometry.kappa.size() == selected_cartesian.kappa.size()) {
            selected_cartesian.yaw = stop_geometry.yaw;
            selected_cartesian.kappa = stop_geometry.kappa;
        }
    }

    // STOP은 절대 후진하지 않는다. 정지 목표를 이미 지나쳤거나 아주 가까울 때
    // stopping quintic은 목표 s에 "도달"하려고 0을 지나 음수 속도로 내려가
    // 후진 궤적을 만든다(실측: v가 0 이후 -0.15m/s까지 내려감). 속도가 0에
    // 닿는 시점에서 궤적을 잘라 그 자리에 정지시킨다.
    if (external_stop_mode) {
        TrimExternalStopReverseTrajectory(selected_cartesian);
    }

    // 모든 모드의 현재 상태를 20Hz 기준 약 4Hz로 출력한다. 모드 전환 로그만으로는
    // 후보가 사라지는 바로 그 순간이 어느 상태였는지 알 수 없으므로, s/d/속도와
    // 필터 후 후보 수를 같은 줄에 남긴다.
    static int state_log_tick = 0;
    if (++state_log_tick % 5 == 0) {
        if (cmd.mode == FOLLOWING) {
            const double gap = leader_s - s;
            const double desired_gap = following_cfg_.min_gap
                                     + following_cfg_.time_gap * leader_speed;
            const double approach_speed = std::max(0.0, std::min(
                cmd.target_speed,
                leader_speed + following_cfg_.gap_gain * (gap - desired_gap)));
            std::printf("[FrenetPlanner-STATE] mode=FOLLOWING leader_id=%d s=%.2f d=%.3f "
                        "ego_speed=%.2f leader_speed=%.2f relative_speed=%.2f "
                        "gap=%.2f desired_gap=%.2f gap_error=%.2f approach_speed=%.2f chosen_d=%.3f "
                        "cost=%.2f candidates=%zu/%zu missing=%d\n",
                        cmd.target_object_id, s, d, ego.v, leader_speed,
                        ego.v - leader_speed, gap, desired_gap, gap - desired_gap,
                        approach_speed, best->d.empty() ? d : best->d.back(), best->cost_total,
                        stats.combined_valid_after_collision, stats.combined_total,
                        v2_following_.missing_cycles);
        } else {
            std::printf("[FrenetPlanner-STATE] mode=%s s=%.2f d=%.3f speed=%.2f target_speed=%.2f "
                        "chosen_d=%.3f cost=%.2f candidates=%zu/%zu\n",
                        ModeName(cmd.mode),
                        s, d, ego.v, cmd.target_speed,
                        best->d.empty() ? d : best->d.back(), best->cost_total,
                        stats.combined_valid_after_collision, stats.combined_total);
        }
    }

    out_path = std::move(selected_cartesian);
    return true;
}

bool FrenetPlanner::KeepProcess(
    const CartesianState& ego, const FrenetState& start,
    double s, double d, double s_dot, double s_ddot,
    double d_dot, double d_ddot,
    const std::vector<TrackedObject>& obstacles,
    const std::vector<TrackedObject>& predicted_obstacles,
    double resolved_target_speed, const PlannerCommand& external_command,
    CartesianPath& out_path) {
    // KEEP은 모드별 준비 작업이 없다 - 공용 pipeline을 그대로 위임한다.
    return ExecuteManeuverPipeline(
        ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
        predicted_obstacles, resolved_target_speed, external_command,
        out_path);
}
