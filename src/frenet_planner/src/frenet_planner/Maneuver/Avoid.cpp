/**
 * @file Avoid.cpp
 * @brief External/v2 AVOID target resolution, commit state, lifecycle,
 *        target-preparation, maintenance path, PREPARE/COMMITTED
 *        execution, and legacy AVOID context/acquisition/adaptive
 *        candidate planning.
 *
 * PrepareAvoid/PlanCommittedAvoid stay in frenet_planner.cpp - they
 * depend on the anonymous-namespace ValidateExternalCommand, whose
 * linkage this refactor does not expose.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>

#include "frenet_planner/Evaluation/Cost.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Maneuver/ManeuverUtils.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"
#include "frenet_planner/Math/Utils.hpp"

namespace {

// 후보 경로의 최대 기하 곡률에서 a_lat=v^2*|kappa| 한계를
// 만족하는 속도를 계산한다. 제어/모델 오차 여유로 한계의 80%를 쓴다.
double CandidateCurvatureSpeedLimit(const FrenetPath& path, const RefLine& ref,
                                    double max_lateral_accel) {
    constexpr double kLateralAccelMargin = 0.8;
    constexpr double kMinCurvature = 1e-4;
    const GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
    double max_kappa = 0.0;
    for (double kappa : geo.kappa) max_kappa = std::max(max_kappa, std::fabs(kappa));
    if (max_kappa < kMinCurvature) return std::numeric_limits<double>::infinity();
    return std::sqrt(max_lateral_accel * kLateralAccelMargin / max_kappa);
}

}  // namespace

double FrenetPlanner::EffectiveAvoidDetectionDistance() const {
    // legacy AVOID 등록거리와 같은 식이다(UpdateLegacyAvoidContext의 acquisition_cfg).
    // config 값을 그대로 쓰면 충돌 lookahead보다 짧아져, 장애물을 회피 대상으로
    // 등록하기 전에 충돌 필터가 중앙 후보를 전부 제거한다 -> 속도 급락 후
    // 등록되면 다시 가속하는 진동이 생긴다.
    //
    // 현재 config 기준:
    //   config 값     60.0 m
    //   충돌 horizon  10.0 * 8.0 + 4.5 + 0.3 = 84.8 m
    const double collision_horizon =
        curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead +
        vehicle_shape_.length + collision_cfg_.safety_margin;
    return std::max(avoid_cfg_.detection_distance, collision_horizon);
}

FrenetPlanner::AvoidTargetStatus FrenetPlanner::ResolveExternalAvoidTarget(
    double ego_s, int target_object_id,
    const std::vector<TrackedObject>& obstacles,
    AvoidanceContext& context) const {
    if (target_object_id < 0) return AvoidTargetStatus::LOST;

    // 지정된 ID만 찾는다. FindStaticAvoidanceTarget()처럼 "가장 가까운 정적
    // 장애물"을 고르지 않는다 - 대상 선택 권한은 FSM에 있다(헤더 주석 참고).
    const auto it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [target_object_id](const TrackedObject& object) {
            return object.id == target_object_id;
        });

    if (it != obstacles.end()) {
        double object_s = 0.0, object_d = 0.0, object_s_dot = 0.0;
        ProjectObjectToFrenet(ref_, *it, object_s, object_d, object_s_dot);

        // 폐루프 트랙에서 gap이 음수로 크게 튀지 않도록 감아준다(FOLLOW와 동일).
        const double track_length =
            ref_.points.empty() ? 0.0 : ref_.points.back().s;
        double gap = object_s - ego_s;
        if (gap <= 0.0 && track_length > 0.0) gap += track_length;

        const double search_limit = EffectiveAvoidDetectionDistance();

        // 속도 조건(trigger_max_speed)을 적용하지 않는다 - 헤더 주석 참고.
        // 정지/저속/주행 중 어느 쪽이든 FSM이 AVOID를 지정했으면 대상이다.
        const bool in_front = gap > 0.0;
        const bool in_range = gap <= search_limit;
        // 회피는 같은 corridor의 장애물을 대상으로 한다. 옆 차선 객체를
        // 회피하려고 횡이동하면 오히려 그쪽으로 들어간다.
        const bool same_corridor = std::fabs(object_d) <= lane_width_ * 0.5;
        // geometry가 0이면 회피 오프셋을 계산할 수 없다. 후보 생성 단계에서
        // 0 폭 장애물을 비켜가는 target_d는 의미가 없다.
        const bool geometry_ok = it->width > 0.0 && it->length > 0.0;

        if (in_front && in_range && same_corridor && geometry_ok) {
            context.active = true;
            // 요청 ID가 아니라 **실제로 찾은 객체의 ID**를 기록한다. 둘은
            // 항상 같아야 하지만, 같다고 가정해 요청 ID를 그대로 넣으면
            // 조회가 잘못되어도(예: nearest selection으로 회귀) 확보 ID는
            // 정상으로 보인다 - 진단과 회귀 검증이 그 오류를 놓친다.
            context.obstacle_id = it->id;
            context.obstacle_s = ego_s + gap;
            context.obstacle_d = object_d;
            context.obstacle_width = it->width;
            context.obstacle_length = it->length;
            context.missing_cycles = 0;
            return AvoidTargetStatus::VALID;
        }

        // 관측되었지만 쓸 수 없는 상태다. dropout이 아니므로 유예를 주지
        // 않는다 - 예측으로 메꿀 대상이 아니라 조건 자체가 틀렸다
        // (계약 8.2절). phase/target_d/start_d는 호출자가 관리하므로
        // 여기서는 확보 상태만 해제한다.
        context.active = false;
        context.obstacle_id = -1;
        context.missing_cycles = 0;
        return AvoidTargetStatus::LOST;
    }

    // 관측되지 않았다. 직전에 확보한 상태가 있으면 유예한다.
    //
    // 한 번도 확보하지 못한 대상에는 유예를 주지 않는다. FSM이 존재하지 않는
    // ID를 보낸 경우를 조용히 넘기게 된다(FOLLOW와 동일한 판단).
    if (!context.active || context.obstacle_id != target_object_id) {
        context.active = false;
        context.obstacle_id = -1;
        context.missing_cycles = 0;
        return AvoidTargetStatus::LOST;
    }

    if (++context.missing_cycles > avoid_cfg_.dropout_grace_cycles) {
        context.active = false;
        context.obstacle_id = -1;
        context.missing_cycles = 0;
        return AvoidTargetStatus::LOST;
    }

    // 유예 중에는 마지막으로 관측한 위치를 그대로 유지한다.
    //
    // FOLLOW는 leader_speed로 leader_s를 전진시키지만, AVOID 대상은 정적
    // 장애물이 주 대상이고 관측된 속도를 보관하지도 않는다. 없는 속도를
    // 추정해 위치를 밀면 실제보다 멀리 있다고 판단해 회피 시작이 늦어진다.
    return AvoidTargetStatus::GRACE;
}

FrenetPlanner::AvoidTargetPlan FrenetPlanner::DecideAvoidTargets(
    const AvoidanceContext& context, LateralDirection direction) const {
    AvoidTargetPlan plan;
    if (!context.active) return plan;

    // ---- target pose가 있으면 그 d가 authoritative ----------------------
    if (context.has_requested_target_d) {
        plan.candidates[0] = context.requested_target_d;
        plan.count = 1;
        plan.from_target_pose = true;
        // 방향 요청과의 정합성만 표시한다. 여기서 거부하지 않는 이유는
        // 헤더 AvoidTargetPlan 주석에 있다 - clearance 부족은 충돌 검사가
        // 판정할 일이고, 그 결과는 WAITING이다.
        if (direction == LateralDirection::LEFT) {
            plan.direction_conflict =
                context.requested_target_d <= context.obstacle_d;
        } else if (direction == LateralDirection::RIGHT) {
            plan.direction_conflict =
                context.requested_target_d >= context.obstacle_d;
        }
        return plan;
    }

    // ---- pose가 없으면 legacy geometry 계산을 재사용 --------------------
    //
    // legacy begin_avoidance 분기와 **같은 식**이다. 고정 ±avoid_offset(5.0)이
    // 아니라 장애물의 실제 횡위치 + 실제 폭을 반영한다.
    constexpr double kDirectionExtraMargin = 0.3;
    const double clearance =
        0.5 * (vehicle_shape_.width + context.obstacle_width) +
        collision_cfg_.safety_margin + kDirectionExtraMargin;
    const double left_target = context.obstacle_d + clearance;
    const double right_target = context.obstacle_d - clearance;

    switch (direction) {
        case LateralDirection::LEFT:
            plan.candidates[0] = left_target;
            plan.count = 1;
            break;
        case LateralDirection::RIGHT:
            plan.candidates[0] = right_target;
            plan.count = 1;
            break;
        case LateralDirection::NONE:
            // 방향 미지정이면 Frenet이 고른다 - 그것이 DIRECTION_NONE의
            // 의미다(BehaviorCommand.msg). 순서는 legacy와 같이 left 먼저.
            plan.candidates[0] = left_target;
            plan.candidates[1] = right_target;
            plan.count = 2;
            break;
    }
    return plan;
}

bool FrenetPlanner::CanCommitAvoid(const AvoidPrepareResult& result) const {
    // 확보하지 않은 목표를 잠그지 않는다. 호출자가 candidate_valid를 먼저
    // 확인해야 하지만, 여기서도 막는다.
    if (!result.candidate_valid) return false;
    if (!v2_avoidance_.active) return false;
    if (v2_avoidance_.committed) return false;
    if (!std::isfinite(result.selected_target_d)) return false;
    return true;
}

bool FrenetPlanner::CommitAvoid(const AvoidPrepareResult& result) {
    if (!CanCommitAvoid(result)) return false;

    ApplyAvoidCommit(result);
    return true;
}

void FrenetPlanner::ApplyAvoidCommit(const AvoidPrepareResult& result) {
    v2_avoidance_.committed = true;
    v2_avoidance_.completed = false;
    v2_avoidance_.maneuver_progress = 0.0;
    // 이 시점에 처음 lock한다. PREPARE에서는 사이클마다 좌/우가 바뀔 수
    // 있으므로 잠그지 않았다(계약 12.7.3절).
    v2_avoidance_.target_d = result.selected_target_d;
    // 진행률 기준점. maneuver_progress를 (start_d -> target_d) 구간으로
    // 계산한다(D6-3c).
    v2_avoidance_.start_d = last_d_;
    v2_avoidance_.phase = AvoidPhase::SHIFT;
}

void FrenetPlanner::UpdateCommittedAvoidLifecycle(
    double ego_s, double ego_d, AvoidPrepareResult& result) {
    if (!v2_avoidance_.active || !v2_avoidance_.committed) return;

    constexpr double kProgressEpsilon = 1e-6;
    bool entered_pass_this_cycle = false;
    if (!v2_avoidance_.completed &&
        v2_avoidance_.phase == AvoidPhase::SHIFT &&
        std::isfinite(ego_d) && std::isfinite(v2_avoidance_.target_d) &&
        std::fabs(ego_d - v2_avoidance_.target_d) <=
            avoid_cfg_.lateral_tolerance) {
        v2_avoidance_.phase = AvoidPhase::PASS;
        entered_pass_this_cycle = true;
    }

    const double clear_distance =
        0.5 * (vehicle_shape_.length + v2_avoidance_.obstacle_length) +
        avoid_cfg_.pass_clearance;
    const double track_length =
        ref_.points.empty() ? 0.0 : ref_.points.back().s;
    // Resolver는 seam 너머 전방 target을 ego_s + forward gap의 unwrapped
    // obstacle_s로 저장한다. raw subtraction은 ego가 seam을 지난 순간 거의
    // 한 바퀴 양수가 되므로, 기존 signed arc helper로 앞/뒤 의미를 보존한다.
    const double gap = ForwardArcDistance(
        ego_s, v2_avoidance_.obstacle_s, track_length);

    // SHIFT -> PASS가 관측된 다음 cycle부터 완료를 허용한다. ego가 이미 멀리
    // 지나간 pathological snapshot에서도 lifecycle이 SHIFT에서 곧바로
    // COMPLETED로 건너뛰지 않는다.
    if (!v2_avoidance_.completed && !entered_pass_this_cycle &&
        v2_avoidance_.phase == AvoidPhase::PASS &&
        std::isfinite(gap) && std::isfinite(clear_distance) &&
        clear_distance > kProgressEpsilon && gap < -clear_distance) {
        v2_avoidance_.completed = true;
    }

    double progress = 0.0;
    if (v2_avoidance_.completed) {
        progress = 1.0;
    } else if (v2_avoidance_.phase == AvoidPhase::SHIFT) {
        const double denominator = std::max(
            std::fabs(v2_avoidance_.target_d - v2_avoidance_.start_d),
            kProgressEpsilon);
        const double lateral_ratio =
            std::isfinite(ego_d) && std::isfinite(v2_avoidance_.start_d)
                ? clip(std::fabs(ego_d - v2_avoidance_.start_d) /
                           denominator,
                       0.0, 1.0)
                : 0.0;
        progress = 0.5 * lateral_ratio;
    } else {
        const double pass_ratio =
            std::isfinite(gap) && std::isfinite(clear_distance) &&
                    clear_distance > kProgressEpsilon
                ? clip(-gap / clear_distance, 0.0, 1.0)
                : 0.0;
        progress = 0.5 + 0.5 * pass_ratio;
    }

    if (!std::isfinite(progress)) progress = 0.0;
    progress = clip(progress, 0.0, 1.0);
    // localization jitter로 current d/s가 잠시 역행해도 FSM progress는
    // 되돌아가지 않는다.
    v2_avoidance_.maneuver_progress = std::max(
        v2_avoidance_.maneuver_progress, progress);
    result.maneuver_completed = v2_avoidance_.completed;
    result.maneuver_progress = v2_avoidance_.maneuver_progress;
}

const char* FrenetPlanner::avoid_phase_name() const {
    if (!v2_avoidance_.active) return "NONE";
    switch (v2_avoidance_.phase) {
        case AvoidPhase::TRACKING: return "TRACKING";
        case AvoidPhase::SHIFT: return "SHIFT";
        case AvoidPhase::PASS: return "PASS";
    }
    return "UNKNOWN";
}

void FrenetPlanner::ResetExternalAvoidance() {
    v2_avoidance_ = AvoidanceContext{};
    v2_avoid_targets_ = AvoidTargetPlan{};
}

void FrenetPlanner::BuildZeroSpeedHoldPath(const CartesianState& ego, double s,
                                           CartesianPath& out_path) const {
    // STOP HOLD와 같은 형태다. 한 점 0속도 궤적.
    const RefPoint rp = Interpolate(ref_, s);
    out_path.x.assign(1, ego.x);
    out_path.y.assign(1, ego.y);
    out_path.yaw.assign(1, std::isfinite(ego.yaw) ? ego.yaw : rp.theta);
    out_path.kappa.assign(1, 0.0);
    out_path.v.assign(1, 0.0);
    out_path.a.assign(1, 0.0);
}

bool FrenetPlanner::PrepareExternalAvoidTargets(
    double ego_s, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command, bool avoid_prepare, bool avoid_committed) {
    if (avoid_committed) {
        // COMMIT 이후 geometry의 소유권은 잠긴 context에 있다. request
        // direction/pose를 다시 읽거나 target identity를 재검증하지 않는다.
        v2_avoid_targets_ = AvoidTargetPlan{};
        v2_avoid_targets_.candidates[0] = v2_avoidance_.target_d;
        v2_avoid_targets_.count = 1;
        v2_avoid_targets_.from_target_pose =
            v2_avoidance_.has_requested_target_d;
        v2_avoid_targets_.direction_conflict = false;
    } else {
    // 요청 방향을 context에 보존한다. mode=AVOID 하나로는
    // LEFT/RIGHT/NONE을 구분할 수 없다(계약 3.3.1절).
    v2_avoidance_.requested_direction =
        command.requested_direction;

    // target pose 투영 결과도 보존한다 (D6-1a). node가 이미 Frenet
    // (s, d)로 바꿔서 [A] 필드에 넣어 보냈다.
    //
    // avoidance_d_offset([B])에 옮기지 않는다. 그러면 "FSM이 지정한
    // 목표"와 "planner가 후보 비교로 정한 목표"를 구분할 수 없게 된다
    // (계약 3.3.3절).
    v2_avoidance_.has_requested_target_d =
        command.has_avoid_target_d;
    v2_avoidance_.requested_target_d =
        command.has_avoid_target_d
            ? command.avoid_target_d : 0.0;
    v2_avoidance_.requested_target_s =
        command.has_avoid_target_d
            ? command.avoid_target_s : 0.0;

    // FindStaticAvoidanceTarget()을 타지 않는다 - 그 함수는 가장 가까운
    // 정적 장애물과 회피 방향까지 자동 결정하는 legacy policy다
    // (계약 D1 0.1절 B 분류, 6.4절).
    const AvoidTargetStatus status = ResolveExternalAvoidTarget(
        ego_s, command.target_object_id, obstacles, v2_avoidance_);
    if (status == AvoidTargetStatus::LOST) {
        // 대상을 쓸 수 없다. KEEP으로 자동 전환하지 않는다 - 다음 행동은
        // FSM이 새 command_id로 결정한다(계약 8.2절).
        //
        // D6-3에서 COMMIT 경계가 들어오면 COMMITTED/EXECUTING 중에는 이
        // 분기로 오지 않는다(계약 8.2.1절).
        // 목표 목록도 비운다 - 확보하지 못한 대상의 목표는 없다.
        v2_avoid_targets_ = AvoidTargetPlan{};
        last_mode_ = AVOID;
        last_failure_reason_ = PlanFailureReason::TARGET_OBJECT_LOST;
        return false;
    }

    // 확보 성공. 평가할 목표 d를 정한다.
    //
    // ===== COMMIT 이후에는 잠긴 목표만 쓴다 (D6-3a) =====
    //
    // DecideAvoidTargets는 매 사이클 요청(target pose 또는 geometry)에서
    // 목표를 다시 계산한다. COMMIT 이후에도 그것을 쓰면 잠근 값이 생성에
    // 반영되지 않는다. 실측으로 확인한 문제:
    //
    //   PREPARE  요청 2.8 -> 격자 탐색으로 선택 +3.30
    //   COMMIT   locked = +3.30
    //   post     생성 목표가 다시 +2.80 (요청값)   <- lock이 무의미
    //
    // COMMIT의 의미는 "이 궤적을 실행한다"이므로, 잠근 목표를 그대로
    // 겨냥해야 한다. 아래 생성부의 tolerance 0과 함께 동작한다.
    if (v2_avoidance_.committed) {
        v2_avoid_targets_ = AvoidTargetPlan{};
        v2_avoid_targets_.candidates[0] = v2_avoidance_.target_d;
        v2_avoid_targets_.count = 1;
        // 진단 값은 보존한다. 이 목표가 pose에서 왔는지는 여전히 사실이다.
        v2_avoid_targets_.from_target_pose =
            v2_avoidance_.has_requested_target_d;
        // COMMIT된 목표에는 방향 모순이 없다 - COMMIT 전에 이미 걸렀다.
        v2_avoid_targets_.direction_conflict = false;
    } else {
        v2_avoid_targets_ = DecideAvoidTargets(
            v2_avoidance_, command.requested_direction);
    }
    }

    // 후보 생성과 안전성 판정은 아래 AVOID PREPARE 블록에서 한다
    // (D6-2b). 그 블록은 FrenetState start가 만들어진 뒤에 있다.
    //
    // NORMAL purpose로 AVOID가 들어오면 아직 실행 경로가 없다. 그대로
    // 내려보내면 ResolveManeuver의 AVOID 분기가 avoidance_d_offset=0으로
    // 중앙 유지 궤적을 만들어, FSM 입장에서는 "회피 성공"으로 보이면서
    // 실제로는 장애물 쪽으로 직진한다(계약 11.4절과 같은 원칙).
    // 실제 실행은 D6-3에서 붙인다.
    if (!avoid_prepare && !avoid_committed) {
        last_mode_ = AVOID;
        last_failure_reason_ = PlanFailureReason::UNSUPPORTED_BEHAVIOR;
        return false;
    }
    return true;
}

bool FrenetPlanner::BuildAvoidMaintenancePath(
    const CartesianState& ego, const FrenetState& start,
    const PlannerCommand& base_command,
    const std::vector<TrackedObject>& collision_obstacles,
    CartesianPath& out_path) const {
    const double s = start.s;
    const double d = start.d;

    // ---- 1순위: 충돌 검사를 통과한 유지 궤적 --------------------------
    //
    // 저속에서는 정상 후보 파이프라인이 lateral candidate를 제한하므로
    // crawl을 쓴다. 정상 속도에서는 중앙 유지(KEEP) 후보를 만든다.
    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s] (PlanInternal과 동일)
    const bool low_speed = std::fabs(ego.v) < kLowSpeedFallbackV;

    if (low_speed) {
        // crawl은 현재 d를 유지한다. PREPARE 중에 횡이동을 시작하면 안 된다.
        CartesianPath crawl;
        if (PlanLowSpeedFallback(ego, s, d, d, crawl, 2.0,
                                 base_command.target_speed) &&
            crawl.x.size() >= 2) {
            // PlanLowSpeedFallback은 충돌 필터를 타지 않는다. 그대로 쓰면
            // 장애물 쪽으로 기어간다.
            int blocking_id = -1;
            if (CartesianTrajectoryClear(crawl, collision_obstacles,
                                         vehicle_shape_, collision_cfg_,
                                         path_cfg_.dt, &blocking_id)) {
                out_path = std::move(crawl);
                return true;
            }
        }
    } else {
        PlannerCommand keep_cmd = base_command;
        keep_cmd.mode = LANE_KEEPING;
        keep_cmd.avoidance_d_offset = 0.0;
        PlannerDebugStats keep_stats;
        auto keep_candidates =
            ResolveManeuver(start, keep_cmd, ref_, path_cfg_, limits_,
                            lane_width_, collision_obstacles, vehicle_shape_,
                            collision_cfg_, &keep_stats);
        EvaluateCosts(keep_candidates, cost_weights_, 0.0);
        FilterByCartesianLateralAcceleration(keep_candidates, ref_,
                                             limits_.max_lateral_accel);
        const FrenetPath* keep_best = SelectBestPath(keep_candidates);
        if (keep_best) {
            out_path = ConvertToCartesianPath(*keep_best, ref_);
            if (out_path.x.size() >= 2) return true;
        }
    }

    // ---- 2순위: 종방향 감속 -------------------------------------------
    //
    // 여기 오는 것은 정상이다 - AVOID 대상이 같은 corridor에 있으므로 직진
    // 후보가 그 장애물 때문에 전부 막힐 수 있다.
    CartesianPath decel;
    if (GenerateDecelToZeroPath(s, d, start.s_d, decel) &&
        decel.x.size() >= 2) {
        int blocking_id = -1;
        if (CartesianTrajectoryClear(decel, collision_obstacles, vehicle_shape_,
                                     collision_cfg_, path_cfg_.dt,
                                     &blocking_id)) {
            out_path = std::move(decel);
            return true;
        }
    }

    // ---- 3순위: 0속도 hold --------------------------------------------
    //
    // 감속 궤적조차 충돌로 막히면 그 자리에 멈추는 것이 마지막 수단이다.
    // 이 경로는 이동하지 않으므로 새 충돌을 만들지 않는다.
    BuildZeroSpeedHoldPath(ego, s, out_path);
    return !out_path.x.empty();
}

bool FrenetPlanner::PlanExternalAvoidCycle(
    const CartesianState& ego, const FrenetState& start,
    double s, double d, double resolved_target_speed,
    const std::vector<TrackedObject>& predicted_collision_obstacles,
    const PlannerCommand& command, bool avoid_prepare, bool avoid_committed,
    CartesianPath& out_path, AvoidPrepareResult& result) {
    last_mode_ = AVOID;
    last_target_speed_ = resolved_target_speed;
    result.direction_conflict = v2_avoid_targets_.direction_conflict;

    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s]
    // 상황 정보다. 상태 판단(READY/WAITING)에는 쓰지 않는다 - 그것은
    // candidate_valid만으로 정한다(헤더 주석 참고).
    result.low_speed_context = std::fabs(ego.v) < kLowSpeedFallbackV;

    // ---- target pose sanity: ego 전방이어야 한다 -------------------
    //
    // 뒤에 있는 목표 pose의 d를 그대로 lateral goal로 쓰면, 이미 지나간
    // 지점의 횡위치로 회피하게 된다. 다만 "장애물보다 반드시 앞/뒤"
    // 같은 강한 규칙은 넣지 않는다 - AVOID 목표점은 shift 완료점일 수도,
    // 통과 후 점일 수도 있어 FSM 계약이 더 명확해지기 전에는 과하다
    // (계약 3.3.4절).
    if (avoid_prepare && v2_avoidance_.has_requested_target_d) {
        const double track_length =
            ref_.points.empty() ? 0.0 : ref_.points.back().s;
        double target_gap = v2_avoidance_.requested_target_s - s;
        if (target_gap <= 0.0 && track_length > 0.0) {
            target_gap += track_length;
        }
        // 폐루프에서 감아 올린 gap이 트랙 절반을 넘으면 실제로는 후방이다.
        const bool behind =
            !(target_gap > 0.0) ||
            (track_length > 0.0 && target_gap > track_length * 0.5);
        result.target_behind_ego = behind;
    }

    // ---- AVOID 후보 평가 -------------------------------------------
    //
    // 방향 모순이나 후방 목표이면 후보를 만들지 않는다.
    //
    // ===== 방향 모순을 충돌 검사에만 맡기면 안 되는 이유 =====
    //
    // FSM이 LEFT를 지시했고 pose가 장애물 오른쪽 -4 m라면, 그 오른쪽
    // 경로는 물리적으로 아주 안전할 수 있다. 충돌 검사는 CLEAR를
    // 돌려주고, 그대로 READY까지 올라가면
    //   FSM은 LEFT를 명령 / Frenet은 RIGHT 후보를 READY
    // 라는 계약 위반이 된다. 그래서 feasibility를 여기서 false로 만든다.
    //
    // 단 terminal 실패가 아니다. "현재 요청으로 실행 가능한 후보가
    // 없다"이므로 결과는 WAITING이다(계약 3.3.4절).
    const bool blocked_by_request = avoid_prepare &&
        (result.direction_conflict || result.target_behind_ego);

    // 정상 후보 생성기는 정지 근처에서 횡 후보를 만들지 못할 수 있다.
    // COMMIT 이후에는 중앙 fallback 대신 잠긴 target_d를 향하는 crawl을
    // 만들고, fresh Cartesian collision checker를 반드시 통과시킨다.
    if (avoid_committed && result.low_speed_context) {
        result.generated_candidate_count = 1;
        CartesianPath crawl;
        PlanLowSpeedFallback(ego, s, d, v2_avoidance_.target_d, crawl,
                             2.0, resolved_target_speed);
        int blocking_id = -1;
        if (crawl.x.size() < 2 ||
            !CartesianTrajectoryClear(crawl,
                predicted_collision_obstacles, vehicle_shape_,
                collision_cfg_, path_cfg_.dt, &blocking_id)) {
            result.blocking_object_id = blocking_id;
            last_mode_ = AVOID;
            last_failure_reason_ = blocking_id >= 0
                ? PlanFailureReason::COLLISION_BLOCKED
                : PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            return false;
        }
        result.candidate_valid = true;
        result.valid_candidate_count = 1;
        result.selected_target_d = v2_avoidance_.target_d;
        result.selected_cost = 0.0;
        result.candidate_path = crawl;
        out_path = std::move(crawl);
        UpdateCommittedAvoidLifecycle(s, d, result);
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    // 저속이라고 후보 생성을 건너뛰지 않는다. "저속에서는 어차피 안 될
    // 것"이라고 가정하면, 실제로 가능한 상황에서도 READY에 도달하지
    // 못한다. 생성을 시도하고 결과로 판단한다 - 저속에서 후보가 0개면
    // low_speed_context이 그 사유를 설명한다.
    if (!blocked_by_request && v2_avoid_targets_.count > 0) {
        const FrenetPath* best_candidate = nullptr;
        std::vector<FrenetPath> best_set;
        double best_target_d = 0.0;
        int first_blocking_id = -1;

        // ===== COMMIT 이후에는 목표를 정확히 겨냥한다 (D6-3a) =====
        //
        // ResolveManeuver의 lateral 격자는
        //   target_d ± path_cfg_.lateral_target_tolerance (0.5 m)
        // 이다. COMMIT에서 +2.55를 잠갔는데 이후 사이클이 +2.05/+3.05를
        // 만들어 +3.05를 고르면 "목표를 lock했다"는 의미가 흐려진다.
        //
        // 그래서 committed AVOID는 tolerance를 0으로 해서 잠긴 목표만
        // 생성한다. PREPARE는 기존 격자를 그대로 쓴다 - 아직 목표를
        // 확정하지 않았으므로 주변 탐색이 맞다.
        //
        // ===== 현재 이 줄은 회귀로 덮이지 않는다 (정직한 기록) =====
        //
        // 위쪽에서 committed일 때 목표 목록을 잠긴 값 하나로 바꾸므로,
        // 격자 중심이 이미 잠긴 목표다. cost가 중심을 선호하기 때문에
        // tolerance를 0으로 하지 않아도 같은 값이 선택된다.
        //
        // 관측 조건을 찾으려 했지만(잠긴 d만 막고 이웃 ±0.5는 남기는
        // 장애물 배치) 실패했다 - 충돌 마진(0.3)과 차폭 때문에 잠긴 d
        // 근처를 막으면 이웃도 함께 막힌다. 즉 이 줄을 제거해도 현재
        // geometry에서는 동작이 같다(돌연변이 MUT-2가 통과했다).
        //
        // 그래도 남겨두는 이유는 방어 계층이기 때문이다. 위쪽 목표 목록
        // 대체가 깨지면(예: D7에서 목표 목록 로직을 고칠 때) 이 줄이
        // 마지막 방어선이 된다. 두 방어를 동시에 잃는 것보다 낫다.
        PathGeneratorConfig generation_cfg = path_cfg_;
        if (v2_avoidance_.committed) {
            generation_cfg.lateral_target_tolerance = 0.0;
        }

        for (int index = 0; index < v2_avoid_targets_.count; ++index) {
            const double target_d = v2_avoid_targets_.candidates[index];
            PlannerCommand side_cmd = command;
            side_cmd.mode = AVOID;
            side_cmd.avoidance_d_offset = target_d;
            side_cmd.target_speed = resolved_target_speed;

            PlannerDebugStats side_stats;
            auto generated = ResolveManeuver(
                start, side_cmd, ref_, generation_cfg, limits_, lane_width_,
                predicted_collision_obstacles, vehicle_shape_,
                collision_cfg_, &side_stats);
            EvaluateCosts(generated, cost_weights_, target_d);
            FilterByCartesianLateralAcceleration(generated, ref_,
                                                 limits_.max_lateral_accel);

            result.generated_candidate_count += generated.size();
            for (const auto& path : generated) {
                if (path.valid) ++result.valid_candidate_count;
                else if (first_blocking_id < 0 &&
                         path.rejection_reason == RejectionReason::COLLISION &&
                         path.collision_object_id >= 0) {
                    first_blocking_id = path.collision_object_id;
                }
            }

            const FrenetPath* side_best = SelectBestPath(generated);
            if (!side_best) continue;
            // NONE이면 cost로 좌/우를 고른다. LEFT/RIGHT는 count가 1이라
            // 이 비교가 일어나지 않는다.
            if (!best_candidate ||
                side_best->cost_total < best_candidate->cost_total) {
                best_set = std::move(generated);
                best_candidate = SelectBestPath(best_set);
                best_target_d = target_d;
            }
        }

        if (best_candidate) {
            result.candidate_valid = true;
            // **요청 목표가 아니라 선택된 후보의 실제 종단 d를 보고한다.**
            //
            // ResolveManeuver의 lateral 격자는 target_d 주변
            // ±lateral_target_tolerance(0.5 m)이므로, 선택된 후보는 요청
            // 목표에서 그 범위 안으로 벗어날 수 있다. 실측: 요청 +2.50 ->
            // 실제 종단 +3.00.
            //
            // 요청값을 그대로 FrenetFeedback.target_d로 내보내면 FSM이
            // 실제 회피 폭을 0.5 m까지 잘못 알게 된다(계약 9.2절의
            // "지정했다 vs 확보했다"와 같은 원칙).
            result.selected_target_d = best_candidate->d.empty()
                ? best_target_d : best_candidate->d.back();
            result.selected_cost = best_candidate->cost_total;
            result.candidate_path =
                ConvertToCartesianPath(*best_candidate, ref_);
        } else {
            result.blocking_object_id = first_blocking_id;
        }
    } else if (blocked_by_request) {
        // 요청 자체로 후보를 만들지 않았다. 충돌이 아니므로 blocking
        // object는 없다.
        result.blocking_object_id = -1;
    }

    if (avoid_committed) {
        if (!result.candidate_valid) {
            last_failure_reason_ = result.blocking_object_id >= 0
                ? PlanFailureReason::COLLISION_BLOCKED
                : PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            return false;
        }
        out_path = result.candidate_path;
        UpdateCommittedAvoidLifecycle(s, d, result);
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    // ---- 유지 궤적 --------------------------------------------------
    //
    // **AVOID 후보를 여기 넣지 않는다.** PREPARE 중 횡이동 발행은 계약
    // 5.2절이 금지한다 - FSM이 EXECUTE를 주기 전에 차가 움직인다.
    PlannerCommand maintenance_base = command;
    maintenance_base.target_speed = resolved_target_speed;
    if (!BuildAvoidMaintenancePath(ego, start, maintenance_base,
                                   predicted_collision_obstacles,
                                   out_path)) {
        last_failure_reason_ =
            PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
        return false;
    }

    // PREPARE 중에는 target_d를 lock하지 않는다. NONE이면 사이클마다
    // 좌/우가 바뀔 수 있고, FSM이 EXECUTE를 주지 않았으므로 그것이
    // 정상이다. lock은 D6-3의 COMMIT에서 한다(헤더 주석 참고).

    static int prepare_log_tick = 0;
    if (++prepare_log_tick % 5 == 0) {
        std::printf("[FrenetPlanner-STATE] mode=AVOID phase=PREPARE "
                    "s=%.2f d=%.3f speed=%.2f target_id=%d "
                    "candidate=%d target_d=%.3f evaluated=%d "
                    "low_speed=%d conflict=%d behind=%d "
                    "candidates=%zu/%zu maintenance=%zu\n",
                    s, d, ego.v, v2_avoidance_.obstacle_id,
                    result.candidate_valid ? 1 : 0, result.selected_target_d,
                    v2_avoid_targets_.count,
                    result.low_speed_context ? 1 : 0,
                    result.direction_conflict ? 1 : 0,
                    result.target_behind_ego ? 1 : 0,
                    result.valid_candidate_count,
                    result.generated_candidate_count, out_path.x.size());
    }

    // 반환값은 **maintenance가 유효한가**다. 후보 성공 여부는
    // result.candidate_valid로 따로 나간다(계약 8.3.1절).
    last_failure_reason_ = PlanFailureReason::NONE;
    return true;
}

void FrenetPlanner::UpdateLegacyAvoidContext(
    double s, double d, const FrenetState& start,
    const std::vector<TrackedObject>& obstacles, bool& begin_avoidance) {
    // 활성 context의 장애물 위치를 새 UDP 프레임으로 갱신한다. 일시적
    // packet dropout이 있어도 저장된 위치/길이로 회피를 계속한다.
    if (avoidance_.active) {
        for (const auto& obj : obstacles) {
            if (obj.id != avoidance_.obstacle_id) continue;
            double obj_s, obj_d, obj_s_dot;
            ProjectObjectToFrenet(ref_, obj, obj_s, obj_d, obj_s_dot);
            avoidance_.obstacle_s = obj_s;
            avoidance_.obstacle_d = obj_d;
            avoidance_.obstacle_width = obj.width;
            avoidance_.obstacle_length = obj.length;
            break;
        }

        const double gap = avoidance_.obstacle_s - s;
        const double clear_distance = 0.5 * (vehicle_shape_.length + avoidance_.obstacle_length)
                                    + avoid_cfg_.pass_clearance;
        if (avoidance_.phase == AvoidPhase::TRACKING && gap <= avoid_cfg_.shift_start_distance) {
            // 이 사이클에서 좌/우 전체 후보를 비교한 뒤, 유효 경로가
            // 있는 방향만 SHIFT로 잠그다.
            begin_avoidance = true;
        } else if (avoidance_.phase == AvoidPhase::SHIFT &&
                   std::fabs(d - avoidance_.target_d) <= avoid_cfg_.lateral_tolerance) {
            avoidance_.phase = AvoidPhase::PASS;
            std::printf("[FrenetPlanner] avoid phase: SHIFT -> PASS (id=%d gap=%.2f)\n",
                        avoidance_.obstacle_id, gap);
        } else if ((avoidance_.phase == AvoidPhase::SHIFT || avoidance_.phase == AvoidPhase::PASS) &&
                   gap < -clear_distance) {
            // 자차 뒤가 장애물을 완전히 통과했으므로 AVOID를 즉시 종료한다.
            // 이 아래의 !avoidance_.active 탐색이 같은 사이클에서 다음 정적
            // 장애물을 TRACKING으로 등록하고, 중앙 복귀는 LANE_KEEPING이 수행한다.
            std::printf("[FrenetPlanner] avoid complete after PASS: id=%d gap=%.2f\n",
                        avoidance_.obstacle_id, gap);
            avoidance_ = AvoidanceContext{};
        }
    }

    if (!avoidance_.active) {
        StaticObstacleTarget target;
        // LANE_KEEPING 충돌 필터는 reactive_lookahead 시간까지 미리 본다.
        // 정적 장애물 추적 거리가 그보다 짧으면, 장애물을 TRACKING으로
        // 등록하기 전에 충돌 필터가 중앙 후보를 전부 제거 -> stop path ->
        // 등록 후 다시 가속하는 속도 급락/반등이 생긴다. 설정값은 최소
        // 탐지거리로 쓰고, 실제 등록 거리는 "최고속도×충돌 lookahead +
        // 차량 길이/마진"보다 항상 길게 잡는다.
        AvoidConfig acquisition_cfg = avoid_cfg_;
        const double collision_horizon_distance =
            curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead
            + vehicle_shape_.length + collision_cfg_.safety_margin;
        acquisition_cfg.detection_distance =
            std::max(avoid_cfg_.detection_distance, collision_horizon_distance);
        if (FindStaticAvoidanceTarget(ref_, start, obstacles, lane_width_, vehicle_shape_,
                                      acquisition_cfg, target)) {
            avoidance_.active = true;
            avoidance_.obstacle_id = target.id;
            avoidance_.phase = AvoidPhase::TRACKING;
            avoidance_.obstacle_s = target.s;
            avoidance_.obstacle_d = target.d;
            avoidance_.obstacle_width = target.width;
            avoidance_.obstacle_length = target.length;
            // 저속/정지 상태에서도 영구 정지하지 않도록 selector가 확인한
            // 1차 방향을 보관한다. 정상 속도에서는 회피 시작 직전 양쪽의
            // 전체 후보를 비교해 더 정확한 target_d로 덮어쓴다.
            avoidance_.target_d = target.avoidance_offset;
            std::printf("[FrenetPlanner] static obstacle tracked: id=%d gap=%.2f "
                        "effective_detection=%.2f\n",
                        target.id, target.s - s, acquisition_cfg.detection_distance);
        }
    }
}

std::vector<FrenetPath> FrenetPlanner::GenerateAdaptiveLegacyAvoidCandidates(
    const FrenetState& start, PlannerCommand& avoid_cmd, double target_d,
    const std::vector<TrackedObject>& planning_collision_obstacles,
    PlannerDebugStats& avoid_stats) const {
    // AVOID의 고정 속도 대신 후보 곡률로 속도를 자동 결정한다.
    // 1차 일반 목표속도 생성 -> 최적 후보 곡률의 허용속도 계산 ->
    // 필요할 때만 낮은 목표속도로 2차 재생성 순서다.
    auto generate_once = [&](PlannerDebugStats& generated_stats) {
        generated_stats = PlannerDebugStats{};
        auto generated = ResolveManeuver(start, avoid_cmd, ref_, path_cfg_, limits_,
                                         lane_width_, planning_collision_obstacles,
                                         vehicle_shape_,
                                         collision_cfg_, &generated_stats);
        EvaluateCosts(generated, cost_weights_, target_d);
        return generated;
    };

    std::vector<FrenetPath> generated = generate_once(avoid_stats);
    const FrenetPath* initial_best = SelectBestPath(generated);
    if (initial_best) {
        const double speed_limit = CandidateCurvatureSpeedLimit(
            *initial_best, ref_, limits_.max_lateral_accel);
        if (std::isfinite(speed_limit) && speed_limit + 0.1 < avoid_cmd.target_speed) {
            avoid_cmd.target_speed = speed_limit;
            generated = generate_once(avoid_stats);
        }
    }

    FilterByCartesianLateralAcceleration(generated, ref_, limits_.max_lateral_accel);
    // 추가 횡가속도 필터를 반영해 시각화 통계를 재계산한다.
    avoid_stats.combined_valid_after_curvature = 0;
    avoid_stats.combined_valid_after_collision = 0;
    for (const auto& path : generated) {
        if (path.valid || path.rejection_reason == RejectionReason::COLLISION) {
            ++avoid_stats.combined_valid_after_curvature;
        }
        if (path.valid) ++avoid_stats.combined_valid_after_collision;
    }
    return generated;
}

void FrenetPlanner::GenerateLegacyAvoidDirectionCandidates(
    double s, const FrenetState& start,
    const std::vector<TrackedObject>& planning_collision_obstacles,
    PlannerCommand& cmd, double& avoid_offset, PlannerDebugStats& stats,
    std::vector<FrenetPath>& candidates) {
    // 장애물 d에서 자차/장애물 반폭과 SAT 안전 마진을 더한 지점을
    // 양쪽 목표로 삼는다. 기존 ±3m 절대값보다 장애물의 실제
    // 횡위치를 반영하므로 불필요하게 멀리 이동하지 않는다.
    constexpr double kDirectionExtraMargin = 0.3;
    const double clearance = 0.5 * (vehicle_shape_.width + avoidance_.obstacle_width)
                           + collision_cfg_.safety_margin + kDirectionExtraMargin;
    const double left_target = avoidance_.obstacle_d + clearance;
    const double right_target = avoidance_.obstacle_d - clearance;

    auto generate_side = [&](double target_d, PlannerDebugStats& side_stats,
                             double& used_target_speed) {
        PlannerCommand side_cmd = cmd;
        side_cmd.avoidance_d_offset = target_d;
        auto side = GenerateAdaptiveLegacyAvoidCandidates(
            start, side_cmd, target_d, planning_collision_obstacles, side_stats);
        used_target_speed = side_cmd.target_speed;
        return side;
    };

    PlannerDebugStats left_stats, right_stats;
    double left_target_speed = cmd.target_speed;
    double right_target_speed = cmd.target_speed;
    std::vector<FrenetPath> left_candidates =
        generate_side(left_target, left_stats, left_target_speed);
    std::vector<FrenetPath> right_candidates =
        generate_side(right_target, right_stats, right_target_speed);
    const FrenetPath* left_best = SelectBestPath(left_candidates);
    const FrenetPath* right_best = SelectBestPath(right_candidates);

    bool choose_right = false;
    bool direction_found = left_best || right_best;
    if (!left_best && right_best) {
        choose_right = true;
    } else if (left_best && right_best) {
        const double tie_margin = std::max(1.0, std::fabs(left_best->cost_total)) * 0.05;
        choose_right = right_best->cost_total < left_best->cost_total ||
            (avoid_cfg_.prefer_right_when_equal &&
             right_best->cost_total <= left_best->cost_total + tie_margin);
    }

    if (direction_found) {
        avoidance_.target_d = choose_right ? right_target : left_target;
        avoidance_.phase = AvoidPhase::SHIFT;
        cmd.avoidance_d_offset = avoidance_.target_d;
        cmd.target_speed = choose_right ? right_target_speed : left_target_speed;
        avoid_offset = avoidance_.target_d;
        candidates = choose_right ? std::move(right_candidates) : std::move(left_candidates);
        stats = choose_right ? right_stats : left_stats;
        std::printf("[FrenetPlanner] avoid direction locked: %s id=%d gap=%.2f "
                    "target_d=%.2f target_speed=%.2f valid(left=%zu right=%zu)\n",
                    choose_right ? "RIGHT" : "LEFT", avoidance_.obstacle_id,
                    avoidance_.obstacle_s - s, avoidance_.target_d, cmd.target_speed,
                    left_stats.combined_valid_after_collision,
                    right_stats.combined_valid_after_collision);
    } else {
        // 양쪽 모두 실패한 경우 방향을 잠그지 않고 두 집합을 모두
        // 시각화한다. 상위의 no-valid 처리가 안전 정지를 보낸 뒤
        // 다음 사이클에서 좌/우를 다시 평가한다.
        candidates = std::move(left_candidates);
        candidates.insert(candidates.end(),
                          std::make_move_iterator(right_candidates.begin()),
                          std::make_move_iterator(right_candidates.end()));
        stats.lateral_total = left_stats.lateral_total + right_stats.lateral_total;
        stats.lateral_valid = left_stats.lateral_valid + right_stats.lateral_valid;
        stats.longitudinal_total = left_stats.longitudinal_total + right_stats.longitudinal_total;
        stats.longitudinal_valid = left_stats.longitudinal_valid + right_stats.longitudinal_valid;
        stats.combined_total = left_stats.combined_total + right_stats.combined_total;
        stats.combined_valid_after_curvature = left_stats.combined_valid_after_curvature
                                             + right_stats.combined_valid_after_curvature;
        stats.combined_valid_after_collision = 0;
        std::printf("[FrenetPlanner] avoid direction unavailable: id=%d gap=%.2f "
                    "valid(left=0 right=0)\n",
                    avoidance_.obstacle_id, avoidance_.obstacle_s - s);
    }
}
