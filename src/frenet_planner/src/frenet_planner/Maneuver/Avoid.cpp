/**
 * @file Avoid.cpp
 * @brief External/v2 AVOID target resolution, commit state, lifecycle,
 *        target-preparation helpers.
 *
 * PlanInternal의 PREPARE/EXECUTE candidate orchestration, legacy AVOID
 * acquisition은 후속 AVOID refactor 단계까지 남겨둔다. PrepareAvoid/
 * PlanCommittedAvoid(내부 anonymous-namespace ValidateExternalCommand
 * 의존)와 BuildAvoidMaintenancePath(내부 anonymous-namespace
 * FilterByCartesianLateralAcceleration 의존)도 이 파일로 옮기지 않는다 -
 * 옮기려면 그 helper들의 linkage를 새로 공개해야 하며, 이번 단계의 범위를
 * 넘어선다.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/Math/Utils.hpp"

double FrenetPlanner::EffectiveAvoidDetectionDistance() const {
    // legacy AVOID 등록거리와 같은 식이다(frenet_planner.cpp의 acquisition_cfg).
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
    const std::vector<ObjectInfo>& obstacles,
    AvoidanceContext& context) const {
    if (target_object_id < 0) return AvoidTargetStatus::LOST;

    // 지정된 ID만 찾는다. FindStaticAvoidanceTarget()처럼 "가장 가까운 정적
    // 장애물"을 고르지 않는다 - 대상 선택 권한은 FSM에 있다(헤더 주석 참고).
    const auto it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [target_object_id](const ObjectInfo& object) {
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
    double ego_s, const std::vector<ObjectInfo>& obstacles,
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
