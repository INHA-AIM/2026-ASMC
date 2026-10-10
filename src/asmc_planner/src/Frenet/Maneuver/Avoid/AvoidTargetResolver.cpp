/**
 * @file AvoidTargetResolver.cpp
 * @brief External/v2 AVOID target acquisition and candidate target_d
 *        selection: "어떤 장애물을 회피할지 / 어디로 회피할지 결정하는가?"
 *        (R6-F1a - pure file split, no logic change).
 *
 * 함수 정의만 이 파일로 옮겼다 - 전부 여전히 FrenetPlanner의 member
 * function이고(선언은 frenet_planner.hpp), v2_avoidance_/v2_avoid_targets_
 * 등 기존 상태 소유권도 그대로다.
 */

#include "Frenet/frenet_planner.hpp"

#include <algorithm>
#include <cmath>

#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Math/FrenetConverter.hpp"
#include "Frenet/Math/Utils.hpp"

double FrenetPlanner::EffectiveAvoidDetectionDistance() const {
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

const TrackedObject* FrenetPlanner::FindSpatialAvoidReplacement(
    double ego_s, const std::vector<TrackedObject>& obstacles,
    const AvoidanceContext& context) const {
    if (!avoid_cfg_.spatial_reconnect_enabled || !execution_->avoid.active ||
        execution_->avoid.command_obstacle_id < 0 || context.obstacle_width <= 0.0 ||
        context.obstacle_length <= 0.0) {
        return nullptr;
    }

    const double max_ds = std::max(0.0, avoid_cfg_.reconnect_max_longitudinal_delta);
    const double max_dd = std::max(0.0, avoid_cfg_.reconnect_max_lateral_delta);
    const double max_size_delta = std::max(0.0, avoid_cfg_.reconnect_max_size_delta);
    const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
    const double search_limit = EffectiveAvoidDetectionDistance();
    const TrackedObject* unique_match = nullptr;

    for (const auto& object : obstacles) {
        if ((object.type != 0 && object.type != 3) ||
            object.motion_state == ObjectMotionState::DYNAMIC ||
            object.width <= 0.0 || object.length <= 0.0) {
            continue;
        }

        double object_s = 0.0, object_d = 0.0, object_s_dot = 0.0;
        ProjectObjectToFrenet(ref_, object, object_s, object_d, object_s_dot,
                             ref_config_.ego_projection_search_window);
        double gap = object_s - ego_s;
        if (gap <= 0.0 && track_length > 0.0) gap += track_length;
        const double unwrapped_s = ego_s + gap;
        if (!(gap > 0.0 && gap <= search_limit) ||
            std::fabs(unwrapped_s - context.obstacle_s) > max_ds ||
            std::fabs(object_d - context.obstacle_d) > max_dd ||
            std::fabs(object.length - context.obstacle_length) > max_size_delta ||
            std::fabs(object.width - context.obstacle_width) > max_size_delta) {
            continue;
        }

        const RefPoint object_ref = Interpolate(ref_, object_s);
        const double heading_delta = object.heading - object_ref.theta;
        const double lateral_half_extent =
            0.5 * std::fabs(object.length * std::sin(heading_delta)) +
            0.5 * std::fabs(object.width * std::cos(heading_delta));
        const double nearest_lateral_distance =
            std::max(0.0, std::fabs(object_d) - lateral_half_extent);
        if (nearest_lateral_distance > lane_width_ * 0.5) continue;

        // 여러 관측이 동시에 gate를 통과하면 잘못된 identity 결정을 하지 않는다.
        if (unique_match != nullptr) return nullptr;
        unique_match = &object;
    }
    return unique_match;
}

void FrenetPlanner::RefreshAvoidObservation(
    double ego_s, const TrackedObject& object, AvoidanceContext& context,
    bool refresh_committed_target) const {
    double object_s = 0.0, object_d = 0.0, object_s_dot = 0.0;
    ProjectObjectToFrenet(ref_, object, object_s, object_d, object_s_dot,
                         ref_config_.ego_projection_search_window);

    const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
    const double gap = ForwardArcDistance(ego_s, object_s, track_length);
    const RefPoint object_ref = Interpolate(ref_, object_s);
    const double heading_delta = object.heading - object_ref.theta;
    const double lateral_half_extent =
        0.5 * std::fabs(object.length * std::sin(heading_delta)) +
        0.5 * std::fabs(object.width * std::cos(heading_delta));
    const double longitudinal_half_extent =
        0.5 * std::fabs(object.length * std::cos(heading_delta)) +
        0.5 * std::fabs(object.width * std::sin(heading_delta));

    double outward_sign = context.target_d - context.obstacle_d;
    if (std::fabs(outward_sign) <= 1e-6) {
        if (execution_->avoid.requested_direction == LateralDirection::LEFT) outward_sign = 1.0;
        else if (execution_->avoid.requested_direction == LateralDirection::RIGHT) outward_sign = -1.0;
        else outward_sign = context.target_d >= 0.0 ? 1.0 : -1.0;
    } else {
        outward_sign = outward_sign > 0.0 ? 1.0 : -1.0;
    }

    execution_.observeAvoidObject(object.id);
    context.obstacle_s = ego_s + gap;
    context.obstacle_d = object_d;
    context.obstacle_width = object.width;
    context.obstacle_length = object.length;
    context.obstacle_lateral_half_extent = lateral_half_extent;
    context.obstacle_longitudinal_half_extent = longitudinal_half_extent;

    if (refresh_committed_target && execution_->avoid.committed &&
        !context.has_requested_target_d) {
        const double clearance =
            0.5 * vehicle_shape_.width + lateral_half_extent +
            collision_cfg_.safety_margin + avoid_cfg_.direction_extra_margin;
        const double required_target = object_d + outward_sign * clearance;
        context.target_d = outward_sign > 0.0
            ? std::max(context.target_d, required_target)
            : std::min(context.target_d, required_target);
    }
}

FrenetPlanner::AvoidTargetStatus FrenetPlanner::ResolveExternalAvoidTarget(
    double ego_s, int target_object_id,
    const std::vector<TrackedObject>& obstacles,
    AvoidanceContext& context) const {
    if (target_object_id < 0) return AvoidTargetStatus::LOST;

    // 지정된 ID만 찾는다. "가장 가까운 정적 장애물"을 스스로 고르지 않는다
    // - 대상 선택 권한은 FSM에 있다(헤더 주석 참고).
    const auto it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [target_object_id](const TrackedObject& object) {
            return object.id == target_object_id;
        });

    if (it != obstacles.end()) {
        double object_s = 0.0, object_d = 0.0, object_s_dot = 0.0;
        ProjectObjectToFrenet(ref_, *it, object_s, object_d, object_s_dot,
                             ref_config_.ego_projection_search_window);

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
        // geometry가 0이면 회피 오프셋을 계산할 수 없다. 후보 생성 단계에서
        // 0 폭 장애물을 비켜가는 target_d는 의미가 없다.
        const bool geometry_ok = it->width > 0.0 && it->length > 0.0;
        // 회피는 같은 corridor와 실제로 겹치는 장애물을 대상으로 한다.
        // 중심 d만 검사하면 폭이 넓은 장애물의 외곽이 차로에 들어와도 중심이
        // 경계에서 몇 cm 벗어났다는 이유로 Behavior는 AVOID, Frenet은 LOST가
        // 된다. OBB의 route-normal 반폭을 투영해 가장 가까운 외곽을 검사한다.
        double nearest_lateral_distance = std::fabs(object_d);
        if (geometry_ok) {
            const RefPoint object_ref = Interpolate(ref_, object_s);
            const double heading_delta = it->heading - object_ref.theta;
            const double lateral_half_extent =
                0.5 * std::fabs(it->length * std::sin(heading_delta)) +
                0.5 * std::fabs(it->width * std::cos(heading_delta));
            nearest_lateral_distance =
                std::max(0.0, nearest_lateral_distance - lateral_half_extent);
        }
        const bool same_corridor =
            nearest_lateral_distance <= lane_width_ * 0.5;

        if (in_front && in_range && same_corridor && geometry_ok) {
            RefreshAvoidObservation(
                ego_s, *it, context, /*refresh_committed_target=*/false);
            execution_.bindAvoidCommandTarget(target_object_id);
            // 요청 ID가 아니라 **실제로 찾은 객체의 ID**를 기록한다. 둘은
            // 항상 같아야 하지만, 같다고 가정해 요청 ID를 그대로 넣으면
            // 조회가 잘못되어도(예: nearest selection으로 회귀) 확보 ID는
            // 정상으로 보인다 - 진단과 회귀 검증이 그 오류를 놓친다.
            execution_.observeAvoidVisible();
            return AvoidTargetStatus::VALID;
        }

        // 관측되었지만 쓸 수 없는 상태다. dropout이 아니므로 유예를 주지
        // 않는다 - 예측으로 메꿀 대상이 아니라 조건 자체가 틀렸다
        // (계약 8.2절). phase/target_d/start_d는 호출자가 관리하므로
        // 여기서는 확보 상태만 해제한다.
        execution_.clearAvoidBinding();
        return AvoidTargetStatus::LOST;
    }

    if (execution_->avoid.active && execution_->avoid.command_obstacle_id == target_object_id) {
        const TrackedObject* replacement =
            FindSpatialAvoidReplacement(ego_s, obstacles, context);
        if (replacement != nullptr) {
            RefreshAvoidObservation(
                ego_s, *replacement, context,
                /*refresh_committed_target=*/false);
            execution_.observeAvoidVisible();
            return AvoidTargetStatus::VALID;
        }
    }

    // 관측되지 않았다. 직전에 확보한 상태가 있으면 유예한다.
    //
    // 한 번도 확보하지 못한 대상에는 유예를 주지 않는다. FSM이 존재하지 않는
    // ID를 보낸 경우를 조용히 넘기게 된다(FOLLOW와 동일한 판단).
    if (!execution_->avoid.active || execution_->avoid.command_obstacle_id != target_object_id) {
        execution_.clearAvoidBinding();
        return AvoidTargetStatus::LOST;
    }

    if (execution_.observeAvoidMissing(avoid_cfg_.dropout_grace_cycles)) {
        execution_.clearAvoidBinding();
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
    if (!execution_->avoid.active) return plan;

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
    const double clearance =
        0.5 * vehicle_shape_.width + context.obstacle_lateral_half_extent +
        collision_cfg_.safety_margin + avoid_cfg_.direction_extra_margin;
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
    execution_.requestAvoidDirection(command.requested_direction);

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

    // 가장 가까운 정적 장애물과 회피 방향을 스스로 자동 결정하지 않는다
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
    if (execution_->avoid.committed) {
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
