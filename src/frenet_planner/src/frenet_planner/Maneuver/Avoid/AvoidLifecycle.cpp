/**
 * @file AvoidLifecycle.cpp
 * @brief External/v2 AVOID commit state, TRACKING/SHIFT/PASS/RETURN
 *        lifecycle, progress, and reset (R6-F1b - pure file split, no
 *        logic change).
 *
 * 함수 정의만 이 파일로 옮겼다 - 전부 여전히 FrenetPlanner의 member
 * function이고(선언은 frenet_planner.hpp), v2_avoidance_ 등 기존 상태
 * 소유권도 그대로다.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/Maneuver/ManeuverUtils.hpp"
#include "frenet_planner/Math/Utils.hpp"

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
    v2_avoidance_.escape_recovery_active = false;
    v2_avoidance_.maneuver_progress = 0.0;
    // 이 시점에 처음 lock한다. PREPARE에서는 사이클마다 좌/우가 바뀔 수
    // 있으므로 잠그지 않았다(계약 12.7.3절).
    v2_avoidance_.target_d = result.selected_target_d;
    // 진행률 기준점. maneuver_progress를 (start_d -> target_d) 구간으로
    // 계산한다(D6-3c).
    v2_avoidance_.start_d = last_d_;
    // AVOID를 멀리서 확보/COMMIT해도 곧바로 옆으로 흐르지 않는다.
    // TRACKING에서 global path를 유지하다 장애물 기준 진입거리에 도달하면
    // SHIFT로 넘어간다.
    const double track_length =
        ref_.points.empty() ? 0.0 : ref_.points.back().s;
    const double initial_gap = ForwardArcDistance(
        last_s_, v2_avoidance_.obstacle_s, track_length);
    // 이후 lifecycle과 동일하게 실제 ego 속도를 사용한다. 과거에는 planner
    // target speed를 써서 저속 차량도 고속 기준으로 너무 일찍 SHIFT했다.
    const double shift_distance = EffectiveAvoidShiftStartDistance(
        result.ego_speed_mps, v2_avoidance_.start_d);
    v2_avoidance_.phase = std::isfinite(initial_gap) &&
                                  initial_gap <= shift_distance
                              ? AvoidPhase::SHIFT
                              : AvoidPhase::TRACKING;
    // PREPARE에서 복구 후보로만 READY가 된 transaction은 COMMIT 직후에도
    // 같은 거리 기반 정책을 이어가야 한다. TRACKING이면 아직 복구가 필요할
    // 단계가 아니므로 latch하지 않는다.
    v2_avoidance_.escape_recovery_active =
        result.escape_recovery_required &&
        v2_avoidance_.phase == AvoidPhase::SHIFT;
}

double FrenetPlanner::EffectiveAvoidShiftStartDistance(
    double ego_speed, double ego_d) const {
    return EffectiveAvoidShiftStartDistance(
        ego_speed, ego_d, v2_avoidance_.target_d);
}

double FrenetPlanner::EffectiveAvoidShiftStartDistance(
    double ego_speed, double ego_d, double target_d) const {
    const double min_distance =
        std::max(1.0, avoid_cfg_.min_shift_start_distance);
    const double max_distance =
        std::max(min_distance, avoid_cfg_.max_shift_start_distance);
    const double lateral_distance = std::fabs(
        target_d - (std::isfinite(ego_d) ? ego_d : 0.0));
    // Rest-to-rest quintic의 peak |d_ddot|은 대략 5.77*D/T^2다.
    // 실제 Cartesian 횡가속도 필터와 동일한 한계를 사용해 필요한 최소
    // 횡이동 시간을 구하고, 제어 응답 여유를 더한다.
    const double usable_lateral_accel = std::max(
        0.1, limits_.max_lateral_accel *
                 limits_.avoid_shift_usable_lateral_accel_fraction);
    const double shift_time =
        std::sqrt(5.77 * lateral_distance / usable_lateral_accel) +
        std::max(0.0, avoid_cfg_.shift_time_margin);
    // VehicleState x/y는 후륜축 GPS 기준이다. 접근 중 필요한 자차 길이는
    // length/2가 아니라 후륜축에서 앞 범퍼까지의 거리다.
    const double rear_overhang = clip(
        avoid_cfg_.rear_overhang, 0.0, vehicle_shape_.length);
    const double front_extent = vehicle_shape_.length - rear_overhang;
    const double geometry_clearance =
        front_extent + v2_avoidance_.obstacle_longitudinal_half_extent +
        collision_cfg_.safety_margin;
    double dynamic_distance =
        std::max(0.0, ego_speed) * shift_time + geometry_clearance;
    // limits_.low_speed_fallback_v 미만에서는 시간 기반 quintic 대신 거리
    // 기반 fallback을 실행한다. 그 거리는 고정값이 아니라 목표 횡이동량과
    // max_curvature로 계산하며, 차량/장애물 형상 여유를 별도로 합쳐 SHIFT를
    // 시작한다.
    if (std::fabs(ego_speed) < limits_.low_speed_fallback_v) {
        dynamic_distance = std::max(
            dynamic_distance,
            LowSpeedLateralTransitionDistance(
                ego_d, target_d) + geometry_clearance);
    }
    // min/max는 안전 경계로만 유지하고, 사용자가 실제 시작거리를 직접
    // 튜닝할 수 있는 signed offset은 동적 계산에 명시적으로 적용한다.
    dynamic_distance += avoid_cfg_.shift_start_distance_offset;
    return clip(dynamic_distance, min_distance, max_distance);
}

void FrenetPlanner::UpdateCommittedAvoidLifecycle(
    double ego_s, double ego_d, double ego_speed,
    AvoidPrepareResult& result) {
    if (!v2_avoidance_.active || !v2_avoidance_.committed) return;

    constexpr double kProgressEpsilon = 1e-6;
    bool entered_pass_this_cycle = false;
    const double track_length =
        ref_.points.empty() ? 0.0 : ref_.points.back().s;
    // Resolver가 저장한 장애물 s와 ego s 사이의 signed arc 거리.
    const double gap = ForwardArcDistance(
        ego_s, v2_avoidance_.obstacle_s, track_length);

    if (!v2_avoidance_.completed &&
        v2_avoidance_.phase == AvoidPhase::TRACKING &&
        std::isfinite(gap) &&
        gap <= EffectiveAvoidShiftStartDistance(ego_speed, ego_d)) {
        v2_avoidance_.phase = AvoidPhase::SHIFT;
        // 실제 SHIFT가 시작되는 횡위치를 진행률 기준으로 사용한다.
        if (std::isfinite(ego_d)) v2_avoidance_.start_d = ego_d;
    }
    // ego_s는 후륜축 위치다. 장애물을 완전히 통과했는지는 차량 반길이가
    // 아니라 후륜축에서 뒤 범퍼까지의 실제 거리로 판정해야 한다.
    const double rear_overhang = clip(
        avoid_cfg_.rear_overhang, 0.0, vehicle_shape_.length);
    const double clear_distance =
        rear_overhang + v2_avoidance_.obstacle_longitudinal_half_extent +
        avoid_cfg_.pass_clearance;
    const bool longitudinally_cleared =
        std::isfinite(gap) && std::isfinite(clear_distance) &&
        clear_distance > kProgressEpsilon && gap < -clear_distance;

    // LiDAR OBB는 접근하며 보이는 면이 늘어 target_d가 바깥쪽으로 확장될
    // 수 있다. 예전에는 그 목표 d를 끝까지 찍어야 PASS가 되어, 장애물을
    // 이미 완전히 지난 뒤에도 SHIFT가 남아 복귀가 늦거나 margin 안에서
    // 정지했다. 종방향으로 차체와 target OBB가 완전히 분리된 경우에는
    // target_d 미도달과 무관하게 RETURN을 시작한다. RETURN 궤적 자체는
    // 이후 동일한 전체 OBB 충돌 검사를 거치므로 안전 검사를 우회하지 않는다.
    if (!v2_avoidance_.completed &&
        v2_avoidance_.phase == AvoidPhase::SHIFT &&
        longitudinally_cleared) {
        v2_avoidance_.phase = AvoidPhase::RETURN;
    } else if (!v2_avoidance_.completed &&
               v2_avoidance_.phase == AvoidPhase::SHIFT &&
               std::isfinite(ego_d) &&
               std::isfinite(v2_avoidance_.target_d) &&
               std::fabs(ego_d - v2_avoidance_.target_d) <=
                   avoid_cfg_.lateral_tolerance) {
        v2_avoidance_.phase = AvoidPhase::PASS;
        entered_pass_this_cycle = true;
    }
    // Resolver는 seam 너머 전방 target을 ego_s + forward gap의 unwrapped
    // obstacle_s로 저장한다. raw subtraction은 ego가 seam을 지난 순간 거의
    // 한 바퀴 양수가 되므로, 기존 signed arc helper로 앞/뒤 의미를 보존한다.
    // 장애물을 완전히 지난 뒤에도 바로 COMPLETED하지 않는다. RETURN에서
    // d=0 복귀가 끝나야 Behavior가 KEEP으로 transaction을 넘길 수 있다.
    if (!v2_avoidance_.completed && !entered_pass_this_cycle &&
        v2_avoidance_.phase == AvoidPhase::PASS &&
        longitudinally_cleared) {
        v2_avoidance_.phase = AvoidPhase::RETURN;
    }
    if (!v2_avoidance_.completed &&
        v2_avoidance_.phase == AvoidPhase::RETURN &&
        std::isfinite(ego_d) &&
        std::fabs(ego_d) <= avoid_cfg_.lateral_tolerance) {
        v2_avoidance_.completed = true;
    }

    // 복구 latch는 장애물 옆으로 이동하는 SHIFT에만 유효하다. 목표 d에
    // PASS 이후에는 속도 경계 진동 방지용 거리 기반 generator latch만
    // 해제한다. margin-recovery 충돌 규칙 자체는 모든 phase에서 동일하다.
    if (v2_avoidance_.phase != AvoidPhase::SHIFT) {
        v2_avoidance_.escape_recovery_active = false;
    }

    double progress = 0.0;
    if (v2_avoidance_.completed) {
        progress = 1.0;
    } else if (v2_avoidance_.phase == AvoidPhase::TRACKING) {
        const double start_distance =
            EffectiveAvoidShiftStartDistance(ego_speed, ego_d);
        const double approach_ratio = std::isfinite(gap)
            ? clip((EffectiveAvoidDetectionDistance() - gap) /
                       std::max(1.0, EffectiveAvoidDetectionDistance() -
                                         start_distance),
                   0.0, 1.0)
            : 0.0;
        progress = 0.2 * approach_ratio;
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
        progress = 0.2 + 0.3 * lateral_ratio;
    } else if (v2_avoidance_.phase == AvoidPhase::PASS) {
        const double pass_ratio =
            std::isfinite(gap) && std::isfinite(clear_distance) &&
                    clear_distance > kProgressEpsilon
                ? clip(-gap / clear_distance, 0.0, 1.0)
                : 0.0;
        progress = 0.5 + 0.25 * pass_ratio;
    } else {
        const double return_ratio = std::isfinite(ego_d) &&
                                    std::isfinite(v2_avoidance_.target_d)
            ? clip(1.0 - std::fabs(ego_d) /
                             std::max(std::fabs(v2_avoidance_.target_d),
                                      kProgressEpsilon),
                   0.0, 1.0)
            : 0.0;
        progress = 0.75 + 0.25 * return_ratio;
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
        case AvoidPhase::RETURN: return "RETURN";
    }
    return "UNKNOWN";
}

void FrenetPlanner::ResetExternalAvoidance() {
    v2_avoidance_ = AvoidanceContext{};
    v2_avoid_targets_ = AvoidTargetPlan{};
}
