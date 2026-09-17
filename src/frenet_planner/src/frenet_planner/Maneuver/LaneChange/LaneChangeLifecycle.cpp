/**
 * @file LaneChangeLifecycle.cpp
 * @brief External/v2 LANE_CHANGE commit state, SHIFT -> HOLD_TARGET_LANE
 *        transition, progress, and reset (R6-F2 - pure file split, no
 *        logic change).
 *
 * 함수 정의만 이 파일로 옮겼다 - 전부 여전히 FrenetPlanner의 member
 * function이고(선언은 frenet_planner.hpp), v2_lane_change_ 등 기존 상태
 * 소유권도 그대로다.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/Math/Utils.hpp"

bool FrenetPlanner::CanCommitLaneChange(
    const LaneChangePrepareResult& result) const {
    if (!result.candidate_valid) return false;
    if (!v2_lane_change_.active) return false;
    if (v2_lane_change_.committed) return false;
    if (!std::isfinite(result.selected_target_d)) return false;
    return true;
}

void FrenetPlanner::ApplyLaneChangeCommit(
    const LaneChangePrepareResult& result) {
    v2_lane_change_.committed = true;
    v2_lane_change_.maneuver_progress = 0.0;
    // 이 시점에 처음 잠근다(AVOID와 동일한 이유 - PREPARE 사이클마다 후보
    // 재평가 결과가 미세하게 흔들릴 수 있으므로 COMMIT 전에는 잠그지
    // 않는다). target_d 자체는 PrepareExternalLaneChangeTarget이 이미
    // target_pose_projection_d를 candidate 생성에 넣었으므로, 여기서는 그
    // 값을 그대로 잠근다 - 다시 계산하지 않는다.
    v2_lane_change_.target_d = result.selected_target_d;
    v2_lane_change_.phase = LaneChangePhase::SHIFT;
}

void FrenetPlanner::UpdateCommittedLaneChangeLifecycle(
    double ego_s, double ego_d, double ego_d_dot, double ego_yaw,
    LaneChangePrepareResult& result) {
    if (!v2_lane_change_.active || !v2_lane_change_.committed) return;

    constexpr double kProgressEpsilon = 1e-6;

    // ---- SHIFT -> HOLD_TARGET_LANE 전환 판정 -----------------------------
    //
    // 단순 |d - target_d| < threshold 하나만으로 전환하지 않는다. 횡방향
    // 위치 + 속도 + heading 정렬 세 조건을 모두 요구한다 - 위치만 보면 옆을
    // 스치듯 지나가는 순간에도 "도달"로 잘못 판정될 수 있다. PlannerCommand는
    // target_pose_map 투영의 d/s만 옮겨싣고 목표 yaw는 옮기지 않으므로
    // (target_pose_projection_* 주석 참고), heading 정렬은 현재 ego 위치의
    // reference heading(직진 차로라면 목표 차로와 사실상 같은 방향)과
    // 비교한다.
    if (v2_lane_change_.phase == LaneChangePhase::SHIFT) {
        const bool position_ok =
            std::isfinite(ego_d) &&
            std::fabs(ego_d - v2_lane_change_.target_d) <=
                lane_change_cfg_.lateral_position_tolerance;
        const bool speed_ok =
            std::isfinite(ego_d_dot) &&
            std::fabs(ego_d_dot) <= lane_change_cfg_.lateral_speed_tolerance;

        bool heading_ok = true;
        double reference_heading = 0.0;
        if (std::isfinite(ego_yaw) && ReferenceHeadingAt(ego_s, reference_heading)) {
            heading_ok = std::fabs(angleDiff(ego_yaw, reference_heading)) <=
                         lane_change_cfg_.heading_tolerance;
        }

        if (position_ok && speed_ok && heading_ok) {
            v2_lane_change_.phase = LaneChangePhase::HOLD_TARGET_LANE;
        }
    }

    const bool holding = v2_lane_change_.phase == LaneChangePhase::HOLD_TARGET_LANE;
    double progress = 0.0;
    if (holding) {
        progress = 1.0;
    } else {
        const double denominator = std::max(
            std::fabs(v2_lane_change_.target_d - v2_lane_change_.start_d),
            kProgressEpsilon);
        progress =
            std::isfinite(ego_d) && std::isfinite(v2_lane_change_.start_d)
                ? clip(std::fabs(ego_d - v2_lane_change_.start_d) /
                           denominator,
                       0.0, 1.0)
                : 0.0;
    }

    if (!std::isfinite(progress)) progress = 0.0;
    progress = clip(progress, 0.0, 1.0);
    // localization jitter로 current d가 잠시 역행해도 FSM progress는
    // 되돌아가지 않는다(AVOID와 동일한 정책).
    v2_lane_change_.maneuver_progress =
        std::max(v2_lane_change_.maneuver_progress, progress);
    result.reached_target_lane = holding;
    result.maneuver_progress = v2_lane_change_.maneuver_progress;
}

void FrenetPlanner::ResetExternalLaneChange() {
    v2_lane_change_ = LaneChangeContext{};
}
