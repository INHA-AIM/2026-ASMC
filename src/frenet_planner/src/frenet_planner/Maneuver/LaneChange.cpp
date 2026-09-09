/**
 * @file LaneChange.cpp
 * @brief External/v2 LANE_CHANGE target context, commit state, lifecycle,
 *        maintenance path, PREPARE/COMMITTED execution (D7).
 *
 * Maneuver/Avoid.cpp의 PREPARE -> READY -> EXECUTE -> COMMITTED -> EXECUTING
 * -> COMPLETED 패턴을 그대로 재사용한다(§1). 차이점:
 *   - 회피 대상 객체가 없다. 목표는 항상 FSM의 target_pose_map 투영값이다.
 *   - 좌/우 후보 경쟁이 없다 - mode(LANE_CHANGE_LEFT/RIGHT)가 이미 방향을
 *     정하므로 평가할 후보는 항상 하나뿐이다.
 *   - PASS phase가 없다 - 지나쳐야 할 장애물이 없으므로 SHIFT 완료(목표 d +
 *     lateral rate + heading 정렬)가 곧 maneuver 완료다(§17).
 *
 * PrepareLaneChange/PlanCommittedLaneChange는 frenet_planner.cpp에 남는다 -
 * 익명 namespace의 ValidateExternalCommand에 의존하기 때문이다(Avoid.cpp와
 * 같은 이유).
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "frenet_planner/Evaluation/Cost.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Maneuver/ManeuverUtils.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"
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
    v2_lane_change_.completed = false;
    v2_lane_change_.maneuver_progress = 0.0;
    // 이 시점에 처음 lock한다(AVOID와 동일한 이유 - PREPARE 사이클마다
    // 재투영 결과가 미세하게 흔들릴 수 있으므로 COMMIT 전에는 잠그지 않는다).
    v2_lane_change_.target_d = result.selected_target_d;
    v2_lane_change_.start_d = last_d_;
    v2_lane_change_.phase = LaneChangePhase::SHIFT;
}

void FrenetPlanner::UpdateCommittedLaneChangeLifecycle(
    double ego_d, double ego_d_dot, double ego_yaw,
    LaneChangePrepareResult& result) {
    if (!v2_lane_change_.active || !v2_lane_change_.committed) return;

    constexpr double kProgressEpsilon = 1e-6;

    // ---- 완료 판정 (§17) ------------------------------------------------
    //
    // 단순 |d - target_d| < threshold 하나만으로 완료하지 않는다. 횡방향
    // 위치 + 속도 + heading 정렬 세 조건을 모두 요구한다 - 위치만 보면
    // 옆으로 빠르게 스치듯 지나가는 순간에도 "완료"로 잘못 판정될 수 있다.
    if (!v2_lane_change_.completed &&
        v2_lane_change_.phase == LaneChangePhase::SHIFT) {
        const bool position_ok =
            std::isfinite(ego_d) &&
            std::fabs(ego_d - v2_lane_change_.target_d) <=
                lane_change_cfg_.lateral_position_tolerance;
        const bool speed_ok =
            std::isfinite(ego_d_dot) &&
            std::fabs(ego_d_dot) <= lane_change_cfg_.lateral_speed_tolerance;

        // heading 정렬: 목표 heading이 있으면 그것과, 없으면(투영 실패로
        // heading을 못 채운 극단적 상황) reference heading과 비교한다.
        // ±pi wrap이 거짓으로 완료를 막지 않도록 반드시 angleDiff로
        // 정규화한 차이를 쓴다.
        bool heading_ok = true;
        if (v2_lane_change_.has_target_yaw && std::isfinite(ego_yaw)) {
            heading_ok = std::fabs(angleDiff(
                             ego_yaw, v2_lane_change_.target_yaw)) <=
                         lane_change_cfg_.heading_tolerance;
        }

        if (position_ok && speed_ok && heading_ok) {
            v2_lane_change_.completed = true;
        }
    }

    double progress = 0.0;
    if (v2_lane_change_.completed) {
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
    // 되돌아가지 않는다(AVOID와 동일한 정책, §18).
    v2_lane_change_.maneuver_progress =
        std::max(v2_lane_change_.maneuver_progress, progress);
    result.maneuver_completed = v2_lane_change_.completed;
    result.maneuver_progress = v2_lane_change_.maneuver_progress;
}

void FrenetPlanner::ResetExternalLaneChange() {
    v2_lane_change_ = LaneChangeContext{};
}

bool FrenetPlanner::PrepareExternalLaneChangeTarget(
    double ego_s, const PlannerCommand& command, bool lane_change_prepare,
    bool lane_change_committed) {
    if (lane_change_committed) {
        // COMMIT 이후 geometry의 소유권은 잠긴 context에 있다. target pose를
        // 다시 읽지 않는다(AVOID의 committed 분기와 동일한 이유).
        return true;
    }

    // mode가 이미 방향을 정하므로 요청 방향은 진단용으로만 보존한다.
    v2_lane_change_.requested_direction =
        command.mode == LANE_CHANGE_LEFT ? LateralDirection::LEFT
        : command.mode == LANE_CHANGE_RIGHT ? LateralDirection::RIGHT
                                            : LateralDirection::NONE;

    // FSM이 준 target_pose_map 투영 결과를 context에 보존한다. LANE_CHANGE는
    // has_target_pose가 항상 true여야 하므로(계약 3.3절 검증 matrix),
    // has_lane_change_target_d=false는 투영 실패를 뜻한다 - AVOID처럼 "pose가
    // 선택 사항이라 없다"가 아니다.
    v2_lane_change_.has_requested_target_d = command.has_lane_change_target_d;
    v2_lane_change_.requested_target_d =
        command.has_lane_change_target_d ? command.lane_change_target_d : 0.0;
    v2_lane_change_.requested_target_s =
        command.has_lane_change_target_d ? command.lane_change_target_s : 0.0;
    v2_lane_change_.active = command.has_lane_change_target_d;

    if (!lane_change_prepare && !lane_change_committed) {
        // NORMAL purpose로 LANE_CHANGE가 들어오면 아직 실행 경로가 없다.
        // 그대로 내려보내면 ResolveManeuver의 LANE_CHANGE 분기가
        // lane_change_d_offset=0으로 중앙 유지 궤적을 만들어, FSM 입장에서는
        // "차선변경 성공"으로 보이면서 실제로는 차선을 바꾸지 않는다
        // (AVOID의 동일 분기와 같은 원칙, 계약 11.4절).
        last_mode_ = command.mode;
        last_failure_reason_ = PlanFailureReason::UNSUPPORTED_BEHAVIOR;
        return false;
    }

    if (!command.has_lane_change_target_d) {
        // 목표점 투영 실패. LANE_CHANGE는 target pose 없이는 목표 d를 알 수
        // 없다 - AVOID의 geometry fallback(장애물 위치+폭으로 자체 계산)에
        // 대응하는 것이 없다. FSM이 지정한 목표를 해석하지 못했으므로 명령
        // 결함으로 본다.
        last_mode_ = command.mode;
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    (void)ego_s;
    return true;
}

bool FrenetPlanner::BuildLaneChangeMaintenancePath(
    const CartesianState& ego, const FrenetState& start,
    const PlannerCommand& base_command,
    const std::vector<TrackedObject>& collision_obstacles,
    CartesianPath& out_path) const {
    const double s = start.s;
    const double d = start.d;

    // 우선순위는 BuildAvoidMaintenancePath와 동일하다(§9):
    //   1. 충돌 검사를 통과한 현재 차선 유지(KEEP) 궤적 - LC candidate가
    //      아직 unsafe라는 이유만으로 감속/정지하지 않는다.
    //   2. 종방향 감속
    //   3. 0속도 hold
    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s] (PlanInternal과 동일)
    const bool low_speed = std::fabs(ego.v) < kLowSpeedFallbackV;

    if (low_speed) {
        CartesianPath crawl;
        if (PlanLowSpeedFallback(ego, s, d, d, crawl, 2.0,
                                 base_command.target_speed) &&
            crawl.x.size() >= 2) {
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
        keep_cmd.lane_change_d_offset = 0.0;
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

    BuildZeroSpeedHoldPath(ego, s, out_path);
    return !out_path.x.empty();
}

bool FrenetPlanner::LaneChangeProcess(
    const CartesianState& ego, const FrenetState& start,
    double s, double d, double resolved_target_speed,
    const std::vector<TrackedObject>& predicted_collision_obstacles,
    const PlannerCommand& command, bool lane_change_prepare,
    bool lane_change_committed, CartesianPath& out_path,
    LaneChangePrepareResult& result) {
    last_mode_ = command.mode;
    last_target_speed_ = resolved_target_speed;

    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s]
    result.low_speed_context = std::fabs(ego.v) < kLowSpeedFallbackV;

    // ---- target pose sanity: ego 전방이어야 한다 (AVOID와 동일한 이유) ----
    if (lane_change_prepare && v2_lane_change_.has_requested_target_d) {
        const double track_length =
            ref_.points.empty() ? 0.0 : ref_.points.back().s;
        double target_gap = v2_lane_change_.requested_target_s - s;
        if (target_gap <= 0.0 && track_length > 0.0) {
            target_gap += track_length;
        }
        const bool behind =
            !(target_gap > 0.0) ||
            (track_length > 0.0 && target_gap > track_length * 0.5);
        result.target_behind_ego = behind;
    }

    // heading 정렬 판정용 목표 heading을 채운다. 투영 시점의 reference
    // heading을 그대로 목표 heading으로 쓴다 - LANE_CHANGE 완료 시점에는
    // 차량이 (새) 차로와 나란해야 하므로, target_pose_map 자체의 orientation이
    // 아니라 그 s 위치의 참조선 진행방향이 기준이다.
    if (v2_lane_change_.has_requested_target_d) {
        double heading = 0.0;
        v2_lane_change_.has_target_yaw =
            ReferenceHeadingAt(v2_lane_change_.requested_target_s, heading);
        v2_lane_change_.target_yaw = heading;
    }

    const bool blocked_by_request =
        lane_change_prepare && result.target_behind_ego;

    // 정상 후보 생성기가 정지 근처에서 횡 후보를 만들지 못하는 경우
    // COMMIT 이후에는 잠긴 target_d를 향하는 crawl로 대체한다(AVOID와 동일).
    if (lane_change_committed && result.low_speed_context) {
        result.generated_candidate_count = 1;
        CartesianPath crawl;
        PlanLowSpeedFallback(ego, s, d, v2_lane_change_.target_d, crawl, 2.0,
                             resolved_target_speed);
        int blocking_id = -1;
        if (crawl.x.size() < 2 ||
            !CartesianTrajectoryClear(crawl, predicted_collision_obstacles,
                                      vehicle_shape_, collision_cfg_,
                                      path_cfg_.dt, &blocking_id)) {
            result.blocking_object_id = blocking_id;
            last_failure_reason_ = blocking_id >= 0
                ? PlanFailureReason::COLLISION_BLOCKED
                : PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            return false;
        }
        result.candidate_valid = true;
        result.valid_candidate_count = 1;
        result.selected_target_d = v2_lane_change_.target_d;
        result.selected_cost = 0.0;
        result.candidate_path = crawl;
        out_path = std::move(crawl);
        UpdateCommittedLaneChangeLifecycle(d, start.d_d, ego.yaw, result);
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    if (!blocked_by_request && v2_lane_change_.has_requested_target_d) {
        // COMMIT 이후에는 목표를 정확히 겨냥한다(AVOID의 tolerance=0과 동일한
        // 이유 - COMMIT의 의미는 "이 궤적을 실행한다"이므로 잠근 목표를
        // 그대로 겨냥해야 한다).
        const double target_d = lane_change_committed
            ? v2_lane_change_.target_d : v2_lane_change_.requested_target_d;

        PathGeneratorConfig generation_cfg = path_cfg_;
        if (lane_change_committed) {
            generation_cfg.lateral_target_tolerance = 0.0;
        }

        PlannerCommand side_cmd = command;
        side_cmd.lane_change_d_offset = target_d;
        side_cmd.target_speed = resolved_target_speed;

        PlannerDebugStats side_stats;
        auto generated = ResolveManeuver(
            start, side_cmd, ref_, generation_cfg, limits_, lane_width_,
            predicted_collision_obstacles, vehicle_shape_, collision_cfg_,
            &side_stats);
        EvaluateCosts(generated, cost_weights_, target_d);
        FilterByCartesianLateralAcceleration(generated, ref_,
                                             limits_.max_lateral_accel);

        result.generated_candidate_count = generated.size();
        int first_blocking_id = -1;
        for (const auto& path : generated) {
            if (path.valid) {
                ++result.valid_candidate_count;
            } else if (first_blocking_id < 0 &&
                       path.rejection_reason == RejectionReason::COLLISION &&
                       path.collision_object_id >= 0) {
                first_blocking_id = path.collision_object_id;
            }
        }

        const FrenetPath* best_candidate = SelectBestPath(generated);
        if (best_candidate) {
            result.candidate_valid = true;
            result.selected_target_d = best_candidate->d.empty()
                ? target_d : best_candidate->d.back();
            result.selected_cost = best_candidate->cost_total;
            result.candidate_path =
                ConvertToCartesianPath(*best_candidate, ref_);
        } else {
            result.blocking_object_id = first_blocking_id;
        }
    } else if (blocked_by_request) {
        result.blocking_object_id = -1;
    }

    if (lane_change_committed) {
        if (!result.candidate_valid) {
            last_failure_reason_ = result.blocking_object_id >= 0
                ? PlanFailureReason::COLLISION_BLOCKED
                : PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            return false;
        }
        out_path = result.candidate_path;
        UpdateCommittedLaneChangeLifecycle(d, start.d_d, ego.yaw, result);
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    // ---- 유지 궤적 --------------------------------------------------
    //
    // **LANE_CHANGE candidate를 여기 넣지 않는다.** PREPARE 중 횡이동 발행은
    // AVOID와 동일하게 계약 5.2절이 금지한다.
    PlannerCommand maintenance_base = command;
    maintenance_base.target_speed = resolved_target_speed;
    if (!BuildLaneChangeMaintenancePath(ego, start, maintenance_base,
                                        predicted_collision_obstacles,
                                        out_path)) {
        last_failure_reason_ = PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
        return false;
    }

    static int prepare_log_tick = 0;
    if (++prepare_log_tick % 5 == 0) {
        std::printf("[FrenetPlanner-STATE] mode=LANE_CHANGE phase=PREPARE "
                    "s=%.2f d=%.3f speed=%.2f candidate=%d target_d=%.3f "
                    "low_speed=%d behind=%d candidates=%zu/%zu "
                    "maintenance=%zu\n",
                    s, d, ego.v, result.candidate_valid ? 1 : 0,
                    result.selected_target_d,
                    result.low_speed_context ? 1 : 0,
                    result.target_behind_ego ? 1 : 0,
                    result.valid_candidate_count,
                    result.generated_candidate_count, out_path.x.size());
    }

    last_failure_reason_ = PlanFailureReason::NONE;
    return true;
}
