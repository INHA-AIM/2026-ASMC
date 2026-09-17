/**
 * @file LaneChange.cpp
 * @brief External/v2 LANE_CHANGE target selection, commit state, lifecycle,
 *        maintenance path, PREPARE/COMMITTED execution.
 *
 * Maneuver/Avoid.cpp의 PREPARE -> READY -> EXECUTE -> COMMITTED -> EXECUTING
 * 패턴을 재사용한다. 차이점:
 *   - 회피 대상 객체가 없다.
 *   - 좌/우 후보 경쟁이 없다 - mode(LANE_CHANGE_LEFT/RIGHT)가 이미 방향을
 *     정하므로 평가할 후보는 항상 하나뿐이다.
 *   - 실행 목표(target_d)는 FSM/MGeo가 만든 target_pose_map을 현재 RefLine에
 *     투영한 값이다(node가 이미 방향/헤딩 검증까지 끝내고
 *     command.target_pose_projection_*로 넘긴다) - Behavior가 이미 Link
 *     centerline에서 뽑은 실제 목표 차선 위치를 core가 다시 근사하지 않는다.
 *   - 목표 도달 후 COMPLETED로 보고하지 않는다. HOLD_TARGET_LANE으로 전환해
 *     잠긴 target_d를 유지한 채 계속 전진한다 - 아직 참조선을 새 차선으로
 *     바꿀 방법이 없으므로 d=0으로 복귀하면 오히려 원래 차선으로 돌아간다.
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

bool FrenetPlanner::PrepareExternalLaneChangeTarget(
    double ego_s, double ego_d, const PlannerCommand& command,
    bool lane_change_prepare, bool lane_change_committed) {
    if (lane_change_committed) {
        // COMMIT 이후 geometry의 소유권은 잠긴 context에 있다. target_d를
        // 다시 캡처하지 않는다(AVOID의 committed 분기와 동일한 이유) - 새로
        // 들어오는 target_pose_projection_*이 있어도 이미 진행 중인
        // transaction의 목표를 바꾸지 않는다.
        return true;
    }

    if (!lane_change_prepare) {
        // NORMAL purpose로 LANE_CHANGE가 들어오면 아직 실행 경로가 없다.
        // 그대로 내려보내면 ResolveManeuver의 LANE_CHANGE 분기가
        // lane_change_d_offset=0으로 중앙 유지 궤적을 만들어, FSM 입장에서는
        // "차선변경 성공"으로 보이면서 실제로는 차선을 바꾸지 않는다
        // (AVOID의 동일 분기와 같은 원칙).
        last_mode_ = command.mode;
        last_failure_reason_ = PlanFailureReason::UNSUPPORTED_BEHAVIOR;
        return false;
    }

    // has_target_pose_projection=false는 node가 이미 걸러야 정상이지만
    // (target_pose_map 투영/heading/direction 검증 실패는 PREPARE 진입 전에
    // 실패로 수렴한다), core도 방어적으로 같은 결과로 수렴시킨다 - 진행
    // 중이 아닌 target으로 candidate를 만들지 않는다.
    if (!command.has_target_pose_projection ||
        !std::isfinite(command.target_pose_projection_d)) {
        last_mode_ = command.mode;
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // 이 transaction에서 처음 PREPARE를 관측하는 사이클에만 target_d를
    // 캡처한다. active가 이미 true이면(같은 command_id로 반복되는 PREPARE)
    // 다시 캡처하지 않는다 - PREPARE 동안 매 사이클 재투영된 값으로 흔들리지
    // 않고, "이 transaction이 겨냥할 차선"이라는 의미를 유지한다.
    if (!v2_lane_change_.active) {
        v2_lane_change_.direction = command.mode == LANE_CHANGE_LEFT
            ? LateralDirection::LEFT
            : LateralDirection::RIGHT;
        v2_lane_change_.start_d = ego_d;
        // 실행 목표: Behavior/MGeo가 만든 target_pose_map을 node가 현재
        // RefLine에 투영하고 방향/헤딩 sanity 검증까지 끝낸 값이다
        // (PlannerCommand.hpp의 target_pose_projection_* 주석 참고) - core는
        // 그 값을 그대로 신뢰한다. 이 이상 별도로 lane_width_ 기반 근사를
        // 더하지 않는다.
        v2_lane_change_.target_d = command.target_pose_projection_d;
        v2_lane_change_.phase = LaneChangePhase::SHIFT;
        v2_lane_change_.active = true;
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

    // 우선순위(BuildAvoidMaintenancePath와 동일한 원칙):
    //   1. 충돌 검사를 통과한 현재 차선 유지(KEEP) 궤적 - LC candidate가
    //      아직 unsafe라는 이유만으로 감속/정지하지 않는다.
    //   2. 종방향 감속
    //   3. 0속도 hold
    const bool low_speed = std::fabs(ego.v) < limits_.low_speed_fallback_v;

    if (low_speed) {
        CartesianPath crawl;
        if (PlanLowSpeedFallback(ego, s, d, d, crawl, limits_.low_speed_crawl_duration_s,
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
                            collision_cfg_, stop_cfg_, merge_cfg_, &keep_stats);
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

    result.low_speed_context = std::fabs(ego.v) < limits_.low_speed_fallback_v;

    // HOLD_TARGET_LANE에서도 매 cycle 잠긴 target_d로 계속 replan한다 -
    // 정지 상태가 아니다(frenet_planner.hpp의 LaneChangePhase 주석 참고).
    // PREPARE/SHIFT/HOLD 모두 아래에서 동일하게 tolerance=0으로 정확한
    // target_d를 겨냥하므로, 이 phase 값 자체로 생성 경로를 분기할 필요가
    // 없다.

    // 정상 후보 생성기가 정지 근처에서 횡 후보를 만들지 못하는 경우
    // COMMIT 이후에는 잠긴 target_d를 향하는 crawl로 대체한다(AVOID와 동일).
    if (lane_change_committed && result.low_speed_context) {
        result.generated_candidate_count = 1;
        CartesianPath crawl;
        PlanLowSpeedFallback(ego, s, d, v2_lane_change_.target_d, crawl, limits_.low_speed_crawl_duration_s,
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
        UpdateCommittedLaneChangeLifecycle(s, d, start.d_d, ego.yaw, result);
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    // 목표 d: COMMIT 전에는 이번 사이클에 캡처/유지 중인 투영 target,
    // COMMIT 후에는 잠긴 target_d. 둘 다 v2_lane_change_.target_d에 있다 -
    // PrepareExternalLaneChangeTarget이 이미 채워 놓았다.
    const double target_d = v2_lane_change_.target_d;

    // LANE_CHANGE는 PREPARE 단계부터 항상 정확한 목표를 겨냥한다 - AVOID와
    // 달리 좌/우 후보 경쟁이 없고(mode가 이미 방향을 정한다), 목표 자체가
    // Behavior/MGeo가 이미 확정한 인접 차로 중심이다(target_pose_projection_d).
    // 전역 lateral_target_tolerance(기본 0.5m)를 그대로 두면 PREPARE candidate가
    // target_d 주변 ±tolerance 범위에서 cost로 하나를 고르게 되어, "어디로 갈지"
    // 를 Frenet이 다시 정하는 셈이 된다(Behavior가 이미 정한 값을 core가
    // 재해석) - PREPARE에서 선택한 값이 COMMIT에서 그대로 잠기므로, PREPARE도
    // COMMIT/HOLD와 같은 정확도로 겨냥해야 둘 사이에 값이 갈리지 않는다.
    PathGeneratorConfig generation_cfg = path_cfg_;
    generation_cfg.lateral_target_tolerance = 0.0;

    PlannerCommand side_cmd = command;
    side_cmd.lane_change_d_offset = target_d;
    side_cmd.target_speed = resolved_target_speed;

    PlannerDebugStats side_stats;
    auto generated = ResolveManeuver(
        start, side_cmd, ref_, generation_cfg, limits_, lane_width_,
        predicted_collision_obstacles, vehicle_shape_, collision_cfg_,
        stop_cfg_, merge_cfg_, &side_stats);
    EvaluateCosts(generated, cost_weights_, target_d);
    FilterByCartesianLateralAcceleration(generated, ref_,
                                         limits_.max_lateral_accel);

    // 저속(ego.v < kLowSpeedFallbackV)에서는 GenerateLateralCandidates가
    // lateral_d1 격자 자체를 무시하고 현재 d/차선 중앙 후보만 만든다(수학적
    // 특이점 회피, 헤더 주석 참고) - 위에서 겨냥한 target_d와 무관한 후보가
    // 나올 수 있다는 뜻이다. LANE_CHANGE candidate_valid는 "요청한 target_d에
    // 실제로 도달하는 후보가 있다"를 뜻해야 하므로, 종단 d가 target_d와
    // 정확히 일치하는지가 유효성의 일부다 - 카운팅(valid_candidate_count)과
    // 최종 선택(best_candidate 채택)이 서로 다른 기준을 쓰면 "valid=0인데
    // candidate_valid=true" 같은 모순된 진단이 나올 수 있으므로 같은 술어를
    // 공유한다.
    constexpr double kExactTargetEpsilon = 1e-6;
    const auto is_exact_lane_change_candidate = [target_d](
        const FrenetPath& path) {
        return path.valid && !path.d.empty() &&
               std::fabs(path.d.back() - target_d) <= kExactTargetEpsilon;
    };

    result.generated_candidate_count = generated.size();
    int first_blocking_id = -1;
    for (const auto& path : generated) {
        if (is_exact_lane_change_candidate(path)) {
            ++result.valid_candidate_count;
        } else if (!path.valid && first_blocking_id < 0 &&
                   path.rejection_reason == RejectionReason::COLLISION &&
                   path.collision_object_id >= 0) {
            first_blocking_id = path.collision_object_id;
        }
    }

    const FrenetPath* best_candidate = SelectBestPath(generated);
    if (best_candidate && !is_exact_lane_change_candidate(*best_candidate)) {
        best_candidate = nullptr;
    }
    if (best_candidate) {
        result.candidate_valid = true;
        result.selected_target_d = best_candidate->d.empty()
            ? target_d : best_candidate->d.back();
        result.selected_cost = best_candidate->cost_total;
        result.candidate_path = ConvertToCartesianPath(*best_candidate, ref_);
    } else {
        result.blocking_object_id = first_blocking_id;
    }

    if (lane_change_committed) {
        if (!result.candidate_valid) {
            last_failure_reason_ = result.blocking_object_id >= 0
                ? PlanFailureReason::COLLISION_BLOCKED
                : PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            return false;
        }
        out_path = result.candidate_path;
        UpdateCommittedLaneChangeLifecycle(s, d, start.d_d, ego.yaw, result);
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    // ---- 유지 궤적 --------------------------------------------------
    //
    // **LANE_CHANGE candidate를 여기 넣지 않는다.** PREPARE 중 횡이동
    // 발행은 FSM의 EXECUTE 전에 실제 이동이 시작되는 것을 막기 위해
    // 금지한다.
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
                    "low_speed=%d candidates=%zu/%zu maintenance=%zu\n",
                    s, d, ego.v, result.candidate_valid ? 1 : 0,
                    result.selected_target_d,
                    result.low_speed_context ? 1 : 0,
                    result.valid_candidate_count,
                    result.generated_candidate_count, out_path.x.size());
    }

    last_failure_reason_ = PlanFailureReason::NONE;
    return true;
}
