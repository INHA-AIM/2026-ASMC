/**
 * @file Avoid.cpp
 * @brief External/v2 AVOID orchestration: maintenance path priority and
 *        the main PREPARE/COMMITTED execution flow (AvoidProcess).
 *
 * R6-F1: target acquisition/candidate target_d selection moved to
 * AvoidTargetResolver.cpp, and commit/lifecycle/progress moved to
 * AvoidLifecycle.cpp - both still define FrenetPlanner member functions
 * declared in frenet_planner.hpp, only the .cpp location changed. This
 * file keeps the orchestration that reads AVOID phase/context, decides
 * low-speed vs normal-speed execution, evaluates candidates, calls the
 * lifecycle updater, and produces output.
 *
 * PrepareAvoid/PlanCommittedAvoid stay in frenet_planner.cpp - they
 * depend on the anonymous-namespace ValidateExternalCommand, whose
 * linkage this refactor does not expose.
 */

#include "Frenet/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <utility>

#include "Frenet/Evaluation/Cost.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Generator/CandidateCombiner.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Math/FrenetConverter.hpp"
#include "Frenet/Math/Utils.hpp"
#include "Frenet/Pipeline/TrajectoryRequest.hpp"

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
    const bool low_speed = std::fabs(ego.v) < limits_.low_speed_fallback_v;

    if (low_speed) {
        // crawl은 현재 d를 유지한다. PREPARE 중에 횡이동을 시작하면 안 된다.
        CartesianPath crawl;
        if (PlanLowSpeedFallback(ego, s, d, d, crawl, limits_.low_speed_crawl_duration_s,
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
        // R6-C: 이 유지 궤적은 순수 KEEP 요청이다(AVOID 고유 상태 불필요) -
        // request-native path를 직접 사용한다.
        TrajectoryRequest keep_request;
        keep_request.lateral.target_d = 0.0;
        keep_request.lateral.search_tolerance = 0.0;
        keep_request.longitudinal_mode = LongitudinalMode::VELOCITY_KEEPING;
        keep_request.target_speed = base_command.target_speed;
        PlannerDebugStats keep_stats;
        auto keep_candidates = GenerateRequestedCandidates(
            start, keep_request, ref_, path_cfg_, limits_,
            collision_obstacles, vehicle_shape_, collision_cfg_, &keep_stats);
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

bool FrenetPlanner::AvoidProcess(
    const CartesianState& ego, const FrenetState& start,
    double s, double d, double resolved_target_speed,
    const std::vector<TrackedObject>& predicted_collision_obstacles,
    const PlannerCommand& command, bool avoid_prepare, bool avoid_committed,
    CartesianPath& out_path, AvoidPrepareResult& result) {
    last_mode_ = AVOID;
    last_target_speed_ = resolved_target_speed;
    result.direction_conflict = v2_avoid_targets_.direction_conflict;

    // 상황 정보다. 상태 판단(READY/WAITING)에는 쓰지 않는다 - 그것은
    // candidate_valid만으로 정한다(헤더 주석 참고).
    result.low_speed_context = std::fabs(ego.v) < limits_.low_speed_fallback_v;
    result.ego_speed_mps = std::max(0.0, ego.v);

    // COMMIT 뒤에도 실제 관측 OBB와 tracker identity를 갱신한다. 정확한
    // 현재 ID를 우선하고, 그 ID가 사라졌을 때만 유일한 공간 일치 대상을
    // 같은 물체로 재연결한다.
    const TrackedObject* observed_target = nullptr;
    std::vector<TrackedObject> collision_obstacles = predicted_collision_obstacles;
    if (avoid_committed) {
        const auto exact = std::find_if(
            predicted_collision_obstacles.begin(),
            predicted_collision_obstacles.end(),
            [this](const TrackedObject& object) {
                return object.id == execution_->avoid.obstacle_id &&
                       object.width > 0.0 && object.length > 0.0;
            });
        if (exact != predicted_collision_obstacles.end()) {
            observed_target = &*exact;
        } else {
            observed_target = FindSpatialAvoidReplacement(
                s, predicted_collision_obstacles, v2_avoidance_);
        }
        if (observed_target != nullptr) {
            RefreshAvoidObservation(
                s, *observed_target, v2_avoidance_,
                /*refresh_committed_target=*/true);
        } else if (!v2_avoidance_.dynamic_target &&
                   v2_avoidance_.has_last_static_target &&
                   ++v2_avoidance_.static_target_missing_cycles<=
                       static_cast<unsigned>(avoid_cfg_.static_dropout_grace_cycles)) {
            // Keep the last authoritative static OBB in the collision set for
            // the dropout grace.  Otherwise a planner can accelerate through
            // the temporarily invisible target before the tracker reconnects.
            collision_obstacles.push_back(v2_avoidance_.last_static_target);
        }
    }

    // COMMIT 이후의 종방향 위치와 실제 d를 먼저 반영해야 이번 cycle의
    // 생성 목표가 TRACKING/SHIFT/PASS/RETURN 중 올바른 단계에 맞는다.
    if (avoid_committed) {
        if(v2_avoidance_.static_recovery_engaged&&execution_->avoid.phase==AvoidPhase::TRACKING)
            execution_.beginAvoidShiftProbe();
        UpdateCommittedAvoidLifecycle(s, d, ego.v, result);

        // 아직 회피를 시작하지 않은 TRACKING에서만 영구 소실을 실패로
        // 처리한다. SHIFT/PASS 중 관측 누락으로 KEEP에 급복귀하면 더 위험하다.
        const int grace_cycles=v2_avoidance_.dynamic_target
            ? avoid_cfg_.dropout_grace_cycles : avoid_cfg_.static_dropout_grace_cycles;
        if (!execution_.observeCommittedAvoidTarget(observed_target != nullptr,
                                                    grace_cycles)) {
            last_failure_reason_ = PlanFailureReason::TARGET_OBJECT_LOST;
            return false;
        }
    }

    if (avoid_road_map_.enabled()) {
        return MapAvoidProcess(ego, start, s, d, resolved_target_speed,
            collision_obstacles, command, avoid_prepare,
            avoid_committed, out_path, result);
    }

    const bool approach_phase = avoid_committed &&
        execution_->avoid.phase == AvoidPhase::TRACKING;
    const bool shift_phase = avoid_committed &&
        execution_->avoid.phase == AvoidPhase::SHIFT;
    const bool return_phase = avoid_committed &&
        execution_->avoid.phase == AvoidPhase::RETURN;
    const bool shortest_lateral_phase = shift_phase || return_phase;
    const double execution_target_d =
        (approach_phase || return_phase) ? 0.0 : v2_avoidance_.target_d;

    const double track_length =
        ref_.points.empty() ? 0.0 : ref_.points.back().s;
    const double committed_target_gap = avoid_committed
        ? ForwardArcDistance(s, v2_avoidance_.obstacle_s, track_length)
        : std::numeric_limits<double>::infinity();
    // LiDAR OBB의 종방향 길이가 커지거나 흔들려도 target 중심을 지난 뒤
    // d=0 복귀 후보가 전체 OBB 충돌검사를 통과하면 그 후보를 우선한다.
    // 통과하지 못하면 아래의 잠긴 회피 목표 후보를 그대로 사용하므로
    // 성급한 phase 변경 때문에 멈추지 않는다.
    const bool early_return_probe = avoid_committed &&
        (shift_phase || execution_->avoid.phase == AvoidPhase::PASS) &&
        std::isfinite(committed_target_gap) &&
        committed_target_gap < 0.0;

    // COMMIT 목표가 실제 충돌검사에서 막혔을 때 사용할 단방향 복구 목표.
    // 원래 목표를 항상 첫 번째로 검사하고, SHIFT/PASS에서만 장애물 반대편
    // 바깥쪽으로 확장한다. RETURN의 d=0이나 접근 단계에는 적용하지 않는다.
    std::vector<double> committed_targets;
    if (early_return_probe) committed_targets.push_back(0.0);
    committed_targets.push_back(execution_target_d);
    if (avoid_committed && (shift_phase ||
                            execution_->avoid.phase == AvoidPhase::PASS)) {
        double outward_sign = 0.0;
        const double side = v2_avoidance_.target_d - v2_avoidance_.obstacle_d;
        if (std::fabs(side) > 1e-6) {
            outward_sign = side > 0.0 ? 1.0 : -1.0;
        } else if (execution_->avoid.requested_direction ==
                   LateralDirection::LEFT) {
            outward_sign = 1.0;
        } else if (execution_->avoid.requested_direction ==
                   LateralDirection::RIGHT) {
            outward_sign = -1.0;
        } else {
            outward_sign = v2_avoidance_.target_d >= 0.0 ? 1.0 : -1.0;
        }

        const double step = std::max(0.05,
            avoid_cfg_.committed_recovery_step);
        const double max_expansion = std::max(0.0,
            avoid_cfg_.committed_recovery_max_expansion);
        for (double expansion = step;
             expansion <= max_expansion + 1e-9; expansion += step) {
            const double candidate =
                v2_avoidance_.target_d + outward_sign * expansion;
            if (candidate < path_cfg_.lateral_d1.min - 1e-9 ||
                candidate > path_cfg_.lateral_d1.max + 1e-9) {
                break;
            }
            committed_targets.push_back(candidate);
        }
    }

    // TRACKING에서도 실제 후보 구간은 target과 충돌 검사한다. 먼 target까지
    // 이어지는 가상 reactive coast만 아래 coast-exempt ID로 제외한다.
    const std::vector<TrackedObject>& phase_collision_obstacles =
        predicted_collision_obstacles;

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
    // 저속에서는 d(t)가 아니라 d(s)인 거리 기반 후보를 사용한다. 기존
    // 구현은 COMMIT 뒤 2초짜리 경로 하나만 만들어, 정지 출발 시 1~2 m
    // 전진한 지점에서 횡이동이 끝나기도 전에 충돌로 모두 탈락했다.
    // PREPARE/COMMIT 모두 여러 target_d와 여러 전환거리를 평가해 저속도
    // 정상적인 후보 탐색 상태로 만든다. margin recovery가 필요한 SHIFT가
    // 한 번 승인되면 생성기 종류만 SHIFT가 끝날 때까지 유지한다. 충돌
    // 판정 자체는 phase/속도와 무관하게 일반 후보와 동일한 공용 판정기를
    // 쓴다. 현재 속도만으로 생성기를 바꾸면 0.5 m/s 경계에서 출발/정지가
    // 반복될 수 있기 때문이다.
    const bool use_low_speed_candidate_pipeline =
        result.low_speed_context ||
        (avoid_committed && shortest_lateral_phase &&
         (std::fabs(ego.v) < limits_.near_low_speed_fallback_v ||
          v2_avoidance_.spatial_lateral_active ||
          v2_avoidance_.escape_recovery_active));
    if (use_low_speed_candidate_pipeline && !blocked_by_request) {
        std::vector<double> low_speed_targets;
        if (avoid_committed) {
            low_speed_targets = committed_targets;

            // 방향을 FSM이 지정하지 않은 회피라면, 잠긴 쪽이 새 장애물로
            // 완전히 막혔을 때 반대 corridor도 마지막 복구 후보로 평가한다.
            // 원래 잠긴 목표를 항상 먼저 평가하므로 유효한 실행 중 경로가
            // 매 cycle 좌우로 바뀌지는 않는다.
            if ((shift_phase || execution_->avoid.phase == AvoidPhase::PASS) &&
                execution_->avoid.requested_direction == LateralDirection::NONE) {
                const AvoidTargetPlan alternatives =
                    DecideAvoidTargets(v2_avoidance_, LateralDirection::NONE);
                for (int i = 0; i < alternatives.count; ++i) {
                    const double candidate = alternatives.candidates[i];
                    const bool duplicate = std::any_of(
                        low_speed_targets.begin(), low_speed_targets.end(),
                        [candidate](double value) {
                            return std::fabs(value - candidate) < 1e-6;
                        });
                    if (!duplicate &&
                        candidate >= path_cfg_.lateral_d1.min - 1e-9 &&
                        candidate <= path_cfg_.lateral_d1.max + 1e-9) {
                        low_speed_targets.push_back(candidate);
                    }
                }
            }
        } else {
            for (int i = 0; i < v2_avoid_targets_.count; ++i) {
                const double candidate = v2_avoid_targets_.candidates[i];
                if (candidate >= path_cfg_.lateral_d1.min - 1e-9 &&
                    candidate <= path_cfg_.lateral_d1.max + 1e-9) {
                    low_speed_targets.push_back(candidate);
                }
            }
        }

        const double scale_min = std::max(
            1.0, avoid_cfg_.low_speed_transition_scale_min);
        const double scale_max = std::max(
            scale_min, avoid_cfg_.low_speed_transition_scale_max);
        const double scale_step = std::max(
            0.05, avoid_cfg_.low_speed_transition_scale_step);
        const double path_extension = std::max(
            0.0, avoid_cfg_.low_speed_path_extension);

        struct LowSpeedCandidateEvaluation {
            CartesianPath path;
            double target_d = 0.0;
            double cost = std::numeric_limits<double>::infinity();
            bool early_return = false;
            bool collision_clear = false;
            bool margin_recovery_used = false;
            int blocking_id = -1;
        };
        std::vector<LowSpeedCandidateEvaluation> evaluations;
        for (std::size_t target_index = 0;
             target_index < low_speed_targets.size(); ++target_index) {
            const double target_d = low_speed_targets[target_index];
            const bool is_early_return = avoid_committed &&
                early_return_probe && target_index == 0;
            const double natural_distance =
                LowSpeedLateralTransitionDistance(d, target_d);
            for (double scale = scale_min;
                 scale <= scale_max + 1e-9; scale += scale_step) {
                const double transition_distance = natural_distance * scale;
                CartesianPath candidate;
                const bool generated = PlanLowSpeedFallback(
                    ego, s, d, target_d, candidate, 0.0,
                    resolved_target_speed, transition_distance,
                    transition_distance + path_extension);
                ++result.generated_candidate_count;

                bool kinematically_valid = generated;
                if (kinematically_valid) {
                    for (std::size_t i = 0; i < candidate.kappa.size(); ++i) {
                        const double kappa = std::fabs(candidate.kappa[i]);
                        const double speed = i < candidate.v.size()
                            ? std::fabs(candidate.v[i]) : 0.0;
                        if (!std::isfinite(kappa) ||
                            kappa > limits_.max_curvature + 1e-6 ||
                            speed * speed * kappa >
                                limits_.max_lateral_accel + 1e-6) {
                            kinematically_valid = false;
                            break;
                        }
                    }
                }

                int blocking_id = -1;
                bool margin_recovery_used = false;
                // AVOID lifecycle phase와 무관하다. 현재 pose가 target margin
                // 안에서 시작했는지는 공용 collision checker가 직접 판단한다.
                const int margin_recovery_object_id =
                    (avoid_prepare || avoid_committed)
                        ? execution_->avoid.obstacle_id : -1;
                const bool collision_clear = kinematically_valid &&
                    CartesianTrajectoryClear(
                        candidate, phase_collision_obstacles, vehicle_shape_,
                        collision_cfg_, path_cfg_.dt, &blocking_id,
                        margin_recovery_object_id,
                        &margin_recovery_used);

                double cost = transition_distance;
                if (avoid_committed) {
                    cost += static_cast<double>(target_index) *
                            avoid_cfg_.committed_target_index_cost_scale;
                } else {
                    const bool is_right =
                        target_d < v2_avoidance_.obstacle_d;
                    const bool preferred =
                        avoid_cfg_.prefer_right_when_equal
                            ? is_right : !is_right;
                    if (!preferred) cost += avoid_cfg_.direction_tiebreak_cost;
                }
                evaluations.push_back(LowSpeedCandidateEvaluation{
                    std::move(candidate), target_d, cost, is_early_return,
                    collision_clear, margin_recovery_used, blocking_id});
            }
        }

        CartesianPath selected;
        double selected_target = execution_target_d;
        double selected_cost = std::numeric_limits<double>::infinity();
        bool selected_from_recovery = false;
        bool selected_early_return = false;
        int first_blocking_id = -1;
        for (auto& evaluation : evaluations) {
            const bool clear = evaluation.collision_clear;
            const int blocking_id = evaluation.blocking_id;

            if (!evaluation.path.x.empty()) {
                last_candidate_paths_.push_back(evaluation.path);
                last_candidate_validity_.push_back(clear);
            }
            if (clear) {
                ++result.valid_candidate_count;
                // 잠긴 target 목록의 우선순위를 보존하면서, 같은 target
                // 안에서는 전환거리가 짧아 더 빨리 바깥으로 이격되는
                // 경로를 우선한다.
                if (evaluation.cost < selected_cost) {
                    selected = evaluation.path;
                    selected_target = evaluation.target_d;
                    selected_cost = evaluation.cost;
                    selected_from_recovery =
                        evaluation.margin_recovery_used;
                    selected_early_return = evaluation.early_return;
                }
            } else if (first_blocking_id < 0 && blocking_id >= 0) {
                first_blocking_id = blocking_id;
            }
        }

        if (selected.x.empty()) {
            result.blocking_object_id = first_blocking_id;
            // COMMITTED failure also reaches the single SHIFT probe below.
            // PREPARE 후보 없음은 terminal failure가 아니라 WAITING이다.
            // 아래 maintenance 생성 결과를 그대로 반환한다.
        } else {
            result.escape_recovery_required = selected_from_recovery;
            if (selected_from_recovery && avoid_committed && shift_phase) {
                v2_avoidance_.escape_recovery_active = true;
            }
            if (execution_.observeEarlyReturnCandidate(selected_early_return)) {
                v2_avoidance_.escape_recovery_active = false;
            }
            if (avoid_committed &&
                std::fabs(selected_target - v2_avoidance_.target_d) > 1e-9 &&
                !approach_phase && !return_phase && !selected_early_return) {
                v2_avoidance_.target_d = selected_target;
            }
            result.candidate_valid = true;
            result.spatial_candidate_selected = true;
            result.selected_target_d = selected_target;
            result.selected_cost = selected_cost;
            result.candidate_path = selected;
            if (avoid_committed) {
                if (shortest_lateral_phase) {
                    v2_avoidance_.spatial_lateral_active = true;
                }
                out_path = std::move(selected);
                UpdateCommittedAvoidLifecycle(s, d, ego.v, result,
                    CanHandoffAvoidToKeep(ego,s,d,predicted_collision_obstacles));
                last_failure_reason_ = PlanFailureReason::NONE;
                return true;
            }
        }
    }

    // 저속이라고 후보 생성을 건너뛰지 않는다. "저속에서는 어차피 안 될
    // 것"이라고 가정하면, 실제로 가능한 상황에서도 READY에 도달하지
    // 못한다. 생성을 시도하고 결과로 판단한다 - 저속에서 후보가 0개면
    // low_speed_context이 그 사유를 설명한다.
    if (!use_low_speed_candidate_pipeline && !blocked_by_request &&
        v2_avoid_targets_.count > 0) {
        const FrenetPath* best_candidate = nullptr;
        std::vector<FrenetPath> best_set;
        double best_target_d = 0.0;
        bool best_early_return = false;
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
        // R6-C: search_tolerance는 legacy lateral_target_tolerance override와
        // 동일한 규칙(committed면 0, 그렇지 않으면 전역 tolerance)을 그대로
        // 옮긴다 - PathGeneratorConfig를 복제해 override하는 대신 request가
        // 그 값을 직접 담는다.
        const double search_tolerance =
            execution_->avoid.committed ? 0.0 : path_cfg_.lateral_target_tolerance;

        // wire command의 논리 ID가 아니라 공간 재연결까지 반영한 현재 관측
        // ID를 충돌 스코핑에 사용한다.
        const int margin_recovery_object_id = execution_->avoid.obstacle_id;

        const int target_count = avoid_committed
            ? static_cast<int>(committed_targets.size())
            : v2_avoid_targets_.count;
        for (int index = 0; index < target_count; ++index) {
            const double target_d = avoid_committed
                ? committed_targets[index] : v2_avoid_targets_.candidates[index];
            const bool is_early_return = avoid_committed &&
                early_return_probe && index == 0;

            TrajectoryRequest request;
            request.lateral.target_d = target_d;
            request.lateral.search_tolerance = search_tolerance;
            request.longitudinal_mode = LongitudinalMode::VELOCITY_KEEPING;
            request.target_speed = resolved_target_speed;
            request.collision.margin_recovery_object_id =
                margin_recovery_object_id;
            if (approach_phase) {
                request.collision.coast_exempt_object_id =
                    margin_recovery_object_id;
            }

            PlannerDebugStats side_stats;
            auto generated = GenerateRequestedCandidates(
                start, request, ref_, path_cfg_, limits_,
                phase_collision_obstacles, vehicle_shape_, collision_cfg_,
                &side_stats);
            EvaluateCosts(generated, cost_weights_, target_d);
            // 공용 횡가속도 생성 필터와 같은 설정 한계(3.0 m/s^2)를 사용한다.
            // 과거 AVOID에만 중복 적용되던 80% 축소는 제거한다.
            FilterByCartesianLateralAcceleration(generated, ref_,
                                                 limits_.max_lateral_accel);

            result.generated_candidate_count += generated.size();
            for (const auto& path : generated) {
                // KEEP과 동일한 dashboard/RViz 계약: 선택 여부와 무관하게
                // 생성된 모든 후보를 Cartesian debug snapshot에 보존한다.
                // invalid 후보도 보존해야 곡률/충돌 탈락 분포를 볼 수 있다.
                if (!path.s.empty() && path.s.size() == path.d.size()) {
                    last_candidate_paths_.push_back(
                        ConvertToCartesianPath(path, ref_));
                    last_candidate_validity_.push_back(path.valid);
                }
                if (path.valid) ++result.valid_candidate_count;
                else if (first_blocking_id < 0 &&
                         path.rejection_reason == RejectionReason::COLLISION &&
                         path.collision_object_id >= 0) {
                    first_blocking_id = path.collision_object_id;
                }
            }

            const FrenetPath* side_best = nullptr;
            if (shortest_lateral_phase) {
                // 실제 횡이동 단계는 안전 집합 안에서 완료시간을 우선하고,
                // 같은 시간에서만 기존 비용으로 부드러운 후보를 고른다.
                side_best = SelectShortestValidPath(generated);
            } else {
                side_best = SelectBestPath(generated);
            }
            if (!side_best) continue;
            // NONE이면 cost로 좌/우를 고른다. LEFT/RIGHT는 count가 1이라
            // 이 비교가 일어나지 않는다.
            if (!best_candidate ||
                side_best->cost_total < best_candidate->cost_total) {
                best_set = std::move(generated);
                // side_best는 move 전 vector를 가리키므로 저장 후 반드시 다시
                // 찾는다. 여기서 공용 cost selector를 호출하면 위에서 고른
                // RETURN 최단시간 후보가 조용히 긴 후보로 덮어써진다.
                best_candidate = shortest_lateral_phase
                    ? SelectShortestValidPath(best_set)
                    : SelectBestPath(best_set);
                best_target_d = target_d;
                best_early_return = is_early_return;
            }
            // COMMIT 복구는 가장 가까운 바깥쪽 유효 목표를 사용한다.
            // 더 먼 목표가 비용상 유리하더라도 불필요하게 회피 폭을 키우지 않는다.
            if (avoid_committed && best_candidate) break;
        }

        if (best_candidate) {
            const double endpoint_d = best_candidate->d.empty()
                ? best_target_d : best_candidate->d.back();
            // 저속 공용 generator는 물리적으로 불가능한 제자리 횡이동을
            // 피하려고 current d 유지 후보만 만든다. 그 후보를 실제 AVOID
            // 목표로 오인해 d=0을 COMMIT하지 않도록 목표 도달 여부를 확인한다.
            const bool reaches_avoid_target =
                std::fabs(endpoint_d - best_target_d) <=
                path_cfg_.lateral_target_tolerance + 1e-6;
            result.candidate_valid = reaches_avoid_target;
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
            result.selected_target_d = endpoint_d;
            result.selected_cost = best_candidate->cost_total;
            result.escape_recovery_required =
                best_candidate->margin_recovery_used;
            result.candidate_path =
                ConvertToCartesianPath(*best_candidate, ref_);
            if (!reaches_avoid_target) {
                result.selected_cost = -1.0;
                result.candidate_path = CartesianPath{};
            } else if (execution_.observeEarlyReturnCandidate(best_early_return)) {
                v2_avoidance_.escape_recovery_active = false;
            } else if (avoid_committed && !approach_phase && !return_phase &&
                       std::fabs(endpoint_d - v2_avoidance_.target_d) > 1e-9) {
                v2_avoidance_.target_d = endpoint_d;
            }
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
            // A centreline-only approach can be invalid even though the
            // committed lateral corridor is clear. Do not repeatedly brake
            // toward a shrinking entry distance: test SHIFT once, using the
            // same observed ego/OBBs and all normal safety filters. The preview
            // bounds this promotion; unrelated blocking objects do not trigger
            // it. PREPARE never takes this branch.
            const double preview_entry = std::max(
                v2_avoidance_.shift_entry_distance,
                EffectiveAvoidShiftStartDistance(
                    std::max(std::max(0.0, ego.v), resolved_target_speed), d));
            const bool target_blocks_approach =
                result.blocking_object_id == execution_->avoid.obstacle_id;
            const bool approach_generation_failed =
                last_failure_reason_ ==
                    PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            if (approach_phase && std::isfinite(committed_target_gap) &&
                committed_target_gap > 0.0 &&
                committed_target_gap <= preview_entry &&
                (target_blocks_approach || approach_generation_failed)) {
                const AvoidanceContext saved_context = v2_avoidance_;
                const auto saved_lifecycle = execution_->avoid;
                const PlanFailureReason saved_failure = last_failure_reason_;
                execution_.beginAvoidShiftProbe();
                v2_avoidance_.start_d = d;
                CartesianPath shift_path;
                AvoidPrepareResult shift_result;
                // Recursion is bounded to one probe: its phase is SHIFT, so
                // approach_phase is false in the nested invocation.
                if (AvoidProcess(ego, start, s, d, resolved_target_speed,
                                 predicted_collision_obstacles, command,
                                 false, true, shift_path, shift_result)) {
                    shift_result.generated_candidate_count +=
                        result.generated_candidate_count;
                    shift_result.valid_candidate_count +=
                        result.valid_candidate_count;
                    result = std::move(shift_result);
                    out_path = std::move(shift_path);
                    return true;
                }
                // Failed SHIFT must not unlock or advance the transaction.
                v2_avoidance_ = saved_context;
                execution_.restoreAvoid(saved_lifecycle);
                last_failure_reason_ = saved_failure;
            }
            return false;
        }
        out_path = result.candidate_path;
        UpdateCommittedAvoidLifecycle(s, d, ego.v, result,
                    CanHandoffAvoidToKeep(ego,s,d,predicted_collision_obstacles));
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
                    s, d, ego.v, execution_->avoid.obstacle_id,
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

