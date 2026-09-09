#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_map>

#include "frenet_planner/Math/Utils.hpp"
#include "frenet_planner/Speed/CurveSpeed.hpp"
#include "frenet_planner/Speed/SpeedPolicy.hpp"

bool FrenetPlanner::Init(const std::string& yaml_path) {
    std::string waypoint_file;
    LoadParams(yaml_path, path_cfg_, limits_, cost_weights_, vehicle_shape_,
               collision_cfg_, curve_speed_cfg_, following_cfg_, avoid_cfg_,
               stop_cfg_, emergency_stop_cfg_, merge_cfg_, highway_merge_cfg_,
               visualization_cfg_,
               wheelbase_, lane_width_, replanning_period_s_,
               max_target_speed_accel_, waypoint_file);

    if (!waypoint_file.empty() && waypoint_file.front() != '/') {
        waypoint_file = std::string(FRENET_PACKAGE_SRC_DIR) + "/" + waypoint_file;
    }

    debug_writer_ = std::make_unique<PlannerDebugWriter>(visualization_cfg_);

    if (waypoint_file.empty() || !LoadReferenceLine(waypoint_file, ref_, limits_.max_curvature)) {
        std::printf("[FrenetPlanner] Reference line load failed (waypoint_file='%s')\n",
                    waypoint_file.c_str());
        return false;
    }
    if (merge_cfg_.conflict_s < 0.0) {
        std::printf("[FrenetPlanner] Roundabout MERGE disabled: set "
                    "planner.merge.conflict_s in params.yaml\n");
    } else {
        const RefPoint conflict = Interpolate(ref_, merge_cfg_.conflict_s);
        std::printf("[FrenetPlanner] Roundabout MERGE conflict: s=%.2f pos=(%.2f, %.2f)\n",
                    merge_cfg_.conflict_s, conflict.x, conflict.y);
    }
    if (!highway_merge_cfg_.enabled || highway_merge_cfg_.zones.empty()) {
        std::printf("[FrenetPlanner] Highway MERGE disabled: measure start/conflict/"
                    "completion_s, then set planner.highway_merge.enabled=true\n");
    } else {
        for (const auto& zone : highway_merge_cfg_.zones) {
            bool valid_zone = zone.start_s >= 0.0 && !zone.checkpoints.empty();
            double previous_clear_s = zone.start_s;
            for (const auto& checkpoint : zone.checkpoints) {
                valid_zone = valid_zone && checkpoint.conflict_s > previous_clear_s &&
                    checkpoint.clear_s > checkpoint.conflict_s;
                previous_clear_s = checkpoint.clear_s;
            }
            valid_zone = valid_zone && zone.completion_s >= previous_clear_s;
            if (!valid_zone) {
                std::printf("[FrenetPlanner] Highway MERGE zone '%s' invalid s ordering; "
                            "policy disabled\n", zone.name.c_str());
                highway_merge_cfg_.enabled = false;
                break;
            }
            std::printf("[FrenetPlanner] Highway MERGE '%s': start=%.2f complete=%.2f "
                        "checkpoints=%zu\n", zone.name.c_str(), zone.start_s,
                        zone.completion_s, zone.checkpoints.size());
            for (size_t i = 0; i < zone.checkpoints.size(); ++i) {
                const auto& checkpoint = zone.checkpoints[i];
                const RefPoint conflict = Interpolate(ref_, checkpoint.conflict_s);
                std::printf("  checkpoint[%zu]: conflict=%.2f clear=%.2f pos=(%.2f, %.2f)\n",
                            i, checkpoint.conflict_s, checkpoint.clear_s,
                            conflict.x, conflict.y);
            }
        }
    }
    ref_loaded_ = true;
    return true;
}

bool FrenetPlanner::GenerateDecelToZeroPath(double s, double d, double s_dot,
                                            CartesianPath& out_path) const {
    // D4-3에서 STOP BRAKING fail-safe에서 추출했다. 두 경로가 정확히 같은
    // 감속 프로파일을 쓰도록 공유한다 - 복사해두면 한쪽만 고쳐지는 문제가
    // 생긴다.
    const double decel = std::max(
        0.1, last_mode_ == EMERGENCY ? emergency_stop_cfg_.decel
                                     : stop_cfg_.comfortable_decel);
    const double v0 = std::max(0.0, s_dot);
    const int n = static_cast<int>(std::ceil(v0 / decel / path_cfg_.dt)) + 1;
    out_path.x.assign(n, 0.0); out_path.y.assign(n, 0.0);
    out_path.yaw.assign(n, 0.0); out_path.kappa.assign(n, 0.0);
    out_path.v.assign(n, 0.0); out_path.a.assign(n, 0.0);
    double s_i = s;
    double v_prev = v0;
    for (int i = 0; i < n; ++i) {
        const double t = i * path_cfg_.dt;
        const double v_i = std::max(0.0, v0 - decel * t);
        if (i > 0) s_i += 0.5 * (v_i + v_prev) * path_cfg_.dt;
        v_prev = v_i;
        const RefPoint rp = Interpolate(ref_, s_i);
        // 정지 직전 구간이라 횡방향은 현재 오프셋을 그대로 유지한다.
        const CartesianState cs =
            FrenetToCartesian(rp, s_i, v_i, v_i > 0.0 ? -decel : 0.0,
                              d, 0.0, 0.0);
        out_path.x[i] = cs.x;
        out_path.y[i] = cs.y;
        out_path.yaw[i] = cs.yaw;
        out_path.kappa[i] = cs.kappa;
        out_path.v[i] = v_i;
        out_path.a[i] = v_i > 0.0 ? -decel : 0.0;
    }
    return n > 0;
}

bool FrenetPlanner::PlanLowSpeedFallback(const CartesianState& ego, double s, double d,
                                          double target_d,
                                          CartesianPath& out_path, double duration_s,
                                          double speed_cap) const {
    constexpr double kRecenterDistM = 15.0;   // [m] d를 0으로 되돌리는 데 쓰는 이동거리
    const int n = static_cast<int>(duration_s / path_cfg_.dt) + 1;
    out_path.x.resize(n); out_path.y.resize(n); out_path.yaw.resize(n);
    out_path.kappa.resize(n); out_path.v.resize(n); out_path.a.resize(n);

    double s_i = s;
    double v_prev = std::max(ego.v, kMinCreepSpeed);
    for (int i = 0; i < n; ++i) {
        const double t = i * path_cfg_.dt;
        const double crawl_speed = clip(
            ego.v + kCrawlAccel * t, kMinCreepSpeed, kCrawlSpeedCap);
        const double v_i = std::min(crawl_speed, speed_cap);
        if (i > 0) s_i += 0.5 * (v_i + v_prev) * path_cfg_.dt;
        v_prev = v_i;

        const double ds = s_i - s;
        const double frac = clip(ds / kRecenterDistM, 0.0, 1.0);
        // smoothstep(frac) = 0.5*(1+cos(pi*frac)): frac=0 -> d, frac=1 -> 0, 양끝 기울기 0.
        const double d_error = d - target_d;
        const double d_i      = target_d + d_error * 0.5 * (1.0 + std::cos(M_PI * frac));
        const double d_prime  = (ds >= kRecenterDistM) ? 0.0
                               : d_error * (-0.5 * M_PI / kRecenterDistM) * std::sin(M_PI * frac);
        const double d_pprime = (ds >= kRecenterDistM) ? 0.0
                               : d_error * (-0.5 * M_PI * M_PI / (kRecenterDistM * kRecenterDistM))
                                     * std::cos(M_PI * frac);

        RefPoint rp = Interpolate(ref_, s_i);
        const double a_i = (v_i < kCrawlSpeedCap) ? kCrawlAccel : 0.0;
        CartesianState cs = FrenetToCartesian(rp, s_i, v_i, a_i, d_i, d_prime, d_pprime);

        out_path.x[i] = cs.x;
        out_path.y[i] = cs.y;
        out_path.yaw[i] = cs.yaw;
        out_path.kappa[i] = cs.kappa;
        out_path.v[i] = v_i;
        out_path.a[i] = a_i;
    }
    return true;
}

// Cartesian 정지 목표 -> reference line 종방향 좌표.
bool FrenetPlanner::ProjectPointToFrenet(double x, double y, double s_hint,
                                         double search_window,
                                         double max_lateral_distance,
                                         double& out_s, double& out_d) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    // 창과 허용 거리는 호출자가 정한다. 잘못된 값으로 조용히 통과시키면
    // 투영 방어가 사실상 사라진다.
    if (!std::isfinite(search_window) || search_window <= 0.0) return false;
    if (!std::isfinite(max_lateral_distance) || max_lateral_distance <= 0.0) {
        return false;
    }

    // 탐색창은 ego localization용 기본값(30m)이 아니라 호출자가 준 값을 쓴다.
    // 목표점은 ego보다 앞에 있어 기본 창을 벗어나고, 그러면 잘린 초기
    // 추정값에서 출발한 Newton이 트랙의 다른 구간으로 발산한다.
    const double* hint = std::isfinite(s_hint) ? &s_hint : nullptr;
    const double s = FindClosestS(ref_, x, y, hint, search_window);
    if (!std::isfinite(s)) return false;

    // 최근접 투영은 목표가 경로에서 아무리 멀어도 어떤 s든 돌려준다. 실제로
    // 그 s가 목표를 대표하는지 확인하지 않으면, 다른 도로 위의 목표점이
    // 엉뚱한 위치의 명령으로 둔갑한다.
    const RefPoint rp = Interpolate(ref_, s);
    const double lateral_error = std::hypot(x - rp.x, y - rp.y);
    if (!std::isfinite(lateral_error) ||
        lateral_error > max_lateral_distance) {
        return false;
    }

    // d의 부호 규약은 CartesianToFrenet 식 (4)와 같아야 한다. 다른 식을 쓰면
    // 같은 점이 경로에 따라 좌/우가 반대로 나온다.
    //   d = (x - r(s))^T n_r(s),  n_r = (-sin(theta_r), cos(theta_r))
    // 즉 진행 방향 기준 좌측이 양수다.
    const double d = (x - rp.x) * (-std::sin(rp.theta)) +
                     (y - rp.y) * std::cos(rp.theta);
    if (!std::isfinite(d)) return false;

    out_s = s;
    out_d = d;
    return true;
}

bool FrenetPlanner::ProjectPointToS(double x, double y, double s_hint,
                                    double& out_s) const {
    // STOP 전용 wrapper. 탐색창과 허용 횡거리를 STOP config로 고정하고 d는
    // 버린다. v1 STOP도 이 함수를 쓰고 있으므로 동작을 바꾸지 않는다
    // (ProjectPointToFrenet 헤더 주석 참고).
    double s = 0.0;
    double ignored_d = 0.0;
    if (!ProjectPointToFrenet(x, y, s_hint,
                              stop_cfg_.projection_search_window,
                              stop_cfg_.max_lateral_offset, s, ignored_d)) {
        return false;
    }
    out_s = s;
    return true;
}

bool FrenetPlanner::ReferenceHeadingAt(double s, double& out_heading) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(s)) return false;
    // topology(wrap 폐루프 vs clamp 열린 경로) 정규화는 Interpolate가 유일한
    // owner다(Commit F, "one policy, one authoritative owner") - 여기서
    // s_max로 다시 fmod하지 않는다.
    const RefPoint rp = Interpolate(ref_, s);
    if (!std::isfinite(rp.theta)) return false;
    out_heading = rp.theta;
    return true;
}

bool FrenetPlanner::ProjectEgoToS(const CartesianState& ego, double& out_s) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(ego.x) || !std::isfinite(ego.y)) return false;
    const double s = FindClosestS(ref_, ego.x, ego.y,
                                  has_last_s_ ? &last_s_ : nullptr);
    if (!std::isfinite(s)) return false;
    out_s = s;
    return true;
}

namespace {

// 외부 command로 실제 열려 있는 모드. MERGE/LANE_CHANGE는 FSM 계약이
// 확정될 때까지 production 진입점에서 허용하지 않는다. 새 모드를 열 때는
// 여기에 case를 추가하고 ValidateExternalCommand에 필드 검증을 같이 넣는다.
bool IsSupportedExternalMode(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING:
        case STOP:
        case EMERGENCY:
        case FOLLOWING:
        // D6-1: AVOID는 대상 확보(resolver)까지만 열렸다. 후보 생성은 D6-2다.
        // 여기서 열지 않으면 resolver를 검증할 수 없고, 열어도 아래
        // ResolveManeuver 직전에서 UNSUPPORTED_BEHAVIOR로 막으므로
        // 조용히 velocity keeping 궤적이 나가지는 않는다.
        //
        // node의 SetImplementedManeuver(AVOID)는 계속 닫혀 있으므로 ROS
        // 경로로는 여기까지 도달하지 않는다(계약 12.7.1절).
        case AVOID:
            return true;

        default:
            return false;
    }
}

bool IsValidOptionalSpeed(bool has_value, double value) {
    if (!has_value) return true;
    return std::isfinite(value) && value >= 0.0;
}

// 모드 whitelist + 모드별 필수 필드 검증. 실패 시 reason으로 원인을 구분해
// 호출자가 "즉시 정지"와 "이전 궤적 재사용"을 나눌 수 있게 한다.
bool ValidateExternalCommand(const PlannerCommand& command,
                             FrenetPlanner::PlanFailureReason& reason) {
    if (!IsSupportedExternalMode(command.mode)) {
        reason = FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR;
        return false;
    }

    // 전 모드 공통: 외부가 지정한 속도는 유한하고 음수가 아니어야 한다.
    if (!IsValidOptionalSpeed(command.has_target_speed, command.target_speed) ||
        !IsValidOptionalSpeed(command.has_max_speed, command.max_speed)) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // EMERGENCY는 stop_position을 요구하지 않는다. BehaviorCommand가 목표점
    // 없이(has_target_pose=false) 즉시 긴급정지를 지시하므로 정지 위치 계산
    // 소유권이 Frenet에 있다(계약 8.1절). 아래 STOP 검증을 EMERGENCY에
    // 적용하면 정상 긴급정지가 INVALID_COMMAND로 거부된다.

    // FOLLOWING: 추종 대상이 지정되어야 한다. v2에서 대상 선택 권한은 FSM에
    // 있으므로(BehaviorCommand.primary_object_id) planner가 스스로 고르지
    // 않는다. -1이면 무엇을 따라갈지 알 수 없다.
    if (command.mode == FOLLOWING && command.target_object_id < 0) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // AVOID: 회피 대상이 지정되어야 한다. FOLLOWING과 같은 이유다 - v2에서
    // 대상 선택 권한은 FSM에 있고(primary_object_id), planner가 스스로
    // 고르지 않는다(D6-1).
    if (command.mode == AVOID && command.target_object_id < 0) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // STOP: 정지 목표 s가 실제로 지정되어야 한다. 기본값 NaN이 그대로 오면
    // GenerateStoppingCandidates가 NaN 경계조건으로 후보를 만들게 된다.
    if (command.mode == STOP) {
        if (!std::isfinite(command.stop_position) || command.stop_position < 0.0) {
            reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
            return false;
        }
    }

    reason = FrenetPlanner::PlanFailureReason::NONE;
    return true;
}

}  // namespace

bool FrenetPlanner::Plan(const CartesianState& ego,
                         const std::vector<TrackedObject>& obstacles,
                         const PlannerCommand& command,
                         CartesianPath& out_path) {
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    return PlanInternal(ego, obstacles, &command,
                        ExternalPlanningPurpose::NORMAL, out_path, nullptr);
}

bool FrenetPlanner::PrepareAvoid(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command, CartesianPath& maintenance_path,
    AvoidPrepareResult& result) {
    result = AvoidPrepareResult{};
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    // PREPARE는 AVOID 전용이다. 다른 모드로 호출되면 호출자의 실수다.
    if (command.mode != AVOID) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    return PlanInternal(ego, obstacles, &command,
                        ExternalPlanningPurpose::AVOID_PREPARE,
                        maintenance_path, &result);
}

bool FrenetPlanner::PlanCommittedAvoid(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command, CartesianPath& out_path,
    AvoidPrepareResult& result) {
    result = AvoidPrepareResult{};
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    if (command.mode != AVOID ||
        !v2_avoidance_.active || !v2_avoidance_.committed) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    return PlanInternal(ego, obstacles, &command,
                        ExternalPlanningPurpose::AVOID_COMMITTED,
                        out_path, &result);
}

FrenetPlanner::PlanningContext FrenetPlanner::PreparePlanningContext(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& external_command) {
    PlanningContext ctx;

    CartesianToFrenet(ref_, ego, ctx.s, ctx.s_dot, ctx.s_ddot, ctx.d,
                       ctx.d_prime, ctx.d_pprime,
                       has_last_s_ ? &last_s_ : nullptr);
    last_s_ = ctx.s;
    has_last_s_ = true;

    // ObjectInfo의 연속 heading으로 yaw-rate를 추정한다. MORAI 객체의 heading은
    // 회전교차로에서 계속 변하므로 이를 보존해야 접선 직선 예측의 누락을 피할 수 있다.
    std::unordered_map<int, double> object_yaw_rates;
    for (auto& item : object_tracks_) ++item.second.missing_cycles;
    for (const auto& obj : obstacles) {
        auto it = object_tracks_.find(obj.id);
        if (it == object_tracks_.end()) {
            object_tracks_[obj.id] = ObjectTrack{obj.heading, 0.0, 0};
        } else {
            const double measured = std::remainder(obj.heading - it->second.heading,
                                                    2.0 * M_PI) / 0.05;
            // UDP heading 양자화/순간 점프를 억제하고 차량 수준의 yaw-rate로 제한한다.
            const double bounded = clip(measured, -1.0, 1.0);
            it->second.yaw_rate = 0.8 * it->second.yaw_rate + 0.2 * bounded;
            it->second.heading = obj.heading;
            it->second.missing_cycles = 0;
        }
        object_yaw_rates[obj.id] = object_tracks_[obj.id].yaw_rate;
    }
    for (auto it = object_tracks_.begin(); it != object_tracks_.end();) {
        if (it->second.missing_cycles > 20) it = object_tracks_.erase(it);
        else ++it;
    }
    // obstacles의 각 TrackedObject는 이미 authoritative corners(perception의
    // obb_corners)를 갖고 있다 - 여기서 다시 만들지 않는다. yaw_rate만
    // 연속 관측으로 갱신한 사본을 만들어 motion과 collision geometry 양쪽에
    // 그대로 쓴다(더 이상 두 개의 병렬 vector로 나누지 않는다).
    ctx.predicted_obstacles = obstacles;
    for (auto& obj : ctx.predicted_obstacles) {
        const auto it = object_yaw_rates.find(obj.id);
        obj.yaw_rate = it == object_yaw_rates.end() ? 0.0 : it->second;
    }

    ctx.resolved_target_speed = LookaheadTargetSpeed(
        ref_, ctx.s, ego.v, curve_speed_cfg_, limits_.max_longitudinal_accel,
        limits_.max_lateral_accel, limits_.max_curvature);
    if (external_command.has_target_speed) {
        ctx.resolved_target_speed = std::min(ctx.resolved_target_speed,
                                             external_command.target_speed);
    }
    if (external_command.has_max_speed) {
        ctx.resolved_target_speed = std::min(ctx.resolved_target_speed,
                                             external_command.max_speed);
    }

    // Cycle-to-cycle target speed recovery continuity
    // (fix/yuntae-frenet-speed-continuity Phase 2).
    ctx.resolved_target_speed = ApplyTargetSpeedRateLimit(
        ctx.resolved_target_speed, last_target_speed_, has_target_speed_history_,
        max_target_speed_accel_, replanning_period_s_);
    has_target_speed_history_ = true;

    last_d_ = ctx.d;

    // FOLLOW가 아닌 외부 명령으로 넘어가면 v2 FOLLOW 상태를 즉시 비운다.
    //
    // **모든 early-return보다 앞에서** 해야 한다. PlanInternal의 EMERGENCY
    // 분기와 저속 fallback은 명령 조립 지점에 도달하기 전에 return하므로,
    // 그곳에서만 reset하면 context가 살아남는다. 그러면
    //
    //   FOLLOW id=10 target=15   -> 15 확보
    //   E-STOP id=11             -> context가 남는다
    //   FOLLOW id=12 target=15   -> 15가 관측되지 않아도 GRACE
    //
    // 가 되어, 첫 binding 실패인데 유예를 받아 존재하지 않는 leader를
    // 예측 추종한다(실측으로 재현 확인).
    if (external_command.mode != FOLLOWING) {
        v2_following_ = FollowingContext{};
    }
    // v2 AVOID 상태도 같은 이유로 같은 위치에서 비운다(D6-1). 여기서
    // reset하지 않으면
    //   AVOID id=20 target=21   -> 21 확보
    //   E-STOP id=21            -> context가 남는다
    //   AVOID id=22 target=21   -> 21이 관측되지 않아도 GRACE
    // 가 되어 첫 binding 실패인데 유예를 받는다(D5-2에서 FOLLOW로 실측 확인한
    // 것과 같은 구조).
    if (external_command.mode != AVOID) {
        v2_avoidance_ = AvoidanceContext{};
        // 목표 목록도 함께 비운다(D6-2a). 남겨두면 다음 AVOID가 확보에
        // 실패했는데 이전 사이클의 목표로 후보를 만들 수 있다.
        v2_avoid_targets_ = AvoidTargetPlan{};
    }

    return ctx;
}

bool FrenetPlanner::PlanInternal(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand* external_command,
    ExternalPlanningPurpose purpose, CartesianPath& out_path,
    AvoidPrepareResult* avoid_prepare_result) {
    // purpose가 계산 내용을 정한다. 포인터 유무로 판단하지 않는다 - 진단용
    // 으로 포인터만 넘겨도 동작이 바뀌면 안 된다(헤더 주석 참고).
    const bool avoid_prepare =
        purpose == ExternalPlanningPurpose::AVOID_PREPARE;
    const bool avoid_committed =
        purpose == ExternalPlanningPurpose::AVOID_COMMITTED;
    last_failure_reason_ = PlanFailureReason::NONE;
    last_candidate_paths_.clear();
    last_candidate_validity_.clear();
    if (!ref_loaded_) {
        last_failure_reason_ = PlanFailureReason::NOT_INITIALIZED;
        return false;
    }

    // ===== 공용 준비(§3): Cartesian->Frenet, object tracking/예측, target
    // speed 해석/연속성, 모드 전환 시 v2 FOLLOW/AVOID context reset. =====
    PlanningContext ctx = PreparePlanningContext(ego, obstacles, *external_command);
    const double s = ctx.s;
    const double s_dot = ctx.s_dot;
    double s_ddot = ctx.s_ddot;
    const double d = ctx.d;
    const double d_prime = ctx.d_prime;
    const double d_pprime = ctx.d_pprime;
    std::vector<TrackedObject>& predicted_obstacles = ctx.predicted_obstacles;
    const double resolved_target_speed = ctx.resolved_target_speed;

    // ===== 이 아래부터는 모드별 dispatch다: EMERGENCY -> AVOID 대상 확보 ->
    // 저속 fallback -> FrenetState boundary 구성 -> AVOID PREPARE/COMMITTED
    // -> switch(mode)의 KEEP/FOLLOW/STOP Process. =====

    // 저속(정지 근처)에서는 quintic 후보/곡률필터 파이프라인 전체를 건너뛰고
    // fallback으로 직접 경로를 만든다 (클래스 헤더 주석 참고 - 나누기 없는 안전한
    // 경로). AVOID/장애물 회피는 이 fallback에서는 다루지 않는다 - 정지 근처
    // 속도로는 위험한 회피 기동이 필요할 상황이 원래 적고, 속도가 오르면 바로
    // 아래의 정상 파이프라인으로 자동 복귀한다.
    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s]

    // =====================================================================
    // 외부 EMERGENCY_STOP: 후보 파이프라인을 건너뛰고 직접 감속한다.
    //
    // BehaviorCommand는 목표점 없이(has_target_pose=false) 즉시 긴급정지를
    // 지시하므로 정지 위치 계산 소유권이 Frenet에 있다(계약 8.1절).
    //
    // ResolveManeuver / 곡률 필터 / 충돌 필터로 내려보내지 않는 이유:
    //   1. stop_position이 NaN이므로 GenerateStoppingCandidates가 NaN
    //      경계조건으로 후보를 만든다.
    //   2. 후보가 전부 탈락하면 "긴급정지인데 아무 궤적도 못 보냄"이 된다.
    //      긴급정지에서 그 실패 방향 자체를 제거하는 것이 목적이다.
    //
    // 저속 fallback보다 먼저 처리한다. 그 fallback은 kMinCreepSpeed로 기어서
    // 전진하는 경로라 긴급정지 중에 차량이 계속 움직이게 된다.
    //
    // 차량이 이미 멈춰 있으면(v0 ~= 0) 0속도 궤적이 나온다. 새 정상 명령이
    // 올 때까지 긴급정지 상태를 계속 강제하는 것이 E-stop의 의미이므로,
    // 여기서 COMPLETED로 올리지 않는다 - ROS 계층이 EXECUTING을 유지한다.
    if (external_command->mode == EMERGENCY) {
        return EmergencyStopProcess(ego, s, d, s_dot, out_path);
    }

    // =====================================================================
    // 외부 AVOID 대상 확보 (D6-2a)
    //
    // ===== 왜 여기인가 =====
    //
    // D6-1에서는 이 처리가 명령 조립 지점(아래 external_command 분기)에
    // 있었다. 그런데 그 지점은 저속 fallback보다 **아래**다. 그래서
    // ego.v < kLowSpeedFallbackV(0.5 m/s)이면 fallback이 먼저 return해
    // resolver에 도달하지 않았다. 실측(AVOID target=21, 장애물 25 m 전방):
    //
    //   ego_v  Plan()  active_avoid_target_id  failure_reason
    //   0.60   false   21                      UNSUPPORTED_BEHAVIOR
    //   0.40   true    -1                      NONE        <- 확보 없이 성공
    //   0.00   true    -1                      NONE
    //
    // 저속 세 행은 Plan()이 성공을 반환하면서 대상을 확보하지 않았고, 나오는
    // 궤적은 fallback_target_d=0.0의 중앙 크롤링 경로다. gate를 열면 FSM
    // 입장에서 "회피 성공"으로 보이면서 장애물 쪽으로 기어간다.
    //
    // 그래서 확보를 저속 판단보다 위로 올린다. 장애물 앞에서 감속한 뒤
    // AVOID 명령을 받는 것은 오히려 **정상적인 시나리오**이므로, 그때
    // 대상조차 확보하지 못하는 구조는 맞지 않다(계약 12.7.2절).
    //
    // 확보만 여기서 하고, "그 목표로 실제 후보가 나오는가"는 D6-2b다.
    if (external_command->mode == AVOID) {
        if (!PrepareExternalAvoidTargets(s, obstacles, *external_command,
                                         avoid_prepare, avoid_committed)) {
            return false;
        }
    }
    // PREPARE인데 AVOID가 아니면 호출자의 실수다. PrepareAvoid()가 이미
    // 막지만, PlanInternal 내부에서도 조용히 다른 모드를 준비하지 않는다.
    if ((avoid_prepare || avoid_committed) && external_command->mode != AVOID) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    const bool external_stop = external_command->mode == STOP;
    // 외부 FOLLOW도 저속 fallback을 타지 않는다. 그 경로는 kMinCreepSpeed로
    // 기어서 전진하는데, 앞차가 정지해 있으면 그대로 다가가 붙는다. FOLLOW의
    // 종방향 목표는 선행차와의 gap이므로 정상 following 후보 생성기
    // (GenerateFollowingCandidates)로 보내야 한다 - 그 생성기는 leader가
    // 멈춰 있으면 min_gap을 유지하는 정지 목표를 만든다.
    const bool external_follow = external_command->mode == FOLLOWING;
    // AVOID PREPARE도 이 fallback을 타지 않는다 (D6-2b).
    //
    // 이 경로는 충돌 필터를 거치지 않은 crawl을 그대로 반환하므로, PREPARE의
    // 유지 궤적으로 쓰면 장애물 쪽으로 기어간다. 저속 PREPARE의 유지 궤적은
    // BuildAvoidMaintenancePath가 **충돌 검사를 통과시킨 뒤** 만든다.
    //
    // 여기서 제외하지 않으면 PREPARE 블록에 도달하지도 못한다 - 이 분기가
    // 먼저 return하기 때문이다(실측으로 확인: v=0.3에서 low_speed_context이
    // 0으로 나오고 maintenance가 미검사 crawl이었다).
    if (std::fabs(ego.v) < kLowSpeedFallbackV &&
        !external_stop && !external_follow && !avoid_prepare &&
        !avoid_committed) {
        static int low_speed_log_tick = 0;
        if (++low_speed_log_tick % 5 == 0) {
            std::printf("[FrenetPlanner-STATE] mode=LOW_SPEED_FALLBACK "
                        "s=%.2f d=%.3f speed=%.2f candidates=N/A\n",
                        s, d, ego.v);
        }
        last_mode_ = external_command->mode;
        last_target_speed_ = resolved_target_speed;
        const bool fallback_ok = PlanLowSpeedFallback(
            ego, s, d, 0.0, out_path, 2.0, resolved_target_speed);
        if (fallback_ok) {
            last_candidate_paths_.push_back(out_path);
            last_candidate_validity_.push_back(true);
        }
        return fallback_ok;
    }

    double d_dot, d_ddot;
    ArcDerivToTimeDeriv(s_dot, s_ddot, d_prime, d_pprime, d_dot, d_ddot);
    last_d_dot_ = d_dot;  // 클램프 전 원본 값 - 진단 목적(아래 kMaxDDot 클램프 폭 자체를 보려고)

    // 투영 오차가 후보의 초기 가속도 경계조건을 비현실적으로 키우지 않도록
    // 다항식 중간구간의 overshoot 여유를 포함해 한계의 80%로 제한한다.
    constexpr double kBoundaryMargin = 0.8;
    s_ddot = clip(s_ddot, -limits_.max_longitudinal_accel * kBoundaryMargin,
                  limits_.max_longitudinal_accel * kBoundaryMargin);
    d_ddot = clip(d_ddot, -limits_.max_lateral_accel * kBoundaryMargin,
                  limits_.max_lateral_accel * kBoundaryMargin);

    // 큰 heading 오차로 과대 추정될 수 있는 초기 횡속도도 제한한다.
    constexpr double kMaxDDot = 0.5;  // [m/s]
    d_dot = clip(d_dot, -kMaxDDot, kMaxDDot);

    FrenetState start{s, s_dot, s_ddot, d, d_dot, d_ddot};

    // =====================================================================
    // AVOID PREPARE: 후보 평가와 유지 궤적을 한 사이클에서 만든다 (D6-2b)
    //
    // 계약 5.2절. 두 결과를 절대 같은 것으로 취급하지 않는다:
    //   candidate  "AVOID를 실행해도 되는가"     -> result.candidate_valid
    //   out_path   "지금 MPC가 따라갈 것"        -> maintenance
    //
    // 위쪽 전처리(Cartesian->Frenet, object tracking/prediction)를 **한 번만**
    // 통과했다. PlanInternal을 두 번 부르면 last_s_ / missing_cycles /
    // yaw-rate 재추정이 중복된다(계약 12.7.1절).
    // =====================================================================
    if (avoid_prepare || avoid_committed) {
        return AvoidProcess(
            ego, start, s, d, resolved_target_speed,
            predicted_obstacles, *external_command,
            avoid_prepare, avoid_committed, out_path, *avoid_prepare_result);
    }

    // 여기 도달하면 external_command->mode는 IsSupportedExternalMode의
    // whitelist와 위쪽 EMERGENCY/AVOID 분기에 의해 반드시
    // {LANE_KEEPING, STOP, FOLLOWING} 중 하나다. Plan()이 validation ->
    // 공용 준비(위) -> switch(mode) -> Process() -> 공용 후처리
    // (ExecuteManeuverPipeline 내부)로 읽히도록 각 모드의 명시적 진입점으로
    // 보낸다(§6).
    switch (external_command->mode) {
        case FOLLOWING:
            return FollowProcess(
                ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
                predicted_obstacles, resolved_target_speed,
                *external_command, out_path);
        case STOP:
            return StopProcess(
                ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
                predicted_obstacles, resolved_target_speed,
                *external_command, out_path);
        default:
            return KeepProcess(
                ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
                predicted_obstacles, resolved_target_speed,
                *external_command, out_path);
    }
}
