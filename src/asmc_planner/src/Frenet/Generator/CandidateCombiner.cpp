#include "Frenet/Generator/CandidateCombiner.hpp"

#include <algorithm>
#include <cmath>

#include "Frenet/Generator/LateralGenerator.hpp"
#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Frenet/Generator/SpatialStop.hpp"

namespace {
// Keep the normal search unchanged. Only when every longitudinal candidate
// violates kinematics, try intermediate acceleration targets in bounded work.
// All recovered candidates still enter the common curvature/collision filters.
std::vector<FrenetPath> FeasibleFollow(
    const FrenetState& start,double leader_s,double leader_speed,double leader_accel,
    double time_gap,double min_gap,double target_speed,double gap_gain,
    const PathGeneratorConfig& cfg,const KinematicLimits& limits) {
    const auto generate=[&](double speed) {
        auto paths=GenerateFollowingCandidates(start,leader_s,leader_speed,
            leader_accel,time_gap,min_gap,speed,gap_gain,cfg);
        FilterLongitudinalByKinematicLimits(paths,limits);
        return paths;
    };
    const auto usable=[](const auto& paths) {
        return std::any_of(paths.begin(),paths.end(),[](const auto& p){return p.valid;});
    };
    auto paths=generate(target_speed);
    if(usable(paths)||target_speed<=start.s_d)return paths;
    for(double fraction:{.75,.5,.25}) {
        auto reduced=generate(start.s_d+fraction*(target_speed-start.s_d));
        if(usable(reduced))return reduced;
    }
    return paths;
}
}

std::vector<FrenetPath> CombineLateralLongitudinal(const std::vector<FrenetPath>& lateral_set,
                                                    const std::vector<FrenetPath>& longitudinal_set) {
    std::vector<FrenetPath> result;

    for (const auto& lat : lateral_set) {
        if (!lat.valid || lat.t.empty()) continue;
        const double T_lat = lat.t.back();

        for (const auto& lon : longitudinal_set) {
            if (!lon.valid || lon.t.empty()) continue;
            const double T_lon = lon.t.back();

            if (std::abs(T_lat - T_lon) > 1e-6) continue;   // 같은 T만 결합
            if (lat.t.size() != lon.t.size()) continue;      // 안전장치 (동일 T, dt면 항상 같아야 함)

            FrenetPath combined;
            combined.t     = lat.t;
            combined.d     = lat.d;
            combined.d_d   = lat.d_d;
            combined.d_dd  = lat.d_dd;
            combined.s     = lon.s;
            combined.s_d   = lon.s_d;
            combined.s_dd  = lon.s_dd;
            combined.s_ddd = lon.s_ddd;
            combined.valid = true;
            combined.rejection_reason = RejectionReason::NONE;
            combined.cost_lat = combined.cost_lon = combined.cost_total = 0.0;
            combined.jerk_cost_lat = lat.jerk_cost_lat;
            combined.jerk_cost_lon = lon.jerk_cost_lon;
            combined.delta_s       = lon.delta_s;
            combined.delta_s_dot   = lon.delta_s_dot;

            result.push_back(std::move(combined));
        }
    }

    return result;
}

std::vector<FrenetPath> ResolveManeuver(const FrenetState& start,
                                         const PlannerCommand& cmd,
                                         const RefLine& ref,
                                         const PathGeneratorConfig& cfg,
                                         const KinematicLimits& limits,
                                         double lane_width,
                                         const std::vector<TrackedObject>& obstacles,
                                         const VehicleShape& vehicle_shape,
                                         const CollisionCheckConfig& collision_cfg,
                                         const StopConfig& stop_cfg,
                                         const MergeConfig& merge_cfg,
                                         PlannerDebugStats* stats,
                                         CartesianGeometryMode geometry_mode) {
    // 1. lateral 후보. 전 모드에 공유된 [-3,+3]m 격자를 쓰면
    // LANE_KEEPING에서도 먼 장애물을 보고 옆 후보가 선택된다. 모드가
    // 지정한 목표 d 주변의 작은 범위만 생성해 역할을 분리한다.
    PathGeneratorConfig lateral_cfg = cfg;
    // COMMIT/CROSS에서는 짧은 종방향 거리 안에 d=0 복귀까지 강제하지 않는다.
    // 현재 횡위치를 유지해 합류 타이밍 궤적과 횡복귀 궤적을 분리하고, CLEAR 후
    // LANE_KEEPING이 차선 중앙 복귀를 담당한다.
    const double d_offset = cmd.has_stop_lateral_hold
        ? cmd.stop_lateral_hold_d
        : ((cmd.mode == MERGE && cmd.merge_committed)
               ? start.d : ResolveLateralOffset(cmd, lane_width));
    // MERGE는 차선 변경 모드가 아니라 회전교차로 global path의 종방향 진입
    // 타이밍만 조절한다. ±0.5m 후보가 충돌 회피처럼 번갈아 선택되며 조향이
    // 흔들리지 않도록 종단 d를 차선 중심 하나로 고정한다.
    //
    // LANE_KEEPING도 동일한 문제가 있다: 전역 tolerance를 그대로 쓰면 코너에서
    // d=-0.5/0/+0.5 후보가 cost 경쟁을 하며 cycle마다 선택된 terminal d가 바뀐다
    // (조향 방향 반전/oscillation의 원인). KEEP nominal target은 차선 중앙
    // d=0 하나로 고정하고, 급커브에서 중앙 후보가 전부 곡률 필터에 걸리면
    // AppendExternalKeepContinuityCandidates()가 별도로 현재 d 유지 후보를
    // 추가하므로 recovery 경로는 그대로 유지된다. AVOID/LANE_CHANGE 등 다른
    // maneuver는 목표 d 주변 탐색이 필요하므로 전역 tolerance를 유지한다.
    const double lateral_tolerance =
        (cmd.has_stop_lateral_hold || cmd.mode == MERGE ||
         cmd.mode == LANE_KEEPING)
            ? 0.0 : cfg.lateral_target_tolerance;
    lateral_cfg.lateral_d1.min = d_offset - lateral_tolerance;
    lateral_cfg.lateral_d1.max = d_offset + lateral_tolerance;

    std::vector<FrenetPath> lateral_set =
        GenerateLateralCandidates(start, lateral_cfg, limits.low_speed_fallback_v);
    FilterLateralByAcceleration(lateral_set, limits);
    if (stats) {
        stats->lateral_total = lateral_set.size();
        for (const auto& p : lateral_set) if (p.valid) stats->lateral_valid++;
    }

    // 2. longitudinal 후보 (FSM이 지정한 모드에 따라 생성 방식 선택)
    std::vector<FrenetPath> longitudinal_set;

    switch (cmd.mode) {
        case FOLLOWING:
            longitudinal_set = FeasibleFollow(start, cmd.leader_s, cmd.leader_speed,
                                                            cmd.leader_accel, cmd.time_gap,
                                                            cmd.min_gap, cmd.target_speed,
                                                            cmd.gap_gain, cfg, limits);
            break;

        case STOP:
        case INTERSECTION_WAIT: {
            // 공용 delta_s는 [-3, +3]m이라 +쪽 후보는 정지 목표를 지나쳐서
            // 멈춘다. 정지선/정지 지점 semantics에서는 오버슈트가 곧 위반이므로
            // STOP 계열은 목표 앞에서만 멈추도록 delta 범위를 제한한다.
            PathGeneratorConfig stop_delta_cfg = cfg;
            stop_delta_cfg.delta_s = stop_cfg.delta_s_override;
            longitudinal_set =
                GenerateStoppingCandidates(start, cmd.stop_position, stop_delta_cfg);
            break;
        }

        case EMERGENCY:
            // FSM이 emergency 상황에 맞는 stop_position(예: 현재 위치 바로 앞)을
            // 설정해준다고 가정. path_generator는 물리적으로 안전한 제동거리를
            // 스스로 계산하지 않는다 (TODO: 추후 FSM과 함께 정책 확정 필요).
            longitudinal_set = GenerateStoppingCandidates(start, cmd.stop_position, cfg);
            break;

        // R6-D audit: frenet_planner.cpp의 IsSupportedExternalMode()가 MERGE를
        // 허용하지 않고, PrepareMerge/PlanCommittedMerge 같은 진입점 자체가
        // 없다 - 이 case는 production에서 어떤 caller로부터도 도달하지
        // 않는다(테스트가 PlannerCommand{mode=MERGE}로 직접 호출할 때만
        // 실행된다). AVOID/LANE_CHANGE가 request-native로 이전한 뒤에도 이
        // case가 legacy ResolveManeuver를 완전히 제거하지 못하게 막는 마지막
        // 이유다 - MERGE의 FSM 계약이 열리기 전까지는 건드리지 않는다
        // (Option 1: behavior-preserving 유지, 임의로 활성화하거나 삭제하지
        // 않는다).
        case MERGE:
            if (cmd.merge_crossing) {
                if (cmd.target_object_id >= 0) {
                    longitudinal_set = GenerateFollowingCandidates(
                        start, cmd.leader_s, cmd.leader_speed, cmd.leader_accel,
                        cmd.time_gap, cmd.min_gap, cmd.target_speed, cmd.gap_gain, cfg);
                } else {
                    longitudinal_set = GenerateVelocityKeepingCandidates(
                        start, cmd.target_speed, cfg);
                }
            } else if (!cmd.merge_gap_safe) {
                const double stop_distance = std::max(0.0, cmd.merge_stop_s - start.s);
                const double braking_distance = start.s_d * start.s_d /
                    (2.0 * limits.max_longitudinal_accel);
                if (stop_distance > braking_distance + merge_cfg.braking_distance_margin) {
                    // 먼 거리부터 정지 quintic을 강제하면 모든 T 후보가 조기 감속한다.
                    // 제동구간 전까지는 global-path 순항을 유지한다.
                    longitudinal_set = GenerateVelocityKeepingCandidates(
                        start, cmd.target_speed, cfg);
                } else {
                    longitudinal_set = GenerateStoppingCandidates(start, cmd.merge_stop_s, cfg);
                }
            } else {
                const double distance = std::max(0.0, cmd.merge_conflict_s - start.s);
                const double arrival_speed =
                    cmd.merge_entry_time > merge_cfg.min_entry_time_for_arrival_speed
                        ? distance / cmd.merge_entry_time : cmd.target_speed;
                if (cmd.merge_committed &&
                    cmd.merge_entry_time >= cfg.time_horizon.min - 1e-6) {
                    longitudinal_set = GenerateMergeArrivalCandidates(
                        start, cmd.merge_conflict_s, cmd.merge_entry_time,
                        std::min(cmd.target_speed,
                                 std::max(arrival_speed, merge_cfg.min_arrival_speed)),
                        cfg, merge_cfg.arrival_time_match_tolerance);
                } else if (cmd.merge_committed) {
                    // conflict 도착까지 남은 시간이 lateral/longitudinal 공통
                    // 최소 horizon보다 짧아지면 exact-arrival quintic을 만들 수 없다.
                    // 기존 구현은 entry_time(예: 1.2s)을 1.5s로 강제 확장해 가까운
                    // conflict point에 늦게 도착하도록 급감속/재가속했고, 결국
                    // 종가속도 필터에서 유일한 후보가 사라져 zone 앞에서 정지했다.
                    // COMMIT은 이미 전체 후보 충돌 검증을 통과했으므로 이 구간부터는
                    // 교차구역 통과 목표속도를 유지해 시간창을 끝까지 실행한다.
                    longitudinal_set = GenerateVelocityKeepingCandidates(
                        start, cmd.target_speed, cfg);
                } else {
                    if (cmd.target_object_id >= 0) {
                        longitudinal_set = GenerateFollowingCandidates(
                            start, cmd.leader_s, cmd.leader_speed, cmd.leader_accel,
                            cmd.time_gap, cmd.min_gap,
                            std::min(cmd.target_speed,
                                     std::max(arrival_speed, merge_cfg.min_arrival_speed)),
                            cmd.gap_gain, cfg);
                    } else {
                        longitudinal_set = GenerateVelocityKeepingCandidates(
                            start,
                            std::min(cmd.target_speed,
                                     std::max(arrival_speed, merge_cfg.min_arrival_speed)),
                            cfg);
                    }
                }
            }
            break;

        case LANE_KEEPING:
        case LANE_CHANGE_LEFT:
        case LANE_CHANGE_RIGHT:
        case TURN_LEFT:
        case TURN_RIGHT:
        case AVOID:
        default:
            // 논문 Sec.VII: velocity keeping은 앞 차가 없는 대부분의 상황(차선유지,
            // 차선변경, 회피 등)의 기본 종방향 모드.
            longitudinal_set = GenerateVelocityKeepingCandidates(start, cmd.target_speed, cfg);
            break;
    }

    FilterLongitudinalByKinematicLimits(longitudinal_set, limits);
    if (stats) {
        stats->longitudinal_total = longitudinal_set.size();
        for (const auto& p : longitudinal_set) if (p.valid) stats->longitudinal_valid++;
    }

    // 3. 결합 + 사후 곡률 필터링
    std::vector<FrenetPath> combined = CombineLateralLongitudinal(lateral_set, longitudinal_set);
    if (stats) stats->combined_total = combined.size();
    FilterByCurvature(combined, ref, limits, geometry_mode);
    if (stats) {
        for (const auto& p : combined) if (p.valid) stats->combined_valid_after_curvature++;
    }

    // 4. 모든 모드에 공통 OBB 충돌 필터를 적용한다. FOLLOWING/MERGE leader는
    // 실제 후보 구간에서는 검사하고 reactive coast 연장에서만 제외한다.
    if (!obstacles.empty()) {
        const int coast_exempt_id =
            (cmd.mode == FOLLOWING || cmd.mode == MERGE) ? cmd.target_object_id : -1;
        const int avoid_escape_id =
            cmd.mode == AVOID ? cmd.target_object_id : -1;
        FilterByCollision(combined, ref, obstacles, vehicle_shape, collision_cfg,
                          coast_exempt_id, avoid_escape_id);
    }
    if (stats) {
        for (const auto& p : combined) if (p.valid) stats->combined_valid_after_collision++;
    }

    return combined;
}

std::vector<FrenetPath> GenerateRequestedCandidates(
    const FrenetState& start,
    const TrajectoryRequest& request,
    const RefLine& ref,
    const PathGeneratorConfig& cfg,
    const KinematicLimits& limits,
    const std::vector<TrackedObject>& obstacles,
    const VehicleShape& vehicle_shape,
    const CollisionCheckConfig& collision_cfg,
    PlannerDebugStats* stats,
    CartesianGeometryMode geometry_mode) {
    // 1. lateral 후보. request.lateral이 authoritative input이다 - target_d/
    // search_tolerance를 ResolveLateralOffset이나 cmd.mode로 다시 계산하지
    // 않는다(§5).
    PathGeneratorConfig lateral_cfg = cfg;
    lateral_cfg.lateral_d1.min =
        request.lateral.target_d - request.lateral.search_tolerance;
    lateral_cfg.lateral_d1.max =
        request.lateral.target_d + request.lateral.search_tolerance;

    std::vector<FrenetPath> lateral_set =
        GenerateLateralCandidates(start, lateral_cfg, limits.low_speed_fallback_v);
    FilterLateralByAcceleration(lateral_set, limits);
    if (stats) {
        stats->lateral_total = lateral_set.size();
        for (const auto& p : lateral_set) if (p.valid) stats->lateral_valid++;
    }

    // 2. longitudinal 후보 (request.longitudinal_mode에 따라 생성 방식 선택)
    std::vector<FrenetPath> longitudinal_set;

    switch (request.longitudinal_mode) {
        case LongitudinalMode::FOLLOWING:
            longitudinal_set = FeasibleFollow(
                start, request.following.leader_s, request.following.leader_speed,
                request.following.leader_accel, request.following.time_gap,
                request.following.min_gap, request.target_speed,
                request.following.gap_gain, cfg, limits);
            break;

        case LongitudinalMode::STOPPING: {
            // 공용 delta_s는 [-3, +3]m이라 +쪽 후보는 정지 목표를 지나쳐서
            // 멈춘다. STOP은 목표 앞에서만 멈추도록 delta 범위를 제한한다 -
            // 이 범위는 request.stop.terminal_offset으로 온다(호출자가
            // StopConfig.delta_s_override를 그대로 옮겨 채운 값).
            PathGeneratorConfig stop_delta_cfg = cfg;
            stop_delta_cfg.delta_s = request.stop.terminal_offset;
            longitudinal_set = request.stop.spatial_profile
                ? GenerateSpatialStoppingCandidates(start, request.stop.stop_s,
                    request.stop.cruise_speed, request.stop.comfortable_decel,
                    stop_delta_cfg, limits, request.stop.acceleration_limit,request.stop.braking_only)
                : GenerateStoppingCandidates(start, request.stop.stop_s, stop_delta_cfg);
            break;
        }

        case LongitudinalMode::VELOCITY_KEEPING:
        default:
            longitudinal_set = request.speed_envelope.empty()
                ? GenerateVelocityKeepingCandidates(start, request.target_speed, cfg)
                : GenerateEnvelopeVelocityCandidates(
                    start, request.target_speed, request.speed_envelope, cfg, limits);
            break;
    }

    FilterLongitudinalByKinematicLimits(longitudinal_set, limits);
    if (stats) {
        stats->longitudinal_total = longitudinal_set.size();
        for (const auto& p : longitudinal_set) if (p.valid) stats->longitudinal_valid++;
    }

    // 3. 결합 + 사후 곡률 필터링
    std::vector<FrenetPath> combined = CombineLateralLongitudinal(lateral_set, longitudinal_set);
    if (stats) stats->combined_total = combined.size();
    FilterByCurvature(combined, ref, limits, geometry_mode);
    if (stats) {
        for (const auto& p : combined) if (p.valid) stats->combined_valid_after_curvature++;
    }

    // 4. 충돌 필터. request.collision이 예외 정책의 단일 authoritative
    // 입력이다(R6-A) - longitudinal_mode를 보고 coast_exempt/escape를 다시
    // 추론하지 않는다. 호출자가 FOLLOWING이면 leader를, AVOID면 회피 대상을
    // 채워 넣고, 해당 없으면 -1을 그대로 둔다.
    if (!obstacles.empty()) {
        FilterByCollision(combined, ref, obstacles, vehicle_shape, collision_cfg,
                          request.collision.coast_exempt_object_id,
                          request.collision.margin_recovery_object_id);
    }
    if (stats) {
        for (const auto& p : combined) if (p.valid) stats->combined_valid_after_collision++;
    }

    return combined;
}

