/**
 * @file Merge.cpp
 * @brief legacy MERGE(roundabout/highway)의 approach/window/gap 판단과
 *        phase/context 갱신 (MERGE M1).
 *
 * locked COMMIT trajectory의 replay/emergency recheck, commit_pending
 * candidate의 full-path 검증, 최종 selected path commit(gap_locked=true
 * 전환 및 locked_path 저장)은 이 파일로 옮기지 않는다(MERGE M2 대상) -
 * PlanInternal이 그 코드를 계속 소유하며, 이 함수가 갱신한 merge_를 그대로
 * 읽는다. SelectLegacyBehavior(AVOID > MERGE > FOLLOW > KEEP 우선순위 결정)도
 * 이 파일로 옮기지 않는다.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cstdio>

#include "frenet_planner/Maneuver/MergeInternal.hpp"

bool FrenetPlanner::UpdateLegacyMergeContext(
    double s, const CartesianState& ego, const FrenetState& start,
    const std::vector<ObjectInfo>& predicted_obstacles,
    const std::vector<TrackedObstacle>& predicted_collision_obstacles,
    const std::unordered_map<int, double>& object_yaw_rates,
    const HighwayMergeZone* context_highway_zone) {
    // 두 MERGE 모두 Global Path(d=0)를 그대로 따른다. 회전교차로는 conflict
    // point 도착시간, 고주로는 목표 본선의 완료시점 앞/뒤 gap+TTC를 판단한다.
    const double braking_distance = ego.v * ego.v /
        (2.0 * std::max(limits_.max_longitudinal_accel, 0.1));
    const double dynamic_roundabout_approach = std::max(
        merge_cfg_.approach_distance,
        ego.v * collision_cfg_.reactive_lookahead + braking_distance + 5.0);
    const double roundabout_conflict_gap = merge_cfg_.conflict_s - s;
    const bool roundabout_window = merge_cfg_.conflict_s >= 0.0 &&
        roundabout_conflict_gap >= -merge_cfg_.completion_distance &&
        roundabout_conflict_gap <= dynamic_roundabout_approach;
    const int detected_highway_zone = FindHighwayMergeZone(highway_merge_cfg_, s);
    const bool highway_window = detected_highway_zone >= 0;

    MergeType requested_merge_type = merge_.active ? merge_.type : MergeType::NONE;
    if (requested_merge_type == MergeType::NONE) {
        if (roundabout_window) requested_merge_type = MergeType::ROUNDABOUT;
        else if (highway_window) requested_merge_type = MergeType::HIGHWAY;
    }
    const bool in_merge_approach =
        (requested_merge_type == MergeType::ROUNDABOUT &&
         (roundabout_window || (merge_.active &&
          roundabout_conflict_gap >= -merge_cfg_.completion_distance))) ||
        (requested_merge_type == MergeType::HIGHWAY &&
         (merge_.active
              ? (context_highway_zone && s <= context_highway_zone->completion_s)
              : highway_window));
    if (in_merge_approach) {
        if (!merge_.active) {
            merge_ = MergeContext{};
            merge_.type = requested_merge_type;
            if (merge_.type == MergeType::HIGHWAY)
                merge_.highway_zone_index = detected_highway_zone;
            const HighwayMergeZone* activated_zone = HighwayZoneAt(
                highway_merge_cfg_, merge_.highway_zone_index);
            std::printf("[FrenetPlanner-MERGE] policy activated: %s zone=%s s=%.2f\n",
                        merge_.type == MergeType::HIGHWAY ? "HIGHWAY" : "ROUNDABOUT",
                        activated_zone ? activated_zone->name.c_str() : "-", s);
        }
        merge_.active = true;

        const bool highway = merge_.type == MergeType::HIGHWAY;
        const HighwayMergeZone* active_highway_zone = HighwayZoneAt(
            highway_merge_cfg_, merge_.highway_zone_index);
        const HighwayMergeCheckpoint* active_highway_checkpoint = HighwayCheckpointAt(
            highway_merge_cfg_, merge_.highway_zone_index, merge_.highway_conflict_index);
        if (highway && !active_highway_checkpoint) {
            std::printf("[FrenetPlanner-MERGE] active highway zone missing; reset\n");
            merge_ = MergeContext{};
            return false;
        }
        const double active_conflict_s = highway
            ? active_highway_checkpoint->conflict_s : merge_cfg_.conflict_s;
        const double conflict_gap = active_conflict_s - s;
        const double active_commit_distance = highway
            ? highway_merge_cfg_.commit_distance : merge_cfg_.commit_distance;
        const int active_confirm_cycles = highway
            ? highway_merge_cfg_.safe_confirm_cycles : merge_cfg_.safe_confirm_cycles;
        const double active_cross_speed = highway
            ? highway_merge_cfg_.cross_speed_floor : merge_cfg_.cross_speed_floor;

        if (merge_.gap_locked) {
            merge_.commit_pending = false;
            merge_.entry_time = std::max(0.0, merge_.entry_time - 0.05);
            if (merge_.preceding_time >= 0.0)
                merge_.preceding_time = std::max(0.0, merge_.preceding_time - 0.05);
            if (merge_.following_time >= 0.0)
                merge_.following_time = std::max(0.0, merge_.following_time - 0.05);
            merge_.gap_safe = true;
            // stop_s는 WAIT 정지 위치이지 CROSS 시작점이 아니다. 대표 충돌점에
            // 실제로 도달한 뒤부터 교차구역 통과 단계로 전환한다.
            if (s >= active_conflict_s) {
                const HighwayMergeCheckpoint* next_checkpoint = highway
                    ? HighwayCheckpointAt(highway_merge_cfg_, merge_.highway_zone_index,
                                          merge_.highway_conflict_index + 1)
                    : nullptr;
                if (next_checkpoint) {
                    ++merge_.highway_conflict_index;
                    merge_.gap_locked = false;
                    merge_.gap_safe = false;
                    merge_.safe_cycles = 0;
                    merge_.phase = MergePhase::APPROACH;
                    merge_.locked_path = CartesianPath{};
                    merge_.locked_elapsed = 0.0;
                    merge_.locked_path_index = 0;
                    merge_.preceding_id = -1;
                    merge_.following_id = -1;
                    merge_.preceding_time = -1.0;
                    merge_.following_time = -1.0;
                    std::printf("[FrenetPlanner-MERGE] highway checkpoint advanced: "
                                "zone=%s index=%d next_conflict=%.2f s=%.2f\n",
                                active_highway_zone ? active_highway_zone->name.c_str() : "-",
                                merge_.highway_conflict_index,
                                next_checkpoint->conflict_s, s);
                } else {
                    merge_.phase = MergePhase::CROSS;
                }
            }
        } else {
            const RoundaboutGap gap = highway
                ? FindHighwayMergeGap(
                    ref_, start, predicted_obstacles, predicted_collision_obstacles,
                    highway_merge_cfg_,
                    *active_highway_checkpoint,
                    vehicle_shape_, collision_cfg_,
                    std::max(active_cross_speed, ego.v),
                    limits_.max_longitudinal_accel)
                : FindRoundaboutGap(
                    ref_, start, predicted_obstacles, merge_cfg_,
                    std::max(active_cross_speed, ego.v),
                    limits_.max_longitudinal_accel, &object_yaw_rates);
            const bool same_gap = merge_.preceding_id == gap.preceding_id &&
                                  merge_.following_id == gap.following_id;
            merge_.safe_cycles = gap.confirmed && gap.safe
                ? (same_gap ? merge_.safe_cycles + 1 : 1) : 0;
            merge_.gap_safe = gap.safe;
            merge_.entry_time = gap.entry_time;
            merge_.clear_time = gap.clear_time;
            merge_.preceding_id = gap.preceding_id;
            merge_.preceding_time = gap.preceding_time;
            merge_.following_id = gap.following_id;
            merge_.following_time = gap.following_time;
            merge_.crossing_vehicle_count = gap.crossing_vehicle_count;

            if (conflict_gap > active_commit_distance) {
                // APPROACH에서도 실제 gap 판정을 유지한다. unsafe이면 path_generator가
                // 제동거리 밖에서는 순항하고 제동구간부터 정지 경로로 전환한다.
                merge_.phase = MergePhase::APPROACH;
                // 예측창 밖(unconfirmed)은 진입 허가가 아니라 접근만 허용한다.
                merge_.gap_safe = !gap.confirmed || gap.safe;
                merge_.commit_pending = false;
            } else if (gap.confirmed && gap.safe &&
                       merge_.safe_cycles >= active_confirm_cycles) {
                // 아직 상태를 잠그지 않는다. 이 cycle의 실제 종/횡 결합 후보가
                // 곡률과 충돌검사를 통과한 뒤에만 아래에서 COMMIT으로 승격한다.
                merge_.phase = MergePhase::WAIT;
                merge_.commit_pending = true;
            } else {
                merge_.phase = MergePhase::WAIT;
                merge_.gap_safe = false;
                merge_.commit_pending = false;
            }
        }
    } else {
        merge_ = MergeContext{};
    }
    return true;
}
