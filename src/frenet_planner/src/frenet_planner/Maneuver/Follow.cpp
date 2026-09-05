/**
 * @file Follow.cpp
 * @brief FOLLOW 대상 확보/유지 - external(v2)와 legacy 두 경로.
 *
 * 외부 FSM이 지정한 대상 객체(target_object_id)가 유효한지 검증하고
 * (ResolveExternalFollowTarget), legacy 간이 FSM 경로에서는 같은 차선
 * 최인접 차량을 직접 골라 following_ 컨텍스트를 유지한다
 * (UpdateLegacyFollowContext). 이 파일은 FrenetPlanner::following_ /
 * v2_following_ 상태를 갱신하는 역할까지만 담당한다 - 어떤 종방향 후보
 * 생성기를 쓸지(Generator/CandidateCombiner의 ResolveManeuver)나 gap 계산은
 * 이 파일의 책임이 아니다.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

// 저속 fallback은 시간 대신 이동거리 기준 cosine smoothstep으로 d를 복귀시킨다.
// 양 끝의 d'가 0이므로 정지 근처에서도 급격한 횡이동과 곡률 spike가 생기지 않는다.
FrenetPlanner::FollowTargetStatus FrenetPlanner::ResolveExternalFollowTarget(
    double ego_s, int target_object_id,
    const std::vector<ObjectInfo>& obstacles,
    FollowingContext& context) const {
    if (target_object_id < 0) return FollowTargetStatus::LOST;

    // 지정된 ID만 찾는다. FindLeader()처럼 "가장 가까운" 차량을 고르지
    // 않는다 - 대상 선택 권한은 FSM에 있다.
    const auto it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [target_object_id](const ObjectInfo& object) {
            return object.id == target_object_id;
        });

    if (it != obstacles.end()) {
        double object_s = 0.0, object_d = 0.0, object_s_dot = 0.0;
        ProjectObjectToFrenet(ref_, *it, object_s, object_d, object_s_dot);

        // 폐루프 트랙에서 gap이 음수로 크게 튀지 않도록 감아준다.
        const double track_length =
            ref_.points.empty() ? 0.0 : ref_.points.back().s;
        double gap = object_s - ego_s;
        if (gap <= 0.0 && track_length > 0.0) gap += track_length;

        // 검증 기준은 legacy와 동일하다. 다만 "이 조건을 만족하는 차량을
        // 찾는다"가 아니라 "지정된 차량이 이 조건을 만족하는가"를 본다.
        //
        // 탐색거리도 legacy와 같은 방식으로 확장한다. config 값을 그대로 쓰면
        // 충돌 lookahead보다 짧아져, leader를 FOLLOW 대상이 아니라 충돌물로만
        // 먼저 보고 정지하는 현상이 생긴다(legacy가 이 문제로 확장을 넣었다).
        //
        // 현재 config 기준:
        //   config 값        60.0 m
        //   충돌 horizon     10.0 * 8.0 + 4.5 + 0.3 = 84.8 m
        //   유지 한계        84.8 + 10.0 = 94.8 m
        // 확장하지 않으면 70 m가 되어 80 m 앞의 leader가 v2에서만
        // TARGET_OBJECT_LOST가 된다.
        const double collision_horizon =
            curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead +
            vehicle_shape_.length + collision_cfg_.safety_margin;
        const double effective_search_s =
            std::max(following_cfg_.max_leader_search_s, collision_horizon);
        const double search_limit =
            effective_search_s + following_cfg_.exit_search_margin;
        // 속도 하한(min_leader_speed)을 적용하지 않는다.
        //
        // 그 값(0.5 m/s)은 legacy에서 "움직이는 차량은 FOLLOW, 정지 객체는
        // AVOID"를 planner가 스스로 분류하기 위한 policy였다. v2에서 그
        // 분류 권한은 FSM에 있으므로, FSM이 FOLLOW id=15를 지정했는데
        // Frenet이 "15가 0 m/s니까 FOLLOW하지 않겠다"고 하면 행동 선택에
        // 다시 개입하는 셈이다.
        //
        // 실제 도로에서도 앞차가 신호에 멈추면 그 뒤에 같이 멈추는 것이
        // FOLLOW다. GenerateFollowingCandidates는 정지 leader에 대해
        // min_gap을 유지하는 종방향 계획을 이미 할 수 있다.
        //
        // 역방향으로 움직이는 객체만 거부한다 - 마주 오는 차량을 추종하면
        // gap이 음수로 발산한다.
        const bool same_direction = object_s_dot >= 0.0;
        const bool same_lane = std::fabs(object_d) <= lane_width_ * 0.5;
        const bool in_range = gap > 0.0 && gap <= search_limit;

        if (same_direction && same_lane && in_range) {
            context.active = true;
            context.leader_id = target_object_id;
            context.leader_s = ego_s + gap;
            context.leader_d = object_d;
            context.leader_speed = object_s_dot;
            context.leader_accel = 0.0;
            context.missing_cycles = 0;
            return FollowTargetStatus::VALID;
        }

        // 관측되었지만 쓸 수 없는 상태다. dropout이 아니므로 유예를 주지
        // 않는다 - 예측으로 메꿀 대상이 아니라 조건 자체가 틀렸다
        // (계약 8.2절).
        context = FollowingContext{};
        return FollowTargetStatus::LOST;
    }

    // 관측되지 않았다. 직전에 확보한 상태가 있으면 유예한다.
    //
    // 한 번도 확보하지 못한 대상에는 유예를 주지 않는다. 예측할 기반
    // (leader_s / leader_speed)이 없으므로 유예는 의미가 없고, FSM이 존재하지
    // 않는 ID를 보낸 경우를 조용히 넘기게 된다.
    if (!context.active || context.leader_id != target_object_id) {
        context = FollowingContext{};
        return FollowTargetStatus::LOST;
    }

    if (++context.missing_cycles > following_cfg_.dropout_grace_cycles) {
        context = FollowingContext{};
        return FollowTargetStatus::LOST;
    }

    // 유예 중에는 직전 속도로 leader 위치를 예측한다(legacy와 동일).
    constexpr double kPlannerCycleDt = 0.05;  // 20 Hz
    context.leader_s += context.leader_speed * kPlannerCycleDt;

    return FollowTargetStatus::GRACE;
}

void FrenetPlanner::UpdateLegacyFollowContext(double ego_s, const FrenetState& start,
                                               const std::vector<ObjectInfo>& obstacles,
                                               bool& out_nearest_found,
                                               LeaderTarget& out_nearest_leader) {
    // FOLLOWING 탐색거리도 충돌 lookahead보다 항상 길게 잡아,
    // leader를 모드 전환 전에 충돌물로만 먼저 보고 정지하는 현상을 막는다.
    FollowingConfig effective_following_cfg = following_cfg_;
    const double following_collision_horizon =
        curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead
        + vehicle_shape_.length + collision_cfg_.safety_margin;
    effective_following_cfg.max_leader_search_s =
        std::max(following_cfg_.max_leader_search_s, following_collision_horizon);

    // 직전 leader ID를 유지해 탐색거리 경계/패킷 누락에서 모드가
    // FOLLOWING<->LANE_KEEPING으로 번복되지 않게 한다.
    if (following_.active) {
        bool id_seen = false;
        bool leader_valid = false;
        const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
        for (const auto& obj : obstacles) {
            if (obj.id != following_.leader_id) continue;
            id_seen = true;
            double obj_s, obj_d, obj_s_dot;
            ProjectObjectToFrenet(ref_, obj, obj_s, obj_d, obj_s_dot);
            double gap = obj_s - ego_s;
            if (gap <= 0.0 && track_length > 0.0) gap += track_length;
            const bool same_direction = obj_s_dot > following_cfg_.min_leader_speed;
            const bool same_lane = std::fabs(obj_d) <= lane_width_ * 0.5;
            const bool in_range = gap > 0.0 &&
                gap <= effective_following_cfg.max_leader_search_s
                     + following_cfg_.exit_search_margin;
            if (same_direction && same_lane && in_range) {
                following_.leader_s = ego_s + gap;
                following_.leader_d = obj_d;
                following_.leader_speed = obj_s_dot;
                following_.leader_accel = 0.0;
                following_.missing_cycles = 0;
                leader_valid = true;
            }
            break;
        }

        if (!leader_valid) {
            if (id_seen) {
                // 관측됐지만 차선/방향/범위가 바뀐 것은 dropout이 아니므로 즉시 해제.
                following_ = FollowingContext{};
            } else if (++following_.missing_cycles <= following_cfg_.dropout_grace_cycles) {
                // ObjectInfo 일시 누락 동안은 직전 속도로 leader s를 예측.
                constexpr double kPlannerCycleDt = 0.05;  // main.cpp 20Hz
                following_.leader_s += following_.leader_speed * kPlannerCycleDt;
            } else {
                following_ = FollowingContext{};
            }
        }
    }

    // 유지 중인 ID만 보는 것으로 끝내면 합류 직후 실제로 더 가까워진 본선 차량을
    // 놓칠 수 있다. 매 cycle 동일차선 최인접 차량도 찾되, 작은 투영 흔들림으로
    // ID가 왕복하지 않도록 거리 hysteresis를 만족할 때만 인계한다.
    LeaderTarget& nearest_leader = out_nearest_leader;
    const bool nearest_found = FindLeader(
        ref_, start, obstacles, lane_width_, effective_following_cfg, nearest_leader);
    out_nearest_found = nearest_found;
    if (following_.active && nearest_found &&
        nearest_leader.id != following_.leader_id) {
        const double current_gap = following_.leader_s - ego_s;
        const double nearest_gap = nearest_leader.s - ego_s;
        if (nearest_gap + following_cfg_.leader_switch_margin < current_gap) {
            const int old_id = following_.leader_id;
            following_.leader_id = nearest_leader.id;
            following_.leader_s = nearest_leader.s;
            following_.leader_d = nearest_leader.d;
            following_.leader_speed = nearest_leader.speed;
            following_.leader_accel = nearest_leader.accel;
            following_.missing_cycles = 0;
            std::printf("[FrenetPlanner] leader handoff: old=%d new=%d "
                        "old_gap=%.2f new_gap=%.2f\n",
                        old_id, nearest_leader.id, current_gap, nearest_gap);
        }
    }

    if (!following_.active && nearest_found) {
            const LeaderTarget& leader = nearest_leader;
            following_.active = true;
            following_.leader_id = leader.id;
            following_.leader_s = leader.s;
            following_.leader_d = leader.d;
            following_.leader_speed = leader.speed;
            following_.leader_accel = leader.accel;
            following_.missing_cycles = 0;
            std::printf("[FrenetPlanner] leader acquired: id=%d gap=%.2f speed=%.2f "
                        "effective_search=%.2f\n",
                        leader.id, leader.s - ego_s, leader.speed,
                        effective_following_cfg.max_leader_search_s);
    }
}
