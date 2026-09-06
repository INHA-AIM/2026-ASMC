/**
 * @file Follow.cpp
 * @brief 외부 FSM이 지정한 FOLLOW 대상의 확보/유지.
 *
 * 외부 FSM이 지정한 대상 객체(target_object_id)가 유효한지 검증하고
 * (ResolveExternalFollowTarget) FrenetPlanner::v2_following_ 상태를
 * 갱신하는 역할까지만 담당한다 - 어떤 종방향 후보 생성기를 쓸지
 * (Generator/CandidateCombiner의 ResolveManeuver)나 gap 계산은 이 파일의
 * 책임이 아니다.
 */

#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "frenet_planner/Maneuver/ManeuverUtils.hpp"

// 저속 fallback은 시간 대신 이동거리 기준 cosine smoothstep으로 d를 복귀시킨다.
// 양 끝의 d'가 0이므로 정지 근처에서도 급격한 횡이동과 곡률 spike가 생기지 않는다.
FrenetPlanner::FollowTargetStatus FrenetPlanner::ResolveExternalFollowTarget(
    double ego_s, int target_object_id,
    const std::vector<TrackedObject>& obstacles,
    FollowingContext& context) const {
    if (target_object_id < 0) return FollowTargetStatus::LOST;

    // 지정된 ID만 찾는다. "가장 가까운" 차량을 스스로 고르지 않는다 -
    // 대상 선택 권한은 FSM에 있다.
    const auto it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [target_object_id](const TrackedObject& object) {
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
