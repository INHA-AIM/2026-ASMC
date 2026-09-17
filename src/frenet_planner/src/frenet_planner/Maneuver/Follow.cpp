/**
 * @file Follow.cpp
 * @brief 외부 FSM이 지정한 FOLLOW 대상의 확보/유지와 FollowProcess 진입점.
 *
 * 외부 FSM이 지정한 대상 객체(target_object_id)가 유효한지 검증하고
 * (ResolveExternalFollowTarget) FrenetPlanner::v2_following_ 상태를
 * 갱신한다. FollowProcess는 이 확보 결과로 leader_s/leader_speed/
 * leader_accel/time_gap/min_gap/gap_gain을 채운 뒤, KEEP/FOLLOW/STOP이
 * 공유하는 candidate 파이프라인(Keep.cpp의 ExecuteManeuverPipeline)에
 * 위임한다 - 어떤 종방향 후보 생성기를 쓸지(Generator/CandidateCombiner의
 * ResolveManeuver)나 gap 계산 자체는 이 파일의 책임이 아니다.
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
        ProjectObjectToFrenet(ref_, *it, object_s, object_d, object_s_dot,
                             ref_config_.ego_projection_search_window);

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
    context.leader_s += context.leader_speed * replanning_period_s_;

    return FollowTargetStatus::GRACE;
}

bool FrenetPlanner::FollowProcess(
    const CartesianState& ego, const FrenetState& start,
    double s, double d, double s_dot, double s_ddot,
    double d_dot, double d_ddot,
    const std::vector<TrackedObject>& obstacles,
    const std::vector<TrackedObject>& predicted_obstacles,
    double resolved_target_speed, const PlannerCommand& external_command,
    CartesianPath& out_path) {
    // 외부 FOLLOW: FSM이 지정한 대상만 확보한다. 자동 선택/handoff를 타지
    // 않는다(계약 D1 0.1절 B 분류). 확보에 실패하면 공용 pipeline을 부르지
    // 않고 여기서 바로 실패를 보고한다.
    PlannerCommand cmd = external_command;
    const FollowTargetStatus status = ResolveExternalFollowTarget(
        s, cmd.target_object_id, obstacles, v2_following_);
    if (status == FollowTargetStatus::LOST) {
        // 대상을 쓸 수 없다. KEEP으로 자동 전환하지 않는다 - 다음
        // 행동은 FSM이 새 command_id로 결정한다(계약 12.5절 D5).
        last_mode_ = FOLLOWING;
        last_failure_reason_ = PlanFailureReason::TARGET_OBJECT_LOST;
        return false;
    }
    // VALID / GRACE 모두 확보된 상태를 후보 생성기에 전달한다.
    cmd.leader_s = v2_following_.leader_s;
    cmd.leader_speed = v2_following_.leader_speed;
    cmd.leader_accel = v2_following_.leader_accel;
    // 계약 11.8절: adapter는 custom gap이 꺼져 있으면 0.0 sentinel을
    // 넘긴다. config 기본값 적용은 이 소비 지점에서 한다 - 변환이
    // config에 의존하면 "FSM이 준 값"과 "Frenet 기본값"을 피드백에서
    // 구분할 수 없다.
    if (!(cmd.time_gap > 0.0)) cmd.time_gap = following_cfg_.time_gap;
    if (!(cmd.min_gap > 0.0)) cmd.min_gap = following_cfg_.min_gap;
    cmd.gap_gain = following_cfg_.gap_gain;

    return ExecuteManeuverPipeline(
        ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
        predicted_obstacles, resolved_target_speed, cmd, out_path);
}
