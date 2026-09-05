// C0 regression: PlannerCommand는 aggregate 구조를 유지하면서도 어떤 방식으로
// 선언하든(cmd; / cmd{}) 모든 필드가 안전한 기본값을 갖는지 검증한다.
// 특히 mode가 미초기화 쓰레기 값이 되어 임의 모드로 진입하는 것을 막고,
// stop_position은 "미지정"을 0.0(유효한 s)이 아니라 NaN으로 구분한다.

#include "frenet_planner/global/global.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <type_traits>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

void CheckDefaults(const PlannerCommand& cmd, const std::string& prefix) {
    Expect(cmd.mode == LANE_KEEPING, prefix + ": default mode is LANE_KEEPING");

    Expect(cmd.target_lane == 0, prefix + ": target_lane keeps current lane");
    Expect(cmd.avoidance_d_offset == 0.0, prefix + ": avoidance_d_offset is zero");

    Expect(!cmd.has_target_speed, prefix + ": has_target_speed is false");
    Expect(cmd.target_speed == 0.0, prefix + ": target_speed is zero");
    Expect(!cmd.has_max_speed, prefix + ": has_max_speed is false");
    Expect(cmd.max_speed == 0.0, prefix + ": max_speed is zero");

    // 0.0은 유효한 Frenet s이므로 미지정 표현으로 쓸 수 없다.
    Expect(std::isnan(cmd.stop_position), prefix + ": stop_position defaults to NaN");
    Expect(!std::isfinite(cmd.stop_position),
           prefix + ": stop_position is not finite until commanded");

    // D2: FOLLOW의 선행차와 AVOID의 회피 장애물이 같은 필드를 쓴다.
    // BehaviorCommand가 둘 다 primary_object_id 하나로 주기 때문이다
    // (docs/fsm-frenet-interface-v2.md 3.3절). 대상이 없는 maneuver는 -1.
    Expect(cmd.target_object_id == -1, prefix + ": target_object_id is invalid id");
    Expect(cmd.leader_s == 0.0, prefix + ": leader_s is zero");
    Expect(cmd.leader_speed == 0.0, prefix + ": leader_speed is zero");
    Expect(cmd.leader_accel == 0.0, prefix + ": leader_accel is zero");
    Expect(cmd.time_gap == 0.0, prefix + ": time_gap is zero");
    Expect(cmd.min_gap == 0.0, prefix + ": min_gap is zero");
    Expect(cmd.gap_gain == 0.0, prefix + ": gap_gain is zero");

    Expect(cmd.merge_sa_id == -1, prefix + ": merge_sa_id is invalid id");
    Expect(cmd.merge_sb_id == -1, prefix + ": merge_sb_id is invalid id");
    Expect(cmd.merge_sa_s == 0.0 && cmd.merge_sa_speed == 0.0 &&
               cmd.merge_sa_accel == 0.0,
           prefix + ": merge sa scalars are zero");
    Expect(cmd.merge_sb_s == 0.0 && cmd.merge_sb_speed == 0.0 &&
               cmd.merge_sb_accel == 0.0,
           prefix + ": merge sb scalars are zero");
    Expect(cmd.merge_target_d == 0.0 && cmd.merge_conflict_s == 0.0 &&
               cmd.merge_stop_s == 0.0 && cmd.merge_entry_time == 0.0,
           prefix + ": merge geometry scalars are zero");

    Expect(!cmd.merge_gap_safe, prefix + ": merge_gap_safe is false");
    Expect(!cmd.merge_committed, prefix + ": merge_committed is false");
    Expect(!cmd.merge_crossing, prefix + ": merge_crossing is false");
    Expect(cmd.merge_type == MergeType::NONE, prefix + ": merge_type is NONE");

    // D6-1: 요청 방향의 기본값은 "미지정"이다. LEFT/RIGHT가 기본값이 되면
    // 방향을 주지 않은 AVOID가 한쪽으로만 회피하게 된다.
    Expect(cmd.requested_direction == LateralDirection::NONE,
           prefix + ": requested_direction defaults to NONE");
}

}  // namespace

int main() {
    // 생성자를 추가하면 aggregate initialization을 쓰는 기존 코드가 깨진다.
    static_assert(std::is_aggregate<PlannerCommand>::value,
                  "PlannerCommand must stay an aggregate");

    PlannerCommand plain;
    CheckDefaults(plain, "default-init");

    PlannerCommand braced{};
    CheckDefaults(braced, "value-init");

    // --- D2: target_object_id가 FOLLOW/AVOID 공용임을 명시 ---------------
    // 선택 (a) 공통 필드의 계약. 모드별 필드로 되돌리면 이 검사가 깨진다.
    {
        PlannerCommand follow{};
        follow.mode = FOLLOWING;
        follow.target_object_id = 7;
        Expect(follow.target_object_id == 7,
               "FOLLOW carries its leader in target_object_id");

        PlannerCommand avoid{};
        avoid.mode = AVOID;
        avoid.target_object_id = 7;
        Expect(avoid.target_object_id == 7,
               "AVOID carries its obstacle in the same target_object_id field");

        // 대상이 없는 maneuver는 기본값 -1을 유지해야 한다.
        for (BehaviorState mode : {LANE_KEEPING, STOP, EMERGENCY}) {
            PlannerCommand targetless{};
            targetless.mode = mode;
            Expect(targetless.target_object_id == -1,
                   "targetless maneuver keeps target_object_id at -1");
        }
    }

    if (g_failures == 0) {
        std::cout << "All PlannerCommand default tests passed\n";
        return 0;
    }
    return 1;
}
