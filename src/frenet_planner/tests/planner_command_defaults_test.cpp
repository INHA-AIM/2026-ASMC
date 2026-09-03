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

    Expect(cmd.leader_id == -1, prefix + ": leader_id is invalid id");
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

    if (g_failures == 0) {
        std::cout << "All PlannerCommand default tests passed\n";
        return 0;
    }
    return 1;
}
