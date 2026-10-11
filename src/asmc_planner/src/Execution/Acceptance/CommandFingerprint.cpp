#include "Execution/Acceptance/CommandFingerprint.hpp"
#include <cmath>
namespace frenet_planner_ros {
namespace {
// 계약 4.4절의 허용오차. **fingerprint 비교 전용**이다.
// validation의 sentinel 검사(-1.0)에는 쓰지 않는다 - 허용오차를 쓰면
// -0.9999995가 -1.0으로 인정되어 계약이 금지한 구간이 통과한다.
constexpr double kScalarTolerance = 1e-6;      // 속도, gap
constexpr double kPositionTolerance = 1e-3;    // [m]
constexpr double kOrientationTolerance = 1e-3;

bool NearlyEqual(double lhs, double rhs, double tolerance) {
    // 두 값이 모두 무한/NaN인 경우까지 같다고 볼 필요는 없다. validation을
    // 통과한 명령에는 비유한 값이 없다.
    return std::fabs(lhs - rhs) <= tolerance;
}

}
CommandFingerprint MakeCommandFingerprint(
    const asmc_msgs::BehaviorCommand& message) {
    CommandFingerprint fingerprint;
    fingerprint.maneuver_type = message.maneuver_type;
    fingerprint.direction = message.direction;
    fingerprint.merge_type = message.merge_type;
    fingerprint.source_link_id = message.source_link_id;
    fingerprint.target_link_id = message.target_link_id;
    fingerprint.has_target_pose = message.has_target_pose;
    if (message.has_target_pose) {
        fingerprint.target_position_x = message.target_pose_map.position.x;
        fingerprint.target_position_y = message.target_pose_map.position.y;
        fingerprint.target_position_z = message.target_pose_map.position.z;
        fingerprint.target_orientation_x = message.target_pose_map.orientation.x;
        fingerprint.target_orientation_y = message.target_pose_map.orientation.y;
        fingerprint.target_orientation_z = message.target_pose_map.orientation.z;
        fingerprint.target_orientation_w = message.target_pose_map.orientation.w;
    }
    fingerprint.desired_speed_mps = message.desired_speed_mps;
    fingerprint.primary_object_id = message.primary_object_id;
    fingerprint.preferred_front_object_id = message.preferred_front_object_id;
    fingerprint.preferred_rear_object_id = message.preferred_rear_object_id;
    fingerprint.use_custom_follow_gap = message.use_custom_follow_gap;
    fingerprint.desired_time_gap_s = message.desired_time_gap_s;
    fingerprint.minimum_gap_m = message.minimum_gap_m;
    return fingerprint;
}

bool SameCommandFingerprint(const CommandFingerprint& lhs,
                            const CommandFingerprint& rhs) {
    if (lhs.maneuver_type != rhs.maneuver_type) return false;
    if (lhs.direction != rhs.direction) return false;
    if (lhs.merge_type != rhs.merge_type) return false;
    if (lhs.source_link_id != rhs.source_link_id) return false;
    if (lhs.target_link_id != rhs.target_link_id) return false;
    if (lhs.has_target_pose != rhs.has_target_pose) return false;
    if (lhs.primary_object_id != rhs.primary_object_id) return false;
    if (lhs.preferred_front_object_id != rhs.preferred_front_object_id) return false;
    if (lhs.preferred_rear_object_id != rhs.preferred_rear_object_id) return false;
    if (lhs.use_custom_follow_gap != rhs.use_custom_follow_gap) return false;

    if (!NearlyEqual(lhs.desired_speed_mps, rhs.desired_speed_mps,
                     kScalarTolerance)) return false;
    if (!NearlyEqual(lhs.desired_time_gap_s, rhs.desired_time_gap_s,
                     kScalarTolerance)) return false;
    if (!NearlyEqual(lhs.minimum_gap_m, rhs.minimum_gap_m,
                     kScalarTolerance)) return false;

    // target pose는 유효할 때만 비교한다. has_target_pose=false이면 pose
    // 필드가 채워지지 않으므로(MakeCommandFingerprint 참고) 비교가 무의미하다.
    if (lhs.has_target_pose) {
        if (!NearlyEqual(lhs.target_position_x, rhs.target_position_x,
                         kPositionTolerance)) return false;
        if (!NearlyEqual(lhs.target_position_y, rhs.target_position_y,
                         kPositionTolerance)) return false;
        if (!NearlyEqual(lhs.target_position_z, rhs.target_position_z,
                         kPositionTolerance)) return false;
        if (!NearlyEqual(lhs.target_orientation_x, rhs.target_orientation_x,
                         kOrientationTolerance)) return false;
        if (!NearlyEqual(lhs.target_orientation_y, rhs.target_orientation_y,
                         kOrientationTolerance)) return false;
        if (!NearlyEqual(lhs.target_orientation_z, rhs.target_orientation_z,
                         kOrientationTolerance)) return false;
        if (!NearlyEqual(lhs.target_orientation_w, rhs.target_orientation_w,
                         kOrientationTolerance)) return false;
    }
    return true;
}

}
