#pragma once
#include <cstdint>
#include <string>
#include <asmc_msgs/BehaviorCommand.h>
namespace frenet_planner_ros {
struct CommandFingerprint {
    uint8_t maneuver_type = 0;
    uint8_t direction = 0;
    uint8_t merge_type = 0;

    std::string source_link_id;
    std::string target_link_id;

    bool has_target_pose = false;
    // has_target_pose=false이면 비교하지 않는다.
    double target_position_x = 0.0;
    double target_position_y = 0.0;
    double target_position_z = 0.0;
    double target_orientation_x = 0.0;
    double target_orientation_y = 0.0;
    double target_orientation_z = 0.0;
    double target_orientation_w = 0.0;

    double desired_speed_mps = 0.0;

    int32_t primary_object_id = -1;
    int32_t preferred_front_object_id = -1;
    int32_t preferred_rear_object_id = -1;

    bool use_custom_follow_gap = false;
    double desired_time_gap_s = 0.0;
    double minimum_gap_m = 0.0;
};

CommandFingerprint MakeCommandFingerprint(
    const asmc_msgs::BehaviorCommand& message);

// 두 fingerprint가 같은 명령을 나타내는지.
//
// 실수 필드는 정확한 동등 비교를 하지 않는다. FSM이 같은 값을 재발행할 때
// 부동소수 표현이 동일할 것으로 가정하지 않기 위함이다(계약 4.4절).
//   속도·gap        1e-6
//   pose position   1e-3 [m]
//   pose orientation 1e-3
bool SameCommandFingerprint(const CommandFingerprint& lhs,
                            const CommandFingerprint& rhs);

}
