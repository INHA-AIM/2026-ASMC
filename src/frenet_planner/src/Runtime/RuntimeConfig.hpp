#pragma once

#include <string>

namespace frenet_planner_ros {

// ROS node 계층 설정. Frenet planner core의 FrenetConfig(Config/FrenetConfig.hpp)
// 와는 별개다 - 이쪽은 planning frequency/timeout/topic/v2 계약 검증 임계값처럼
// node 자신의 동작을 결정하는 값이고, core는 이 값들을 모른다.
//
// params.yaml의 runtime: 섹션에서 로드하며, Parameter Single-Source 원칙에
// 따라 C++ fallback default를 두지 않는다 - 키 누락/타입 오류는 로드 실패다.
struct RuntimeConfig {
    double planning_frequency_hz;

    double vehicle_state_timeout_s;
    double objects_timeout_s;
    double behavior_timeout_s;

    std::string vehicle_state_topic;
    std::string objects_topic;
    std::string behavior_topic;
    std::string behavior_command_v2_topic;

    std::string trajectory_topic;
    std::string feedback_topic;
    std::string feedback_v2_topic;
    std::string markers_topic;

    double stop_max_target_heading_error_rad;

    double avoid_projection_search_window;
    double avoid_max_target_lateral_offset;
    double avoid_max_target_heading_error_rad;

    double lane_change_projection_search_window;
    double lane_change_max_target_lateral_offset;
    double lane_change_max_target_heading_error_rad;
};

// params.yaml의 runtime: 섹션을 로드한다. 필수 key 누락, 타입 오류,
// NaN/Inf, range 위반(주기<=0, timeout<=0, heading error가 (0, pi] 밖)이면
// false를 반환하고 error_message에 원인을 남긴다. 이 경우 caller는 node
// 초기화를 실패시켜야 한다 - 어떤 필드도 값이 정의되지 않은 채 사용되지
// 않는다.
bool LoadRuntimeConfig(const std::string& yaml_path, RuntimeConfig& out,
                       std::string& error_message);

}  // namespace frenet_planner_ros
