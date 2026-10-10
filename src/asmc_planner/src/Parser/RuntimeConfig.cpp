#include "Parser/RuntimeConfig.hpp"

#include <cmath>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace frenet_planner_ros {

namespace {

// dotted_key: "runtime/timeouts/vehicle_state" 같은 '/' 구분 경로.
// 누락 시 false를 반환하고 missing_key에 dotted_key를 채운다 - 어떤 필드도
// 조용히 기본값에 머무르지 않는다.
template <typename T>
bool RequireKey(const YAML::Node& root, const std::string& dotted_key, T& out,
                 std::string& missing_key) {
    YAML::Node node = YAML::Clone(root);
    size_t start = 0;
    while (true) {
        const size_t slash = dotted_key.find('/', start);
        const std::string key = dotted_key.substr(
            start, slash == std::string::npos ? std::string::npos : slash - start);
        if (!node[key]) {
            missing_key = dotted_key;
            return false;
        }
        node = node[key];
        if (slash == std::string::npos) break;
        start = slash + 1;
    }
    try {
        out = node.as<T>();
    } catch (const std::exception&) {
        missing_key = dotted_key;
        return false;
    }
    return true;
}

bool IsPositiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool IsValidHeadingErrorBound(double value) {
    return std::isfinite(value) && value > 0.0 && value <= M_PI;
}

}  // namespace

bool LoadRuntimeConfig(const std::string& yaml_path, RuntimeConfig& out,
                       std::string& error_message) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        error_message = "failed to load yaml '" + yaml_path + "': " + e.what();
        return false;
    }

    std::string missing_key;
    const bool ok =
        RequireKey(root, "runtime/planning_frequency_hz", out.planning_frequency_hz, missing_key) &&
        RequireKey(root, "runtime/timeouts/vehicle_state", out.vehicle_state_timeout_s, missing_key) &&
        RequireKey(root, "runtime/timeouts/objects", out.objects_timeout_s, missing_key) &&
        RequireKey(root, "runtime/timeouts/behavior", out.behavior_timeout_s, missing_key) &&
        RequireKey(root, "runtime/topics/vehicle_state", out.vehicle_state_topic, missing_key) &&
        RequireKey(root, "runtime/topics/objects", out.objects_topic, missing_key) &&
        RequireKey(root, "runtime/topics/behavior", out.behavior_topic, missing_key) &&
        RequireKey(root, "runtime/topics/behavior_command_v2", out.behavior_command_v2_topic, missing_key) &&
        RequireKey(root, "runtime/publish_topics/trajectory", out.trajectory_topic, missing_key) &&
        RequireKey(root, "runtime/publish_topics/feedback", out.feedback_topic, missing_key) &&
        RequireKey(root, "runtime/publish_topics/feedback_v2", out.feedback_v2_topic, missing_key) &&
        RequireKey(root, "runtime/publish_topics/markers", out.markers_topic, missing_key) &&
        RequireKey(root, "runtime/interface_validation/stop/max_target_heading_error_rad",
                   out.stop_max_target_heading_error_rad, missing_key) &&
        RequireKey(root, "runtime/interface_validation/avoid/projection_search_window",
                   out.avoid_projection_search_window, missing_key) &&
        RequireKey(root, "runtime/interface_validation/avoid/max_target_lateral_offset",
                   out.avoid_max_target_lateral_offset, missing_key) &&
        RequireKey(root, "runtime/interface_validation/avoid/max_target_heading_error_rad",
                   out.avoid_max_target_heading_error_rad, missing_key) &&
        RequireKey(root, "runtime/interface_validation/lane_change/projection_search_window",
                   out.lane_change_projection_search_window, missing_key) &&
        RequireKey(root, "runtime/interface_validation/lane_change/max_target_lateral_offset",
                   out.lane_change_max_target_lateral_offset, missing_key) &&
        RequireKey(root, "runtime/interface_validation/lane_change/max_target_heading_error_rad",
                   out.lane_change_max_target_heading_error_rad, missing_key);

    if (!ok) {
        error_message = "missing or invalid required runtime key: " + missing_key;
        return false;
    }

    if (!IsPositiveFinite(out.planning_frequency_hz)) {
        error_message = "runtime/planning_frequency_hz must be > 0";
        return false;
    }
    if (!IsPositiveFinite(out.vehicle_state_timeout_s) ||
        !IsPositiveFinite(out.objects_timeout_s) ||
        !IsPositiveFinite(out.behavior_timeout_s)) {
        error_message = "runtime/timeouts/* must all be > 0";
        return false;
    }
    if (!IsPositiveFinite(out.avoid_projection_search_window) ||
        !IsPositiveFinite(out.avoid_max_target_lateral_offset) ||
        !IsPositiveFinite(out.lane_change_projection_search_window) ||
        !IsPositiveFinite(out.lane_change_max_target_lateral_offset)) {
        error_message =
            "runtime/interface_validation/*/projection_search_window and "
            "max_target_lateral_offset must all be > 0";
        return false;
    }
    if (!IsValidHeadingErrorBound(out.stop_max_target_heading_error_rad) ||
        !IsValidHeadingErrorBound(out.avoid_max_target_heading_error_rad) ||
        !IsValidHeadingErrorBound(out.lane_change_max_target_heading_error_rad)) {
        error_message =
            "runtime/interface_validation/*/max_target_heading_error_rad must be in (0, pi]";
        return false;
    }

    return true;
}

}  // namespace frenet_planner_ros
