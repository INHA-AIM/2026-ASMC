#include <PlannerInterface/PlannerObjectRosPublisher.hpp>

#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>

#include <Msg/TrackedObject.h>
#include <Msg/TrackedObjectArray.h>
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

std::string JsonEscape(const std::string& value) {
    std::ostringstream escaped;
    for (const unsigned char character : value) {
        switch (character) {
            case '\"':
                escaped << "\\\"";
                break;
            case '\\':
                escaped << "\\\\";
                break;
            case '\b':
                escaped << "\\b";
                break;
            case '\f':
                escaped << "\\f";
                break;
            case '\n':
                escaped << "\\n";
                break;
            case '\r':
                escaped << "\\r";
                break;
            case '\t':
                escaped << "\\t";
                break;
            default:
                if (character < 0x20) {
                    escaped << "\\u" << std::hex << std::setw(4)
                            << std::setfill('0')
                            << static_cast<int>(character) << std::dec;
                } else {
                    escaped << static_cast<char>(character);
                }
        }
    }
    return escaped.str();
}

void AppendMotionDebugObject(std::ostringstream& stream,
                             const PlannerObject& object,
                             const DrivableFilterDecision& boundary_filter,
                             const KnownStaticDecision& static_filter,
                             const StaticStructureExclusionDecision&
                                 configured_static_filter,
                             const PlanningRelevanceDecision& relevance,
                             const PlannerObject& planning_object,
                             bool geometry_clipped) {
    const auto motion_state = static_cast<MotionState>(object.motion_state);
    const bool final_keep = relevance.keep;
    const char* filter_stage = "keep";
    const std::string* final_reason = &boundary_filter.reason;
    if (!boundary_filter.keep) {
        filter_stage = "drivable";
    } else if (!static_filter.keep) {
        filter_stage = "known_static";
        final_reason = &static_filter.reason;
    } else if (!configured_static_filter.keep) {
        filter_stage = "static_zone";
        final_reason = &configured_static_filter.reason;
    } else if (!relevance.keep) {
        filter_stage = "planning_relevance";
        final_reason = &relevance.reason;
    } else if (relevance.policy_applied) {
        final_reason = &relevance.reason;
    } else if (configured_static_filter.policy_applied) {
        final_reason = &configured_static_filter.reason;
    } else if (static_filter.policy_applied) {
        final_reason = &static_filter.reason;
    }
    stream << "{\"id\":" << object.id
           << ",\"motion_state\":\""
           << MotionStateName(motion_state) << "\""
           << ",\"motion_state_code\":" << object.motion_state
           << ",\"is_dynamic\":"
           << (object.is_dynamic ? "true" : "false")
           << ",\"classification\":\""
           << JsonEscape(object.classification) << "\""
           << ",\"yolo_class\":\""
           << JsonEscape(object.yolo_class) << "\""
           << ",\"center_x\":" << object.obb_center_map.x
           << ",\"center_y\":" << object.obb_center_map.y
           << ",\"length\":" << object.length
           << ",\"width\":" << object.width
           << ",\"height\":" << object.height
           << ",\"ground_evidence_valid\":"
           << (object.ground_evidence.valid ? "true" : "false")
           << ",\"ground_sample_count\":" << object.ground_evidence.sample_count
           << ",\"ground_median_clearance_m\":"
           << object.ground_evidence.median_clearance_m
           << ",\"ground_rejection_band_m\":"
           << object.ground_evidence.rejection_band_m
           << ",\"obb_corners\":[";
    for (std::size_t i = 0; i < object.obb_corners_map.size(); ++i) {
        if (i > 0) stream << ',';
        stream << '[' << object.obb_corners_map[i].x << ','
               << object.obb_corners_map[i].y << ']';
    }
    stream << ']'
           << ",\"age\":" << object.age
           << ",\"miss_count\":" << object.miss_count
           << ",\"valid_observations\":"
           << object.motion_valid_observations
           << ",\"accepted_samples\":"
           << object.motion_accepted_samples
           << ",\"candidate_samples\":"
           << object.motion_candidate_samples
           << ",\"raw_speed_kph\":" << object.raw_speed_kph
           << ",\"raw_lidar_vx_kph\":"
           << object.raw_velocity_lidar_mps.x * 3.6
           << ",\"raw_lidar_vy_kph\":"
           << object.raw_velocity_lidar_mps.y * 3.6
           << ",\"raw_map_vx_mps\":"
           << object.raw_velocity_map_mps.x
           << ",\"raw_map_vy_mps\":"
           << object.raw_velocity_map_mps.y
           << ",\"decision_speed_kph\":"
           << object.decision_speed_kph
           << ",\"decision_vx_kph\":" << object.decision_vx_kph
           << ",\"decision_vy_kph\":" << object.decision_vy_kph
           << ",\"applied_speed_kph\":" << object.speed_kph
           << ",\"applied_map_vx_mps\":"
           << object.velocity_map_mps.x
           << ",\"applied_map_vy_mps\":"
           << object.velocity_map_mps.y
           << ",\"map_displacement_speed_kph\":"
           << object.map_displacement_speed_kph
           << ",\"corrected_displacement_speed_kph\":"
           << object.corrected_displacement_speed_kph
           << ",\"velocity_jump_kph\":"
           << object.motion_velocity_jump_kph
           << ",\"road_ratio\":" << boundary_filter.road_ratio
           << ",\"predicted_road_ratio\":"
           << boundary_filter.predicted_road_ratio
           << ",\"boundary_filter_available\":"
           << (boundary_filter.filter_available ? "true" : "false")
           << ",\"boundary_keep\":"
           << (boundary_filter.keep ? "true" : "false")
           << ",\"boundary_reason\":\""
           << JsonEscape(boundary_filter.reason) << "\""
           << ",\"known_static_filter_available\":"
           << (static_filter.filter_available ? "true" : "false")
           << ",\"known_static_keep\":"
           << (static_filter.keep ? "true" : "false")
           << ",\"known_static_reason\":\""
           << JsonEscape(static_filter.reason) << "\""
           << ",\"known_static_reference_id\":"
           << static_filter.reference_track_id
           << ",\"known_static_center_distance_m\":"
           << static_filter.center_distance_m
           << ",\"configured_static_filter_available\":"
           << (configured_static_filter.filter_available ? "true" : "false")
           << ",\"configured_static_keep\":"
           << (configured_static_filter.keep ? "true" : "false")
           << ",\"configured_static_policy_applied\":"
           << (configured_static_filter.policy_applied ? "true" : "false")
           << ",\"configured_static_reason\":\""
           << JsonEscape(configured_static_filter.reason) << "\""
           << ",\"configured_static_zone\":\""
           << JsonEscape(configured_static_filter.zone_name) << "\""
           << ",\"planning_relevance_keep\":"
           << (relevance.keep ? "true" : "false")
           << ",\"planning_relevance_applied\":"
           << (relevance.policy_applied ? "true" : "false")
           << ",\"elevated_ground_evidence\":"
           << (relevance.elevated_ground_evidence ? "true" : "false")
           << ",\"planning_relevance_reason\":\""
           << JsonEscape(relevance.reason) << "\""
           << ",\"height_confirmations\":"
           << relevance.height_confirmations
           << ",\"planning_confirmed\":"
           << (relevance.planning_confirmed ? "true" : "false")
           << ",\"good_height_frames\":"
           << relevance.good_height_frames
           << ",\"bad_height_frames\":"
           << relevance.bad_height_frames
           << ",\"planning_release_frames\":"
           << relevance.planning_release_frames
           << ",\"retained_by_height_grace\":"
           << (relevance.retained_by_height_grace ? "true" : "false")
           << ",\"emergency_override\":"
           << (relevance.emergency_override ? "true" : "false")
           << ",\"obstacle_clearance_m\":"
           << relevance.obstacle_clearance_m
           << ",\"emergency_distance_m\":"
           << relevance.emergency_distance_m
           << ",\"geometry_clipped\":"
           << (geometry_clipped ? "true" : "false")
           << ",\"planning_obb_corners\":[";
    for (std::size_t i = 0;
         i < planning_object.obb_corners_map.size(); ++i) {
        if (i > 0) stream << ',';
        stream << '[' << planning_object.obb_corners_map[i].x << ','
               << planning_object.obb_corners_map[i].y << ']';
    }
    stream << ']'
           << ",\"final_keep\":" << (final_keep ? "true" : "false")
           << ",\"filter_stage\":\"" << filter_stage << "\""
           << ",\"filter_reason\":\""
           << JsonEscape(*final_reason) << "\"}";
}

geometry_msgs::Point ToRosPoint(const PlannerPoint3D& point) {
    geometry_msgs::Point result;
    result.x = point.x;
    result.y = point.y;
    result.z = point.z;
    return result;
}

std_msgs::Header MapHeader(const ros::Time& stamp) {
    std_msgs::Header header;
    header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    header.frame_id = "map";
    return header;
}

visualization_msgs::Marker BaseMarker(
    const std_msgs::Header& header, const std::string& marker_namespace,
    int id, int type) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.3);
    return marker;
}

Msg::TrackedObject ToRosObject(const PlannerObject& object) {
    Msg::TrackedObject message;
    message.id = object.id;
    message.type = object.type;
    message.classification = object.classification;
    message.yolo_class = object.yolo_class;
    message.is_dynamic = object.is_dynamic;
    message.age = object.age;
    message.miss_count = object.miss_count;
    message.cluster_center = ToRosPoint(object.cluster_center_map);
    message.obb_center = ToRosPoint(object.obb_center_map);
    for (std::size_t i = 0; i < object.obb_corners_map.size(); ++i) {
        message.obb_corners[i] = ToRosPoint(object.obb_corners_map[i]);
    }
    message.obb_yaw = object.obb_yaw_map_rad;
    message.velocity_mps.x = object.velocity_map_mps.x;
    message.velocity_mps.y = object.velocity_map_mps.y;
    message.velocity_mps.z = object.velocity_map_mps.z;
    message.speed_kph = object.speed_kph;
    message.length = object.length;
    message.width = object.width;
    message.height = object.height;
    return message;
}

visualization_msgs::Marker MakeObbMarker(
    const PlannerObject& object, const std_msgs::Header& header) {
    auto marker = BaseMarker(
        header, "map_obb", object.id,
        visualization_msgs::Marker::LINE_LIST);
    marker.scale.x = 0.06;
    marker.color.r = 1.0f;
    marker.color.g = 0.9f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    static constexpr std::array<std::pair<int, int>, 12> kEdges{{
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    }};
    for (const auto& edge : kEdges) {
        marker.points.push_back(ToRosPoint(object.obb_corners_map[edge.first]));
        marker.points.push_back(ToRosPoint(object.obb_corners_map[edge.second]));
    }
    return marker;
}

visualization_msgs::Marker MakeCenterMarker(
    const PlannerObject& object, const std_msgs::Header& header) {
    auto marker = BaseMarker(
        header, "map_center", object.id,
        visualization_msgs::Marker::SPHERE);
    marker.pose.position = ToRosPoint(object.obb_center_map);
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 0.1f;
    marker.color.g = 0.7f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    return marker;
}

visualization_msgs::Marker MakeVelocityMarker(
    const PlannerObject& object, const std_msgs::Header& header,
    double arrow_seconds) {
    auto marker = BaseMarker(
        header, "map_velocity", object.id,
        visualization_msgs::Marker::ARROW);
    marker.points.push_back(ToRosPoint(object.obb_center_map));
    PlannerPoint3D end = object.obb_center_map;
    end.x += object.velocity_map_mps.x * arrow_seconds;
    end.y += object.velocity_map_mps.y * arrow_seconds;
    end.z += object.velocity_map_mps.z * arrow_seconds;
    marker.points.push_back(ToRosPoint(end));
    marker.scale.x = 0.10;
    marker.scale.y = 0.22;
    marker.scale.z = 0.28;
    marker.color.r = object.is_dynamic ? 0.1f : 0.2f;
    marker.color.g = object.is_dynamic ? 1.0f : 0.6f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0f;
    return marker;
}

visualization_msgs::Marker MakeTextMarker(
    const PlannerObject& object, const std_msgs::Header& header) {
    auto marker = BaseMarker(
        header, "map_text", object.id,
        visualization_msgs::Marker::TEXT_VIEW_FACING);
    marker.pose.position = ToRosPoint(object.obb_center_map);
    marker.pose.position.z += 0.5 * object.height + 0.8;
    marker.scale.z = 0.55;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    char text[256];
    std::snprintf(
        text, sizeof(text),
        "ID:%d %s age:%d miss:%d\n%.2f km/h  vE:%.2f vN:%.2f m/s",
        object.id, object.classification.c_str(), object.age,
        object.miss_count, object.speed_kph,
        object.velocity_map_mps.x, object.velocity_map_mps.y);
    marker.text = text;
    return marker;
}

visualization_msgs::Marker MakeLastObbMarker(
    const PlannerObject& object, const std_msgs::Header& header) {
    auto marker = BaseMarker(
        header, "obb", object.id, visualization_msgs::Marker::LINE_LIST);
    marker.scale.x = 0.06;
    marker.color.r = object.is_dynamic ? 1.0f : 0.0f;
    marker.color.g = object.is_dynamic ? 0.1f : 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    static constexpr std::array<std::pair<int, int>, 12> kEdges{{
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    }};
    for (const auto& edge : kEdges) {
        marker.points.push_back(ToRosPoint(
            object.obb_corners_lidar[edge.first]));
        marker.points.push_back(ToRosPoint(
            object.obb_corners_lidar[edge.second]));
    }
    return marker;
}

visualization_msgs::Marker MakeLastVelocityMarker(
    const PlannerObject& object, const std_msgs::Header& header) {
    auto marker = BaseMarker(
        header, "velocity", object.id,
        visualization_msgs::Marker::ARROW);
    marker.points.push_back(ToRosPoint(object.obb_center_lidar));
    PlannerPoint3D end = object.obb_center_lidar;
    const double norm = std::hypot(
        object.velocity_lidar_mps.x, object.velocity_lidar_mps.y);
    if (norm > 1e-6) {
        end.x += 0.5 * object.velocity_lidar_mps.x / norm;
        end.y += 0.5 * object.velocity_lidar_mps.y / norm;
    }
    marker.points.push_back(ToRosPoint(end));
    marker.scale.x = 0.10;
    marker.scale.y = 0.22;
    marker.scale.z = 0.28;
    marker.color.r = 0.1f;
    marker.color.g = 0.7f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    return marker;
}

visualization_msgs::Marker MakeLastTextMarker(
    const PlannerObject& object, const std_msgs::Header& header,
    double road_ratio) {
    auto marker = BaseMarker(
        header, "info", object.id,
        visualization_msgs::Marker::TEXT_VIEW_FACING);
    marker.pose.position = ToRosPoint(object.obb_center_lidar);
    marker.pose.position.z += 0.5 * object.height + 0.8;
    marker.scale.z = 0.48;
    marker.color.r = object.is_dynamic ? 1.0f : 0.0f;
    marker.color.g = object.is_dynamic ? 0.2f : 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    const auto motion_state = static_cast<MotionState>(object.motion_state);
    char text[512];
    std::snprintf(
        text, sizeof(text),
        "ID:%d %s age:%d n:%d\n"
        "class:%s yolo:%s\n"
        "raw |v|:%.2f x:%.2f y:%.2f km/h\n"
        "judge |v|:%.2f x:%.2f y:%.2f km/h\n"
        "applied |v|:%.2f km/h map:%.2f/%.2f\n"
        "ENU:(%.2f, %.2f)\nroad:%.2f",
        object.id, MotionStateName(motion_state), object.age,
        object.motion_accepted_samples,
        object.classification.c_str(), object.yolo_class.c_str(),
        object.raw_speed_kph,
        object.raw_velocity_lidar_mps.x * 3.6,
        object.raw_velocity_lidar_mps.y * 3.6,
        object.decision_speed_kph,
        object.decision_vx_kph,
        object.decision_vy_kph,
        object.speed_kph, object.map_displacement_speed_kph,
        object.corrected_displacement_speed_kph,
        object.obb_center_map.x, object.obb_center_map.y, road_ratio);
    marker.text = text;
    return marker;
}

}  // namespace

PlannerObjectRosPublisher::PlannerObjectRosPublisher(
    ros::NodeHandle& node_handle) {
    ros::NodeHandle private_node("~");
    int max_objects = static_cast<int>(max_objects_);
    private_node.param("map_output_max_objects", max_objects, max_objects);
    private_node.param(
        "velocity_arrow_seconds", velocity_arrow_seconds_,
        velocity_arrow_seconds_);
    double drivable_ratio_threshold = 0.15;
    int drivable_samples = 5;
    private_node.param(
        "drivable_ratio_threshold", drivable_ratio_threshold,
        drivable_ratio_threshold);
    private_node.param(
        "drivable_samples", drivable_samples, drivable_samples);
    max_objects_ = static_cast<std::size_t>(std::max(1, max_objects));
    velocity_arrow_seconds_ = std::max(0.05, velocity_arrow_seconds_);

    const std::string default_output =
        ros::package::getPath("Main") + "/../../../planning_inputs/config/validated_0781402/";
    std::string mask_image = default_output + "road_mask.png";
    std::string mask_metadata = default_output + "road_mask.yaml";
    std::string expected_map_id = "waize_katri_2025-10-23";
    double expected_origin_easting = 302595.0;
    double expected_origin_northing = 4124145.0;
    double expected_origin_altitude = 0.0;
    double margin_m = 0.0;
    bool drivable_filter_enabled = true;
    private_node.param(
        "drivable_filter_enabled", drivable_filter_enabled,
        drivable_filter_enabled);
    private_node.param("drivable_mask_image", mask_image, mask_image);
    private_node.param("drivable_mask_metadata", mask_metadata, mask_metadata);
    private_node.param("drivable_map_id", expected_map_id, expected_map_id);
    private_node.param(
        "drivable_origin_easting", expected_origin_easting,
        expected_origin_easting);
    private_node.param(
        "drivable_origin_northing", expected_origin_northing,
        expected_origin_northing);
    private_node.param(
        "drivable_origin_altitude", expected_origin_altitude,
        expected_origin_altitude);
    private_node.param("drivable_margin_m", margin_m, margin_m);
    if (!drivable_filter_enabled) {
        ROS_WARN(
            "Drivable boundary filter explicitly disabled; fail-safe KEEP for every object");
    } else if (!drivable_filter_.Configure(
            mask_image, mask_metadata, expected_map_id,
            {{expected_origin_easting, expected_origin_northing,
              expected_origin_altitude}},
            margin_m, drivable_ratio_threshold, drivable_samples)) {
        ROS_ERROR(
            "Drivable boundary filter disabled; fail-safe KEEP for every object: %s",
            drivable_filter_.status_message().c_str());
    } else {
        ROS_INFO(
            "Drivable boundary filter ready: map=%s margin=%.2fm threshold=%.2f supersample=%d policy=road-overlap-with-motion-prediction",
            drivable_filter_.loaded_map_id().c_str(), margin_m,
            std::max(0.0, std::min(1.0, drivable_ratio_threshold)),
            std::max(1, drivable_samples));
    }

    double planning_minimum_obstacle_height_m = 0.10;
    int planning_confirmation_frames = 3;
    int planning_release_frames = 5;
    private_node.param(
        "planning_minimum_obstacle_height_m",
        planning_minimum_obstacle_height_m,
        planning_minimum_obstacle_height_m);
    private_node.param(
        "planning_confirmation_frames",
        planning_confirmation_frames,
        planning_confirmation_frames);
    private_node.param(
        "planning_release_frames",
        planning_release_frames,
        planning_release_frames);
    planning_relevance_filter_.Configure(
        planning_minimum_obstacle_height_m,
        planning_confirmation_frames,
        planning_release_frames);
    ROS_INFO(
        "Planning relevance filter: minimum_height=%.2fm confirmation_frames=%d release_frames=%d policy=strict-acquire/hysteretic-release",
        planning_relevance_filter_.minimum_obstacle_height_m(),
        planning_relevance_filter_.confirmation_frames(),
        planning_relevance_filter_.release_frames());

    bool known_static_filter_enabled = true;
    std::string known_static_catalog =
        ros::package::getPath("Main") + "/known_static_objects.yaml";
    double known_static_center_tolerance_m = 2.5;
    bool known_static_application_region_enabled = true;
    double known_static_application_min_x = -35.0;
    double known_static_application_max_x = 15.0;
    double known_static_application_min_y = 108.0;
    double known_static_application_max_y = 180.0;
    bool known_static_guardrail_extension_enabled = true;
    double known_static_guardrail_extension_min_x = -107.0;
    double known_static_guardrail_extension_max_x = 15.0;
    double known_static_guardrail_extension_min_y = 175.0;
    double known_static_guardrail_extension_max_y = 345.0;
    private_node.param(
        "known_static_filter_enabled", known_static_filter_enabled,
        known_static_filter_enabled);
    private_node.param(
        "known_static_catalog", known_static_catalog, known_static_catalog);
    private_node.param(
        "known_static_center_tolerance_m", known_static_center_tolerance_m,
        known_static_center_tolerance_m);
    private_node.param(
        "known_static_application_region_enabled",
        known_static_application_region_enabled,
        known_static_application_region_enabled);
    private_node.param(
        "known_static_application_min_x", known_static_application_min_x,
        known_static_application_min_x);
    private_node.param(
        "known_static_application_max_x", known_static_application_max_x,
        known_static_application_max_x);
    private_node.param(
        "known_static_application_min_y", known_static_application_min_y,
        known_static_application_min_y);
    private_node.param(
        "known_static_application_max_y", known_static_application_max_y,
        known_static_application_max_y);
    private_node.param(
        "known_static_guardrail_extension_enabled",
        known_static_guardrail_extension_enabled,
        known_static_guardrail_extension_enabled);
    private_node.param(
        "known_static_guardrail_extension_min_x",
        known_static_guardrail_extension_min_x,
        known_static_guardrail_extension_min_x);
    private_node.param(
        "known_static_guardrail_extension_max_x",
        known_static_guardrail_extension_max_x,
        known_static_guardrail_extension_max_x);
    private_node.param(
        "known_static_guardrail_extension_min_y",
        known_static_guardrail_extension_min_y,
        known_static_guardrail_extension_min_y);
    private_node.param(
        "known_static_guardrail_extension_max_y",
        known_static_guardrail_extension_max_y,
        known_static_guardrail_extension_max_y);
    std::vector<KnownStaticApplicationRegion> known_static_regions;
    if (known_static_application_region_enabled) {
        known_static_regions.push_back({
            known_static_application_min_x,
            known_static_application_max_x,
            known_static_application_min_y,
            known_static_application_max_y});
    }
    if (known_static_guardrail_extension_enabled) {
        known_static_regions.push_back({
            known_static_guardrail_extension_min_x,
            known_static_guardrail_extension_max_x,
            known_static_guardrail_extension_min_y,
            known_static_guardrail_extension_max_y});
    }
    if (!known_static_filter_enabled) {
        ROS_WARN("Known-static object filter explicitly disabled; fail-safe KEEP");
    } else if (!known_static_filter_.ConfigureWithRegions(
                   known_static_catalog, expected_map_id,
                   {{expected_origin_easting, expected_origin_northing,
                     expected_origin_altitude}},
                   known_static_center_tolerance_m,
                   known_static_regions)) {
        ROS_ERROR(
            "Known-static object filter unavailable; fail-safe KEEP: %s",
            known_static_filter_.status_message().c_str());
    } else {
        ROS_INFO(
            "Known-static center-trace filter ready: map=%s references=%zu center_tolerance=%.2fm regions=%zu bounds=[%.2f, %.2f]x[%.2f, %.2f] policy=clean-lap-explicit-unknown-only",
            known_static_filter_.loaded_map_id().c_str(),
            known_static_filter_.reference_count(),
            known_static_filter_.center_tolerance_m(),
            known_static_filter_.application_regions().size(),
            known_static_filter_.application_min_x(),
            known_static_filter_.application_max_x(),
            known_static_filter_.application_min_y(),
            known_static_filter_.application_max_y());
    }

    bool static_structure_exclusion_enabled = true;
    std::string static_structure_exclusion_config =
        ros::package::getPath("Main") +
        "/../../../planning_inputs/config/static_structure_exclusions.yaml";
    double static_structure_minimum_height_m = 2.5;
    double static_structure_minimum_bottom_z_m = 2.0;
    private_node.param(
        "static_structure_exclusion_enabled",
        static_structure_exclusion_enabled,
        static_structure_exclusion_enabled);
    private_node.param(
        "static_structure_exclusion_config",
        static_structure_exclusion_config,
        static_structure_exclusion_config);
    private_node.param(
        "static_structure_minimum_height_m",
        static_structure_minimum_height_m,
        static_structure_minimum_height_m);
    private_node.param(
        "static_structure_minimum_bottom_z_m",
        static_structure_minimum_bottom_z_m,
        static_structure_minimum_bottom_z_m);
    if (!static_structure_exclusion_enabled) {
        ROS_WARN(
            "Configured static-structure exclusion filter explicitly disabled");
    } else if (!static_structure_exclusion_filter_.Configure(
                   static_structure_exclusion_config,
                   static_structure_minimum_height_m,
                   static_structure_minimum_bottom_z_m)) {
        ROS_ERROR(
            "Configured static-structure exclusion unavailable; fail-safe KEEP: %s",
            static_structure_exclusion_filter_.status_message().c_str());
    } else {
        ROS_INFO(
            "Configured static-structure exclusion ready: zones=%zu policy=static-unknown-only",
            static_structure_exclusion_filter_.zone_count());
    }

    objects_publisher_ = node_handle.advertise<Msg::TrackedObjectArray>(
        "/lidar/tracked_objects_map", 2);
    markers_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(
        "/lidar/tracking_markers_map", 2);
    last_objects_publisher_ = node_handle.advertise<Msg::TrackedObjectArray>(
        "/last_lidar", 2);
    last_markers_publisher_ =
        node_handle.advertise<visualization_msgs::MarkerArray>(
            "/last_lidar_markers", 2);
    std::string motion_debug_topic = "/lidar/motion_debug";
    private_node.param(
        "motion_debug_topic", motion_debug_topic, motion_debug_topic);
    motion_debug_publisher_ = node_handle.advertise<std_msgs::String>(
        motion_debug_topic, 2);
    ROS_INFO("LiDAR motion debug topic: %s", motion_debug_topic.c_str());
}

void PlannerObjectRosPublisher::Publish(
    const LiDAR& lidar, const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle, const ros::Time& stamp) {
    if (!stamp.isZero() && stamp == last_stamp_) return;
    last_stamp_ = stamp;
    const auto objects = MakePlannerObjects(
        lidar, gps_imu, vehicle, max_objects_);
    const std_msgs::Header header = MapHeader(stamp);

    Msg::TrackedObjectArray object_array;
    object_array.header = header;
    object_array.objects.reserve(objects.size());
    for (const auto& object : objects) {
        object_array.objects.push_back(ToRosObject(object));
    }
    objects_publisher_.publish(object_array);

    Msg::TrackedObjectArray last_object_array;
    last_object_array.header = header;
    visualization_msgs::MarkerArray last_markers;
    visualization_msgs::Marker last_clear;
    last_clear.header.stamp = header.stamp;
    last_clear.header.frame_id = "velodyne";
    last_clear.action = visualization_msgs::Marker::DELETEALL;
    last_markers.markers.push_back(last_clear);
    const std_msgs::Header lidar_header = [] (const ros::Time& stamp) {
        std_msgs::Header value;
        value.stamp = stamp;
        value.frame_id = "velodyne";
        return value;
    }(header.stamp);
    std::ostringstream motion_debug_json;
    motion_debug_json << "{\"schema\":\"lidar_motion_debug_v2\""
                      << ",\"stamp_secs\":" << header.stamp.sec
                      << ",\"stamp_nsecs\":" << header.stamp.nsec
                      << ",\"stamp\":" << std::fixed
                      << std::setprecision(9) << header.stamp.toSec()
                      << std::defaultfloat << std::setprecision(10)
                      << ",\"frame_id\":\"map\""
                      << ",\"boundary_filter\":{\"available\":"
                      << (drivable_filter_.ready() ? "true" : "false")
                      << ",\"map_id\":\""
                      << JsonEscape(drivable_filter_.loaded_map_id()) << "\""
                      << ",\"margin_m\":" << drivable_filter_.margin_m()
                      << ",\"road_ratio_threshold\":"
                      << drivable_filter_.road_ratio_threshold()
                      << ",\"supersample\":"
                      << drivable_filter_.supersample()
                      << ",\"status\":\""
                      << JsonEscape(drivable_filter_.status_message())
                      << "\"},\"known_static_filter\":{"
                      << "\"available\":"
                      << (known_static_filter_.ready() ? "true" : "false")
                      << ",\"map_id\":\""
                      << JsonEscape(known_static_filter_.loaded_map_id())
                      << "\",\"reference_count\":"
                      << known_static_filter_.reference_count()
                      << ",\"center_tolerance_m\":"
                      << known_static_filter_.center_tolerance_m()
                      << ",\"application_region_enabled\":"
                      << (known_static_filter_.application_region_enabled()
                              ? "true" : "false")
                      << ",\"application_region\":["
                      << known_static_filter_.application_min_x() << ','
                      << known_static_filter_.application_max_x() << ','
                      << known_static_filter_.application_min_y() << ','
                      << known_static_filter_.application_max_y() << ']'
                      << ",\"application_regions\":[";
    for (std::size_t region_index = 0;
         region_index < known_static_filter_.application_regions().size();
         ++region_index) {
        if (region_index > 0) motion_debug_json << ',';
        const auto& region =
            known_static_filter_.application_regions()[region_index];
        motion_debug_json << '[' << region.min_x << ',' << region.max_x
                          << ',' << region.min_y << ',' << region.max_y
                          << ']';
    }
    motion_debug_json << ']'
                      << ",\"status\":\""
                      << JsonEscape(known_static_filter_.status_message())
                      << "\"},\"static_structure_filter\":{"
                      << "\"available\":"
                      << (static_structure_exclusion_filter_.ready()
                              ? "true" : "false")
                      << ",\"zone_count\":"
                      << static_structure_exclusion_filter_.zone_count()
                      << ",\"status\":\""
                      << JsonEscape(
                             static_structure_exclusion_filter_.status_message())
                      << "\"},\"objects\":[";
    bool first_debug_object = true;
    std::size_t boundary_dropped = 0;
    std::size_t known_static_dropped = 0;
    std::size_t configured_static_dropped = 0;
    std::size_t planning_relevance_dropped = 0;
    std::size_t planning_geometry_clipped = 0;
    planning_relevance_filter_.BeginFrame();
    for (const auto& object : objects) {
        const DrivableFilterDecision boundary_filter =
            drivable_filter_.Evaluate(object);
        const KnownStaticDecision static_filter =
            known_static_filter_.Evaluate(object);
        const StaticStructureExclusionDecision configured_static_filter =
            static_structure_exclusion_filter_.Evaluate(object);
        const bool upstream_keep =
            boundary_filter.keep && static_filter.keep &&
            configured_static_filter.keep;
        const PlanningRelevanceDecision relevance =
            planning_relevance_filter_.Evaluate(
                object, upstream_keep,
                gps_imu.st_GPS.f64_E, gps_imu.st_GPS.f64_N,
                gps_imu.st_CAN.f32_CurrentVel_MPS,
                vehicle.f32_Length);
        PlannerObject planning_object = object;
        bool geometry_clipped = false;
        if (relevance.keep) {
            drivable_filter_.BuildPlanningObject(
                object, boundary_filter, &planning_object,
                &geometry_clipped);
        }
        if (!first_debug_object) motion_debug_json << ',';
        first_debug_object = false;
        AppendMotionDebugObject(
            motion_debug_json, object, boundary_filter, static_filter,
            configured_static_filter, relevance, planning_object,
            geometry_clipped);
        if (!boundary_filter.keep) {
            ++boundary_dropped;
            continue;
        }
        if (!static_filter.keep) {
            ++known_static_dropped;
            continue;
        }
        if (!configured_static_filter.keep) {
            ++configured_static_dropped;
            continue;
        }
        if (!relevance.keep) {
            ++planning_relevance_dropped;
            continue;
        }
        if (geometry_clipped) ++planning_geometry_clipped;
        last_object_array.objects.push_back(ToRosObject(planning_object));
        if (object.is_dynamic) {
            const auto motion_state =
                static_cast<MotionState>(object.motion_state);
            // A single frame can contain many dynamic hypotheses.  Printing
            // every object at LiDAR rate floods rosout and can starve the same
            // callback queue that feeds GPS/IMU/CAN pose synchronization.
            ROS_INFO_THROTTLE(
                1.0,
                "Dynamic_object|id:%d|state:%s|raw_velocity:%.2f km/h|raw_x:%.2f km/h|raw_y:%.2f km/h|judge_velocity:%.2f km/h|judge_x:%.2f km/h|judge_y:%.2f km/h|applied_velocity:%.2f km/h|map_displacement:%.2f km/h|corrected_displacement:%.2f km/h|jump:%.2f km/h|accepted:%d|candidates:%d|age:%d frame",
                object.id, MotionStateName(motion_state),
                object.raw_speed_kph,
                object.raw_velocity_lidar_mps.x * 3.6,
                object.raw_velocity_lidar_mps.y * 3.6,
                object.decision_speed_kph,
                object.decision_vx_kph,
                object.decision_vy_kph,
                object.speed_kph,
                object.map_displacement_speed_kph,
                object.corrected_displacement_speed_kph,
                object.motion_velocity_jump_kph,
                object.motion_accepted_samples,
                object.motion_candidate_samples,
                object.age);
        }
        last_markers.markers.push_back(
            MakeLastObbMarker(planning_object, lidar_header));
        if (std::hypot(planning_object.velocity_lidar_mps.x,
                       planning_object.velocity_lidar_mps.y) > 1e-3) {
            last_markers.markers.push_back(
                MakeLastVelocityMarker(planning_object, lidar_header));
        }
        last_markers.markers.push_back(
            MakeLastTextMarker(
                planning_object, lidar_header, boundary_filter.road_ratio));
    }
    if (known_static_dropped > 0) {
        ROS_INFO_THROTTLE(
            2.0,
            "Known-static catalog dropped %zu/%zu explicit UNKNOWN objects after tracking/fusion",
            known_static_dropped, objects.size());
    }
    if (configured_static_dropped > 0) {
        ROS_INFO_THROTTLE(
            2.0,
            "Configured static-structure zones dropped %zu/%zu static UNKNOWN objects after tracking/fusion",
            configured_static_dropped, objects.size());
    }
    if (boundary_dropped > 0) {
        ROS_WARN_THROTTLE(
            2.0,
            "Drivable boundary dropped %zu/%zu static non-pedestrian objects below full-OBB road-overlap threshold",
            boundary_dropped, objects.size());
    }
    if (planning_relevance_dropped > 0) {
        ROS_INFO_THROTTLE(
            2.0,
            "Planning relevance dropped %zu/%zu ground or unconfirmed objects before shared /last_lidar publication",
            planning_relevance_dropped, objects.size());
    }
    motion_debug_json << "],\"filter_counts\":{"
                      << "\"raw\":" << objects.size()
                      << ",\"kept\":" << last_object_array.objects.size()
                      << ",\"drivable_dropped\":" << boundary_dropped
                      << ",\"known_static_dropped\":"
                      << known_static_dropped
                      << ",\"static_zone_dropped\":"
                      << configured_static_dropped
                      << ",\"planning_relevance_dropped\":"
                      << planning_relevance_dropped
                      << ",\"planning_geometry_clipped\":"
                      << planning_geometry_clipped << "}}";
    last_objects_publisher_.publish(last_object_array);
    last_markers_publisher_.publish(last_markers);
    std_msgs::String motion_debug_message;
    motion_debug_message.data = motion_debug_json.str();
    motion_debug_publisher_.publish(motion_debug_message);

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    for (const auto& object : objects) {
        markers.markers.push_back(MakeObbMarker(object, header));
        markers.markers.push_back(MakeCenterMarker(object, header));
        if (object.speed_kph > 0.1) {
            markers.markers.push_back(MakeVelocityMarker(
                object, header, velocity_arrow_seconds_));
        }
        markers.markers.push_back(MakeTextMarker(object, header));
    }
    markers_publisher_.publish(markers);
}
