#include <PlannerInterface/PlannerObjectRosPublisher.hpp>

#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>
#include <Lidar/TunnelFilter/TunnelFilterRos.hpp>

#include <Msg/TrackedObject.h>
#include <Msg/TrackedObjectArray.h>
#include <geometry_msgs/Point.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

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

std_msgs::Header LidarHeader(const ros::Time& stamp) {
    std_msgs::Header header;
    header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    header.frame_id = "velodyne";
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

Msg::TrackedObject ToRosLidarObject(const PlannerObject& object) {
    Msg::TrackedObject message;
    message.id = object.id;
    message.type = object.type;
    message.classification = object.classification;
    message.yolo_class = object.yolo_class;
    message.is_dynamic = object.is_dynamic;
    message.age = object.age;
    message.miss_count = object.miss_count;
    message.cluster_center = ToRosPoint(object.cluster_center_lidar);
    message.obb_center = ToRosPoint(object.obb_center_lidar);
    for (std::size_t i = 0; i < object.obb_corners_lidar.size(); ++i) {
        message.obb_corners[i] = ToRosPoint(object.obb_corners_lidar[i]);
    }
    message.obb_yaw = object.obb_yaw_lidar_rad;
    message.velocity_mps.x = object.velocity_lidar_mps.x;
    message.velocity_mps.y = object.velocity_lidar_mps.y;
    message.velocity_mps.z = object.velocity_lidar_mps.z;
    message.speed_kph = object.speed_kph;
    message.length = object.length;
    message.width = object.width;
    message.height = object.height;
    return message;
}

bool IsProtectedRoadUser(const PlannerObject& object) {
    const auto protected_class = [](const std::string& value) {
        return value == "car" || value == "truck" || value == "bus" ||
               value == "person" || value == "pedestrian";
    };
    return protected_class(object.classification) ||
           protected_class(object.yolo_class);
}

bool IsTunnelStructure(const PlannerObject& object,
                       double ceiling_center_z_m,
                       double wall_min_length_m,
                       double wall_min_aspect_ratio,
                       double wall_min_abs_y_m) {
    if (IsProtectedRoadUser(object)) return false;

    const double long_side = std::max(object.length, object.width);
    const double short_side = std::max(
        0.05, std::min(object.length, object.width));
    const bool ceiling =
        object.obb_center_lidar.z >= ceiling_center_z_m;
    const bool wall =
        std::abs(object.obb_center_lidar.y) >= wall_min_abs_y_m &&
        long_side >= wall_min_length_m &&
        long_side / short_side >= wall_min_aspect_ratio;
    return ceiling || wall;
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
    char text[384];
    std::snprintf(
        text, sizeof(text),
        "ID:%d %s age:%d n:%d\n"
        "class:%s yolo:%s\n"
        "raw |v|:%.2f x:%.2f y:%.2f km/h\n"
        "judge |v|:%.2f x:%.2f y:%.2f km/h\n"
        "ENU:(%.2f, %.2f)\nroad:%.2f",
        object.id, MotionStateName(motion_state), object.age,
        object.motion_accepted_samples,
        object.classification.c_str(), object.yolo_class.c_str(),
        object.speed_kph,
        std::abs(object.velocity_lidar_mps.x) * 3.6,
        std::abs(object.velocity_lidar_mps.y) * 3.6,
        object.decision_speed_kph,
        std::abs(object.decision_vx_kph),
        std::abs(object.decision_vy_kph),
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
    private_node.param(
        "drivable_ratio_threshold", drivable_ratio_threshold_,
        drivable_ratio_threshold_);
    private_node.param(
        "drivable_samples", drivable_samples_, drivable_samples_);
    private_node.param(
        "tunnel_ceiling_center_z_m", tunnel_ceiling_center_z_m_,
        tunnel_ceiling_center_z_m_);
    private_node.param(
        "tunnel_wall_min_length_m", tunnel_wall_min_length_m_,
        tunnel_wall_min_length_m_);
    private_node.param(
        "tunnel_wall_min_aspect_ratio", tunnel_wall_min_aspect_ratio_,
        tunnel_wall_min_aspect_ratio_);
    private_node.param(
        "tunnel_wall_min_abs_y_m", tunnel_wall_min_abs_y_m_,
        tunnel_wall_min_abs_y_m_);
    max_objects_ = static_cast<std::size_t>(std::max(1, max_objects));
    velocity_arrow_seconds_ = std::max(0.05, velocity_arrow_seconds_);
    drivable_ratio_threshold_ = std::max(
        0.0, std::min(1.0, drivable_ratio_threshold_));
    drivable_samples_ = std::max(2, drivable_samples_);

    const std::string default_output =
        ros::package::getPath("Main") + "/../../../link_node/out/";
    std::string mask_image = default_output + "road_mask.png";
    std::string mask_metadata = default_output + "road_mask.yaml";
    std::string expected_map_id = "waize_katri_2025-10-23";
    double margin_m = 1.0;
    private_node.param("drivable_mask_image", mask_image, mask_image);
    private_node.param("drivable_mask_metadata", mask_metadata, mask_metadata);
    private_node.param("drivable_map_id", expected_map_id, expected_map_id);
    private_node.param("drivable_margin_m", margin_m, margin_m);
    LoadDrivableMask(
        mask_image, mask_metadata, expected_map_id, margin_m);

    objects_publisher_ = node_handle.advertise<Msg::TrackedObjectArray>(
        "/lidar/tracked_objects_map", 2);
    markers_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(
        "/lidar/tracking_markers_map", 2);
    last_objects_publisher_ = node_handle.advertise<Msg::TrackedObjectArray>(
        "/last_lidar", 2);
    last_markers_publisher_ =
        node_handle.advertise<visualization_msgs::MarkerArray>(
            "/last_lidar_markers", 2);
    nogps_objects_publisher_ = node_handle.advertise<Msg::TrackedObjectArray>(
        "/lidar_nogps", 2);
    nogps_markers_publisher_ =
        node_handle.advertise<visualization_msgs::MarkerArray>(
            "/lidar_nogps_markers", 2);
}

void PlannerObjectRosPublisher::LoadDrivableMask(
    const std::string& image_path, const std::string& metadata_path,
    const std::string& expected_map_id, double margin_m) {
    drivable_mask_ = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (drivable_mask_.empty()) {
        throw std::runtime_error("Cannot load drivable mask: " + image_path);
    }
    const YAML::Node metadata = YAML::LoadFile(metadata_path);
    const std::string map_id = metadata["map_id"].as<std::string>();
    if (!expected_map_id.empty() && map_id != expected_map_id) {
        throw std::runtime_error(
            "Drivable map id mismatch: expected " + expected_map_id +
            ", got " + map_id);
    }
    pixels_per_meter_ = metadata["pixels_per_meter"].as<double>();
    const auto offset = metadata["world_offset"];
    world_offset_x_ = offset[0].as<double>();
    world_offset_y_ = offset[1].as<double>();
    if (!(pixels_per_meter_ > 0.0)) {
        throw std::runtime_error("Invalid drivable-mask pixels_per_meter");
    }
    if (margin_m > 0.0) {
        const int kernel_size = std::max(
            3, static_cast<int>(std::lround(
                2.0 * margin_m * pixels_per_meter_)) + 1);
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::dilate(drivable_mask_, drivable_mask_, kernel);
    }
    ROS_INFO(
        "Drivable mask loaded: map=%s size=%dx%d margin=%.2fm threshold=%.2f samples=%d",
        map_id.c_str(), drivable_mask_.cols, drivable_mask_.rows,
        margin_m, drivable_ratio_threshold_, drivable_samples_);
}

double PlannerObjectRosPublisher::DrivableRatio(
    const PlannerObject& object) const {
    int inside = 0;
    const int total = drivable_samples_ * drivable_samples_;
    const auto& q = object.obb_corners_map;
    for (int row = 0; row < drivable_samples_; ++row) {
        const double v = static_cast<double>(row) /
            static_cast<double>(drivable_samples_ - 1);
        for (int col = 0; col < drivable_samples_; ++col) {
            const double u = static_cast<double>(col) /
                static_cast<double>(drivable_samples_ - 1);
            const double x =
                (1.0 - u) * (1.0 - v) * q[0].x +
                u * (1.0 - v) * q[1].x + u * v * q[2].x +
                (1.0 - u) * v * q[3].x;
            const double y =
                (1.0 - u) * (1.0 - v) * q[0].y +
                u * (1.0 - v) * q[1].y + u * v * q[2].y +
                (1.0 - u) * v * q[3].y;
            const int px = static_cast<int>(std::lround(
                pixels_per_meter_ * (x - world_offset_x_)));
            const int py = static_cast<int>(std::lround(
                pixels_per_meter_ * (y - world_offset_y_)));
            if (px >= 0 && px < drivable_mask_.cols &&
                py >= 0 && py < drivable_mask_.rows &&
                drivable_mask_.at<unsigned char>(py, px) != 0) {
                ++inside;
            }
        }
    }
    return static_cast<double>(inside) / static_cast<double>(total);
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
    Msg::TrackedObjectArray nogps_object_array;
    nogps_object_array.header = LidarHeader(header.stamp);
    visualization_msgs::MarkerArray last_markers;
    visualization_msgs::Marker last_clear;
    last_clear.header.stamp = header.stamp;
    last_clear.header.frame_id = "velodyne";
    last_clear.action = visualization_msgs::Marker::DELETEALL;
    last_markers.markers.push_back(last_clear);
    visualization_msgs::MarkerArray nogps_markers;
    visualization_msgs::Marker nogps_clear;
    nogps_clear.header = nogps_object_array.header;
    nogps_clear.action = visualization_msgs::Marker::DELETEALL;
    nogps_markers.markers.push_back(nogps_clear);
    const std_msgs::Header lidar_header = [] (const ros::Time& stamp) {
        std_msgs::Header value;
        value.stamp = stamp;
        value.frame_id = "velodyne";
        return value;
    }(header.stamp);
    // Enter immediately on an invalid GPS sample so zero coordinates never
    // reach /last_lidar. Keep tunnel mode until three consecutive valid GPS
    // messages have arrived (b_isTunnel exit hysteresis).
    const bool tunnel_mode = tunnel_filter::IsGpsDenied(
        gps_imu.st_GPS.b_isJamming, gps_imu.st_GPS.b_isTunnel);
    for (const auto& object : objects) {
        if (tunnel_mode) {
            // 통합 터널 필터 결과는 이미 점 단위 벽/천장/지면 제거와 OBB 안전
            // 필터를 통과했다. 레거시 OBB 규칙을 다시 적용하면 긴 차량을 벽으로
            // 오인할 수 있으므로, 통합 결과가 없는 fallback에서만 사용한다.
            if (!tunnel_filter::TunnelExperimentOutputReady(header.stamp) &&
                IsTunnelStructure(
                    object, tunnel_ceiling_center_z_m_,
                    tunnel_wall_min_length_m_,
                    tunnel_wall_min_aspect_ratio_,
                    tunnel_wall_min_abs_y_m_)) {
                continue;
            }
            nogps_object_array.objects.push_back(ToRosLidarObject(object));
            nogps_markers.markers.push_back(
                MakeLastObbMarker(object, nogps_object_array.header));
            nogps_markers.markers.push_back(
                MakeLastVelocityMarker(object, nogps_object_array.header));
            continue;
        }
        const double road_ratio = DrivableRatio(object);
        if (road_ratio < drivable_ratio_threshold_) continue;
        last_object_array.objects.push_back(ToRosObject(object));
        if (object.is_dynamic) {
            const auto motion_state =
                static_cast<MotionState>(object.motion_state);
            ROS_INFO(
                "Dynamic_object|id:%d|state:%s|raw_velocity:%.2f km/h|raw_x:%.2f km/h|raw_y:%.2f km/h|judge_velocity:%.2f km/h|judge_x:%.2f km/h|judge_y:%.2f km/h|map_displacement:%.2f km/h|jump:%.2f km/h|accepted:%d|candidates:%d|age:%d frame",
                object.id, MotionStateName(motion_state),
                object.speed_kph,
                std::abs(object.velocity_lidar_mps.x) * 3.6,
                std::abs(object.velocity_lidar_mps.y) * 3.6,
                object.decision_speed_kph,
                std::abs(object.decision_vx_kph),
                std::abs(object.decision_vy_kph),
                object.map_displacement_speed_kph,
                object.motion_velocity_jump_kph,
                object.motion_accepted_samples,
                object.motion_candidate_samples,
                object.age);
        }
        last_markers.markers.push_back(
            MakeLastObbMarker(object, lidar_header));
        if (std::hypot(object.velocity_lidar_mps.x,
                       object.velocity_lidar_mps.y) > 1e-3) {
            last_markers.markers.push_back(
                MakeLastVelocityMarker(object, lidar_header));
        }
        last_markers.markers.push_back(
            MakeLastTextMarker(object, lidar_header, road_ratio));
    }
    last_objects_publisher_.publish(last_object_array);
    last_markers_publisher_.publish(last_markers);
    nogps_objects_publisher_.publish(nogps_object_array);
    nogps_markers_publisher_.publish(nogps_markers);

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
