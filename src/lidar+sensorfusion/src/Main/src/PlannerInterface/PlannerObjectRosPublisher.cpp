#include <PlannerInterface/PlannerObjectRosPublisher.hpp>

#include <PlannerInterface/PlannerObjectAdapter.hpp>

#include <Msg/TrackedObject.h>
#include <Msg/TrackedObjectArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
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

}  // namespace

PlannerObjectRosPublisher::PlannerObjectRosPublisher(
    ros::NodeHandle& node_handle) {
    ros::NodeHandle private_node("~");
    int max_objects = static_cast<int>(max_objects_);
    private_node.param("map_output_max_objects", max_objects, max_objects);
    private_node.param(
        "velocity_arrow_seconds", velocity_arrow_seconds_,
        velocity_arrow_seconds_);
    max_objects_ = static_cast<std::size_t>(std::max(1, max_objects));
    velocity_arrow_seconds_ = std::max(0.05, velocity_arrow_seconds_);

    objects_publisher_ = node_handle.advertise<Msg::TrackedObjectArray>(
        "/lidar/tracked_objects_map", 2);
    markers_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(
        "/lidar/tracking_markers_map", 2);
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
