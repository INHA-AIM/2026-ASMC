/**
 * @file debug_marker_publisher.cpp
 * @brief RViz 디버그 마커 발행: publishDebugMarkers(v1)/publishDebugMarkersV2.
 *
 * R8-C2: frenet_planner_node.cpp에서 그대로 옮겼다 - 여전히 FrenetPlannerNode
 * 멤버 함수이고, marker_pub_도 FrenetPlannerNode가 그대로 소유한다. marker
 * 내용/스타일/namespace/id/색상/위치는 이 이동에서 바꾸지 않는다(순수 이동).
 */

#include "frenet_planner_node.hpp"

#include <sstream>
#include <utility>

#include <geometry_msgs/Point.h>

namespace frenet_planner_ros {

void FrenetPlannerNode::publishDebugMarkers(
    const asmc_msgs::Trajectory& trajectory,
    const InputSnapshot& snapshot) {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    {
        int candidate_id = 0;
        const auto& candidate_paths = planner_.last_candidate_paths();
        const auto& candidate_validity = planner_.last_candidate_validity();
        for (std::size_t candidate_index = 0;
             candidate_index < candidate_paths.size(); ++candidate_index) {
            const CartesianPath& candidate = candidate_paths[candidate_index];
            const bool valid = candidate_index < candidate_validity.size()
                && candidate_validity[candidate_index];
            visualization_msgs::Marker marker;
            marker.header = trajectory.header;
            marker.header.frame_id = "map";
            marker.ns = valid ? "frenet_candidates_valid"
                              : "frenet_candidates_rejected";
            marker.id = candidate_id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.08;
            marker.color.r = valid ? 0.35 : 1.0;
            marker.color.g = valid ? 0.65 : 0.20;
            marker.color.b = valid ? 1.0 : 0.25;
            marker.color.a = valid ? 0.50 : 0.40;
            for (std::size_t i = 0; i < candidate.x.size(); ++i) {
                geometry_msgs::Point point;
                point.x = candidate.x[i];
                point.y = candidate.y[i];
                point.z = 0.25;
                marker.points.push_back(point);
            }
            markers.markers.push_back(std::move(marker));
        }
    }

    visualization_msgs::Marker selected;
    selected.header = trajectory.header;
    selected.header.frame_id = "map";
    selected.ns = "frenet_selected_trajectory";
    selected.id = 0;
    selected.type = visualization_msgs::Marker::LINE_STRIP;
    selected.action = visualization_msgs::Marker::ADD;
    selected.pose.orientation.w = 1.0;
    selected.scale.x = 0.35;
    selected.color.r = 1.0;
    selected.color.g = 0.82;
    selected.color.a = trajectory.valid ? 1.0 : 0.45;
    for (const auto& point : trajectory.points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.45;
        selected.points.push_back(p);
    }
    markers.markers.push_back(selected);

    if (snapshot.has_behavior && snapshot.behavior.has_stop_target) {
        visualization_msgs::Marker stop;
        stop.header = selected.header;
        stop.ns = "frenet_stop_target";
        stop.id = 0;
        stop.type = visualization_msgs::Marker::CYLINDER;
        stop.action = visualization_msgs::Marker::ADD;
        stop.pose.position.x = snapshot.behavior.stop_x;
        stop.pose.position.y = snapshot.behavior.stop_y;
        stop.pose.position.z = 0.15;
        stop.pose.orientation.w = 1.0;
        stop.scale.x = 1.3;
        stop.scale.y = 1.3;
        stop.scale.z = 0.3;
        stop.color.r = 1.0;
        stop.color.a = 0.9;
        markers.markers.push_back(stop);
    }

    visualization_msgs::Marker status;
    status.header = selected.header;
    status.ns = "frenet_status";
    status.id = 0;
    status.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::Marker::ADD;
    status.pose.position.x = trajectory.ego_at_plan.x;
    status.pose.position.y = trajectory.ego_at_plan.y;
    status.pose.position.z = 4.7;
    status.pose.orientation.w = 1.0;
    status.scale.z = 0.7;
    status.color.r = 1.0;
    status.color.g = 0.82;
    status.color.a = 1.0;
    std::ostringstream text;
    text << "Frenet selected: " << trajectory.points.size() << " points"
         << "  valid=" << trajectory.valid
         << "  failure=" << static_cast<int>(trajectory.failure_reason);
    if (snapshot.has_behavior) {
        text << "  command=" << snapshot.behavior.command_id
             << "  behavior="
             << static_cast<int>(snapshot.behavior.selected_behavior);
    }
    status.text = text.str();
    markers.markers.push_back(status);
    marker_pub_.publish(markers);
}

void FrenetPlannerNode::publishDebugMarkersV2(
    const asmc_msgs::Trajectory& trajectory,
    const asmc_msgs::BehaviorCommand& command) {
    // Reuse the established candidate/selected-path renderer.  InputSnapshot
    // is only used here for debug labels and the STOP marker, so adapt the v2
    // command without feeding it back into the v1 planning path.
    InputSnapshot snapshot;
    snapshot.has_behavior = true;
    snapshot.behavior.command_id = command.command_id;
    snapshot.behavior.selected_behavior = command.maneuver_type;
    snapshot.behavior.target_link_id = command.target_link_id;
    snapshot.behavior.has_stop_target =
        command.maneuver_type == asmc_msgs::BehaviorCommand::STOP &&
        command.has_target_pose;
    if (snapshot.behavior.has_stop_target) {
        snapshot.behavior.stop_x = command.target_pose_map.position.x;
        snapshot.behavior.stop_y = command.target_pose_map.position.y;
    }
    publishDebugMarkers(trajectory, snapshot);
}

}  // namespace frenet_planner_ros
