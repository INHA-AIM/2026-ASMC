#include "frenet_planner_node.hpp"

#include <cmath>
#include <sstream>
#include <vector>
#include <utility>

#include <geometry_msgs/Point.h>
#include "message_converter.hpp"
#include "steady_freshness.hpp"

namespace frenet_planner_ros {

FrenetPlannerNode::FrenetPlannerNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
    std::string planner_config_path;
    pnh_.param<std::string>("planner/config_path", planner_config_path, "");

    if (planner_config_path.empty()) {
        ROS_ERROR("Missing required private parameter: planner/config_path");
    } else {
        std::string runtime_error;
        if (!LoadRuntimeConfig(planner_config_path, runtime_, runtime_error)) {
            ROS_ERROR_STREAM("Frenet runtime config load failed ('"
                             << planner_config_path << "'): " << runtime_error);
        } else {
            runtime_loaded_ = true;
        }

        planner_initialized_ = planner_.Init(planner_config_path);
        if (!planner_initialized_) {
            ROS_ERROR_STREAM("Frenet planner initialization failed: "
                             << planner_config_path);
        }
    }

    trajectory_pub_ =
        nh_.advertise<asmc_msgs::Trajectory>(runtime_.trajectory_topic, 1);
    feedback_pub_ =
        nh_.advertise<asmc_msgs::PlanFeedback>(runtime_.feedback_topic, 1);
    feedback_v2_pub_ = nh_.advertise<asmc_msgs::FrenetFeedback>(
        runtime_.feedback_v2_topic, 1);

    // v2 구현 상태. 열린 maneuver만 planner를 호출한다(계약 11.4절).
    // 닫혀 있는 maneuver는 FAILED로 회신한다 - 조용히 velocity keeping
    // 궤적을 만드는 것보다 안전하다.
    //
    // LANE_CHANGE는 fixed-reference 검증을 위해 이 gate를 연다 - Behavior
    // Planner의 scorer/policy는 그대로 닫혀 있으므로 실제 FSM이 자동으로
    // LANE_CHANGE를 고르지는 않는다. 수동 검증은
    // tools/lane_change_test_publisher.cpp로 명령을 직접 발행해 수행한다.
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::KEEP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::STOP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::EMERGENCY_STOP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::FOLLOW, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::AVOID, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::LANE_CHANGE, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        runtime_.markers_topic, 1);

    vehicle_sub_ = nh_.subscribe(
        runtime_.vehicle_state_topic, 1, &FrenetPlannerNode::vehicleCallback, this);
    objects_sub_ = nh_.subscribe(
        runtime_.objects_topic, 1, &FrenetPlannerNode::objectsCallback, this);
    behavior_sub_ = nh_.subscribe(
        runtime_.behavior_topic, 1, &FrenetPlannerNode::behaviorCallback, this);
    behavior_command_v2_sub_ = nh_.subscribe(
        runtime_.behavior_command_v2_topic, 1,
        &FrenetPlannerNode::behaviorCommandV2Callback, this);

    // runtime_loaded_==false면 planning_frequency_hz가 정의되지 않은 값일 수
    // 있다 - 이 경우 timer를 만들지 않는다(1Hz fallback으로 조용히 계속
    // 도는 대신 명백히 동작하지 않는 상태로 둔다).
    if (runtime_loaded_) {
        planning_timer_ = nh_.createSteadyTimer(
            ros::WallDuration(1.0 / runtime_.planning_frequency_hz),
            &FrenetPlannerNode::planningTimerCallback, this);
    }

    ROS_INFO_STREAM("Frenet ROS runtime initialized at "
                    << runtime_.planning_frequency_hz
                    << " Hz; inputs: " << runtime_.vehicle_state_topic << ", "
                    << runtime_.objects_topic << ", " << runtime_.behavior_topic
                    << "; outputs: " << runtime_.trajectory_topic << ", "
                    << runtime_.feedback_topic << "; v2: "
                    << runtime_.behavior_command_v2_topic
                    << " -> " << runtime_.feedback_v2_topic
                    << " (KEEP/STOP/E-STOP/FOLLOW/AVOID/LANE_CHANGE enabled)"
                    << "; planner_initialized="
                    << (planner_initialized_ ? "true" : "false")
                    << "; runtime_loaded="
                    << (runtime_loaded_ ? "true" : "false"));
}

double FrenetPlannerNode::trajectoryDt() const {
    const double dt = planner_.sample_dt();
    return planner_initialized_ && std::isfinite(dt) && dt > 0.0 ? dt : 0.1;
}

asmc_msgs::VehicleState FrenetPlannerNode::stopReferenceVehicle(
    const InputSnapshot& snapshot) const {
    if (snapshot.has_vehicle && std::isfinite(snapshot.vehicle.x) &&
        std::isfinite(snapshot.vehicle.y) && std::isfinite(snapshot.vehicle.yaw)) {
        return snapshot.vehicle;
    }
    if (has_stop_reference_vehicle_) return last_stop_reference_vehicle_;

    asmc_msgs::VehicleState unknown;
    unknown.header.frame_id = "map";
    unknown.valid = false;
    return unknown;
}

void FrenetPlannerNode::publishResult(
    const asmc_msgs::Trajectory& trajectory, const InputSnapshot& snapshot,
    bool previous_plan_still_valid) {
    trajectory_pub_.publish(trajectory);
    publishDebugMarkers(trajectory, snapshot);

    asmc_msgs::PlanFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.header.frame_id = "map";
    feedback.command_id = snapshot.has_behavior ? snapshot.behavior.command_id : 0;
    feedback.plan_valid = trajectory.valid;
    feedback.failure_reason = trajectory.failure_reason;
    feedback.selected_behavior = snapshot.has_behavior
        ? snapshot.behavior.selected_behavior
        : asmc_msgs::BehaviorContext::KEEP;
    feedback.selected_target_link_id = snapshot.has_behavior
        ? snapshot.behavior.target_link_id : "";
    feedback.previous_plan_still_valid = previous_plan_still_valid;
    feedback_pub_.publish(feedback);
}

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

void FrenetPlannerNode::publishStop(const InputSnapshot& snapshot,
                                    uint8_t failure_reason) {
    has_last_valid_trajectory_ = false;
    plan_fail_streak_ = 0;
    const asmc_msgs::Trajectory stop = MakeStopTrajectory(
        stopReferenceVehicle(snapshot), trajectoryDt(), failure_reason);
    publishResult(stop, snapshot, false);
}

void FrenetPlannerNode::planningTimerCallback(const ros::SteadyTimerEvent&) {
    // v2 명령이 하나라도 도착했으면 v2가 이 사이클을 소유한다. v1 경로는
    // v2 명령이 없을 때만 동작하므로, FSM이 v2로 전환하기 전까지 기존
    // KEEP/STOP 주행이 그대로 유지된다(계약 10.6절).
    //
    // 두 계통이 같은 사이클에 궤적을 발행하면 MPC가 어느 것을 따를지 알 수
    // 없으므로 반드시 하나만 처리한다.
    if (runV2PlanningCycle(ros::Time::now())) return;

    runV1PlanningCycle();
}

// 기존 v1 planning path(레거시 /behavior/context -> /planning/feedback).
// v2 명령이 없을 때만 호출된다(planningTimerCallback 주석 참고). 이 함수는
// 예전 timer callback 본문을 그대로 옮긴 것으로, ros::SteadyTime::now()를
// 자체적으로 다시 얻는다 - v2와 달리 이 경로는 처음부터 외부에서 주입받는
// timestamp가 아니라 이 시점의 실제 시각을 썼다(이 semantics를 바꾸지
// 않는다).
void FrenetPlannerNode::runV1PlanningCycle() {
    if (!planner_initialized_) {
        ROS_ERROR_THROTTLE(
            5.0, "Frenet planning disabled: planner is not initialized");
        publishStop(takeSnapshot(), asmc_msgs::Trajectory::FAILURE_NOT_INITIALIZED);
        return;
    }

    const InputSnapshot snapshot = takeSnapshot();
    if (!snapshot.has_vehicle || !snapshot.has_objects || !snapshot.has_behavior) {
        ROS_WARN_THROTTLE(
            5.0,
            "Frenet input snapshot incomplete: vehicle=%d objects=%d behavior=%d",
            snapshot.has_vehicle, snapshot.has_objects, snapshot.has_behavior);
        const uint8_t reason = !snapshot.has_vehicle
            ? asmc_msgs::Trajectory::FAILURE_STATE_STALE
            : (!snapshot.has_objects
                ? asmc_msgs::Trajectory::FAILURE_OBJECTS_STALE
                : asmc_msgs::Trajectory::FAILURE_BEHAVIOR_STALE);
        publishStop(snapshot, reason);
        return;
    }

    const ros::SteadyTime steady_now = ros::SteadyTime::now();
    double vehicle_age = 0.0;
    double objects_age = 0.0;
    double behavior_age = 0.0;
    if (!IsSteadyFresh(snapshot.vehicle_received_at, steady_now,
                       runtime_.vehicle_state_timeout_s, &vehicle_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale vehicle state age=%.3f s",
            vehicle_age);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_STATE_STALE);
        return;
    }
    if (!IsSteadyFresh(snapshot.objects_received_at, steady_now,
                       runtime_.objects_timeout_s, &objects_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale objects age=%.3f s",
            objects_age);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_OBJECTS_STALE);
        return;
    }
    if (!IsSteadyFresh(snapshot.behavior_received_at, steady_now,
                       runtime_.behavior_timeout_s, &behavior_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale behavior age=%.3f s",
            behavior_age);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_BEHAVIOR_STALE);
        return;
    }

    if (snapshot.behavior.force_stop) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: behavior force_stop command_id=%u",
            snapshot.behavior.command_id);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_NONE);
        return;
    }

    CartesianState ego{};
    PlannerCommand command{};
    std::vector<TrackedObject> objects;

    const ConversionStatus vehicle_status =
        ConvertVehicleState(snapshot.vehicle, ego);
    if (vehicle_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: vehicle conversion failed (%s)",
            ConversionStatusName(vehicle_status));
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        return;
    }
    last_stop_reference_vehicle_ = snapshot.vehicle;
    has_stop_reference_vehicle_ = true;

    const ConversionStatus behavior_status =
        ConvertBehaviorContext(snapshot.behavior, command);
    if (behavior_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: behavior conversion failed (%s)",
            ConversionStatusName(behavior_status));
        publishStop(
            snapshot,
            behavior_status == ConversionStatus::UNSUPPORTED_BEHAVIOR
                ? asmc_msgs::Trajectory::FAILURE_UNSUPPORTED_BEHAVIOR
                : asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        return;
    }

    // STOP 목표는 BehaviorContext가 map 좌표(stop_x/y)로 준다. message_converter는
    // RefLine을 모르므로 유효성만 검사하고 stop_position을 NaN으로 남긴다.
    // RefLine을 가진 planner가 여기서 Frenet s로 투영해 명령을 완성한다.
    if (command.mode == STOP) {
        double ego_s = 0.0;
        if (!planner_.ProjectEgoToS(ego, ego_s)) {
            ROS_WARN_THROTTLE(
                1.0, "Frenet STOP required: cannot project ego onto reference line");
            publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
            return;
        }
        double stop_s = 0.0;
        // ego_s를 hint로 넘겨 트랙이 자기 자신과 가까워지는 구간에서 최근접점이
        // 다른 구간으로 튀지 않게 한다. 목표가 경로에서 너무 멀면 false.
        if (!planner_.ProjectPointToS(snapshot.behavior.stop_x,
                                      snapshot.behavior.stop_y,
                                      ego_s, stop_s)) {
            ROS_WARN_THROTTLE(
                1.0,
                "Frenet STOP required: stop target (%.2f, %.2f) is not on the "
                "reference line (command_id=%u)",
                snapshot.behavior.stop_x, snapshot.behavior.stop_y,
                snapshot.behavior.command_id);
            publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
            return;
        }
        command.stop_position = stop_s;
    }

    const ConversionStatus objects_status =
        ConvertDetectedObjects(snapshot.objects, objects);
    if (objects_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: objects conversion failed (%s)",
            ConversionStatusName(objects_status));
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        return;
    }

    CartesianPath path;
    if (!planner_.Plan(ego, objects, command, path)) {
        ++plan_fail_streak_;
        // 이전 궤적 재사용은 "이번 사이클만 후보를 못 만든" 일시적 실패에만
        // 허용된다. 명령 자체가 잘못됐거나(INVALID_COMMAND) 지원하지 않는
        // 모드(UNSUPPORTED_BEHAVIOR)인데 이전 궤적을 계속 내보내면, 예를 들어
        // STOP 명령을 받고도 직전 KEEP 궤적을 따라 계속 주행하게 된다.
        // 이런 실패는 재사용 없이 즉시 정지한다.
        const FrenetPlanner::PlanFailureReason reason =
            planner_.last_failure_reason();
        bool immediate_stop = true;
        uint8_t failure_code = asmc_msgs::Trajectory::FAILURE_GENERATION;
        const char* reason_name = "UNKNOWN";
        switch (reason) {
            case FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED:
                failure_code = asmc_msgs::Trajectory::FAILURE_COLLISION_BLOCKED;
                reason_name = "COLLISION_BLOCKED";
                break;
            case FrenetPlanner::PlanFailureReason::INVALID_COMMAND:
                failure_code = asmc_msgs::Trajectory::FAILURE_INVALID_INPUT;
                reason_name = "INVALID_COMMAND";
                break;
            case FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR:
                failure_code =
                    asmc_msgs::Trajectory::FAILURE_UNSUPPORTED_BEHAVIOR;
                reason_name = "UNSUPPORTED_BEHAVIOR";
                break;
            case FrenetPlanner::PlanFailureReason::NOT_INITIALIZED:
                failure_code = asmc_msgs::Trajectory::FAILURE_NOT_INITIALIZED;
                reason_name = "NOT_INITIALIZED";
                break;
            case FrenetPlanner::PlanFailureReason::TRANSIENT_GENERATION_FAILURE:
                immediate_stop = false;
                reason_name = "TRANSIENT_GENERATION_FAILURE";
                break;
            case FrenetPlanner::PlanFailureReason::NONE:
                // Plan()이 false를 반환했는데 NONE이면 planner 내부 계약 위반이다.
                // 재사용 대상으로 삼지 않고 안전하게 정지한다.
                reason_name = "NONE";
                break;
        }

        if (immediate_stop) {
            ROS_WARN_THROTTLE(
                1.0,
                "Frenet immediate STOP: reason=%s command_id=%u",
                reason_name, snapshot.behavior.command_id);
            publishStop(snapshot, failure_code);
            return;
        }

        if (has_last_valid_trajectory_ &&
            plan_fail_streak_ <= kMaxTransientFailCycles) {
            asmc_msgs::Trajectory reused = last_valid_trajectory_;
            reused.failure_reason = asmc_msgs::Trajectory::FAILURE_GENERATION;
            publishResult(reused, snapshot, true);
            ROS_WARN("Frenet transient failure %d/%d: reusing previous trajectory",
                     plan_fail_streak_, kMaxTransientFailCycles);
            return;
        }
        ROS_WARN_THROTTLE(
            1.0,
            "Frenet Plan(command) failed: reason=%s command_id=%u; "
            "publishing STOP after transient reuse window",
            reason_name, snapshot.behavior.command_id);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_GENERATION);
        return;
    }

    asmc_msgs::Trajectory trajectory;
    const ConversionStatus trajectory_status = ConvertCartesianPath(
        path, planner_.sample_dt(), snapshot.vehicle, trajectory);
    if (trajectory_status != ConversionStatus::OK) {
        ++plan_fail_streak_;
        if (has_last_valid_trajectory_ &&
            plan_fail_streak_ <= kMaxTransientFailCycles) {
            asmc_msgs::Trajectory reused = last_valid_trajectory_;
            reused.failure_reason = asmc_msgs::Trajectory::FAILURE_GENERATION;
            publishResult(reused, snapshot, true);
            return;
        }
        ROS_WARN_THROTTLE(
            1.0, "Frenet trajectory conversion failed (%s)",
            ConversionStatusName(trajectory_status));
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_GENERATION);
        return;
    }

    plan_fail_streak_ = 0;
    last_valid_trajectory_ = trajectory;
    has_last_valid_trajectory_ = true;
    publishResult(trajectory, snapshot, false);

    ROS_INFO_THROTTLE(
        1.0,
        "Frenet KEEP plan ready: command_id=%u points=%zu dt=%.3f "
        "vehicle_age=%.3f objects_age=%.3f behavior_age=%.3f; "
        "trajectory published",
        snapshot.behavior.command_id, path.x.size(), planner_.sample_dt(),
        vehicle_age, objects_age, behavior_age);
}

}  // namespace frenet_planner_ros
