#pragma once
#include "Planning/Planner.hpp"
#include "Frenet/FrenetResult.hpp"
#include <cmath>
#include <vector>
#include "Parser/PlanningInput.hpp"
#include "Planning/Compatibility/steady_freshness.hpp"
namespace asmc_planner {
template<class RuntimePort>
void Planner::RunV1(RuntimePort& port) {
    using namespace frenet_planner_ros;
    using InputSnapshot = typename RuntimePort::InputSnapshot;

    if (!port.planner_initialized_) {
        ROS_ERROR_THROTTLE(
            5.0, "Frenet planning disabled: planner is not initialized");
        port.publishStop(port.takeSnapshot(), asmc_msgs::Trajectory::FAILURE_NOT_INITIALIZED);
        return;
    }

    const InputSnapshot snapshot = port.takeSnapshot();
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
        port.publishStop(snapshot, reason);
        return;
    }

    const ros::SteadyTime steady_now = ros::SteadyTime::now();
    double vehicle_age = 0.0;
    double objects_age = 0.0;
    double behavior_age = 0.0;
    if (!IsSteadyFresh(snapshot.vehicle_received_at, steady_now,
                       port.runtime_.vehicle_state_timeout_s, &vehicle_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale vehicle state age=%.3f s",
            vehicle_age);
        port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_STATE_STALE);
        return;
    }
    if (!IsSteadyFresh(snapshot.objects_received_at, steady_now,
                       port.runtime_.objects_timeout_s, &objects_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale objects age=%.3f s",
            objects_age);
        port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_OBJECTS_STALE);
        return;
    }
    if (!IsSteadyFresh(snapshot.behavior_received_at, steady_now,
                       port.runtime_.behavior_timeout_s, &behavior_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale behavior age=%.3f s",
            behavior_age);
        port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_BEHAVIOR_STALE);
        return;
    }

    if (snapshot.behavior.force_stop) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: behavior force_stop command_id=%u",
            snapshot.behavior.command_id);
        port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_NONE);
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
        port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        return;
    }
    port.last_stop_reference_vehicle_ = snapshot.vehicle;
    port.has_stop_reference_vehicle_ = true;

    const ConversionStatus behavior_status =
        ConvertBehaviorContext(snapshot.behavior, command);
    if (behavior_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: behavior conversion failed (%s)",
            ConversionStatusName(behavior_status));
        port.publishStop(
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
        if (!port.planner_.ProjectEgoToS(ego, ego_s)) {
            ROS_WARN_THROTTLE(
                1.0, "Frenet STOP required: cannot project ego onto reference line");
            port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
            return;
        }
        double stop_s = 0.0;
        // ego_s를 hint로 넘겨 트랙이 자기 자신과 가까워지는 구간에서 최근접점이
        // 다른 구간으로 튀지 않게 한다. 목표가 경로에서 너무 멀면 false.
        if (!port.planner_.ProjectPointToS(snapshot.behavior.stop_x,
                                      snapshot.behavior.stop_y,
                                      ego_s, stop_s)) {
            ROS_WARN_THROTTLE(
                1.0,
                "Frenet STOP required: stop target (%.2f, %.2f) is not on the "
                "reference line (command_id=%u)",
                snapshot.behavior.stop_x, snapshot.behavior.stop_y,
                snapshot.behavior.command_id);
            port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
            return;
        }
        command.stop_position = stop_s;
    }

    port.planner_.SetObjectObservationTime(snapshot.objects.header.stamp.toSec());
    const ConversionStatus objects_status =
        ConvertDetectedObjects(snapshot.objects, objects);
    if (objects_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: objects conversion failed (%s)",
            ConversionStatusName(objects_status));
        port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        return;
    }

    CartesianPath path;
    if (!CalculateFrenet(path, [&] { return port.planner_.Plan(ego, objects, command, path); }).success) {
        ++port.plan_fail_streak_;
        // 이전 궤적 재사용은 "이번 사이클만 후보를 못 만든" 일시적 실패에만
        // 허용된다. 명령 자체가 잘못됐거나(INVALID_COMMAND) 지원하지 않는
        // 모드(UNSUPPORTED_BEHAVIOR)인데 이전 궤적을 계속 내보내면, 예를 들어
        // STOP 명령을 받고도 직전 KEEP 궤적을 따라 계속 주행하게 된다.
        // 이런 실패는 재사용 없이 즉시 정지한다.
        const FrenetPlanner::PlanFailureReason reason =
            port.planner_.last_failure_reason();
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
            port.publishStop(snapshot, failure_code);
            return;
        }

        if (port.has_last_valid_trajectory_ &&
            port.plan_fail_streak_ <= port.kMaxTransientFailCycles) {
            asmc_msgs::Trajectory reused = port.last_valid_trajectory_;
            reused.failure_reason = asmc_msgs::Trajectory::FAILURE_GENERATION;
            port.publishResult(reused, snapshot, true);
            ROS_WARN("Frenet transient failure %d/%d: reusing previous trajectory",
                     port.plan_fail_streak_, port.kMaxTransientFailCycles);
            return;
        }
        ROS_WARN_THROTTLE(
            1.0,
            "Frenet Plan(command) failed: reason=%s command_id=%u; "
            "publishing STOP after transient reuse window",
            reason_name, snapshot.behavior.command_id);
        port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_GENERATION);
        return;
    }

    asmc_msgs::Trajectory trajectory;
    const ConversionStatus trajectory_status = ConvertCartesianPath(
        path, port.planner_.sample_dt(), snapshot.vehicle, trajectory);
    if (trajectory_status != ConversionStatus::OK) {
        ++port.plan_fail_streak_;
        if (port.has_last_valid_trajectory_ &&
            port.plan_fail_streak_ <= port.kMaxTransientFailCycles) {
            asmc_msgs::Trajectory reused = port.last_valid_trajectory_;
            reused.failure_reason = asmc_msgs::Trajectory::FAILURE_GENERATION;
            port.publishResult(reused, snapshot, true);
            return;
        }
        ROS_WARN_THROTTLE(
            1.0, "Frenet trajectory conversion failed (%s)",
            ConversionStatusName(trajectory_status));
        port.publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_GENERATION);
        return;
    }

    port.plan_fail_streak_ = 0;
    port.last_valid_trajectory_ = trajectory;
    port.has_last_valid_trajectory_ = true;
    port.publishResult(trajectory, snapshot, false);

    ROS_INFO_THROTTLE(
        1.0,
        "Frenet KEEP plan ready: command_id=%u points=%zu dt=%.3f "
        "vehicle_age=%.3f objects_age=%.3f behavior_age=%.3f; "
        "trajectory published",
        snapshot.behavior.command_id, path.x.size(), port.planner_.sample_dt(),
        vehicle_age, objects_age, behavior_age);
}
}

