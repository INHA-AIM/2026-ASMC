/**
 * @file target_projection.cpp
 * @brief v2 STOP target pose 투영/heading validation.
 *
 * R8-B2: v2_planning.cpp의 runV2PlanningCycle() 안에 있던 STOP 목표 투영
 * 블록을 그대로 옮겼다 - 그 블록은 실패 시 자체적으로 SAFE_STOP 궤적/피드백을
 * 발행하고 끝나는(caller가 곧바로 return true 하는) 완전히 닫힌 조각이었기
 * 때문에, 이후 execution 로직과 공유 mutable state 없이 분리할 수 있었다.
 *
 * AVOID/LANE_CHANGE target 투영은 이 파일로 옮기지 않는다 - 두 블록은 그
 * 뒤의 execution/feedback 구성과 깊게 얽혀 있어 같은 방식으로 깔끔하게
 * 떼어지지 않는다. 공통 투영 추상화도 이 단계에서 만들지 않는다 - 경계가
 * 하나(STOP)만 명확한 상태에서 인터페이스를 일반화하면 추측성 설계가 된다.
 */

#include "Planning/PlanningPort.hpp"

#include "Planning/Compatibility/angle_utils.hpp"
#include "Parser/PlanningInput.hpp"

namespace frenet_planner_ros {

bool PlanningPort::prepareStopTarget(
    const asmc_msgs::BehaviorCommand& command, uint32_t feedback_command_id,
    const CommandAcceptance& acceptance, const InputSnapshot& snapshot,
    double ego_s, bool ego_s_valid, PlannerCommand& planner_command) {
    if (!ego_s_valid) {
        // 투영 기준점을 얻지 못하면 정지 목표도 만들 수 없다.
        command_manager_.NotifyPlanFailed(
            /*terminal=*/true,
            asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
            "cannot project ego onto the reference line");
        CommandAcceptance failed = acceptance;
        failed.planning_target = PlanningTarget::NONE;
        failed.execution_state = command_manager_.execution_state();
        failed.result_code = asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
        failed.detail = "cannot project ego onto the reference line";
        has_last_v2_trajectory_ = false;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
        return true;
    }
    double stop_s = 0.0;
    // ego_s를 hint로 넘겨 트랙이 자기 자신과 가까워지는 구간에서 최근접점이
    // 다른 구간으로 튀지 않게 한다. 목표가 경로에서 너무 멀면 false다.
    if (!planner().ProjectPointToS(command.target_pose_map.position.x,
                                  command.target_pose_map.position.y, ego_s,
                                  stop_s)) {
        command_manager_.NotifyPlanFailed(
            /*terminal=*/true,
            asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
            "stop target is not on the reference line");
        CommandAcceptance failed = acceptance;
        failed.planning_target = PlanningTarget::NONE;
        failed.execution_state = command_manager_.execution_state();
        failed.result_code = asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
        failed.detail = "stop target is not on the reference line";
        ROS_WARN_THROTTLE(1.0,
                          "v2 STOP target (%.2f, %.2f) is not on the "
                          "reference line (command_id=%u)",
                          command.target_pose_map.position.x,
                          command.target_pose_map.position.y,
                          command.command_id);
        has_last_v2_trajectory_ = false;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
        return true;
    }

    // 투영이 성공해도 방향이 맞는지는 별개다. 왕복 구간이
    // max_lateral_offset(5m) 이내로 나란히 지나가면 Cartesian 거리는
    // 가깝지만 진행 방향이 반대인 지점에 목표가 붙을 수 있다.
    // 계약 3.4절 4항의 heading 검증이다.
    double reference_heading = 0.0;
    if (!planner().ReferenceHeadingAt(stop_s, reference_heading)) {
        command_manager_.NotifyPlanFailed(
            /*terminal=*/true,
            asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
            "cannot read reference heading at the stop target");
        CommandAcceptance failed = acceptance;
        failed.planning_target = PlanningTarget::NONE;
        failed.execution_state = command_manager_.execution_state();
        failed.result_code = asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
        failed.detail = "cannot read reference heading at the stop target";
        has_last_v2_trajectory_ = false;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
        return true;
    }

    const double target_yaw =
        YawFromQuaternion(command.target_pose_map.orientation);
    // 단순 뺄셈이 아니라 정규화해서 비교한다. 179도와 -179도는 실제로
    // 2도 차이인데 뺄셈만 하면 358도가 된다.
    const double heading_error =
        std::fabs(NormalizeAngle(target_yaw - reference_heading));
    if (heading_error > runtime_.stop_max_target_heading_error_rad) {
        command_manager_.NotifyPlanFailed(
            /*terminal=*/true,
            asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
            "stop target heading disagrees with the reference line");
        CommandAcceptance failed = acceptance;
        failed.planning_target = PlanningTarget::NONE;
        failed.execution_state = command_manager_.execution_state();
        failed.result_code = asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
        failed.detail = "stop target heading disagrees with the reference line";
        ROS_WARN_THROTTLE(
            1.0,
            "v2 STOP target heading rejected: target=%.3f ref=%.3f "
            "error=%.3f rad > %.3f (command_id=%u)",
            target_yaw, reference_heading, heading_error,
            runtime_.stop_max_target_heading_error_rad, command.command_id);
        has_last_v2_trajectory_ = false;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
        return true;
    }

    planner_command.stop_position = stop_s;
    const auto& signal=cycle_control_request_.signal;
    if(control_binding_enabled_ && cycle_control_request_.token.valid() && signal.enabled &&
       cycle_control_request_.command.command_id==command.command_id) {
        auto& target=planner_command.signal_stop;
        target.key={signal.route_generation,signal.stop_line,signal.movement};
        target.target_revision=signal.target_revision;
        target.physical_line={signal.line_x,signal.line_y};target.approach={signal.normal_x,signal.normal_y};
        target.clearance=signal.clearance;target.position_error=signal.position_error;
        if(!planner_.ProjectPointToS(signal.line_x,signal.line_y,ego_s,target.physical_line_s))
            target.physical_line_s=std::numeric_limits<double>::quiet_NaN();
        target.braking_only=signal.braking_only;
        // A malformed partial constraint is rejected by native Plan(), never
        // downgraded to a legacy STOP or a forward-arc target behind the car.
    }
    return false;
}

}  // namespace frenet_planner_ros
