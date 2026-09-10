#include "mpc_controller_node.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "config/parameter_loader.hpp"
#include "message_converter.hpp"
#include "reference/path_resampler.hpp"
#include "reference/reference_validator.hpp"
#include "reference/trajectory_timing.hpp"

namespace mpc_controller_ros {
namespace {

MPCParams LoadParamsForNode(ros::NodeHandle& private_nh) {
    std::string params_path;
    if (!private_nh.getParam("mpc/config_path", params_path) ||
        params_path.empty()) {
        throw std::runtime_error(
            "required private parameter '~mpc/config_path' is missing");
    }
    MPCParams params;
    loadMPCParameters(params_path, params);
    return params;
}

}  // namespace

MpcControllerNode::MpcControllerNode(ros::NodeHandle nh,
                                     ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)),
      controller_(LoadParamsForNode(private_nh_)) {
    std::string trajectory_topic = "/planning/trajectory";
    std::string control_topic = "/ctrl_cmd";
    std::string behavior_command_topic = "/behavior/command_v2";
    std::string vehicle_state_topic = "/vehicle/state";
    private_nh_.param("topics/trajectory", trajectory_topic, trajectory_topic);
    private_nh_.param("publish_topics/control", control_topic, control_topic);
    private_nh_.param("topics/behavior_command_v2", behavior_command_topic,
                      behavior_command_topic);
    private_nh_.param("topics/vehicle_state", vehicle_state_topic,
                      vehicle_state_topic);
    private_nh_.param("timeouts/trajectory", trajectory_timeout_,
                      trajectory_timeout_);
    private_nh_.param("timeouts/vehicle_state", vehicle_state_timeout_,
                      vehicle_state_timeout_);

    double bridge_max_steer_degrees = 40.0;
    private_nh_.param("vehicle/max_steer_deg", bridge_max_steer_degrees,
                      bridge_max_steer_degrees);
    constexpr double kRadiansToDegrees =
        180.0 / 3.14159265358979323846;
    const double mpc_max_steer_degrees =
        controller_.params().steering_max * kRadiansToDegrees;
    if (std::fabs(mpc_max_steer_degrees - bridge_max_steer_degrees) > 0.1) {
        ROS_WARN(
            "MPC max steering %.2f deg differs from MORAI bridge %.2f deg; "
            "align both to the physical vehicle limit before driving",
            mpc_max_steer_degrees, bridge_max_steer_degrees);
    }

    trajectory_sub_ = nh_.subscribe(
        trajectory_topic, 1, &MpcControllerNode::trajectoryCallback, this);
    vehicle_state_sub_ = nh_.subscribe(
        vehicle_state_topic, 1, &MpcControllerNode::vehicleStateCallback, this);
    behavior_command_sub_ = nh_.subscribe(
        behavior_command_topic, 1,
        &MpcControllerNode::behaviorCommandCallback, this);
    control_pub_ = nh_.advertise<morai_msgs::CtrlCmd>(control_topic, 1);
    const double frequency = controller_.params().control_frequency > 0.0
        ? controller_.params().control_frequency : 20.0;
    control_timer_ = nh_.createSteadyTimer(
        ros::WallDuration(1.0 / frequency),
        &MpcControllerNode::controlTimerCallback, this);

    ROS_INFO("MPC ROS runtime @ %.1f Hz: %s -> %s, stale timeout %.3f s",
             frequency, trajectory_topic.c_str(), control_topic.c_str(),
             trajectory_timeout_);
}

void MpcControllerNode::behaviorCommandCallback(
    const asmc_msgs::BehaviorCommand::ConstPtr& message) {
    if (!message) return;
    const bool emergency =
        message->maneuver_type == asmc_msgs::BehaviorCommand::EMERGENCY_STOP;
    if (emergency != emergency_stop_active_) {
        ROS_WARN("MPC emergency brake override %s (command_id=%u)",
                 emergency ? "ACTIVE" : "RELEASED", message->command_id);
    }
    emergency_stop_active_ = emergency;
}

void MpcControllerNode::trajectoryCallback(
    const asmc_msgs::Trajectory::ConstPtr& message) {
    if (!message) return;
    latest_trajectory_ = *message;
    trajectory_received_at_ = ros::SteadyTime::now();
    has_trajectory_ = true;
}

void MpcControllerNode::vehicleStateCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
    if (!message) return;
    latest_vehicle_state_ = *message;
    vehicle_state_received_at_ = ros::SteadyTime::now();
    has_vehicle_state_ = true;
}

void MpcControllerNode::publishStop(const char* reason) {
    controller_.ResetOnFailSafe();
    control_pub_.publish(MakeRosStopCommand());
    ROS_WARN_THROTTLE(1.0, "MPC fail-safe STOP: %s", reason);
}

void MpcControllerNode::controlTimerCallback(const ros::SteadyTimerEvent&) {
    if (emergency_stop_active_) {
        controller_.ResetOnFailSafe();
        control_pub_.publish(MakeRosEmergencyStopCommand());
        return;
    }
    if (!has_trajectory_) {
        publishStop("trajectory missing");
        return;
    }
    if (!IsMessageFresh(trajectory_received_at_, ros::SteadyTime::now(),
                       trajectory_timeout_)) {
        publishStop("trajectory stale");
        return;
    }
    if (!has_vehicle_state_) {
        publishStop("vehicle state missing");
        return;
    }
    if (!IsMessageFresh(vehicle_state_received_at_, ros::SteadyTime::now(),
                       vehicle_state_timeout_)) {
        publishStop("vehicle state stale");
        return;
    }

    // Trajectory -> PlannedPath만 변환한다. ego_at_plan의 provenance는
    // ConvertTrajectory가 내부적으로 검증하지만 MPCState로는 변환하지 않는다
    // - x0는 아래에서 최신 /vehicle/state로부터 독립적으로 만든다.
    PlannedPath path;
    const TrajectoryStatus status = ConvertTrajectory(latest_trajectory_, path);
    if (status == TrajectoryStatus::STOP_TRAJECTORY) {
        publishStop("valid planner STOP trajectory");
        return;
    }
    if (status != TrajectoryStatus::OK) {
        publishStop(TrajectoryStatusName(status));
        return;
    }

    MPCState current_state;
    const VehicleStateStatus vehicle_status =
        ConvertVehicleState(latest_vehicle_state_, current_state);
    if (vehicle_status != VehicleStateStatus::OK) {
        publishStop(VehicleStateStatusName(vehicle_status));
        return;
    }

    const double start_time_offset = ComputeReferenceStartOffset(
        latest_trajectory_.header.stamp.toSec(),
        latest_vehicle_state_.header.stamp.toSec());
    const ReferencePath reference = ToReferencePath(
        path, controller_.params().dt, controller_.params().horizon,
        start_time_offset);
    if (reference.empty()) {
        publishStop("reference interpolation failed");
        return;
    }

    // 물리적으로 말이 되는 참조인지 검증한다(방어적 defense-in-depth) -
    // 예를 들어 Planner/interface 쪽 미래 버그가 다시 불연속 참조를 보내도
    // solver가 그대로 따라가지 않는다. 여기서 거부하면 이 사이클은 ControlProcess를
    // 호출하지 않고 기존 fail-safe STOP 경로를 그대로 재사용한다.
    const ReferenceValidationResult validation =
        ValidateReference(reference, controller_.params());
    if (!validation.ok()) {
        publishStop(ReferenceValidationStatusName(validation.status));
        return;
    }

    const MpcController::Output output =
        controller_.ControlProcess(current_state, reference);
    if (output.stuck_event ==
        MpcController::StuckEvent::RECOVERY_STARTED) {
        ROS_WARN("MPC stuck recovery start: ego_v=%.2f max_ref_v=%.2f "
                 "path_progress=%.2f duration=%.2fs launch_accel=%.2f",
                 output.stuck_event_ego_v, output.stuck_event_max_ref_v,
                 output.stuck_event_path_progress,
                 controller_.params().stuck_recovery_duration,
                 controller_.params().stuck_launch_accel);
    } else if (output.stuck_event ==
               MpcController::StuckEvent::RECOVERY_COMPLETE) {
        ROS_INFO("MPC stuck recovery pulse complete; normal MPC resumed");
    }
    if (!output.success) {
        publishStop(output.failure_reason ? output.failure_reason
                                          : "solver failure");
        return;
    }

    control_pub_.publish(ToRosControlCommand(output.control,
                                             controller_.params()));
}

}  // namespace mpc_controller_ros
