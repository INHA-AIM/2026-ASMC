#include "mpc_controller_node.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "control/braking_steering.hpp"
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

double SampleDiagnosticCartesianAcceleration(
    const PlannedPath& path, double start_time_offset) {
    if (path.empty() || path.dt <= 0.0 ||
        path.a.size() != path.size() ||
        !std::isfinite(start_time_offset)) {
        return 0.0;
    }
    const double offset = std::max(0.0, start_time_offset);
    const double unclipped_index = offset / path.dt;
    const double index = std::max(
        0.0, std::min(unclipped_index,
                      static_cast<double>(path.size() - 1)));
    const std::size_t first = static_cast<std::size_t>(index);
    const std::size_t second = std::min(first + 1, path.size() - 1);
    const double fraction = index - static_cast<double>(first);
    return path.a[first] + fraction * (path.a[second] - path.a[first]);
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
    has_successful_control_ = false;
    controller_.ResetOnFailSafe();
    control_pub_.publish(MakeRosStopCommand());
    ROS_WARN_THROTTLE(1.0, "MPC fail-safe STOP: %s", reason);
}

void MpcControllerNode::controlTimerCallback(const ros::SteadyTimerEvent&) {
    if (emergency_stop_active_) {
        has_successful_control_ = false;
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
        MPCState stop_state;
        if (ConvertVehicleState(latest_vehicle_state_, stop_state) != VehicleStateStatus::OK) {
            publishStop("invalid vehicle state during planner STOP");
            return;
        }
        auto stop = MakeRosStopCommand();
        if (has_successful_control_) {
            const double age=(ros::SteadyTime::now()-last_successful_control_at_).toSec();
            stop.steering=BrakingSteering(last_successful_steering_,age,
                stop_state.vx,controller_.params().steering_rate_max);
        }
        controller_.ResetOnFailSafe();
        control_pub_.publish(stop);
        ROS_WARN_THROTTLE(1.0,"MPC planner STOP braking: steering=%+.3f",stop.steering);
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
        if (validation.status ==
                ReferenceValidationStatus::CURVATURE_OUT_OF_RANGE &&
            validation.failed_index < reference.k_ref.size()) {
            const MPCParams& params = controller_.params();
            const double curvature_limit = params.steering_curvature_gain *
                std::tan(params.steering_max) /
                std::max(params.wheelbase, 1e-6);
            char reason[192];
            std::snprintf(
                reason, sizeof(reason),
                "CURVATURE_OUT_OF_RANGE index=%zu reference=%.5f limit=%.5f",
                validation.failed_index,
                reference.k_ref[validation.failed_index], curvature_limit);
            publishStop(reason);
            return;
        }
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

    constexpr double kRadiansToDegrees =
        180.0 / 3.14159265358979323846;
    const double raw_cartesian_a0 = SampleDiagnosticCartesianAcceleration(
        path, start_time_offset);
    const double v_ref1 = reference.v_ref.size() >= 2
        ? reference.v_ref[1] : reference.v_ref.front();
    ROS_INFO_THROTTLE(
        0.25,
        "[MPC-LONG] source_path_dt=%.3f mpc_destination_dt=%.3f "
        "current_v=%.2f v_ref0=%.2f v_ref1=%.2f "
        "raw_cartesian_a0=%+.2f derived_a_ref0=%+.2f "
        "clipped_a_ref0=%+.2f a_cmd=%+.2f",
        path.dt, controller_.params().dt, output.current_speed,
        output.first_reference_speed, v_ref1, raw_cartesian_a0,
        output.first_reference_acceleration,
        output.first_used_reference_acceleration, output.control.accel);
    ROS_INFO_THROTTLE(
        0.25,
        "[MPC-TRACK] contour_cost=%.3f lag_cost=%.3f heading_cost=%.3f "
        "speed_cost=%.3f control_cost=%.3f rate_cost=%.3f terminal=%.3f "
        "steer_cmd_deg=%.2f ref_kappa0=%.5f steer_ff_deg=%.2f "
        "v=%.2f v_ref0=%.2f v_error=%.2f a_ref0=%.2f a_cmd=%.2f "
        "a_residual=%.2f accel_control_cost=%.3f accel_rate_cost=%.3f",
        output.cost.contour, output.cost.lag, output.cost.heading,
        output.cost.speed, output.cost.control, output.cost.control_rate,
        output.cost.terminal, output.control.delta * kRadiansToDegrees,
        output.first_reference_curvature,
        output.first_feedforward_steering * kRadiansToDegrees,
        output.current_speed, output.first_reference_speed,
        output.first_speed_error, output.first_used_reference_acceleration,
        output.control.accel, output.first_acceleration_residual,
        output.cost.acceleration_control, output.cost.acceleration_rate);

    last_successful_steering_=output.control.delta;
    last_successful_control_at_=ros::SteadyTime::now();
    has_successful_control_=true;
    control_pub_.publish(ToRosControlCommand(output.control,
                                             controller_.params()));
}

}  // namespace mpc_controller_ros

