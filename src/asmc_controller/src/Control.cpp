#include "Control.hpp"
#include "Global/ControlContext.hpp"
#include "MPC/MPC.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>

#include "Util/BrakingSteering.hpp"
#include "Util/StopResponseScope.hpp"
#include "Util/MessageConverter.hpp"
#include "MPC/Reference/path_resampler.hpp"
#include "MPC/Reference/reference_validator.hpp"
#include "MPC/Reference/trajectory_timing.hpp"

ControlContext g_control;

namespace mpc_controller_ros {
namespace {

bool stop_response_active = false;
StopResponseScope stop_response_scope = StopResponseScope::NONE;
ros::SteadyTime behavior_received_at;
StopBrakeResponse stop_brake_response;
StopAccelerationResponse stop_acceleration_response;

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

void BehaviorCommandCallback(
    const asmc_msgs::BehaviorCommand::ConstPtr& message) {
    if (!message) return;
    const auto scope = ResolveStopResponseScope(*message);
    if (scope != stop_response_scope) {
        stop_brake_response.Reset();
        stop_acceleration_response.Reset();
    }
    stop_response_scope = scope;
    stop_response_active = scope != StopResponseScope::NONE;
    behavior_received_at = ros::SteadyTime::now();
    const bool emergency =
        message->maneuver_type == asmc_msgs::BehaviorCommand::EMERGENCY_STOP;
    if (emergency != g_control.emergency_stop_active) {
        ROS_WARN("MPC emergency brake override %s (command_id=%u)",
                 emergency ? "ACTIVE" : "RELEASED", message->command_id);
    }
    g_control.emergency_stop_active = emergency;
}

void TrajectoryCallback(
    const asmc_msgs::Trajectory::ConstPtr& message) {
    if (!message) return;
    g_control.latest_trajectory = *message;
    g_control.trajectory_received_at = ros::SteadyTime::now();
    g_control.has_trajectory = true;
}

void VehicleStateCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
    if (!message) return;
    g_control.latest_vehicle_state = *message;
    g_control.vehicle_state_received_at = ros::SteadyTime::now();
    g_control.has_vehicle_state = true;
}

void PublishStop(const char* reason) {
    stop_brake_response.Reset();
    stop_acceleration_response.Reset();
    g_control.has_successful_control = false;
    ResetMPCOnFailSafe();
    g_control.control_pub.publish(MakeRosStopCommand());
    ROS_WARN_THROTTLE(1.0, "MPC fail-safe STOP: %s", reason);
}

void ControlProcess(const ros::SteadyTimerEvent&) {
    if (g_control.emergency_stop_active) {
        stop_brake_response.Reset();
        stop_acceleration_response.Reset();
        g_control.has_successful_control = false;
        ResetMPCOnFailSafe();
        g_control.control_pub.publish(MakeRosEmergencyStopCommand());
        return;
    }
    if (!g_control.has_trajectory) {
        PublishStop("trajectory missing");
        return;
    }
    if (!IsMessageFresh(g_control.trajectory_received_at, ros::SteadyTime::now(),
                       g_control.trajectory_timeout)) {
        PublishStop("trajectory stale");
        return;
    }
    if (!g_control.has_vehicle_state) {
        PublishStop("vehicle state missing");
        return;
    }
    if (!IsMessageFresh(g_control.vehicle_state_received_at, ros::SteadyTime::now(),
                       g_control.vehicle_state_timeout)) {
        PublishStop("vehicle state stale");
        return;
    }

    // Trajectory -> PlannedPath만 변환한다. ego_at_plan의 provenance는
    // ConvertTrajectory가 내부적으로 검증하지만 MPCState로는 변환하지 않는다
    // - x0는 아래에서 최신 /vehicle/state로부터 독립적으로 만든다.
    PlannedPath path;
    const TrajectoryStatus status = ConvertTrajectory(g_control.latest_trajectory, path);
    if (status == TrajectoryStatus::STOP_TRAJECTORY) {
        stop_brake_response.Reset();
        stop_acceleration_response.Reset();
        MPCState stop_state;
        if (ConvertVehicleState(g_control.latest_vehicle_state, stop_state) != VehicleStateStatus::OK) {
            PublishStop("invalid vehicle state during planner STOP");
            return;
        }
        auto stop = MakeRosStopCommand();
        if (g_control.has_successful_control) {
            const double age=(ros::SteadyTime::now()-g_control.last_successful_control_at).toSec();
            stop.steering=BrakingSteering(g_control.last_successful_steering,age,
                stop_state.vx,GetMPCParams().steering_rate_max);
        }
        ResetMPCForPlannerStop(stop.steering);
        g_control.control_pub.publish(stop);
        ROS_WARN_THROTTLE(1.0,"MPC planner STOP braking: steering=%+.3f",stop.steering);
        return;
    }
    if (status != TrajectoryStatus::OK) {
        PublishStop(TrajectoryStatusName(status));
        return;
    }

    MPCState current_state;
    const VehicleStateStatus vehicle_status =
        ConvertVehicleState(g_control.latest_vehicle_state, current_state);
    if (vehicle_status != VehicleStateStatus::OK) {
        PublishStop(VehicleStateStatusName(vehicle_status));
        return;
    }

    const double start_time_offset = ComputeReferenceStartOffset(
        g_control.latest_trajectory.header.stamp.toSec(),
        g_control.latest_vehicle_state.header.stamp.toSec());
    const ReferencePath reference = ToReferencePath(
        path, GetMPCParams().dt, GetMPCParams().horizon,
        start_time_offset);
    if (reference.empty()) {
        PublishStop("reference interpolation failed");
        return;
    }

    // 물리적으로 말이 되는 참조인지 검증한다(방어적 defense-in-depth) -
    // 예를 들어 Planner/interface 쪽 미래 버그가 다시 불연속 참조를 보내도
    // solver가 그대로 따라가지 않는다. 여기서 거부하면 이 사이클은 ControlProcess를
    // 호출하지 않고 기존 fail-safe STOP 경로를 그대로 재사용한다.
    const ReferenceValidationResult validation =
        ValidateReference(reference, GetMPCParams());
    if (!validation.ok()) {
        if (validation.status ==
                ReferenceValidationStatus::CURVATURE_OUT_OF_RANGE &&
            validation.failed_index < reference.k_ref.size()) {
            const MPCParams& params = GetMPCParams();
            const double curvature_limit = params.steering_curvature_gain *
                std::tan(params.steering_max) /
                std::max(params.wheelbase, 1e-6);
            char reason[192];
            std::snprintf(
                reason, sizeof(reason),
                "CURVATURE_OUT_OF_RANGE index=%zu reference=%.5f limit=%.5f",
                validation.failed_index,
                reference.k_ref[validation.failed_index], curvature_limit);
            PublishStop(reason);
            return;
        }
        PublishStop(ReferenceValidationStatusName(validation.status));
        return;
    }

    const auto& response_config = GetMPCParams().stop_brake_response;
    const bool compensate = stop_response_active && response_config.enabled &&
        IsMessageFresh(behavior_received_at, ros::SteadyTime::now(),
                       g_control.trajectory_timeout);
    const MPCOutput output = MPCProcess(current_state, reference,
        compensate ? response_config.acceleration_limit
                   : std::numeric_limits<double>::quiet_NaN());
    if (output.stuck_event ==
        StuckEvent::RECOVERY_STARTED) {
        ROS_WARN("MPC stuck recovery start: ego_v=%.2f max_ref_v=%.2f "
                 "path_progress=%.2f duration=%.2fs launch_accel=%.2f",
                 output.stuck_event_ego_v, output.stuck_event_max_ref_v,
                 output.stuck_event_path_progress,
                 GetMPCParams().stuck_recovery_duration,
                 GetMPCParams().stuck_launch_accel);
    } else if (output.stuck_event ==
               StuckEvent::RECOVERY_COMPLETE) {
        ROS_INFO("MPC stuck recovery pulse complete; normal MPC resumed");
    }
    if (!output.success) {
        PublishStop(output.failure_reason ? output.failure_reason
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
        path.dt, GetMPCParams().dt, output.current_speed,
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

    g_control.last_successful_steering=output.control.delta;
    g_control.last_successful_control_at=ros::SteadyTime::now();
    g_control.has_successful_control=true;
    MPCControl mapped_control = output.control;
    if (compensate) {
        stop_brake_response.Observe(g_control.latest_vehicle_state.acceleration,
            current_state.vx,g_control.latest_vehicle_state.header.stamp.toSec(),
            response_config);
        stop_acceleration_response.Observe(g_control.latest_vehicle_state.acceleration,
            current_state.vx,g_control.latest_vehicle_state.header.stamp.toSec(),response_config);
        mapped_control.accel = output.control.accel > 0
            ? stop_acceleration_response.Compensate(output.control.accel,GetMPCParams().accel_max)
            : stop_brake_response.Compensate(
            output.control.accel,GetMPCParams().accel_min);
        stop_brake_response.Issued(mapped_control.accel,ros::Time::now().toSec(),
                                   response_config);
        stop_acceleration_response.Issued(mapped_control.accel,ros::Time::now().toSec(),response_config);
        ROS_INFO_THROTTLE(.25,"[STOP-RESPONSE] gain=%.3f throttle_gain=%.3f desired_a=%+.2f "
            "mapped_a=%+.2f measured_a=%+.2f",stop_brake_response.gain(),stop_acceleration_response.gain(),
            output.control.accel,mapped_control.accel,
            g_control.latest_vehicle_state.acceleration);
    } else {
        stop_brake_response.Reset();
        stop_acceleration_response.Reset();
    }
    g_control.control_pub.publish(ToRosControlCommand(mapped_control,
                                             GetMPCParams()));
}

}  // namespace mpc_controller_ros
