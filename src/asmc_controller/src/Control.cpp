#include "Control.hpp"
#include "Global/ControlContext.hpp"
#include "MPC/MPC.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <future>
#include <chrono>
#include "MPC/Reference/braking_reference.hpp"
#include "MPC/Solver/mpc_solver.hpp"

#include "Util/BrakingSteering.hpp"
#include "Util/StopResponseScope.hpp"
#include "Util/RecoverySpeed.hpp"
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
struct EmergencySteeringResult {
    std::uint64_t generation=0;
    double geometry_stamp=0,started=0,steering=0;
    bool valid=false;
};
std::future<EmergencySteeringResult> emergency_steering_job;
std::uint64_t emergency_generation=0;

void PublishControl(const morai_msgs::CtrlCmd& command) {
    g_control.last_published_brake=command.brake;
    g_control.control_pub.publish(command);
    if(!g_control.binding_enabled || !g_control.control_status_pub || !g_control.binding_gate.has_request())return;
    const auto& gate=g_control.binding_gate;
    const auto& pending=gate.pending();
    asmc_msgs::binding::Status status;status.token=gate.request().token;
    status.output_sequence=++g_control.output_sequence;status.actuated=true;
    status.brake=command.brake;status.steering=command.steering;status.accel=command.accel;
    status.execution_failed=g_control.brake_watchdog.failed();
    status.failure_mask=g_control.brake_watchdog.failure_mask();
    status.brake_generation=g_control.brake_watchdog.generation();
    status.brake_level=g_control.brake_watchdog.level();
    status.response_observed=g_control.brake_watchdog.observed();
    status.response_remaining=g_control.brake_watchdog.remaining(ros::SteadyTime::now().toSec());
    status.linked=gate.linked() &&
        IsMessageFresh(g_control.binding_plan_received_at,ros::SteadyTime::now(),g_control.trajectory_timeout);
    if(asmc_msgs::binding::SameRequest(pending.request,gate.request())) {
        status.planner_epoch=pending.planner_epoch;status.plan_sequence=pending.sequence;
        status.planning_checked=true;status.stop_path_valid=pending.feedback.command_accepted &&
            (pending.feedback.plan_valid || pending.stationary_hold) && pending.trajectory.valid;
        status.goal_satisfied=pending.goal_satisfied;status.exit_path_valid=pending.exit_path_valid;
        status.execution_recovery_safe=pending.execution_recovery_safe;
    }
    g_control.control_status_pub.publish(asmc_msgs::binding::Encode(status));
}

void PublishBindingHold() {
    auto command=MakeRosStopCommand();
    command.brake=std::max(command.brake,g_control.last_published_brake);
    MPCState state;
    const auto now=ros::SteadyTime::now();
    if(g_control.has_successful_control &&
       ConvertVehicleState(g_control.latest_vehicle_state,state)==VehicleStateStatus::OK) {
        command.steering=BrakingSteering(g_control.last_successful_steering,
            (now-g_control.last_successful_control_at).toSec(),state.vx,GetMPCParams());
        g_control.last_successful_steering=command.steering;g_control.last_successful_control_at=now;
    }
    // A brief identity handshake must not erase the valid curve geometry that
    // the next emergency callback needs. Its original source stamp is retained.
    ResetMPCForPlannerStop(command.steering);PublishControl(command);
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

void PublishEmergencyStop() {
    const auto now=ros::SteadyTime::now();
    auto& saved=g_control.emergency_steering;
    if(!saved.active) saved.Capture(g_control.last_successful_steering,now.toSec(),
        g_control.has_successful_control &&
        IsMessageFresh(g_control.last_successful_control_at,now,g_control.trajectory_timeout),
        ++emergency_generation,g_control.braking_geometry_stamp);
    MPCState state;
    const bool state_valid=g_control.has_vehicle_state &&
        IsMessageFresh(g_control.vehicle_state_received_at,now,g_control.vehicle_state_timeout) &&
        ConvertVehicleState(g_control.latest_vehicle_state,state)==VehicleStateStatus::OK;
    auto command=MakeRosEmergencyStopCommand();
    command.steering=saved.Fallback(now.toSec(),state.vx,state_valid,GetMPCParams());
    if(emergency_steering_job.valid() &&
       emergency_steering_job.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        try {
            const auto result=emergency_steering_job.get();
            if(result.valid && state_valid && result.generation==saved.generation &&
               result.geometry_stamp==saved.geometry_stamp &&
               now.toSec()-result.started<=g_control.trajectory_timeout)
                command.steering=saved.Bound(result.steering,now.toSec(),state.vx,GetMPCParams());
        } catch (...) { /* Scalar fallback already selected; braking is unconditional. */ }
    }
    stop_brake_response.Reset();
    stop_acceleration_response.Reset();
    g_control.has_successful_control = false;
    ResetMPCForPlannerStop(command.steering);
    PublishControl(command);  // Never wait for the steering solver.
    saved.Published(command.steering,now.toSec());
    // Immutable local copies: worker never accesses or resets g_mpc/g_control.
    const double ros_now=ros::Time::now().toSec();
    if(!emergency_steering_job.valid() && state_valid && saved.valid &&
       saved.geometry_stamp>0 && ros_now>=saved.geometry_stamp &&
       ros_now-saved.geometry_stamp<=g_control.trajectory_timeout &&
       g_mpc.braking_geometry.size()>=2) {
        const auto geometry=g_mpc.braking_geometry;
        const auto history=g_mpc.steering_history;
        const auto previous=g_mpc.last_control;
        const auto params=GetMPCParams();
        const auto generation=saved.generation;
        const double stamp=saved.geometry_stamp,started=now.toSec();
        try {
            emergency_steering_job=std::async(std::launch::async,
                [geometry,history,previous,params,state,generation,stamp,started]() {
                    EmergencySteeringResult out;out.generation=generation;
                    out.geometry_stamp=stamp;out.started=started;
                    const double decel=params.brake_full_decel;
                    const auto reference=MakeBrakingReference(geometry,state,decel,params,false);
                    if(reference.empty())return out;
                    auto fixed=params;fixed.accel_min=fixed.accel_max=-decel;
                    const auto result=solveMPC(state,reference,previous,{},fixed,history);
                    out.valid=result.success && std::isfinite(result.control.delta);
                    out.steering=result.control.delta;return out;
                });
        } catch (...) { /* Brake was already published. */ }
    }
}

}  // namespace

void BehaviorCommandCallback(
    const asmc_msgs::BehaviorCommand::ConstPtr& message) {
    if (!message) return;
    if(g_control.binding_enabled)return; // Mirror only; never releases a bound latch.
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
    if(!emergency && g_control.emergency_stop_active && g_control.require_verified_release) {
        // There is no wire proof to consume yet. Heartbeats keep this waiting
        // state; neither timestamps nor a newly received trajectory unlock it.
        if(message->command_id!=g_control.pending_release_command)
            g_control.pending_release_command=message->command_id;
        return;
    }
    if (emergency != g_control.emergency_stop_active) {
        ROS_WARN("MPC emergency brake override %s (command_id=%u)",
                 emergency ? "ACTIVE" : "RELEASED", message->command_id);
    }
    g_control.emergency_stop_active = emergency;
    if(!emergency) g_control.emergency_steering={};
    // The runtime uses a single ROS callback queue (ros::spin). Do not leave
    // an accepted emergency waiting for the next 20 Hz control timer; reuse
    // exactly the same reset/output path as subsequent emergency ticks.
    if (emergency) PublishEmergencyStop();
}

void TrajectoryCallback(
    const asmc_msgs::Trajectory::ConstPtr& message) {
    if (!message) return;
    if(g_control.binding_enabled)return;
    g_control.latest_trajectory = *message;
    g_control.trajectory_received_at = ros::SteadyTime::now();
    g_control.has_trajectory = true;
}

void ControlRequestCallback(const std_msgs::UInt8MultiArray::ConstPtr& message) {
    if(!g_control.binding_enabled || !message)return;
    asmc_msgs::binding::Request request;
    if(!asmc_msgs::binding::Decode(*message,request) ||
       !g_control.binding_gate.RequestReceived(request))return;
    g_control.binding_request_received_at=ros::SteadyTime::now();
    const auto scope=ResolveStopResponseScope(request.command);
    if(scope!=stop_response_scope) {stop_brake_response.Reset();stop_acceleration_response.Reset();}
    stop_response_scope=scope;stop_response_active=scope!=StopResponseScope::NONE;
    behavior_received_at=g_control.binding_request_received_at;
    g_control.brake_watchdog.Request(request,ros::Time::now().toSec(),g_control.binding_request_received_at.toSec());
    if(g_control.binding_gate.linked() &&
       g_control.binding_gate.plan().sequence==g_control.pending_plan_sequence &&
       g_control.binding_gate.plan().planner_epoch==g_control.pending_planner_epoch)
        g_control.binding_plan_received_at=g_control.pending_plan_received_at;
    if(request.command.maneuver_type==asmc_msgs::BehaviorCommand::EMERGENCY_STOP ||
       (request.brake_level==2 && !request.release_authorized)) {
        g_control.emergency_stop_active=true;
        PublishEmergencyStop();
    }
}

void ControlPlanCallback(const std_msgs::UInt8MultiArray::ConstPtr& message) {
    if(!g_control.binding_enabled || !message)return;
    asmc_msgs::binding::Plan plan;
    if(!asmc_msgs::binding::Decode(*message,plan))return;
    const bool linked=g_control.binding_gate.PlanReceived(plan);
    if(g_control.binding_gate.pending().sequence==plan.sequence &&
       g_control.binding_gate.pending().planner_epoch==plan.planner_epoch) {
        g_control.pending_plan_received_at=ros::SteadyTime::now();
        g_control.pending_plan_sequence=plan.sequence;g_control.pending_planner_epoch=plan.planner_epoch;
        if(linked)g_control.binding_plan_received_at=g_control.pending_plan_received_at;
    }
    if(g_control.binding_gate.has_request() && plan.request.signal.enabled &&
       asmc_msgs::binding::SameRequest(plan.request,g_control.binding_gate.request()) &&
       (!plan.feedback.command_accepted || (!plan.feedback.plan_valid && !plan.stationary_hold) ||
        !plan.trajectory.valid || !plan.goal_satisfied)) {
        g_control.brake_watchdog.Fail();
        g_control.emergency_stop_active=true;PublishEmergencyStop();
    }
}

void VehicleStateCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
    if (!message) return;
    g_control.latest_vehicle_state = *message;
    g_control.vehicle_state_received_at = ros::SteadyTime::now();
    g_control.has_vehicle_state = true;
    if(message->valid && std::isfinite(message->speed) && std::abs(message->speed)<=.15) {
        if(g_control.stopped_since<0)g_control.stopped_since=ros::SteadyTime::now().toSec();
    } else g_control.stopped_since=-1;
    if(g_control.binding_enabled && g_control.brake_watchdog.Observe(ros::SteadyTime::now().toSec(),
       message->header.stamp.toSec(),message->speed,message->acceleration,message->valid)) {
        g_control.emergency_stop_active=true;PublishEmergencyStop();
    }
}

void PublishStop(const char* reason) {
    if(g_control.binding_enabled && g_control.binding_gate.has_request()) {
        g_control.brake_watchdog.Fail(2);g_control.emergency_stop_active=true;
        PublishEmergencyStop();
        ROS_WARN_THROTTLE(1.,"Bound Controller path execution failure: %s",reason);
        return;
    }
    g_control.emergency_steering={};
    g_control.braking_geometry_stamp=0;
    stop_brake_response.Reset();
    stop_acceleration_response.Reset();
    g_control.has_successful_control = false;
    ResetMPCOnFailSafe();
    PublishControl(MakeRosStopCommand());
    ROS_WARN_THROTTLE(1.0, "MPC fail-safe STOP: %s", reason);
}

void ControlProcess(const ros::SteadyTimerEvent&) {
    if(g_control.binding_enabled) {
        auto& gate=g_control.binding_gate;
        const auto now=ros::SteadyTime::now();
        const bool request_fresh=gate.has_request() &&
            IsMessageFresh(g_control.binding_request_received_at,now,.5) &&
            gate.request().command.valid_until>=ros::Time::now();
        MPCState checked_state;
        const double state_age=ros::Time::now().toSec()-g_control.latest_vehicle_state.header.stamp.toSec();
        const bool state_ready=g_control.has_vehicle_state &&
            IsMessageFresh(g_control.vehicle_state_received_at,now,g_control.vehicle_state_timeout) &&
            state_age>=0 && state_age<=g_control.vehicle_state_timeout &&
            ConvertVehicleState(g_control.latest_vehicle_state,checked_state)==VehicleStateStatus::OK;
        const bool linked=request_fresh && state_ready && gate.linked() &&
            (!gate.request().recovery_release || gate.plan().exit_path_valid) &&
            IsMessageFresh(g_control.binding_plan_received_at,now,g_control.trajectory_timeout);
        const bool emergency=request_fresh && (gate.request().command.maneuver_type==asmc_msgs::BehaviorCommand::EMERGENCY_STOP ||
            (gate.request().brake_level==2 && !gate.request().release_authorized));
        if(linked && gate.request().execution_recovery_ack && gate.request().release_authorized &&
           gate.plan().execution_recovery_safe && g_control.stopped_since>=0 && now.toSec()-g_control.stopped_since>=.2) {
            PlannedPath check;
            const auto converted=ConvertTrajectory(gate.plan().trajectory,check);
            const bool valid=converted==TrajectoryStatus::STOP_TRAJECTORY ||
                (converted==TrajectoryStatus::OK && ValidateReference(ToReferencePath(check,GetMPCParams().dt,
                    GetMPCParams().horizon,ComputeReferenceStartOffset(gate.plan().trajectory.header.stamp.toSec(),
                    g_control.latest_vehicle_state.header.stamp.toSec())),GetMPCParams()).ok());
            // A planner proof cannot clear the controller's failure until its
            // own conversion and full reference checks accept that same path.
            g_control.brake_watchdog.AcknowledgePathRecovery(valid);
        }
        const bool execution_failed=g_control.brake_watchdog.Observe(now.toSec(),0,0,0,false);
        if(emergency || !linked || execution_failed) {
            if(g_control.emergency_stop_active || emergency || execution_failed) {
                g_control.emergency_stop_active=true;PublishEmergencyStop();
            } else PublishBindingHold();
            return;
        }
        if(!gate.request().release_authorized &&
           gate.request().command.maneuver_type!=asmc_msgs::BehaviorCommand::STOP) {
            if(g_control.emergency_stop_active)PublishEmergencyStop();
            else PublishBindingHold();
            return;
        }
        // The plan atomically contains acceptance and the actual trajectory.
        // The arbiter has already checked normal/recovery release conditions.
        if(g_control.emergency_stop_active) {
            PlannedPath check;
            const auto status=ConvertTrajectory(gate.plan().trajectory,check);
            if(status!=TrajectoryStatus::STOP_TRAJECTORY &&
               (status!=TrajectoryStatus::OK || !ValidateReference(ToReferencePath(check,GetMPCParams().dt,
                    GetMPCParams().horizon,ComputeReferenceStartOffset(gate.plan().trajectory.header.stamp.toSec(),
                    g_control.latest_vehicle_state.header.stamp.toSec())),GetMPCParams()).ok())) {
                PublishEmergencyStop();return;
            }
            g_control.emergency_steering={};
        }
        g_control.emergency_stop_active=false;
        if(!gate.request().brake_level)g_control.brake_watchdog.Released();
        g_control.latest_trajectory=gate.plan().trajectory;
        g_control.trajectory_received_at=g_control.binding_plan_received_at;
        g_control.has_trajectory=true;
        stop_response_active=gate.request().command.maneuver_type==asmc_msgs::BehaviorCommand::STOP;
    }
    if (g_control.emergency_stop_active) {
        PublishEmergencyStop();
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
    // - x0는 아래에서 설정된 최신 vehicle-state 토픽으로부터 독립적으로 만든다.
    // 통합 launch는 /planning/vehicle/state, 단독 기본은 /vehicle/state다.
    PlannedPath path;
    const TrajectoryStatus status = ConvertTrajectory(g_control.latest_trajectory, path);
    if (status == TrajectoryStatus::STOP_TRAJECTORY) {
        // A planner failure is not a completed, intentional STOP. In
        // particular, a collision-blocked one-point fallback at road speed
        // needs maximum braking immediately; the ordinary hold uses only 0.3.
        if (g_control.latest_trajectory.failure_reason ==
            asmc_msgs::Trajectory::FAILURE_COLLISION_BLOCKED) {
            g_control.emergency_stop_active = true;
            PublishEmergencyStop();
            ROS_WARN_THROTTLE(1.0, "Planner trajectory collision blocked: emergency brake");
            return;
        }
        stop_brake_response.Reset();
        stop_acceleration_response.Reset();
        MPCState stop_state;
        if (ConvertVehicleState(g_control.latest_vehicle_state, stop_state) != VehicleStateStatus::OK) {
            PublishStop("invalid vehicle state during planner STOP");
            return;
        }
        auto stop = MakeRosStopCommand();
        if (g_control.has_successful_control) {
            const auto now = ros::SteadyTime::now();
            const double elapsed = (now - g_control.last_successful_control_at).toSec();
            stop.steering = BrakingSteering(g_control.last_successful_steering,
                elapsed, stop_state.vx, GetMPCParams());
            MPCPlannerStopSteering(stop_state, stop.brake, stop.steering);
            g_control.last_successful_steering = stop.steering;
            g_control.last_successful_control_at = now;
        }
        ResetMPCForPlannerStop(stop.steering);
        PublishControl(stop);
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
    const bool signal_braking=g_control.binding_enabled && g_control.binding_gate.has_request() &&
        g_control.binding_gate.request().signal.enabled && g_control.binding_gate.request().signal.braking_only;
    double acceleration_limit=signal_braking ? 0. : compensate ? response_config.acceleration_limit :
        std::numeric_limits<double>::quiet_NaN();
    if(g_control.binding_enabled && g_control.binding_gate.has_request() &&
       g_control.binding_gate.request().recovery_release) {
        const auto& request=g_control.binding_gate.request();
        const double recovery_limit=RecoveryAccelerationLimit(request.command.desired_speed_mps,current_state.vx,
            g_control.latest_vehicle_state.acceleration,request.brake_budget,
            GetMPCParams().accel_min,GetMPCParams().accel_max);
        acceleration_limit=std::isfinite(acceleration_limit) ? std::min(acceleration_limit,recovery_limit) : recovery_limit;
    }
    const MPCOutput output = MPCProcess(current_state, reference,acceleration_limit);
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
    g_control.braking_geometry_stamp=g_control.latest_trajectory.header.stamp.toSec();
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
    if(signal_braking)
        mapped_control.accel=std::min(current_state.vx>.15 ? -response_config.minimum_request : 0.,mapped_control.accel);
    PublishControl(ToRosControlCommand(mapped_control,
                                             GetMPCParams()));
}

}  // namespace mpc_controller_ros
