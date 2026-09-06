#include "mpc_controller_node.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "config/parameter_loader.hpp"
#include "message_converter.hpp"
#include "reference/path_resampler.hpp"

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
    private_nh_.param("topics/trajectory", trajectory_topic, trajectory_topic);
    private_nh_.param("publish_topics/control", control_topic, control_topic);
    private_nh_.param("timeouts/trajectory", trajectory_timeout_,
                      trajectory_timeout_);

    double bridge_max_steer_degrees = 36.25;
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

void MpcControllerNode::trajectoryCallback(
    const asmc_msgs::Trajectory::ConstPtr& message) {
    if (!message) return;
    latest_trajectory_ = *message;
    trajectory_received_at_ = ros::SteadyTime::now();
    has_trajectory_ = true;
}

void MpcControllerNode::publishStop(const char* reason) {
    controller_.ResetOnFailSafe();
    control_pub_.publish(MakeRosStopCommand());
    ROS_WARN_THROTTLE(1.0, "MPC fail-safe STOP: %s", reason);
}

void MpcControllerNode::controlTimerCallback(const ros::SteadyTimerEvent&) {
    if (!has_trajectory_) {
        publishStop("trajectory missing");
        return;
    }
    if (!IsTrajectoryFresh(trajectory_received_at_, ros::SteadyTime::now(),
                           trajectory_timeout_)) {
        publishStop("trajectory stale");
        return;
    }

    PlannedPath path;
    MPCState ego_snapshot;
    const TrajectoryStatus status =
        ConvertTrajectory(latest_trajectory_, path, ego_snapshot);
    if (status == TrajectoryStatus::STOP_TRAJECTORY) {
        publishStop("valid planner STOP trajectory");
        return;
    }
    if (status != TrajectoryStatus::OK) {
        publishStop(TrajectoryStatusName(status));
        return;
    }

    const ReferencePath reference = ToReferencePath(
        path, controller_.params().dt, controller_.params().horizon);
    if (reference.empty()) {
        publishStop("reference interpolation failed");
        return;
    }

    const MpcController::Output output =
        controller_.ControlProcess(ego_snapshot, reference);
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
