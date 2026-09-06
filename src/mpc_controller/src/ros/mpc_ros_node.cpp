#include "ros/mpc_ros_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "global/parameter_loader.hpp"
#include "ros/message_converter.hpp"
#include "runtime/path_resampler.hpp"
#include "solver/mpc_solver.hpp"

namespace mpc_controller_ros {
namespace {

constexpr const char* kDefaultParamsPath =
    MPC_PACKAGE_SRC_DIR "/config/mpc_params.yaml";

}  // namespace

MpcRosNode::MpcRosNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)) {
    std::string trajectory_topic = "/planning/trajectory";
    std::string control_topic = "/ctrl_cmd";
    std::string params_path = kDefaultParamsPath;
    private_nh_.param("topics/trajectory", trajectory_topic, trajectory_topic);
    private_nh_.param("publish_topics/control", control_topic, control_topic);
    private_nh_.param("timeouts/trajectory", trajectory_timeout_,
                      trajectory_timeout_);
    private_nh_.param("mpc/config_path", params_path, params_path);

    loadMPCParameters(params_path);
    double bridge_max_steer_degrees = 36.25;
    private_nh_.param("vehicle/max_steer_deg", bridge_max_steer_degrees,
                      bridge_max_steer_degrees);
    constexpr double kRadiansToDegrees =
        180.0 / 3.14159265358979323846;
    const double mpc_max_steer_degrees =
        mpc_params.steering_max * kRadiansToDegrees;
    if (std::fabs(mpc_max_steer_degrees - bridge_max_steer_degrees) > 0.1) {
        ROS_WARN(
            "MPC max steering %.2f deg differs from MORAI bridge %.2f deg; "
            "align both to the physical vehicle limit before driving",
            mpc_max_steer_degrees, bridge_max_steer_degrees);
    }
    ego = MPCState{};
    last_control = MPCControl{};

    trajectory_sub_ = nh_.subscribe(
        trajectory_topic, 1, &MpcRosNode::trajectoryCallback, this);
    control_pub_ = nh_.advertise<morai_msgs::CtrlCmd>(control_topic, 1);
    const double frequency = mpc_params.control_frequency > 0.0
        ? mpc_params.control_frequency : 20.0;
    stuck_timeout_cycles_ = std::max(
        1, static_cast<int>(std::ceil(mpc_params.stuck_timeout * frequency)));
    recovery_duration_cycles_ = std::max(
        1, static_cast<int>(std::ceil(
               mpc_params.stuck_recovery_duration * frequency)));
    control_timer_ = nh_.createSteadyTimer(
        ros::WallDuration(1.0 / frequency),
        &MpcRosNode::controlTimerCallback, this);

    ROS_INFO("MPC ROS runtime @ %.1f Hz: %s -> %s, stale timeout %.3f s",
             frequency, trajectory_topic.c_str(), control_topic.c_str(),
             trajectory_timeout_);
}

void MpcRosNode::trajectoryCallback(
    const asmc_msgs::Trajectory::ConstPtr& message) {
    if (!message) return;
    latest_trajectory_ = *message;
    trajectory_received_at_ = ros::SteadyTime::now();
    has_trajectory_ = true;
}

void MpcRosNode::publishStop(const char* reason) {
    warm_start_.clear();
    last_control = MPCControl{};
    stuck_cycles_ = 0;
    recovery_cycles_left_ = 0;
    control_pub_.publish(MakeRosStopCommand());
    ROS_WARN_THROTTLE(1.0, "MPC fail-safe STOP: %s", reason);
}

void MpcRosNode::controlTimerCallback(const ros::SteadyTimerEvent&) {
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

    const ReferencePath reference =
        ToReferencePath(path, mpc_params.dt, mpc_params.horizon);
    if (reference.empty()) {
        publishStop("reference interpolation failed");
        return;
    }

    // UDP MPC와 같은 정지 데드락 복구. 정상 STOP은 위에서 이미 반환되며,
    // 전진 속도와 실제 경로 진행량이 함께 확인될 때만 launch assist를 허용한다.
    double max_reference_speed = 0.0;
    for (double speed : reference.v_ref) {
        max_reference_speed = std::max(max_reference_speed, speed);
    }
    double path_progress = 0.0;
    if (reference.size() >= 2) {
        path_progress = std::hypot(
            reference.x_ref.back() - reference.x_ref.front(),
            reference.y_ref.back() - reference.y_ref.front());
    }
    const bool forward_intent =
        max_reference_speed > mpc_params.stuck_target_speed &&
        path_progress > mpc_params.stuck_min_path_progress;
    const bool ego_stopped =
        std::fabs(ego_snapshot.vx) < mpc_params.stuck_ego_speed;

    if (!forward_intent || !ego_stopped) {
        stuck_cycles_ = 0;
        recovery_cycles_left_ = 0;
    } else if (recovery_cycles_left_ == 0 &&
               ++stuck_cycles_ >= stuck_timeout_cycles_) {
        warm_start_.clear();
        last_control = MPCControl{};
        recovery_cycles_left_ = recovery_duration_cycles_;
        stuck_cycles_ = 0;
        ROS_WARN("MPC stuck recovery start: ego_v=%.2f max_ref_v=%.2f "
                 "path_progress=%.2f duration=%.2fs launch_accel=%.2f",
                 ego_snapshot.vx, max_reference_speed, path_progress,
                 mpc_params.stuck_recovery_duration,
                 mpc_params.stuck_launch_accel);
    }

    ego = ego_snapshot;
    const MPCResult result = solveMPC(
        ego_snapshot, reference, last_control, warm_start_, mpc_params);
    if (!result.success ||
        result.controls.size() !=
            static_cast<std::size_t>(mpc_params.horizon) ||
        !std::isfinite(result.control.delta) ||
        !std::isfinite(result.control.accel)) {
        publishStop("solver failure");
        return;
    }

    warm_start_ = result.controls;
    last_control = result.control;
    MPCControl output_control = result.control;
    if (recovery_cycles_left_ > 0 && forward_intent && ego_stopped) {
        output_control.accel = std::max(
            output_control.accel, mpc_params.stuck_launch_accel);
        // 다음 cycle의 rate cost가 실제 출력한 가속과 이어지도록 동기화한다.
        last_control.accel = output_control.accel;
        --recovery_cycles_left_;
        if (recovery_cycles_left_ == 0) {
            ROS_INFO("MPC stuck recovery pulse complete; normal MPC resumed");
        }
    }
    control_pub_.publish(ToRosControlCommand(output_control, mpc_params));
}

}  // namespace mpc_controller_ros
