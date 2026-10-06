#include <cmath>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <morai_msgs/CtrlCmd.h>

#include "Control.hpp"
#include "Util/ParameterLoader.hpp"

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    const MPCParams params = LoadParamsForNode(private_nh);

    std::string trajectory_topic = "/planning/trajectory";
    std::string control_topic = "/ctrl_cmd";
    std::string behavior_command_topic = "/behavior/command_v2";
    std::string vehicle_state_topic = "/vehicle/state";
    private_nh.param("topics/trajectory", trajectory_topic, trajectory_topic);
    private_nh.param("publish_topics/control", control_topic, control_topic);
    private_nh.param("topics/behavior_command_v2", behavior_command_topic,
                     behavior_command_topic);
    private_nh.param("topics/vehicle_state", vehicle_state_topic,
                     vehicle_state_topic);
    double trajectory_timeout = 0.25;
    double vehicle_state_timeout = 0.20;
    private_nh.param("timeouts/trajectory", trajectory_timeout,
                     trajectory_timeout);
    private_nh.param("timeouts/vehicle_state", vehicle_state_timeout,
                     vehicle_state_timeout);

    double bridge_max_steer_degrees = 40.0;
    private_nh.param("vehicle/max_steer_deg", bridge_max_steer_degrees,
                     bridge_max_steer_degrees);
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    const double mpc_max_steer_degrees =
        params.steering_max * kRadiansToDegrees;
    if (std::fabs(mpc_max_steer_degrees - bridge_max_steer_degrees) > 0.1) {
        ROS_WARN(
            "MPC max steering %.2f deg differs from MORAI bridge %.2f deg; "
            "align both to the physical vehicle limit before driving",
            mpc_max_steer_degrees, bridge_max_steer_degrees);
    }

    auto control_pub = nh.advertise<morai_msgs::CtrlCmd>(control_topic, 1);
    mpc_controller_ros::Control control(params, control_pub,
                                       trajectory_timeout,
                                       vehicle_state_timeout);
    auto trajectory_sub = nh.subscribe(
        trajectory_topic, 1, &mpc_controller_ros::Control::trajectoryCallback,
        &control);
    auto vehicle_state_sub = nh.subscribe(
        vehicle_state_topic, 1,
        &mpc_controller_ros::Control::vehicleStateCallback, &control);
    auto behavior_command_sub = nh.subscribe(
        behavior_command_topic, 1,
        &mpc_controller_ros::Control::behaviorCommandCallback, &control);
    const double frequency = params.control_frequency > 0.0
        ? params.control_frequency : 20.0;
    auto control_timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / frequency),
        &mpc_controller_ros::Control::ControlProcess, &control);

    ROS_INFO("MPC ROS runtime @ %.1f Hz: %s -> %s, stale timeout %.3f s",
             frequency, trajectory_topic.c_str(), control_topic.c_str(),
             trajectory_timeout);
    ros::spin();
    return 0;
}
