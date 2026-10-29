#include <cmath>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <morai_msgs/CtrlCmd.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include "Control.hpp"
#include "Global/ControlContext.hpp"
#include "MPC/MPC.hpp"
#include "Util/ParameterLoader.hpp"
#include "Util/Handoff.hpp"
#include "Util/RecoverySpeed.hpp"

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
    std::string control_topic = "/ctrl_cmd/normal";
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
    InitMPC(params);
    g_control.control_pub = control_pub;
    g_control.trajectory_timeout = trajectory_timeout;
    g_control.vehicle_state_timeout = vehicle_state_timeout;
    nh.param("/control_binding/enabled",g_control.binding_enabled,false);
    ros::Publisher binding_epoch_pub;
    ros::Subscriber control_request_sub,control_plan_sub;
    if(g_control.binding_enabled) {
        g_control.require_verified_release=true;
        binding_epoch_pub=nh.advertise<std_msgs::String>("/control/binding_epoch",1,true);
        g_control.control_status_pub=nh.advertise<std_msgs::UInt8MultiArray>("/control/binding_status",10);
        std_msgs::String epoch;epoch.data=g_control.binding_gate.epoch();binding_epoch_pub.publish(epoch);
        control_request_sub=nh.subscribe("/behavior/control_request",10,&mpc_controller_ros::ControlRequestCallback);
        control_plan_sub=nh.subscribe("/planning/control_plan",10,&mpc_controller_ros::ControlPlanCallback);
    }
    auto trajectory_sub = nh.subscribe(
        trajectory_topic, 1, &mpc_controller_ros::TrajectoryCallback);
    auto vehicle_state_sub = nh.subscribe(
        vehicle_state_topic, 1,
        &mpc_controller_ros::VehicleStateCallback);
    auto behavior_command_sub = nh.subscribe(
        behavior_command_topic, 1,
        &mpc_controller_ros::BehaviorCommandCallback);
    const double frequency = params.control_frequency > 0.0
        ? params.control_frequency : 20.0;
    bool suspend_in_jamming=false,normal_selected=false;
    private_nh.param("suspend_in_jamming",suspend_in_jamming,false);
    double validation_speed_cap_kph=0;
    private_nh.param("validation_speed_cap_kph",validation_speed_cap_kph,0.);
    if(!std::isfinite(validation_speed_cap_kph)||validation_speed_cap_kph<0||validation_speed_cap_kph>60)
        throw std::runtime_error("invalid private validation speed cap");
    g_control.validation_speed_cap_mps=validation_speed_cap_kph/3.6;
    double approach_kph=0;private_nh.param("localization_approach_speed_kph",approach_kph,0.);
    if(!std::isfinite(approach_kph)||approach_kph<0||approach_kph>15.)
        throw std::runtime_error("invalid localization approach speed");
    g_control.localization_approach_speed_mps=approach_kph/3.6;
    ros::Subscriber approach_sub;ros::Publisher approach_trace;
    ros::SteadyTime approach_received,last_approach_trace;
    if(approach_kph>0){
        approach_sub=nh.subscribe<std_msgs::Bool>("/planning/localization/prepare",1,
          [&](const std_msgs::Bool::ConstPtr& m){
            // A missing update may retain a conservative cap, never release it.
            g_control.localization_approaching=m->data;approach_received=ros::SteadyTime::now();
          });
        approach_trace=nh.advertise<std_msgs::String>("/planning/localization/approach_trace",1);
    }
    bool smooth_handoff=false;private_nh.param("smooth_handoff",smooth_handoff,false);
    HandoffParticipant handoff;handoff.setup(nh,smooth_handoff,0);
    bool initial_anchor_ready=!smooth_handoff;
    ros::Subscriber anchor_sub;
    if(smooth_handoff)anchor_sub=nh.subscribe<std_msgs::Bool>("/planning/localization/anchor_ready",1,
      [&](const std_msgs::Bool::ConstPtr& m){initial_anchor_ready|=m->data;});
    if(smooth_handoff)g_control.before_solve=[&] {
        if(!normal_selected&&handoff.preparing())handoff.seed(g_control.latest_vehicle_state.acceleration);
    };
    if(smooth_handoff)g_control.output_observer=[&](const morai_msgs::CtrlCmd& cmd) {
        if(!g_control.handoff_solved||g_control.emergency_stop_active)handoff.reset_seed();
        handoff.publish(g_control.handoff_solved&&!g_control.emergency_stop_active,
            cmd,g_control.latest_vehicle_state.header.stamp,g_control.latest_trajectory.header.stamp,
            g_control.latest_trajectory.header.seq,g_control.latest_vehicle_state.header.frame_id,"normal_mpc");
    };
    ros::Subscriber mode_sub;
    if(suspend_in_jamming)mode_sub=nh.subscribe<std_msgs::UInt8>("/planning/mode",1,
        [&](const std_msgs::UInt8::ConstPtr& mode) {
            const bool next=mode->data==0;
            if(next!=normal_selected&&!smooth_handoff)ResetMPCOnFailSafe();
            normal_selected=next;
        });
    auto control_timer = nh.createSteadyTimer(
        ros::WallDuration(1.0 / frequency),
        [&](const ros::SteadyTimerEvent& event) {
            const auto now=ros::SteadyTime::now();
            if(approach_kph>0&&(last_approach_trace.isZero()||(now-last_approach_trace).toSec()>=.2)){
                std_msgs::String trace;
                trace.data=std::string("{\"approaching\":")+(g_control.localization_approaching?"true":"false")+
                    ",\"normal_selected\":"+(normal_selected?"true":"false")+
                    ",\"effective_cap_kph\":"+std::to_string(3.6*CombinedSpeedCap(g_control.validation_speed_cap_mps,
                       g_control.localization_approach_speed_mps,g_control.localization_approaching))+
                    ",\"prepare_age_s\":"+std::to_string(approach_received.isZero()?-1.:(now-approach_received).toSec())+"}";
                approach_trace.publish(trace);last_approach_trace=now;
            }
            // The selector owns timeout stops; an unselected NORMAL MPC need
            // not solve or publish commands while the research MPC is selected.
            if(initial_anchor_ready && (!suspend_in_jamming || normal_selected || handoff.preparing()))
                mpc_controller_ros::ControlProcess(event);
        });

    ROS_INFO("MPC ROS runtime @ %.1f Hz: %s -> %s, stale timeout %.3f s",
             frequency, trajectory_topic.c_str(), control_topic.c_str(),
             trajectory_timeout);
    ros::spin();
    return 0;
}
