#pragma once

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>
#include <ros/ros.h>
#include <morai_msgs/CtrlCmd.h>
#include <functional>
#include <asmc_msgs/control_binding.hpp>
#include "Util/EmergencySteering.hpp"
#include "Util/BrakeResponseWatchdog.hpp"

struct ControlContext {
    ros::Publisher control_pub;
    std::function<void(const morai_msgs::CtrlCmd&)> output_observer;
    std::function<void()> before_solve;
    double validation_speed_cap_mps=0; // private trial fixture; production disabled
    bool handoff_solved=false;
    asmc_msgs::Trajectory latest_trajectory;
    bool has_trajectory = false;
    ros::SteadyTime trajectory_received_at;
    double trajectory_timeout = 0.25;
    asmc_msgs::VehicleState latest_vehicle_state;
    bool has_vehicle_state = false;
    ros::SteadyTime vehicle_state_received_at;
    double vehicle_state_timeout = 0.20;
    // Last valid published command: normal MPC or a valid planner STOP.
    // Watchdog/emergency resets invalidate it; STOP ticks advance this anchor.
    double last_successful_steering = 0.0;
    ros::SteadyTime last_successful_control_at;
    bool has_successful_control = false;
    bool emergency_stop_active = false;
    EmergencySteeringState emergency_steering;
    double braking_geometry_stamp=0;
    // Enabled by explicit wire binding. The legacy-only test gate also uses
    // this latch to reject unrelated fresh trajectories.
    bool require_verified_release=false;
    std::uint32_t pending_release_command=0;
    bool binding_enabled=false;
    double last_published_brake=0;
    asmc_msgs::binding::Gate binding_gate;
    ros::SteadyTime binding_request_received_at,binding_plan_received_at;
    ros::SteadyTime pending_plan_received_at;
    std::uint64_t pending_plan_sequence=0;
    std::string pending_planner_epoch;
    ros::Publisher control_status_pub;
    std::uint64_t output_sequence=0;
    BrakeResponseWatchdog brake_watchdog;
    double stopped_since=-1;
};

extern ControlContext g_control;
