#pragma once

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>
#include <ros/ros.h>

struct ControlContext {
    ros::Publisher control_pub;
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
};

extern ControlContext g_control;
