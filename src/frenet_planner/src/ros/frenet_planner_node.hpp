#pragma once

#include <mutex>
#include <string>

#include <asmc_msgs/BehaviorContext.h>
#include <asmc_msgs/DetectedObjectArray.h>
#include <asmc_msgs/PlanFeedback.h>
#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>
#include <ros/ros.h>

#include "frenet_planner/frenet_planner.hpp"

namespace frenet_planner_ros {

class FrenetPlannerNode {
 public:
    FrenetPlannerNode(ros::NodeHandle nh, ros::NodeHandle pnh);

 private:
    struct InputSnapshot {
        asmc_msgs::VehicleState vehicle;
        asmc_msgs::DetectedObjectArray objects;
        asmc_msgs::BehaviorContext behavior;
        ros::SteadyTime vehicle_received_at;
        ros::SteadyTime objects_received_at;
        ros::SteadyTime behavior_received_at;
        bool has_vehicle = false;
        bool has_objects = false;
        bool has_behavior = false;
    };

    void vehicleCallback(const asmc_msgs::VehicleState::ConstPtr& message);
    void objectsCallback(const asmc_msgs::DetectedObjectArray::ConstPtr& message);
    void behaviorCallback(const asmc_msgs::BehaviorContext::ConstPtr& message);
    void planningTimerCallback(const ros::SteadyTimerEvent& event);
    InputSnapshot takeSnapshot() const;
    void publishStop(const InputSnapshot& snapshot, uint8_t failure_reason,
                     const ros::Time& now);
    void publishResult(const asmc_msgs::Trajectory& trajectory,
                       const InputSnapshot& snapshot,
                       bool previous_plan_still_valid);
    asmc_msgs::VehicleState stopReferenceVehicle(
        const InputSnapshot& snapshot) const;
    double trajectoryDt() const;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber vehicle_sub_;
    ros::Subscriber objects_sub_;
    ros::Subscriber behavior_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher feedback_pub_;
    ros::SteadyTimer planning_timer_;

    FrenetPlanner planner_;
    bool planner_initialized_ = false;

    mutable std::mutex input_mutex_;
    asmc_msgs::VehicleState latest_vehicle_;
    asmc_msgs::DetectedObjectArray latest_objects_;
    asmc_msgs::BehaviorContext latest_behavior_;
    ros::SteadyTime vehicle_received_at_;
    ros::SteadyTime objects_received_at_;
    ros::SteadyTime behavior_received_at_;
    bool has_vehicle_ = false;
    bool has_objects_ = false;
    bool has_behavior_ = false;

    asmc_msgs::VehicleState last_stop_reference_vehicle_;
    bool has_stop_reference_vehicle_ = false;
    asmc_msgs::Trajectory last_valid_trajectory_;
    bool has_last_valid_trajectory_ = false;
    int plan_fail_streak_ = 0;

    double planning_frequency_hz_ = 20.0;
    double vehicle_timeout_seconds_ = 0.20;
    double objects_timeout_seconds_ = 0.30;
    double behavior_timeout_seconds_ = 0.50;
    static constexpr int kMaxTransientFailCycles = 5;

};

}  // namespace frenet_planner_ros
