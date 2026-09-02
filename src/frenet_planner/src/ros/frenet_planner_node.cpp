#include "ros/frenet_planner_node.hpp"

#include <vector>
#include <utility>

#include "ros/message_converter.hpp"

namespace frenet_planner_ros {

FrenetPlannerNode::FrenetPlannerNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
    std::string vehicle_topic = "/vehicle/state";
    std::string objects_topic = "/perception/objects";
    std::string behavior_topic = "/behavior/context";
    std::string trajectory_topic = "/planning/trajectory";
    std::string feedback_topic = "/planning/feedback";
    std::string planner_config_path;

    pnh_.param("planning/frequency", planning_frequency_hz_, planning_frequency_hz_);
    pnh_.param("timeouts/vehicle_state", vehicle_timeout_seconds_,
               vehicle_timeout_seconds_);
    pnh_.param("timeouts/objects", objects_timeout_seconds_,
               objects_timeout_seconds_);
    pnh_.param("timeouts/behavior", behavior_timeout_seconds_,
               behavior_timeout_seconds_);
    pnh_.param<std::string>("planner/config_path", planner_config_path, "");
    pnh_.param("topics/vehicle_state", vehicle_topic, vehicle_topic);
    pnh_.param("topics/objects", objects_topic, objects_topic);
    pnh_.param("topics/behavior", behavior_topic, behavior_topic);
    pnh_.param("publish_topics/trajectory", trajectory_topic, trajectory_topic);
    pnh_.param("publish_topics/feedback", feedback_topic, feedback_topic);

    if (planning_frequency_hz_ <= 0.0) {
        ROS_WARN("Invalid planning frequency %.3f Hz; using 20 Hz",
                 planning_frequency_hz_);
        planning_frequency_hz_ = 20.0;
    }
    if (vehicle_timeout_seconds_ <= 0.0 || objects_timeout_seconds_ <= 0.0 ||
        behavior_timeout_seconds_ <= 0.0) {
        ROS_WARN("Invalid input timeout configuration; using 0.20/0.30/0.50 s");
        vehicle_timeout_seconds_ = 0.20;
        objects_timeout_seconds_ = 0.30;
        behavior_timeout_seconds_ = 0.50;
    }

    if (planner_config_path.empty()) {
        ROS_ERROR("Missing required private parameter: planner/config_path");
    } else {
        planner_initialized_ = planner_.Init(planner_config_path);
        if (!planner_initialized_) {
            ROS_ERROR_STREAM("Frenet planner initialization failed: "
                             << planner_config_path);
        }
    }

    trajectory_pub_ = nh_.advertise<asmc_msgs::Trajectory>(trajectory_topic, 1);
    feedback_pub_ = nh_.advertise<asmc_msgs::PlanFeedback>(feedback_topic, 1);

    vehicle_sub_ = nh_.subscribe(
        vehicle_topic, 1, &FrenetPlannerNode::vehicleCallback, this);
    objects_sub_ = nh_.subscribe(
        objects_topic, 1, &FrenetPlannerNode::objectsCallback, this);
    behavior_sub_ = nh_.subscribe(
        behavior_topic, 1, &FrenetPlannerNode::behaviorCallback, this);

    planning_timer_ = nh_.createTimer(
        ros::Duration(1.0 / planning_frequency_hz_),
        &FrenetPlannerNode::planningTimerCallback, this);

    ROS_INFO_STREAM("Frenet ROS runtime initialized at " << planning_frequency_hz_
                    << " Hz; inputs: " << vehicle_topic << ", "
                    << objects_topic << ", " << behavior_topic
                    << "; outputs: " << trajectory_topic << ", "
                    << feedback_topic << "; planner_initialized="
                    << (planner_initialized_ ? "true" : "false"));
}

void FrenetPlannerNode::vehicleCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_vehicle_ = *message;
    has_vehicle_ = true;
}

void FrenetPlannerNode::objectsCallback(
    const asmc_msgs::DetectedObjectArray::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_objects_ = *message;
    has_objects_ = true;
}

void FrenetPlannerNode::behaviorCallback(
    const asmc_msgs::BehaviorContext::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_behavior_ = *message;
    has_behavior_ = true;
}

FrenetPlannerNode::InputSnapshot FrenetPlannerNode::takeSnapshot() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    InputSnapshot snapshot;
    snapshot.vehicle = latest_vehicle_;
    snapshot.objects = latest_objects_;
    snapshot.behavior = latest_behavior_;
    snapshot.has_vehicle = has_vehicle_;
    snapshot.has_objects = has_objects_;
    snapshot.has_behavior = has_behavior_;
    return snapshot;
}

bool FrenetPlannerNode::isFresh(const ros::Time& stamp, double timeout_seconds,
                                const ros::Time& now,
                                double& age_seconds) const {
    constexpr double kFutureToleranceSeconds = 0.01;
    if (stamp.isZero()) {
        age_seconds = 0.0;
        return false;
    }
    age_seconds = (now - stamp).toSec();
    return age_seconds >= -kFutureToleranceSeconds &&
           age_seconds <= timeout_seconds;
}

void FrenetPlannerNode::planningTimerCallback(const ros::TimerEvent&) {
    if (!planner_initialized_) {
        ROS_ERROR_THROTTLE(
            5.0, "Frenet planning disabled: planner is not initialized");
        return;
    }

    const InputSnapshot snapshot = takeSnapshot();
    if (!snapshot.has_vehicle || !snapshot.has_objects || !snapshot.has_behavior) {
        ROS_WARN_THROTTLE(
            5.0,
            "Frenet input snapshot incomplete: vehicle=%d objects=%d behavior=%d",
            snapshot.has_vehicle, snapshot.has_objects, snapshot.has_behavior);
        return;
    }

    const ros::Time now = ros::Time::now();
    double vehicle_age = 0.0;
    double objects_age = 0.0;
    double behavior_age = 0.0;
    if (!isFresh(snapshot.vehicle.header.stamp, vehicle_timeout_seconds_,
                 now, vehicle_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale vehicle state age=%.3f s",
            vehicle_age);
        return;
    }
    if (!isFresh(snapshot.objects.header.stamp, objects_timeout_seconds_,
                 now, objects_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale objects age=%.3f s",
            objects_age);
        return;
    }
    if (!isFresh(snapshot.behavior.header.stamp, behavior_timeout_seconds_,
                 now, behavior_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale behavior age=%.3f s",
            behavior_age);
        return;
    }

    if (snapshot.behavior.force_stop) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: behavior force_stop command_id=%u",
            snapshot.behavior.command_id);
        return;
    }

    CartesianState ego{};
    PlannerCommand command{};
    std::vector<ObjectInfo> objects;

    const ConversionStatus vehicle_status =
        ConvertVehicleState(snapshot.vehicle, ego);
    if (vehicle_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: vehicle conversion failed (%s)",
            ConversionStatusName(vehicle_status));
        return;
    }

    const ConversionStatus behavior_status =
        ConvertBehaviorContext(snapshot.behavior, command);
    if (behavior_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: behavior conversion failed (%s)",
            ConversionStatusName(behavior_status));
        return;
    }

    const ConversionStatus objects_status =
        ConvertDetectedObjects(snapshot.objects, objects);
    if (objects_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: objects conversion failed (%s)",
            ConversionStatusName(objects_status));
        return;
    }

    CartesianPath path;
    if (!planner_.Plan(ego, objects, command, path)) {
        ROS_WARN_THROTTLE(
            1.0,
            "Frenet Plan(command) failed: reason=%d command_id=%u; "
            "STOP/reuse publication is pending the fail-safe stage",
            static_cast<int>(planner_.last_failure_reason()),
            snapshot.behavior.command_id);
        return;
    }

    ROS_INFO_THROTTLE(
        1.0,
        "Frenet KEEP plan ready: command_id=%u points=%zu dt=%.3f "
        "vehicle_age=%.3f objects_age=%.3f behavior_age=%.3f; "
        "trajectory publication pending",
        snapshot.behavior.command_id, path.x.size(), planner_.sample_dt(),
        vehicle_age, objects_age, behavior_age);
}

}  // namespace frenet_planner_ros
