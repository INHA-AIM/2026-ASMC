#include "ros/message_converter.hpp"

#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

asmc_msgs::VehicleState MakeVehicle() {
    asmc_msgs::VehicleState message;
    message.header.stamp = ros::Time(10, 0);
    message.header.frame_id = "map";
    message.valid = true;
    message.x = 1.0;
    message.y = 2.0;
    message.yaw = 0.3;
    message.speed = 4.0;
    message.acceleration = 0.5;
    message.yaw_rate = 0.1;
    return message;
}

asmc_msgs::BehaviorContext MakeKeepBehavior() {
    asmc_msgs::BehaviorContext message;
    message.header.stamp = ros::Time(10, 0);
    message.valid = true;
    message.selected_behavior = asmc_msgs::BehaviorContext::KEEP;
    message.has_desired_speed = true;
    message.desired_speed = 8.0;
    return message;
}

asmc_msgs::DetectedObjectArray MakeEmptyObjects() {
    asmc_msgs::DetectedObjectArray message;
    message.header.stamp = ros::Time(10, 0);
    message.header.frame_id = "map";
    return message;
}

}  // namespace

int main() {
    using frenet_planner_ros::ConversionStatus;

    CartesianState state{};
    auto vehicle = MakeVehicle();
    Expect(frenet_planner_ros::ConvertVehicleState(vehicle, state) ==
               ConversionStatus::OK &&
               state.x == 1.0 && state.y == 2.0 && state.yaw == 0.3 &&
               state.v == 4.0 && state.a == 0.5 && state.kappa == 0.0,
           "vehicle fields map to CartesianState");

    vehicle.valid = false;
    Expect(frenet_planner_ros::ConvertVehicleState(vehicle, state) ==
               ConversionStatus::INVALID_FLAG,
           "invalid vehicle flag is rejected");
    vehicle = MakeVehicle();
    vehicle.header.frame_id = "odom";
    Expect(frenet_planner_ros::ConvertVehicleState(vehicle, state) ==
               ConversionStatus::INVALID_FRAME,
           "non-map vehicle frame is rejected");
    vehicle = MakeVehicle();
    vehicle.header.stamp = ros::Time();
    Expect(frenet_planner_ros::ConvertVehicleState(vehicle, state) ==
               ConversionStatus::INVALID_TIMESTAMP,
           "zero vehicle timestamp is rejected");
    vehicle = MakeVehicle();
    vehicle.speed = std::numeric_limits<double>::quiet_NaN();
    Expect(frenet_planner_ros::ConvertVehicleState(vehicle, state) ==
               ConversionStatus::NON_FINITE_VALUE,
           "non-finite vehicle value is rejected");

    PlannerCommand command{};
    auto behavior = MakeKeepBehavior();
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::OK &&
               command.mode == LANE_KEEPING && command.target_lane == 0 &&
               command.target_speed == 8.0,
           "KEEP explicitly maps to LANE_KEEPING");
    behavior.selected_behavior = asmc_msgs::BehaviorContext::FOLLOW;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::UNSUPPORTED_BEHAVIOR,
           "non-KEEP behavior is rejected");

    std::vector<ObjectInfo> objects(1);
    auto object_message = MakeEmptyObjects();
    Expect(frenet_planner_ros::ConvertDetectedObjects(object_message, objects) ==
               ConversionStatus::OK && objects.empty(),
           "empty object message maps to empty ObjectInfo vector");
    object_message.objects.emplace_back();
    Expect(frenet_planner_ros::ConvertDetectedObjects(object_message, objects) ==
               ConversionStatus::NON_EMPTY_OBJECTS_UNSUPPORTED,
           "non-empty object message remains unsupported");

    Expect(std::string(frenet_planner_ros::ConversionStatusName(
               ConversionStatus::UNSUPPORTED_BEHAVIOR)) ==
               "UNSUPPORTED_BEHAVIOR",
           "conversion status has a stable diagnostic name");

    if (g_failures == 0) {
        std::cout << "All message converter tests passed\n";
        return 0;
    }
    return 1;
}
