#include "message_converter.hpp"

#include <cmath>
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

asmc_msgs::BehaviorContext MakeStopBehavior() {
    asmc_msgs::BehaviorContext message;
    message.header.stamp = ros::Time(10, 0);
    message.valid = true;
    message.selected_behavior = asmc_msgs::BehaviorContext::STOP;
    message.has_stop_target = true;
    message.stop_x = 120.0;
    message.stop_y = -35.0;
    return message;
}

asmc_msgs::DetectedObjectArray MakeEmptyObjects() {
    asmc_msgs::DetectedObjectArray message;
    message.header.stamp = ros::Time(10, 0);
    message.header.frame_id = "map";
    return message;
}

asmc_msgs::DetectedObject MakeObject(double center_x, double center_y) {
    asmc_msgs::DetectedObject object;
    object.id = 17;
    object.type = 2;
    object.motion_state = asmc_msgs::DetectedObject::MOTION_DYNAMIC_PENDING;
    object.center.x = center_x;
    object.center.y = center_y;
    object.center.z = 1.0;
    object.yaw = 0.0;
    object.vx = 2.0;
    object.vy = -0.5;
    object.yaw_rate = 0.1;
    object.width = 2.0;
    object.length = 4.0;
    object.height = 2.0;
    const double corners[8][3] = {
        {center_x - 2.0, center_y - 1.0, 0.0},
        {center_x + 2.0, center_y + 1.0, 0.0},
        {center_x + 2.0, center_y - 1.0, 0.0},
        {center_x - 2.0, center_y + 1.0, 0.0},
        {center_x + 2.0, center_y - 1.0, 2.0},
        {center_x - 2.0, center_y + 1.0, 2.0},
        {center_x - 2.0, center_y - 1.0, 2.0},
        {center_x + 2.0, center_y + 1.0, 2.0},
    };
    for (std::size_t i = 0; i < 8; ++i) {
        object.obb_corners[i].x = corners[i][0];
        object.obb_corners[i].y = corners[i][1];
        object.obb_corners[i].z = corners[i][2];
    }
    return object;
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
               command.has_target_speed && command.target_speed == 8.0 &&
               !command.has_max_speed,
           "KEEP explicitly maps to LANE_KEEPING");

    behavior = MakeKeepBehavior();
    behavior.has_desired_speed = false;
    behavior.has_max_speed = true;
    behavior.max_speed = 6.0;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::OK &&
               !command.has_target_speed && command.has_max_speed &&
               command.max_speed == 6.0,
           "max-only behavior preserves planner-default semantics");

    behavior = MakeKeepBehavior();
    behavior.has_max_speed = true;
    behavior.max_speed = 6.0;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::OK &&
               command.has_target_speed && command.target_speed == 8.0 &&
               command.has_max_speed && command.max_speed == 6.0,
           "desired and max speeds remain separate");

    behavior = MakeKeepBehavior();
    behavior.has_desired_speed = false;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::OK &&
               !command.has_target_speed && !command.has_max_speed,
           "missing speed fields preserve planner-default semantics");

    // --- STOP (C3) ---------------------------------------------------------
    // converter는 mode만 변환한다. stop_x/y -> Frenet s 투영은 RefLine을 가진
    // 노드의 책임이므로(C4), 여기서 나온 command의 stop_position은 아직 NaN이다.
    behavior = MakeStopBehavior();
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::OK &&
               command.mode == STOP,
           "STOP explicitly maps to STOP mode");

    behavior = MakeStopBehavior();
    frenet_planner_ros::ConvertBehaviorContext(behavior, command);
    Expect(!std::isfinite(command.stop_position),
           "converter leaves stop_position unresolved for the node to project");

    behavior = MakeStopBehavior();
    behavior.has_desired_speed = true;
    behavior.desired_speed = 5.0;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::OK &&
               command.mode == STOP && command.has_target_speed &&
               command.target_speed == 5.0,
           "STOP still carries optional speed fields");

    // 정지 지점 없는 STOP은 어디에 서야 할지 모르는 명령이다.
    behavior = MakeStopBehavior();
    behavior.has_stop_target = false;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::INVALID_FLAG,
           "STOP without has_stop_target is rejected");

    behavior = MakeStopBehavior();
    behavior.stop_x = std::numeric_limits<double>::quiet_NaN();
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::NON_FINITE_VALUE,
           "STOP with non-finite stop_x is rejected");

    behavior = MakeStopBehavior();
    behavior.stop_y = std::numeric_limits<double>::infinity();
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::NON_FINITE_VALUE,
           "STOP with non-finite stop_y is rejected");

    // KEEP은 stop_x/y를 보지 않는다. FSM이 이전 STOP 값을 남겨둔 채
    // KEEP으로 전환해도 주행이 막히면 안 된다.
    behavior = MakeKeepBehavior();
    behavior.has_stop_target = false;
    behavior.stop_x = std::numeric_limits<double>::quiet_NaN();
    behavior.stop_y = std::numeric_limits<double>::quiet_NaN();
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::OK && command.mode == LANE_KEEPING,
           "KEEP ignores stop target fields");

    behavior = MakeStopBehavior();
    behavior.valid = false;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::INVALID_FLAG,
           "invalid STOP behavior is rejected");

    // --- 아직 열지 않은 모드 ------------------------------------------------
    behavior = MakeKeepBehavior();
    behavior.selected_behavior = asmc_msgs::BehaviorContext::FOLLOW;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::UNSUPPORTED_BEHAVIOR,
           "FOLLOW behavior remains unsupported");

    behavior = MakeKeepBehavior();
    behavior.selected_behavior = asmc_msgs::BehaviorContext::EMERGENCY_STOP;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::UNSUPPORTED_BEHAVIOR,
           "EMERGENCY_STOP is not silently treated as STOP");

    behavior = MakeKeepBehavior();
    behavior.selected_behavior = asmc_msgs::BehaviorContext::AVOID;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::UNSUPPORTED_BEHAVIOR,
           "AVOID behavior remains unsupported");

    behavior = MakeKeepBehavior();
    behavior.selected_behavior = asmc_msgs::BehaviorContext::MERGE;
    Expect(frenet_planner_ros::ConvertBehaviorContext(behavior, command) ==
               ConversionStatus::UNSUPPORTED_BEHAVIOR,
           "MERGE behavior remains unsupported");

    std::vector<TrackedObject> objects(1);
    auto object_message = MakeEmptyObjects();
    Expect(frenet_planner_ros::ConvertDetectedObjects(
               object_message, objects) ==
               ConversionStatus::OK && objects.empty(),
           "empty object message maps to an empty planner obstacle vector");

    object_message.objects.push_back(MakeObject(10.0, 20.0));
    Expect(frenet_planner_ros::ConvertDetectedObjects(
               object_message, objects) ==
               ConversionStatus::OK && objects.size() == 1 &&
               objects[0].id == 17 &&
               objects[0].motion_state ==
                   ObjectMotionState::DYNAMIC_PENDING &&
               objects[0].vx == 2.0 && objects[0].vy == -0.5 &&
               objects[0].geometry_valid &&
               objects[0].z_valid &&
               objects[0].corners[1].x == 12.0 &&
               objects[0].corners[1].y == 21.0 &&
               objects[0].corners[1].z == 0.0,
           "LiDAR OBB corners are copied unchanged to TrackedObject");

    auto malformed_message = MakeEmptyObjects();
    malformed_message.objects.push_back(MakeObject(10.0, 20.0));
    malformed_message.objects[0].obb_corners[3].x =
        std::numeric_limits<double>::quiet_NaN();
    Expect(frenet_planner_ros::ConvertDetectedObjects(
               malformed_message, objects) ==
               ConversionStatus::INVALID_OBJECT_GEOMETRY,
           "non-finite LiDAR corner is rejected without geometry fallback");

    auto zero_corners_message = MakeEmptyObjects();
    zero_corners_message.objects.push_back(MakeObject(10.0, 20.0));
    for (auto& corner : zero_corners_message.objects[0].obb_corners) {
        corner.x = 0.0;
        corner.y = 0.0;
        corner.z = 0.0;
    }
    Expect(frenet_planner_ros::ConvertDetectedObjects(
               zero_corners_message, objects) ==
               ConversionStatus::INVALID_OBJECT_GEOMETRY,
           "an object with no populated OBB corners is rejected, not "
           "synthesized from center/width/length/yaw");

    auto duplicate_message = MakeEmptyObjects();
    duplicate_message.objects.push_back(MakeObject(10.0, 20.0));
    duplicate_message.objects.push_back(MakeObject(30.0, 40.0));
    Expect(frenet_planner_ros::ConvertDetectedObjects(
               duplicate_message, objects) ==
               ConversionStatus::DUPLICATE_OBJECT_ID,
           "duplicate object IDs are rejected");

    Expect(std::string(frenet_planner_ros::ConversionStatusName(
               ConversionStatus::UNSUPPORTED_BEHAVIOR)) ==
               "UNSUPPORTED_BEHAVIOR",
           "conversion status has a stable diagnostic name");

    CartesianPath path;
    path.x = {1.0, 2.0};
    path.y = {3.0, 4.0};
    path.yaw = {0.1, 0.2};
    path.kappa = {0.01, 0.02};
    path.v = {5.0, 6.0};
    path.a = {0.3, 0.4};
    asmc_msgs::Trajectory trajectory;
    const auto ego_at_plan = MakeVehicle();

    // Commit E / Test 11.1: trajectory.header.stamp는 callback 수신 시각이
    // 아니라 ego_at_plan(계획에 쓰인 Ego snapshot)의 source time과 항상
    // 같아야 한다. API에 별도 plan_stamp를 받지 않으므로 호출자가 실수로
    // 다른 시각(예: ros::Time::now())을 넣을 수 없다.
    Expect(frenet_planner_ros::ConvertCartesianPath(
               path, 0.1, ego_at_plan, trajectory) ==
               ConversionStatus::OK && trajectory.valid &&
               trajectory.failure_reason == asmc_msgs::Trajectory::FAILURE_NONE &&
               trajectory.header.frame_id == "map" &&
               trajectory.header.stamp == ego_at_plan.header.stamp &&
               trajectory.dt == 0.1 &&
               trajectory.ego_at_plan.x == ego_at_plan.x &&
               trajectory.points.size() == 2 &&
               trajectory.points[1].x == 2.0 &&
               trajectory.points[1].curvature == 0.02 &&
               trajectory.points[1].velocity == 6.0,
           "CartesianPath maps to valid map-frame Trajectory stamped with "
           "ego_at_plan's source time");

    path.a.pop_back();
    Expect(frenet_planner_ros::ConvertCartesianPath(
               path, 0.1, ego_at_plan, trajectory) ==
               ConversionStatus::INVALID_PATH,
           "trajectory conversion rejects mismatched vector sizes");

    const asmc_msgs::Trajectory stop = frenet_planner_ros::MakeStopTrajectory(
        ego_at_plan, 0.1, asmc_msgs::Trajectory::FAILURE_STATE_STALE);
    Expect(stop.valid &&
               stop.failure_reason == asmc_msgs::Trajectory::FAILURE_STATE_STALE &&
               stop.header.stamp == ego_at_plan.header.stamp &&
               stop.dt == 0.1 && stop.ego_at_plan.x == ego_at_plan.x &&
               stop.points.size() == 1 && stop.points[0].x == ego_at_plan.x &&
               stop.points[0].y == ego_at_plan.y &&
               stop.points[0].yaw == ego_at_plan.yaw &&
               stop.points[0].velocity == 0.0 &&
               stop.points[0].acceleration == 0.0,
           "STOP trajectory is a valid executable zero-speed command "
           "stamped with ego_at_plan's source time");

    // Test 11.2: ego_at_plan의 source time이 무효(0)면 trajectory에도 그대로
    // 반영된다 - now()로 위장해 유효한 것처럼 보이게 하지 않는다. 소비자
    // (MPC ConvertTrajectory)가 header.stamp.isZero()를 독립적으로 거부해
    // fail-safe STOP을 낼 수 있어야 하기 때문이다.
    CartesianPath valid_path;
    valid_path.x = {1.0, 2.0};
    valid_path.y = {3.0, 4.0};
    valid_path.yaw = {0.1, 0.2};
    valid_path.kappa = {0.01, 0.02};
    valid_path.v = {5.0, 6.0};
    valid_path.a = {0.3, 0.4};
    asmc_msgs::VehicleState unknown_ego = ego_at_plan;
    unknown_ego.valid = false;
    unknown_ego.header.stamp = ros::Time();  // zero: 측정된 적 없음
    asmc_msgs::Trajectory unknown_trajectory;
    Expect(frenet_planner_ros::ConvertCartesianPath(
               valid_path, 0.1, unknown_ego, unknown_trajectory) ==
               ConversionStatus::OK &&
               unknown_trajectory.header.stamp.isZero() &&
               unknown_trajectory.ego_at_plan.valid == false,
           "invalid/unknown ego_at_plan source time is not silently "
           "replaced with now() when converting a path");

    const asmc_msgs::Trajectory unknown_stop =
        frenet_planner_ros::MakeStopTrajectory(
            unknown_ego, 0.1, asmc_msgs::Trajectory::FAILURE_STATE_STALE);
    Expect(unknown_stop.header.stamp.isZero() &&
               unknown_stop.ego_at_plan.valid == false,
           "MakeStopTrajectory does not manufacture a fresh timestamp for "
           "an unknown/invalid ego_at_plan snapshot");

    if (g_failures == 0) {
        std::cout << "All message converter tests passed\n";
        return 0;
    }
    return 1;
}
