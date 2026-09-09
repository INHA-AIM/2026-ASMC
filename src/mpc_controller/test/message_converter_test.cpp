#include <gtest/gtest.h>

#include <limits>

#include "message_converter.hpp"
#include "reference/path_resampler.hpp"

namespace mpc_controller_ros {
namespace {

asmc_msgs::Trajectory MakeTrajectory() {
    asmc_msgs::Trajectory message;
    message.header.stamp = ros::Time(10.0);
    message.header.frame_id = "map";
    message.valid = true;
    message.dt = 0.1;
    message.ego_at_plan.header.frame_id = "map";
    message.ego_at_plan.valid = true;
    message.ego_at_plan.x = 1.0;
    message.ego_at_plan.y = 2.0;
    message.ego_at_plan.yaw = 0.2;
    message.ego_at_plan.speed = 3.0;

    asmc_msgs::TrajectoryPoint first;
    first.x = 1.0;
    first.y = 2.0;
    first.yaw = 0.2;
    first.velocity = 3.0;
    first.curvature = 0.01;
    asmc_msgs::TrajectoryPoint second = first;
    second.x = 2.0;
    second.y = 4.0;
    second.yaw = 0.4;
    second.velocity = 5.0;
    second.curvature = 0.03;
    message.points = {first, second};
    return message;
}

TEST(TrajectoryConverterTest, ConvertsAndReusesTimeInterpolation) {
    const asmc_msgs::Trajectory message = MakeTrajectory();
    PlannedPath path;
    EXPECT_EQ(TrajectoryStatus::OK, ConvertTrajectory(message, path));

    const ReferencePath reference = ToReferencePath(path, 0.05, 3);
    ASSERT_EQ(3u, reference.size());
    EXPECT_DOUBLE_EQ(1.5, reference.x_ref[1]);
    EXPECT_DOUBLE_EQ(3.0, reference.y_ref[1]);
    EXPECT_DOUBLE_EQ(4.0, reference.v_ref[1]);
}

TEST(TrajectoryConverterTest, RecognizesValidSinglePointStop) {
    asmc_msgs::Trajectory message = MakeTrajectory();
    message.points.resize(1);
    message.points.front().velocity = 0.0;
    PlannedPath path;
    EXPECT_EQ(TrajectoryStatus::STOP_TRAJECTORY,
              ConvertTrajectory(message, path));

    const morai_msgs::CtrlCmd stop = MakeRosStopCommand();
    EXPECT_EQ(1, stop.longlCmdType);
    EXPECT_DOUBLE_EQ(0.0, stop.accel);
    EXPECT_NEAR(0.3, stop.brake, 1e-6);
    EXPECT_DOUBLE_EQ(0.0, stop.steering);
}

TEST(MpcRosConverterTest, EmergencyStopUsesFullBrake) {
    const morai_msgs::CtrlCmd stop = MakeRosEmergencyStopCommand();
    EXPECT_EQ(1, stop.longlCmdType);
    EXPECT_DOUBLE_EQ(0.0, stop.accel);
    EXPECT_DOUBLE_EQ(1.0, stop.brake);
    EXPECT_DOUBLE_EQ(0.0, stop.steering);
}

TEST(TrajectoryConverterTest, PublishesRosSteeringInRadians) {
    MPCParams params;
    params.steering_max = 0.6981;
    params.accel_max = 3.0;
    params.accel_min = -5.0;
    params.brake_full_decel = 8.9;
    MPCControl control;
    control.delta = params.steering_max * 0.5;
    control.accel = 1.5;

    const morai_msgs::CtrlCmd message =
        ToRosControlCommand(control, params);

    EXPECT_DOUBLE_EQ(params.steering_max * 0.5, message.steering);
    EXPECT_DOUBLE_EQ(0.5, message.accel);
    EXPECT_DOUBLE_EQ(0.0, message.brake);

    control.accel = -1.78;
    const morai_msgs::CtrlCmd braking_message =
        ToRosControlCommand(control, params);
    EXPECT_DOUBLE_EQ(0.0, braking_message.accel);
    EXPECT_NEAR(0.2, braking_message.brake, 1e-6);
}

TEST(TrajectoryConverterTest, RejectsInvalidInputs) {
    PlannedPath path;
    asmc_msgs::Trajectory message = MakeTrajectory();
    message.dt = 0.0;
    EXPECT_EQ(TrajectoryStatus::INVALID_DT, ConvertTrajectory(message, path));

    message = MakeTrajectory();
    message.points.front().x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(TrajectoryStatus::INVALID_POINT,
              ConvertTrajectory(message, path));

    message = MakeTrajectory();
    message.valid = false;
    EXPECT_EQ(TrajectoryStatus::INVALID_MESSAGE,
              ConvertTrajectory(message, path));
}

// Test 11.4 (Commit E): ConvertTrajectory는 ego_at_plan의 provenance 계약을
// 여전히 검증한다 - Trajectory -> PlannedPath 변환으로 책임이 좁혀졌다고
// 해서 이 검증까지 사라지면 안 된다. STOP 계열 fail-safe가 ego_at_plan을
// 통해 독립적으로 작동하는 계약(interfaces/README.md 9절)이 이 검증에
// 달려있다.
TEST(TrajectoryConverterTest, StillValidatesEgoAtPlanProvenance) {
    PlannedPath path;
    asmc_msgs::Trajectory message = MakeTrajectory();
    message.ego_at_plan.valid = false;
    EXPECT_EQ(TrajectoryStatus::INVALID_EGO, ConvertTrajectory(message, path));

    message = MakeTrajectory();
    message.ego_at_plan.header.frame_id = "odom";
    EXPECT_EQ(TrajectoryStatus::INVALID_EGO, ConvertTrajectory(message, path));

    message = MakeTrajectory();
    message.ego_at_plan.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(TrajectoryStatus::INVALID_EGO, ConvertTrajectory(message, path));
}

TEST(TrajectoryConverterTest, RejectsStaleTrajectory) {
    EXPECT_TRUE(IsMessageFresh(ros::SteadyTime(10, 0),
                               ros::SteadyTime(10, 200000000), 0.25));
    EXPECT_FALSE(IsMessageFresh(ros::SteadyTime(10, 0),
                                ros::SteadyTime(10, 300000000), 0.25));
}

TEST(TrajectoryConverterTest, HeaderClockJumpDoesNotAffectFreshness) {
    PlannedPath path;
    asmc_msgs::Trajectory message = MakeTrajectory();
    message.header.stamp = ros::Time(1.0);
    EXPECT_EQ(TrajectoryStatus::OK, ConvertTrajectory(message, path));
    EXPECT_TRUE(IsMessageFresh(ros::SteadyTime(50, 0),
                               ros::SteadyTime(50, 100000000), 0.25));
    message.header.stamp = ros::Time(1000.0);
    EXPECT_EQ(TrajectoryStatus::OK, ConvertTrajectory(message, path));
    EXPECT_TRUE(IsMessageFresh(ros::SteadyTime(50, 100000000),
                               ros::SteadyTime(50, 200000000), 0.25));
}

// ===== Commit B: MPC current-state / trajectory latency alignment =====

namespace {

PlannedPath MakeStraightPlannedPath() {
    // dt=0.1, x/y/yaw/v/kappa가 t에 선형으로 변하는 단순 경로. offset 보간
    // 결과를 손으로 검증하기 쉽게 만든다.
    PlannedPath path;
    path.dt = 0.1;
    for (int i = 0; i < 5; ++i) {
        const double t = i * path.dt;
        path.x.push_back(t * 10.0);   // 10 m/s
        path.y.push_back(0.0);
        path.yaw.push_back(0.0);
        path.v.push_back(5.0);
        path.kappa.push_back(0.0);
    }
    return path;
}

asmc_msgs::VehicleState MakeVehicleState() {
    asmc_msgs::VehicleState message;
    message.header.stamp = ros::Time(10.0);
    message.header.frame_id = "map";
    message.valid = true;
    message.x = 5.0;
    message.y = 1.0;
    message.yaw = 0.1;
    message.speed = 4.0;
    return message;
}

}  // namespace

// Test B-1: offset=0이면 기존 resampler 결과와 완전히 동일.
TEST(ReferencePathOffsetTest, ZeroOffsetMatchesLegacyBehavior) {
    const PlannedPath path = MakeStraightPlannedPath();
    const ReferencePath legacy = ToReferencePath(path, 0.05, 3);
    const ReferencePath explicit_zero = ToReferencePath(path, 0.05, 3, 0.0);
    ASSERT_EQ(legacy.size(), explicit_zero.size());
    for (std::size_t i = 0; i < legacy.size(); ++i) {
        EXPECT_DOUBLE_EQ(legacy.x_ref[i], explicit_zero.x_ref[i]);
        EXPECT_DOUBLE_EQ(legacy.v_ref[i], explicit_zero.v_ref[i]);
    }
}

// Test B-2: source dt=0.1, offset=0.1이면 새 reference의 첫 점이 기존
// trajectory의 t=0.1 위치(x=1.0)와 같다.
TEST(ReferencePathOffsetTest, OffsetShiftsStartToExactSourceSample) {
    const PlannedPath path = MakeStraightPlannedPath();
    const ReferencePath reference = ToReferencePath(path, 0.1, 3, 0.1);
    ASSERT_EQ(3u, reference.size());
    EXPECT_DOUBLE_EQ(1.0, reference.x_ref[0]);   // t=0.1 -> path.x[1]
    EXPECT_DOUBLE_EQ(2.0, reference.x_ref[1]);   // t=0.2 -> path.x[2]
}

// Test B-3: fractional offset(0.05)도 올바르게 선형보간한다.
TEST(ReferencePathOffsetTest, FractionalOffsetInterpolates) {
    const PlannedPath path = MakeStraightPlannedPath();
    const ReferencePath reference = ToReferencePath(path, 0.1, 2, 0.05);
    ASSERT_EQ(2u, reference.size());
    // t=0.05 -> path.x[0]=0.0과 path.x[1]=1.0의 중간
    EXPECT_DOUBLE_EQ(0.5, reference.x_ref[0]);
}

// Test B-4: offset이 마지막 trajectory point 이후로 넘어가도 안전하게
// clamp되어 out-of-range access가 발생하지 않는다.
TEST(ReferencePathOffsetTest, OffsetBeyondPathClampsToLastPoint) {
    const PlannedPath path = MakeStraightPlannedPath();  // 마지막 t=0.4, x=4.0
    const ReferencePath reference = ToReferencePath(path, 0.1, 2, 10.0);
    ASSERT_EQ(2u, reference.size());
    EXPECT_DOUBLE_EQ(4.0, reference.x_ref[0]);
    EXPECT_DOUBLE_EQ(4.0, reference.x_ref[1]);
}

// Test B-5: MPC control initial state가 더 이상 ego_at_plan이 아니라 최신
// VehicleState에서 만들어진다 - 순수 helper(ConvertVehicleState)로 분리해
// ROS node 없이 검증한다.
TEST(VehicleStateConverterTest, ProducesMpcStateFromLatestVehicleState) {
    const asmc_msgs::VehicleState message = MakeVehicleState();
    MPCState state;
    EXPECT_EQ(VehicleStateStatus::OK, ConvertVehicleState(message, state));
    EXPECT_DOUBLE_EQ(5.0, state.x);
    EXPECT_DOUBLE_EQ(1.0, state.y);
    EXPECT_DOUBLE_EQ(0.1, state.yaw);
    EXPECT_DOUBLE_EQ(4.0, state.vx);
}

// Test B-6: VehicleState missing/invalid/stale -> fail-safe STOP 경로.
// message_converter_test의 기존 RejectsInvalidInputs/RejectsStaleTrajectory와
// 같은 스타일로 검증한다.
TEST(VehicleStateConverterTest, RejectsInvalidOrMissingFields) {
    MPCState state;

    asmc_msgs::VehicleState message = MakeVehicleState();
    message.valid = false;
    EXPECT_EQ(VehicleStateStatus::INVALID_MESSAGE,
              ConvertVehicleState(message, state));

    message = MakeVehicleState();
    message.header.frame_id = "odom";
    EXPECT_EQ(VehicleStateStatus::INVALID_FRAME,
              ConvertVehicleState(message, state));

    message = MakeVehicleState();
    message.header.stamp = ros::Time(0.0);
    EXPECT_EQ(VehicleStateStatus::INVALID_TIMESTAMP,
              ConvertVehicleState(message, state));

    message = MakeVehicleState();
    message.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(VehicleStateStatus::NON_FINITE_VALUE,
              ConvertVehicleState(message, state));
}

TEST(VehicleStateConverterTest, StaleVehicleStateIsNotFresh) {
    // MPC node의 fail-safe STOP은 IsMessageFresh를 vehicle-state timeout에도
    // 그대로 재사용한다(정책 중복 없음, Commit E에서 IsTrajectoryFresh를
    // 일반화된 이름으로 rename) - 같은 SteadyTime watchdog 계약이 vehicle
    // state에도 적용됨을 확인한다.
    EXPECT_TRUE(IsMessageFresh(ros::SteadyTime(10, 0),
                               ros::SteadyTime(10, 150000000), 0.20));
    EXPECT_FALSE(IsMessageFresh(ros::SteadyTime(10, 0),
                                ros::SteadyTime(10, 250000000), 0.20));
}

}  // namespace
}  // namespace mpc_controller_ros

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
