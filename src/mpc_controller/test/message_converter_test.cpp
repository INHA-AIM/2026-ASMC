#include <gtest/gtest.h>

#include <limits>

#include "ros/message_converter.hpp"
#include "runtime/control_adapter.hpp"
#include "runtime/path_resampler.hpp"

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
    MPCState state;
    EXPECT_EQ(TrajectoryStatus::OK,
              ConvertTrajectory(message, path, state));
    EXPECT_DOUBLE_EQ(1.0, state.x);
    EXPECT_DOUBLE_EQ(3.0, state.vx);

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
    MPCState state;
    EXPECT_EQ(TrajectoryStatus::STOP_TRAJECTORY,
              ConvertTrajectory(message, path, state));

    const morai_msgs::CtrlCmd stop =
        ToRosControlCommand(MakeStopCommand());
    EXPECT_EQ(1, stop.longlCmdType);
    EXPECT_DOUBLE_EQ(0.0, stop.accel);
    EXPECT_NEAR(0.3, stop.brake, 1e-6);
    EXPECT_DOUBLE_EQ(0.0, stop.steering);
}

TEST(TrajectoryConverterTest, RejectsInvalidInputs) {
    PlannedPath path;
    MPCState state;
    asmc_msgs::Trajectory message = MakeTrajectory();
    message.dt = 0.0;
    EXPECT_EQ(TrajectoryStatus::INVALID_DT,
              ConvertTrajectory(message, path, state));

    message = MakeTrajectory();
    message.points.front().x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(TrajectoryStatus::INVALID_POINT,
              ConvertTrajectory(message, path, state));

    message = MakeTrajectory();
    message.valid = false;
    EXPECT_EQ(TrajectoryStatus::INVALID_MESSAGE,
              ConvertTrajectory(message, path, state));
}

TEST(TrajectoryConverterTest, RejectsStaleTrajectory) {
    const asmc_msgs::Trajectory message = MakeTrajectory();
    EXPECT_TRUE(IsTrajectoryFresh(message, ros::Time(10.2), 0.25));
    EXPECT_FALSE(IsTrajectoryFresh(message, ros::Time(10.3), 0.25));
}

}  // namespace
}  // namespace mpc_controller_ros

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
