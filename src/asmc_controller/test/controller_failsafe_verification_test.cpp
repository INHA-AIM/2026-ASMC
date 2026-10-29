#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/master.h>

#include "Control.hpp"
#include "MPC/MPC.hpp"
#include "MPC/Constraints/constraints.hpp"
#include "Util/MessageConverter.hpp"

namespace {

asmc_msgs::Trajectory MakeValidTrajectory() {
    asmc_msgs::Trajectory message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "map";
    message.valid = true;
    message.dt = 0.1;
    message.ego_at_plan.header.stamp = message.header.stamp;
    message.ego_at_plan.header.frame_id = "map";
    message.ego_at_plan.valid = true;
    message.ego_at_plan.speed = 3.0;

    asmc_msgs::TrajectoryPoint first;
    first.velocity = 3.0;
    asmc_msgs::TrajectoryPoint second = first;
    second.x = 0.3;
    message.points = {first, second};
    return message;
}

class ControllerOutputCapture {
 public:
    bool Initialize(bool receive_vehicle_state = true) {
        if (!ros::master::check()) return false;
        g_control = ControlContext{};
        InitMPC(MPCParams{});

        const std::string topic = std::string("/ctrl005/") +
            ::testing::UnitTest::GetInstance()->current_test_info()->name();
        subscriber_ = nh_.subscribe(
            topic, 1,
            &ControllerOutputCapture::CaptureCommand, this);
        g_control.control_pub = nh_.advertise<morai_msgs::CtrlCmd>(
            topic, 1);
        if (!WaitForSubscriber()) return false;

        if (receive_vehicle_state) {
            asmc_msgs::VehicleState vehicle;
            vehicle.header.stamp = ros::Time::now();
            vehicle.header.frame_id = "map";
            vehicle.valid = true;
            vehicle.speed = 3.0;
            mpc_controller_ros::VehicleStateCallback(
                asmc_msgs::VehicleState::ConstPtr(
                    new asmc_msgs::VehicleState(vehicle)));
            if (!g_control.has_vehicle_state) return false;
        }

        // A prior successful control must not survive a fail-safe cycle.
        g_control.has_successful_control = true;
        g_control.last_successful_steering = 0.2;
        g_mpc.warm_start.push_back(MPCControl{0.2, 1.0});
        g_mpc.last_control = MPCControl{0.2, 1.0};
        g_mpc.stuck_cycles = 3;
        g_mpc.recovery_cycles_left = 2;
        return true;
    }

    ~ControllerOutputCapture() {
        g_control.control_pub.shutdown();
        subscriber_.shutdown();
        g_control = ControlContext{};
    }

    bool WaitForSubscriber() {
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(5.0);
        while (ros::WallTime::now() < deadline) {
            ros::spinOnce();
            if (g_control.control_pub.getNumSubscribers() > 0) return true;
            ros::WallDuration(0.01).sleep();
        }
        return false;
    }

    bool WaitForCommand() {
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(5.0);
        while (ros::WallTime::now() < deadline) {
            ros::spinOnce();
            if (!commands_.empty()) return true;
            ros::WallDuration(0.01).sleep();
        }
        return false;
    }

    void ExpectStopAndReset() {
        ASSERT_EQ(1u, commands_.size());
        const morai_msgs::CtrlCmd expected =
            mpc_controller_ros::MakeRosStopCommand();
        const morai_msgs::CtrlCmd& actual = commands_.front();
        EXPECT_EQ(expected.longlCmdType, actual.longlCmdType);
        EXPECT_DOUBLE_EQ(expected.accel, actual.accel);
        EXPECT_DOUBLE_EQ(expected.brake, actual.brake);
        EXPECT_DOUBLE_EQ(expected.steering, actual.steering);

        EXPECT_FALSE(g_control.has_successful_control);
        EXPECT_TRUE(g_mpc.warm_start.empty());
        EXPECT_DOUBLE_EQ(0.0, g_mpc.last_control.delta);
        EXPECT_DOUBLE_EQ(0.0, g_mpc.last_control.accel);
        EXPECT_EQ(0, g_mpc.stuck_cycles);
        EXPECT_EQ(0, g_mpc.recovery_cycles_left);
    }

    const std::vector<morai_msgs::CtrlCmd>& Commands() const { return commands_; }
    void ClearCommands() { commands_.clear(); }

 private:
    void CaptureCommand(const morai_msgs::CtrlCmd::ConstPtr& message) {
        commands_.push_back(*message);
    }

    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::vector<morai_msgs::CtrlCmd> commands_;
};

TEST(ControllerFailSafeVerification, CTRL005MissingTrajectory) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize());
    ASSERT_FALSE(g_control.has_trajectory);

    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});

    ASSERT_TRUE(capture.WaitForCommand());
    capture.ExpectStopAndReset();
}

TEST(ControllerFailSafeVerification, CTRL005StaleTrajectory) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize());
    const asmc_msgs::Trajectory trajectory = MakeValidTrajectory();
    PlannedPath path;
    ASSERT_EQ(mpc_controller_ros::TrajectoryStatus::OK,
              mpc_controller_ros::ConvertTrajectory(trajectory, path));
    mpc_controller_ros::TrajectoryCallback(
        asmc_msgs::Trajectory::ConstPtr(new asmc_msgs::Trajectory(trajectory)));
    ASSERT_TRUE(g_control.has_trajectory);

    g_control.trajectory_received_at = ros::SteadyTime::now() -
        ros::WallDuration(g_control.trajectory_timeout + 0.1);
    ASSERT_FALSE(mpc_controller_ros::IsMessageFresh(
        g_control.trajectory_received_at, ros::SteadyTime::now(),
        g_control.trajectory_timeout));

    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});

    ASSERT_TRUE(capture.WaitForCommand());
    capture.ExpectStopAndReset();
}

TEST(ControllerFailSafeVerification, CollisionBlockedPlannerStopUsesFullBrake) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize());
    auto trajectory = MakeValidTrajectory();
    trajectory.failure_reason = asmc_msgs::Trajectory::FAILURE_COLLISION_BLOCKED;
    trajectory.points.resize(1);
    trajectory.points.front().velocity = 0.0;
    mpc_controller_ros::TrajectoryCallback(
        asmc_msgs::Trajectory::ConstPtr(new asmc_msgs::Trajectory(trajectory)));

    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});

    ASSERT_TRUE(capture.WaitForCommand());
    EXPECT_DOUBLE_EQ(1.0, capture.Commands().back().brake);
    EXPECT_TRUE(g_control.emergency_stop_active);
}

bool ReceiveValidFreshTrajectory() {
    const asmc_msgs::Trajectory trajectory = MakeValidTrajectory();
    PlannedPath path;
    if (mpc_controller_ros::ConvertTrajectory(trajectory, path) !=
        mpc_controller_ros::TrajectoryStatus::OK) return false;
    mpc_controller_ros::TrajectoryCallback(
        asmc_msgs::Trajectory::ConstPtr(new asmc_msgs::Trajectory(trajectory)));
    return g_control.has_trajectory && mpc_controller_ros::IsMessageFresh(
        g_control.trajectory_received_at, ros::SteadyTime::now(),
        g_control.trajectory_timeout);
}

TEST(ControllerFailSafeVerification, CTRL006MissingVehicleState) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize(false));
    ASSERT_TRUE(ReceiveValidFreshTrajectory());
    ASSERT_FALSE(g_control.has_vehicle_state);

    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});

    ASSERT_TRUE(capture.WaitForCommand());
    capture.ExpectStopAndReset();
}

TEST(ControllerFailSafeVerification, CTRL006StaleVehicleState) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize());
    ASSERT_TRUE(g_control.has_vehicle_state);
    MPCState ego;
    ASSERT_EQ(mpc_controller_ros::VehicleStateStatus::OK,
              mpc_controller_ros::ConvertVehicleState(
                  g_control.latest_vehicle_state, ego));
    g_control.vehicle_state_received_at = ros::SteadyTime::now() -
        ros::WallDuration(g_control.vehicle_state_timeout + 0.1);
    ASSERT_FALSE(mpc_controller_ros::IsMessageFresh(
        g_control.vehicle_state_received_at, ros::SteadyTime::now(),
        g_control.vehicle_state_timeout));
    ASSERT_TRUE(ReceiveValidFreshTrajectory());

    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});

    ASSERT_TRUE(capture.WaitForCommand());
    capture.ExpectStopAndReset();
}


void ExpectRuntimeReset() {
    EXPECT_TRUE(g_mpc.warm_start.empty());
    EXPECT_DOUBLE_EQ(0.0, g_mpc.last_control.accel);
    EXPECT_EQ(0, g_mpc.stuck_cycles);
    EXPECT_EQ(0, g_mpc.recovery_cycles_left);
}

bool ReceiveNormalRuntimeInputs() {
    auto trajectory = MakeValidTrajectory();
    trajectory.dt = GetMPCParams().dt;
    trajectory.points.clear();
    double x = 0.0, y = 0.0, yaw = 0.0;
    constexpr double curvature = 0.05;
    for (int i = 0; i <= GetMPCParams().horizon; ++i) {
        asmc_msgs::TrajectoryPoint point;
        point.x = x;
        point.y = y;
        point.yaw = yaw;
        point.velocity = 3.0;
        point.curvature = curvature;
        trajectory.points.push_back(point);
        const double ds = 3.0 * trajectory.dt;
        x += ds * std::cos(yaw);
        y += ds * std::sin(yaw);
        yaw += ds * curvature;
    }
    PlannedPath path;
    if (mpc_controller_ros::ConvertTrajectory(trajectory, path) !=
        mpc_controller_ros::TrajectoryStatus::OK) return false;
    auto vehicle = trajectory.ego_at_plan;
    MPCState ego;
    if (mpc_controller_ros::ConvertVehicleState(vehicle, ego) !=
        mpc_controller_ros::VehicleStateStatus::OK) return false;
    mpc_controller_ros::VehicleStateCallback(
        asmc_msgs::VehicleState::ConstPtr(new asmc_msgs::VehicleState(vehicle)));
    mpc_controller_ros::TrajectoryCallback(
        asmc_msgs::Trajectory::ConstPtr(new asmc_msgs::Trajectory(trajectory)));
    return g_control.has_trajectory && g_control.has_vehicle_state &&
        mpc_controller_ros::IsMessageFresh(g_control.trajectory_received_at,
            ros::SteadyTime::now(), g_control.trajectory_timeout) &&
        mpc_controller_ros::IsMessageFresh(g_control.vehicle_state_received_at,
            ros::SteadyTime::now(), g_control.vehicle_state_timeout);
}

TEST(ControllerFailSafeVerification, CTRL007EmergencyStop) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize());
    ResetMPCOnFailSafe();
    ASSERT_TRUE(ReceiveNormalRuntimeInputs());
    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});
    ASSERT_TRUE(capture.WaitForCommand());
    ASSERT_TRUE(g_control.has_successful_control);
    capture.ClearCommands();

    ASSERT_TRUE(ReceiveNormalRuntimeInputs());
    asmc_msgs::BehaviorCommand behavior;
    behavior.maneuver_type = asmc_msgs::BehaviorCommand::EMERGENCY_STOP;
    mpc_controller_ros::BehaviorCommandCallback(
        asmc_msgs::BehaviorCommand::ConstPtr(new asmc_msgs::BehaviorCommand(behavior)));
    ASSERT_TRUE(g_control.emergency_stop_active);
    // No control timer runs in this fixture. Receiving EMERGENCY_STOP must
    // itself publish full brake and reset the previous MPC/recovery state.
    ASSERT_TRUE(capture.WaitForCommand());
    ASSERT_EQ(1u, capture.Commands().size());
    EXPECT_DOUBLE_EQ(1.0, capture.Commands().front().brake);
    EXPECT_DOUBLE_EQ(0.0, capture.Commands().front().accel);
    EXPECT_FALSE(g_control.has_successful_control);
    ExpectRuntimeReset();
    capture.ClearCommands();

    // Subsequent timer ticks must keep the same emergency priority/output.
    g_mpc.stuck_cycles = 3;
    g_mpc.recovery_cycles_left = 2;
    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});
    ASSERT_TRUE(capture.WaitForCommand());
    ASSERT_EQ(1u, capture.Commands().size());
    const auto expected = mpc_controller_ros::MakeRosEmergencyStopCommand();
    const auto actual = capture.Commands().front();
    EXPECT_EQ(expected.longlCmdType, actual.longlCmdType);
    EXPECT_DOUBLE_EQ(expected.accel, actual.accel);
    EXPECT_DOUBLE_EQ(expected.brake, actual.brake);
    EXPECT_TRUE(std::isfinite(actual.steering));
    EXPECT_TRUE(std::isfinite(actual.accel));
    EXPECT_TRUE(std::isfinite(actual.brake));
    EXPECT_TRUE(std::isfinite(actual.steering));
    EXPECT_FALSE(g_control.has_successful_control);
    ExpectRuntimeReset();
    EXPECT_DOUBLE_EQ(actual.steering, g_mpc.last_control.delta);
    RecordProperty("emergency_brake", std::to_string(actual.brake));

    // Emergency also precedes the missing-input watchdog branches.
    capture.ClearCommands();
    g_control.has_trajectory = false;
    g_control.has_vehicle_state = false;
    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});
    ASSERT_TRUE(capture.WaitForCommand());
    ASSERT_EQ(1u, capture.Commands().size());
    EXPECT_DOUBLE_EQ(expected.brake, capture.Commands().front().brake);
    behavior.maneuver_type = asmc_msgs::BehaviorCommand::KEEP;
    mpc_controller_ros::BehaviorCommandCallback(
        asmc_msgs::BehaviorCommand::ConstPtr(new asmc_msgs::BehaviorCommand(behavior)));
    EXPECT_FALSE(g_control.emergency_stop_active);
}

TEST(ControllerFailSafeVerification, SignalEmergencyFirstCallbackPreservesSteering) {
    ControllerOutputCapture capture;ASSERT_TRUE(capture.Initialize());
    g_control.last_successful_control_at=ros::SteadyTime::now();
    g_control.last_successful_steering=.25;
    asmc_msgs::BehaviorCommand command;command.command_id=10;
    command.maneuver_type=asmc_msgs::BehaviorCommand::EMERGENCY_STOP;
    mpc_controller_ros::BehaviorCommandCallback(
        asmc_msgs::BehaviorCommand::ConstPtr(new asmc_msgs::BehaviorCommand(command)));
    ASSERT_TRUE(capture.WaitForCommand());ASSERT_EQ(capture.Commands().size(),1u);
    EXPECT_DOUBLE_EQ(capture.Commands().front().brake,1);
    EXPECT_DOUBLE_EQ(capture.Commands().front().accel,0);
    EXPECT_DOUBLE_EQ(capture.Commands().front().steering,.25);
    EXPECT_FALSE(g_mpc.steering_history.empty());ExpectRuntimeReset();
}

TEST(ControllerFailSafeVerification, UnimplementedBindingCannotReleaseEvenWithNewStatePath) {
    ControllerOutputCapture capture;ASSERT_TRUE(capture.Initialize());
    g_control.require_verified_release=true;
    asmc_msgs::BehaviorCommand command;command.command_id=20;
    command.maneuver_type=asmc_msgs::BehaviorCommand::EMERGENCY_STOP;
    mpc_controller_ros::BehaviorCommandCallback(
        asmc_msgs::BehaviorCommand::ConstPtr(new asmc_msgs::BehaviorCommand(command)));
    ASSERT_TRUE(capture.WaitForCommand());capture.ClearCommands();
    command.command_id=21;command.maneuver_type=asmc_msgs::BehaviorCommand::KEEP;
    mpc_controller_ros::BehaviorCommandCallback(
        asmc_msgs::BehaviorCommand::ConstPtr(new asmc_msgs::BehaviorCommand(command)));
    ASSERT_TRUE(ReceiveNormalRuntimeInputs());  // New timestamp supplies no ownership proof.
    EXPECT_TRUE(g_control.emergency_stop_active);EXPECT_EQ(g_control.pending_release_command,21u);
    mpc_controller_ros::BehaviorCommandCallback(
        asmc_msgs::BehaviorCommand::ConstPtr(new asmc_msgs::BehaviorCommand(command)));
    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});
    ASSERT_TRUE(capture.WaitForCommand());EXPECT_DOUBLE_EQ(capture.Commands().back().brake,1);
    EXPECT_TRUE(g_control.emergency_stop_active);
}


void RunPlannerStopCycle(ControllerOutputCapture& capture) {
    ResetMPCOnFailSafe();
    ASSERT_TRUE(ReceiveNormalRuntimeInputs());
    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});
    ASSERT_TRUE(capture.WaitForCommand());
    ASSERT_EQ(1u, capture.Commands().size());
    ASSERT_TRUE(g_control.has_successful_control);
    ASSERT_GT(g_control.last_successful_steering, 0.0);
    ASSERT_FALSE(g_mpc.warm_start.empty());
    const double previous = g_control.last_successful_steering;
    capture.ClearCommands();

    // Deterministic recent success; no production timeout/rate is changed.
    g_control.last_successful_control_at = ros::SteadyTime::now() -
        ros::WallDuration(0.02);
    ASSERT_TRUE(ReceiveNormalRuntimeInputs());
    auto stop = MakeValidTrajectory();
    stop.points.resize(1);
    stop.points.front().velocity = 0.0;
    PlannedPath path;
    ASSERT_EQ(mpc_controller_ros::TrajectoryStatus::STOP_TRAJECTORY,
              mpc_controller_ros::ConvertTrajectory(stop, path));
    mpc_controller_ros::TrajectoryCallback(
        asmc_msgs::Trajectory::ConstPtr(new asmc_msgs::Trajectory(stop)));
    ASSERT_TRUE(mpc_controller_ros::IsMessageFresh(
        g_control.trajectory_received_at, ros::SteadyTime::now(),
        g_control.trajectory_timeout));
    ASSERT_TRUE(mpc_controller_ros::IsMessageFresh(
        g_control.vehicle_state_received_at, ros::SteadyTime::now(),
        g_control.vehicle_state_timeout));
    MPCState ego;
    ASSERT_EQ(mpc_controller_ros::VehicleStateStatus::OK,
              mpc_controller_ros::ConvertVehicleState(g_control.latest_vehicle_state, ego));
    g_mpc.stuck_cycles = 3;
    g_mpc.recovery_cycles_left = 2;
    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});
    ASSERT_TRUE(capture.WaitForCommand());
    ASSERT_EQ(1u, capture.Commands().size());
    const auto actual = capture.Commands().front();
    const auto expected = mpc_controller_ros::MakeRosStopCommand();
    EXPECT_EQ(expected.longlCmdType, actual.longlCmdType);
    EXPECT_DOUBLE_EQ(expected.accel, actual.accel);
    EXPECT_DOUBLE_EQ(expected.brake, actual.brake);
    EXPECT_TRUE(std::isfinite(actual.steering));
    const double rate = SteeringRateLimit(ego.vx, GetMPCParams());
    // A valid STOP tracks cached geometry at the same fixed MPC time step.
    // Braking must not erase the path or let launch recovery override the pedal.
    EXPECT_LE(std::abs(actual.steering-previous), rate*GetMPCParams().dt+1e-8);
    EXPECT_FALSE(g_mpc.braking_geometry.empty());
    EXPECT_LE(std::abs(actual.steering), GetMPCParams().steering_max);
    ExpectRuntimeReset();
    EXPECT_DOUBLE_EQ(actual.steering, g_mpc.last_control.delta);
    ::testing::Test::RecordProperty("pre_stop_steering", std::to_string(previous));
    ::testing::Test::RecordProperty("stop_steering", std::to_string(actual.steering));
    ::testing::Test::RecordProperty("stop_brake", std::to_string(actual.brake));
}

TEST(ControllerFailSafeVerification, CTRL008PlannerStop) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize());
    RunPlannerStopCycle(capture);
}

TEST(ControllerFailSafeVerification, HighSpeedPlannerStopUsesScheduledRateAndKeepsDelay) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize());
    MPCParams params;
    params.dt = 0.05;
    params.steering_response_delay_s = 0.15;
    params.steering_rate_reference_speed_mps = 5.0;
    params.steering_rate_min = 0.1;
    InitMPC(params);
    g_mpc.steering_history = {0.04, 0.04, 0.04, 0.04};
    g_control.last_successful_steering = 0.04;
    g_control.last_successful_control_at = ros::SteadyTime::now() - ros::WallDuration(0.05);
    const auto previous_at = g_control.last_successful_control_at;
    g_control.latest_vehicle_state.speed = 58.0/3.6;
    g_control.vehicle_state_received_at = ros::SteadyTime::now();
    auto stop = MakeValidTrajectory();
    stop.points.resize(1);
    stop.points.front().velocity = 0.0;
    mpc_controller_ros::TrajectoryCallback(
        asmc_msgs::Trajectory::ConstPtr(new asmc_msgs::Trajectory(stop)));
    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});
    ASSERT_TRUE(capture.WaitForCommand());
    ASSERT_EQ(capture.Commands().size(), 1u);
    const auto actual = capture.Commands().front();
    const double elapsed = (g_control.last_successful_control_at-previous_at).toSec();
    EXPECT_NEAR(actual.steering, std::max(0.0, 0.04-0.1*elapsed), 1e-8);
    EXPECT_DOUBLE_EQ(actual.accel, 0.0);
    EXPECT_DOUBLE_EQ(actual.brake, mpc_controller_ros::MakeRosStopCommand().brake);
    EXPECT_DOUBLE_EQ(g_control.last_successful_steering, actual.steering);
    ASSERT_FALSE(g_mpc.steering_history.empty());
    EXPECT_DOUBLE_EQ(g_mpc.steering_history.back(), actual.steering);
    EXPECT_DOUBLE_EQ(g_mpc.steering_history.front(), 0.04);
}


TEST(ControllerFailSafeVerification, CTRL009StopRestart) {
    ControllerOutputCapture capture;
    ASSERT_TRUE(capture.Initialize());
    RunPlannerStopCycle(capture);
    ASSERT_FALSE(HasFailure());
    const double stop_steering = capture.Commands().front().steering;
    ASSERT_TRUE(g_mpc.warm_start.empty());
    ASSERT_EQ(0, g_mpc.stuck_cycles);
    ASSERT_EQ(0, g_mpc.recovery_cycles_left);
    capture.ClearCommands();

    ASSERT_TRUE(ReceiveNormalRuntimeInputs());
    ASSERT_FALSE(g_control.emergency_stop_active);
    mpc_controller_ros::ControlProcess(ros::SteadyTimerEvent{});
    ASSERT_TRUE(capture.WaitForCommand());
    ASSERT_EQ(1u, capture.Commands().size());
    ASSERT_TRUE(g_control.has_successful_control);
    ASSERT_FALSE(g_mpc.warm_start.empty());
    const auto actual = capture.Commands().front();
    const auto control = g_mpc.last_control;
    const auto& params = GetMPCParams();
    const auto expected = mpc_controller_ros::ToRosControlCommand(control, params);
    EXPECT_EQ(expected.longlCmdType, actual.longlCmdType);
    EXPECT_DOUBLE_EQ(expected.accel, actual.accel);
    EXPECT_DOUBLE_EQ(expected.brake, actual.brake);
    EXPECT_DOUBLE_EQ(expected.steering, actual.steering);
    EXPECT_TRUE(std::isfinite(actual.steering));
    EXPECT_TRUE(std::isfinite(actual.accel));
    EXPECT_TRUE(std::isfinite(actual.brake));
    EXPECT_TRUE(std::isfinite(control.delta));
    EXPECT_TRUE(std::isfinite(control.accel));
    EXPECT_GE(control.delta, -params.steering_max);
    EXPECT_LE(control.delta, params.steering_max);
    EXPECT_GE(control.accel, params.accel_min);
    EXPECT_LE(control.accel, params.accel_max);
    EXPECT_LE(std::abs(control.delta - stop_steering),
              params.steering_rate_max * params.dt + 1e-8);
    EXPECT_DOUBLE_EQ(control.delta, g_control.last_successful_steering);
    EXPECT_EQ(0, g_mpc.stuck_cycles);
    EXPECT_EQ(0, g_mpc.recovery_cycles_left);
    const auto stop = mpc_controller_ros::MakeRosStopCommand();
    EXPECT_FALSE(actual.accel == stop.accel && actual.brake == stop.brake &&
                 actual.steering == stop.steering);
    RecordProperty("restart_steering", std::to_string(control.delta));
    RecordProperty("restart_acceleration", std::to_string(control.accel));
    RecordProperty("restart_pedal_accel", std::to_string(actual.accel));
    RecordProperty("restart_brake", std::to_string(actual.brake));
}

TEST(ControllerFailSafeVerification, BoundReleaseRejectsFreshOldTrajectoryAndMirror) {
    ControllerOutputCapture capture;ASSERT_TRUE(capture.Initialize());
    ASSERT_TRUE(ReceiveNormalRuntimeInputs());
    const auto trajectory=g_control.latest_trajectory;
    g_control.binding_enabled=true;g_control.emergency_stop_active=true;
    asmc_msgs::binding::Request r;
    r.token={g_control.binding_gate.epoch(),std::string(32,'b'),2};
    r.command.command_id=2;r.command.maneuver_type=asmc_msgs::BehaviorCommand::KEEP;
    r.command.execution_request=asmc_msgs::BehaviorCommand::EXECUTE;
    r.command.header.stamp=ros::Time::now();r.command.header.frame_id="map";
    r.command.valid_until=ros::Time::now()+ros::Duration(.5);
    auto request=asmc_msgs::binding::Encode(r);
    mpc_controller_ros::ControlRequestCallback(boost::make_shared<std_msgs::UInt8MultiArray>(request));
    mpc_controller_ros::BehaviorCommandCallback(boost::make_shared<asmc_msgs::BehaviorCommand>(r.command));
    mpc_controller_ros::TrajectoryCallback(boost::make_shared<asmc_msgs::Trajectory>(trajectory));
    asmc_msgs::binding::Plan p;p.request=r;p.request.token.revision=1;p.request.command.command_id=1;
    p.planner_epoch=std::string(32,'p');p.sequence=1;p.trajectory=trajectory;
    p.feedback.command_accepted=true;p.feedback.command_id=1;p.feedback.plan_valid=true;
    p.feedback.active_maneuver=asmc_msgs::BehaviorCommand::KEEP;
    p.feedback.trajectory_source=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW;
    mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
    mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
    EXPECT_TRUE(g_control.emergency_stop_active);EXPECT_DOUBLE_EQ(capture.Commands().back().brake,1.0);
    capture.ClearCommands();
    p.request=r;p.feedback.command_id=2;p.sequence=2;
    mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
    ASSERT_TRUE(g_control.binding_gate.linked());
    mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
    EXPECT_FALSE(g_control.emergency_stop_active);
    EXPECT_TRUE(g_control.has_successful_control);
}

TEST(ControllerFailSafeVerification, BoundPreflightHoldsUntilArbiterAuthorizesRelease) {
    ControllerOutputCapture capture;ASSERT_TRUE(capture.Initialize());
    ASSERT_TRUE(ReceiveNormalRuntimeInputs());auto trajectory=g_control.latest_trajectory;
    g_control.binding_enabled=true;g_control.emergency_stop_active=true;
    asmc_msgs::binding::Request r;r.token={g_control.binding_gate.epoch(),std::string(32,'b'),1};
    r.command.command_id=7;r.command.maneuver_type=asmc_msgs::BehaviorCommand::KEEP;
    r.command.valid_until=ros::Time::now()+ros::Duration(.5);r.release_authorized=false;
    asmc_msgs::binding::Plan p;p.request=r;p.planner_epoch=std::string(32,'p');p.sequence=1;p.trajectory=trajectory;
    p.feedback.command_accepted=true;p.feedback.command_id=7;p.feedback.plan_valid=true;
    p.feedback.active_maneuver=asmc_msgs::BehaviorCommand::KEEP;
    p.feedback.trajectory_source=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW;
    // Explicitly exercise trajectory-before-command delivery.
    mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
    mpc_controller_ros::ControlRequestCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(r)));
    ASSERT_TRUE(g_control.binding_gate.linked());
    mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
    EXPECT_TRUE(g_control.emergency_stop_active);EXPECT_DOUBLE_EQ(capture.Commands().back().brake,1.0);
    capture.ClearCommands();
    r.release_authorized=true;++r.token.revision;
    mpc_controller_ros::ControlRequestCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(r)));
    mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
    EXPECT_TRUE(g_control.emergency_stop_active);capture.ClearCommands();
    p.request=r;++p.sequence;
    mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
    mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
    EXPECT_FALSE(g_control.emergency_stop_active);
}

TEST(ControllerFailSafeVerification, BoundAvoidWaitingTracksCheckedStopWithoutGrantingCommit) {
 ControllerOutputCapture capture;ASSERT_TRUE(capture.Initialize());
 auto trajectory=MakeValidTrajectory();trajectory.points.clear();
 trajectory.ego_at_plan.acceleration=-1.;
 for(int i=0;i<=40;++i){
  const double t=std::min(3.,i*trajectory.dt);
  asmc_msgs::TrajectoryPoint point;
  point.x=3*t-.5*t*t;point.velocity=std::max(0.,3-t);
  point.acceleration=t<3?-1.:0.;trajectory.points.push_back(point);
 }
 mpc_controller_ros::VehicleStateCallback(boost::make_shared<asmc_msgs::VehicleState>(trajectory.ego_at_plan));
 g_control.binding_enabled=true;g_control.emergency_stop_active=true;
 asmc_msgs::binding::Request r;
 r.token={g_control.binding_gate.epoch(),std::string(32,'b'),1};
 r.command.command_id=10;r.command.maneuver_type=asmc_msgs::BehaviorCommand::AVOID;
 r.command.execution_request=asmc_msgs::BehaviorCommand::EXECUTE;
 r.command.valid_until=ros::Time::now()+ros::Duration(.5);r.release_authorized=false;
 asmc_msgs::binding::Plan p;p.request=r;p.planner_epoch=std::string(32,'p');p.sequence=1;
 p.trajectory=trajectory;p.feedback.command_id=10;p.feedback.command_accepted=true;
 p.feedback.active_maneuver=asmc_msgs::BehaviorCommand::AVOID;
 p.feedback.execution_state=asmc_msgs::FrenetFeedback::WAITING;p.feedback.plan_valid=false;
 p.feedback.trajectory_source=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW;
 mpc_controller_ros::ControlRequestCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(r)));
 mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
 ASSERT_TRUE(g_control.binding_gate.linked());
 mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
 EXPECT_DOUBLE_EQ(capture.Commands().back().brake,1.);
 capture.ClearCommands();r.release_authorized=true;++r.token.revision;p.request=r;++p.sequence;
 mpc_controller_ros::ControlRequestCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(r)));
 mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
 mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
 EXPECT_FALSE(g_control.emergency_stop_active);EXPECT_TRUE(g_control.has_successful_control);
 EXPECT_LT(capture.Commands().back().brake,1.);
 EXPECT_FALSE(g_control.binding_gate.plan().feedback.plan_valid);
 EXPECT_EQ(g_control.binding_gate.plan().feedback.execution_state,asmc_msgs::FrenetFeedback::WAITING);
}

TEST(ControllerFailSafeVerification, PendingRecoveryPreservesMaximumBrakeAndRequiresCurrentExitProof) {
    ControllerOutputCapture capture;ASSERT_TRUE(capture.Initialize());
    ASSERT_TRUE(ReceiveNormalRuntimeInputs());auto trajectory=g_control.latest_trajectory;
    g_control.binding_enabled=true;
    asmc_msgs::binding::Request r;r.token={g_control.binding_gate.epoch(),std::string(32,'b'),1};
    r.command.command_id=8;r.command.maneuver_type=asmc_msgs::BehaviorCommand::KEEP;
    r.command.valid_until=ros::Time::now()+ros::Duration(.5);
    r.recovery_release=true;r.release_authorized=false;r.brake_generation=1;r.brake_level=2;
    r.brake_decision_ros=ros::Time::now().toSec();r.brake_state_stamp=g_control.latest_vehicle_state.header.stamp.toSec();
    r.brake_speed=3;
    mpc_controller_ros::ControlRequestCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(r)));
    ASSERT_TRUE(capture.WaitForCommand());EXPECT_DOUBLE_EQ(capture.Commands().back().brake,1);
    capture.ClearCommands();
    r.release_authorized=true;r.brake_level=0;++r.token.revision;
    asmc_msgs::binding::Plan p;p.request=r;p.planner_epoch=std::string(32,'p');p.sequence=1;p.trajectory=trajectory;
    p.feedback.command_accepted=true;p.feedback.command_id=8;p.feedback.plan_valid=true;
    p.feedback.active_maneuver=asmc_msgs::BehaviorCommand::KEEP;p.feedback.trajectory_source=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW;
    mpc_controller_ros::ControlRequestCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(r)));
    mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
    mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
    EXPECT_TRUE(g_control.emergency_stop_active);EXPECT_DOUBLE_EQ(capture.Commands().back().brake,1);
    capture.ClearCommands();++p.sequence;p.exit_path_valid=true;
    mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
    mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
    EXPECT_FALSE(g_control.emergency_stop_active);  // Verified internal recovery need not first stop.
}
TEST(ControllerFailSafeVerification,BoundRecoveryBrakesAboveItsOwnCommandSpeedEvenWithAcceleratingReference) {
 ControllerOutputCapture capture;ASSERT_TRUE(capture.Initialize());ASSERT_TRUE(ReceiveNormalRuntimeInputs());
 g_control.binding_enabled=true;
 asmc_msgs::binding::Request r;r.token={g_control.binding_gate.epoch(),std::string(32,'b'),1};
 r.command.command_id=9;r.command.maneuver_type=asmc_msgs::BehaviorCommand::KEEP;
 r.command.desired_speed_mps=2;r.command.valid_until=ros::Time::now()+ros::Duration(.5);
 r.recovery_release=true;r.release_authorized=true;
 asmc_msgs::binding::Plan p;p.request=r;p.planner_epoch=std::string(32,'p');p.sequence=1;
 p.trajectory=g_control.latest_trajectory;p.exit_path_valid=true;
 p.feedback.command_accepted=true;p.feedback.command_id=9;p.feedback.plan_valid=true;
 p.feedback.active_maneuver=asmc_msgs::BehaviorCommand::KEEP;
 p.feedback.trajectory_source=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW;
 mpc_controller_ros::ControlRequestCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(r)));
 mpc_controller_ros::ControlPlanCallback(boost::make_shared<std_msgs::UInt8MultiArray>(asmc_msgs::binding::Encode(p)));
 capture.ClearCommands();mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
 EXPECT_GT(capture.Commands().back().brake,0);EXPECT_DOUBLE_EQ(capture.Commands().back().accel,0);
 EXPECT_FALSE(g_control.brake_watchdog.failed());
}

TEST(ControllerFailSafeVerification,PrivateValidationCapConstrainsNormalSolverWithoutEmergencyStop) {
 ControllerOutputCapture capture;ASSERT_TRUE(capture.Initialize());ASSERT_TRUE(ReceiveNormalRuntimeInputs());
 g_control.validation_speed_cap_mps=1.;
 capture.ClearCommands();mpc_controller_ros::ControlProcess({});ASSERT_TRUE(capture.WaitForCommand());
 EXPECT_DOUBLE_EQ(capture.Commands().back().accel,0.);
 EXPECT_GT(capture.Commands().back().brake,0.);
 EXPECT_FALSE(g_control.emergency_stop_active);
 EXPECT_TRUE(g_control.handoff_solved);
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_failsafe_verification_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
