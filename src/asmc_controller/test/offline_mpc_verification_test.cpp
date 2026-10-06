#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "MPC/MPC.hpp"

TEST(OfflineMPCVerification, CTRL001StraightTracking) {
    MPCParams params;
    params.horizon = 10;
    params.dt = 0.1;
    params.max_iterations = 25;

    ReferencePath reference;
    for (int i = 0; i < params.horizon; ++i) {
        reference.x_ref.push_back(3.0 * params.dt * i);
        reference.y_ref.push_back(0.0);
        reference.yaw_ref.push_back(0.0);
        reference.v_ref.push_back(3.0);
        reference.a_ref.push_back(0.0);
        reference.k_ref.push_back(0.0);
    }

    const MPCState ego{};  // x=y=yaw=vx=0
    InitMPC(params);
    const MPCOutput output = MPCProcess(ego, reference);

    ASSERT_TRUE(output.success);
    EXPECT_TRUE(std::isfinite(output.control.delta));
    EXPECT_TRUE(std::isfinite(output.control.accel));
    EXPECT_LT(std::abs(output.control.delta), 0.05);
    EXPECT_GT(output.control.accel, 0.0);
    EXPECT_GE(output.control.delta, -params.steering_max);
    EXPECT_LE(output.control.delta, params.steering_max);
    EXPECT_GE(output.control.accel, params.accel_min);
    EXPECT_LE(output.control.accel, params.accel_max);
}

TEST(OfflineMPCVerification, CTRL002CurveTracking) {
    MPCParams params;
    params.horizon = 10;
    params.dt = 0.1;
    params.max_iterations = 25;

    constexpr double kSpeed = 3.0;
    constexpr double kCurvature = 0.05;
    ReferencePath reference;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    for (int i = 0; i < params.horizon; ++i) {
        reference.x_ref.push_back(x);
        reference.y_ref.push_back(y);
        reference.yaw_ref.push_back(yaw);
        reference.v_ref.push_back(kSpeed);
        reference.a_ref.push_back(0.0);
        reference.k_ref.push_back(kCurvature);

        // Match the vehicle model's Euler position step and yaw rate.
        const double ds = kSpeed * params.dt;
        x += ds * std::cos(yaw);
        y += ds * std::sin(yaw);
        yaw += ds * kCurvature;
    }

    MPCState ego;
    ego.vx = kSpeed;
    InitMPC(params);
    const MPCOutput output = MPCProcess(ego, reference);

    ASSERT_TRUE(output.success);
    EXPECT_TRUE(std::isfinite(output.control.delta));
    EXPECT_TRUE(std::isfinite(output.control.accel));
    EXPECT_GT(output.first_reference_curvature, 0.0);
    EXPECT_GT(output.first_feedforward_steering, 0.0);
    EXPECT_GT(output.control.delta, 1e-4);
    EXPECT_GE(output.control.delta, -params.steering_max);
    EXPECT_LE(output.control.delta, params.steering_max);
    EXPECT_GE(output.control.accel, params.accel_min);
    EXPECT_LE(output.control.accel, params.accel_max);
}

TEST(OfflineMPCVerification, CTRL003Acceleration) {
    MPCParams params;
    params.horizon = 10;
    params.dt = 0.1;
    params.max_iterations = 25;

    constexpr double kTargetSpeed = 5.0;
    ReferencePath reference;
    for (int i = 0; i < params.horizon; ++i) {
        reference.x_ref.push_back(kTargetSpeed * params.dt * i);
        reference.y_ref.push_back(0.0);
        reference.yaw_ref.push_back(0.0);
        reference.v_ref.push_back(kTargetSpeed);
        reference.a_ref.push_back(0.0);
        reference.k_ref.push_back(0.0);
    }

    MPCState ego;
    ego.vx = 1.0;
    InitMPC(params);
    const MPCOutput output = MPCProcess(ego, reference);

    ASSERT_TRUE(output.success);
    ::testing::Test::RecordProperty(
        "acceleration", std::to_string(output.control.accel));
    ::testing::Test::RecordProperty(
        "first_speed_error", std::to_string(output.first_speed_error));
    EXPECT_TRUE(std::isfinite(output.control.delta));
    EXPECT_TRUE(std::isfinite(output.control.accel));
    EXPECT_GT(output.control.accel, 0.0);
    EXPECT_LE(output.control.accel, params.accel_max);
    EXPECT_GE(output.control.accel, params.accel_min);
    EXPECT_LT(std::abs(output.control.delta), 0.05);
    EXPECT_LT(output.first_speed_error, 0.0);
}

TEST(OfflineMPCVerification, CTRL004Deceleration) {
    MPCParams params;
    params.horizon = 10;
    params.dt = 0.1;
    params.max_iterations = 25;

    constexpr double kTargetSpeed = 1.0;
    ReferencePath reference;
    for (int i = 0; i < params.horizon; ++i) {
        reference.x_ref.push_back(kTargetSpeed * params.dt * i);
        reference.y_ref.push_back(0.0);
        reference.yaw_ref.push_back(0.0);
        reference.v_ref.push_back(kTargetSpeed);
        reference.a_ref.push_back(0.0);
        reference.k_ref.push_back(0.0);
    }

    MPCState ego;
    ego.vx = 5.0;
    InitMPC(params);
    const MPCOutput output = MPCProcess(ego, reference);

    ASSERT_TRUE(output.success);
    ::testing::Test::RecordProperty(
        "acceleration", std::to_string(output.control.accel));
    ::testing::Test::RecordProperty(
        "first_speed_error", std::to_string(output.first_speed_error));
    EXPECT_TRUE(std::isfinite(output.control.delta));
    EXPECT_TRUE(std::isfinite(output.control.accel));
    EXPECT_LT(output.control.accel, 0.0);
    EXPECT_GE(output.control.accel, params.accel_min);
    EXPECT_LE(output.control.accel, params.accel_max);
    EXPECT_LT(std::abs(output.control.delta), 0.05);
    EXPECT_GT(output.first_speed_error, 0.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
