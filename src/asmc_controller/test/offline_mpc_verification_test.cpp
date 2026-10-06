#include <gtest/gtest.h>

#include <cmath>

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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
