#include <gtest/gtest.h>

#include <vector>

#include "MPC/Cost/cost_function.hpp"

namespace {

TEST(AccelerationFeedforwardCostTest, EffortIsZeroAtReference) {
    MPCControl control;
    control.accel = -2.0;
    EXPECT_DOUBLE_EQ(
        0.0, computeControlEffortCost(
                 control, /*weight_steer=*/0.0, /*weight_accel=*/3.0,
                 /*delta_ff=*/0.0, /*accel_ref=*/-2.0));
}

TEST(AccelerationFeedforwardCostTest, ResidualIsSymmetricAroundReference) {
    MPCControl slower_braking;
    slower_braking.accel = -1.0;
    MPCControl stronger_braking;
    stronger_braking.accel = -3.0;
    const double above = computeControlEffortCost(
        slower_braking, 0.0, 2.0, 0.0, -2.0);
    const double below = computeControlEffortCost(
        stronger_braking, 0.0, 2.0, 0.0, -2.0);
    EXPECT_DOUBLE_EQ(above, below);
}

TEST(AccelerationFeedforwardCostTest, ZeroReferenceMatchesLegacySquaredInput) {
    MPCControl control;
    control.accel = -1.5;
    constexpr double weight = 2.5;
    EXPECT_DOUBLE_EQ(
        weight * control.accel * control.accel,
        computeControlEffortCost(control, 0.0, weight, 0.0, 0.0));
}

TEST(AccelerationFeedforwardCostTest, BreakdownUsesStageAccelerationReference) {
    ReferencePath reference;
    reference.x_ref = {0.0};
    reference.y_ref = {0.0};
    reference.yaw_ref = {0.0};
    reference.v_ref = {5.0};
    reference.a_ref = {-2.0};
    reference.k_ref = {0.0};

    MPCParams params;
    params.dt = 0.1;
    params.weight_contour_error = 0.0;
    params.weight_lag_error = 0.0;
    params.weight_heading_error = 0.0;
    params.weight_speed_error = 0.0;
    params.weight_control = 2.0;
    params.weight_control_rate = 0.0;
    params.weight_control_rate_accel = 0.0;
    params.weight_terminal = 0.0;

    MPCControl at_reference;
    at_reference.accel = -2.0;
    const CostBreakdown cost = computeCostBreakdown(
        {MPCState{}}, {at_reference}, reference, MPCControl{}, params);
    EXPECT_DOUBLE_EQ(0.0, cost.control);
    EXPECT_DOUBLE_EQ(0.0, cost.acceleration_control);
}

TEST(AccelerationFeedforwardCostTest, BreakdownClipsRawReferenceToActuatorBounds) {
    ReferencePath reference;
    reference.x_ref = {0.0};
    reference.y_ref = {0.0};
    reference.yaw_ref = {0.0};
    reference.v_ref = {5.0};
    reference.a_ref = {-20.0};
    reference.k_ref = {0.0};

    MPCParams params;
    params.dt = 0.1;
    params.accel_min = -5.0;
    params.accel_max = 3.0;
    params.weight_contour_error = 0.0;
    params.weight_lag_error = 0.0;
    params.weight_heading_error = 0.0;
    params.weight_speed_error = 0.0;
    params.weight_control = 2.0;
    params.weight_control_rate = 0.0;
    params.weight_control_rate_accel = 0.0;
    params.weight_terminal = 0.0;

    MPCControl at_clipped_reference;
    at_clipped_reference.accel = params.accel_min;
    const CostBreakdown cost = computeCostBreakdown(
        {MPCState{}}, {at_clipped_reference}, reference, MPCControl{}, params);
    EXPECT_DOUBLE_EQ(0.0, cost.acceleration_control);
}

TEST(AccelerationFeedforwardCostTest, RatePenaltyIgnoresReference) {
    MPCControl previous;
    previous.accel = -0.5;
    MPCControl current;
    current.accel = -2.0;
    constexpr double weight = 4.5;
    const double expected = weight * 1.5 * 1.5;
    EXPECT_DOUBLE_EQ(expected,
                     computeControlRateCost(previous, current,
                                            /*weight_steer=*/0.0, weight));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
