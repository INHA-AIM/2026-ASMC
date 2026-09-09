#include <gtest/gtest.h>

#include <cmath>

#include "cost/cost_function.hpp"
#include "model/vehicle_model.hpp"

namespace {

constexpr double kWheelbase = 3.0;
constexpr double kSteeringMax = 0.6981;  // ~40 deg, matches mpc_params.yaml

}  // namespace

// Commit C: curvature-aware MPC steering feed-forward.

// Test C-1: kappa = 0 -> delta_ff = 0.
TEST(SteeringFeedforwardTest, ZeroCurvatureProducesZeroFeedforward) {
    EXPECT_DOUBLE_EQ(0.0,
                     SteeringFeedforward(0.0, kWheelbase, kSteeringMax));
}

// Test C-2: 양의 curvature(좌회전, CCW) -> 양의 steering.
// frenet_planner의 부호 규약(kappa_r = dtheta/ds, Apollo류)과 kinematic
// bicycle model의 yaw_dot = vx/L*tan(delta) = vx*kappa가 동일 부호계이므로
// 별도의 부호 반전 없이 atan(L*kappa)를 그대로 쓴다.
TEST(SteeringFeedforwardTest, PositiveCurvatureProducesPositiveSteering) {
    const double delta_ff = SteeringFeedforward(0.05, kWheelbase, kSteeringMax);
    EXPECT_GT(delta_ff, 0.0);
    EXPECT_DOUBLE_EQ(std::atan(kWheelbase * 0.05), delta_ff);
}

// Test C-3: 음의 curvature(우회전) -> 반대 부호.
TEST(SteeringFeedforwardTest, NegativeCurvatureProducesNegativeSteering) {
    const double delta_ff = SteeringFeedforward(-0.05, kWheelbase, kSteeringMax);
    EXPECT_LT(delta_ff, 0.0);
    EXPECT_DOUBLE_EQ(std::atan(kWheelbase * -0.05), delta_ff);
}

// Test C-4: 매우 큰 curvature에서 steering_max로 안전하게 clip.
TEST(SteeringFeedforwardTest, ExtremeCurvatureClipsToSteeringMax) {
    EXPECT_DOUBLE_EQ(kSteeringMax,
                     SteeringFeedforward(1000.0, kWheelbase, kSteeringMax));
    EXPECT_DOUBLE_EQ(-kSteeringMax,
                     SteeringFeedforward(-1000.0, kWheelbase, kSteeringMax));
}

// Test C-5: 곡선 reference에서 steering effort cost는 delta=delta_ff일 때
// delta=0보다 작아야 한다. 직선(delta_ff=0)에서는 기존 semantics(순수
// delta^2)가 그대로 유지된다.
TEST(SteeringFeedforwardTest, CostPrefersFeedforwardDeltaOnCurve) {
    const double curvature = 0.05;
    const double delta_ff =
        SteeringFeedforward(curvature, kWheelbase, kSteeringMax);
    ASSERT_GT(delta_ff, 0.0);

    MPCControl at_zero;
    at_zero.delta = 0.0;
    MPCControl at_feedforward;
    at_feedforward.delta = delta_ff;

    const double cost_zero =
        computeControlEffortCost(at_zero, /*w_steer=*/1.0, /*w_accel=*/0.0,
                                 delta_ff);
    const double cost_feedforward =
        computeControlEffortCost(at_feedforward, /*w_steer=*/1.0,
                                 /*w_accel=*/0.0, delta_ff);
    EXPECT_LT(cost_feedforward, cost_zero);
    EXPECT_DOUBLE_EQ(0.0, cost_feedforward);
}

TEST(SteeringFeedforwardTest, StraightPathCostMatchesLegacyDeltaSquared) {
    MPCControl u;
    u.delta = 0.1;
    // delta_ff 기본값(0.0)이면 (delta-0)^2 = delta^2, 기존 semantics와 동일.
    EXPECT_DOUBLE_EQ(0.1 * 0.1,
                     computeControlEffortCost(u, /*w_steer=*/1.0,
                                              /*w_accel=*/0.0));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
