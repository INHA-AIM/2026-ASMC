#include <gtest/gtest.h>
#include <cmath>
#include "MPC/Solver/mpc_solver.hpp"
#include "MPC/Model/vehicle_model.hpp"

namespace {
MPCParams Params() {
    MPCParams p;
    p.horizon = 36; p.dt = .05; p.max_iterations = 25;
    p.steering_curvature_gain = .844;
    p.weight_contour_error = 6; p.weight_lag_error = .5;
    p.weight_heading_error = 3; p.weight_speed_error = 2;
    p.weight_control = 1.25; p.weight_control_rate = 100;
    p.weight_control_rate_accel = 2.5; p.weight_terminal = 8;
    p.lr_init = .2; p.line_search_steps = 7;
    p.accel_max = 2;
    return p;
}
ReferencePath Reference(const MPCState& ego, const std::vector<MPCControl>& controls,
                        const MPCParams& p) {
    const auto states = predictTrajectory(ego, controls, p.dt, p.wheelbase,
        p.vel_min, p.vel_max, p.steering_curvature_gain);
    ReferencePath ref;
    for (size_t i = 0; i < controls.size(); ++i) {
        ref.x_ref.push_back(states[i].x); ref.y_ref.push_back(states[i].y);
        ref.yaw_ref.push_back(states[i].yaw); ref.v_ref.push_back(states[i].vx);
        ref.a_ref.push_back(controls[i].accel);
        ref.k_ref.push_back(p.steering_curvature_gain * std::tan(controls[i].delta) / p.wheelbase);
    }
    return ref;
}
}

TEST(BrakingWarmStart, StaleAccelerationDoesNotPostponeBrakingOnGentleCurve) {
    const auto p = Params(); const MPCState ego{0, 0, 0, 12};
    std::vector<MPCControl> braking(p.horizon), stale(p.horizon);
    for (int i = 0; i < p.horizon; ++i) {
        braking[i] = {.08, -std::min(2.4, .24 + .24 * i)};
        stale[i] = {.08, 1.5};
    }
    const MPCControl previous{.08, .15};
    const auto result = solveMPC(ego, Reference(ego, braking, p), previous, stale, p);
    ASSERT_TRUE(result.success);
    EXPECT_LT(result.control.accel, 0);
    MPCControl last = previous;
    for (const auto& u : result.controls) {
        EXPECT_LE(std::abs(u.delta - last.delta), p.steering_rate_max * p.dt + 1e-10);
        EXPECT_LE(std::abs(u.delta), p.steering_max);
        EXPECT_GE(u.accel, p.accel_min); EXPECT_LE(u.accel, p.accel_max);
        last = u;
    }
}

TEST(BrakingWarmStart, SeedComparisonRetainsShiftedSteeringAndPreviousActuatorAnchor) {
    auto p = Params(); p.max_iterations = 0;
    const MPCState ego{0, 0, 0, 12};
    std::vector<MPCControl> braking(p.horizon, {.1, -1.5});
    std::vector<MPCControl> stale(p.horizon, {.1, 1.5});
    const MPCControl previous{0, .15};
    const auto result = solveMPC(ego, Reference(ego, braking, p), previous, stale, p);
    ASSERT_TRUE(result.success);
    EXPECT_NEAR(result.control.delta, p.steering_rate_max * p.dt, 1e-12);
    EXPECT_LT(result.control.accel, 0);
}

TEST(BrakingWarmStart, AccelerationReferenceRefreshesStaleWarmStart) {
    auto p = Params(); p.max_iterations = 0;
    const MPCState ego{0, 0, 0, 8};
    std::vector<MPCControl> desired(p.horizon, {.02, 1.0});
    std::vector<MPCControl> warm(p.horizon, {.02, .3});
    const auto result = solveMPC(ego, Reference(ego, desired, p), {.02, .3}, warm, p);
    ASSERT_TRUE(result.success);
    for (const auto& u : result.controls) {
        EXPECT_DOUBLE_EQ(u.delta, .02); EXPECT_DOUBLE_EQ(u.accel, 1.0);
    }
}

TEST(BrakingWarmStart, MatchingBrakingHistoryIsPreserved) {
    auto p = Params(); p.max_iterations = 0;
    const MPCState ego{0, 0, 0, 8};
    std::vector<MPCControl> warm(p.horizon, {.04, -.8});
    const auto result = solveMPC(ego, Reference(ego, warm, p), {.04, -.8}, warm, p);
    ASSERT_TRUE(result.success);
    for (const auto& u : result.controls) {
        EXPECT_DOUBLE_EQ(u.delta, .04); EXPECT_DOUBLE_EQ(u.accel, -.8);
    }
}
