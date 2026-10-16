#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

#include "MPC/Cost/cost_function.hpp"
#include "MPC/Constraints/constraints.hpp"
#include "MPC/Model/vehicle_model.hpp"
#include "MPC/Model/steering_delay.hpp"
#include "MPC/MPC.hpp"
#include "MPC/Solver/mpc_solver.hpp"
#include "Util/ParameterLoader.hpp"

namespace {
struct Metrics { double rate_energy = 0; double max_error = 0; };

Metrics Drive(double speed, double curvature, bool scheduled, bool noisy) {
    MPCParams p;
    loadMPCParameters(CONTROLLER_YAML_PATH, p);
    // This fixture isolates the rate envelope with an instantaneous plant.
    // The delayed-plant fixture below tests the response-delay model.
    p.steering_response_delay_s = 0.0;
    p.steering_rate_reference_speed_mps = scheduled ? 5.0 : 0.0;
    MPCState actual;
    actual.vx = speed;
    actual.y = 0.15;
    MPCControl previous;
    std::vector<MPCControl> warm;
    Metrics metrics;
    for (int step = 0; step < 160; ++step) {
        const double progress = speed * p.dt * step;
        ReferencePath ref;
        for (int i = 0; i < p.horizon; ++i) {
            const double s = progress + speed * p.dt * i;
            ref.x_ref.push_back(curvature == 0 ? s : std::sin(curvature*s)/curvature);
            ref.y_ref.push_back(curvature == 0 ? 0 : (1-std::cos(curvature*s))/curvature);
            ref.yaw_ref.push_back(curvature*s);
            ref.v_ref.push_back(speed);
            ref.a_ref.push_back(0);
            ref.k_ref.push_back(curvature);
        }
        auto measured = actual;
        // Alternating GPS correction at 5 Hz; physical vehicle does not jump.
        if (noisy) measured.y += (step/4 % 2 == 0 ? .08 : -.08);
        const auto result = solveMPC(measured, ref, previous, warm, p);
        EXPECT_TRUE(result.success);
        if (!result.success) return {};
        EXPECT_LE(std::abs(result.control.delta-previous.delta), SteeringRateLimit(measured.vx,p)*p.dt+1e-9);
        if (step > 40) {
            metrics.rate_energy += std::pow((result.control.delta-previous.delta)/p.dt,2);
            metrics.max_error = std::max(metrics.max_error,
                std::abs(computeTrackingError(actual, ref, 0).contour));
        }
        previous = result.control;
        warm = result.controls;
        // Keep speed fixed to isolate lateral dynamics from pedal calibration.
        auto applied = previous;
        applied.accel = 0;
        actual = updateVehicleState(actual, applied, p.dt, p.wheelbase,
                                    p.vel_min, p.vel_max,p.steering_curvature_gain);
    }
    return metrics;
}
}

TEST(StraightStability, LegacyAndLowSpeedPreserveRateLimit) {
    MPCParams p;
    EXPECT_DOUBLE_EQ(SteeringRateLimit(58.0/3.6,p),1);
    p.steering_rate_reference_speed_mps=5;
    EXPECT_DOUBLE_EQ(SteeringRateLimit(5,p),1);
    EXPECT_DOUBLE_EQ(SteeringRateLimit(0,p),1);
    EXPECT_DOUBLE_EQ(SteeringRateLimit(100,p),.1);
}

TEST(StraightStability, HighSpeedGpsStepsReduceSteeringVariationWithoutDrift) {
    for (double speed : {40.0/3.6,58.0/3.6}) {
        const auto baseline=Drive(speed,0,false,true);
        const auto candidate=Drive(speed,0,true,true);
        EXPECT_LT(candidate.rate_energy,baseline.rate_energy);
        EXPECT_LT(candidate.max_error,.20);
        EXPECT_LE(candidate.max_error,baseline.max_error+.05);
    }
}

TEST(StraightStability, CurvedPathStillTracks) {
    for (double curvature : {-.006,.006}) {
        const auto result=Drive(58.0/3.6,curvature,true,false);
        EXPECT_LT(result.max_error,.20);
    }
}

TEST(StraightStability, FiftyKphGentleCurveTracksWithinTwentyCentimeters) {
    // Radius 80 m: 50 km/h requires about 2.41 m/s^2, below the
    // planner's new 2.6 * 0.95 curve-speed budget. This is an ideal plant;
    // simulator sensing and actuator behavior still require a new driving bag.
    for (double curvature : {-1.0/80.0, 1.0/80.0}) {
        const auto result = Drive(50.0/3.6, curvature, true, true);
        EXPECT_LT(result.max_error, .20);
    }
}

TEST(SteeringDelay, PreservesAccelerationAndInterpolatesPastCommands) {
    std::vector<MPCControl> commands(3);
    commands[0].delta = 0.3;
    commands[1].delta = 0.4;
    commands[2].delta = 0.5;
    commands[0].accel = -2.0;
    const auto delayed = ApplySteeringDelay(commands, {0.1, 0.2}, 0.0, 0.075, 0.05);
    EXPECT_NEAR(delayed[0].delta, 0.15, 1e-12);
    EXPECT_NEAR(delayed[1].delta, 0.25, 1e-12);
    EXPECT_NEAR(delayed[2].delta, 0.35, 1e-12);
    EXPECT_DOUBLE_EQ(delayed[0].accel, -2.0);
    const auto immediate = ApplySteeringDelay(commands, {}, 0.0, 0.0, 0.05);
    EXPECT_DOUBLE_EQ(immediate[0].delta, commands[0].delta);
}

TEST(SteeringDelay, PlannerStopPreservesInFlightSteeringAndDrainsHistory) {
    MPCParams params;
    params.dt = 0.05;
    params.steering_response_delay_s = 0.15;
    InitMPC(params);
    g_mpc.steering_history = {0.1, 0.2, 0.3, 0.4};
    g_mpc.warm_start = {{0.4, 1.0}};
    ResetMPCForPlannerStop(0.03);
    EXPECT_TRUE(g_mpc.warm_start.empty());
    EXPECT_DOUBLE_EQ(g_mpc.last_control.delta, 0.03);
    EXPECT_DOUBLE_EQ(g_mpc.last_control.accel, 0.0);
    const auto delayed = ApplySteeringDelay(std::vector<MPCControl>(3),
        g_mpc.steering_history, g_mpc.last_control.delta, 0.15, 0.05);
    // A valid STOP does not make commands already in the actuator disappear.
    EXPECT_NEAR(delayed[0].delta, 0.3, 1e-12);
    EXPECT_NEAR(delayed[1].delta, 0.4, 1e-12);
    EXPECT_NEAR(delayed[2].delta, 0.03, 1e-12);
    for (int tick = 0; tick < 4; ++tick) ResetMPCForPlannerStop(0.03);
    EXPECT_LE(g_mpc.steering_history.size(), 4u);
    for (double steering : g_mpc.steering_history) EXPECT_DOUBLE_EQ(steering, 0.03);
    ResetMPCOnFailSafe();
    EXPECT_TRUE(g_mpc.steering_history.empty());
    EXPECT_DOUBLE_EQ(g_mpc.last_control.delta, 0.0);
}

namespace {
Metrics DriveDelayedPlant(double speed, bool compensate) {
    MPCParams params;
    loadMPCParameters(CONTROLLER_YAML_PATH, params);
    params.steering_response_delay_s = compensate ? 0.15 : 0.0;
    InitMPC(params);
    MPCState actual;
    actual.vx = speed;
    actual.y = 0.15;
    std::vector<double> issued(3, 0.0);
    Metrics metrics;
    for (int step = 0; step < 200; ++step) {
        ReferencePath ref;
        for (int i = 0; i < params.horizon; ++i) {
            const double x = speed * params.dt * (step + i);
            // Smooth S transition, then a straight: checks recovery from a
            // bend instead of only a constant-curvature steady state.
            const double u = (x - 45.0) / 10.0;
            const double z = std::tanh(u);
            const double y = 0.4 * (1.0 + z);
            const double dy = 0.04 * (1.0 - z*z);
            const double ddy = -0.008 * z * (1.0 - z*z);
            ref.x_ref.push_back(x);
            ref.y_ref.push_back(y);
            ref.yaw_ref.push_back(std::atan(dy));
            ref.v_ref.push_back(speed);
            ref.a_ref.push_back(0.0);
            ref.k_ref.push_back(ddy / std::pow(1 + dy*dy, 1.5));
        }
        const auto output = MPCProcess(actual, ref);
        EXPECT_TRUE(output.success);
        if (!output.success) return {};
        MPCControl applied;
        // Independent three-sample delay in the physical plant.
        applied.delta = issued[step];
        issued.push_back(output.control.delta);
        actual = updateVehicleState(actual, applied, params.dt, params.wheelbase,
            params.vel_min, params.vel_max, params.steering_curvature_gain);
        if (step > 20) {
            metrics.max_error = std::max(metrics.max_error,
                std::abs(computeTrackingError(actual, ref, 1).contour));
            const double change = issued.back() - issued[issued.size() - 2];
            metrics.rate_energy += change * change;
        }
    }
    return metrics;
}
}

TEST(SteeringDelay, DelayedPlantTracksBendAndStraightAtHighSpeed) {
    for (double speed : {40.0/3.6, 58.0/3.6}) {
        const auto legacy = DriveDelayedPlant(speed, false);
        const auto corrected = DriveDelayedPlant(speed, true);
        EXPECT_LT(corrected.max_error, 0.2);
        EXPECT_LT(corrected.max_error, legacy.max_error);
        EXPECT_LT(corrected.rate_energy, legacy.rate_energy);
    }
}

TEST(PlannerBraking, FollowsCurveUntilRestWithActuatorDelay) {
    for (double k : {-0.0125, 0.0125}) {
        MPCParams p;
        loadMPCParameters(CONTROLLER_YAML_PATH, p);
        InitMPC(p);
        const double speed = 58.0/3.6;
        ReferencePath ref;
        for (int i=0; i<p.horizon; ++i) {
            const double s = speed*p.dt*i;
            ref.x_ref.push_back(std::sin(k*s)/k);
            ref.y_ref.push_back((1-std::cos(k*s))/k);
            ref.yaw_ref.push_back(k*s); ref.k_ref.push_back(k);
            ref.v_ref.push_back(speed); ref.a_ref.push_back(0.0);
        }
        MPCState actual;
        actual.vx = speed;
        const double initial_delta = SteeringFeedforward(k, p.wheelbase,
            p.steering_max, p.steering_curvature_gain);
        g_mpc.last_control.delta = initial_delta;
        g_mpc.steering_history.assign(4, initial_delta);
        ASSERT_TRUE(MPCProcess(actual, ref).success);
        std::vector<double> issued(3, initial_delta);
        double max_error = 0.0;
        for (int tick=0; actual.vx>=0.1 && tick<100; ++tick) {
            double steering = 0.0;
            ASSERT_TRUE(MPCPlannerStopSteering(actual, .3, steering));
            EXPECT_LE(std::abs(steering-g_mpc.last_control.delta),
                SteeringRateLimit(actual.vx,p)*p.dt+1e-9);
            ResetMPCForPlannerStop(steering);
            EXPECT_DOUBLE_EQ(g_mpc.last_control.accel, 0.0);
            issued.push_back(steering);
            MPCControl applied{issued[tick], -4.75};
            actual = updateVehicleState(actual, applied, p.dt, p.wheelbase,
                p.vel_min, p.vel_max, p.steering_curvature_gain);
            max_error = std::max(max_error, std::abs(
                std::hypot(actual.x, actual.y-1/k)-std::abs(1/k)));
        }
        EXPECT_LT(actual.vx, 0.1);
        EXPECT_LT(max_error, .20);
        double steering = 1.0;
        EXPECT_FALSE(MPCPlannerStopSteering(actual,.3,steering));
        EXPECT_TRUE(g_mpc.braking_geometry.empty());
        ResetMPCOnFailSafe();
        EXPECT_FALSE(MPCPlannerStopSteering(MPCState{0,0,0,speed},.3,steering));
    }
}
