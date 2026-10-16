#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cstdio>
#include <unistd.h>

#include "Util/ParameterLoader.hpp"

TEST(ParameterLoaderRegression, LoadsRealControllerYaml) {
    const YAML::Node root = YAML::LoadFile(CONTROLLER_YAML_PATH);
    MPCParams params;
    // Sentinels also catch missing keys whose YAML values match defaults.
    params.horizon = -123;
    params.dt = -123;
    params.max_iterations = -123;
    params.wheelbase = -123;
    params.steering_curvature_gain = -123;
    params.steering_response_delay_s = -123;
    params.control_frequency = -123;
    params.steering_max = -123;
    params.steering_rate_max = -123;
    params.accel_max = -123;
    params.accel_min = -123;
    params.brake_full_decel = -123;
    params.use_calibrated_pedal_mapping = false;
    params.accel_pedal_lut = {-123.0};
    params.accel_response_lut = {-123.0};
    params.brake_pedal_lut = {-123.0};
    params.brake_response_lut = {-123.0};
    params.vel_min = -123;
    params.vel_max = -123;
    params.stuck_ego_speed = -123;
    params.stuck_target_speed = -123;
    params.stuck_min_path_progress = -123;
    params.stuck_timeout = -123;
    params.stuck_recovery_duration = -123;
    params.stuck_launch_accel = -123;
    params.weight_contour_error = -123;
    params.curve_contour_boost = -123;
    params.contour_boost_start_curvature = -123;
    params.contour_boost_full_curvature = -123;
    params.weight_lag_error = -123;
    params.weight_heading_error = -123;
    params.weight_speed_error = -123;
    params.weight_control = -123;
    params.weight_control_rate = -123;
    params.steering_rate_reference_speed_mps = -123;
    params.steering_rate_min = -123;
    params.weight_control_rate_accel = -123;
    params.weight_terminal = -123;
    params.lr_init = -123;
    params.lr_min = -123;
    params.convergence_eps = -123;
    params.line_search_steps = -123;

    loadMPCParameters(CONTROLLER_YAML_PATH, params);

    EXPECT_EQ(params.horizon, root["mpc"]["horizon"].as<int>());
    EXPECT_DOUBLE_EQ(params.dt, root["mpc"]["dt"].as<double>());
    EXPECT_EQ(params.max_iterations, root["mpc"]["max_iterations"].as<int>());
    EXPECT_DOUBLE_EQ(params.wheelbase, root["mpc"]["wheelbase"].as<double>());
    EXPECT_DOUBLE_EQ(params.steering_curvature_gain, root["mpc"]["steering_curvature_gain"].as<double>());
    EXPECT_DOUBLE_EQ(params.steering_response_delay_s,
        root["mpc"]["steering_response_delay_s"].as<double>());
    EXPECT_DOUBLE_EQ(params.control_frequency, root["control_frequency"].as<double>());
    EXPECT_DOUBLE_EQ(params.steering_max, root["constraints"]["steering"]["max_angle"].as<double>());
    EXPECT_DOUBLE_EQ(params.steering_rate_max, root["constraints"]["steering"]["max_rate"].as<double>());
    EXPECT_DOUBLE_EQ(params.accel_max, root["constraints"]["acceleration"]["max_forward"].as<double>());
    EXPECT_DOUBLE_EQ(params.accel_min, root["constraints"]["acceleration"]["max_reverse"].as<double>());
    EXPECT_DOUBLE_EQ(params.brake_full_decel, root["control_mapping"]["brake_full_decel"].as<double>());
    EXPECT_EQ(params.use_calibrated_pedal_mapping, root["control_mapping"]["use_calibrated_lut"].as<bool>());
    EXPECT_EQ(params.accel_pedal_lut, root["control_mapping"]["accel"]["pedal"].as<std::vector<double>>());
    EXPECT_EQ(params.accel_response_lut, root["control_mapping"]["accel"]["response_mps2"].as<std::vector<double>>());
    EXPECT_EQ(params.brake_pedal_lut, root["control_mapping"]["brake"]["pedal"].as<std::vector<double>>());
    EXPECT_EQ(params.brake_response_lut, root["control_mapping"]["brake"]["response_mps2"].as<std::vector<double>>());
    EXPECT_DOUBLE_EQ(params.vel_min, root["constraints"]["velocity"]["min"].as<double>());
    EXPECT_DOUBLE_EQ(params.vel_max, root["constraints"]["velocity"]["max"].as<double>());
    EXPECT_DOUBLE_EQ(params.stuck_ego_speed, root["stuck_recovery"]["ego_speed_threshold"].as<double>());
    EXPECT_DOUBLE_EQ(params.stuck_target_speed, root["stuck_recovery"]["target_speed_threshold"].as<double>());
    EXPECT_DOUBLE_EQ(params.stuck_min_path_progress, root["stuck_recovery"]["min_path_progress"].as<double>());
    EXPECT_DOUBLE_EQ(params.stuck_timeout, root["stuck_recovery"]["timeout"].as<double>());
    EXPECT_DOUBLE_EQ(params.stuck_recovery_duration, root["stuck_recovery"]["duration"].as<double>());
    EXPECT_DOUBLE_EQ(params.stuck_launch_accel, root["stuck_recovery"]["launch_accel"].as<double>());
    EXPECT_DOUBLE_EQ(params.weight_contour_error, root["cost"]["contour_error"].as<double>());
    EXPECT_DOUBLE_EQ(params.curve_contour_boost, root["cost"]["curve_contour_boost"].as<double>());
    EXPECT_DOUBLE_EQ(params.contour_boost_start_curvature, root["cost"]["contour_boost_start_curvature"].as<double>());
    EXPECT_DOUBLE_EQ(params.contour_boost_full_curvature, root["cost"]["contour_boost_full_curvature"].as<double>());
    EXPECT_DOUBLE_EQ(params.weight_lag_error, root["cost"]["lag_error"].as<double>());
    EXPECT_DOUBLE_EQ(params.weight_heading_error, root["cost"]["heading_error"].as<double>());
    EXPECT_DOUBLE_EQ(params.weight_speed_error, root["cost"]["speed_error"].as<double>());
    EXPECT_DOUBLE_EQ(params.weight_control, root["cost"]["control"].as<double>());
    EXPECT_DOUBLE_EQ(params.weight_control_rate, root["cost"]["control_rate"].as<double>());
    EXPECT_DOUBLE_EQ(params.steering_rate_reference_speed_mps,
        root["constraints"]["steering"]["rate_reference_speed_mps"].as<double>());
    EXPECT_DOUBLE_EQ(params.steering_rate_min,
        root["constraints"]["steering"]["min_rate"].as<double>());
    EXPECT_DOUBLE_EQ(params.weight_control_rate_accel, root["cost"]["control_rate_accel"].as<double>());
    EXPECT_DOUBLE_EQ(params.weight_terminal, root["cost"]["terminal"].as<double>());
    EXPECT_DOUBLE_EQ(params.lr_init, root["solver"]["lr_init"].as<double>());
    EXPECT_DOUBLE_EQ(params.lr_min, root["solver"]["lr_min"].as<double>());
    EXPECT_DOUBLE_EQ(params.convergence_eps, root["solver"]["convergence_eps"].as<double>());
    EXPECT_EQ(params.line_search_steps, root["solver"]["line_search_steps"].as<int>());
}

TEST(ParameterLoaderRegression, UsesValidatedTuningAndPreservesPhysicalLimits) {
    MPCParams params;
    loadMPCParameters(CONTROLLER_YAML_PATH, params);
    EXPECT_DOUBLE_EQ(params.accel_max, 2.0);
    EXPECT_DOUBLE_EQ(params.vel_max, 60.0);
    EXPECT_DOUBLE_EQ(params.weight_heading_error, 3.0);
    EXPECT_DOUBLE_EQ(params.weight_speed_error, 8.0);
    EXPECT_DOUBLE_EQ(params.steering_response_delay_s, 0.15);
    EXPECT_DOUBLE_EQ(params.steering_rate_reference_speed_mps, 5.0);
    EXPECT_DOUBLE_EQ(params.weight_control, 1.25);
    EXPECT_DOUBLE_EQ(params.weight_control_rate, 110.0);
    EXPECT_DOUBLE_EQ(params.weight_control_rate_accel, 2.5);
    EXPECT_DOUBLE_EQ(params.weight_terminal, 8.0);
    EXPECT_DOUBLE_EQ(params.lr_init, 0.2);
    EXPECT_EQ(params.line_search_steps, 7);
}

TEST(ParameterLoaderRegression, BadOrMissingCurveScheduleDisablesBoostOnReload) {
    for (const std::string& cost : {
            "  contour_error: 6.0\n",
            "  curve_contour_boost: 6.0\n",
            "  curve_contour_boost: bad\n  contour_boost_start_curvature: .004\n  contour_boost_full_curvature: .012\n",
            "  curve_contour_boost: .nan\n  contour_boost_start_curvature: .004\n  contour_boost_full_curvature: .012\n",
            "  curve_contour_boost: 6.0\n  contour_boost_start_curvature: .012\n  contour_boost_full_curvature: .004\n"}) {
        char path[] = "/tmp/asmc_curve_cost_XXXXXX";
        const int fd = mkstemp(path);
        ASSERT_GE(fd, 0);
        close(fd);
        { std::ofstream file(path); file << "cost:\n" << cost; }
        MPCParams p;
        p.curve_contour_boost = 99.0;
        loadMPCParameters(path, p);
        std::remove(path);
        EXPECT_DOUBLE_EQ(p.curve_contour_boost, 0.0);
    }
}
