#include "Util/ParameterLoader.hpp"

#include <cstdio>
#include <cmath>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

// node["a"]["b"] 형태의 dotted key 조회, 없으면 default 유지
template <typename T>
void loadInto(const YAML::Node& root, const std::string& dotted_key, T& out) {
    YAML::Node node = YAML::Clone(root);
    size_t start = 0;
    while (true) {
        size_t dot = dotted_key.find('/', start);
        std::string key = dotted_key.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
        if (!node[key]) return;
        node = node[key];
        if (dot == std::string::npos) break;
        start = dot + 1;
    }
    out = node.as<T>();
}

}  // namespace

// ========================================
// yaml 파일 → MPCParams (controller.yaml 구조/키 이름과 일치)
// ========================================
void loadMPCParameters(const std::string& yaml_path, MPCParams& params) {
    auto& p = params;

    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        std::printf("[MPC] Failed to load yaml '%s': %s (기본값으로 진행)\n", yaml_path.c_str(), e.what());
        return;
    }

    // 예측
    loadInto(root, "mpc/horizon",            p.horizon);
    loadInto(root, "mpc/dt",                 p.dt);
    loadInto(root, "mpc/max_iterations",     p.max_iterations);
    loadInto(root, "mpc/wheelbase",          p.wheelbase);
    loadInto(root, "mpc/steering_curvature_gain",
             p.steering_curvature_gain);
    loadInto(root, "control_frequency",      p.control_frequency);

    // 제약
    loadInto(root, "constraints/steering/max_angle", p.steering_max);
    loadInto(root, "constraints/steering/max_rate",  p.steering_rate_max);
    loadInto(root, "constraints/steering/rate_reference_speed_mps",
             p.steering_rate_reference_speed_mps);
    loadInto(root, "constraints/steering/min_rate", p.steering_rate_min);
    loadInto(root, "mpc/steering_response_delay_s", p.steering_response_delay_s);
    loadInto(root, "constraints/acceleration/max_forward", p.accel_max);
    loadInto(root, "constraints/acceleration/max_reverse", p.accel_min);
    loadInto(root, "control_mapping/brake_full_decel", p.brake_full_decel);
    loadInto(root, "control_mapping/use_calibrated_lut",
             p.use_calibrated_pedal_mapping);
    loadInto(root, "control_mapping/accel/pedal", p.accel_pedal_lut);
    loadInto(root, "control_mapping/accel/response_mps2",
             p.accel_response_lut);
    loadInto(root, "control_mapping/brake/pedal", p.brake_pedal_lut);
    loadInto(root, "control_mapping/brake/response_mps2",
             p.brake_response_lut);
    auto& response = p.stop_brake_response;
    loadInto(root, "control_mapping/stop_brake_response/enabled", response.enabled);
    loadInto(root, "control_mapping/stop_brake_response/acceleration_limit", response.acceleration_limit);
    loadInto(root, "control_mapping/stop_brake_response/acceleration_minimum_request", response.acceleration_minimum_request);
    loadInto(root, "control_mapping/stop_brake_response/learning_time_constant", response.learning_time_constant);
    loadInto(root, "control_mapping/stop_brake_response/response_time_constant", response.response_time_constant);
    loadInto(root, "control_mapping/stop_brake_response/minimum_gain", response.minimum_gain);
    loadInto(root, "control_mapping/stop_brake_response/maximum_gain", response.maximum_gain);
    loadInto(root, "control_mapping/stop_brake_response/minimum_request", response.minimum_request);
    loadInto(root, "control_mapping/stop_brake_response/minimum_speed", response.minimum_speed);
    if (!std::isfinite(response.acceleration_limit) || response.acceleration_limit <= 0 ||
        !std::isfinite(response.acceleration_minimum_request) || response.acceleration_minimum_request <= 0 ||
        !std::isfinite(response.learning_time_constant) || response.learning_time_constant <= 0 ||
        !std::isfinite(response.response_time_constant) || response.response_time_constant <= 0 ||
        !std::isfinite(response.minimum_gain) || response.minimum_gain <= 0 || response.minimum_gain > 1 ||
        !std::isfinite(response.maximum_gain) || response.maximum_gain < 1 ||
        !std::isfinite(response.minimum_request) || response.minimum_request <= 0 ||
        !std::isfinite(response.minimum_speed) || response.minimum_speed <= 0) {
        std::printf("[MPC] Invalid STOP brake response settings; compensation disabled\n");
        response = StopBrakeResponseConfig{};
    }
    loadInto(root, "constraints/velocity/min",       p.vel_min);
    loadInto(root, "constraints/velocity/max",       p.vel_max);

    loadInto(root, "stuck_recovery/ego_speed_threshold", p.stuck_ego_speed);
    loadInto(root, "stuck_recovery/target_speed_threshold", p.stuck_target_speed);
    loadInto(root, "stuck_recovery/min_path_progress", p.stuck_min_path_progress);
    loadInto(root, "stuck_recovery/timeout", p.stuck_timeout);
    loadInto(root, "stuck_recovery/duration", p.stuck_recovery_duration);
    loadInto(root, "stuck_recovery/launch_accel", p.stuck_launch_accel);

    // 비용 가중치
    loadInto(root, "cost/contour_error",  p.weight_contour_error);
    // New optional schedule never inherits an old tuning when reloading YAML.
    p.curve_contour_boost = 0.0;
    try {
        const auto cost = root["cost"];
        if (cost && cost["curve_contour_boost"] &&
            cost["contour_boost_start_curvature"] && cost["contour_boost_full_curvature"]) {
            loadInto(root, "cost/curve_contour_boost", p.curve_contour_boost);
            loadInto(root, "cost/contour_boost_start_curvature", p.contour_boost_start_curvature);
            loadInto(root, "cost/contour_boost_full_curvature", p.contour_boost_full_curvature);
        }
    } catch (const YAML::Exception&) {
        p.curve_contour_boost = 0.0;
    }
    if (!std::isfinite(p.curve_contour_boost) || p.curve_contour_boost < 0.0 ||
        !std::isfinite(p.contour_boost_start_curvature) ||
        !std::isfinite(p.contour_boost_full_curvature) ||
        p.contour_boost_start_curvature < 0.0 ||
        p.contour_boost_full_curvature <= p.contour_boost_start_curvature) {
        std::printf("[MPC] Invalid curve contour schedule; using base tracking cost\n");
        p.curve_contour_boost = 0.0;
    }
    loadInto(root, "cost/lag_error",      p.weight_lag_error);
    loadInto(root, "cost/heading_error",  p.weight_heading_error);
    loadInto(root, "cost/speed_error",    p.weight_speed_error);
    loadInto(root, "cost/control",        p.weight_control);
    loadInto(root, "cost/control_rate",       p.weight_control_rate);
    loadInto(root, "cost/control_rate_accel", p.weight_control_rate_accel);
    loadInto(root, "cost/terminal",       p.weight_terminal);

    // 솔버
    loadInto(root, "solver/lr_init",          p.lr_init);
    loadInto(root, "solver/lr_min",           p.lr_min);
    loadInto(root, "solver/convergence_eps",  p.convergence_eps);
    loadInto(root, "solver/line_search_steps",p.line_search_steps);

    if (!std::isfinite(p.steering_response_delay_s) ||
        p.steering_response_delay_s < 0.0 || p.steering_response_delay_s > 0.5) {
        std::printf("[MPC] Invalid steering response delay; disabling delay model\n");
        p.steering_response_delay_s = 0.0;
    }
    if (!std::isfinite(p.steering_rate_reference_speed_mps) ||
        p.steering_rate_reference_speed_mps < 0.0 ||
        !std::isfinite(p.steering_rate_min) ||
        p.steering_rate_min <= 0.0 || p.steering_rate_min > p.steering_rate_max) {
        std::printf("[MPC] Invalid steering rate schedule; disabling schedule\n");
        p.steering_rate_reference_speed_mps = 0.0;
        p.steering_rate_min = p.steering_rate_max;
    }
    if (p.brake_full_decel <= 0.0) {
        std::printf("[MPC] Invalid brake_full_decel %.3f; using 8.9 m/s^2\n",
                    p.brake_full_decel);
        p.brake_full_decel = 8.9;
    }
    if (p.steering_curvature_gain <= 0.0 ||
        !std::isfinite(p.steering_curvature_gain)) {
        std::printf("[MPC] Invalid steering_curvature_gain %.3f; using 1.0\n",
                    p.steering_curvature_gain);
        p.steering_curvature_gain = 1.0;
    }

    const auto valid_lut = [](const std::vector<double>& pedal,
                              const std::vector<double>& response) {
        if (pedal.size() < 2 || pedal.size() != response.size()) return false;
        if (pedal.front() != 0.0 || response.front() != 0.0) return false;
        for (size_t i = 1; i < pedal.size(); ++i) {
            if (pedal[i] <= pedal[i - 1] || response[i] <= response[i - 1] ||
                pedal[i] > 1.0) {
                return false;
            }
        }
        return true;
    };
    if (p.use_calibrated_pedal_mapping &&
        (!valid_lut(p.accel_pedal_lut, p.accel_response_lut) ||
         !valid_lut(p.brake_pedal_lut, p.brake_response_lut))) {
        std::printf("[MPC] Invalid calibrated pedal LUT; using linear mapping\n");
        p.use_calibrated_pedal_mapping = false;
    }

    std::printf("[MPC] Params: horizon=%d dt=%.2f max_iter=%d wheelbase=%.2f "
                "steer_gain=%.3f freq=%.1fHz\n",
                p.horizon, p.dt, p.max_iterations, p.wheelbase,
                p.steering_curvature_gain, p.control_frequency);
    std::printf("[MPC] Weights: contour=%.2f lag=%.2f head=%.2f "
                "speed=%.2f term=%.2f\n",
                p.weight_contour_error, p.weight_lag_error,
                p.weight_heading_error, p.weight_speed_error,
                p.weight_terminal);
    std::printf("[MPC] MORAI brake mapping: brake=1.0 <-> %.2f m/s^2\n",
                p.brake_full_decel);
    std::printf("[MPC] Calibrated nonlinear pedal mapping: %s\n",
                p.use_calibrated_pedal_mapping ? "enabled" : "disabled");
}

