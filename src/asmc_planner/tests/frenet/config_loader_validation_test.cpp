// Strict Config Loading 회귀(Rule 39): params.yaml의 필수 key가 없거나,
// YAML 타입이 잘못됐거나, NaN/Inf/물리적으로 말이 안 되는 값이면 반드시
// FrenetPlanner::Init()이 실패해야 한다. C++ default로 조용히 진행되는
// 경로가 남아 있으면 안 된다.
#include "Frenet/frenet_planner.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

// source(params.yaml)를 읽어 replace_from -> replace_to 치환 후
// out_path에 쓰고 그 경로를 반환한다. waypoint_file은 항상 절대경로로
// 덮어써서 FRENET_PACKAGE_SRC_DIR 상대경로 계산에 영향받지 않게 한다.
std::string MakeMutatedConfig(const std::string& source,
                               const std::string& waypoints,
                               const std::string& replace_from,
                               const std::string& replace_to,
                               const std::string& out_path) {
    std::ifstream in(source);
    if (!in) return {};
    std::ostringstream buffer;
    buffer << in.rdbuf();
    std::string content = buffer.str();

    if (!replace_from.empty()) {
        const std::size_t pos = content.find(replace_from);
        if (pos == std::string::npos) return {};
        content.replace(pos, replace_from.size(), replace_to);
    }

    std::ofstream out(out_path);
    if (!out) return {};
    std::istringstream lines(content);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.rfind("waypoint_file:", 0) == 0) {
            out << "waypoint_file: \"" << waypoints << "\"\n";
        } else {
            out << line << '\n';
        }
    }
    return out_path;
}

}  // namespace

int main() {
    const std::string package_src = FRENET_PACKAGE_SRC_DIR;
    const std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    const std::string source = package_src + "/../config/frenet/params.yaml";

    // --- production params.yaml: load + validate가 그대로 PASS해야 한다 ---
    {
        const std::string config = MakeMutatedConfig(
            source, waypoint_file, "", "", "config_loader_validation_baseline.yaml");
        Expect(!config.empty(), "baseline: local config copy created");
        FrenetPlanner planner;
        Expect(planner.Init(config),
               "baseline: production params.yaml loads and validates");
    }

    // --- 필수 key 누락: planner/path_generator/dt 라인 자체를 제거 ---
    {
        std::ifstream in(source);
        std::ostringstream buffer;
        buffer << in.rdbuf();
        std::string content = buffer.str();
        const std::string needle = "    dt: 0.1\n";
        const std::size_t pos = content.find(needle);
        Expect(pos != std::string::npos, "missing key: dt line found in source");
        content.erase(pos, needle.size());

        std::ofstream out("config_loader_validation_missing_key.yaml");
        std::istringstream lines(content);
        std::string line;
        while (std::getline(lines, line)) {
            if (line.rfind("waypoint_file:", 0) == 0) {
                out << "waypoint_file: \"" << waypoint_file << "\"\n";
            } else {
                out << line << '\n';
            }
        }
        out.close();

        FrenetPlanner planner;
        Expect(!planner.Init("config_loader_validation_missing_key.yaml"),
               "missing key: Init fails when planner/path_generator/dt is absent");
    }

    // --- YAML 타입 오류: dt를 문자열로 ---
    {
        const std::string config = MakeMutatedConfig(
            source, waypoint_file, "dt: 0.1", "dt: \"not_a_number\"",
            "config_loader_validation_wrong_type.yaml");
        Expect(!config.empty(), "wrong type: local config copy created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),
               "wrong type: Init fails when dt is a string instead of a number");
    }

    // --- NaN: max_lateral_accel ---
    {
        const std::string config = MakeMutatedConfig(
            source, waypoint_file, "max_lateral_accel: 2.6", "max_lateral_accel: .nan",
            "config_loader_validation_nan.yaml");
        Expect(!config.empty(), "NaN: local config copy created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),
               "NaN: Init fails when max_lateral_accel is NaN");
    }

    // --- Inf: max_curvature ---
    {
        const std::string config = MakeMutatedConfig(
            source, waypoint_file, "max_curvature: 0.23", "max_curvature: .inf",
            "config_loader_validation_inf.yaml");
        Expect(!config.empty(), "Inf: local config copy created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),
               "Inf: Init fails when max_curvature is Inf");
    }

    // --- invalid sampling range: time_horizon min > max ---
    {
        const std::string config = MakeMutatedConfig(
            source, waypoint_file,
            "time_horizon:   { min: 1.5,  max: 5.0, step: 0.5 }",
            "time_horizon:   { min: 9.0,  max: 5.0, step: 0.5 }",
            "config_loader_validation_bad_range.yaml");
        Expect(!config.empty(), "invalid range: local config copy created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),
               "invalid range: Init fails when time_horizon.min > max");
    }

    // --- negative dt ---
    {
        const std::string config = MakeMutatedConfig(
            source, waypoint_file, "dt: 0.1", "dt: -0.1",
            "config_loader_validation_negative_dt.yaml");
        Expect(!config.empty(), "negative dt: local config copy created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),
               "negative dt: Init fails when path_generator.dt <= 0");
    }

    // --- invalid physical limit: max_service_decel < comfortable_decel ---
    {
        const std::string config = MakeMutatedConfig(
            source, waypoint_file, "max_service_decel: 5.0", "max_service_decel: 0.5",
            "config_loader_validation_bad_decel.yaml");
        Expect(!config.empty(), "invalid physical limit: local config copy created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),
               "invalid physical limit: Init fails when max_service_decel < comfortable_decel");
    }

    // --- invalid sampling step: delta_s step <= 0 ---
    {
        const std::string config = MakeMutatedConfig(
            source, waypoint_file,
            "delta_s:        { min: -3.0, max: 3.0, step: 1.0 }",
            "delta_s:        { min: -3.0, max: 3.0, step: 0.0 }",
            "config_loader_validation_bad_step.yaml");
        Expect(!config.empty(), "invalid step: local config copy created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),
               "invalid step: Init fails when delta_s.step <= 0");
    }

    // Cruise profile is required and strictly validated even when disabled.
    for (const std::string value : {".nan", ".inf", "-1.0", "0.0", "bad"}) {
        const auto config = MakeMutatedConfig(
            source, waypoint_file, "speed_error_weight: 4.0",
            "speed_error_weight: " + value, "config_loader_bad_cruise.yaml");
        Expect(!config.empty(), "cruise: mutation created");
        FrenetPlanner planner;
        Expect(!planner.Init(config), "cruise: invalid speed cost rejected");
    }
    {
        const auto config = MakeMutatedConfig(
            source, waypoint_file, "    speed_error_weight: 4.0\n", "",
            "config_loader_missing_cruise.yaml");
        Expect(!config.empty(), "cruise: missing-key mutation created");
        FrenetPlanner planner;
        Expect(!planner.Init(config), "cruise: missing cost rejected");
    }

    for (const std::string value : {".nan", ".inf", "1.8", "0.0", "bad"}) {
        const auto config=MakeMutatedConfig(source,waypoint_file,
            "road_outer_width: 2.161", "road_outer_width: "+value,
            "config_loader_bad_road_exterior.yaml");
        Expect(!config.empty(),"road exterior: mutation created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),"road exterior: invalid or undersized width rejected");
    }
    {
        const auto config=MakeMutatedConfig(source,waypoint_file,
            "    road_outer_width: 2.161\n", "", "config_loader_missing_road_exterior.yaml");
        Expect(!config.empty(),"road exterior: missing-key mutation created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),"road exterior: missing calibration rejected");
    }
    for (const auto& mutation : {
            std::pair<std::string,std::string>{"curvature_gain: 0.844","curvature_gain: .nan"},
            {"max_rate: 1.0","max_rate: 0.0"},
            {"reference_speed: 5.0","reference_speed: -1.0"},
            {"min_rate: 0.1","min_rate: 2.0"},
            {"    curvature_gain: 0.844\n",""}}) {
        const auto config=MakeMutatedConfig(source,waypoint_file,mutation.first,mutation.second,
            "config_loader_bad_steering_profile.yaml");
        Expect(!config.empty(),"steering contract: mutation created");
        FrenetPlanner planner;
        Expect(!planner.Init(config),"steering contract: invalid or missing actuator input rejected");
    }
    if (g_failures > 0) {
        std::cerr << g_failures << " config validation check(s) failed\n";
        return 1;
    }
    return 0;
}
