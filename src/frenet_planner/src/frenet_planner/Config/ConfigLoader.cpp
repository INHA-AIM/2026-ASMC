#include "frenet_planner/Config/ConfigLoader.hpp"

#include <yaml-cpp/yaml.h>

namespace {

// dotted_key: "planner/avoid/detection_distance" 같은 '/' 구분 경로.
// 누락되거나 YAML 타입이 T와 맞지 않으면 false를 반환하고 out은 건드리지
// 않는다 - caller가 이 실패를 무시하면 안 된다(LoadParams가 강제한다).
template <typename T>
bool loadInto(const YAML::Node& root, const std::string& dotted_key, T& out) {
    YAML::Node node = YAML::Clone(root);
    size_t start = 0;
    while (true) {
        size_t slash = dotted_key.find('/', start);
        std::string key = dotted_key.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
        if (!node[key]) return false;
        node = node[key];
        if (slash == std::string::npos) break;
        start = slash + 1;
    }
    try {
        out = node.as<T>();
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

}  // namespace

bool LoadParams(const std::string& yaml_path,
                 ReferenceConfig& reference_cfg,
                 PathGeneratorConfig& path_cfg,
                 KinematicLimits& limits,
                 CostWeights& cost_weights,
                 VehicleShape& vehicle_shape,
                 CollisionCheckConfig& collision_cfg,
                 CurveSpeedConfig& curve_speed_cfg,
                 FollowingConfig& following_cfg,
                 AvoidConfig& avoid_cfg,
                 LaneChangeConfig& lane_change_cfg,
                 StopConfig& stop_cfg,
                 EmergencyStopConfig& emergency_stop_cfg,
                 MergeConfig& merge_cfg,
                 HighwayMergeConfig& highway_merge_cfg,
                 PlannerVisualizationConfig& visualization_cfg,
                 double& wheelbase,
                 double& lane_width,
                 double& replanning_period_s,
                 double& max_target_speed_accel,
                 std::string& waypoint_file,
                 std::string& error_message) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        error_message = "failed to load yaml '" + yaml_path + "': " + e.what();
        return false;
    }

    // 실패한 첫 key만 기록하고 이후 호출은 건너뛴다 - 어떤 필드도 조용히
    // 이전 값에 머무르지 않는다(Parameter Single-Source, strict loading).
    std::string missing_key;
    bool ok = true;
    auto require = [&](const std::string& key, auto&& load_call) {
        if (!ok) return;
        if (!load_call()) {
            ok = false;
            missing_key = key;
        }
    };

#define REQUIRE(key, field) \
    require((key), [&]() { return loadInto(root, (key), (field)); })

    REQUIRE("planner/reference/closure_tolerance_m",            reference_cfg.closure_tolerance_m);
    REQUIRE("planner/reference/smoothing_window_m",              reference_cfg.smoothing_window_m);
    REQUIRE("planner/reference/resample_ds",                     reference_cfg.resample_ds);
    REQUIRE("planner/reference/relax_max_iter",                  reference_cfg.relax_max_iter);
    REQUIRE("planner/reference/relax_step",                      reference_cfg.relax_step);
    REQUIRE("planner/reference/ego_projection_search_window",    reference_cfg.ego_projection_search_window);

    REQUIRE("planner/path_generator/lateral_d1/min",   path_cfg.lateral_d1.min);
    REQUIRE("planner/path_generator/lateral_d1/max",   path_cfg.lateral_d1.max);
    REQUIRE("planner/path_generator/lateral_d1/step",  path_cfg.lateral_d1.step);
    REQUIRE("planner/path_generator/lateral_target_tolerance", path_cfg.lateral_target_tolerance);
    REQUIRE("planner/path_generator/time_horizon/min", path_cfg.time_horizon.min);
    REQUIRE("planner/path_generator/time_horizon/max", path_cfg.time_horizon.max);
    REQUIRE("planner/path_generator/time_horizon/step",path_cfg.time_horizon.step);
    REQUIRE("planner/path_generator/delta_s/min",      path_cfg.delta_s.min);
    REQUIRE("planner/path_generator/delta_s/max",      path_cfg.delta_s.max);
    REQUIRE("planner/path_generator/delta_s/step",     path_cfg.delta_s.step);
    REQUIRE("planner/path_generator/delta_s_dot/min",  path_cfg.delta_s_dot.min);
    REQUIRE("planner/path_generator/delta_s_dot/max",  path_cfg.delta_s_dot.max);
    REQUIRE("planner/path_generator/delta_s_dot/step", path_cfg.delta_s_dot.step);
    REQUIRE("planner/path_generator/dt",               path_cfg.dt);

    REQUIRE("planner/kinematic_limits/max_lateral_accel",      limits.max_lateral_accel);
    REQUIRE("planner/kinematic_limits/max_longitudinal_accel", limits.max_longitudinal_accel);
    REQUIRE("planner/kinematic_limits/max_curvature",          limits.max_curvature);
    REQUIRE("planner/kinematic_limits/max_longitudinal_jerk",  limits.max_longitudinal_jerk);

    REQUIRE("planner/cost_weights/kj",     cost_weights.kj);
    REQUIRE("planner/cost_weights/kt",     cost_weights.kt);
    REQUIRE("planner/cost_weights/kd",     cost_weights.kd);
    REQUIRE("planner/cost_weights/kd_path", cost_weights.kd_path);
    REQUIRE("planner/cost_weights/ks",     cost_weights.ks);
    REQUIRE("planner/cost_weights/ks_dot", cost_weights.ks_dot);
    REQUIRE("planner/cost_weights/klat",   cost_weights.klat);
    REQUIRE("planner/cost_weights/klon",   cost_weights.klon);

    REQUIRE("planner/vehicle_shape/width",  vehicle_shape.width);
    REQUIRE("planner/vehicle_shape/length", vehicle_shape.length);
    REQUIRE("planner/vehicle_shape/rear_overhang", vehicle_shape.rear_overhang);

    REQUIRE("planner/collision_check/safety_margin",      collision_cfg.safety_margin);
    REQUIRE("planner/collision_check/margin_growth_rate", collision_cfg.margin_growth_rate);
    REQUIRE("planner/collision_check/reactive_lookahead", collision_cfg.reactive_lookahead);

    REQUIRE("planner/curve_speed/target_vel",        curve_speed_cfg.target_vel);
    REQUIRE("planner/curve_speed/curve_lookahead_m", curve_speed_cfg.curve_lookahead_m);
    REQUIRE("planner/curve_speed/lateral_accel_safety_factor", curve_speed_cfg.lateral_accel_safety_factor);
    REQUIRE("planner/curve_speed/comfortable_decel_factor",    curve_speed_cfg.comfortable_decel_factor);
    REQUIRE("planner/curve_speed/min_usable_decel",            curve_speed_cfg.min_usable_decel);

    REQUIRE("planner/following/time_gap",             following_cfg.time_gap);
    REQUIRE("planner/following/min_gap",              following_cfg.min_gap);
    REQUIRE("planner/following/gap_gain",             following_cfg.gap_gain);
    REQUIRE("planner/following/max_leader_search_s",  following_cfg.max_leader_search_s);
    REQUIRE("planner/following/exit_search_margin",   following_cfg.exit_search_margin);
    REQUIRE("planner/following/dropout_grace_cycles", following_cfg.dropout_grace_cycles);

    REQUIRE("planner/avoid/detection_distance",         avoid_cfg.detection_distance);
    REQUIRE("planner/avoid/avoid_offset",               avoid_cfg.avoid_offset);
    REQUIRE("planner/avoid/min_shift_start_distance",   avoid_cfg.min_shift_start_distance);
    REQUIRE("planner/avoid/max_shift_start_distance",   avoid_cfg.max_shift_start_distance);
    REQUIRE("planner/avoid/shift_time_margin",          avoid_cfg.shift_time_margin);
    REQUIRE("planner/avoid/shift_start_distance_offset", avoid_cfg.shift_start_distance_offset);
    REQUIRE("planner/avoid/lateral_tolerance",          avoid_cfg.lateral_tolerance);
    REQUIRE("planner/avoid/rear_overhang",              avoid_cfg.rear_overhang);
    REQUIRE("planner/avoid/pass_clearance",             avoid_cfg.pass_clearance);
    REQUIRE("planner/avoid/prefer_right_when_equal",    avoid_cfg.prefer_right_when_equal);
    REQUIRE("planner/avoid/committed_recovery_step",    avoid_cfg.committed_recovery_step);
    REQUIRE("planner/avoid/committed_recovery_max_expansion", avoid_cfg.committed_recovery_max_expansion);
    REQUIRE("planner/avoid/low_speed_transition_scale_min",  avoid_cfg.low_speed_transition_scale_min);
    REQUIRE("planner/avoid/low_speed_transition_scale_max",  avoid_cfg.low_speed_transition_scale_max);
    REQUIRE("planner/avoid/low_speed_transition_scale_step", avoid_cfg.low_speed_transition_scale_step);
    REQUIRE("planner/avoid/low_speed_path_extension",   avoid_cfg.low_speed_path_extension);
    REQUIRE("planner/avoid/dropout_grace_cycles",       avoid_cfg.dropout_grace_cycles);

    REQUIRE("planner/lane_change/lateral_position_tolerance", lane_change_cfg.lateral_position_tolerance);
    REQUIRE("planner/lane_change/lateral_speed_tolerance",     lane_change_cfg.lateral_speed_tolerance);
    REQUIRE("planner/lane_change/heading_tolerance",           lane_change_cfg.heading_tolerance);

    REQUIRE("planner/stop/comfortable_decel",       stop_cfg.comfortable_decel);
    REQUIRE("planner/stop/max_service_decel",       stop_cfg.max_service_decel);
    REQUIRE("planner/stop/braking_margin",          stop_cfg.braking_margin);
    REQUIRE("planner/stop/stop_buffer",             stop_cfg.stop_buffer);
    REQUIRE("planner/stop/position_tolerance",      stop_cfg.position_tolerance);
    REQUIRE("planner/stop/speed_tolerance",         stop_cfg.speed_tolerance);
    REQUIRE("planner/stop/lateral_hold_speed",      stop_cfg.lateral_hold_speed);
    REQUIRE("planner/stop/lateral_error_tolerance", stop_cfg.lateral_error_tolerance);
    REQUIRE("planner/stop/max_lateral_offset",      stop_cfg.max_lateral_offset);
    REQUIRE("planner/stop/target_change_tolerance", stop_cfg.target_change_tolerance);
    REQUIRE("planner/stop/projection_search_window", stop_cfg.projection_search_window);
    REQUIRE("planner/stop/min_decel", stop_cfg.min_decel);

    REQUIRE("planner/emergency_stop/decel", emergency_stop_cfg.decel);

    REQUIRE("planner/merge/conflict_s",          merge_cfg.conflict_s);
    REQUIRE("planner/merge/approach_distance",   merge_cfg.approach_distance);
    REQUIRE("planner/merge/stop_buffer",         merge_cfg.stop_buffer);
    REQUIRE("planner/merge/conflict_radius",     merge_cfg.conflict_radius);
    REQUIRE("planner/merge/min_object_speed",    merge_cfg.min_object_speed);
    REQUIRE("planner/merge/min_front_time_gap",  merge_cfg.min_front_time_gap);
    REQUIRE("planner/merge/min_rear_time_gap",   merge_cfg.min_rear_time_gap);
    REQUIRE("planner/merge/ego_clear_time",      merge_cfg.ego_clear_time);
    REQUIRE("planner/merge/max_wait_time",       merge_cfg.max_wait_time);
    REQUIRE("planner/merge/completion_distance", merge_cfg.completion_distance);
    REQUIRE("planner/merge/commit_distance",     merge_cfg.commit_distance);
    REQUIRE("planner/merge/safe_confirm_cycles", merge_cfg.safe_confirm_cycles);
    REQUIRE("planner/merge/cross_speed_floor",   merge_cfg.cross_speed_floor);

    REQUIRE("planner/highway_merge/enabled",                    highway_merge_cfg.enabled);
    REQUIRE("planner/highway_merge/approach_distance",          highway_merge_cfg.approach_distance);
    REQUIRE("planner/highway_merge/stop_buffer",                highway_merge_cfg.stop_buffer);
    REQUIRE("planner/highway_merge/target_corridor_half_width", highway_merge_cfg.target_corridor_half_width);
    REQUIRE("planner/highway_merge/object_search_distance",     highway_merge_cfg.object_search_distance);
    REQUIRE("planner/highway_merge/min_heading_alignment",      highway_merge_cfg.min_heading_alignment);
    REQUIRE("planner/highway_merge/min_front_gap",              highway_merge_cfg.min_front_gap);
    REQUIRE("planner/highway_merge/min_rear_gap",               highway_merge_cfg.min_rear_gap);
    REQUIRE("planner/highway_merge/min_front_ttc",              highway_merge_cfg.min_front_ttc);
    REQUIRE("planner/highway_merge/min_rear_ttc",               highway_merge_cfg.min_rear_ttc);
    REQUIRE("planner/highway_merge/max_wait_time",              highway_merge_cfg.max_wait_time);
    REQUIRE("planner/highway_merge/commit_distance",            highway_merge_cfg.commit_distance);
    REQUIRE("planner/highway_merge/safe_confirm_cycles",        highway_merge_cfg.safe_confirm_cycles);
    REQUIRE("planner/highway_merge/cross_speed_floor",          highway_merge_cfg.cross_speed_floor);

#undef REQUIRE

    if (!ok) {
        error_message = "missing or invalid required key: " + missing_key;
        return false;
    }

    highway_merge_cfg.zones.clear();
    const YAML::Node highway_zones = root["planner"]["highway_merge"]["zones"];
    if (highway_zones && !highway_zones.IsSequence()) {
        error_message = "planner/highway_merge/zones must be a sequence";
        return false;
    }
    if (highway_zones && highway_zones.IsSequence()) {
        for (const auto& node : highway_zones) {
            HighwayMergeZone zone;
            zone.name = node["name"] ? node["name"].as<std::string>()
                                     : "highway_merge";
            if (!node["start_s"] || !node["completion_s"]) {
                error_message =
                    "planner/highway_merge/zones entry missing start_s/completion_s";
                return false;
            }
            try {
                zone.start_s = node["start_s"].as<double>();
                zone.completion_s = node["completion_s"].as<double>();
            } catch (const std::exception&) {
                error_message =
                    "planner/highway_merge/zones start_s/completion_s must be numeric";
                return false;
            }
            const YAML::Node checkpoints = node["checkpoints"];
            if (checkpoints && !checkpoints.IsSequence()) {
                error_message =
                    "planner/highway_merge/zones checkpoints must be a sequence";
                return false;
            }
            if (checkpoints && checkpoints.IsSequence()) {
                for (const auto& checkpoint_node : checkpoints) {
                    if (!checkpoint_node["conflict_s"] || !checkpoint_node["clear_s"]) {
                        error_message =
                            "planner/highway_merge/zones checkpoint missing "
                            "conflict_s/clear_s";
                        return false;
                    }
                    HighwayMergeCheckpoint checkpoint;
                    try {
                        checkpoint.conflict_s = checkpoint_node["conflict_s"].as<double>();
                        checkpoint.clear_s = checkpoint_node["clear_s"].as<double>();
                    } catch (const std::exception&) {
                        error_message =
                            "planner/highway_merge/zones checkpoint conflict_s/clear_s "
                            "must be numeric";
                        return false;
                    }
                    zone.checkpoints.push_back(checkpoint);
                }
            }
            highway_merge_cfg.zones.push_back(zone);
        }
    }

    ok = true;
#define REQUIRE(key, field) \
    require((key), [&]() { return loadInto(root, (key), (field)); })

    REQUIRE("planner/visualization/enabled",       visualization_cfg.enabled);
    REQUIRE("planner/visualization/publish_hz",    visualization_cfg.publish_hz);
    REQUIRE("planner/visualization/snapshot_path", visualization_cfg.snapshot_path);

    REQUIRE("planner/wheelbase",              wheelbase);
    REQUIRE("planner/lane_width",             lane_width);
    REQUIRE("planner/replanning_period_s",    replanning_period_s);
    REQUIRE("planner/max_target_speed_accel", max_target_speed_accel);
    REQUIRE("waypoint_file",                  waypoint_file);

#undef REQUIRE

    if (!ok) {
        error_message = "missing or invalid required key: " + missing_key;
        return false;
    }

    return true;
}
