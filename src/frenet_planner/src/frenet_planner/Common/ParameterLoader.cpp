#include "frenet_planner/Common/ParameterLoader.hpp"

#include <cstdio>

#include <yaml-cpp/yaml.h>

namespace {

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
    out = node.as<T>();
    return true;
}

}  // namespace

void LoadParams(const std::string& yaml_path,
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
                 std::string& waypoint_file) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        std::printf("[FrenetPlanner] Failed to load yaml '%s': %s (기본값으로 진행)\n",
                    yaml_path.c_str(), e.what());
        return;
    }

    loadInto(root, "planner/path_generator/lateral_d1/min",   path_cfg.lateral_d1.min);
    loadInto(root, "planner/path_generator/lateral_d1/max",   path_cfg.lateral_d1.max);
    loadInto(root, "planner/path_generator/lateral_d1/step",  path_cfg.lateral_d1.step);
    loadInto(root, "planner/path_generator/lateral_target_tolerance", path_cfg.lateral_target_tolerance);
    loadInto(root, "planner/path_generator/time_horizon/min", path_cfg.time_horizon.min);
    loadInto(root, "planner/path_generator/time_horizon/max", path_cfg.time_horizon.max);
    loadInto(root, "planner/path_generator/time_horizon/step",path_cfg.time_horizon.step);
    loadInto(root, "planner/path_generator/delta_s/min",      path_cfg.delta_s.min);
    loadInto(root, "planner/path_generator/delta_s/max",      path_cfg.delta_s.max);
    loadInto(root, "planner/path_generator/delta_s/step",     path_cfg.delta_s.step);
    loadInto(root, "planner/path_generator/delta_s_dot/min",  path_cfg.delta_s_dot.min);
    loadInto(root, "planner/path_generator/delta_s_dot/max",  path_cfg.delta_s_dot.max);
    loadInto(root, "planner/path_generator/delta_s_dot/step", path_cfg.delta_s_dot.step);
    loadInto(root, "planner/path_generator/dt",               path_cfg.dt);

    loadInto(root, "planner/kinematic_limits/max_lateral_accel",      limits.max_lateral_accel);
    loadInto(root, "planner/kinematic_limits/max_longitudinal_accel", limits.max_longitudinal_accel);
    loadInto(root, "planner/kinematic_limits/max_curvature",          limits.max_curvature);
    loadInto(root, "planner/kinematic_limits/max_longitudinal_jerk",  limits.max_longitudinal_jerk);

    loadInto(root, "planner/cost_weights/kj",     cost_weights.kj);
    loadInto(root, "planner/cost_weights/kt",     cost_weights.kt);
    loadInto(root, "planner/cost_weights/kd",     cost_weights.kd);
    loadInto(root, "planner/cost_weights/kd_path", cost_weights.kd_path);
    loadInto(root, "planner/cost_weights/ks",     cost_weights.ks);
    loadInto(root, "planner/cost_weights/ks_dot", cost_weights.ks_dot);
    loadInto(root, "planner/cost_weights/klat",   cost_weights.klat);
    loadInto(root, "planner/cost_weights/klon",   cost_weights.klon);

    loadInto(root, "planner/vehicle_shape/width",  vehicle_shape.width);
    loadInto(root, "planner/vehicle_shape/length", vehicle_shape.length);

    loadInto(root, "planner/collision_check/safety_margin",      collision_cfg.safety_margin);
    loadInto(root, "planner/collision_check/margin_growth_rate", collision_cfg.margin_growth_rate);
    loadInto(root, "planner/collision_check/reactive_lookahead", collision_cfg.reactive_lookahead);

    loadInto(root, "planner/curve_speed/target_vel",       curve_speed_cfg.target_vel);
    loadInto(root, "planner/curve_speed/curve_vel_sharp",  curve_speed_cfg.curve_vel_sharp);
    loadInto(root, "planner/curve_speed/curve_vel_mid",    curve_speed_cfg.curve_vel_mid);
    loadInto(root, "planner/curve_speed/curve_vel_mild",   curve_speed_cfg.curve_vel_mild);
    loadInto(root, "planner/curve_speed/curve_th_sharp",   curve_speed_cfg.curve_th_sharp);
    loadInto(root, "planner/curve_speed/curve_th_mid",     curve_speed_cfg.curve_th_mid);
    loadInto(root, "planner/curve_speed/curve_th_mild",    curve_speed_cfg.curve_th_mild);
    loadInto(root, "planner/curve_speed/curve_lookahead_m",curve_speed_cfg.curve_lookahead_m);

    loadInto(root, "planner/following/time_gap",             following_cfg.time_gap);
    loadInto(root, "planner/following/min_gap",              following_cfg.min_gap);
    loadInto(root, "planner/following/gap_gain",             following_cfg.gap_gain);
    loadInto(root, "planner/following/max_leader_search_s",  following_cfg.max_leader_search_s);
    loadInto(root, "planner/following/exit_search_margin",   following_cfg.exit_search_margin);
    loadInto(root, "planner/following/dropout_grace_cycles", following_cfg.dropout_grace_cycles);

    loadInto(root, "planner/avoid/detection_distance", avoid_cfg.detection_distance);
    loadInto(root, "planner/avoid/avoid_offset",      avoid_cfg.avoid_offset);
    loadInto(root, "planner/avoid/min_shift_start_distance",
             avoid_cfg.min_shift_start_distance);
    loadInto(root, "planner/avoid/max_shift_start_distance",
             avoid_cfg.max_shift_start_distance);
    loadInto(root, "planner/avoid/shift_time_margin",
             avoid_cfg.shift_time_margin);
    loadInto(root, "planner/avoid/lateral_tolerance", avoid_cfg.lateral_tolerance);
    loadInto(root, "planner/avoid/rear_overhang",     avoid_cfg.rear_overhang);
    loadInto(root, "planner/avoid/pass_clearance",    avoid_cfg.pass_clearance);
    loadInto(root, "planner/avoid/prefer_right_when_equal", avoid_cfg.prefer_right_when_equal);
    // D6-1. 키가 없으면 기본값(5)이 유지되므로 기존 params.yaml도 그대로 쓴다.
    loadInto(root, "planner/avoid/dropout_grace_cycles", avoid_cfg.dropout_grace_cycles);

    // D7. 키가 없으면 LaneChangeConfig 기본값이 유지된다.
    loadInto(root, "planner/lane_change/lateral_position_tolerance",
             lane_change_cfg.lateral_position_tolerance);
    loadInto(root, "planner/lane_change/lateral_speed_tolerance",
             lane_change_cfg.lateral_speed_tolerance);
    loadInto(root, "planner/lane_change/heading_tolerance",
             lane_change_cfg.heading_tolerance);

    loadInto(root, "planner/stop/comfortable_decel",  stop_cfg.comfortable_decel);
    loadInto(root, "planner/stop/braking_margin",     stop_cfg.braking_margin);
    loadInto(root, "planner/stop/stop_buffer",        stop_cfg.stop_buffer);
    loadInto(root, "planner/stop/position_tolerance", stop_cfg.position_tolerance);
    loadInto(root, "planner/stop/speed_tolerance",    stop_cfg.speed_tolerance);
    loadInto(root, "planner/stop/max_lateral_offset", stop_cfg.max_lateral_offset);
    loadInto(root, "planner/stop/target_change_tolerance",
             stop_cfg.target_change_tolerance);
    loadInto(root, "planner/stop/projection_search_window",
             stop_cfg.projection_search_window);
    loadInto(root, "planner/emergency_stop/decel", emergency_stop_cfg.decel);

    loadInto(root, "planner/merge/conflict_s",             merge_cfg.conflict_s);
    loadInto(root, "planner/merge/approach_distance",      merge_cfg.approach_distance);
    loadInto(root, "planner/merge/stop_buffer",            merge_cfg.stop_buffer);
    loadInto(root, "planner/merge/conflict_radius",        merge_cfg.conflict_radius);
    loadInto(root, "planner/merge/min_object_speed",       merge_cfg.min_object_speed);
    loadInto(root, "planner/merge/min_front_time_gap",     merge_cfg.min_front_time_gap);
    loadInto(root, "planner/merge/min_rear_time_gap",      merge_cfg.min_rear_time_gap);
    loadInto(root, "planner/merge/ego_clear_time",         merge_cfg.ego_clear_time);
    loadInto(root, "planner/merge/max_wait_time",          merge_cfg.max_wait_time);
    loadInto(root, "planner/merge/completion_distance",    merge_cfg.completion_distance);
    loadInto(root, "planner/merge/commit_distance",        merge_cfg.commit_distance);
    loadInto(root, "planner/merge/safe_confirm_cycles",    merge_cfg.safe_confirm_cycles);
    loadInto(root, "planner/merge/cross_speed_floor",      merge_cfg.cross_speed_floor);

    loadInto(root, "planner/highway_merge/enabled",                    highway_merge_cfg.enabled);
    loadInto(root, "planner/highway_merge/approach_distance",          highway_merge_cfg.approach_distance);
    loadInto(root, "planner/highway_merge/stop_buffer",                highway_merge_cfg.stop_buffer);
    loadInto(root, "planner/highway_merge/target_corridor_half_width", highway_merge_cfg.target_corridor_half_width);
    loadInto(root, "planner/highway_merge/object_search_distance",     highway_merge_cfg.object_search_distance);
    loadInto(root, "planner/highway_merge/min_heading_alignment",      highway_merge_cfg.min_heading_alignment);
    loadInto(root, "planner/highway_merge/min_front_gap",              highway_merge_cfg.min_front_gap);
    loadInto(root, "planner/highway_merge/min_rear_gap",               highway_merge_cfg.min_rear_gap);
    loadInto(root, "planner/highway_merge/min_front_ttc",              highway_merge_cfg.min_front_ttc);
    loadInto(root, "planner/highway_merge/min_rear_ttc",               highway_merge_cfg.min_rear_ttc);
    loadInto(root, "planner/highway_merge/max_wait_time",              highway_merge_cfg.max_wait_time);
    loadInto(root, "planner/highway_merge/commit_distance",            highway_merge_cfg.commit_distance);
    loadInto(root, "planner/highway_merge/safe_confirm_cycles",        highway_merge_cfg.safe_confirm_cycles);
    loadInto(root, "planner/highway_merge/cross_speed_floor",          highway_merge_cfg.cross_speed_floor);
    highway_merge_cfg.zones.clear();
    const YAML::Node highway_zones = root["planner"]["highway_merge"]["zones"];
    if (highway_zones && highway_zones.IsSequence()) {
        for (const auto& node : highway_zones) {
            HighwayMergeZone zone;
            zone.name = node["name"] ? node["name"].as<std::string>()
                                     : "highway_merge";
            zone.start_s = node["start_s"].as<double>();
            zone.completion_s = node["completion_s"].as<double>();
            const YAML::Node checkpoints = node["checkpoints"];
            if (checkpoints && checkpoints.IsSequence()) {
                for (const auto& checkpoint_node : checkpoints) {
                    HighwayMergeCheckpoint checkpoint;
                    checkpoint.conflict_s = checkpoint_node["conflict_s"].as<double>();
                    checkpoint.clear_s = checkpoint_node["clear_s"].as<double>();
                    zone.checkpoints.push_back(checkpoint);
                }
            }
            highway_merge_cfg.zones.push_back(zone);
        }
    }

    loadInto(root, "planner/visualization/enabled",       visualization_cfg.enabled);
    loadInto(root, "planner/visualization/publish_hz",    visualization_cfg.publish_hz);
    loadInto(root, "planner/visualization/snapshot_path", visualization_cfg.snapshot_path);

    loadInto(root, "planner/wheelbase",   wheelbase);
    loadInto(root, "planner/lane_width",  lane_width);
    loadInto(root, "planner/replanning_period_s",    replanning_period_s);
    loadInto(root, "planner/max_target_speed_accel", max_target_speed_accel);
    loadInto(root, "waypoint_file",       waypoint_file);
}
