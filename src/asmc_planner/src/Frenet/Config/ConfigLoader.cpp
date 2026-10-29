#include "Frenet/Config/ConfigLoader.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>

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
                 ObjectTrackingConfig& object_tracking_cfg,
                 PathGeneratorConfig& path_cfg,
                 KinematicLimits& limits,
                 CostWeights& cost_weights,
                 KeepCruiseConfig& keep_cruise_cfg,
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

    REQUIRE("planner/object_tracking/yaw_rate_clamp_rad_s",     object_tracking_cfg.yaw_rate_clamp_rad_s);
    REQUIRE("planner/object_tracking/yaw_rate_smoothing_alpha", object_tracking_cfg.yaw_rate_smoothing_alpha);
    REQUIRE("planner/object_tracking/missing_cycles_eviction",  object_tracking_cfg.missing_cycles_eviction);
    // Preserve existing user YAMLs; the committed YAML explicitly sets this.
    object_tracking_cfg.min_motion_direction_change_rad = .05;
    if(root["planner"]["object_tracking"]["min_motion_direction_change_rad"])
        REQUIRE("planner/object_tracking/min_motion_direction_change_rad", object_tracking_cfg.min_motion_direction_change_rad);

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
    REQUIRE("planner/steering_profile/curvature_gain", limits.steering.curvature_gain);
    REQUIRE("planner/steering_profile/max_rate", limits.steering.max_rate);
    REQUIRE("planner/steering_profile/reference_speed", limits.steering.reference_speed);
    REQUIRE("planner/steering_profile/min_rate", limits.steering.min_rate);
    REQUIRE("planner/kinematic_limits/max_longitudinal_jerk",  limits.max_longitudinal_jerk);
    REQUIRE("planner/kinematic_limits/low_speed_fallback_v",       limits.low_speed_fallback_v);
    REQUIRE("planner/kinematic_limits/low_speed_crawl_duration_s", limits.low_speed_crawl_duration_s);
    REQUIRE("planner/kinematic_limits/near_low_speed_fallback_v",  limits.near_low_speed_fallback_v);
    REQUIRE("planner/kinematic_limits/low_speed_usable_curvature_fraction",
            limits.low_speed_usable_curvature_fraction);
    REQUIRE("planner/kinematic_limits/low_speed_transition_distance_margin",
            limits.low_speed_transition_distance_margin);
    REQUIRE("planner/kinematic_limits/boundary_condition_accel_fraction",
            limits.boundary_condition_accel_fraction);
    REQUIRE("planner/kinematic_limits/avoid_shift_usable_lateral_accel_fraction",
            limits.avoid_shift_usable_lateral_accel_fraction);
    REQUIRE("planner/kinematic_limits/max_initial_d_dot", limits.max_initial_d_dot);
    REQUIRE("planner/kinematic_limits/low_speed_crawl_accel",      limits.low_speed_crawl_accel);
    REQUIRE("planner/kinematic_limits/low_speed_min_creep_speed",  limits.low_speed_min_creep_speed);
    REQUIRE("planner/kinematic_limits/low_speed_crawl_speed_cap",  limits.low_speed_crawl_speed_cap);
    REQUIRE("planner/kinematic_limits/accel_intent_deadband_mps",  limits.accel_intent_deadband_mps);

    REQUIRE("planner/keep_cruise/enabled", keep_cruise_cfg.enabled);
    REQUIRE("planner/keep_cruise/source_links", keep_cruise_cfg.source_links);
    REQUIRE("planner/keep_cruise/min_requested_speed", keep_cruise_cfg.min_requested_speed);
    REQUIRE("planner/keep_cruise/max_lateral_accel", keep_cruise_cfg.max_lateral_accel);
    REQUIRE("planner/keep_cruise/speed_error_weight", keep_cruise_cfg.speed_error_weight);

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
    REQUIRE("planner/vehicle_shape/road_outer_width", vehicle_shape.road_outer_width);
    // Older profiles contain only the calibrated total. New profiles expose
    // the separate per-side reserve without changing any existing caller.
    const auto vehicle_node = root["planner"]["vehicle_shape"];
    if (ok && vehicle_node["boundary_reserve_each_side_m"]) {
        double reserve = 0.;
        if (!loadInto(root, "planner/vehicle_shape/boundary_reserve_each_side_m", reserve) ||
            !std::isfinite(reserve) || reserve < 0. ||
            std::abs(vehicle_shape.width + 2.*reserve - vehicle_shape.road_outer_width) > 1e-8) {
            error_message = "planner/vehicle_shape boundary reserve conflicts with physical/outer width";
            return false;
        }
    }

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

    if (root["planner"]["avoid"]["map_search_budget_s"]) {
        REQUIRE("planner/avoid/map_search_budget_s", avoid_cfg.map_search_budget_s);
    }
    REQUIRE("planner/avoid/detection_distance",         avoid_cfg.detection_distance);
    if (root["planner"]["avoid"]["bypass_speed_cap"]) {
        REQUIRE("planner/avoid/bypass_speed_cap", avoid_cfg.bypass_speed_cap);
    } else {
        avoid_cfg.bypass_speed_cap = limits.low_speed_crawl_speed_cap;
    }
    REQUIRE("planner/avoid/avoid_offset",               avoid_cfg.avoid_offset);
    REQUIRE("planner/avoid/direction_extra_margin",      avoid_cfg.direction_extra_margin);
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
    REQUIRE("planner/avoid/static_dropout_grace_cycles", avoid_cfg.static_dropout_grace_cycles);
    REQUIRE("planner/avoid/committed_target_index_cost_scale",
            avoid_cfg.committed_target_index_cost_scale);
    REQUIRE("planner/avoid/direction_tiebreak_cost",    avoid_cfg.direction_tiebreak_cost);
    REQUIRE("planner/avoid/spatial_reconnect_enabled",  avoid_cfg.spatial_reconnect_enabled);
    REQUIRE("planner/avoid/reconnect_max_longitudinal_delta",
            avoid_cfg.reconnect_max_longitudinal_delta);
    REQUIRE("planner/avoid/reconnect_max_lateral_delta",
            avoid_cfg.reconnect_max_lateral_delta);
    REQUIRE("planner/avoid/reconnect_max_size_delta", avoid_cfg.reconnect_max_size_delta);

    REQUIRE("planner/lane_change/lateral_position_tolerance", lane_change_cfg.lateral_position_tolerance);
    REQUIRE("planner/lane_change/lateral_speed_tolerance",     lane_change_cfg.lateral_speed_tolerance);
    REQUIRE("planner/lane_change/heading_tolerance",           lane_change_cfg.heading_tolerance);

    REQUIRE("planner/stop/comfortable_decel",       stop_cfg.comfortable_decel);
    if (root["planner"]["stop"]["reapproach_speed"])
        REQUIRE("planner/stop/reapproach_speed", stop_cfg.reapproach_speed);
    if (root["planner"]["stop"]["reapproach_accel"])
        REQUIRE("planner/stop/reapproach_accel", stop_cfg.reapproach_accel);
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
    REQUIRE("planner/stop/delta_s_override/min",  stop_cfg.delta_s_override.min);
    REQUIRE("planner/stop/delta_s_override/max",  stop_cfg.delta_s_override.max);
    REQUIRE("planner/stop/delta_s_override/step", stop_cfg.delta_s_override.step);

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
    REQUIRE("planner/merge/braking_distance_margin", merge_cfg.braking_distance_margin);
    REQUIRE("planner/merge/min_arrival_speed",       merge_cfg.min_arrival_speed);
    REQUIRE("planner/merge/prediction_dt",           merge_cfg.prediction_dt);
    REQUIRE("planner/merge/entry_search_dt",         merge_cfg.entry_search_dt);
    REQUIRE("planner/merge/min_travel_speed",        merge_cfg.min_travel_speed);
    REQUIRE("planner/merge/min_travel_accel",        merge_cfg.min_travel_accel);
    REQUIRE("planner/merge/arrival_time_match_tolerance",
            merge_cfg.arrival_time_match_tolerance);
    REQUIRE("planner/merge/min_entry_time_for_arrival_speed",
            merge_cfg.min_entry_time_for_arrival_speed);

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
    REQUIRE("planner/highway_merge/entry_search_dt",            highway_merge_cfg.entry_search_dt);
    REQUIRE("planner/highway_merge/collision_check_dt",         highway_merge_cfg.collision_check_dt);
    REQUIRE("planner/highway_merge/min_travel_speed",           highway_merge_cfg.min_travel_speed);
    REQUIRE("planner/highway_merge/min_travel_accel",           highway_merge_cfg.min_travel_accel);

#undef REQUIRE

    if (!ok) {
        error_message = "missing or invalid required key: " + missing_key;
        return false;
    }

    const auto highway_policy=root["planner"]["highway_merge"];
    const HighwayMergeConfig highway_defaults{};
    try {
        highway_merge_cfg.curve_lateral_accel_safety_factor =
            highway_policy["curve_lateral_accel_safety_factor"].as<double>(
                highway_defaults.curve_lateral_accel_safety_factor);
        highway_merge_cfg.curve_comfortable_decel_factor =
            highway_policy["curve_comfortable_decel_factor"].as<double>(
                highway_defaults.curve_comfortable_decel_factor);
    } catch (const std::exception&) {
        error_message="invalid planner/highway_merge curve policy";return false;
    }
    try {highway_merge_cfg.speed_cap_mps=highway_policy["speed_cap_mps"]
        ?highway_policy["speed_cap_mps"].as<double>():highway_defaults.speed_cap_mps;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/speed_cap_mps";return false;}
    try {highway_merge_cfg.forward_accel_mps2=
        highway_policy["forward_accel_mps2"].as<double>(highway_defaults.forward_accel_mps2);}
    catch(const std::exception&){error_message="invalid planner/highway_merge/forward_accel_mps2";return false;}
    try {highway_merge_cfg.max_horizon_s=highway_policy["max_horizon_s"]
        ?highway_policy["max_horizon_s"].as<double>():highway_defaults.max_horizon_s;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/max_horizon_s";return false;}
    try {highway_merge_cfg.post_merge_horizon_s=highway_policy["post_merge_horizon_s"]
        ?highway_policy["post_merge_horizon_s"].as<double>():highway_defaults.post_merge_horizon_s;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/post_merge_horizon_s";return false;}
    try {highway_merge_cfg.source_max_age_s=highway_policy["source_max_age_s"]
        ?highway_policy["source_max_age_s"].as<double>():highway_defaults.source_max_age_s;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/source_max_age_s";return false;}
    try {highway_merge_cfg.confirmation_gap_s=highway_policy["confirmation_gap_s"]
        ?highway_policy["confirmation_gap_s"].as<double>():highway_defaults.confirmation_gap_s;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/confirmation_gap_s";return false;}
    try {
        const auto tracking=highway_policy["transition_tracking"];
        if(tracking) {
            highway_merge_cfg.transition_tracking_position_m=tracking["position_m"].as<double>(
                highway_defaults.transition_tracking_position_m);
            highway_merge_cfg.transition_tracking_heading_rad=tracking["heading_rad"].as<double>(
                highway_defaults.transition_tracking_heading_rad);
            highway_merge_cfg.transition_tracking_curvature_inv_m=tracking["curvature_inv_m"].as<double>(
                highway_defaults.transition_tracking_curvature_inv_m);
        }
    } catch(const std::exception&) {error_message="invalid planner/highway_merge/transition_tracking";return false;}
    try {highway_merge_cfg.speed_error_mps=highway_policy["speed_error_mps"]
        ?highway_policy["speed_error_mps"].as<double>():highway_defaults.speed_error_mps;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/speed_error_mps";return false;}
    try {
        highway_merge_cfg.speed_error_floor_mps=highway_policy["speed_error_floor_mps"].as<double>(highway_defaults.speed_error_floor_mps);
        highway_merge_cfg.speed_error_correlation_s=highway_policy["speed_error_correlation_s"].as<double>(highway_defaults.speed_error_correlation_s);
        highway_merge_cfg.motion_fit_min_span_s=highway_policy["motion_fit_min_span_s"].as<double>(highway_defaults.motion_fit_min_span_s);
        highway_merge_cfg.provisional_lateral_speed_mps=highway_policy["provisional_lateral_speed_mps"].as<double>(highway_defaults.provisional_lateral_speed_mps);
    } catch(const std::exception&) {error_message="invalid planner/highway_merge motion uncertainty policy";return false;}
    try {highway_merge_cfg.max_rear_decel_mps2=highway_policy["max_rear_decel_mps2"]
        ?highway_policy["max_rear_decel_mps2"].as<double>():highway_defaults.max_rear_decel_mps2;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/max_rear_decel_mps2";return false;}
    try {highway_merge_cfg.completion_settle_s=highway_policy["completion_settle_s"]
        ?highway_policy["completion_settle_s"].as<double>():highway_defaults.completion_settle_s;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/completion_settle_s";return false;}
    try {highway_merge_cfg.completion_heading_rad=highway_policy["completion_heading_rad"]
        ?highway_policy["completion_heading_rad"].as<double>():highway_defaults.completion_heading_rad;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/completion_heading_rad";return false;}
    try {highway_merge_cfg.completion_lateral_m=highway_policy["completion_lateral_m"]
        ?highway_policy["completion_lateral_m"].as<double>():highway_defaults.completion_lateral_m;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/completion_lateral_m";return false;}
    try {highway_merge_cfg.collision_max_step_m=highway_policy["collision_max_step_m"]
        ?highway_policy["collision_max_step_m"].as<double>():highway_defaults.collision_max_step_m;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/collision_max_step_m";return false;}
    try {highway_merge_cfg.max_collision_samples=highway_policy["max_collision_samples"]
        ?highway_policy["max_collision_samples"].as<int>():highway_defaults.max_collision_samples;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/max_collision_samples";return false;}

    try {highway_merge_cfg.planning_budget_ms=highway_policy["planning_budget_ms"]
        ?highway_policy["planning_budget_ms"].as<double>():highway_defaults.planning_budget_ms;}
    catch(const std::exception&){error_message="invalid planner/highway_merge/planning_budget_ms";return false;}

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
                zone.stop_buffer = node["stop_buffer"]
                    ? node["stop_buffer"].as<double>() : highway_merge_cfg.stop_buffer;
                zone.hold_unsafe_gap_until_conflict = node["hold_unsafe_gap_until_conflict"]
                    ? node["hold_unsafe_gap_until_conflict"].as<bool>() : false;
            } catch (const std::exception&) {
                error_message =
                    "planner/highway_merge/zones start_s/completion_s/stop_buffer/hold flag invalid";
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
            zone.source_link=node["source_link"].as<std::string>("");
            if(node["target_lanes"]) {
                if(!node["target_lanes"].IsSequence()) {error_message="highway target_lanes must be a sequence";return false;}
                try {zone.target_lanes=node["target_lanes"].as<std::vector<std::string>>();}
                catch(const std::exception&) {error_message="highway target_lanes must contain lane IDs";return false;}
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

