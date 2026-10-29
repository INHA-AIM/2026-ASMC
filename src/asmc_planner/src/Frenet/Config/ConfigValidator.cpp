#include "Frenet/Config/ConfigValidator.hpp"

#include <cmath>
#include <algorithm>

namespace {

bool IsFinite(double v) { return std::isfinite(v); }

bool CheckRange(const SamplingRange& range, const std::string& name,
                 std::string& error_message) {
    if (!IsFinite(range.min) || !IsFinite(range.max) || !IsFinite(range.step)) {
        error_message = name + ": min/max/step must be finite";
        return false;
    }
    if (range.min > range.max) {
        error_message = name + ": min must be <= max";
        return false;
    }
    if (range.step <= 0.0) {
        error_message = name + ": step must be > 0";
        return false;
    }
    return true;
}

bool CheckPositiveFinite(double value, const std::string& name,
                          std::string& error_message) {
    if (!IsFinite(value) || value <= 0.0) {
        error_message = name + " must be > 0";
        return false;
    }
    return true;
}

bool CheckFinite(double value, const std::string& name,
                  std::string& error_message) {
    if (!IsFinite(value)) {
        error_message = name + " must be finite";
        return false;
    }
    return true;
}

}  // namespace

bool ValidateConfig(const ReferenceConfig& reference_cfg,
                     const ObjectTrackingConfig& object_tracking_cfg,
                     const PathGeneratorConfig& path_cfg,
                     const KinematicLimits& limits,
                     const CostWeights& cost_weights,
                     const KeepCruiseConfig& keep_cruise_cfg,
                     const VehicleShape& vehicle_shape,
                     const CollisionCheckConfig& collision_cfg,
                     const CurveSpeedConfig& curve_speed_cfg,
                     const FollowingConfig& following_cfg,
                     const AvoidConfig& avoid_cfg,
                     const LaneChangeConfig& lane_change_cfg,
                     const StopConfig& stop_cfg,
                     const EmergencyStopConfig& emergency_stop_cfg,
                     const MergeConfig& merge_cfg,
                     const HighwayMergeConfig& highway_merge_cfg,
                     const PlannerVisualizationConfig& visualization_cfg,
                     double wheelbase,
                     double lane_width,
                     double replanning_period_s,
                     double max_target_speed_accel,
                     std::string& error_message) {
    // --- reference ---
    if (!CheckPositiveFinite(reference_cfg.closure_tolerance_m,
                              "planner/reference/closure_tolerance_m", error_message))
        return false;
    if (!CheckPositiveFinite(reference_cfg.smoothing_window_m,
                              "planner/reference/smoothing_window_m", error_message))
        return false;
    if (!CheckPositiveFinite(reference_cfg.resample_ds,
                              "planner/reference/resample_ds", error_message))
        return false;
    if (reference_cfg.relax_max_iter <= 0) {
        error_message = "planner/reference/relax_max_iter must be > 0";
        return false;
    }
    if (!IsFinite(reference_cfg.relax_step) || reference_cfg.relax_step <= 0.0 ||
        reference_cfg.relax_step > 1.0) {
        error_message = "planner/reference/relax_step must be in (0, 1]";
        return false;
    }
    if (!CheckPositiveFinite(reference_cfg.ego_projection_search_window,
                              "planner/reference/ego_projection_search_window",
                              error_message))
        return false;

    // --- object_tracking ---
    if (!CheckPositiveFinite(object_tracking_cfg.yaw_rate_clamp_rad_s,
                              "planner/object_tracking/yaw_rate_clamp_rad_s",
                              error_message))
        return false;
    if (!IsFinite(object_tracking_cfg.yaw_rate_smoothing_alpha) ||
        object_tracking_cfg.yaw_rate_smoothing_alpha <= 0.0 ||
        object_tracking_cfg.yaw_rate_smoothing_alpha > 1.0) {
        error_message =
            "planner/object_tracking/yaw_rate_smoothing_alpha must be in (0, 1]";
        return false;
    }
    if (object_tracking_cfg.missing_cycles_eviction <= 0) {
        error_message = "planner/object_tracking/missing_cycles_eviction must be > 0";
        return false;
    }

    if (!CheckPositiveFinite(object_tracking_cfg.min_motion_direction_change_rad,
                              "planner/object_tracking/min_motion_direction_change_rad", error_message) ||
        object_tracking_cfg.min_motion_direction_change_rad > 3.141592653589793) {
        error_message="planner/object_tracking/min_motion_direction_change_rad must be in (0, pi]";
        return false;
    }

    // --- path_generator ---
    if (!CheckRange(path_cfg.lateral_d1, "planner/path_generator/lateral_d1", error_message))
        return false;
    if (!CheckRange(path_cfg.time_horizon, "planner/path_generator/time_horizon", error_message))
        return false;
    if (!CheckRange(path_cfg.delta_s, "planner/path_generator/delta_s", error_message))
        return false;
    if (!CheckRange(path_cfg.delta_s_dot, "planner/path_generator/delta_s_dot", error_message))
        return false;
    if (!CheckPositiveFinite(path_cfg.dt, "planner/path_generator/dt", error_message))
        return false;
    if (!CheckPositiveFinite(path_cfg.lateral_target_tolerance,
                              "planner/path_generator/lateral_target_tolerance", error_message))
        return false;

    // Validate disabled profiles as well: enabling one must not expose bad data.
    if (keep_cruise_cfg.source_links.empty() ||
        std::any_of(keep_cruise_cfg.source_links.begin(), keep_cruise_cfg.source_links.end(),
                    [](const std::string& link) { return link.empty(); })) {
        error_message = "planner/keep_cruise/source_links must contain nonempty road IDs";
        return false;
    }
    if (!CheckPositiveFinite(keep_cruise_cfg.min_requested_speed,
                              "planner/keep_cruise/min_requested_speed", error_message) ||
        !CheckPositiveFinite(keep_cruise_cfg.max_lateral_accel,
                              "planner/keep_cruise/max_lateral_accel", error_message) ||
        !CheckPositiveFinite(keep_cruise_cfg.speed_error_weight,
                              "planner/keep_cruise/speed_error_weight", error_message))
        return false;

    // --- kinematic_limits ---
    if (!CheckPositiveFinite(limits.steering.curvature_gain,"planner/steering_profile/curvature_gain",error_message) ||
        !CheckPositiveFinite(limits.steering.max_rate,"planner/steering_profile/max_rate",error_message) ||
        !CheckPositiveFinite(limits.steering.min_rate,"planner/steering_profile/min_rate",error_message))return false;
    if(!IsFinite(limits.steering.reference_speed) || limits.steering.reference_speed<0 ||
       limits.steering.min_rate>limits.steering.max_rate) {
        error_message="planner/steering_profile requires reference_speed >= 0 and min_rate <= max_rate";
        return false;
    }
    if (!CheckPositiveFinite(limits.max_lateral_accel, "planner/kinematic_limits/max_lateral_accel", error_message))
        return false;
    if (!CheckPositiveFinite(limits.max_longitudinal_accel, "planner/kinematic_limits/max_longitudinal_accel", error_message))
        return false;
    if (!CheckPositiveFinite(limits.max_curvature, "planner/kinematic_limits/max_curvature", error_message))
        return false;
    if (!CheckPositiveFinite(limits.max_longitudinal_jerk, "planner/kinematic_limits/max_longitudinal_jerk", error_message))
        return false;
    if (!CheckPositiveFinite(limits.low_speed_fallback_v,
                              "planner/kinematic_limits/low_speed_fallback_v", error_message))
        return false;
    if (!CheckPositiveFinite(limits.low_speed_crawl_duration_s,
                              "planner/kinematic_limits/low_speed_crawl_duration_s", error_message))
        return false;
    if (!IsFinite(limits.near_low_speed_fallback_v) ||
        limits.near_low_speed_fallback_v < limits.low_speed_fallback_v) {
        error_message =
            "planner/kinematic_limits/near_low_speed_fallback_v must be finite "
            "and >= low_speed_fallback_v";
        return false;
    }
    if (!IsFinite(limits.low_speed_usable_curvature_fraction) ||
        limits.low_speed_usable_curvature_fraction <= 0.0 ||
        limits.low_speed_usable_curvature_fraction > 1.0) {
        error_message =
            "planner/kinematic_limits/low_speed_usable_curvature_fraction must be in (0, 1]";
        return false;
    }
    if (!CheckPositiveFinite(limits.low_speed_transition_distance_margin,
                              "planner/kinematic_limits/low_speed_transition_distance_margin",
                              error_message))
        return false;
    if (!IsFinite(limits.boundary_condition_accel_fraction) ||
        limits.boundary_condition_accel_fraction <= 0.0 ||
        limits.boundary_condition_accel_fraction > 1.0) {
        error_message =
            "planner/kinematic_limits/boundary_condition_accel_fraction must be in (0, 1]";
        return false;
    }
    if (!IsFinite(limits.avoid_shift_usable_lateral_accel_fraction) ||
        limits.avoid_shift_usable_lateral_accel_fraction <= 0.0 ||
        limits.avoid_shift_usable_lateral_accel_fraction > 1.0) {
        error_message =
            "planner/kinematic_limits/avoid_shift_usable_lateral_accel_fraction must be in (0, 1]";
        return false;
    }
    if (!CheckPositiveFinite(limits.max_initial_d_dot,
                              "planner/kinematic_limits/max_initial_d_dot", error_message))
        return false;
    if (!CheckPositiveFinite(limits.low_speed_crawl_accel,
                              "planner/kinematic_limits/low_speed_crawl_accel", error_message))
        return false;
    if (!CheckPositiveFinite(limits.low_speed_min_creep_speed,
                              "planner/kinematic_limits/low_speed_min_creep_speed", error_message))
        return false;
    if (!IsFinite(limits.low_speed_crawl_speed_cap) ||
        limits.low_speed_crawl_speed_cap < limits.low_speed_min_creep_speed) {
        error_message =
            "planner/kinematic_limits/low_speed_crawl_speed_cap must be finite "
            "and >= low_speed_min_creep_speed";
        return false;
    }
    if (!CheckPositiveFinite(limits.accel_intent_deadband_mps,
                              "planner/kinematic_limits/accel_intent_deadband_mps", error_message))
        return false;

    // --- cost_weights: 논문/설계상 음수 가중치는 의미가 없다(비용을 보상으로
    // 뒤집는 것과 같다). 0은 "이 항을 끈다"는 뜻으로 허용한다.
    for (const auto& w : {cost_weights.kj, cost_weights.kt, cost_weights.kd,
                           cost_weights.kd_path, cost_weights.ks, cost_weights.ks_dot,
                           cost_weights.klat, cost_weights.klon}) {
        if (!IsFinite(w) || w < 0.0) {
            error_message = "planner/cost_weights/* must all be finite and >= 0";
            return false;
        }
    }

    // --- vehicle_shape ---
    if (!CheckPositiveFinite(vehicle_shape.width, "planner/vehicle_shape/width", error_message))
        return false;
    if (!CheckPositiveFinite(vehicle_shape.length, "planner/vehicle_shape/length", error_message))
        return false;
    if (!CheckPositiveFinite(vehicle_shape.road_outer_width,
            "planner/vehicle_shape/road_outer_width", error_message)) return false;
    if (vehicle_shape.road_outer_width < vehicle_shape.width) {
        error_message = "planner/vehicle_shape/road_outer_width must be >= width";
        return false;
    }
    if (!IsFinite(vehicle_shape.rear_overhang) ||
        (vehicle_shape.rear_overhang >= 0.0 && vehicle_shape.rear_overhang > vehicle_shape.length)) {
        error_message =
            "planner/vehicle_shape/rear_overhang must be finite and, if >= 0, <= length";
        return false;
    }

    // --- collision_check ---
    if (!IsFinite(collision_cfg.safety_margin) || collision_cfg.safety_margin < 0.0) {
        error_message = "planner/collision_check/safety_margin must be finite and >= 0";
        return false;
    }
    if (!IsFinite(collision_cfg.margin_growth_rate) || collision_cfg.margin_growth_rate < 0.0) {
        error_message = "planner/collision_check/margin_growth_rate must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(collision_cfg.reactive_lookahead, "planner/collision_check/reactive_lookahead", error_message))
        return false;

    // --- curve_speed ---
    if (!CheckPositiveFinite(curve_speed_cfg.target_vel, "planner/curve_speed/target_vel", error_message))
        return false;
    if (!IsFinite(curve_speed_cfg.curve_lookahead_m) || curve_speed_cfg.curve_lookahead_m < 0.0) {
        error_message = "planner/curve_speed/curve_lookahead_m must be finite and >= 0";
        return false;
    }
    if (!IsFinite(curve_speed_cfg.lateral_accel_safety_factor) ||
        curve_speed_cfg.lateral_accel_safety_factor <= 0.0 ||
        curve_speed_cfg.lateral_accel_safety_factor > 1.0) {
        error_message = "planner/curve_speed/lateral_accel_safety_factor must be in (0, 1]";
        return false;
    }
    if (!IsFinite(curve_speed_cfg.comfortable_decel_factor) ||
        curve_speed_cfg.comfortable_decel_factor <= 0.0 ||
        curve_speed_cfg.comfortable_decel_factor > 1.0) {
        error_message = "planner/curve_speed/comfortable_decel_factor must be in (0, 1]";
        return false;
    }
    if (!CheckPositiveFinite(curve_speed_cfg.min_usable_decel,
                              "planner/curve_speed/min_usable_decel", error_message))
        return false;

    // --- following ---
    if (!IsFinite(following_cfg.time_gap) || following_cfg.time_gap < 0.0) {
        error_message = "planner/following/time_gap must be finite and >= 0";
        return false;
    }
    if (!IsFinite(following_cfg.min_gap) || following_cfg.min_gap < 0.0) {
        error_message = "planner/following/min_gap must be finite and >= 0";
        return false;
    }
    if (!CheckFinite(following_cfg.gap_gain, "planner/following/gap_gain", error_message))
        return false;
    if (!CheckPositiveFinite(following_cfg.max_leader_search_s, "planner/following/max_leader_search_s", error_message))
        return false;
    if (!IsFinite(following_cfg.exit_search_margin) || following_cfg.exit_search_margin < 0.0) {
        error_message = "planner/following/exit_search_margin must be finite and >= 0";
        return false;
    }
    if (following_cfg.dropout_grace_cycles < 0) {
        error_message = "planner/following/dropout_grace_cycles must be >= 0";
        return false;
    }

    // --- avoid ---
    if(!CheckPositiveFinite(avoid_cfg.static_recovery_speed_cap,"planner/avoid/static_recovery_speed_cap",error_message)||
       !CheckPositiveFinite(avoid_cfg.static_contact_speed_cap,"planner/avoid/static_contact_speed_cap",error_message))return false;
    if(avoid_cfg.static_contact_speed_cap>avoid_cfg.static_recovery_speed_cap) {
        error_message="static recovery caps must satisfy contact <= recovery";return false;
    }
    if(!IsFinite(avoid_cfg.bypass_accel_cap)||avoid_cfg.bypass_accel_cap<=0) {
        error_message="planner/avoid/bypass_accel_cap must be finite and positive";
        return false;
    }
    if (!IsFinite(avoid_cfg.map_search_budget_s) || avoid_cfg.map_search_budget_s <= 0.0 ||
        avoid_cfg.map_search_budget_s > 0.15) {
        error_message = "planner/avoid/map_search_budget_s must be finite and in (0, 0.15]";
        return false;
    }
    if (!CheckPositiveFinite(avoid_cfg.detection_distance, "planner/avoid/detection_distance", error_message))
        return false;
    if (!IsFinite(avoid_cfg.bypass_speed_cap) ||
        avoid_cfg.bypass_speed_cap < limits.low_speed_min_creep_speed) {
        error_message = "planner/avoid/bypass_speed_cap must be finite and >= low_speed_min_creep_speed";
        return false;
    }
    if (!IsFinite(avoid_cfg.avoid_offset) || avoid_cfg.avoid_offset < 0.0) {
        error_message = "planner/avoid/avoid_offset must be finite and >= 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.direction_extra_margin) || avoid_cfg.direction_extra_margin < 0.0) {
        error_message = "planner/avoid/direction_extra_margin must be finite and >= 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.min_shift_start_distance) || avoid_cfg.min_shift_start_distance < 0.0) {
        error_message = "planner/avoid/min_shift_start_distance must be finite and >= 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.max_shift_start_distance) ||
        avoid_cfg.max_shift_start_distance < avoid_cfg.min_shift_start_distance) {
        error_message =
            "planner/avoid/max_shift_start_distance must be finite and >= min_shift_start_distance";
        return false;
    }
    if (!IsFinite(avoid_cfg.shift_time_margin) || avoid_cfg.shift_time_margin < 0.0) {
        error_message = "planner/avoid/shift_time_margin must be finite and >= 0";
        return false;
    }
    if (!CheckFinite(avoid_cfg.shift_start_distance_offset,
                      "planner/avoid/shift_start_distance_offset", error_message))
        return false;
    if (!IsFinite(avoid_cfg.lateral_tolerance) || avoid_cfg.lateral_tolerance <= 0.0) {
        error_message = "planner/avoid/lateral_tolerance must be finite and > 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.rear_overhang) ||
        (avoid_cfg.rear_overhang >= 0.0 && avoid_cfg.rear_overhang > vehicle_shape.length)) {
        error_message = "planner/avoid/rear_overhang must be finite and, if >= 0, <= vehicle length";
        return false;
    }
    if (!IsFinite(avoid_cfg.pass_clearance) || avoid_cfg.pass_clearance < 0.0) {
        error_message = "planner/avoid/pass_clearance must be finite and >= 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.committed_recovery_step) || avoid_cfg.committed_recovery_step < 0.0) {
        error_message = "planner/avoid/committed_recovery_step must be finite and >= 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.committed_recovery_max_expansion) ||
        avoid_cfg.committed_recovery_max_expansion < 0.0) {
        error_message = "planner/avoid/committed_recovery_max_expansion must be finite and >= 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.low_speed_transition_scale_min) ||
        avoid_cfg.low_speed_transition_scale_min <= 0.0) {
        error_message = "planner/avoid/low_speed_transition_scale_min must be finite and > 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.low_speed_transition_scale_max) ||
        avoid_cfg.low_speed_transition_scale_max < avoid_cfg.low_speed_transition_scale_min) {
        error_message =
            "planner/avoid/low_speed_transition_scale_max must be finite and >= "
            "low_speed_transition_scale_min";
        return false;
    }
    if (!IsFinite(avoid_cfg.low_speed_transition_scale_step) ||
        avoid_cfg.low_speed_transition_scale_step <= 0.0) {
        error_message = "planner/avoid/low_speed_transition_scale_step must be finite and > 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.low_speed_path_extension) || avoid_cfg.low_speed_path_extension < 0.0) {
        error_message = "planner/avoid/low_speed_path_extension must be finite and >= 0";
        return false;
    }
    if (avoid_cfg.dropout_grace_cycles < 0) {
        error_message = "planner/avoid/dropout_grace_cycles must be >= 0";
        return false;
    }
    if (avoid_cfg.static_dropout_grace_cycles < avoid_cfg.dropout_grace_cycles) {
        error_message = "planner/avoid/static_dropout_grace_cycles must be >= dropout_grace_cycles";
        return false;
    }
    if (!CheckPositiveFinite(avoid_cfg.committed_target_index_cost_scale,
                              "planner/avoid/committed_target_index_cost_scale", error_message))
        return false;
    if (!CheckPositiveFinite(avoid_cfg.direction_tiebreak_cost,
                              "planner/avoid/direction_tiebreak_cost", error_message))
        return false;
    if (!IsFinite(avoid_cfg.reconnect_max_longitudinal_delta) ||
        avoid_cfg.reconnect_max_longitudinal_delta < 0.0) {
        error_message = "planner/avoid/reconnect_max_longitudinal_delta must be finite and >= 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.reconnect_max_lateral_delta) ||
        avoid_cfg.reconnect_max_lateral_delta < 0.0) {
        error_message = "planner/avoid/reconnect_max_lateral_delta must be finite and >= 0";
        return false;
    }
    if (!IsFinite(avoid_cfg.reconnect_max_size_delta) ||
        avoid_cfg.reconnect_max_size_delta < 0.0) {
        error_message = "planner/avoid/reconnect_max_size_delta must be finite and >= 0";
        return false;
    }

    // --- lane_change ---
    if (!IsFinite(lane_change_cfg.lateral_position_tolerance) ||
        lane_change_cfg.lateral_position_tolerance <= 0.0) {
        error_message = "planner/lane_change/lateral_position_tolerance must be finite and > 0";
        return false;
    }
    if (!IsFinite(lane_change_cfg.lateral_speed_tolerance) ||
        lane_change_cfg.lateral_speed_tolerance <= 0.0) {
        error_message = "planner/lane_change/lateral_speed_tolerance must be finite and > 0";
        return false;
    }
    if (!IsFinite(lane_change_cfg.heading_tolerance) || lane_change_cfg.heading_tolerance <= 0.0) {
        error_message = "planner/lane_change/heading_tolerance must be finite and > 0";
        return false;
    }

    // --- stop ---
    if (!CheckPositiveFinite(stop_cfg.comfortable_decel, "planner/stop/comfortable_decel", error_message))
        return false;
    if (!CheckPositiveFinite(stop_cfg.reapproach_speed, "planner/stop/reapproach_speed", error_message) ||
        !CheckPositiveFinite(stop_cfg.reapproach_accel, "planner/stop/reapproach_accel", error_message))
        return false;
    if (!IsFinite(stop_cfg.max_service_decel) ||
        stop_cfg.max_service_decel < stop_cfg.comfortable_decel) {
        error_message = "planner/stop/max_service_decel must be finite and >= comfortable_decel";
        return false;
    }
    if (!IsFinite(stop_cfg.braking_margin) || stop_cfg.braking_margin < 0.0) {
        error_message = "planner/stop/braking_margin must be finite and >= 0";
        return false;
    }
    if (!IsFinite(stop_cfg.stop_buffer) || stop_cfg.stop_buffer < 0.0) {
        error_message = "planner/stop/stop_buffer must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(stop_cfg.position_tolerance, "planner/stop/position_tolerance", error_message))
        return false;
    if (!CheckPositiveFinite(stop_cfg.speed_tolerance, "planner/stop/speed_tolerance", error_message))
        return false;
    if (!IsFinite(stop_cfg.lateral_hold_speed) || stop_cfg.lateral_hold_speed < 0.0) {
        error_message = "planner/stop/lateral_hold_speed must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(stop_cfg.lateral_error_tolerance,
                              "planner/stop/lateral_error_tolerance", error_message))
        return false;
    if (!CheckPositiveFinite(stop_cfg.max_lateral_offset, "planner/stop/max_lateral_offset", error_message))
        return false;
    if (!IsFinite(stop_cfg.target_change_tolerance) || stop_cfg.target_change_tolerance < 0.0) {
        error_message = "planner/stop/target_change_tolerance must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(stop_cfg.projection_search_window,
                              "planner/stop/projection_search_window", error_message))
        return false;
    if (!CheckPositiveFinite(stop_cfg.min_decel, "planner/stop/min_decel", error_message))
        return false;
    if (!CheckRange(stop_cfg.delta_s_override, "planner/stop/delta_s_override", error_message))
        return false;
    if (stop_cfg.delta_s_override.max > 0.0) {
        error_message =
            "planner/stop/delta_s_override/max must be <= 0 (STOP candidates must "
            "not overshoot the stop target)";
        return false;
    }

    // --- emergency_stop ---
    if (!CheckPositiveFinite(emergency_stop_cfg.decel, "planner/emergency_stop/decel", error_message))
        return false;

    // --- merge (roundabout). conflict_s < 0 means disabled - only positivity
    // is required for everything else so a disabled MERGE doesn't force
    // meaningless geometry to be tuned. ---
    if (!CheckFinite(merge_cfg.conflict_s, "planner/merge/conflict_s", error_message))
        return false;
    if (!CheckPositiveFinite(merge_cfg.approach_distance, "planner/merge/approach_distance", error_message))
        return false;
    if (!IsFinite(merge_cfg.stop_buffer) || merge_cfg.stop_buffer < 0.0) {
        error_message = "planner/merge/stop_buffer must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(merge_cfg.conflict_radius, "planner/merge/conflict_radius", error_message))
        return false;
    if (!IsFinite(merge_cfg.min_object_speed) || merge_cfg.min_object_speed < 0.0) {
        error_message = "planner/merge/min_object_speed must be finite and >= 0";
        return false;
    }
    if (!IsFinite(merge_cfg.min_front_time_gap) || merge_cfg.min_front_time_gap < 0.0) {
        error_message = "planner/merge/min_front_time_gap must be finite and >= 0";
        return false;
    }
    if (!IsFinite(merge_cfg.min_rear_time_gap) || merge_cfg.min_rear_time_gap < 0.0) {
        error_message = "planner/merge/min_rear_time_gap must be finite and >= 0";
        return false;
    }
    if (!IsFinite(merge_cfg.ego_clear_time) || merge_cfg.ego_clear_time < 0.0) {
        error_message = "planner/merge/ego_clear_time must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(merge_cfg.max_wait_time, "planner/merge/max_wait_time", error_message))
        return false;
    if (!IsFinite(merge_cfg.completion_distance) || merge_cfg.completion_distance < 0.0) {
        error_message = "planner/merge/completion_distance must be finite and >= 0";
        return false;
    }
    if (!IsFinite(merge_cfg.commit_distance) || merge_cfg.commit_distance < 0.0) {
        error_message = "planner/merge/commit_distance must be finite and >= 0";
        return false;
    }
    if (merge_cfg.safe_confirm_cycles < 0) {
        error_message = "planner/merge/safe_confirm_cycles must be >= 0";
        return false;
    }
    if (!IsFinite(merge_cfg.cross_speed_floor) || merge_cfg.cross_speed_floor < 0.0) {
        error_message = "planner/merge/cross_speed_floor must be finite and >= 0";
        return false;
    }
    if (!IsFinite(merge_cfg.braking_distance_margin) || merge_cfg.braking_distance_margin < 0.0) {
        error_message = "planner/merge/braking_distance_margin must be finite and >= 0";
        return false;
    }
    if (!IsFinite(merge_cfg.min_arrival_speed) || merge_cfg.min_arrival_speed < 0.0) {
        error_message = "planner/merge/min_arrival_speed must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(merge_cfg.prediction_dt, "planner/merge/prediction_dt", error_message))
        return false;
    if (!CheckPositiveFinite(merge_cfg.entry_search_dt, "planner/merge/entry_search_dt", error_message))
        return false;
    if (!CheckPositiveFinite(merge_cfg.min_travel_speed, "planner/merge/min_travel_speed", error_message))
        return false;
    if (!CheckPositiveFinite(merge_cfg.min_travel_accel, "planner/merge/min_travel_accel", error_message))
        return false;
    if (!CheckPositiveFinite(merge_cfg.arrival_time_match_tolerance,
                              "planner/merge/arrival_time_match_tolerance", error_message))
        return false;
    if (!CheckPositiveFinite(merge_cfg.min_entry_time_for_arrival_speed,
                              "planner/merge/min_entry_time_for_arrival_speed", error_message))
        return false;

    // --- highway_merge. Disabled configs (enabled=false) still get their
    // scalar fields range-checked - a garbage value shouldn't hide behind
    // "it's disabled anyway". Zone s-ordering is already checked by
    // FrenetPlanner::Init() against the loaded reference line, since that
    // check needs ref_ (not available here). ---
    if (!CheckPositiveFinite(highway_merge_cfg.approach_distance,
                              "planner/highway_merge/approach_distance", error_message))
        return false;
    if (!IsFinite(highway_merge_cfg.stop_buffer) || highway_merge_cfg.stop_buffer < 0.0) {
        error_message = "planner/highway_merge/stop_buffer must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(highway_merge_cfg.target_corridor_half_width,
                              "planner/highway_merge/target_corridor_half_width", error_message))
        return false;
    if (!CheckPositiveFinite(highway_merge_cfg.object_search_distance,
                              "planner/highway_merge/object_search_distance", error_message))
        return false;
    if (!IsFinite(highway_merge_cfg.min_heading_alignment) ||
        highway_merge_cfg.min_heading_alignment < -1.0 ||
        highway_merge_cfg.min_heading_alignment > 1.0) {
        error_message =
            "planner/highway_merge/min_heading_alignment must be finite and in [-1, 1] (cos bound)";
        return false;
    }
    if (!IsFinite(highway_merge_cfg.min_front_gap) || highway_merge_cfg.min_front_gap < 0.0) {
        error_message = "planner/highway_merge/min_front_gap must be finite and >= 0";
        return false;
    }
    if (!IsFinite(highway_merge_cfg.min_rear_gap) || highway_merge_cfg.min_rear_gap < 0.0) {
        error_message = "planner/highway_merge/min_rear_gap must be finite and >= 0";
        return false;
    }
    if (!IsFinite(highway_merge_cfg.min_front_ttc) || highway_merge_cfg.min_front_ttc < 0.0) {
        error_message = "planner/highway_merge/min_front_ttc must be finite and >= 0";
        return false;
    }
    if (!IsFinite(highway_merge_cfg.min_rear_ttc) || highway_merge_cfg.min_rear_ttc < 0.0) {
        error_message = "planner/highway_merge/min_rear_ttc must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(highway_merge_cfg.max_wait_time,
                              "planner/highway_merge/max_wait_time", error_message))
        return false;
    if (!IsFinite(highway_merge_cfg.commit_distance) || highway_merge_cfg.commit_distance < 0.0) {
        error_message = "planner/highway_merge/commit_distance must be finite and >= 0";
        return false;
    }
    if (highway_merge_cfg.safe_confirm_cycles < 0) {
        error_message = "planner/highway_merge/safe_confirm_cycles must be >= 0";
        return false;
    }
    if (!IsFinite(highway_merge_cfg.cross_speed_floor) || highway_merge_cfg.cross_speed_floor < 0.0) {
        error_message = "planner/highway_merge/cross_speed_floor must be finite and >= 0";
        return false;
    }
    if (!CheckPositiveFinite(highway_merge_cfg.entry_search_dt,
                              "planner/highway_merge/entry_search_dt", error_message))
        return false;
    if (!CheckPositiveFinite(highway_merge_cfg.collision_check_dt,
                              "planner/highway_merge/collision_check_dt", error_message))
        return false;
    if (!CheckPositiveFinite(highway_merge_cfg.min_travel_speed,
                              "planner/highway_merge/min_travel_speed", error_message))
        return false;
    if (!CheckPositiveFinite(highway_merge_cfg.min_travel_accel,
                              "planner/highway_merge/min_travel_accel", error_message))
        return false;
    for (const auto& zone : highway_merge_cfg.zones) {
        if (highway_merge_cfg.enabled &&
            (!IsFinite(zone.start_s) || zone.start_s < 0.0 || zone.checkpoints.empty())) {
            error_message =
                "planner/highway_merge/zones/" + zone.name +
                ": start_s must be finite and >= 0, and checkpoints must not be empty";
            return false;
        }
        if (!IsFinite(zone.start_s) || !IsFinite(zone.completion_s) ||
            zone.completion_s < zone.start_s) {
            error_message =
                "planner/highway_merge/zones/" + zone.name +
                ": start_s/completion_s must be finite with completion_s >= start_s";
            return false;
        }
        if (!IsFinite(zone.stop_buffer) || zone.stop_buffer < 0.0 ||
            (!zone.checkpoints.empty() &&
             zone.checkpoints.front().conflict_s - zone.stop_buffer <= zone.start_s)) {
            error_message =
                "planner/highway_merge/zones/" + zone.name +
                ": stop_buffer must leave a wait point after start_s";
            return false;
        }
        double previous_clear_s = zone.start_s;
        for (const auto& checkpoint : zone.checkpoints) {
            if (!IsFinite(checkpoint.conflict_s) || !IsFinite(checkpoint.clear_s) ||
                checkpoint.conflict_s <= previous_clear_s ||
                checkpoint.clear_s <= checkpoint.conflict_s) {
                error_message =
                    "planner/highway_merge/zones/" + zone.name +
                    ": checkpoints must be finite and strictly ordered "
                    "(previous_clear_s < conflict_s < clear_s)";
                return false;
            }
            previous_clear_s = checkpoint.clear_s;
        }
        if (zone.completion_s < previous_clear_s) {
            error_message =
                "planner/highway_merge/zones/" + zone.name +
                ": completion_s must be >= the last checkpoint's clear_s";
            return false;
        }
    }

    // --- visualization ---
    if (!CheckPositiveFinite(visualization_cfg.publish_hz,
                              "planner/visualization/publish_hz", error_message))
        return false;
    if (visualization_cfg.snapshot_path.empty()) {
        error_message = "planner/visualization/snapshot_path must not be empty";
        return false;
    }

    // --- top-level planner scalars ---
    if (!CheckPositiveFinite(wheelbase, "planner/wheelbase", error_message))
        return false;
    if (!CheckPositiveFinite(lane_width, "planner/lane_width", error_message))
        return false;
    if (!CheckPositiveFinite(replanning_period_s, "planner/replanning_period_s", error_message))
        return false;
    if (!IsFinite(max_target_speed_accel) || max_target_speed_accel <= 0.0) {
        error_message = "planner/max_target_speed_accel must be finite and > 0";
        return false;
    }

    return true;
}

