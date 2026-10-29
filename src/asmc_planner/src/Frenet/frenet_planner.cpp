#include "Frenet/frenet_planner.hpp"
#include "Frenet/Maneuver/KeepDepartureGeometry.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_map>
#include <utility>
#include <iterator>
#include <stdexcept>

#include "Frenet/Generator/BoundaryConditionPolicy.hpp"
#include "Frenet/Maneuver/KeepCruise.hpp"
#include "Frenet/Maneuver/Merge/HighwayPrediction.hpp"
#include "Frenet/Geometry/HighwayLaneMotion.hpp"
#include "Frenet/Maneuver/Merge/MergeGapEvaluator.hpp"
#include "Frenet/Math/Utils.hpp"
#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Signal/BrakingModel.hpp"
#include "Frenet/Speed/CurveSpeed.hpp"
#include "Frenet/Speed/SpeedProfiler.hpp"
#include "Frenet/Speed/TargetSpeedResolver.hpp"

namespace {

bool PathObeysCurvatureLimit(const CartesianPath& path, double limit) {
    if (path.x.size() < 2 || path.kappa.size() != path.x.size() ||
        !std::isfinite(limit) || limit <= 0.0) {
        return false;
    }
    double observed_max = 0.0;
    for (double kappa : path.kappa) {
        if (!std::isfinite(kappa)) return false;
        observed_max = std::max(observed_max, std::fabs(kappa));
    }
    return observed_max <= limit + 1e-6;
}

}  // namespace

struct FrenetPlanner::PreparedStaticInputs {
    std::string config_path;
    std::string map_root;
    std::vector<std::pair<std::string, std::string>> files;
    AvoidRoadMap road_map;
    RefLine reference;
    // Circulating references in the zone are shared_ptr<const RefLine>.
    RoundaboutMergeZone roundabout;

    static std::string Read(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file) throw std::runtime_error("cannot read startup input: " + path);
        std::string bytes{std::istreambuf_iterator<char>(file), {}};
        if (file.bad()) throw std::runtime_error("cannot finish startup input: " + path);
        return bytes;
    }
    void Capture(const std::string& path) { files.emplace_back(path, Read(path)); }
    bool Unchanged() const {
        try {
            for (const auto& input : files)
                if (Read(input.first) != input.second) return false;
            return true;
        } catch (const std::exception&) { return false; }
    }
};

bool FrenetPlanner::SetPreparedReference(const RefLine& reference) {
    if(highway_route_)return false; // mission stations cannot follow a reference swap
    if (!ref_loaded_ || has_last_s_ || reference.closed_loop || reference.points.size()<3 ||
        !std::isfinite(reference.length) || reference.length<=0 ||
        std::abs(reference.points.front().s)>1e-9 ||
        std::abs(reference.points.back().s-reference.length)>1e-6) return false;
    for (std::size_t i=0;i<reference.points.size();++i) {
        const auto& p=reference.points[i];
        if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.theta)||
            !std::isfinite(p.s)||!std::isfinite(p.kappa)||!std::isfinite(p.d_kappa)||std::abs(p.kappa)>limits_.max_curvature+1e-9 ||
            (i&&p.s<=reference.points[i-1].s)) return false;
    }
    ref_=reference;
    departure_road_recovery_ = DepartureRoadRecovery{};
    ResetHighwayMerge(); // prior confirmation/READY belongs to the old reference
    debug_writer_.reset(); // an isolated probe must not overwrite production debug files
    // These mission stations belong to the original reference. A local trial
    // cannot reuse a roundabout zone, exit prediction or projection hint.
    ResetRoundaboutMerge(); roundabout_merge_cfg_.enabled=false;
    roundabout_zone_=RoundaboutMergeZone{};
    return true;
}

bool FrenetPlanner::Init(const std::string& yaml_path) {
    return InitImpl(yaml_path, nullptr);
}

bool FrenetPlanner::InitFromPrepared(const std::string& yaml_path,
                                     const FrenetPlanner& source) {
    if (ref_loaded_ || has_last_s_ || !source.prepared_static_inputs_) return false;
    return InitImpl(yaml_path, source.prepared_static_inputs_);
}

bool FrenetPlanner::InitImpl(const std::string& yaml_path,
                            std::shared_ptr<const PreparedStaticInputs> prepared) {
    shadow_avoidance_profile_=false;
    if (prepared && (prepared->config_path != yaml_path || !prepared->Unchanged()))
        prepared.reset();
    prepared_static_inputs_.reset();
    auto snapshot = std::make_shared<PreparedStaticInputs>();
    snapshot->config_path = yaml_path;
    try { if (!prepared) snapshot->Capture(yaml_path); }
    catch (const std::exception& error) {
        ref_loaded_ = false;
        std::fprintf(stderr, "[FrenetPlanner] %s\n", error.what());
        return false;
    }
    ref_loaded_ = false;
    departure_road_recovery_ = DepartureRoadRecovery{};
    highway_route_.reset();highway_prediction_map_.reset();highway_owned_lanes_.clear();highway_areas_path_.clear();ResetHighwayMerge();
    solid_paint_contact_active_ = false;
    solid_paint_contact_elapsed_s_ = 0.0;
    highway_merge_checkpoint_s_ = -1.0;
    highway_merge_safe_count_ = 0;
    roundabout_merge_cfg_ = RoundaboutMergeConfig{};
    roundabout_zone_ = RoundaboutMergeZone{};
    ResetRoundaboutMerge();
    // Optional for synthetic/core tests; production explicitly enables the pinned map.
    avoid_road_map_ = AvoidRoadMap{};
    try {
        auto config=YAML::LoadFile(yaml_path);
        auto map=config["avoid_hd_map"];
        if(map && map["enabled"].as<bool>(false)) {
            std::string root=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/hd_map/";
            const auto slash=yaml_path.find_last_of("/\\");
            const std::string config_dir=slash==std::string::npos?"./":yaml_path.substr(0,slash+1);
            if (map["directory"]) {
                root=map["directory"].as<std::string>();
                if(root.empty())throw std::runtime_error("empty HD map directory");
                if(root.front()!='/')root=config_dir+root;
                if(root.back()!='/')root+='/';
            } else if(std::ifstream(config_dir+"hd_map/drivable.json").good()) {
                root=config_dir+"hd_map/";
            }
            // The implicit config-local map directory may appear after the
            // source initialized. Reuse must follow the resolved input set too.
            if (prepared && prepared->map_root != root) {
                prepared.reset();
                snapshot->Capture(yaml_path);
            }
            snapshot->map_root = root;
            std::string error;
            // This file bundle is independent of the /hd_map/global topics.
            // Update drivable.json, lines.json and drivable_grid.rle together;
            // publishing a newer HD map does not replace this planner cache.
            // See docs/handoffs/hd-map-to-planning.md for bundle acceptance.
            if (!prepared) {
                snapshot->Capture(root+"drivable.json");
                snapshot->Capture(root+"lines.json");
                snapshot->Capture(root+"drivable_grid.rle");
            }
            if (prepared) avoid_road_map_ = prepared->road_map;
            else if(!avoid_road_map_.Load(root+"drivable.json",root+"lines.json",error)) {
                std::fprintf(stderr,"[FrenetPlanner] HD map init failed: %s\n",error.c_str());
                return false;
            }
            highway_areas_path_=root+"drivable.json";
        }
    } catch(const std::exception& error) { std::fprintf(stderr,"%s\n",error.what()); return false; }
    std::string waypoint_file;
    std::string config_error;
    if (!LoadParams(yaml_path, ref_config_, object_tracking_cfg_, path_cfg_, limits_,
                     cost_weights_, keep_cruise_cfg_,
                     vehicle_shape_,
                     collision_cfg_, curve_speed_cfg_, following_cfg_, avoid_cfg_,
                     lane_change_cfg_,
                     stop_cfg_, emergency_stop_cfg_, merge_cfg_, highway_merge_cfg_,
                     visualization_cfg_,
                     wheelbase_, lane_width_, replanning_period_s_,
                     max_target_speed_accel_, waypoint_file, config_error)) {
        std::printf("[FrenetPlanner] Config load failed ('%s'): %s\n",
                    yaml_path.c_str(), config_error.c_str());
        return false;
    }
    if (!ValidateConfig(ref_config_, object_tracking_cfg_, path_cfg_, limits_,
                         cost_weights_, keep_cruise_cfg_, vehicle_shape_,
                         collision_cfg_, curve_speed_cfg_, following_cfg_,
                         avoid_cfg_, lane_change_cfg_, stop_cfg_,
                         emergency_stop_cfg_, merge_cfg_, highway_merge_cfg_,
                         visualization_cfg_, wheelbase_, lane_width_,
                         replanning_period_s_, max_target_speed_accel_,
                         config_error)) {
        std::printf("[FrenetPlanner] Config validation failed ('%s'): %s\n",
                    yaml_path.c_str(), config_error.c_str());
        return false;
    }

    // Init is also the planner-state reset boundary.  A reused planner object
    // must not carry a normal KEEP acceleration history into a new reference
    // or configuration session.
    speed_profiler_.Reset();
    last_profiled_acceleration_ = 0.0;

    if (!waypoint_file.empty() && waypoint_file.front() != '/') {
        waypoint_file = std::string(FRENET_PACKAGE_SRC_DIR) + "/" + waypoint_file;
    }

    debug_writer_ = std::make_unique<PlannerDebugWriter>(visualization_cfg_);

    // reference line 자체가 만족해야 하는 곡률 한계는 KinematicFilter가
    // 후보를 걸러내는 물리적 한계와 같은 차량 능력이다 - 별도 YAML key로
    // 중복 관리하지 않고 loaded KinematicLimits에서 그대로 읽어 넘긴다.
    try { if (!prepared && !waypoint_file.empty()) snapshot->Capture(waypoint_file); }
    catch (const std::exception& error) {
        std::fprintf(stderr, "[FrenetPlanner] %s\n", error.what()); return false;
    }
    if (prepared) ref_ = prepared->reference;
    else if (waypoint_file.empty() ||
        !LoadReferenceLine(waypoint_file, ref_, ref_config_, limits_.max_curvature)) {
        std::printf("[FrenetPlanner] Reference line load failed (waypoint_file='%s')\n",
                    waypoint_file.c_str());
        return false;
    }
    if (!prepared) {
        if (merge_cfg_.conflict_s < 0.0) {
            std::printf("[FrenetPlanner] Roundabout MERGE disabled: set "
                        "planner.merge.conflict_s in params.yaml\n");
        } else {
            const RefPoint conflict = Interpolate(ref_, merge_cfg_.conflict_s);
            std::printf("[FrenetPlanner] Roundabout MERGE conflict: s=%.2f pos=(%.2f, %.2f)\n",
                        merge_cfg_.conflict_s, conflict.x, conflict.y);
        }
        // zone s-ordering은 ConfigValidator::ValidateConfig가 이미 검증했다 -
        // 여기서는 로그만 남긴다. 잘못된 zone 설정은 Init() 자체를 실패시키므로
        // (Rule: config가 유효하지 않으면 조용히 기능을 끄지 않는다) 이 지점에
        // 도달했다면 highway_merge_cfg_는 항상 유효하다.
        if (!highway_merge_cfg_.enabled || highway_merge_cfg_.zones.empty()) {
            std::printf("[FrenetPlanner] Highway MERGE disabled: measure start/conflict/"
                        "completion_s, then set planner.highway_merge.enabled=true\n");
        } else {
            for (const auto& zone : highway_merge_cfg_.zones) {
                std::printf("[FrenetPlanner] Highway MERGE '%s': start=%.2f complete=%.2f "
                            "checkpoints=%zu\n", zone.name.c_str(), zone.start_s,
                            zone.completion_s, zone.checkpoints.size());
                for (size_t i = 0; i < zone.checkpoints.size(); ++i) {
                    const auto& checkpoint = zone.checkpoints[i];
                    const RefPoint conflict = Interpolate(ref_, checkpoint.conflict_s);
                    std::printf("  checkpoint[%zu]: conflict=%.2f clear=%.2f pos=(%.2f, %.2f)\n",
                                i, checkpoint.conflict_s, checkpoint.clear_s,
                                conflict.x, conflict.y);
                }
            }
        }
    }
    ref_loaded_ = true;
    try {
        auto config = YAML::LoadFile(yaml_path);
        auto merge = config["roundabout_merge"];
        if (merge && merge["enabled"].as<bool>(false)) {
            roundabout_merge_cfg_.enabled = true;
            roundabout_merge_cfg_.speed_cap = merge["speed_cap_mps"].as<double>(25.0/3.6);
            roundabout_merge_cfg_.max_horizon = merge["max_horizon_s"].as<double>(10.0);
            roundabout_merge_cfg_.min_front_time_gap = merge["min_front_time_gap_s"].as<double>(merge_cfg_.min_front_time_gap);
            roundabout_merge_cfg_.preview_distance = merge["preview_distance_m"].as<double>(45.0);
            roundabout_merge_cfg_.speed_measurement_error = merge["speed_measurement_error_mps"].as<double>(5.0);
            roundabout_merge_cfg_.partial_vehicle_length = merge["partial_vehicle_length_m"].as<double>(6.0);
            roundabout_merge_cfg_.partial_vehicle_width = merge["partial_vehicle_width_m"].as<double>(3.0);
            roundabout_merge_cfg_.execution_reaction_time = merge["execution_reaction_time_s"].as<double>(1.0);
            roundabout_merge_cfg_.execution_max_rear_decel = merge["execution_max_rear_decel_mps2"].as<double>(3.0);
            roundabout_merge_cfg_.source_link = merge["source_link"].as<std::string>(roundabout_merge_cfg_.source_link);
            roundabout_merge_cfg_.target_link = merge["target_link"].as<std::string>(roundabout_merge_cfg_.target_link);
            roundabout_merge_cfg_.stop_node = merge["stop_node"].as<std::string>(roundabout_merge_cfg_.stop_node);
            if (merge["conflict_links"]) roundabout_merge_cfg_.conflict_links = merge["conflict_links"].as<std::vector<std::string>>();
            if (!std::isfinite(roundabout_merge_cfg_.speed_cap) || roundabout_merge_cfg_.speed_cap <= 0 ||
                !std::isfinite(roundabout_merge_cfg_.max_horizon) || roundabout_merge_cfg_.max_horizon < path_cfg_.time_horizon.max ||
                roundabout_merge_cfg_.max_horizon > 10 || roundabout_merge_cfg_.conflict_links.empty() ||
                !std::isfinite(roundabout_merge_cfg_.min_front_time_gap) || roundabout_merge_cfg_.min_front_time_gap < 0 ||
                !std::isfinite(roundabout_merge_cfg_.speed_measurement_error) || roundabout_merge_cfg_.speed_measurement_error <= 0 ||
                !std::isfinite(roundabout_merge_cfg_.partial_vehicle_length) || roundabout_merge_cfg_.partial_vehicle_length <= 0 ||
                !std::isfinite(roundabout_merge_cfg_.partial_vehicle_width) || roundabout_merge_cfg_.partial_vehicle_width <= 0 ||
                !std::isfinite(roundabout_merge_cfg_.execution_reaction_time) || roundabout_merge_cfg_.execution_reaction_time < 0 ||
                !std::isfinite(roundabout_merge_cfg_.execution_max_rear_decel) || roundabout_merge_cfg_.execution_max_rear_decel <= 0 ||
                !std::isfinite(roundabout_merge_cfg_.preview_distance) || roundabout_merge_cfg_.preview_distance <= 0)
                throw std::runtime_error("invalid roundabout merge configuration");
            if (!avoid_road_map_.enabled()) throw std::runtime_error("roundabout MERGE requires the HD map");
            const auto slash=yaml_path.find_last_of("/\\");
            std::string config_dir=slash==std::string::npos?"./":yaml_path.substr(0,slash+1);
            std::string root=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/hd_map/";
            auto map=config["avoid_hd_map"];
            if(map["directory"]) { root=map["directory"].as<std::string>(); if(root.empty())throw std::runtime_error("empty HD map directory"); if(root.front()!='/')root=config_dir+root; if(root.back()!='/')root+='/'; }
            else if(std::ifstream(config_dir+"hd_map/drivable.json").good())root=config_dir+"hd_map/";
            std::string error;
            if (prepared) roundabout_zone_ = prepared->roundabout;
            else if(!roundabout_zone_.Load(root+"drivable.json",root+"lines.json",ref_,vehicle_shape_,roundabout_merge_cfg_,collision_cfg_.safety_margin,error))
                throw std::runtime_error(error);
            auto p=Interpolate(ref_,roundabout_zone_.stop_s);
            if(!avoid_road_map_.FootprintInside(p.x,p.y,p.theta,vehicle_shape_))
                throw std::runtime_error("roundabout waiting footprint leaves drivable area");
        }
    } catch(const std::exception& error) {
        ref_loaded_ = false;
        std::fprintf(stderr,"[FrenetPlanner] roundabout init failed: %s\n",error.what());
        return false;
    }
    const auto& inputs = prepared ? *prepared : *snapshot;
    if (!inputs.Unchanged()) {
        ref_loaded_ = false;
        std::fprintf(stderr, "[FrenetPlanner] startup inputs changed while initializing\n");
        return false;
    }
    if (prepared) {
        prepared_static_inputs_ = std::move(prepared);
        std::printf("[FrenetPlanner] Reused validated map/reference inputs; fresh execution state\n");
    } else {
        snapshot->road_map = avoid_road_map_;
        snapshot->reference = ref_;
        snapshot->roundabout = roundabout_zone_;
        prepared_static_inputs_ = std::move(snapshot);
    }
    return true;
}

bool FrenetPlanner::GenerateDecelToZeroPath(double s, double d, double s_dot,
                                            CartesianPath& out_path) const {
    // D4-3에서 STOP BRAKING fail-safe에서 추출했다. 두 경로가 정확히 같은
    // 감속 프로파일을 쓰도록 공유한다 - 복사해두면 한쪽만 고쳐지는 문제가
    // 생긴다.
    // 이 helper는 AVOID/LANE_CHANGE maintenance도 공유한다. 거리 기반 강화값은
    // 실제 STOP에서만 사용하고, 다른 모드로 전환된 뒤 과거 STOP 감속도가
    // 새어 나가지 않게 한다.
    const double normal_stop_decel =
        last_mode_ == STOP && stop_.commanded_decel > 0.0
            ? stop_.commanded_decel
            : stop_cfg_.comfortable_decel;
    const double decel = std::max(
        stop_cfg_.min_decel,
        last_mode_ == EMERGENCY ? emergency_stop_cfg_.decel
                                : normal_stop_decel);
    const double v0 = std::max(0.0, s_dot);
    const int n = static_cast<int>(std::ceil(v0 / decel / path_cfg_.dt)) + 1;
    out_path.x.assign(n, 0.0); out_path.y.assign(n, 0.0);
    out_path.yaw.assign(n, 0.0); out_path.kappa.assign(n, 0.0);
    out_path.v.assign(n, 0.0); out_path.a.assign(n, 0.0);
    double s_i = s;
    double v_prev = v0;
    for (int i = 0; i < n; ++i) {
        const double t = i * path_cfg_.dt;
        const double v_i = std::max(0.0, v0 - decel * t);
        if (i > 0) s_i += 0.5 * (v_i + v_prev) * path_cfg_.dt;
        v_prev = v_i;
        const RefPoint rp = Interpolate(ref_, s_i);
        // 정지 직전 구간이라 횡방향은 현재 오프셋을 그대로 유지한다.
        const CartesianState cs =
            FrenetToCartesian(rp, s_i, v_i, v_i > 0.0 ? -decel : 0.0,
                              d, 0.0, 0.0);
        out_path.x[i] = cs.x;
        out_path.y[i] = cs.y;
        out_path.yaw[i] = cs.yaw;
        out_path.kappa[i] = cs.kappa;
        out_path.v[i] = v_i;
        out_path.a[i] = v_i > 0.0 ? -decel : 0.0;
    }
    return n > 0;
}

bool FrenetPlanner::PlanLowSpeedFallback(const CartesianState& ego, double s, double d,
                                          double target_d,
                                          CartesianPath& out_path, double duration_s,
                                          double speed_cap,
                                          double transition_distance_override,
                                          double minimum_path_distance) const {
    const double natural_transition_distance =
        LowSpeedLateralTransitionDistance(d, target_d);
    const double transition_distance =
        std::isfinite(transition_distance_override) &&
                transition_distance_override > 0.0
            ? std::max(natural_transition_distance,
                       transition_distance_override)
            : natural_transition_distance;
    const int minimum_samples =
        std::max(2, static_cast<int>(duration_s / path_cfg_.dt) + 1);
    const double required_distance =
        std::isfinite(minimum_path_distance) && minimum_path_distance > 0.0
            ? std::max(transition_distance, minimum_path_distance)
            : -1.0;
    // A full signal exit uses distance-domain lateral geometry even at v=0.
    // Reuse the common jerk-limited longitudinal integrator: a horizon-based
    // velocity polynomial would postpone its initial jerk on every replan.
    const bool complete_exit=required_distance>0;
    const double exit_cap=std::min(speed_cap,limits_.low_speed_crawl_speed_cap);
    const double initial_acceleration=complete_exit ? asmc_planner::signal::StopBoundaryAcceleration(
        std::max(0.,ego.v),ego.a,{limits_.max_longitudinal_accel*limits_.boundary_condition_accel_fraction,
        limits_.max_longitudinal_jerk*limits_.boundary_condition_accel_fraction,
        limits_.max_longitudinal_accel*limits_.boundary_condition_accel_fraction}) : ego.a;
    constexpr int kMaximumSamples = 600;
    FrenetPath exit_profile;
    if(complete_exit) {
        if(!std::isfinite(exit_cap) || exit_cap<=0)return false;
        auto cfg=path_cfg_;const double duration=(kMaximumSamples-1)*cfg.dt;
        cfg.time_horizon={duration,duration,1.};cfg.delta_s_dot={0,0,1.};
        auto profile_limits=limits_;
        profile_limits.max_longitudinal_accel=std::min(limits_.max_longitudinal_accel,
            std::max(limits_.low_speed_crawl_accel,std::abs(initial_acceleration))/
                limits_.boundary_condition_accel_fraction);
        // Reserve the speed gained while positive acceleration is released
        // plus one integration interval. A desired ceiling alone would allow
        // this discrete jerk-limited integrator to overshoot the 2m/s exit cap.
        const double a_limit=profile_limits.max_longitudinal_accel*profile_limits.boundary_condition_accel_fraction;
        const double j_limit=profile_limits.max_longitudinal_jerk*profile_limits.boundary_condition_accel_fraction;
        const double tracking_cap=exit_cap-a_limit*a_limit/(2*j_limit)-a_limit*cfg.dt;
        if(tracking_cap<=0)return false;
        FrenetState initial{};initial.s=s;initial.s_d=std::max(0.,ego.v);initial.s_dd=initial_acceleration;
        const double span=std::max(required_distance,std::max(exit_cap,initial.s_d)*duration)+10.;
        auto profiles=GenerateEnvelopeVelocityCandidates(initial,tracking_cap,
            {{s,tracking_cap*tracking_cap},{s+span,tracking_cap*tracking_cap}},cfg,profile_limits);
        FilterLongitudinalByKinematicLimits(profiles,profile_limits);
        if(profiles.size()!=1 || !profiles.front().valid || profiles.front().s.size()<kMaximumSamples)return false;
        exit_profile=std::move(profiles.front());
    }

    // The distance-based fallback must start with the vehicle's observed
    // heading.  Starting every spatial profile with d'=0 snaps the footprint
    // to the reference-line heading at the first sample.  That is especially
    // wrong after a stopped vehicle has already steered away from an obstacle:
    // collision checking then rotates the ego OBB back toward the obstacle.
    const RefPoint start_ref = Interpolate(ref_, s);
    const double one_minus_kd = 1.0 - start_ref.kappa * d;
    const double heading_error = normalizeAngle(ego.yaw - start_ref.theta);
    const double heading_cosine = std::cos(heading_error);
    double initial_d_prime = 0.0;
    if (std::isfinite(one_minus_kd) &&
        std::isfinite(heading_error) &&
        std::fabs(heading_cosine) > 1e-3) {
        initial_d_prime = one_minus_kd * std::tan(heading_error);
    }

    // Spatial quintic d(ds).  It preserves the observed initial slope while
    // reaching target_d with zero slope/second derivative.  We deliberately
    // use zero initial d'' because VehicleState has no reliable steering/
    // curvature measurement at standstill; inventing it from yaw_rate/v would
    // reintroduce the low-speed singularity this fallback avoids.
    const double a0 = d;
    const double a1 = initial_d_prime * transition_distance;
    const double a2 = 0.0;
    const double position_residual = target_d - (a0 + a1 + a2);
    const double slope_residual = -(a1 + 2.0 * a2);
    const double curvature_residual = -2.0 * a2;
    const double a3 = 10.0 * position_residual -
                      4.0 * slope_residual +
                      0.5 * curvature_residual;
    const double a4 = -15.0 * position_residual +
                      7.0 * slope_residual -
                      curvature_residual;
    const double a5 = 6.0 * position_residual -
                      3.0 * slope_residual +
                      0.5 * curvature_residual;

    out_path = CartesianPath{};
    out_path.x.reserve(minimum_samples); out_path.y.reserve(minimum_samples);
    out_path.yaw.reserve(minimum_samples); out_path.kappa.reserve(minimum_samples);
    out_path.v.reserve(minimum_samples); out_path.a.reserve(minimum_samples);

    double s_i = s;
    double v_prev = complete_exit ? std::max(0.,ego.v) : std::max(ego.v, limits_.low_speed_min_creep_speed);
    // 거리 기반 후보는 목표 횡이동을 실제로 끝낼 때까지 생성한다. 안전을
    // 위해 비정상 파라미터에도 유한한 상한을 둔다(0.1 s 기준 60 s).
    for (int i = 0; i < kMaximumSamples; ++i) {
        const double t = i * path_cfg_.dt;
        const double crawl_speed = clip(
            ego.v + limits_.low_speed_crawl_accel * t,
            limits_.low_speed_min_creep_speed, limits_.low_speed_crawl_speed_cap);
        const double v_i = complete_exit ? exit_profile.s_d[i] :
            std::min(crawl_speed, speed_cap);
        if(complete_exit)s_i=exit_profile.s[i];
        else if (i > 0) s_i += 0.5 * (v_i + v_prev) * path_cfg_.dt;
        v_prev = v_i;

        const double ds = s_i - s;
        const double frac = clip(ds / transition_distance, 0.0, 1.0);
        const double frac2 = frac * frac;
        const double frac3 = frac2 * frac;
        const double frac4 = frac3 * frac;
        const double frac5 = frac4 * frac;
        const bool transition_complete = ds >= transition_distance;
        const double d_i = transition_complete
            ? target_d
            : a0 + a1 * frac + a2 * frac2 + a3 * frac3 +
                  a4 * frac4 + a5 * frac5;
        const double d_prime = transition_complete
            ? 0.0
            : (a1 + 2.0 * a2 * frac + 3.0 * a3 * frac2 +
               4.0 * a4 * frac3 + 5.0 * a5 * frac4) /
                  transition_distance;
        const double d_pprime = transition_complete
            ? 0.0
            : (2.0 * a2 + 6.0 * a3 * frac + 12.0 * a4 * frac2 +
               20.0 * a5 * frac3) /
                  (transition_distance * transition_distance);

        RefPoint rp = Interpolate(ref_, s_i);
        const double a_i = complete_exit ? exit_profile.s_dd[i] :
            (v_i < limits_.low_speed_crawl_speed_cap ? limits_.low_speed_crawl_accel : 0.0);
        CartesianState cs = FrenetToCartesian(rp, s_i, v_i, a_i, d_i, d_prime, d_pprime);

        out_path.x.push_back(cs.x);
        out_path.y.push_back(cs.y);
        out_path.yaw.push_back(cs.yaw);
        out_path.kappa.push_back(cs.kappa);
        out_path.v.push_back(v_i);
        out_path.a.push_back(a_i);

        const bool duration_satisfied = i + 1 >= minimum_samples;
        const bool distance_satisfied =
            required_distance <= 0.0 || ds >= required_distance;
        if (duration_satisfied && distance_satisfied) break;
    }
    return out_path.x.size() >= 2 &&
           (required_distance <= 0.0 || s_i - s >= required_distance - 1e-6);
}

bool FrenetPlanner::KeepDepartureRoadClear(const CartesianPath& path) const {
    auto exterior=vehicle_shape_;
    exterior.width=std::max(exterior.width,exterior.road_outer_width);
    return avoid_road_map_.EvaluatePath(path,exterior,path_cfg_.dt).drivable;
}

bool FrenetPlanner::PlanCurvatureFeasibleKeepLowSpeedFallback(
    const CartesianState& ego, double s, double d,
    double resolved_target_speed, CartesianPath& out_path,
    double duration_s, bool* road_blocked,bool* actuation_blocked,double minimum_path_distance) {
    if (road_blocked) *road_blocked=false;
    if (actuation_blocked) *actuation_blocked=false;
    CartesianPath initial_pose;
    initial_pose.x={ego.x};initial_pose.y={ego.y};initial_pose.yaw={ego.yaw};
    const bool recovering=departure_road_recovery_.active;
    auto exterior=vehicle_shape_;
    exterior.width=std::max(exterior.width,exterior.road_outer_width);
    if (!recovering&&!KeepDepartureRoadClear(initial_pose)) {
        if(road_blocked)*road_blocked=true;
        out_path=CartesianPath{};return false;
    }
    const double initial_distance = LowSpeedLateralTransitionDistance(d, 0.0);
    double recovery_cost=0.0;
    auto generate = [&](double target_d, double transition_distance,
                        CartesianPath& candidate) {
        // Recovery must include the return and settled clearance, not merely
        // a short prefix pointing toward the road. The explicit departure
        // policy scores exposure without changing existing maneuver limits.
        const double required_distance=recovering
            ? std::max(minimum_path_distance,transition_distance+.5)
            : minimum_path_distance;
        if (!PlanLowSpeedFallback(ego, s, d, target_d, candidate, duration_s,
                                  resolved_target_speed,
                                  transition_distance,required_distance)) {
            return false;
        }
        if(recovering&&
           (std::hypot(candidate.x.front()-ego.x,candidate.y.front()-ego.y)>1e-6||
            std::fabs(normalizeAngle(candidate.yaw.front()-ego.yaw))>1e-6)) {
            // A spatial profile must not silently choose the opposite branch
            // of atan(tan(yaw)) or rotate a near-perpendicular starting body.
            if(actuation_blocked)*actuation_blocked=true;
            return false;
        }
        if (!PathObeysCurvatureLimit(candidate, limits_.max_curvature)) return false;
        if(required_distance>0)for(std::size_t i=0;i<candidate.v.size();++i)
            if(candidate.v[i]*candidate.v[i]*std::abs(candidate.kappa[i])>limits_.max_lateral_accel)
                return false;
        if (!PathObeysSteeringRate(candidate,path_cfg_.dt,wheelbase_,limits_.steering)) {
            if (actuation_blocked) *actuation_blocked=true;
            return false;
        }
        const auto road=recovering
            ? avoid_road_map_.EvaluateDepartureRecoveryPath(candidate,exterior,path_cfg_.dt,
                true,departure_road_recovery_.settled_travel_m)
            : avoid_road_map_.EvaluatePath(candidate,exterior,path_cfg_.dt);
        const bool road_clear=road.drivable;
        recovery_cost=AvoidRoadMap::DepartureRecoveryCost(road);
        if (!road_clear) {
            if (road_blocked) *road_blocked=true;
            return false;
        }
        return true;
    };

    constexpr double kExpansionFactor = 1.5;
    constexpr int kMaximumExpansionAttempts = 8;
    constexpr int kRefinementIterations = 6;
    if (!recovering&&KeepDepartureOffsetHoldAllowed(ego,ref_,s,vehicle_shape_,lane_width_,
                                      limits_.near_low_speed_fallback_v)) {
        double distance=LowSpeedLateralTransitionDistance(d,d);
        for(int attempt=0;attempt<kMaximumExpansionAttempts;++attempt,distance*=kExpansionFactor) {
            CartesianPath held;
            if(generate(d,distance,held)) {
                out_path=std::move(held);
                return true;
            }
        }
    }

    CartesianPath initial;
    if (generate(0.0, initial_distance, initial)) {
        out_path = std::move(initial);
        return true;
    }

    double infeasible_distance = initial_distance;
    double feasible_distance = std::numeric_limits<double>::quiet_NaN();
    CartesianPath feasible_path;
    double best_recovery_cost=std::numeric_limits<double>::infinity();
    double trial_distance = initial_distance;
    for (int attempt = 0; attempt < kMaximumExpansionAttempts; ++attempt) {
        trial_distance *= kExpansionFactor;
        CartesianPath candidate;
        if (generate(0.0, trial_distance, candidate)) {
            feasible_distance = trial_distance;
            feasible_path = std::move(candidate);
            best_recovery_cost=recovery_cost;
            break;
        }
        infeasible_distance = trial_distance;
    }

    if (std::isfinite(feasible_distance)) {
        for (int iteration = 0; iteration < kRefinementIterations; ++iteration) {
            const double midpoint = 0.5 * (infeasible_distance + feasible_distance);
            CartesianPath candidate;
            if (generate(0.0, midpoint, candidate)) {
                feasible_distance = midpoint;
                if(!recovering||recovery_cost<best_recovery_cost) {
                    feasible_path = std::move(candidate);
                    best_recovery_cost=recovery_cost;
                }
            } else {
                infeasible_distance = midpoint;
            }
        }
        out_path = std::move(feasible_path);
        return true;
    }

    CartesianPath lateral_hold;
    if (!recovering&&generate(d, 1.0, lateral_hold)) {
        out_path = std::move(lateral_hold);
        return true;
    }
    out_path = CartesianPath{};
    return false;
}

double FrenetPlanner::LowSpeedLateralTransitionDistance(
    double from_d, double to_d) const {
    const double lateral_distance = std::fabs(to_d - from_d);
    if (!std::isfinite(lateral_distance) || lateral_distance < 1e-3) {
        return 1.0;
    }

    // 대칭인 두 원호로 횡이동하는 S-curve는 작은 각도 영역에서
    // longitudinal distance ~= 2*sqrt(D/kappa)다. 차량 한계값을 그대로
    // 쓰면 이산 샘플/MPC 오차에 취약하므로 곡률의 80%와 거리 10% 여유를
    // 사용한다. D=2.75m, max_curvature=0.28이면 약 7.7m다.
    const double usable_curvature = std::max(
        1e-3, limits_.max_curvature * limits_.low_speed_usable_curvature_fraction);
    return std::max(
        1.0,
        limits_.low_speed_transition_distance_margin * 2.0 *
            std::sqrt(lateral_distance / usable_curvature));
}

bool FrenetPlanner::HighwayCruiseFor(const PlannerCommand& command) const {
    if((command.mode==LANE_KEEPING||command.mode==FOLLOWING)&&
       std::find(highway_owned_lanes_.begin(),highway_owned_lanes_.end(),
                 "lane:"+command.source_link_id)!=highway_owned_lanes_.end())return true;
    const auto map=highway_prediction_map_?highway_prediction_map_:
        (highway_route_?highway_route_->map:nullptr);
    return IsHighwayCruiseCommand(highway_merge_cfg_,map.get(),command);
}

CurveSpeedConfig FrenetPlanner::CurveSpeedFor(const PlannerCommand& command) const {
    return ResolveHighwayCurveSpeed(curve_speed_cfg_,highway_merge_cfg_,
        highway_merge_enabled()||bool(highway_prediction_map_),command,HighwayCruiseFor(command));
}

PlannerConfigBundle FrenetPlanner::MakePipelineConfigBundle() const {
    return MakePipelineConfigBundle(curve_speed_cfg_);
}

PlannerConfigBundle FrenetPlanner::MakePipelineConfigBundle(const CurveSpeedConfig& curve) const {
    return PlannerConfigBundle{
        path_cfg_, limits_, cost_weights_, keep_cruise_cfg_, vehicle_shape_, collision_cfg_,
        stop_cfg_, merge_cfg_, avoid_cfg_, following_cfg_, curve,
        lane_width_};
}

// Cartesian 정지 목표 -> reference line 종방향 좌표.
bool FrenetPlanner::ProjectPointToFrenet(double x, double y, double s_hint,
                                         double search_window,
                                         double max_lateral_distance,
                                         double& out_s, double& out_d) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    // 창과 허용 거리는 호출자가 정한다. 잘못된 값으로 조용히 통과시키면
    // 투영 방어가 사실상 사라진다.
    if (!std::isfinite(search_window) || search_window <= 0.0) return false;
    if (!std::isfinite(max_lateral_distance) || max_lateral_distance <= 0.0) {
        return false;
    }

    // 탐색창은 ego localization용 기본값(30m)이 아니라 호출자가 준 값을 쓴다.
    // 목표점은 ego보다 앞에 있어 기본 창을 벗어나고, 그러면 잘린 초기
    // 추정값에서 출발한 Newton이 트랙의 다른 구간으로 발산한다.
    const double* hint = std::isfinite(s_hint) ? &s_hint : nullptr;
    const double s = FindClosestS(ref_, x, y, hint, search_window);
    if (!std::isfinite(s)) return false;

    // 최근접 투영은 목표가 경로에서 아무리 멀어도 어떤 s든 돌려준다. 실제로
    // 그 s가 목표를 대표하는지 확인하지 않으면, 다른 도로 위의 목표점이
    // 엉뚱한 위치의 명령으로 둔갑한다.
    const RefPoint rp = Interpolate(ref_, s);
    const double lateral_error = std::hypot(x - rp.x, y - rp.y);
    if (!std::isfinite(lateral_error) ||
        lateral_error > max_lateral_distance) {
        return false;
    }

    // d의 부호 규약은 CartesianToFrenet 식 (4)와 같아야 한다. 다른 식을 쓰면
    // 같은 점이 경로에 따라 좌/우가 반대로 나온다.
    //   d = (x - r(s))^T n_r(s),  n_r = (-sin(theta_r), cos(theta_r))
    // 즉 진행 방향 기준 좌측이 양수다.
    const double d = (x - rp.x) * (-std::sin(rp.theta)) +
                     (y - rp.y) * std::cos(rp.theta);
    if (!std::isfinite(d)) return false;

    out_s = s;
    out_d = d;
    return true;
}

bool FrenetPlanner::ProjectPointToS(double x, double y, double s_hint,
                                    double& out_s) const {
    // STOP 전용 wrapper. 탐색창과 허용 횡거리를 STOP config로 고정하고 d는
    // 버린다. v1 STOP도 이 함수를 쓰고 있으므로 동작을 바꾸지 않는다
    // (ProjectPointToFrenet 헤더 주석 참고).
    double s = 0.0;
    double ignored_d = 0.0;
    if (!ProjectPointToFrenet(x, y, s_hint,
                              stop_cfg_.projection_search_window,
                              stop_cfg_.max_lateral_offset, s, ignored_d)) {
        return false;
    }
    out_s = s;
    return true;
}

bool FrenetPlanner::ReferenceHeadingAt(double s, double& out_heading) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(s)) return false;
    // topology(wrap 폐루프 vs clamp 열린 경로) 정규화는 Interpolate가 유일한
    // owner다(Commit F, "one policy, one authoritative owner") - 여기서
    // s_max로 다시 fmod하지 않는다.
    const RefPoint rp = Interpolate(ref_, s);
    if (!std::isfinite(rp.theta)) return false;
    out_heading = rp.theta;
    return true;
}

bool FrenetPlanner::ProjectEgoToS(const CartesianState& ego, double& out_s) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(ego.x) || !std::isfinite(ego.y)) return false;
    const double s = FindClosestS(ref_, ego.x, ego.y,
                                  has_last_s_ ? &last_s_ : nullptr,
                                  ref_config_.ego_projection_search_window);
    if (!std::isfinite(s)) return false;
    out_s = s;
    return true;
}

bool FrenetPlanner::ReferencePoseAt(double s, CartesianState& pose) const {
    if (!ref_loaded_ || ref_.points.empty() || !std::isfinite(s)) return false;
    const auto p = Interpolate(ref_, s);
    pose = {p.x, p.y, p.theta, p.kappa, 0., 0.};
    return std::isfinite(p.x+p.y+p.theta+p.kappa);
}

namespace {

// 외부 command로 실제 열려 있는 모드. MERGE/LANE_CHANGE는 FSM 계약이
// 확정될 때까지 production 진입점에서 허용하지 않는다. 새 모드를 열 때는
// 여기에 case를 추가하고 ValidateExternalCommand에 필드 검증을 같이 넣는다.
bool IsSupportedExternalMode(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING:
        case STOP:
        case EMERGENCY:
        case FOLLOWING:
        // D6-1: AVOID는 대상 확보(resolver)까지만 열렸다. 후보 생성은 D6-2다.
        // 여기서 열지 않으면 resolver를 검증할 수 없고, 열어도 아래
        // ResolveManeuver 직전에서 UNSUPPORTED_BEHAVIOR로 막으므로
        // 조용히 velocity keeping 궤적이 나가지는 않는다.
        //
        // node의 SetImplementedManeuver(AVOID)는 계속 닫혀 있으므로 ROS
        // 경로로는 여기까지 도달하지 않는다(계약 12.7.1절).
        case AVOID:
            return true;

        // LANE_CHANGE도 AVOID와 같은 단계로 열려 있다 - target 확보까지만
        // 허용하고, 후보 생성이 실제로 막히면 아래 ResolveManeuver 직전에서
        // UNSUPPORTED_BEHAVIOR로 막는다(AVOID와 동일한 이유).
        case LANE_CHANGE_LEFT:
        case LANE_CHANGE_RIGHT:
            return true;

        default:
            return false;
    }
}

bool IsValidOptionalSpeed(bool has_value, double value) {
    if (!has_value) return true;
    return std::isfinite(value) && value >= 0.0;
}

// 모드 whitelist + 모드별 필수 필드 검증. 실패 시 reason으로 원인을 구분해
// 호출자가 "즉시 정지"와 "이전 궤적 재사용"을 나눌 수 있게 한다.
bool ValidateExternalCommand(const PlannerCommand& command,
                             FrenetPlanner::PlanFailureReason& reason) {
    if (!IsSupportedExternalMode(command.mode)) {
        reason = FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR;
        return false;
    }
    if(command.recovery_clear &&
       ((command.mode!=LANE_KEEPING && command.mode!=AVOID) ||
        !std::isfinite(command.recovery_min_end_s))) {
        reason=FrenetPlanner::PlanFailureReason::INVALID_COMMAND;return false;
    }
    const auto& signal=command.signal_stop;
    const bool native_signal=signal.target_revision || signal.key.route_generation ||
        !signal.key.stop_line.empty() || !signal.key.movement.empty();
    if(native_signal && command.mode!=EMERGENCY &&
       (command.mode!=STOP || !signal.valid() ||
        !std::isfinite(signal.physical_line.x) || !std::isfinite(signal.physical_line.y) ||
        !std::isfinite(signal.approach.x) || !std::isfinite(signal.approach.y) ||
        std::abs(std::hypot(signal.approach.x,signal.approach.y)-1)>1e-5 ||
        !std::isfinite(signal.position_error) || signal.position_error<0 ||
        !std::isfinite(signal.clearance) || signal.clearance<0)) {
        // A partial native constraint must not silently become a legacy STOP.
        reason=FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // 전 모드 공통: 외부가 지정한 속도는 유한하고 음수가 아니어야 한다.
    if (!IsValidOptionalSpeed(command.has_target_speed, command.target_speed) ||
        !IsValidOptionalSpeed(command.has_max_speed, command.max_speed)) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // EMERGENCY는 stop_position을 요구하지 않는다. BehaviorCommand가 목표점
    // 없이(has_target_pose=false) 즉시 긴급정지를 지시하므로 정지 위치 계산
    // 소유권이 Frenet에 있다(계약 8.1절). 아래 STOP 검증을 EMERGENCY에
    // 적용하면 정상 긴급정지가 INVALID_COMMAND로 거부된다.

    // FOLLOWING: 추종 대상이 지정되어야 한다. v2에서 대상 선택 권한은 FSM에
    // 있으므로(BehaviorCommand.primary_object_id) planner가 스스로 고르지
    // 않는다. -1이면 무엇을 따라갈지 알 수 없다.
    if (command.mode == FOLLOWING && command.target_object_id < 0) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // AVOID: 회피 대상이 지정되어야 한다. FOLLOWING과 같은 이유다 - v2에서
    // 대상 선택 권한은 FSM에 있고(primary_object_id), planner가 스스로
    // 고르지 않는다(D6-1).
    if (command.mode == AVOID && command.target_object_id < 0) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // LANE_CHANGE: target_pose_map 투영 결과가 있어야 한다. AVOID의
    // target_object_id 검증과 같은 이유다 - v2에서 has_target_pose는 항상
    // true로 강제되므로(계약 3.3절), has_target_pose_projection=false는
    // 투영 실패를 뜻한다. 이 투영값은 core의 authoritative 실행 목표이므로
    // (PlannerCommand 주석 참고) 투영조차 되지 않는 pose는 애초에 이
    // 참조선 근방의 명령이 아니라는 뜻이므로 거부한다. 투영 결과 값의
    // 유한성도 확인한다 - node의 ProjectPointToFrenet은 유한한 값만
    // 반환하지만, 다른 호출자(테스트)가 has_..._projection=true와 함께
    // NaN을 넣는 경우까지 방어한다.
    if ((command.mode == LANE_CHANGE_LEFT ||
         command.mode == LANE_CHANGE_RIGHT) &&
        (!command.has_target_pose_projection ||
         !std::isfinite(command.target_pose_projection_d) ||
         !std::isfinite(command.target_pose_projection_s))) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // STOP: 정지 목표 s가 실제로 지정되어야 한다. 기본값 NaN이 그대로 오면
    // GenerateStoppingCandidates가 NaN 경계조건으로 후보를 만들게 된다.
    if (command.mode == STOP) {
        if (!std::isfinite(command.stop_position) || command.stop_position < 0.0) {
            reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
            return false;
        }
    }

    reason = FrenetPlanner::PlanFailureReason::NONE;
    return true;
}

}  // namespace

bool FrenetPlanner::Plan(const CartesianState& ego,
                         const std::vector<TrackedObject>& obstacles,
                         const PlannerCommand& command,
                         CartesianPath& out_path) {
    if (command.mode != MERGE) ResetRoundaboutMerge(command.mode == LANE_KEEPING);
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        speed_profiler_.Reset();
        last_profiled_acceleration_ = 0.0;
        last_failure_reason_ = reason;
        return false;
    }
    const bool valid=PlanInternal(ego, obstacles, &command,
                        ExternalPlanningPurpose::NORMAL, out_path, nullptr);
    if(valid && command.mode==STOP && command.signal_stop.valid()) {
        // A checked braking path can still violate its signal constraint.
        // Preserve the braking geometry but expose failure independently of
        // Plan() success; the (currently gated) owner must escalate braking.
        for(std::size_t i=0;i<out_path.x.size();++i) {
            if(i>=out_path.y.size() || i>=out_path.yaw.size() || i>=out_path.v.size() ||
               i>=out_path.a.size()) {stop_.goal_failed=true;break;}
            CartesianState sample{};sample.x=out_path.x[i];sample.y=out_path.y[i];
            sample.yaw=out_path.yaw[i];sample.v=out_path.v[i];sample.a=out_path.a[i];
            if(!SignalStopFootprintBeforeLine(sample,command.signal_stop)) {
                stop_.goal_failed=true;break;
            }
            double sample_s=0;
            if(!ProjectPointToS(sample.x,sample.y,command.stop_position,sample_s) ||
               sample_s>command.stop_position-stop_cfg_.stop_buffer+
                            stop_cfg_.position_tolerance) {
                // A road-valid fallback before the physical line can still
                // overshoot this selected endpoint; keep both results apart.
                stop_.goal_failed=true;break;
            }
        }
    }
    if(valid && command.recovery_clear) {
        double end_s=0;
        if(out_path.x.empty() || !ProjectPointToS(out_path.x.back(),out_path.y.back(),command.recovery_min_end_s,end_s) ||
           end_s<command.recovery_min_end_s ||
           !avoid_road_map_.EvaluateRecoveryPath(out_path,vehicle_shape_,path_cfg_.dt).drivable) {
            last_failure_reason_=PlanFailureReason::ROAD_BOUNDARY_BLOCKED;return false;
        }
    }
    return valid;
}

bool FrenetPlanner::PrepareAvoid(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command, CartesianPath& maintenance_path,
    AvoidPrepareResult& result) {
    result = AvoidPrepareResult{};
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    // PREPARE는 AVOID 전용이다. 다른 모드로 호출되면 호출자의 실수다.
    if (command.mode != AVOID) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    return PlanInternal(ego, obstacles, &command,
                        ExternalPlanningPurpose::AVOID_PREPARE,
                        maintenance_path, &result);
}

bool FrenetPlanner::PlanCommittedAvoid(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command, CartesianPath& out_path,
    AvoidPrepareResult& result) {
    result = AvoidPrepareResult{};
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    if (command.mode != AVOID ||
        !execution_->avoid.active || !execution_->avoid.committed) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    return PlanInternal(ego, obstacles, &command,
                        ExternalPlanningPurpose::AVOID_COMMITTED,
                        out_path, &result);
}

bool FrenetPlanner::PrepareLaneChange(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command, CartesianPath& maintenance_path,
    LaneChangePrepareResult& result) {
    result = LaneChangePrepareResult{};
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    if (command.mode != LANE_CHANGE_LEFT && command.mode != LANE_CHANGE_RIGHT) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    return PlanInternal(ego, obstacles, &command,
                        ExternalPlanningPurpose::LANE_CHANGE_PREPARE,
                        maintenance_path, nullptr, &result);
}

bool FrenetPlanner::PlanCommittedLaneChange(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& command, CartesianPath& out_path,
    LaneChangePrepareResult& result) {
    result = LaneChangePrepareResult{};
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    if ((command.mode != LANE_CHANGE_LEFT &&
         command.mode != LANE_CHANGE_RIGHT) ||
        !execution_->lane_change.active || !execution_->lane_change.committed) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    return PlanInternal(ego, obstacles, &command,
                        ExternalPlanningPurpose::LANE_CHANGE_COMMITTED,
                        out_path, nullptr, &result);
}

FrenetPlanner::PlanningContext FrenetPlanner::PreparePlanningContext(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand& external_command) {
    PlanningContext ctx;

    CartesianToFrenet(ref_, ego, ctx.s, ctx.s_dot, ctx.s_ddot, ctx.d,
                       ctx.d_prime, ctx.d_pprime,
                       has_last_s_ ? &last_s_ : nullptr,
                       ref_config_.ego_projection_search_window);
    last_s_ = ctx.s;
    has_last_s_ = true;

    // The OBB angle can converge while a car drives straight. Fit motion
    // direction on unique sensor frames using their actual timestamps.
    const double observation_time = std::isfinite(object_observation_time_) &&
        object_observation_time_ >= 0 ? object_observation_time_ :
        std::max(0.0, object_motion_time_) + replanning_period_s_;
    if (observation_time < object_motion_time_ - 1e-6) {
        object_tracks_.clear();v2_following_=FollowingContext{};execution_.resetFollow();
    }
    object_motion_time_ = observation_time;
    const bool highway_normal=(external_command.mode==LANE_KEEPING||external_command.mode==FOLLOWING)&&
        (highway_prediction_map_ || (highway_route_&&HighwayApproachAt(*highway_route_,ego,&ctx.s).valid));
    const bool highway_observation=highway_normal||external_command.mode==LANE_CHANGE_LEFT||
        external_command.mode==LANE_CHANGE_RIGHT||(external_command.mode==MERGE&&
        external_command.merge_type==MergeType::HIGHWAY);
    std::unordered_map<int, double> object_yaw_rates;
    for (auto& item : object_tracks_) ++item.second.missing_cycles;
    for (const auto& obj : obstacles) {
        auto& track = object_tracks_[obj.id];
        track.Observe(obj, observation_time, object_tracking_cfg_.yaw_rate_clamp_rad_s,
            object_tracking_cfg_.min_motion_direction_change_rad,
            highway_observation?12:6,highway_observation?.8:.5,highway_observation);
        track.missing_cycles = 0;
        object_yaw_rates[obj.id] = track.yaw_rate;
        if(track.continuity_reset&&execution_->follow.leader_id==obj.id) {
            v2_following_=FollowingContext{};execution_.resetFollow();
        }
    }
    for (auto it = object_tracks_.begin(); it != object_tracks_.end();) {
        if (it->second.missing_cycles > object_tracking_cfg_.missing_cycles_eviction)
            it = object_tracks_.erase(it);
        else ++it;
    }
    // obstacles의 각 TrackedObject는 이미 authoritative corners(perception의
    // obb_corners)를 갖고 있다 - 여기서 다시 만들지 않는다. yaw_rate만
    // 연속 관측으로 갱신한 사본을 만들어 motion과 collision geometry 양쪽에
    // 그대로 쓴다(더 이상 두 개의 병렬 vector로 나누지 않는다).
    ctx.predicted_obstacles = obstacles;
    for (auto& obj : ctx.predicted_obstacles) {
        const auto it = object_yaw_rates.find(obj.id);
        obj.yaw_rate = it == object_yaw_rates.end() ? 0.0 : it->second;
        if(highway_normal) {
            const auto& map=highway_prediction_map_?highway_prediction_map_:highway_route_->map;
            const auto heading=map->laneHeading(obj.x,obj.y,obj.vx,obj.vy);
            const auto track=object_tracks_.find(obj.id);
            obj=PrepareHighwayObjectPrediction(obj,track==object_tracks_.end()?nullptr:&track->second,
                highway_merge_cfg_,nullptr,heading.value_or(NAN),
                object_tracking_cfg_.min_motion_direction_change_rad);
            obj=PrepareHighwayLaneMotion(obj,track==object_tracks_.end()?nullptr:&track->second,
                highway_traffic_lanes_,highway_merge_cfg_.motion_fit_min_span_s,
                highway_merge_cfg_.min_heading_alignment);
            obj=PrepareHighwayUnresolvedMotion(obj,track==object_tracks_.end()?nullptr:&track->second,
                highway_merge_cfg_,heading.value_or(NAN),ego,vehicle_shape_);
            // Ordinary KEEP/FOLLOW keeps its existing collision margin. The
            // departure-only longitudinal swept interval is not imported.
            obj.prediction_min_speed=obj.prediction_max_speed=NAN;
        }
    }

    if(roundabout_exit_prediction_active_&&external_command.mode==LANE_KEEPING) {
        if(ctx.s<roundabout_zone_.rear_clear_s-1||ctx.s>roundabout_zone_.rear_clear_s+20) {
            roundabout_exit_prediction_active_=false;roundabout_prediction_.Reset();
        } else for(auto& object:ctx.predicted_obstacles)
            object.yaw_rate=roundabout_prediction_.Rate(object.id);
    }

    // Motion history uses original sensor frames above. Collision predictions
    // start at the ego observation time, rather than mixing a current ego with
    // a delayed object pose. Advance only this derived vector, exactly once.
    if (std::isfinite(object_evaluation_time_) && object_evaluation_time_ > observation_time) {
        const double age = object_evaluation_time_ - observation_time;
        for (auto& object : ctx.predicted_obstacles)
            object = AdvanceObjectObservation(object, age);
    }

    // TargetSpeedResolver는 공간/상위계층 제한을 합친 raw base cap만
    // 계산한다. KEEP의 cycle-to-cycle 연속성은 candidate cap까지 확인한 뒤
    // SpeedProfiler가 단독으로 적용한다.
    TargetSpeedInput target_speed_input;
    target_speed_input.ego_s = ctx.s;
    target_speed_input.ego_speed = ego.v;
    target_speed_input.has_external_target_speed = external_command.has_target_speed;
    target_speed_input.external_target_speed = external_command.target_speed;
    target_speed_input.has_external_max_speed = external_command.has_max_speed;
    target_speed_input.external_max_speed = external_command.max_speed;
    ctx.resolved_target_speed = ResolveTargetSpeed(
        ref_, target_speed_input,
        CurveSpeedFor(external_command),
        ResolveKeepCruiseLimits(external_command, keep_cruise_cfg_, limits_));

    last_d_ = ctx.d;

    // FOLLOW가 아닌 외부 명령으로 넘어가면 v2 FOLLOW 상태를 즉시 비운다.
    //
    // **모든 early-return보다 앞에서** 해야 한다. PlanInternal의 EMERGENCY
    // 분기와 저속 fallback은 명령 조립 지점에 도달하기 전에 return하므로,
    // 그곳에서만 reset하면 context가 살아남는다. 그러면
    //
    //   FOLLOW id=10 target=15   -> 15 확보
    //   E-STOP id=11             -> context가 남는다
    //   FOLLOW id=12 target=15   -> 15가 관측되지 않아도 GRACE
    //
    // 가 되어, 첫 binding 실패인데 유예를 받아 존재하지 않는 leader를
    // 예측 추종한다(실측으로 재현 확인).
    if (external_command.mode != FOLLOWING) {
        v2_following_ = FollowingContext{};
        execution_.resetFollow();
    }
    // v2 AVOID 상태도 같은 이유로 같은 위치에서 비운다(D6-1). 여기서
    // reset하지 않으면
    //   AVOID id=20 target=21   -> 21 확보
    //   E-STOP id=21            -> context가 남는다
    //   AVOID id=22 target=21   -> 21이 관측되지 않아도 GRACE
    // 가 되어 첫 binding 실패인데 유예를 받는다(D5-2에서 FOLLOW로 실측 확인한
    // 것과 같은 구조).
    if (external_command.mode != AVOID) {
        v2_avoidance_ = AvoidanceContext{};
        execution_.resetAvoid();
        // 목표 목록도 함께 비운다(D6-2a). 남겨두면 다음 AVOID가 확보에
        // 실패했는데 이전 사이클의 목표로 후보를 만들 수 있다.
        v2_avoid_targets_ = AvoidTargetPlan{};
    }
    // v2 LANE_CHANGE 상태도 같은 이유로 같은 위치에서 비운다. AVOID와
    // 동일한 구조 - 다른 mode로 전환된 뒤에도 context가 남으면 다음
    // LANE_CHANGE가 stale 목표로 유예를 받는다.
    if (external_command.mode != LANE_CHANGE_LEFT &&
        external_command.mode != LANE_CHANGE_RIGHT) {
        v2_lane_change_ = LaneChangeContext{};
        execution_.resetLaneChange();
    }

    return ctx;
}

bool FrenetPlanner::PlanInternal(
    const CartesianState& ego, const std::vector<TrackedObject>& obstacles,
    const PlannerCommand* external_command,
    ExternalPlanningPurpose purpose, CartesianPath& out_path,
    AvoidPrepareResult* avoid_prepare_result,
    LaneChangePrepareResult* lane_change_prepare_result) {
    // purpose가 계산 내용을 정한다. 포인터 유무로 판단하지 않는다 - 진단용
    // 으로 포인터만 넘겨도 동작이 바뀌면 안 된다(헤더 주석 참고).
    const bool avoid_prepare =
        purpose == ExternalPlanningPurpose::AVOID_PREPARE;
    const bool avoid_committed =
        purpose == ExternalPlanningPurpose::AVOID_COMMITTED;
    const bool lane_change_prepare =
        purpose == ExternalPlanningPurpose::LANE_CHANGE_PREPARE;
    const bool lane_change_committed =
        purpose == ExternalPlanningPurpose::LANE_CHANGE_COMMITTED;
    last_failure_reason_ = PlanFailureReason::NONE;
    last_candidate_paths_.clear();
    last_candidate_validity_.clear();
    // General SpeedProfiler is a normal external KEEP state.  It must not
    // leak acceleration history through STOP/EMERGENCY/fail-safe or any other
    // maneuver.  It is intentionally not tied to command_id, so consecutive
    // KEEP commands preserve continuity across speed-cap transactions.
    if (external_command->mode != LANE_KEEPING) {
        speed_profiler_.Reset();
        last_profiled_acceleration_ = 0.0;
    }
    const bool departure_mode=external_command->mode==LANE_KEEPING ||
        (external_command->mode==STOP&&external_command->merge_type==MergeType::NONE);
    // Entry permission belongs to a departure, not every slow sample of an
    // existing maneuver. In particular, ordinary STOP braking must not change
    // policy merely because it slows beside a raster boundary near its goal.
    const bool departure_entry=!has_last_s_||last_mode_!=external_command->mode;
    if(!departure_mode||!avoid_road_map_.enabled()||!std::isfinite(ego.x)||
       !std::isfinite(ego.y)||!std::isfinite(ego.yaw)||!std::isfinite(ego.v)) {
        departure_road_recovery_ = DepartureRoadRecovery{};
    } else if(departure_road_recovery_.active) {
        const double movement=std::hypot(ego.x-departure_road_recovery_.last_x,
                                         ego.y-departure_road_recovery_.last_y);
        departure_road_recovery_.last_x=ego.x;departure_road_recovery_.last_y=ego.y;
        auto settled_shape=vehicle_shape_;
        settled_shape.width=std::max(settled_shape.width,settled_shape.road_outer_width)+.10;
        settled_shape.length+=.10;settled_shape.rear_overhang+=.05;
        departure_road_recovery_.settled_travel_m=avoid_road_map_.FootprintInside(
            ego.x,ego.y,ego.yaw,settled_shape)
            ? departure_road_recovery_.settled_travel_m+movement : 0.0;
        if(departure_road_recovery_.settled_travel_m>=.5)
            departure_road_recovery_ = DepartureRoadRecovery{};
    }
    if (!ref_loaded_) {
        speed_profiler_.Reset();
        last_profiled_acceleration_ = 0.0;
        last_failure_reason_ = PlanFailureReason::NOT_INITIALIZED;
        return false;
    }

    if(departure_mode&&departure_entry&&avoid_road_map_.enabled()&&!departure_road_recovery_.active&&
       std::isfinite(ego.v)&&std::fabs(ego.v)<limits_.near_low_speed_fallback_v) {
        auto exterior=vehicle_shape_;
        exterior.width=std::max(exterior.width,exterior.road_outer_width);
        if(!avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,exterior)&&
           avoid_road_map_.IntersectsDrivable(MakeEgoFootprint(ego,exterior,0),true)) {
            departure_road_recovery_.active=true;
            departure_road_recovery_.last_x=ego.x;departure_road_recovery_.last_y=ego.y;
            departure_road_recovery_.settled_travel_m=0.0;
        }
    }

    if (avoid_road_map_.enabled()) {
        const auto now = std::chrono::steady_clock::now();
        const bool contact = avoid_road_map_.PoseTouchesSolidPaint(
            ego.x, ego.y, ego.yaw, vehicle_shape_);
        if (contact) {
            if (!solid_paint_contact_active_) {
                solid_paint_contact_started_at_ = now;
                solid_paint_contact_active_ = true;
            }
            solid_paint_contact_elapsed_s_ = std::chrono::duration<double>(
                now - solid_paint_contact_started_at_).count();
        } else {
            solid_paint_contact_active_ = false;
            solid_paint_contact_elapsed_s_ = 0.0;
        }
    }

    // ===== 공용 준비(§3): Cartesian->Frenet, object tracking/예측, target
    // speed 해석/연속성, 모드 전환 시 v2 FOLLOW/AVOID context reset. =====
    PlanningContext ctx = PreparePlanningContext(ego, obstacles, *external_command);
    const double s = ctx.s;
    const double s_dot = ctx.s_dot;
    double s_ddot = ctx.s_ddot;
    const double d = ctx.d;
    const double d_prime = ctx.d_prime;
    const double d_pprime = ctx.d_pprime;
    std::vector<TrackedObject>& predicted_obstacles = ctx.predicted_obstacles;
    const double resolved_target_speed = ctx.resolved_target_speed;
    // maneuver가 KEEP 공용 pipeline에 도달하지 않는 cycle에도 이전 KEEP의
    // candidate-aware 진단이 남아 있지 않게 base/reset 값을 먼저 기록한다.
    last_base_target_speed_ = resolved_target_speed;
    last_raw_speed_cap_ = resolved_target_speed;
    last_candidate_curvature_speed_cap_ = -1.0;
    last_candidate_max_abs_curvature_ = -1.0;
    last_candidate_max_lateral_accel_ = -1.0;
    last_candidate_aware_regenerations_ = 0;

    // ===== 이 아래부터는 모드별 dispatch다: EMERGENCY -> AVOID 대상 확보 ->
    // 저속 fallback -> FrenetState boundary 구성 -> AVOID PREPARE/COMMITTED
    // -> switch(mode)의 KEEP/FOLLOW/STOP Process. =====

    // 저속(정지 근처)에서는 quintic 후보/곡률필터 파이프라인 전체를 건너뛰고
    // fallback으로 직접 경로를 만든다 (클래스 헤더 주석 참고 - 나누기 없는 안전한
    // 경로). AVOID/장애물 회피는 이 fallback에서는 다루지 않는다 - 정지 근처
    // 속도로는 위험한 회피 기동이 필요할 상황이 원래 적고, 속도가 오르면 바로
    // 아래의 정상 파이프라인으로 자동 복귀한다. 문턱은 limits_.low_speed_fallback_v.

    // =====================================================================
    // 외부 EMERGENCY_STOP: 후보 파이프라인을 건너뛰고 직접 감속한다.
    //
    // BehaviorCommand는 목표점 없이(has_target_pose=false) 즉시 긴급정지를
    // 지시하므로 정지 위치 계산 소유권이 Frenet에 있다(계약 8.1절).
    //
    // ResolveManeuver / 곡률 필터 / 충돌 필터로 내려보내지 않는 이유:
    //   1. stop_position이 NaN이므로 GenerateStoppingCandidates가 NaN
    //      경계조건으로 후보를 만든다.
    //   2. 후보가 전부 탈락하면 "긴급정지인데 아무 궤적도 못 보냄"이 된다.
    //      긴급정지에서 그 실패 방향 자체를 제거하는 것이 목적이다.
    //
    // 저속 fallback보다 먼저 처리한다. 그 fallback은 limits_.low_speed_min_creep_speed로 기어서
    // 전진하는 경로라 긴급정지 중에 차량이 계속 움직이게 된다.
    //
    // 차량이 이미 멈춰 있으면(v0 ~= 0) 0속도 궤적이 나온다. 새 정상 명령이
    // 올 때까지 긴급정지 상태를 계속 강제하는 것이 E-stop의 의미이므로,
    // 여기서 COMPLETED로 올리지 않는다 - ROS 계층이 EXECUTING을 유지한다.
    if (external_command->mode == EMERGENCY) {
        return EmergencyStopProcess(ego, s, d, s_dot, out_path);
    }

    // =====================================================================
    // 외부 AVOID 대상 확보 (D6-2a)
    //
    // ===== 왜 여기인가 =====
    //
    // D6-1에서는 이 처리가 명령 조립 지점(아래 external_command 분기)에
    // 있었다. 그런데 그 지점은 저속 fallback보다 **아래**다. 그래서
    // ego.v < kLowSpeedFallbackV(0.5 m/s)이면 fallback이 먼저 return해
    // resolver에 도달하지 않았다. 실측(AVOID target=21, 장애물 25 m 전방):
    //
    //   ego_v  Plan()  active_avoid_target_id  failure_reason
    //   0.60   false   21                      UNSUPPORTED_BEHAVIOR
    //   0.40   true    -1                      NONE        <- 확보 없이 성공
    //   0.00   true    -1                      NONE
    //
    // 저속 세 행은 Plan()이 성공을 반환하면서 대상을 확보하지 않았고, 나오는
    // 궤적은 fallback_target_d=0.0의 중앙 크롤링 경로다. gate를 열면 FSM
    // 입장에서 "회피 성공"으로 보이면서 장애물 쪽으로 기어간다.
    //
    // 그래서 확보를 저속 판단보다 위로 올린다. 장애물 앞에서 감속한 뒤
    // AVOID 명령을 받는 것은 오히려 **정상적인 시나리오**이므로, 그때
    // 대상조차 확보하지 못하는 구조는 맞지 않다(계약 12.7.2절).
    //
    // 확보만 여기서 하고, "그 목표로 실제 후보가 나오는가"는 D6-2b다.
    if (external_command->mode == AVOID) {
        if (!PrepareExternalAvoidTargets(s, obstacles, *external_command,
                                         avoid_prepare, avoid_committed)) {
            return false;
        }
    }
    // PREPARE인데 AVOID가 아니면 호출자의 실수다. PrepareAvoid()가 이미
    // 막지만, PlanInternal 내부에서도 조용히 다른 모드를 준비하지 않는다.
    if ((avoid_prepare || avoid_committed) && external_command->mode != AVOID) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // =====================================================================
    // 외부 LANE_CHANGE 목표 준비
    //
    // AVOID의 대상 확보와 같은 위치다 - 저속 fallback보다 먼저 처리해야
    // 감속 중에 LANE_CHANGE 명령을 받아도 목표를 정상적으로 준비한다
    // (AVOID 헤더 주석의 실측 사례와 동일한 이유).
    if (external_command->mode == LANE_CHANGE_LEFT ||
        external_command->mode == LANE_CHANGE_RIGHT) {
        if (!PrepareExternalLaneChangeTarget(
                s, d, *external_command, lane_change_prepare,
                lane_change_committed)) {
            return false;
        }
    }
    if ((lane_change_prepare || lane_change_committed) &&
        external_command->mode != LANE_CHANGE_LEFT &&
        external_command->mode != LANE_CHANGE_RIGHT) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    const bool external_stop = external_command->mode == STOP;
    // 외부 FOLLOW도 저속 fallback을 타지 않는다. 그 경로는 limits_.low_speed_min_creep_speed로
    // 기어서 전진하는데, 앞차가 정지해 있으면 그대로 다가가 붙는다. FOLLOW의
    // 종방향 목표는 선행차와의 gap이므로 정상 following 후보 생성기
    // (GenerateFollowingCandidates)로 보내야 한다 - 그 생성기는 leader가
    // 멈춰 있으면 min_gap을 유지하는 정지 목표를 만든다.
    const bool external_follow = external_command->mode == FOLLOWING;
    // AVOID PREPARE도 이 fallback을 타지 않는다 (D6-2b).
    //
    // PlanLowSpeedFallback 생성기 자체는 충돌 필터를 거치지 않는다. KEEP는
    // 전용 wrapper가 최종 객체를 검사한다. 저속 PREPARE의 유지 궤적은
    // BuildAvoidMaintenancePath가 **충돌 검사를 통과시킨 뒤** 만든다.
    //
    // 여기서 제외하지 않으면 PREPARE 블록에 도달하지도 못한다 - 이 분기가
    // 먼저 return하기 때문이다(실측으로 확인: v=0.3에서 low_speed_context이
    // 0으로 나오고 maintenance가 미검사 crawl이었다).
    bool blocked_low_speed_crawl = false;
    if(external_command->recovery_clear && external_command->mode==LANE_KEEPING) {
        speed_profiler_.Reset();last_profiled_acceleration_=0;last_mode_=LANE_KEEPING;
        const bool ok=PlanExternalKeepLowSpeedFallback(ego,s,d,std::min(limits_.low_speed_crawl_speed_cap,resolved_target_speed),
            predicted_obstacles,out_path,std::max(.1,external_command->recovery_min_end_s-s));
        if(ok) {last_candidate_paths_.push_back(out_path);last_candidate_validity_.push_back(true);}
        return ok;
    }
    if ((std::fabs(ego.v) < limits_.low_speed_fallback_v ||
         (external_command->mode==LANE_KEEPING&&departure_road_recovery_.active)) &&
        !external_command->recovery_clear &&
        !external_stop && !external_follow && !avoid_prepare &&
        !avoid_committed && !lane_change_prepare && !lane_change_committed) {
        static int low_speed_log_tick = 0;
        if (++low_speed_log_tick % 5 == 0) {
            std::printf("[FrenetPlanner-STATE] mode=LOW_SPEED_FALLBACK "
                        "s=%.2f d=%.3f speed=%.2f candidates=N/A\n",
                        s, d, ego.v);
        }
        last_mode_ = external_command->mode;
        // This bypass is not the normal longitudinal candidate generator.
        // Re-entering normal KEEP must initialize from the then-current ego
        // speed instead of stale pre-fallback profile state.
        speed_profiler_.Reset();
        last_profiled_acceleration_ = 0.0;
        const bool keep_fallback = external_command->mode == LANE_KEEPING;
        const bool fallback_ok = keep_fallback
            ? PlanExternalKeepLowSpeedFallback(
                  ego, s, d, resolved_target_speed, predicted_obstacles,out_path)
            : PlanLowSpeedFallback(
                  ego, s, d, 0.0, out_path,
                  limits_.low_speed_crawl_duration_s,
                  resolved_target_speed);
        if (fallback_ok) {
            last_target_speed_ = out_path.v.empty()
                ? 0.0 : out_path.v.back();
            last_candidate_paths_.push_back(out_path);
            last_candidate_validity_.push_back(true);
        }
        if (fallback_ok || !keep_fallback ||
            last_failure_reason_ != PlanFailureReason::COLLISION_BLOCKED) {
            return fallback_ok;
        }
        // A rejected slow crawl does not establish that every physically
        // reachable departure is blocked. Continue through the same prepared
        // observation/state into KEEP's fully checked candidate pipeline.
        // No second PlanInternal/tracking update and no rejected-path reuse.
        blocked_low_speed_crawl = true;
    }

    double d_dot, d_ddot;
    ArcDerivToTimeDeriv(s_dot, s_ddot, d_prime, d_pprime, d_dot, d_ddot);
    last_d_dot_ = d_dot;  // 클램프 전 원본 값 - 진단 목적(아래 kMaxDDot 클램프 폭 자체를 보려고)

    // R7-C: 물리적 boundary clamp(투영 오차로 인한 비현실적 초기 가속도/
    // 횡속도 제한)는 Generator/BoundaryConditionPolicy로 옮겼다 - raw 값은
    // 위에서 이미 last_d_dot_로 보존했고(clamp 이전), 여기서는 clamp된
    // planning용 초기조건만 만든다. 수식/파라미터 의미는 그대로다.
    //
    // s_ddot/d_dot/d_ddot 지역변수는 예전에는 clip()이 제자리에서 덮어썼다 -
    // 아래 FOLLOW/STOP/KEEP Process 호출부가 start와 별도로 이 값들을 받으므로
    // (§16: raw와 planning state를 섞지 않되, 여기서는 두 호출부 모두
    // clamp-이후 값을 기대하는 기존 계약을 그대로 보존한다), start 생성 후
    // 그 clamp된 필드로 재대입한다.
    const FrenetState start = ClampInitialBoundaryState(
        s, s_dot, s_ddot, d, d_dot, d_ddot,
        ResolveKeepCruiseLimits(*external_command, keep_cruise_cfg_, limits_));
    s_ddot = start.s_dd;
    d_dot = start.d_d;
    d_ddot = start.d_dd;

    // Highway admission/continuation belongs to PlanHighwayTransitionImpl.
    // Its actual candidate already checks source approach, target occupancy
    // and rear response; KEEP must not run a second legacy gap/STOP selector.

    // =====================================================================
    // AVOID PREPARE: 후보 평가와 유지 궤적을 한 사이클에서 만든다 (D6-2b)
    //
    // 계약 5.2절. 두 결과를 절대 같은 것으로 취급하지 않는다:
    //   candidate  "AVOID를 실행해도 되는가"     -> result.candidate_valid
    //   out_path   "지금 MPC가 따라갈 것"        -> maintenance
    //
    // 위쪽 전처리(Cartesian->Frenet, object tracking/prediction)를 **한 번만**
    // 통과했다. PlanInternal을 두 번 부르면 last_s_ / missing_cycles /
    // yaw-rate 재추정이 중복된다(계약 12.7.1절).
    // =====================================================================
    if (avoid_prepare || avoid_committed) {
        return AvoidProcess(
            ego, start, s, d, resolved_target_speed,
            predicted_obstacles, *external_command,
            avoid_prepare, avoid_committed, out_path, *avoid_prepare_result);
    }

    // =====================================================================
    // LANE_CHANGE PREPARE/COMMITTED (D7). AVOID와 동일한 구조 - 후보(candidate)와
    // 유지 궤적(maintenance)을 절대 같은 것으로 취급하지 않는다.
    // =====================================================================
    if (lane_change_prepare || lane_change_committed) {
        return LaneChangeProcess(
            ego, start, s, d, resolved_target_speed,
            predicted_obstacles, *external_command,
            lane_change_prepare, lane_change_committed, out_path,
            *lane_change_prepare_result);
    }

    // 여기 도달하면 external_command->mode는 IsSupportedExternalMode의
    // whitelist와 위쪽 EMERGENCY/AVOID/LANE_CHANGE 분기에 의해 반드시
    // {LANE_KEEPING, STOP, FOLLOWING} 중 하나다. Plan()이 validation ->
    // 공용 준비(위) -> switch(mode) -> Process() -> 공용 후처리
    // (ExecuteManeuverPipeline 내부)로 읽히도록 각 모드의 명시적 진입점으로
    // 보낸다(§6).
    switch (external_command->mode) {
        case FOLLOWING:
            return FollowProcess(
                ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
                predicted_obstacles, resolved_target_speed,
                *external_command, out_path);
        case STOP:
            return StopProcess(
                ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
                predicted_obstacles, resolved_target_speed,
                *external_command, out_path);
        default: {
            const bool planned = KeepProcess(
                ego, start, s, d, s_dot, s_ddot, d_dot, d_ddot, obstacles,
                predicted_obstacles, resolved_target_speed,
                *external_command, out_path, blocked_low_speed_crawl);
            if (!planned && blocked_low_speed_crawl) {
                out_path = CartesianPath{};
                last_failure_reason_ = PlanFailureReason::COLLISION_BLOCKED;
            }
            return planned;
        }
    }
}

