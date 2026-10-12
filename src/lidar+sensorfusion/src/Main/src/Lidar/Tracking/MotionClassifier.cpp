#include <Lidar/Tracking/MotionClassifier.hpp>
#include <Geometry/FrameBoundary.hpp>
#include <PlannerInterface/PlannerObjectAdapter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kMpsToKph = 3.6;
constexpr double kDirectionEpsilon = 1e-6;

double EstimateRigidAxisTranslation(double previous_min,
                                    double previous_max,
                                    double current_min,
                                    double current_max,
                                    double agreement_tolerance) {
    const double min_delta = current_min - previous_min;
    const double max_delta = current_max - previous_max;
    if (std::abs(min_delta - max_delta) <= agreement_tolerance) {
        return 0.5 * (min_delta + max_delta);
    }

    // Only one face moved: the OBB most likely grew/shrank as a newly visible
    // surface entered the cluster.  The quieter face is the lower-bound rigid
    // translation shared by the physical object.
    return std::abs(min_delta) <= std::abs(max_delta) ? min_delta : max_delta;
}

bool FillMapObbBounds(const LiDARObjectInfo& object,
                      const GPS_IMU& gps_imu,
                      const VehicleSpec& vehicle,
                      double* min_e, double* max_e,
                      double* min_n, double* max_n) {
    if(frame_boundary::Enabled()) {
        *min_e=*min_n=std::numeric_limits<double>::infinity();
        *max_e=*max_n=-std::numeric_limits<double>::infinity();
        for(const auto& p:object.OBB_Corners_Body) {
            const auto q=TransformLidarPointToMap({p.x,p.y,p.z},gps_imu,vehicle);
            if(!std::isfinite(q.x) || !std::isfinite(q.y)) return false;
            *min_e=std::min(*min_e,q.x); *max_e=std::max(*max_e,q.x);
            *min_n=std::min(*min_n,q.y); *max_n=std::max(*max_n,q.y);
        }
        return *max_e-*min_e>.05 && *max_n-*min_n>.05;
    }
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double roll = gps_imu.st_IMU.f32_Roll_deg * kDegToRad;
    const double pitch = gps_imu.st_IMU.f32_Pitch_deg * kDegToRad;
    const double yaw = gps_imu.st_IMU.f32_Yaw_deg * kDegToRad;
    const double cos_roll = std::cos(roll);
    const double sin_roll = std::sin(roll);
    const double cos_pitch = std::cos(pitch);
    const double sin_pitch = std::sin(pitch);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double r11 = cos_yaw * cos_pitch;
    const double r12 = cos_yaw * sin_pitch * sin_roll -
                       sin_yaw * cos_roll;
    const double r21 = sin_yaw * cos_pitch;
    const double r22 = sin_yaw * sin_pitch * sin_roll +
                       cos_yaw * cos_roll;

    *min_e = std::numeric_limits<double>::infinity();
    *max_e = -std::numeric_limits<double>::infinity();
    *min_n = std::numeric_limits<double>::infinity();
    *max_n = -std::numeric_limits<double>::infinity();
    for (const auto& corner : object.OBB_Corners_Body) {
        if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) return false;
        const double body_x = corner.x + vehicle.st_LiDARPos.f32_X -
                              vehicle.st_GPSPos.f32_X;
        const double body_y = corner.y + vehicle.st_LiDARPos.f32_Y -
                              vehicle.st_GPSPos.f32_Y;
        const double map_e = r11 * body_x + r12 * body_y +
                             gps_imu.st_GPS.f64_E;
        const double map_n = r21 * body_x + r22 * body_y +
                             gps_imu.st_GPS.f64_N;
        if (!std::isfinite(map_e) || !std::isfinite(map_n)) return false;
        *min_e = std::min(*min_e, map_e);
        *max_e = std::max(*max_e, map_e);
        *min_n = std::min(*min_n, map_n);
        *max_n = std::max(*max_n, map_n);
    }
    return *max_e - *min_e > 0.05 && *max_n - *min_n > 0.05;
}

MotionClassifierConfig LoadConfig(ros::NodeHandle& node) {
    MotionClassifierConfig config;
    int history_size = static_cast<int>(config.history_size);
    node.param("motion_history_size", history_size, history_size);
    node.param("motion_warmup_observations", config.warmup_observations,
               config.warmup_observations);
    node.param("motion_low_threat_confirmations",
               config.low_threat_confirmations,
               config.low_threat_confirmations);
    node.param("motion_dynamic_confirmations", config.dynamic_confirmations,
               config.dynamic_confirmations);
    node.param("motion_pending_static_confirmations",
               config.pending_static_confirmations,
               config.pending_static_confirmations);
    node.param("motion_dynamic_release_confirmations",
               config.dynamic_release_confirmations,
               config.dynamic_release_confirmations);
    node.param("motion_pending_max_frames", config.pending_max_frames,
               config.pending_max_frames);
    node.param("motion_change_confirmations", config.change_confirmations,
               config.change_confirmations);
    node.param("motion_short_miss_limit", config.short_miss_limit,
               config.short_miss_limit);
    node.param("motion_stale_track_frames", config.stale_track_frames,
               config.stale_track_frames);
    node.param("motion_lateral_threshold_kph",
               config.lateral_threshold_kph,
               config.lateral_threshold_kph);
    node.param("motion_longitudinal_threshold_kph",
               config.longitudinal_threshold_kph,
               config.longitudinal_threshold_kph);
    node.param("motion_fast_lateral_threshold_kph",
               config.fast_lateral_threshold_kph,
               config.fast_lateral_threshold_kph);
    node.param("motion_velocity_jump_kph", config.velocity_jump_kph,
               config.velocity_jump_kph);
    node.param("motion_direction_cosine_min",
               config.direction_cosine_min,
               config.direction_cosine_min);
    node.param("motion_longitudinal_displacement_kph",
               config.longitudinal_displacement_kph,
               config.longitudinal_displacement_kph);
    node.param("motion_lateral_displacement_kph",
               config.lateral_displacement_kph,
               config.lateral_displacement_kph);
    node.param("motion_exit_speed_kph", config.exit_speed_kph,
               config.exit_speed_kph);
    node.param("motion_edge_agreement_tolerance_m",
               config.edge_agreement_tolerance_m,
               config.edge_agreement_tolerance_m);

    config.history_size = static_cast<std::size_t>(std::max(3, history_size));
    config.warmup_observations = std::max(0, config.warmup_observations);
    config.low_threat_confirmations =
        std::max(1, config.low_threat_confirmations);
    config.dynamic_confirmations =
        std::max(1, config.dynamic_confirmations);
    config.pending_static_confirmations =
        std::max(1, config.pending_static_confirmations);
    config.dynamic_release_confirmations =
        std::max(1, config.dynamic_release_confirmations);
    config.pending_max_frames = std::max(1, config.pending_max_frames);
    config.change_confirmations =
        std::max(2, config.change_confirmations);
    config.short_miss_limit = std::max(0, config.short_miss_limit);
    config.stale_track_frames = std::max(1, config.stale_track_frames);
    config.lateral_threshold_kph =
        std::max(0.0, config.lateral_threshold_kph);
    config.longitudinal_threshold_kph =
        std::max(0.0, config.longitudinal_threshold_kph);
    config.fast_lateral_threshold_kph = std::max(
        config.lateral_threshold_kph,
        config.fast_lateral_threshold_kph);
    config.velocity_jump_kph = std::max(0.1, config.velocity_jump_kph);
    config.direction_cosine_min = std::max(
        -1.0, std::min(1.0, config.direction_cosine_min));
    config.longitudinal_displacement_kph =
        std::max(0.0, config.longitudinal_displacement_kph);
    config.lateral_displacement_kph =
        std::max(0.0, config.lateral_displacement_kph);
    config.exit_speed_kph = std::max(0.0, config.exit_speed_kph);
    config.edge_agreement_tolerance_m =
        std::max(0.0, config.edge_agreement_tolerance_m);
    return config;
}

}  // namespace

const char* MotionStateName(MotionState state) {
    switch (state) {
        case MotionState::LowThreat:
            return "LOW_THREAT";
        case MotionState::DynamicPending:
            return "PENDING";
        case MotionState::Dynamic:
            return "DYNAMIC";
        case MotionState::Unknown:
        default:
            return "UNKNOWN";
    }
}

MotionClassifier::MotionClassifier(const MotionClassifierConfig& config)
    : config_(config) {
    config_.history_size = std::max<std::size_t>(3, config_.history_size);
    config_.warmup_observations = std::max(0, config_.warmup_observations);
    config_.low_threat_confirmations =
        std::max(1, config_.low_threat_confirmations);
    config_.dynamic_confirmations =
        std::max(1, config_.dynamic_confirmations);
    config_.pending_static_confirmations =
        std::max(1, config_.pending_static_confirmations);
    config_.dynamic_release_confirmations =
        std::max(1, config_.dynamic_release_confirmations);
    config_.pending_max_frames = std::max(1, config_.pending_max_frames);
    config_.change_confirmations =
        std::max(2, config_.change_confirmations);
    config_.short_miss_limit = std::max(0, config_.short_miss_limit);
    config_.stale_track_frames = std::max(1, config_.stale_track_frames);
    config_.lateral_displacement_kph =
        std::max(0.0, config_.lateral_displacement_kph);
    config_.exit_speed_kph = std::max(0.0, config_.exit_speed_kph);
    config_.edge_agreement_tolerance_m =
        std::max(0.0, config_.edge_agreement_tolerance_m);
}

MotionClassifier::MotionClassifier(ros::NodeHandle& private_node)
    : MotionClassifier(LoadConfig(private_node)) {}

double MotionClassifier::Median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if (values.size() % 2 != 0) return upper;
    const double lower = *std::max_element(
        values.begin(), values.begin() + middle);
    return 0.5 * (lower + upper);
}

bool MotionClassifier::DirectionConsistent(
    const ChangeCandidate& reference, double delta_vx_kph,
    double delta_vy_kph, double minimum_cosine) {
    const double reference_norm = std::hypot(
        reference.delta_vx_kph, reference.delta_vy_kph);
    const double current_norm = std::hypot(delta_vx_kph, delta_vy_kph);
    if (reference_norm <= kDirectionEpsilon ||
        current_norm <= kDirectionEpsilon) {
        return false;
    }
    const double cosine =
        (reference.delta_vx_kph * delta_vx_kph +
         reference.delta_vy_kph * delta_vy_kph) /
        (reference_norm * current_norm);
    return cosine >= minimum_cosine;
}

void MotionClassifier::ResetDecision(TrackState& track) const {
    track.warmup.clear();
    track.accepted.clear();
    track.observations.clear();
    track.candidates.clear();
    track.valid_observations = 0;
    track.low_threat_streak = 0;
    track.dynamic_evidence_streak = 0;
    track.static_evidence_streak = 0;
    track.pending_frames = 0;
    track.state = MotionState::Unknown;
    track.state_before_pending = MotionState::Unknown;
    track.decision_vx_kph = 0.0;
    track.decision_vy_kph = 0.0;
    track.decision_speed_kph = 0.0;
    track.map_displacement_speed_kph = 0.0;
    track.corrected_displacement_speed_kph = 0.0;
    track.last_jump_kph = 0.0;
}

void MotionClassifier::AppendAccepted(
    TrackState& track, const VelocitySample& sample) const {
    track.accepted.push_back(sample);
    while (track.accepted.size() > config_.history_size) {
        track.accepted.pop_front();
    }
}

void MotionClassifier::AppendObservation(
    TrackState& track, const VelocitySample& sample) const {
    track.observations.push_back(sample);
    while (track.observations.size() > config_.history_size) {
        track.observations.pop_front();
    }
}

void MotionClassifier::UpdateAcceptedHistory(
    TrackState& track, const VelocitySample& sample) const {
    if (track.accepted.empty()) {
        AppendAccepted(track, sample);
        track.last_jump_kph = 0.0;
        return;
    }

    std::vector<double> vx_values;
    std::vector<double> vy_values;
    vx_values.reserve(track.accepted.size());
    vy_values.reserve(track.accepted.size());
    for (const auto& accepted : track.accepted) {
        vx_values.push_back(accepted.vx_kph);
        vy_values.push_back(accepted.vy_kph);
    }
    const double reference_vx = Median(std::move(vx_values));
    const double reference_vy = Median(std::move(vy_values));
    const double delta_vx = sample.vx_kph - reference_vx;
    const double delta_vy = sample.vy_kph - reference_vy;
    const double jump = std::hypot(delta_vx, delta_vy);
    track.last_jump_kph = jump;

    auto start_candidate = [&] {
        track.candidates.clear();
        track.candidates.push_back({sample, delta_vx, delta_vy});
    };

    if (jump <= config_.velocity_jump_kph) {
        track.candidates.clear();
        AppendAccepted(track, sample);
        return;
    }

    if (track.candidates.empty()) {
        start_candidate();
        return;
    }

    const auto& previous_candidate = track.candidates.back().sample;
    const double candidate_step = std::hypot(
        sample.vx_kph - previous_candidate.vx_kph,
        sample.vy_kph - previous_candidate.vy_kph);
    if (candidate_step > config_.velocity_jump_kph ||
        !DirectionConsistent(
            track.candidates.front(), delta_vx, delta_vy,
            config_.direction_cosine_min)) {
        // 기존 이력과 먼 것만으로는 부족하다. 새 후보들도 서로
        // velocity_jump_kph 이내여야 수렴한 새 속도로 본다. 아니면
        // 현재 표본을 새 후보의 첫 표본으로 교체하고 정상 이력은 보존한다.
        start_candidate();
        return;
    }

    track.candidates.push_back({sample, delta_vx, delta_vy});
    if (static_cast<int>(track.candidates.size()) <
        config_.change_confirmations) {
        return;
    }

    track.accepted.clear();
    for (const auto& candidate : track.candidates) {
        AppendAccepted(track, candidate.sample);
    }
    track.candidates.clear();
    track.low_threat_streak = 0;
}

void MotionClassifier::UpdateDecision(
    TrackState& track, const VelocitySample& raw_sample) const {
    if (track.accepted.empty()) {
        track.state = MotionState::Unknown;
        return;
    }

    std::vector<double> vx_values;
    std::vector<double> vy_values;
    vx_values.reserve(track.accepted.size());
    vy_values.reserve(track.accepted.size());
    for (const auto& sample : track.accepted) {
        vx_values.push_back(sample.vx_kph);
        vy_values.push_back(sample.vy_kph);
    }
    track.decision_vx_kph = Median(std::move(vx_values));
    track.decision_vy_kph = Median(std::move(vy_values));
    track.decision_speed_kph = std::hypot(
        track.decision_vx_kph, track.decision_vy_kph);

    track.map_displacement_speed_kph = 0.0;
    track.corrected_displacement_speed_kph = 0.0;
    if (track.observations.size() >= config_.history_size) {
        const auto& first = track.observations.front();
        const auto& last = track.observations.back();
        const double dt = last.stamp_sec - first.stamp_sec;
        if (dt > 1e-3 && std::isfinite(first.map_e) &&
            std::isfinite(first.map_n) && std::isfinite(last.map_e) &&
            std::isfinite(last.map_n)) {
            track.map_displacement_speed_kph = kMpsToKph * std::hypot(
                last.map_e - first.map_e, last.map_n - first.map_n) / dt;

            // Compare both OBB faces on each fixed map axis.  Rigid motion
            // shifts min/max together; visibility growth moves mainly one
            // face.  In the latter case EstimateRigidAxisTranslation keeps
            // the quieter face and removes the apparent center motion.
            if (first.obb_bounds_valid && last.obb_bounds_valid) {
                const double translation_e = EstimateRigidAxisTranslation(
                    first.obb_min_e, first.obb_max_e,
                    last.obb_min_e, last.obb_max_e,
                    config_.edge_agreement_tolerance_m);
                const double translation_n = EstimateRigidAxisTranslation(
                    first.obb_min_n, first.obb_max_n,
                    last.obb_min_n, last.obb_max_n,
                    config_.edge_agreement_tolerance_m);
                track.corrected_displacement_speed_kph =
                    kMpsToKph * std::hypot(translation_e, translation_n) / dt;
            } else {
                track.corrected_displacement_speed_kph =
                    track.map_displacement_speed_kph;
            }
        }
    }

    const double raw_abs_vy = std::abs(raw_sample.vy_kph);
    const bool displacement_ready =
        track.observations.size() >= config_.history_size;
    const bool lateral_motion_evidence =
        displacement_ready &&
        std::abs(track.decision_vy_kph) >= config_.lateral_threshold_kph &&
        track.corrected_displacement_speed_kph >=
            config_.lateral_displacement_kph;
    const bool longitudinal_motion_evidence =
        displacement_ready &&
        std::abs(track.decision_vx_kph) >=
            config_.longitudinal_threshold_kph &&
        track.corrected_displacement_speed_kph >=
            config_.longitudinal_displacement_kph;
    const bool dynamic_evidence =
        lateral_motion_evidence || longitudinal_motion_evidence;
    const bool fast_motion_hint =
        raw_abs_vy >= config_.fast_lateral_threshold_kph ||
        std::abs(raw_sample.vx_kph) >= config_.longitudinal_threshold_kph;
    const bool static_evidence =
        displacement_ready &&
        track.corrected_displacement_speed_kph <= config_.exit_speed_kph;

    if (track.state == MotionState::Dynamic) {
        track.dynamic_evidence_streak = 0;
        if (static_evidence) {
            ++track.static_evidence_streak;
            if (track.static_evidence_streak >=
                config_.dynamic_release_confirmations) {
                track.state = MotionState::LowThreat;
                track.static_evidence_streak = 0;
                track.low_threat_streak = config_.low_threat_confirmations;
            }
        } else {
            track.static_evidence_streak = 0;
        }
        return;
    }

    if (dynamic_evidence) {
        ++track.dynamic_evidence_streak;
        track.static_evidence_streak = 0;
        track.pending_frames = 0;
        track.low_threat_streak = 0;
        if (track.state != MotionState::DynamicPending) {
            track.state_before_pending = track.state;
        }
        if (track.dynamic_evidence_streak >=
            config_.dynamic_confirmations) {
            track.state = MotionState::Dynamic;
            track.dynamic_evidence_streak = 0;
            track.pending_frames = 0;
        } else {
            track.state = MotionState::DynamicPending;
        }
        return;
    }

    if (track.state == MotionState::DynamicPending) {
        track.dynamic_evidence_streak = 0;
        if (static_evidence) {
            ++track.static_evidence_streak;
            track.pending_frames = 0;
            if (track.static_evidence_streak >=
                config_.pending_static_confirmations) {
                track.state = track.state_before_pending == MotionState::Dynamic
                                  ? MotionState::Dynamic
                                  : MotionState::LowThreat;
                track.static_evidence_streak = 0;
                track.pending_frames = 0;
            }
        } else {
            track.static_evidence_streak = 0;
            // Only count frames for which a full geometry window exists but
            // neither STATIC nor DYNAMIC evidence is supported. Startup time
            // alone must never decide the state.
            if (displacement_ready) ++track.pending_frames;
        }
        if (track.state == MotionState::DynamicPending &&
            track.pending_frames >= config_.pending_max_frames) {
            // Timeout never votes STATIC/DYNAMIC.  It only prevents an
            // indefinite pending state when geometry and velocity disagree.
            track.state = MotionState::Unknown;
            track.dynamic_evidence_streak = 0;
            track.static_evidence_streak = 0;
            track.pending_frames = 0;
        }
        return;
    }

    // Once a full geometry window supports STATIC, a noisy tracker speed
    // alone must not immediately re-enter PENDING. A new velocity change
    // candidate still gets the fast conservative response. Rigid displacement
    // takes the dynamic-evidence branch above; startup remains conservative.
    if (fast_motion_hint && (!static_evidence || !track.candidates.empty())) {
        track.state_before_pending = track.state;
        track.state = MotionState::DynamicPending;
        track.dynamic_evidence_streak = 0;
        track.static_evidence_streak = 0;
        track.pending_frames = 0;
        track.low_threat_streak = 0;
        return;
    }

    if (static_evidence) {
        ++track.low_threat_streak;
        if (track.low_threat_streak >= config_.low_threat_confirmations) {
            track.state = MotionState::LowThreat;
        }
    } else {
        track.low_threat_streak = 0;
    }
}

void MotionClassifier::WriteDiagnostics(
    const TrackState& track, LiDARObjectInfo& object) const {
    object.f32_DecisionVxKPH =
        static_cast<float32>(track.decision_vx_kph);
    object.f32_DecisionVyKPH =
        static_cast<float32>(track.decision_vy_kph);
    object.f32_DecisionSpeedKPH =
        static_cast<float32>(track.decision_speed_kph);

    double planner_vx_kph = track.decision_vx_kph;
    double planner_vy_kph = track.decision_vy_kph;
    if (track.state == MotionState::LowThreat) {
        planner_vx_kph = 0.0;
        planner_vy_kph = 0.0;
    } else if (track.state == MotionState::DynamicPending &&
               !track.candidates.empty()) {
        std::vector<double> candidate_vx;
        std::vector<double> candidate_vy;
        candidate_vx.reserve(track.candidates.size());
        candidate_vy.reserve(track.candidates.size());
        for (const auto& candidate : track.candidates) {
            candidate_vx.push_back(candidate.sample.vx_kph);
            candidate_vy.push_back(candidate.sample.vy_kph);
        }
        const double alpha = std::min(
            1.0, static_cast<double>(track.candidates.size()) /
                     static_cast<double>(config_.change_confirmations));
        planner_vx_kph = (1.0 - alpha) * planner_vx_kph +
                         alpha * Median(std::move(candidate_vx));
        planner_vy_kph = (1.0 - alpha) * planner_vy_kph +
                         alpha * Median(std::move(candidate_vy));
    }
    object.f32_PlannerVxKPH = static_cast<float32>(planner_vx_kph);
    object.f32_PlannerVyKPH = static_cast<float32>(planner_vy_kph);
    object.f32_MapDisplacementSpeedKPH =
        static_cast<float32>(track.map_displacement_speed_kph);
    object.f32_CorrectedDisplacementSpeedKPH =
        static_cast<float32>(track.corrected_displacement_speed_kph);
    object.f32_MotionVelocityJumpKPH =
        static_cast<float32>(track.last_jump_kph);
    object.s32_MotionState = static_cast<std::int32_t>(track.state);
    object.s32_MotionValidObservations = track.valid_observations;
    object.s32_MotionAcceptedSamples =
        static_cast<std::int32_t>(track.accepted.size());
    object.s32_MotionCandidateSamples =
        static_cast<std::int32_t>(track.candidates.size());
    object.b_IsDynamic =
        track.state == MotionState::DynamicPending ||
        track.state == MotionState::Dynamic;
}

void MotionClassifier::Update(
    std::vector<LiDARObjectInfo>& objects, const ros::Time& stamp) {
    UpdateImpl(objects, stamp, nullptr, nullptr);
}

void MotionClassifier::Update(
    std::vector<LiDARObjectInfo>& objects, const ros::Time& stamp,
    const GPS_IMU& gps_imu, const VehicleSpec& vehicle) {
    UpdateImpl(objects, stamp, &gps_imu, &vehicle);
}

void MotionClassifier::UpdateImpl(
    std::vector<LiDARObjectInfo>& objects, const ros::Time& stamp,
    const GPS_IMU* gps_imu, const VehicleSpec* vehicle) {
    for (auto& entry : tracks_) ++entry.second.unseen_frames;

    for (auto& object : objects) {
        if (object.s32_Id < 0) continue;
        auto& track = tracks_[object.s32_Id];
        track.unseen_frames = 0;

        if (object.s32_MissCount != 0) {
            ++track.consecutive_misses;
            if (track.consecutive_misses > config_.short_miss_limit) {
                ResetDecision(track);
                track.consecutive_misses = object.s32_MissCount;
            }
            WriteDiagnostics(track, object);
            continue;
        }

        if (!std::isfinite(object.f32_AbsVx) ||
            !std::isfinite(object.f32_AbsVy) ||
            !std::isfinite(object.f32_CoreE) ||
            !std::isfinite(object.f32_CoreN)) {
            WriteDiagnostics(track, object);
            continue;
        }

        track.consecutive_misses = 0;
        ++track.valid_observations;
        VelocitySample sample;
        sample.stamp_sec = stamp.toSec();
        sample.vx_kph = static_cast<double>(object.f32_AbsVx) * kMpsToKph;
        sample.vy_kph = static_cast<double>(object.f32_AbsVy) * kMpsToKph;
        sample.map_e = static_cast<double>(object.f32_CoreE);
        sample.map_n = static_cast<double>(object.f32_CoreN);
        if (gps_imu && vehicle) {
            sample.obb_bounds_valid = FillMapObbBounds(
                object, *gps_imu, *vehicle,
                &sample.obb_min_e, &sample.obb_max_e,
                &sample.obb_min_n, &sample.obb_max_n);
        }
        if (track.valid_observations <= config_.warmup_observations) {
            track.state = MotionState::Unknown;
            if(sample.obb_bounds_valid && gps_imu && vehicle) {
                const auto velocity=RotateLidarVectorToMap(
                    {object.f32_AbsVx,object.f32_AbsVy,0},*gps_imu,*vehicle);
                track.warmup.push_back({sample.stamp_sec,sample.obb_min_e,sample.obb_max_e,
                    sample.obb_min_n,sample.obb_max_n,velocity.x,velocity.y});
                while(track.warmup.size()>3)track.warmup.pop_front();
                if(track.previously_dynamic && motion_evidence::Certify(track.warmup,
                        config_.edge_agreement_tolerance_m,config_.velocity_jump_kph/kMpsToKph,
                        config_.longitudinal_threshold_kph/kMpsToKph,config_.direction_cosine_min)) {
                    // Geometry supports motion, but full Dynamic confirmation
                    // still follows the ordinary history/evidence policy.
                    track.state=MotionState::DynamicPending;
                    track.decision_vx_kph=sample.vx_kph;
                    track.decision_vy_kph=sample.vy_kph;
                    track.decision_speed_kph=std::hypot(sample.vx_kph,sample.vy_kph);
                }
            } else track.warmup.clear();
            WriteDiagnostics(track, object);
            object.motion_valid=track.state!=MotionState::Unknown;
            continue;
        }
        track.warmup.clear();
        AppendObservation(track, sample);
        UpdateAcceptedHistory(track, sample);
        UpdateDecision(track, sample);
        if(track.state==MotionState::Dynamic)track.previously_dynamic=true;
        WriteDiagnostics(track, object);
        object.motion_valid=track.state!=MotionState::Unknown;
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (it->second.unseen_frames > config_.stale_track_frames) {
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}


void MotionClassifier::InvalidateMapHistory(std::vector<LiDARObjectInfo>& objects) {
    tracks_.clear();
    for(auto& object:objects) {
        object.motion_valid=false;
        WriteDiagnostics(TrackState{},object);
    }
}

