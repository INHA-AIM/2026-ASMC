#include <Lidar/Tracking/MotionClassifier.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kMpsToKph = 3.6;
constexpr double kDirectionEpsilon = 1e-6;

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

    config.history_size = static_cast<std::size_t>(std::max(3, history_size));
    config.warmup_observations = std::max(0, config.warmup_observations);
    config.low_threat_confirmations =
        std::max(1, config.low_threat_confirmations);
    config.dynamic_confirmations =
        std::max(1, config.dynamic_confirmations);
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
    config_.change_confirmations =
        std::max(2, config_.change_confirmations);
    config_.short_miss_limit = std::max(0, config_.short_miss_limit);
    config_.stale_track_frames = std::max(1, config_.stale_track_frames);
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

int MotionClassifier::Sign(double value) {
    return (value > 0.0) - (value < 0.0);
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
    track.accepted.clear();
    track.candidates.clear();
    track.valid_observations = 0;
    track.low_threat_streak = 0;
    track.lateral_streak = 0;
    track.lateral_sign = 0;
    track.state = MotionState::Unknown;
    track.state_before_pending = MotionState::Unknown;
    track.decision_vx_kph = 0.0;
    track.decision_vy_kph = 0.0;
    track.decision_speed_kph = 0.0;
    track.map_displacement_speed_kph = 0.0;
    track.last_jump_kph = 0.0;
}

void MotionClassifier::AppendAccepted(
    TrackState& track, const VelocitySample& sample) const {
    track.accepted.push_back(sample);
    while (track.accepted.size() > config_.history_size) {
        track.accepted.pop_front();
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
    if (track.accepted.size() >= config_.history_size) {
        const auto& first = track.accepted.front();
        const auto& last = track.accepted.back();
        const double dt = last.stamp_sec - first.stamp_sec;
        if (dt > 1e-3 && std::isfinite(first.map_e) &&
            std::isfinite(first.map_n) && std::isfinite(last.map_e) &&
            std::isfinite(last.map_n)) {
            track.map_displacement_speed_kph = kMpsToKph * std::hypot(
                last.map_e - first.map_e, last.map_n - first.map_n) / dt;
        }
    }

    const double raw_abs_vy = std::abs(raw_sample.vy_kph);
    if (raw_abs_vy >= config_.lateral_threshold_kph) {
        const int lateral_sign = Sign(raw_sample.vy_kph);
        if (lateral_sign != 0 && lateral_sign == track.lateral_sign) {
            ++track.lateral_streak;
        } else {
            track.lateral_streak = 1;
            track.lateral_sign = lateral_sign;
        }
    } else {
        track.lateral_streak = 0;
        track.lateral_sign = 0;
    }

    const bool lateral_threat =
        (track.accepted.size() >= 2 &&
         std::abs(track.decision_vy_kph) >=
             config_.lateral_threshold_kph) ||
        track.lateral_streak >= config_.dynamic_confirmations;
    const bool longitudinal_threat =
        track.accepted.size() >= config_.history_size &&
        std::abs(track.decision_vx_kph) >=
            config_.longitudinal_threshold_kph &&
        track.map_displacement_speed_kph >=
            config_.longitudinal_displacement_kph;
    const bool fast_lateral_pending =
        raw_abs_vy >= config_.fast_lateral_threshold_kph;
    const bool threat = lateral_threat || longitudinal_threat;

    if (threat) {
        track.low_threat_streak = 0;
        track.state = MotionState::Dynamic;
        return;
    }

    if (fast_lateral_pending) {
        if (track.state != MotionState::DynamicPending) {
            track.state_before_pending = track.state;
        }
        track.low_threat_streak = 0;
        track.state = MotionState::DynamicPending;
        return;
    }

    if (track.state == MotionState::DynamicPending) {
        // 한 프레임 fast-path 오측정이면 즉시 직전 안정 상태로 돌아간다.
        track.state = track.state_before_pending;
    }
    ++track.low_threat_streak;
    if (track.low_threat_streak >= config_.low_threat_confirmations) {
        track.state = MotionState::LowThreat;
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
    object.f32_MapDisplacementSpeedKPH =
        static_cast<float32>(track.map_displacement_speed_kph);
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
        if (track.valid_observations <= config_.warmup_observations) {
            track.state = MotionState::Unknown;
            WriteDiagnostics(track, object);
            continue;
        }

        const VelocitySample sample{
            stamp.toSec(),
            static_cast<double>(object.f32_AbsVx) * kMpsToKph,
            static_cast<double>(object.f32_AbsVy) * kMpsToKph,
            static_cast<double>(object.f32_CoreE),
            static_cast<double>(object.f32_CoreN),
        };
        UpdateAcceptedHistory(track, sample);
        UpdateDecision(track, sample);
        WriteDiagnostics(track, object);
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (it->second.unseen_frames > config_.stale_track_frames) {
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}
