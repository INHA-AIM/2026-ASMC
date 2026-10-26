// Ported unchanged from Main PlannerInterface/PlanningRelevanceFilter.cpp at ab8db094.
// Namespace/types/includes only; retain acquisition/release and legacy policy for cutover.
#include <asmc_lidar/filtering/observation_quality.hpp>

#include <asmc_lidar/filtering/delivery_object.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>

namespace asmc::lidar::filtering {
namespace {

std::string NormalizeLabel(std::string label) {
    label.erase(label.begin(), std::find_if(
        label.begin(), label.end(), [](unsigned char value) {
            return !std::isspace(value);
        }));
    label.erase(std::find_if(
        label.rbegin(), label.rend(), [](unsigned char value) {
            return !std::isspace(value);
        }).base(), label.end());
    std::transform(label.begin(), label.end(), label.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    return label;
}

bool IsPedestrian(const DeliveryObject& object) {
    const std::string classification = NormalizeLabel(object.classification);
    const std::string yolo = NormalizeLabel(object.yolo_class);
    return classification == "person" || classification == "pedestrian" ||
           yolo == "person" || yolo == "pedestrian";
}

bool HasKnownVehicleSemantics(const DeliveryObject& object) {
    const std::string classification = NormalizeLabel(object.classification);
    const std::string yolo = NormalizeLabel(object.yolo_class);
    const auto vehicle_label = [](const std::string& label) {
        return label == "car" || label == "vehicle" || label == "truck" ||
               label == "bus" || label == "motorcycle" || label == "bicycle";
    };
    return vehicle_label(classification) || vehicle_label(yolo);
}

double EmergencyDistance(double speed_mps) {
    // 이 단계의 override는 일반 comfort 감속이 아니라, 새 ID로 나타난 실제
    // 장애물을 confirmation 동안 놓치지 않기 위한 마지막 안전장치다. 최대
    // 6 m/s^2 감속과 0.25 s 반응거리, 2 m 여유로 물리적으로 유도한다.
    const double speed = std::max(0.0, speed_mps);
    constexpr double kEmergencyDeceleration = 6.0;
    constexpr double kReactionSeconds = 0.25;
    constexpr double kStandstillBuffer = 2.0;
    constexpr double kMinimumEmergencyDistance = 6.0;
    return std::max(
        kMinimumEmergencyDistance,
        speed * kReactionSeconds +
            speed * speed / (2.0 * kEmergencyDeceleration) +
            kStandstillBuffer);
}

}  // namespace

void ObservationQualityFilter::Configure(
    double minimum_obstacle_height_m, int confirmation_frames,
    int release_frames) {
    minimum_obstacle_height_m_ =
        std::max(0.0, minimum_obstacle_height_m);
    confirmation_frames_ = std::max(1, confirmation_frames);
    release_frames_ = std::max(1, release_frames);
    tracks_.clear();
    frame_index_ = 0;
}

void ObservationQualityFilter::BeginFrame() {
    ++frame_index_;
    constexpr std::uint64_t kForgetAfterFrames = 20;
    for (auto iterator = tracks_.begin(); iterator != tracks_.end();) {
        if (frame_index_ > iterator->second.last_frame + kForgetAfterFrames) {
            iterator = tracks_.erase(iterator);
        } else {
            ++iterator;
        }
    }
}

ObservationQualityDecision ObservationQualityFilter::Evaluate(
    const DeliveryObject& object, bool upstream_keep,
    double ego_x, double ego_y, double ego_speed_mps,
    double ego_length_m) {
    ObservationQualityDecision decision;
    decision.planning_release_frames = release_frames_;
    if (!upstream_keep) {
        decision.keep = false;
        decision.reason = "upstream_filter_drop";
        return decision;
    }
    if (IsPedestrian(object) || NormalizeLabel(object.yolo_class) == "dog") {
        decision.reason = "pedestrian_immediate_keep";
        return decision;
    }
    if (HasKnownVehicleSemantics(object)) {
        // Camera/fusion semantics가 이미 차량으로 확정한 객체는 LiDAR OBB의
        // 순간적인 높이 축소 때문에 planning 입력에서 사라지면 안 된다.
        decision.reason = "classified_vehicle_immediate_keep";
        return decision;
    }

    decision.policy_applied = true;
    TrackState& track = tracks_[object.id];
    const double observation_radius =
        std::isfinite(object.length) && std::isfinite(object.width) &&
        object.length > 0.0 && object.width > 0.0
            ? 0.5 * std::hypot(object.length, object.width) : 0.0;
    if (track.last_frame + 1 != frame_index_) {
        // Acquisition still requires consecutive observations. A confirmed
        // track may survive a bounded dropout when the same ID reappears near
        // its last footprint. Never publish an object during its absence.
        const bool short_dropout = track.last_frame > 0 &&
            frame_index_ > track.last_frame &&
            frame_index_ - track.last_frame - 1 <=
                static_cast<std::uint64_t>(release_frames_);
        const bool same_location = observation_radius > 0.0 &&
            track.last_radius > 0.0 &&
            std::isfinite(object.obb_center_map.x) &&
            std::isfinite(object.obb_center_map.y) &&
            std::hypot(object.obb_center_map.x - track.last_x,
                       object.obb_center_map.y - track.last_y) <=
                observation_radius + track.last_radius;
        if (!track.confirmed || !short_dropout || !same_location) {
            track = TrackState{};
        } else {
            track.consecutive_good_height_frames = 0;
            // Preserve bad evidence so intermittent dropouts cannot restart
            // the existing height-release grace indefinitely.
        }
    }
    track.last_frame = frame_index_;
    track.last_x = object.obb_center_map.x;
    track.last_y = object.obb_center_map.y;
    track.last_radius = observation_radius;
    const auto copy_track_debug = [&track, &decision]() {
        decision.planning_confirmed = track.confirmed;
        decision.good_height_frames =
            track.consecutive_good_height_frames;
        decision.height_confirmations =
            track.consecutive_good_height_frames;
        decision.bad_height_frames =
            track.consecutive_bad_height_frames;
    };

    if (!std::isfinite(object.height)) {
        // 숫자 검증은 adapter에도 있지만 이 경계에서는 삭제 대신 fail-open한다.
        copy_track_debug();
        decision.reason = "invalid_height_keep";
        return decision;
    }
    // OBB Z span is sampled thickness, not height above the road. A distant
    // obstacle hit by a single scan ring can have a 5 cm box at 70 cm above
    // ground. Preserve the existing acquire/release policy, but count robust
    // above-ground observations as good evidence even when the span collapses.
    const auto& ground = object.ground_evidence;
    const bool elevated_returns = ground.valid && ground.sample_count >= 3 &&
        std::isfinite(ground.median_clearance_m) &&
        std::isfinite(ground.rejection_band_m) && ground.rejection_band_m >= 0.0 &&
        ground.median_clearance_m > std::max(
            minimum_obstacle_height_m_, ground.rejection_band_m);
    decision.elevated_ground_evidence = elevated_returns;
    if (!elevated_returns && object.height + 1e-6 < minimum_obstacle_height_m_) {
        track.consecutive_good_height_frames = 0;
        if (track.confirmed) {
            ++track.consecutive_bad_height_frames;
            if (track.consecutive_bad_height_frames < release_frames_) {
                copy_track_debug();
                decision.retained_by_height_grace = true;
                decision.reason = "confirmed_obstacle_height_grace";
                return decision;
            }

            // Report the release evidence on this decision, while making the
            // next good observation start a fresh acquisition immediately.
            track.confirmed = false;
            copy_track_debug();
            decision.keep = false;
            decision.reason = "confirmed_obstacle_height_release";
            return decision;
        }

        track.consecutive_bad_height_frames = 0;
        copy_track_debug();
        decision.keep = false;
        decision.reason = "ground_fragment_drop";
        return decision;
    }
    track.consecutive_bad_height_frames = 0;
    ++track.consecutive_good_height_frames;

    const double center_distance = std::hypot(
        object.obb_center_map.x - ego_x,
        object.obb_center_map.y - ego_y);
    const double object_radius = 0.5 * std::hypot(
        std::max(0.0, object.length), std::max(0.0, object.width));
    const double ego_radius = 0.5 * std::max(0.0, ego_length_m);
    decision.obstacle_clearance_m = std::max(
        0.0, center_distance - object_radius - ego_radius);
    decision.emergency_distance_m = EmergencyDistance(ego_speed_mps);
    if (decision.obstacle_clearance_m <= decision.emergency_distance_m) {
        decision.emergency_override = true;
        copy_track_debug();
        decision.reason = "close_obstacle_immediate_keep";
        return decision;
    }
    if (track.confirmed) {
        copy_track_debug();
        decision.reason = "obstacle_confirmed_keep";
        return decision;
    }
    if (track.consecutive_good_height_frames < confirmation_frames_) {
        copy_track_debug();
        decision.keep = false;
        decision.reason = "obstacle_confirmation_pending";
        return decision;
    }
    track.confirmed = true;
    copy_track_debug();
    decision.reason = "obstacle_confirmed_keep";
    return decision;
}

}  // namespace asmc::lidar::filtering
