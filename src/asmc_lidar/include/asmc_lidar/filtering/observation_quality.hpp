// Ported unchanged from Main PlannerInterface/PlanningRelevanceFilter.hpp at ab8db094.
// Namespace/types/includes only; retain acquisition/release and legacy policy for cutover.
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace asmc::lidar::filtering {
struct DeliveryObject;

struct ObservationQualityDecision {
    bool keep = true;
    bool policy_applied = false;
    bool emergency_override = false;
    bool planning_confirmed = false;
    bool retained_by_height_grace = false;
    bool elevated_ground_evidence = false;
    int height_confirmations = 0;
    int good_height_frames = 0;
    int bad_height_frames = 0;
    int planning_release_frames = 0;
    double obstacle_clearance_m = 0.0;
    double emergency_distance_m = 0.0;
    std::string reason = "planning_relevance_keep";
};

// /last_lidar 직전의 공용 planning 입력 게이트다. Behavior와 Frenet이 같은
// 배열을 구독하므로 여기서 제거한 지면 파편이나 미확정 객체는 양쪽 모두에서
// 동일하게 사라진다. 원본 tracking/debug 토픽은 건드리지 않는다.
class ObservationQualityFilter {
public:
    void Configure(double minimum_obstacle_height_m,
                   int confirmation_frames,
                   int release_frames);
    void BeginFrame();

    ObservationQualityDecision Evaluate(
        const DeliveryObject& object,
        bool upstream_keep,
        double ego_x,
        double ego_y,
        double ego_speed_mps,
        double ego_length_m);

    double minimum_obstacle_height_m() const {
        return minimum_obstacle_height_m_;
    }
    int confirmation_frames() const { return confirmation_frames_; }
    int release_frames() const { return release_frames_; }

private:
    struct TrackState {
        std::uint64_t last_frame = 0;
        int consecutive_good_height_frames = 0;
        int consecutive_bad_height_frames = 0;
        bool confirmed = false;
        double last_x = 0.0;
        double last_y = 0.0;
        double last_radius = 0.0;
    };

    std::uint64_t frame_index_ = 0;
    double minimum_obstacle_height_m_ = 0.10;
    int confirmation_frames_ = 3;
    int release_frames_ = 5;
    std::unordered_map<std::int32_t, TrackState> tracks_;
};

}  // namespace asmc::lidar::filtering
