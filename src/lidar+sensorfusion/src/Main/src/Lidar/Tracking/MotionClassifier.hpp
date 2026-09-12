#pragma once

#include <Global/Global.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

enum class MotionState : std::int32_t {
    Unknown = 0,
    LowThreat = 1,
    DynamicPending = 2,
    Dynamic = 3,
};

const char* MotionStateName(MotionState state);

struct MotionClassifierConfig {
    std::size_t history_size = 5;
    int warmup_observations = 3;
    int low_threat_confirmations = 3;
    int dynamic_confirmations = 2;
    int change_confirmations = 3;
    int short_miss_limit = 3;
    int stale_track_frames = 30;

    double lateral_threshold_kph = 3.0;
    double longitudinal_threshold_kph = 10.0;
    double fast_lateral_threshold_kph = 5.0;
    double velocity_jump_kph = 10.0;
    double direction_cosine_min = 0.7;
    double longitudinal_displacement_kph = 5.0;
};

// LiDAR tracker가 만든 순간 속도는 수정하지 않고, track id별 판정 이력만
// 별도로 관리한다. Update()가 변경하는 원본 객체 필드는 b_IsDynamic과
// f32_Decision* 진단 필드뿐이다.
class MotionClassifier {
public:
    explicit MotionClassifier(const MotionClassifierConfig& config = {});
    explicit MotionClassifier(ros::NodeHandle& private_node);

    void Update(std::vector<LiDARObjectInfo>& objects,
                const ros::Time& stamp);

private:
    struct VelocitySample {
        double stamp_sec = 0.0;
        double vx_kph = 0.0;
        double vy_kph = 0.0;
        double map_e = 0.0;
        double map_n = 0.0;
    };

    struct ChangeCandidate {
        VelocitySample sample;
        double delta_vx_kph = 0.0;
        double delta_vy_kph = 0.0;
    };

    struct TrackState {
        std::deque<VelocitySample> accepted;
        std::deque<ChangeCandidate> candidates;
        int valid_observations = 0;
        int consecutive_misses = 0;
        int unseen_frames = 0;
        int low_threat_streak = 0;
        int lateral_streak = 0;
        int lateral_sign = 0;
        MotionState state = MotionState::Unknown;
        MotionState state_before_pending = MotionState::Unknown;
        double decision_vx_kph = 0.0;
        double decision_vy_kph = 0.0;
        double decision_speed_kph = 0.0;
        double map_displacement_speed_kph = 0.0;
        double last_jump_kph = 0.0;
    };

    static double Median(std::vector<double> values);
    static int Sign(double value);
    static bool DirectionConsistent(const ChangeCandidate& reference,
                                    double delta_vx_kph,
                                    double delta_vy_kph,
                                    double minimum_cosine);

    void ResetDecision(TrackState& track) const;
    void AppendAccepted(TrackState& track,
                        const VelocitySample& sample) const;
    void UpdateAcceptedHistory(TrackState& track,
                               const VelocitySample& sample) const;
    void UpdateDecision(TrackState& track,
                        const VelocitySample& raw_sample) const;
    void WriteDiagnostics(const TrackState& track,
                          LiDARObjectInfo& object) const;

    MotionClassifierConfig config_;
    std::unordered_map<std::int32_t, TrackState> tracks_;
};
