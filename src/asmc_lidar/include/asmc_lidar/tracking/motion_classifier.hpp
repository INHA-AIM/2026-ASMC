#pragma once
#include <asmc_lidar/tracking/motion_evidence.hpp>






#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>
#include <string>

namespace asmc { namespace lidar { namespace pipeline { namespace motion_classifier {
using float32 = float;
// Full-scan Main-compatible applied estimates; UNKNOWN is not confirmed static.
struct MotionObject {
 int s32_Id=-1,s32_MissCount=0;
 double f32_AbsVx=0,f32_AbsVy=0,f32_CoreE=0,f32_CoreN=0;
 bool bounds_valid=false; double min_e=0,max_e=0,min_n=0,max_n=0,map_vx=0,map_vy=0;
 bool motion_valid=false,b_IsDynamic=false;
 bool semantic_static_eligible=false,semantic_static_applied=false;
 std::string semantic_reason="off";
 int s32_MotionState=0,s32_MotionValidObservations=0,s32_MotionAcceptedSamples=0,s32_MotionCandidateSamples=0;
 double f32_DecisionVxKPH=0,f32_DecisionVyKPH=0,f32_DecisionSpeedKPH=0,f32_PlannerVxKPH=0,f32_PlannerVyKPH=0;
 double f32_MapDisplacementSpeedKPH=0,f32_CorrectedDisplacementSpeedKPH=0,f32_MotionVelocityJumpKPH=0;
};
enum class MotionState : std::int32_t {
    Unknown = 0,
    LowThreat = 1,
    DynamicPending = 2,
    Dynamic = 3,
};

const char* MotionStateName(MotionState state);

struct MotionClassifierConfig {
    bool semantic_static_enabled=false;
    double semantic_min_window_s=.25,semantic_max_window_s=.6;
    double semantic_min_translation_m=.35;
    std::size_t history_size = 5;
    int warmup_observations = 3;
    int low_threat_confirmations = 3;
    int dynamic_confirmations = 3;
    int pending_static_confirmations = 3;
    int dynamic_release_confirmations = 8;
    int pending_max_frames = 5;
    int change_confirmations = 3;
    int short_miss_limit = 3;
    int stale_track_frames = 30;

    double lateral_threshold_kph = 4.0;
    double longitudinal_threshold_kph = 10.0;
    double fast_lateral_threshold_kph = 12.0;
    double velocity_jump_kph = 10.0;
    double direction_cosine_min = 0.7;
    double longitudinal_displacement_kph = 8.0;
    double lateral_displacement_kph = 3.0;
    double exit_speed_kph = 2.5;
    // Consecutive OBB bounds which move by a similar amount represent rigid
    // translation.  When only one side moves, use the stable side so newly
    // visible points do not become false object velocity.
    double edge_agreement_tolerance_m = 0.35;
};

// LiDAR tracker가 만든 순간 속도는 수정하지 않고, track id별 판정 이력만
// 별도로 관리한다. Update()는 b_IsDynamic, planner-facing 속도와
// f32_Decision* 진단 필드만 기록한다.
class MotionClassifier {
public:
    explicit MotionClassifier(const MotionClassifierConfig& config = {});

    void Update(std::vector<MotionObject>& objects, double stamp_sec);

    void InvalidateMapHistory(std::vector<MotionObject>& objects);

private:
    struct VelocitySample {
        double stamp_sec = 0.0;
        double vx_kph = 0.0;
        double vy_kph = 0.0;
        double map_e = 0.0;
        double map_n = 0.0;
        double obb_min_e = 0.0;
        double obb_max_e = 0.0;
        double obb_min_n = 0.0;
        double obb_max_n = 0.0;
        bool obb_bounds_valid = false;
    };

    struct ChangeCandidate {
        VelocitySample sample;
        double delta_vx_kph = 0.0;
        double delta_vy_kph = 0.0;
    };

    struct TrackState {
        bool previously_dynamic = false;
        bool semantic_active=false, semantic_motion_watch=false;
        int semantic_static_streak=0;
        std::string semantic_reason="off";
        std::deque<motion_evidence::Sample> warmup;
        std::deque<VelocitySample> accepted;
        std::deque<VelocitySample> observations;
        std::deque<ChangeCandidate> candidates;
        int valid_observations = 0;
        int consecutive_misses = 0;
        int unseen_frames = 0;
        int low_threat_streak = 0;
        int dynamic_evidence_streak = 0;
        int static_evidence_streak = 0;
        int pending_frames = 0;
        MotionState state = MotionState::Unknown;
        MotionState state_before_pending = MotionState::Unknown;
        double decision_vx_kph = 0.0;
        double decision_vy_kph = 0.0;
        double decision_speed_kph = 0.0;
        double map_displacement_speed_kph = 0.0;
        double corrected_displacement_speed_kph = 0.0;
        double last_jump_kph = 0.0;
    };

    static double Median(std::vector<double> values);
    static bool DirectionConsistent(const ChangeCandidate& reference,
                                    double delta_vx_kph,
                                    double delta_vy_kph,
                                    double minimum_cosine);

    void ResetDecision(TrackState& track) const;
    void AppendAccepted(TrackState& track,
                        const VelocitySample& sample) const;
    void AppendObservation(TrackState& track,
                           const VelocitySample& sample) const;
    void UpdateAcceptedHistory(TrackState& track,
                               const VelocitySample& sample) const;
    void UpdateDecision(TrackState& track,
                        const VelocitySample& raw_sample, bool semantic_eligible) const;
    void WriteDiagnostics(const TrackState& track,
                          MotionObject& object) const;
    void UpdateImpl(std::vector<MotionObject>& objects, double stamp_sec);

    MotionClassifierConfig config_;
    std::unordered_map<std::int32_t, TrackState> tracks_;
};


}}}} // namespace asmc::lidar::pipeline::motion_classifier
