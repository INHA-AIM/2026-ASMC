// =========================================================
// [Sec.V-A/B] 종방향(longitudinal) 후보 집합 생성 — Following/Stopping/
// Merging/MergeArrival/VelocityKeeping.
//
// 이 파일은 "종방향 목표(s, ṡ, s̈)를 계산하고 그 종료조건으로 quintic/quartic
// 후보를 만드는" 역할까지만 담당한다. 어떤 생성기를 쓸지 고르는 모드별
// 분기(ResolveManeuver)와 lateral과의 결합/필터/비용 평가는 이 파일의
// 책임이 아니다.
// =========================================================

#ifndef FRENET_GENERATOR_LONGITUDINAL_GENERATOR_HPP
#define FRENET_GENERATOR_LONGITUDINAL_GENERATOR_HPP

#include <vector>

#include "Frenet/Geometry/FrenetTypes.hpp"
#include "Frenet/Generator/GeneratorConfig.hpp"

// One constant-time-gap target calculation shared by the following generator
// and its boundary-condition policy. This does not select a leader.
double FollowingApproachSpeed(double ego_s,double leader_s,double leader_speed,
                             double time_gap,double min_gap,double cruise_speed,
                             double gap_gain);

struct KinematicLimits;

// =========================================================
// [Sec.V-A] Following — 선두 차량과의 constant-time-gap 추종
//
// 선두 차량이 등가속(leader_accel=const)이라 가정하고 시간 적분한
// s_lv(t), ṡ_lv(t) 로부터
//   s_target(t)  = s_lv(t) - [min_gap + time_gap * ṡ_lv(t)]
//   ṡ_target(t)  = ṡ_lv(t) - time_gap * leader_accel
//   s̈_target(t)  = leader_accel                              (논문 Sec.V-A "Following")
// 를 구성한다. 목표가 현재 horizon 밖에 있을 때는
//   v_approach = clamp(v_lv + gap_gain * (gap - desired_gap), 0, cruise_target_speed)
// 로 접근하며, 목표가 horizon 안에 들어오면 위 constant-time-gap 종료조건을 쓴다.
// 각 Tj에서 [s_target(Tj)+Δsi, ṡ_target(Tj), s̈_target(Tj), Tj]를 종료조건으로 하는
// quintic 후보 집합을 cfg.delta_s × cfg.time_horizon 격자로 생성한다.
// =========================================================

std::vector<FrenetPath> GenerateFollowingCandidates(const FrenetState& start,
                                                     double leader_s,
                                                     double leader_speed,
                                                     double leader_accel,
                                                     double time_gap,
                                                     double min_gap,
                                                     double cruise_target_speed,
                                                     double gap_gain,
                                                     const PathGeneratorConfig& cfg);

// =========================================================
// [Sec.V-A] Stopping — 정지선/정지점 s_stop에서 정지
//
// s_target ≡ s_stop, ṡ_target ≡ 0, s̈_target ≡ 0 (논문 명시)
// Following과 동일하게 Δsi × Tj 격자로 quintic 후보 집합 생성.
// =========================================================

std::vector<FrenetPath> GenerateStoppingCandidates(const FrenetState& start,
                                                    double stop_s,
                                                    const PathGeneratorConfig& cfg);

// =========================================================
// [Sec.V-A] Merging — 두 차량 sa(t), sb(t) 사이로 병합
//
// s_target(t) = 1/2 * [sa(t) + sb(t)]  (논문 식 (3))
// sa, sb 모두 leader와 동일하게 등가속 예측을 적용해 시간적분한 뒤 평균낸다.
// =========================================================

std::vector<FrenetPath> GenerateMergingCandidates(const FrenetState& start,
                                                   double sa, double sa_speed, double sa_accel,
                                                   double sb, double sb_speed, double sb_accel,
                                                   const PathGeneratorConfig& cfg);

// 회전교차로 conflict point를 지정 시간에 통과하도록 위치/속도를 함께 고정한다.
// arrival_time_match_tolerance: entry_time 주변 T 격자 매칭 허용폭의 하한
// (MergeConfig.arrival_time_match_tolerance).
std::vector<FrenetPath> GenerateMergeArrivalCandidates(const FrenetState& start,
                                                        double conflict_s,
                                                        double entry_time,
                                                        double entry_speed,
                                                        const PathGeneratorConfig& cfg,
                                                        double arrival_time_match_tolerance);

// =========================================================
// [Sec.V-B] Velocity Keeping — 목표 속도 유지 (quartic, 종료 위치 자유)
//
// 종료조건 [ṡd + Δṡi, 0, Tj] 를 cfg.delta_s_dot × cfg.time_horizon 격자로
// 순회하며 MakeQuartic으로 후보를 만든다.
// =========================================================

std::vector<FrenetPath> GenerateVelocityKeepingCandidates(const FrenetState& start,
                                                           double target_speed,
                                                           const PathGeneratorConfig& cfg);

// A current admissible ceiling is not a terminal velocity boundary condition.
// Integrate a jerk-limited profile against its position-dependent ceiling.
std::vector<FrenetPath> GenerateEnvelopeVelocityCandidates(
    const FrenetState& start, double target_speed,
    const std::vector<SpeedEnvelopePoint>& envelope,
    const PathGeneratorConfig& cfg, const KinematicLimits& limits);

#endif  // FRENET_GENERATOR_LONGITUDINAL_GENERATOR_HPP
