// =========================================================
// [Sec.IV-A] 횡방향(lateral) 후보 집합 생성 — 고속 모드 전용 (d(t) quintic)
//
// D0 = [d0, ḋ0, d̈0] = start (FrenetState의 d, d_d, d_dd)
// 종료조건 [d1, 0, 0, Tj] 를 cfg.lateral_d1 × cfg.time_horizon 격자로 순회하며
// MakeQuintic으로 각 후보를 만들고, cfg.dt 간격으로 샘플링해 FrenetPath.d/d_d/d_dd,
// t 를 채운다. (FrenetPath.s 계열은 비워둔 채로 반환 — longitudinal과 결합 시 채워짐)
//
// 저속 상태는 FrenetPlanner::PlanLowSpeedFallback에서 거리 기반 경로로 처리한다.
//
// 이 파일은 "횡방향 후보를 만드는" 역할까지만 담당한다. 어떤 d를 목표로
// 삼을지(모드별 offset 결정)는 ResolveLateralOffset이, 종/횡 결합·필터·비용
// 평가는 CandidateCombiner/Evaluation이 담당한다.
// =========================================================

#ifndef FRENET_GENERATOR_LATERAL_GENERATOR_HPP
#define FRENET_GENERATOR_LATERAL_GENERATOR_HPP

#include <vector>

#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/PlannerCommand.hpp"
#include "frenet_planner/Generator/GeneratorConfig.hpp"

std::vector<FrenetPath> GenerateLateralCandidates(const FrenetState& start,
                                                   const PathGeneratorConfig& cfg);

// =========================================================
// Lateral continuity candidate (fix/yuntae-frenet-speed-continuity Phase B)
//
// MORAI 실측: 중간 속도로 급커브에 근접할 때, 차선 중앙(d1=0) 복귀 후보들이
// 전부 사후 곡률 필터(FilterByCurvature)에서 탈락하는 경우가 있었다(유효
// 결합 후보는 있으나 곡률 필터 이후 0개). GenerateLateralCandidates는 이미
// d1 목표를 cfg.lateral_d1 격자로 순회하므로, 여기서는 그 격자를 start.d
// 하나로 좁힌 config로 같은 함수를 재사용해 "현재 횡오프셋을 유지"하는
// candidate 집합만 만든다(로직 복제 없음).
//
// 이 candidate는 안전 대안이다 - 중앙 복귀가 곡률 한계 안에서 가능하면
// EvaluateCosts의 기존 kd/kd_path 항이 여전히 d1=0 후보를 더 낮은 비용으로
// 선호하므로 SelectBestPath가 그것을 그대로 선택한다. 중앙 복귀 후보가 전부
// 곡률 필터에서 탈락했을 때만 이 candidate가 유일한 valid 후보로 남아
// 선택된다.
std::vector<FrenetPath> GenerateLateralContinuityCandidates(const FrenetState& start,
                                                             const PathGeneratorConfig& cfg);

// AVOID 모드는 PlannerCommand.avoidance_d_offset을 그대로 lateral 목표 중심으로 사용.
// LANE_CHANGE_*는 d 양의 방향이 좌측(FrenetToCartesian의 n_r 정의와 동일)이므로
// LEFT는 +lane_width, RIGHT는 -lane_width. 회전교차로 MERGE는 global path 유지.
double ResolveLateralOffset(const PlannerCommand& cmd, double lane_width);

#endif  // FRENET_GENERATOR_LATERAL_GENERATOR_HPP
