// =========================================================
// [Sec.VI 1/3단] 후보 궤적 킨매틱 실현가능성 필터.
//
// 논문 Sec.VI 1번째 문단: "각 집합은 결합 전에 outsized 가속도(s̈, d̈/d″)
// 값에 대해 먼저 체크된다" — lateral/longitudinal 집합을 합치기 *전에*
// 각자 필터링. 곡률(κx) 체크는 결합 이후 Cartesian 변환을 해야 가능하므로
// (FrenetConverter 필요) 별도 단계로 여기 함께 둔다 - 세 필터 모두 "생성된
// 후보가 물리적으로 유효한가"만 판정하며, 비용 계산이나 후보 선택은 이
// 파일의 책임이 아니다.
// =========================================================

#ifndef FRENET_EVALUATION_KINEMATIC_FILTER_HPP
#define FRENET_EVALUATION_KINEMATIC_FILTER_HPP

#include <vector>

#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Reference/Reference.hpp"

struct KinematicLimits {
    double max_lateral_accel;      // |d̈| 허용치 [m/s^2]
    double max_longitudinal_accel; // |s̈| 허용치 [m/s^2]
    double max_curvature;          // |kappa_x| 허용치 [1/m] (차량 최소 회전반경 역수)
    // |s⃛| 허용치 [m/s^3]. 초기 safety limit이며 최종 MORAI tuning 값이 아니다
    // (fix/yuntae-frenet-speed-continuity).
    double max_longitudinal_jerk;
};

// [Sec.VI 1단] 결합 전 가속도 필터링.
// 각 집합 안에서 |d̈|(또는 |s̈|)가 limits를 넘는 후보를 valid=false 처리.
// (논문 그림의 회색/검정 구분과 동일한 목적 — 지우지 않고 valid 플래그만 갱신해
//  디버그 시각화에 그대로 쓸 수 있게 한다)
void FilterLateralByAcceleration(std::vector<FrenetPath>& candidates,
                                  const KinematicLimits& limits);

// [fix/yuntae-frenet-speed-continuity] 세 invariant를 모두 확인한다:
//   |s̈| <= max_longitudinal_accel
//   s_d >= 0 (forward-only invariant, numerical epsilon만 예외)
//   |s⃛| <= max_longitudinal_jerk
// 이름을 FilterLongitudinalByAcceleration에서 바꾼 이유는 책임 범위가
// accel 단일 체크에서 kinematic validity 전체로 늘었기 때문이다.
void FilterLongitudinalByKinematicLimits(std::vector<FrenetPath>& candidates,
                                          const KinematicLimits& limits);

// [Sec.VI 3단] 결합 이후 곡률(kappa_x) 필터링.
// 각 샘플을 FrenetConverter::ComputeGeometricPath로 변환해 kappa_x를 구하고
// limits.max_curvature를 넘으면 valid=false 처리.
void FilterByCurvature(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const KinematicLimits& limits);

// FilterByCurvature와 달리 기하 곡률 한계 자체가 아니라, 재생성된 후보의
// 실제 샘플별 속도²×곡률(a_lat = s_d^2 * |kappa|)이 max_lateral_accel을
// 만족하는지 검증한다. 단순히 기하 곡률만 작은 경로가 아니라, 해당 속도에서
// 실제로 횡가속도 한계를 만족하는 경로만 남긴다(AVOID 재생성 후보 등에서
// 목표 속도가 곡률 한계 계산에 이미 반영되지 않은 경우를 잡는다).
void FilterByCartesianLateralAcceleration(std::vector<FrenetPath>& candidates,
                                          const RefLine& ref,
                                          double max_lateral_accel);

#endif  // FRENET_EVALUATION_KINEMATIC_FILTER_HPP
