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
    // [m/s] 이 속도 미만이면 quintic candidate 파이프라인 대신 저속
    // crawl/geometric fallback을 쓴다. KEEP/AVOID/LANE_CHANGE가 공유하는
    // 단일 값이다 - 파일마다 다른 값을 쓰면 maneuver 전환 시 저속 판정
    // 경계가 어긋난다.
    double low_speed_fallback_v;
    // [s] 저속 crawl fallback이 만드는 경로의 기본 지속시간.
    double low_speed_crawl_duration_s;
    // [m/s] low_speed_fallback_v보다 넉넉히 위의 경계. 이 속도 미만이고
    // external KEEP의 정상 velocity-keeping 후보가 전부 탈락했을 때만(=
    // starvation), 이미 검증된 저속 fallback 경로로 안전하게 이어간다
    // (fix/yuntae-frenet-speed-continuity Phase 5).
    double near_low_speed_fallback_v;
    // 저속 lateral transition 거리 계산에서 max_curvature의 몇 %만
    // 사용할지(여유폭 확보) - 이산 샘플/MPC 오차에 대한 여유다.
    double low_speed_usable_curvature_fraction;
    // 저속 lateral transition 거리에 곱하는 여유 배수(10% 정도).
    double low_speed_transition_distance_margin;
    // 후보 생성 초기 경계조건(s_ddot/d_ddot)을 max_longitudinal_accel/
    // max_lateral_accel의 몇 %로 clip할지 - 투영 오차가 초기 가속도 경계를
    // 비현실적으로 키우지 않도록, 다항식 중간구간 overshoot 여유를 남긴다.
    double boundary_condition_accel_fraction;
    // AVOID SHIFT 시작거리 계산에서 max_lateral_accel의 몇 %만 사용할지
    // (조향/제어 응답 여유).
    double avoid_shift_usable_lateral_accel_fraction;
    // [m/s] 후보 생성 초기 횡속도(d_dot) 경계조건의 절대값 상한. 큰 heading
    // 오차로 과대 추정될 수 있는 초기 d_dot을 제한한다.
    double max_initial_d_dot;
    // [m/s^2] 일반 저속 fallback(PlanLowSpeedFallback)의 크롤링 가속도.
    // 회전교차로 MERGE는 이 프로파일을 쓰지 않고 정상 MERGE 후보 생성기로
    // 처리한다.
    double low_speed_crawl_accel;
    // [m/s] 저속 fallback의 최소 전진속도(0 근처에 갇히지 않도록).
    double low_speed_min_creep_speed;
    // [m/s] 저속 fallback 안에서의 속도 상한.
    double low_speed_crawl_speed_cap;
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
                                          double max_lateral_accel,
                                          double utilization = 1.0);

#endif  // FRENET_EVALUATION_KINEMATIC_FILTER_HPP
