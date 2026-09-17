// =========================================================
// [Sec.VI 2단] Tlat × Tlon 결합, 그리고 [Sec.VII] ResolveManeuver —
// FSM ↔ Path Generator 연동 지점.
//
// PlannerCommand.mode에 따라 활성화할 longitudinal 모드를 결정하고
// (예: LANE_KEEPING -> VelocityKeeping, FOLLOWING -> Following,
//      STOP/INTERSECTION_WAIT -> Stopping, LANE_CHANGE_* -> VelocityKeeping
//      + lateral target_lane 반영 등), lateral 후보 집합과 해당 longitudinal
// 후보 집합(들)을 생성해 Tlat × Tlon으로 결합한 뒤, 사전/사후 필터링까지
// 적용한 전체 후보 목록을 반환한다.
//
// 비용 계산(C_lat, C_lon)과 충돌 체크, 그리고 여러 활성 모드 중 최종 1개를
// 고르는 override 로직(Sec.VII, "가장 작은 초기 저크값" 기준)은 이 파일의
// 책임이 아니라 Evaluation/Cost.hpp가 담당한다. (이 파일은 "후보를 만들고
// 결합하는" 역할까지만)
// =========================================================

#ifndef FRENET_GENERATOR_CANDIDATE_COMBINER_HPP
#define FRENET_GENERATOR_CANDIDATE_COMBINER_HPP

#include <cstddef>
#include <vector>

#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/PlannerCommand.hpp"
#include "frenet_planner/Config/FrenetConfig.hpp"
#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Generator/GeneratorConfig.hpp"
#include "frenet_planner/Pipeline/TrajectoryRequest.hpp"
#include "frenet_planner/Reference/Reference.hpp"

// [Sec.VI 2단] Tlat x Tlon 결합 — 같은 T(종료시간)를 가진 후보끼리만 결합.
//
// 동일한 T(종료시간)를 가진 lateral/longitudinal 후보끼리만 묶어
// 완전한 FrenetPath(t, d.., s..)를 만든다. (헤더 상단 설계 노트 참고)
// valid=false인 후보는 건너뛴다.
std::vector<FrenetPath> CombineLateralLongitudinal(const std::vector<FrenetPath>& lateral_set,
                                                    const std::vector<FrenetPath>& longitudinal_set);

// 실패 원인 진단용(정상 경로에는 영향 없음, nullptr이면 아무것도 안 함):
// 어느 필터 단계에서 후보가 다 죽었는지 구분하기 위한 카운터.
struct PlannerDebugStats {
    size_t lateral_total = 0, lateral_valid = 0;
    size_t longitudinal_total = 0, longitudinal_valid = 0;
    size_t combined_total = 0, combined_valid_after_curvature = 0;
    size_t combined_valid_after_collision = 0;
};

// obstacles/vehicle_shape/collision_cfg: Sec.VI 충돌 필터(FilterByCollision,
// SAT 기반)를 곡률 필터 다음에 적용한다. AVOID 전용이 아니라 모든 모드에
// 공통으로 적용된다(논문 Sec.VI도 충돌검사를 모드와 무관하게 항상 수행) -
// 장애물이 없거나 멀면 아무 후보도 걸러지지 않아 기존 동작과 동일하다.
// obstacles가 비어있으면(장애물 없음) 호출 자체를 건너뛴다.
std::vector<FrenetPath> ResolveManeuver(const FrenetState& start,
                                         const PlannerCommand& cmd,
                                         const RefLine& ref,
                                         const PathGeneratorConfig& cfg,
                                         const KinematicLimits& limits,
                                         double lane_width,
                                         const std::vector<TrackedObject>& obstacles,
                                         const VehicleShape& vehicle_shape,
                                         const CollisionCheckConfig& collision_cfg,
                                         const StopConfig& stop_cfg,
                                         const MergeConfig& merge_cfg,
                                         PlannerDebugStats* stats = nullptr);

// =========================================================
// R5: TrajectoryRequest 기반 candidate 생성 — KEEP/FOLLOW/STOP 전용.
//
// ResolveManeuver와 동일한 3단계(lateral 생성 -> longitudinal 생성 ->
// 결합/곡률/충돌 필터)를 거치지만, 입력을 PlannerCommand/BehaviorState가
// 아니라 TrajectoryRequest 하나로만 받는다 - 이 함수와 그 아래에서 호출하는
// 어떤 코드도 BehaviorState/PlannerCommand/ROS 타입을 참조하지 않는다.
//
// R6: KEEP/FOLLOW/STOP에 이어 LANE_CHANGE/AVOID(정상 속도 구간)도 이 함수를
// 사용한다. MERGE는 production에서 도달 불가능하므로(§13 audit 참고) 여전히
// 대상이 아니다 - 남은 legacy caller는 ResolveManeuver(PlannerCommand) 직접
// 호출뿐이다.
//
// 충돌 필터 예외 정책(coast_exempt/escape)은 request.collision에서 그대로
// 읽는다(R6-A) - longitudinal_mode를 보고 추론하지 않는다.
std::vector<FrenetPath> GenerateRequestedCandidates(
    const FrenetState& start,
    const TrajectoryRequest& request,
    const RefLine& ref,
    const PathGeneratorConfig& cfg,
    const KinematicLimits& limits,
    const std::vector<TrackedObject>& obstacles,
    const VehicleShape& vehicle_shape,
    const CollisionCheckConfig& collision_cfg,
    PlannerDebugStats* stats = nullptr);

#endif  // FRENET_GENERATOR_CANDIDATE_COMBINER_HPP
