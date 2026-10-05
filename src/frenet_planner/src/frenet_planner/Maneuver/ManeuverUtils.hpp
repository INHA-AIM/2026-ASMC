#ifndef FRENET_MANEUVER_MANEUVER_UTILS_HPP
#define FRENET_MANEUVER_MANEUVER_UTILS_HPP

#include <vector>

#include "frenet_planner/Common/CartesianTypes.hpp"
#include "frenet_planner/Common/ObjectTypes.hpp"
#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/Reference/Reference.hpp"

// 여러 Maneuver/*.cpp가 공유하는 package-internal helper. FrenetPlanner
// public API가 아니므로 frenet_planner.hpp에는 선언하지 않는다.

// 장애물 하나를 참조선 기준 Frenet 좌표로 투영한다.
// out_s: 장애물의 s(참조선 arc length), out_d: 장애물의 lateral offset,
// out_s_dot: 참조선 진행 방향으로 vx/vy를 투영한 장애물의
// 종방향 속도. OBB heading은 형상 방향이므로 속도 계산에 쓰지 않는다.
// 탐색 범위 제한 없이 항상 전체 참조선에서 최근접점을 찾는다(장애물은
// ego처럼 매 사이클 위치가 조금씩만 바뀐다는 연속성 가정을 할 수 없고,
// FOLLOW 탐색 거리(수십 m)가 ego용 search window보다 클 수 있어 hint를 안
// 쓰는 게 안전하다). s_hint를 안 쓰므로 CartesianToFrenet의 search_window
// 인자는 실제로 쓰이지 않지만, 함수 시그니처 일관성을 위해 그대로 넘긴다.
void ProjectObjectToFrenet(const RefLine& ref, const TrackedObject& obj,
                           double& out_s, double& out_d, double& out_s_dot,
                           double ego_projection_search_window);

// FOLLOW target의 OBB 안에 겹쳐 나타난 작은 정적 UNKNOWN OBB를 동일 물체의
// LiDAR fragment로 연결한다. geometry는 보존하고 미래 위치 예측에 쓰는
// motion만 leader와 맞춘다. 반환값은 연결된 fragment 수다.
int SynchronizeFollowLeaderFragments(
    int leader_id, std::vector<TrackedObject>& obstacles);

// 폐루프 트랙에서 current_s -> target_s의 전방 호길이. STOP(stop_distance
// 계산, Maneuver/Stop.cpp)과 AVOID(post-COMMIT gap, Maneuver/Avoid/Avoid.cpp
// 및 Maneuver/Avoid/AvoidLifecycle.cpp)가 쓴다.
double ForwardArcDistance(double current_s, double target_s, double s_max);

// 지정 AVOID 대상에만 적용할 충돌 여유 처리 방식.
//
// INITIAL_MARGIN_ONLY는 기존 정책이다. 첫 sample부터 이미 확장 safety
// margin 안에 있으면서 실제 OBB는 겹치지 않을 때에만 바깥으로 빠져나가는
// 경로를 허용한다.
//
// CONTINUOUS_OUTWARD_RECOVERY는 committed AVOID가 정지한 뒤 쓰는 제한적
// 복구 정책이다. 경로 도중 확장 margin에 들어가더라도 실제 OBB가 한 번도
// 겹치지 않고, 대상과의 횡방향 거리가 계속 증가할 때만 허용한다. 지정
// 대상 이외의 객체에는 언제나 기존 확장 margin을 그대로 적용한다.
enum class TargetEscapePolicy {
    INITIAL_MARGIN_ONLY,
    CONTINUOUS_OUTWARD_RECOVERY,
};

// fresh Cartesian collision recheck. AVOID(저속 crawl/PREPARE,
// Maneuver/Avoid/Avoid.cpp)와 MERGE(M2: locked replay/commit_pending
// candidate 검증, Maneuver/Merge/MergeGapEvaluator.cpp)가 쓴다.
bool CartesianTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObject>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id = nullptr,
                              int avoid_escape_object_id = -1,
                              TargetEscapePolicy escape_policy =
                                  TargetEscapePolicy::INITIAL_MARGIN_ONLY);

#endif  // FRENET_MANEUVER_MANEUVER_UTILS_HPP
