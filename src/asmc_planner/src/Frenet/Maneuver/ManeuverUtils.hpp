#ifndef FRENET_MANEUVER_MANEUVER_UTILS_HPP
#define FRENET_MANEUVER_MANEUVER_UTILS_HPP

#include <vector>

#include "Frenet/Common/CartesianTypes.hpp"
#include "Frenet/Common/ObjectTypes.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Reference/Reference.hpp"

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
// Signed distance on the actual reference topology; open routes never wrap.
double ReferenceArcDistance(const RefLine& ref, double current_s, double target_s);

// fresh Cartesian collision recheck. AVOID(저속 crawl/PREPARE,
// Maneuver/Avoid/Avoid.cpp)와 MERGE(M2: locked replay/commit_pending
// candidate 검증, Maneuver/Merge/MergeGapEvaluator.cpp)가 쓴다. 내부에서
// 일반 Frenet 후보와 동일한 EvaluateCartesianTrajectorySafety를 사용한다.
bool CartesianTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObject>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id = nullptr,
                              int margin_recovery_object_id = -1,
                              bool* margin_recovery_used = nullptr,
                              const std::vector<int>* static_margin_recovery_ids = nullptr);

// AVOID must not depend on unconfirmed motion vacating current occupancy.
// Confirmed DYNAMIC objects retain their normal moving prediction.
bool CartesianAvoidTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObject>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id = nullptr,
                              int margin_recovery_object_id = -1,
                              bool* margin_recovery_used = nullptr,
                              const std::vector<int>* static_margin_recovery_ids = nullptr);

#endif  // FRENET_MANEUVER_MANEUVER_UTILS_HPP

