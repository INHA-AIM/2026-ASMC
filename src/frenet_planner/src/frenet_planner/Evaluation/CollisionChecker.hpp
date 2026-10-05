#ifndef FRENET_COLLISION_CHECKER_HPP
#define FRENET_COLLISION_CHECKER_HPP

#include <utility>
#include <vector>

#include "frenet_planner/Reference/Reference.hpp"
#include "frenet_planner/Geometry/PolygonUtils.hpp"
#include "frenet_planner/Common/CartesianTypes.hpp"
#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/ObjectTypes.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"

// =========================================================
// 아래 두 구조체는 논문 Sec.VI의 개념(안전마진, zero/one 판정, 시간에 따라 부풀리는 컨투어)을 구체화하기 위해 이 프로젝트가 내린 설계 판단이다.
// 논문은 "얼마나", "어떤 모양으로"는 규정하지 않는다.
// =========================================================

// 자차 충돌판정용 크기. parameter.yaml에서 로드될 예정.
struct VehicleShape {
    double width;   // [m]
    double length;  // [m]
    // CartesianState x/y에서 뒤 범퍼까지 거리. 음수이면 기존처럼 x/y를
    // 차체 중심으로 취급한다(기존 단위 테스트/다른 차량 설정 호환). 이 sentinel
    // 의미는 코드 계약이지 튜닝 가능한 parameter가 아니므로 params.yaml로
    // 옮기지 않는다 - 호출자가 항상 명시적으로 값을 지정해야 한다.
    double rear_overhang; // [m]
};

struct CollisionCheckConfig {
    double safety_margin;       // 기본 안전마진 [m] (자차 폭/길이에 사방으로 더해짐)
    double margin_growth_rate;  // 시간에 따라 마진이 커지는 비율 [m/s], margin(t)=safety_margin+rate*t
    double reactive_lookahead;  // [s], 논문 기본값 3.0
};

// 공용 객체 운동 예측 결과. 충돌 형상은 이 pose로 재생성하지
// 않고 perception의 authoritative TrackedObject corner를 중심 이동만 시킨다.
struct PredictedObjectPose {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
};

// Perception motion state를 적용한 map-frame velocity. LOW_THREAT만 정지로
// 취급하고 UNKNOWN/PENDING/DYNAMIC은 전달된 vx/vy를 그대로 사용한다.
std::pair<double, double> EffectiveObjectVelocity(
    const TrackedObject& obj);

// yaw_rate가 있으면 현재 vx/vy 벡터 자체를 회전시킨다. OBB heading은
// 형상 방향이므로 이동 벡터 생성에 사용하지 않는다.
std::pair<double, double> PredictTrackedObjectVelocity(
    const TrackedObject& obj, double t);

PredictedObjectPose PredictTrackedObjectPose(const TrackedObject& obj, double t);

geometry::Polygon2D MakeEgoFootprint(const CartesianState& cs,
                                     const VehicleShape& shape,
                                     double margin);

geometry::Polygon2D PredictObstacleFootprint(
    const TrackedObject& obstacle, double prediction_time);

// Invalid geometry/Z is handled conservatively. ego_z may be invalid when the
// Planner has no trustworthy map-frame height; in that case XY SAT still runs.
bool CheckObstacleOverlap(const geometry::Polygon2D& ego_footprint,
                          const geometry::ZRange& ego_z,
                          const TrackedObject& obstacle,
                          double prediction_time);

// 후보 실제 구간과 reactive coast 구간을 평행이동 polygon으로 검사한다.
// AVOID escape ID는 시작 시 이미 안전마진과 겹친 경우 실제 OBB 비충돌이며
// 계속 멀어지는 후보만 탈출할 수 있게 한다. coast exempt ID는 실제 후보
// 구간에는 적용되지 않고 coast 연장에서만 제외된다.
void FilterByCollision(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const std::vector<TrackedObject>& obstacles,
                        const VehicleShape& ego_shape,
                        const CollisionCheckConfig& cfg,
                        int reactive_coast_exempt_object_id = -1,
                        int avoid_escape_object_id = -1);

#endif
