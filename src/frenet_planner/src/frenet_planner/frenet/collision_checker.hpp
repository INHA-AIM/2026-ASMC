#ifndef FRENET_COLLISION_CHECKER_HPP
#define FRENET_COLLISION_CHECKER_HPP

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/geometry/polygon_utils.hpp"
#include "frenet_planner/global/global.hpp"
#include "frenet_planner/math/frenet_converter.hpp"
#include "frenet_planner/obstacle/tracked_obstacle.hpp"

// =========================================================
// 아래 두 구조체는 논문 Sec.VI의 개념(안전마진, zero/one 판정,
// 시간에 따라 부풀리는 컨투어)을 구체화하기 위해 이 프로젝트가 내린 설계
// 판단이다. 논문은 "얼마나", "어떤 모양으로"는 규정하지 않는다.
// =========================================================

// 자차 충돌판정용 크기. parameter.yaml에서 로드될 예정.
struct VehicleShape {
    double width;   // [m]
    double length;  // [m]
};

struct CollisionCheckConfig {
    double safety_margin;       // 기본 안전마진 [m] (자차 폭/길이에 사방으로 더해짐)
    double margin_growth_rate;  // 시간에 따라 마진이 커지는 비율 [m/s], margin(t)=safety_margin+rate*t
    double reactive_lookahead;  // [s], 논문 기본값 3.0
};

// Highway gap/TTC의 기존 운동 예측만 유지하기 위한 중심 pose다. 충돌 형상은
// 이 값으로 재생성하지 않고 아래 TrackedObstacle corner만 사용한다.
struct PredictedObjectPose {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
};

PredictedObjectPose PredictLegacyObjectPose(const ObjectInfo& obj, double t);

geometry::Polygon2D MakeEgoFootprint(const CartesianState& cs,
                                     const VehicleShape& shape,
                                     double margin);

geometry::Polygon2D PredictObstacleFootprint(
    const TrackedObstacle& obstacle, double prediction_time);

// Invalid geometry/Z is handled conservatively. ego_z may be invalid when the
// Planner has no trustworthy map-frame height; in that case XY SAT still runs.
bool CheckObstacleOverlap(const geometry::Polygon2D& ego_footprint,
                          const geometry::ZRange& ego_z,
                          const TrackedObstacle& obstacle,
                          double prediction_time);

// 후보 실제 구간과 reactive coast 구간을 평행이동 polygon으로 검사한다.
// exempt ID는 실제 후보 구간에는 적용되지 않고 coast 연장에서만 제외된다.
void FilterByCollision(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const std::vector<TrackedObstacle>& obstacles,
                        const VehicleShape& ego_shape,
                        const CollisionCheckConfig& cfg,
                        int reactive_coast_exempt_object_id = -1);

#endif
