#ifndef FRENET_COLLISION_CHECKER_HPP
#define FRENET_COLLISION_CHECKER_HPP

#include <utility>
#include <vector>

#include "Frenet/Reference/Reference.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include "Frenet/Geometry/CartesianTypes.hpp"
#include "Frenet/Geometry/FrenetTypes.hpp"
#include "Frenet/Geometry/ObjectTypes.hpp"
#include "Frenet/Math/FrenetConverter.hpp"

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
    // Calibrated exterior for road containment only. Zero in manually built
    // test shapes retains width; collision geometry continues to use width.
    double road_outer_width = 0.0; // [m]
};

struct CollisionCheckConfig {
    double safety_margin;       // 공용 궤적 검사에서 실제 차체–OBB의 최단 XY 이격 [m]
    // 미확정/동적 객체의 시간 불확실성 증가율 [m/s]. LOW_THREAT 정적
    // 객체에는 safety_margin만 적용한다.
    double margin_growth_rate;
    double reactive_lookahead;  // [s], 논문 기본값 3.0
};

// Explicit opt-in for lane following / stop-capable approaches. Zero decel
// retains the legacy coast policy used by STOP, AVOID, LC and roundabout MERGE.
struct StoppingContinuationPolicy {
    double decel = 0.0;
    double jerk = 0.0;
    double max_curvature = 0.0;
    double max_lateral_accel = 0.0;
    // Ordinary KEEP may retain an already safe coast continuation. A source
    // approach must prove a stop before its finite road frontier instead.
    bool allow_coast_alternative = false;
};

// Build the safety witness without changing the executable prefix. A partial
// spatial candidate continues its original lateral curve, not a new steering
// correction. Time-polynomial candidates must end with settled lateral motion.
bool BuildStoppingSafetyPath(const FrenetPath& prefix, const RefLine& ref,
    const StoppingContinuationPolicy& policy, double prediction_horizon,
    FrenetPath& checked_path);

// 공용 객체 운동 예측 결과. 측정된 corner는 유지하며 예측 위치로 이동한다.
// 확인된 고주로 차로 모델은 지도 진행 방향의 변화만큼 미래 형상을 강체 회전한다.
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

// Rigidly move the measured corners. Only a confirmed highway lane model
// adds the mapped heading change; time zero always preserves source geometry.
geometry::Corners8 PredictTrackedObjectCorners(const TrackedObject&,double time);
// Advance a derived planning copy from its observation time. Preserve every
// authoritative corner and the original observation held by the caller.
// Precondition: elapsed is finite and nonnegative (input adapter validates it).
TrackedObject AdvanceObjectObservation(const TrackedObject& object, double elapsed);

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

// 모든 Cartesian/Frenet 후보가 공유하는 충돌 판정 결과다. margin recovery는
// 회피 대상의 실제 OBB를 통과시키는 예외가 아니라, 현재 pose가 이미 확장
// safety margin 안에 있을 때 그 margin 밖으로 빠져나가는 과정만 뜻한다.
struct CartesianCollisionCheckResult {
    bool clear = true;
    bool margin_recovery_used = false;
    int blocking_object_id = -1;
    int blocking_sample_index = -1;
};

// 실제 차체 OBB 충돌은 언제나 금지한다. margin_recovery_object_id는 첫
// sample이 이미 확장 margin 안인 경우에만 실제 이격이 개선되는 탈출을
// 허용하며, 밖에서 시작한 후보의 margin 진입과 탈출 후 재진입은 금지한다.
CartesianCollisionCheckResult EvaluateCartesianTrajectorySafety(
    const CartesianPath& path,
    const std::vector<TrackedObject>& obstacles,
    const VehicleShape& ego_shape,
    const CollisionCheckConfig& cfg,
    double dt,
    int margin_recovery_object_id = -1);

// 후보 실제 구간과 reactive coast 구간을 평행이동 polygon으로 검사한다.
// AVOID recovery ID는 시작 시 이미 안전마진과 겹친 경우 실제 OBB 비충돌이며
// 계속 멀어지는 후보만 탈출할 수 있게 한다. coast exempt ID는 실제 후보
// 구간에는 적용되지 않고 coast 연장에서만 제외된다.
// following_lane_half_width > 0 opts an ordinary lane-following request into
// rear-traffic responsibility: a separated, forward-moving object initially
// wholly in this reference corridor is a follower, not a stop obstacle. It
// may leave away from the lane; a predicted return keeps strict checks.
// Current overlap/margin, lateral intrusions and any path
// leaving this corridor remain strict. Zero preserves all maneuver checks.
void FilterByCollision(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const std::vector<TrackedObject>& obstacles,
                        const VehicleShape& ego_shape,
                        const CollisionCheckConfig& cfg,
                        int reactive_coast_exempt_object_id = -1,
                        int margin_recovery_object_id = -1,
                        double following_lane_half_width = 0.0,
                        const StoppingContinuationPolicy* stopping = nullptr,
                        bool reuse_object_predictions = false);

#endif

