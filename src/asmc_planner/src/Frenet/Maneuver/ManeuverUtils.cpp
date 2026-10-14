#include "Frenet/Maneuver/ManeuverUtils.hpp"

#include <algorithm>
#include <cmath>

#include "Frenet/Geometry/PolygonUtils.hpp"
#include "Frenet/Math/FrenetConverter.hpp"

namespace {

bool IsStationaryUnknownFragment(const TrackedObject& object) {
    constexpr int kUnknownType = 0;
    constexpr int kStaticObstacleType = 3;
    const bool static_semantics =
        object.type == kUnknownType || object.type == kStaticObstacleType;
    const bool stationary_motion =
        object.motion_state == ObjectMotionState::UNKNOWN ||
        object.motion_state == ObjectMotionState::LOW_THREAT;
    return static_semantics && stationary_motion;
}

geometry::Polygon2D ObjectFootprint(const TrackedObject& object) {
    if (!object.geometry_valid) return {};
    return geometry::ComputeConvexHull(
        geometry::ProjectCornersToXY(object.corners));
}

}  // namespace

void ProjectObjectToFrenet(const RefLine& ref, const TrackedObject& obj,
                           double& out_s, double& out_d, double& out_s_dot,
                           double ego_projection_search_window) {
    CartesianState cs{};
    cs.x = obj.x;
    cs.y = obj.y;
    cs.yaw = obj.heading;
    cs.kappa = 0.0;   // CartesianToFrenet의 s_dot/d 계산에는 안 쓰임(FrenetToCartesian 역방향 전용 필드)
    cs.v = 0.0;       // s/d 위치만 투영하고 속도는 아래에서 vx/vy로 계산
    cs.a = 0.0;       // 가속도 정보 없음 - s_ddot은 안 쓰므로 무관

    double s, s_dot, s_ddot, d, d_prime, d_pprime;
    CartesianToFrenet(ref, cs, s, s_dot, s_ddot, d, d_prime, d_pprime, nullptr,
                       ego_projection_search_window);

    out_s = s;
    out_d = d;
    const RefPoint local_ref = Interpolate(ref, s);
    const auto velocity = EffectiveObjectVelocity(obj);
    out_s_dot = velocity.first * std::cos(local_ref.theta) +
                velocity.second * std::sin(local_ref.theta);
}

int SynchronizeFollowLeaderFragments(
    int leader_id, std::vector<TrackedObject>& obstacles) {
    const auto leader_it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [leader_id](const TrackedObject& object) {
            return object.id == leader_id;
        });
    if (leader_it == obstacles.end() || !leader_it->geometry_valid) return 0;

    const geometry::Polygon2D leader_footprint = ObjectFootprint(*leader_it);
    const double leader_area = geometry::PolygonArea(leader_footprint);
    if (!(leader_area > 1e-6)) return 0;

    constexpr double kMaxFragmentAreaRatio = 0.50;
    int synchronized = 0;
    for (TrackedObject& object : obstacles) {
        if (object.id == leader_id || !IsStationaryUnknownFragment(object) ||
            !object.geometry_valid) {
            continue;
        }
        const geometry::Polygon2D fragment_footprint =
            ObjectFootprint(object);
        const double fragment_area = geometry::PolygonArea(fragment_footprint);
        if (!(fragment_area > 1e-6) ||
            fragment_area > kMaxFragmentAreaRatio * leader_area ||
            !geometry::CheckConvexPolygonOverlapSAT(
                leader_footprint, fragment_footprint)) {
            continue;
        }

        // OBB/corners는 그대로 둔다. 정적 fragment를 그 자리에 고정해 미래
        // 충돌을 만들지 않도록 이동 상태만 authoritative FOLLOW leader와
        // 맞춘다. 그러면 fragment의 추가 점유 면적은 계속 충돌 검사된다.
        object.vx = leader_it->vx;
        object.vy = leader_it->vy;
        object.speed = leader_it->speed;
        object.yaw_rate = leader_it->yaw_rate;
        object.motion_state = leader_it->motion_state;
        ++synchronized;
    }
    return synchronized;
}

// 폐루프 트랙에서 current_s -> target_s의 전방 호길이.
// 단순 (target - current)는 finish line 이음새에서 음수로 크게 튄다
// (예: ego_s=2200, target_s=20이면 실제로는 전방 30m인데 -2180m).
double ForwardArcDistance(double current_s, double target_s, double s_max) {
    double ds = target_s - current_s;
    if (!std::isfinite(ds)) return ds;
    if (s_max > 0.0) {
        // 이음새를 갓 지난 목표(작은 음수)는 "뒤"가 아니라 한 바퀴 앞이 아니다.
        // 절반 이상 뒤로 떨어진 경우만 wrap으로 간주해 전방 거리로 되돌린다.
        if (ds < -0.5 * s_max) ds += s_max;
        else if (ds > 0.5 * s_max) ds -= s_max;
    }
    return ds;
}

bool CartesianTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObject>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id,
                              int margin_recovery_object_id,
                              bool* margin_recovery_used) {
    const CartesianCollisionCheckResult result =
        EvaluateCartesianTrajectorySafety(
            path, obstacles, ego_shape, collision_cfg, dt,
            margin_recovery_object_id);
    if (blocking_id) *blocking_id = result.blocking_object_id;
    if (margin_recovery_used) {
        *margin_recovery_used = result.margin_recovery_used;
    }
    return result.clear;
}

bool CartesianAvoidTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObject>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id,
                              int margin_recovery_object_id,
                              bool* margin_recovery_used) {
    bool moving_recovery = false;
    if (!CartesianTrajectoryClear(path, obstacles, ego_shape, collision_cfg,
            dt, blocking_id, margin_recovery_object_id, &moving_recovery)) {
        if (margin_recovery_used) *margin_recovery_used = moving_recovery;
        return false;
    }
    std::vector<TrackedObject> stationary;
    for (const auto& object : obstacles) {
        if (object.motion_state != ObjectMotionState::UNKNOWN &&
            object.motion_state != ObjectMotionState::DYNAMIC_PENDING) continue;
        stationary.push_back(object);
        stationary.back().vx = stationary.back().vy = stationary.back().speed = 0.0;
        stationary.back().yaw_rate = 0.0;
    }
    bool stationary_recovery = false;
    const bool clear = CartesianTrajectoryClear(path, stationary, ego_shape,
        collision_cfg, dt, blocking_id, margin_recovery_object_id, &stationary_recovery);
    if (margin_recovery_used) *margin_recovery_used = moving_recovery || stationary_recovery;
    return clear;
}
