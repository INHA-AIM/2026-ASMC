#include "frenet_planner/Maneuver/ManeuverUtils.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/Geometry/PolygonUtils.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"

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
                              int avoid_escape_object_id,
                              TargetEscapePolicy escape_policy) {
    if (blocking_id) *blocking_id = -1;
    const size_t count = std::min({path.x.size(), path.y.size(), path.yaw.size()});
    const double recovery_tx = count > 0 ? std::cos(path.yaw.front()) : 1.0;
    const double recovery_ty = count > 0 ? std::sin(path.yaw.front()) : 0.0;
    const double recovery_nx = -recovery_ty;
    const double recovery_ny = recovery_tx;
    bool target_escape_active = false;
    bool target_margin_exited = false;
    bool target_clearance_progressed = false;
    bool recovery_target_initialized = false;
    bool recovery_target_margin_overlap = false;
    bool target_separation_mode_initialized = false;
    bool target_uses_physical_clearance = false;
    double target_initial_clearance = 0.0;
    double target_max_clearance = 0.0;
    const double clearance_backtrack_tolerance =
        std::max(1e-4,
                 0.1 * std::max(0.0, collision_cfg.safety_margin));
    for (size_t i = 0; i < count; ++i) {
        CartesianState ego{};
        ego.x = path.x[i];
        ego.y = path.y[i];
        ego.yaw = path.yaw[i];
        const double t = static_cast<double>(i) * dt;
        const double margin = collision_cfg.safety_margin +
                              collision_cfg.margin_growth_rate * t;
        const geometry::Polygon2D ego_footprint =
            MakeEgoFootprint(ego, ego_shape, margin);
        for (const auto& obj : obstacles) {
            const bool expanded_overlap =
                CheckObstacleOverlap(ego_footprint, {}, obj, t);
            const PredictedObjectPose predicted =
                PredictTrackedObjectPose(obj, t);

            // 전방 대상은 접근 중 종방향 OBB 간격이 줄어드는 것이 정상이다.
            // 이때는 최초 경로 법선 방향 중심 이격을 쓴다. 반대로 대상 중심이
            // 이미 차체 옆/뒤에 있으면 법선 투영이 차체 회전에 따라 역행할 수
            // 있으므로 실제 두 OBB 경계의 최단 이격을 사용한다.
            const auto escape_separation =
                [&](const geometry::Polygon2D& physical_ego) {
                    if (!target_separation_mode_initialized) {
                        const double longitudinal =
                            (predicted.x - ego.x) * recovery_tx +
                            (predicted.y - ego.y) * recovery_ty;
                        target_uses_physical_clearance = longitudinal <= 0.0;
                        target_separation_mode_initialized = true;
                    }
                    if (target_uses_physical_clearance) {
                        return geometry::ConvexPolygonDistance(
                            physical_ego, PredictObstacleFootprint(obj, t));
                    }
                    return std::fabs(
                        (predicted.x - ego.x) * recovery_nx +
                        (predicted.y - ego.y) * recovery_ny);
                };

            // committed 저속 AVOID의 2차 복구 검사. strict 검사와 달리
            // 경로 시작점이 margin 밖이어도 이후 margin 진입을 허용하지만,
            // 실제 차체 OBB는 전 구간 비충돌이어야 한다. 전방 접근 중에는
            // 법선 방향 이격이, 이미 옆/뒤인 경우에는 실제 OBB 최단 이격이
            // 계속 증가해야 한다. 다른 객체는 이 분기를 타지 않는다.
            if (obj.id == avoid_escape_object_id &&
                escape_policy ==
                    TargetEscapePolicy::CONTINUOUS_OUTWARD_RECOVERY) {
                const geometry::Polygon2D physical_ego =
                    MakeEgoFootprint(ego, ego_shape, 0.0);
                const bool physical_overlap =
                    CheckObstacleOverlap(physical_ego, {}, obj, t);
                const double clearance = escape_separation(physical_ego);
                if (physical_overlap || !std::isfinite(clearance)) {
                    if (blocking_id) *blocking_id = obj.id;
                    return false;
                }
                if (!recovery_target_initialized) {
                    recovery_target_initialized = true;
                    target_initial_clearance = clearance;
                    target_max_clearance = clearance;
                } else {
                    if (clearance + clearance_backtrack_tolerance <
                        target_max_clearance) {
                        if (blocking_id) *blocking_id = obj.id;
                        return false;
                    }
                    if (clearance > target_initial_clearance + 1e-6) {
                        target_clearance_progressed = true;
                    }
                    target_max_clearance = std::max(
                        target_max_clearance, clearance);
                }
                recovery_target_margin_overlap =
                    recovery_target_margin_overlap || expanded_overlap;
                continue;
            }

            if (!expanded_overlap) {
                if (obj.id == avoid_escape_object_id &&
                    target_escape_active) {
                    const geometry::Polygon2D physical_ego =
                        MakeEgoFootprint(ego, ego_shape, 0.0);
                    const double clearance = escape_separation(physical_ego);
                    if (std::isfinite(clearance) &&
                        clearance > target_initial_clearance + 1e-6) {
                        target_clearance_progressed = true;
                    }
                    if (!target_clearance_progressed) {
                        if (blocking_id) *blocking_id = obj.id;
                        return false;
                    }
                    target_escape_active = false;
                    target_margin_exited = true;
                }
                continue;
            }

            bool allow_target_escape = false;
            if (obj.id == avoid_escape_object_id && !target_margin_exited) {
                const geometry::Polygon2D physical_ego =
                    MakeEgoFootprint(ego, ego_shape, 0.0);
                const bool physical_overlap =
                    CheckObstacleOverlap(physical_ego, {}, obj, t);
                const double clearance = escape_separation(physical_ego);
                if (!physical_overlap && std::isfinite(clearance)) {
                    if (i == 0) {
                        target_escape_active = true;
                        target_initial_clearance = clearance;
                        target_max_clearance = clearance;
                        allow_target_escape = true;
                    } else if (target_escape_active &&
                               clearance + clearance_backtrack_tolerance >=
                                   target_max_clearance) {
                        if (clearance > target_initial_clearance + 1e-6) {
                            target_clearance_progressed = true;
                        }
                        target_max_clearance = std::max(
                            target_max_clearance, clearance);
                        allow_target_escape = true;
                    }
                }
            }
            if (!allow_target_escape) {
                if (blocking_id) *blocking_id = obj.id;
                return false;
            }
        }
    }
    if (target_escape_active && !target_clearance_progressed) {
        if (blocking_id) *blocking_id = avoid_escape_object_id;
        return false;
    }
    if (recovery_target_margin_overlap && !target_clearance_progressed) {
        if (blocking_id) *blocking_id = avoid_escape_object_id;
        return false;
    }
    return true;
}
