#include "frenet_planner/Maneuver/ManeuverUtils.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/Math/FrenetConverter.hpp"

void ProjectObjectToFrenet(const RefLine& ref, const TrackedObject& obj,
                           double& out_s, double& out_d, double& out_s_dot) {
    CartesianState cs{};
    cs.x = obj.x;
    cs.y = obj.y;
    cs.yaw = obj.heading;
    cs.kappa = 0.0;   // CartesianToFrenet의 s_dot/d 계산에는 안 쓰임(FrenetToCartesian 역방향 전용 필드)
    cs.v = obj.speed;
    cs.a = 0.0;       // 가속도 정보 없음 - s_ddot은 안 쓰므로 무관

    double s, s_dot, s_ddot, d, d_prime, d_pprime;
    CartesianToFrenet(ref, cs, s, s_dot, s_ddot, d, d_prime, d_pprime, nullptr);

    out_s = s;
    out_d = d;
    out_s_dot = s_dot;
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
                              int avoid_escape_object_id) {
    if (blocking_id) *blocking_id = -1;
    const size_t count = std::min({path.x.size(), path.y.size(), path.yaw.size()});
    bool target_escape_active = false;
    bool target_margin_exited = false;
    bool target_lateral_progressed = false;
    double target_initial_lateral_distance = 0.0;
    double target_previous_lateral_distance = 0.0;
    const double lateral_backtrack_tolerance =
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
            if (!expanded_overlap) {
                if (obj.id == avoid_escape_object_id &&
                    target_escape_active) {
                    const PredictedObjectPose predicted =
                        PredictTrackedObjectPose(obj, t);
                    const double nx = -std::sin(ego.yaw);
                    const double ny = std::cos(ego.yaw);
                    const double lateral_distance = std::fabs(
                        (predicted.x - ego.x) * nx +
                        (predicted.y - ego.y) * ny);
                    if (std::isfinite(lateral_distance) &&
                        lateral_distance >
                            target_initial_lateral_distance + 1e-6) {
                        target_lateral_progressed = true;
                    }
                    if (!target_lateral_progressed) {
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
                const PredictedObjectPose predicted =
                    PredictTrackedObjectPose(obj, t);
                const double nx = -std::sin(ego.yaw);
                const double ny = std::cos(ego.yaw);
                const double lateral_distance = std::fabs(
                    (predicted.x - ego.x) * nx +
                    (predicted.y - ego.y) * ny);
                if (!physical_overlap && std::isfinite(lateral_distance)) {
                    if (i == 0) {
                        target_escape_active = true;
                        target_initial_lateral_distance = lateral_distance;
                        target_previous_lateral_distance = lateral_distance;
                        allow_target_escape = true;
                    } else if (target_escape_active &&
                               lateral_distance +
                                       lateral_backtrack_tolerance >=
                                   target_previous_lateral_distance) {
                        if (lateral_distance >
                            target_initial_lateral_distance + 1e-6) {
                            target_lateral_progressed = true;
                        }
                        target_previous_lateral_distance = lateral_distance;
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
    if (target_escape_active && !target_lateral_progressed) {
        if (blocking_id) *blocking_id = avoid_escape_object_id;
        return false;
    }
    return true;
}
