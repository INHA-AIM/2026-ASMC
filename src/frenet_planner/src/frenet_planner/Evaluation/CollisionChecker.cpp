#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"

PredictedObjectPose PredictLegacyObjectPose(const TrackedObject& obj, double t) {
    PredictedObjectPose pose;
    if (std::fabs(obj.yaw_rate) > 1e-3) {
        pose.x = obj.x + obj.speed / obj.yaw_rate *
            (std::sin(obj.heading + obj.yaw_rate * t) - std::sin(obj.heading));
        pose.y = obj.y - obj.speed / obj.yaw_rate *
            (std::cos(obj.heading + obj.yaw_rate * t) - std::cos(obj.heading));
    } else {
        pose.x = obj.x + obj.speed * std::cos(obj.heading) * t;
        pose.y = obj.y + obj.speed * std::sin(obj.heading) * t;
    }
    pose.heading = obj.heading + obj.yaw_rate * t;
    return pose;
}

geometry::Polygon2D MakeEgoFootprint(const CartesianState& cs,
                                     const VehicleShape& shape,
                                     double margin) {
    const double safe_margin = std::max(0.0, margin);
    const double half_length = shape.length * 0.5 + safe_margin;
    const double half_width = shape.width * 0.5 + safe_margin;
    const double cosine = std::cos(cs.yaw);
    const double sine = std::sin(cs.yaw);
    const double local_xy[4][2] = {
        {-half_length, -half_width},
        { half_length, -half_width},
        { half_length,  half_width},
        {-half_length,  half_width},
    };

    geometry::Polygon2D polygon;
    polygon.reserve(4);
    for (const auto& point : local_xy) {
        polygon.push_back({
            cs.x + cosine * point[0] - sine * point[1],
            cs.y + sine * point[0] + cosine * point[1],
        });
    }
    geometry::EnsureCounterClockwise(polygon);
    return polygon;
}

geometry::Polygon2D PredictObstacleFootprint(
    const TrackedObject& obstacle, double prediction_time) {
    if (!obstacle.geometry_valid || !std::isfinite(obstacle.vx) ||
        !std::isfinite(obstacle.vy) || !std::isfinite(prediction_time)) return {};
    const geometry::Corners8 predicted = geometry::TranslateCorners(
        obstacle.corners, obstacle.vx, obstacle.vy, prediction_time);
    return geometry::ComputeConvexHull(
        geometry::ProjectCornersToXY(predicted));
}

bool CheckObstacleOverlap(const geometry::Polygon2D& ego_footprint,
                          const geometry::ZRange& ego_z,
                          const TrackedObject& obstacle,
                          double prediction_time) {
    if (!obstacle.geometry_valid || !std::isfinite(obstacle.vx) ||
        !std::isfinite(obstacle.vy) || !std::isfinite(prediction_time)) return true;
    const geometry::Corners8 predicted = geometry::TranslateCorners(
        obstacle.corners, obstacle.vx, obstacle.vy, prediction_time);
    geometry::ZRange obstacle_z;
    if (obstacle.z_valid) obstacle_z = geometry::ComputeZRange(predicted);
    if (!geometry::CheckZOverlap(ego_z, obstacle_z)) return false;
    const geometry::Polygon2D footprint = geometry::ComputeConvexHull(
        geometry::ProjectCornersToXY(predicted));
    return geometry::CheckConvexPolygonOverlapSAT(ego_footprint, footprint);
}

// =========================================================
// FilterByCollision
// =========================================================

void FilterByCollision(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const std::vector<TrackedObject>& obstacles,
                        const VehicleShape& ego_shape,
                        const CollisionCheckConfig& cfg,
                        int reactive_coast_exempt_object_id) {
    // 이 값은 나눗셈이 아니라 "정지 상태면 coast 연장 추정을 할 필요가 없다"는
    // 최적화 판단에만 쓰인다 (아래 reactive-lookahead 섹션). 곡률/충돌 본 검사
    // 자체는 더 이상 이 임계값에 의존하지 않는다 (ComputeGeometricPath 참고).
    constexpr double kMinSpeedForCheck = 0.1;  // [m/s]

    for (auto& path : combined) {
        if (!path.valid) continue;

        bool collided = false;

        GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);

        for (size_t i = 0; i < path.t.size() && !collided; i++) {
            CartesianState cs{};
            cs.x = geo.x[i];
            cs.y = geo.y[i];
            cs.yaw = geo.yaw[i];

            const double margin = cfg.safety_margin + cfg.margin_growth_rate * path.t[i];
            const geometry::Polygon2D ego_footprint =
                MakeEgoFootprint(cs, ego_shape, margin);

            for (const auto& obj : obstacles) {
                if (CheckObstacleOverlap(ego_footprint, {}, obj, path.t[i])) {
                    collided = true;
                    path.collision_object_id = obj.id;
                    path.collision_sample_index = static_cast<int>(i);
                    break;
                }
            }
        }

        if (path.valid && !collided && !path.t.empty()) {
            const double dt = (path.t.size() >= 2) ? (path.t[1] - path.t[0]) : 0.1;
            const double last_t     = path.t.back();
            const double last_s     = path.s.back();
            const double last_s_dot = path.s_d.back();
            const double last_d     = path.d.back();
            const double coast_margin = cfg.safety_margin + cfg.margin_growth_rate * last_t;

            if (std::abs(last_s_dot) >= kMinSpeedForCheck) {
                for (double t = last_t + dt; t <= cfg.reactive_lookahead + 1e-9 && !collided; t += dt) {
                    const double s_coast = last_s + last_s_dot * (t - last_t);

                    RefPoint rp = Interpolate(ref, s_coast);
                    // 등속 직진 가정이므로 s_ddot=0, d_dot=0, d_ddot=0 -> d_prime=d_pprime=0
                    CartesianState cs = FrenetToCartesian(rp, s_coast, last_s_dot, 0.0,
                                                           last_d, 0.0, 0.0);

                    const geometry::Polygon2D ego_footprint =
                        MakeEgoFootprint(cs, ego_shape, coast_margin);

                    for (const auto& obj : obstacles) {
                        if (obj.id == reactive_coast_exempt_object_id) continue;
                        if (CheckObstacleOverlap(ego_footprint, {}, obj, t)) {
                            collided = true;
                            path.collision_object_id = obj.id;
                            path.collision_sample_index = static_cast<int>(path.t.size()) - 1;
                            break;
                        }
                    }
                }
            }
        }

        if (collided) {
            path.valid = false;
            path.rejection_reason = RejectionReason::COLLISION;
        }
    }
}
