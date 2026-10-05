#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"

#include <cmath>

namespace {

// 현재 reference normal에서 ego와 obstacle 중심의 횡방향 거리다.
// 실제 충돌은 별도의 authoritative OBB SAT로 판정한다. 여기에는 차체 yaw에
// 따른 투영 폭을 섞지 않아 곡선에서 정상적인 바깥쪽 이동을 오판하지 않는다.
double LateralCenterDistance(const FrenetPath& path, size_t index,
                             const RefLine& ref,
                             const TrackedObject& obstacle) {
    const RefPoint local_ref = Interpolate(ref, path.s[index]);
    const PredictedObjectPose predicted =
        PredictTrackedObjectPose(obstacle, path.t[index]);
    const double nx = -std::sin(local_ref.theta);
    const double ny = std::cos(local_ref.theta);
    const double object_d = (predicted.x - local_ref.x) * nx +
                            (predicted.y - local_ref.y) * ny;
    return std::fabs(path.d[index] - object_d);
}

}  // namespace

std::pair<double, double> EffectiveObjectVelocity(
    const TrackedObject& object) {
    // LOW_THREAT만 perception이 정적으로 확정한 상태다. 원본 vx/vy는
    // 메시지와 TrackedObject에 보존하되 미래 충돌 위치에는 잡음을 쓰지 않는다.
    if (object.motion_state == ObjectMotionState::LOW_THREAT) {
        return {0.0, 0.0};
    }
    return {object.vx, object.vy};
}

std::pair<double, double> PredictTrackedObjectVelocity(
    const TrackedObject& object, double t) {
    const auto velocity = EffectiveObjectVelocity(object);
    if (!std::isfinite(t) || !std::isfinite(object.yaw_rate)) {
        return velocity;
    }
    const double angle = object.yaw_rate * t;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {cosine * velocity.first - sine * velocity.second,
            sine * velocity.first + cosine * velocity.second};
}

PredictedObjectPose PredictTrackedObjectPose(const TrackedObject& obj, double t) {
    PredictedObjectPose pose;
    const auto velocity = EffectiveObjectVelocity(obj);
    // OBB heading은 형상의 방향이고 이동 방향이 아니다. 중심 이동은 항상
    // perception이 준 map-frame 절대속도 vx/vy로 시작한다. yaw_rate가
    // 있으면 그 속도 벡터를 회전시켜 constant-turn-rate로 적분한다.
    if (std::fabs(obj.yaw_rate) > 1e-3) {
        const double angle = obj.yaw_rate * t;
        const double sine = std::sin(angle);
        const double cosine = std::cos(angle);
        pose.x = obj.x +
            (velocity.first * sine + velocity.second * (cosine - 1.0)) /
                obj.yaw_rate;
        pose.y = obj.y +
            (velocity.first * (1.0 - cosine) + velocity.second * sine) /
                obj.yaw_rate;
    } else {
        pose.x = obj.x + velocity.first * t;
        pose.y = obj.y + velocity.second * t;
    }
    pose.heading = obj.heading + obj.yaw_rate * t;
    return pose;
}

geometry::Polygon2D MakeEgoFootprint(const CartesianState& cs,
                                     const VehicleShape& shape,
                                     double margin) {
    const double safe_margin = std::max(0.0, margin);
    const bool rear_axle_reference =
        std::isfinite(shape.rear_overhang) && shape.rear_overhang >= 0.0 &&
        shape.rear_overhang <= shape.length;
    const double rear_extent = rear_axle_reference
        ? shape.rear_overhang : shape.length * 0.5;
    const double front_extent = shape.length - rear_extent;
    const double half_width = shape.width * 0.5 + safe_margin;
    const double cosine = std::cos(cs.yaw);
    const double sine = std::sin(cs.yaw);
    const double local_xy[4][2] = {
        {-rear_extent - safe_margin, -half_width},
        { front_extent + safe_margin, -half_width},
        { front_extent + safe_margin,  half_width},
        {-rear_extent - safe_margin,  half_width},
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
    const PredictedObjectPose pose =
        PredictTrackedObjectPose(obstacle, prediction_time);
    const geometry::Corners8 predicted = geometry::TranslateCorners(
        obstacle.corners, pose.x - obstacle.x, pose.y - obstacle.y, 1.0);
    return geometry::ComputeConvexHull(
        geometry::ProjectCornersToXY(predicted));
}

bool CheckObstacleOverlap(const geometry::Polygon2D& ego_footprint,
                          const geometry::ZRange& ego_z,
                          const TrackedObject& obstacle,
                          double prediction_time) {
    if (!obstacle.geometry_valid || !std::isfinite(obstacle.vx) ||
        !std::isfinite(obstacle.vy) || !std::isfinite(prediction_time)) return true;
    const PredictedObjectPose pose =
        PredictTrackedObjectPose(obstacle, prediction_time);
    const geometry::Corners8 predicted = geometry::TranslateCorners(
        obstacle.corners, pose.x - obstacle.x, pose.y - obstacle.y, 1.0);
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
                        int reactive_coast_exempt_object_id,
                        int avoid_escape_object_id) {
    // 이 값은 나눗셈이 아니라 "정지 상태면 coast 연장 추정을 할 필요가 없다"는
    // 최적화 판단에만 쓰인다 (아래 reactive-lookahead 섹션). 곡률/충돌 본 검사
    // 자체는 더 이상 이 임계값에 의존하지 않는다 (ComputeGeometricPath 참고).
    constexpr double kMinSpeedForCheck = 0.1;  // [m/s]

    for (auto& path : combined) {
        if (!path.valid) continue;

        bool collided = false;
        bool target_escape_active = false;
        bool target_margin_exited = false;
        bool target_lateral_progressed = false;
        double target_initial_lateral_distance = 0.0;
        double target_previous_lateral_distance = 0.0;
        const double lateral_backtrack_tolerance =
            std::max(1e-4, 0.1 * std::max(0.0, cfg.safety_margin));

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
                const bool expanded_overlap =
                    CheckObstacleOverlap(ego_footprint, {}, obj, path.t[i]);
                if (!expanded_overlap) {
                    if (obj.id == avoid_escape_object_id &&
                        target_escape_active) {
                        const double lateral_distance =
                            LateralCenterDistance(path, i, ref, obj);
                        if (std::isfinite(lateral_distance) &&
                            lateral_distance >
                                target_initial_lateral_distance + 1e-6) {
                            target_lateral_progressed = true;
                        }
                        if (!target_lateral_progressed) {
                            collided = true;
                            path.collision_object_id = obj.id;
                            path.collision_sample_index =
                                static_cast<int>(i);
                            break;
                        }
                        target_escape_active = false;
                        target_margin_exited = true;
                    }
                    continue;
                }

                bool allow_target_escape = false;
                if (obj.id == avoid_escape_object_id &&
                    !target_margin_exited) {
                    const geometry::Polygon2D physical_ego =
                        MakeEgoFootprint(cs, ego_shape, 0.0);
                    const bool physical_overlap =
                        CheckObstacleOverlap(physical_ego, {}, obj, path.t[i]);
                    const double lateral_distance =
                        LateralCenterDistance(path, i, ref, obj);

                    if (!physical_overlap &&
                        std::isfinite(lateral_distance)) {
                        if (i == 0) {
                            // Escape is available only when the current pose
                            // already lies in the inflated margin. It cannot
                            // be used to enter that margin later.
                            target_escape_active = true;
                            target_initial_lateral_distance =
                                lateral_distance;
                            target_previous_lateral_distance =
                                lateral_distance;
                            allow_target_escape = true;
                        } else if (target_escape_active &&
                                   lateral_distance +
                                           lateral_backtrack_tolerance >=
                                       target_previous_lateral_distance) {
                            if (lateral_distance >
                                target_initial_lateral_distance + 1e-6) {
                                target_lateral_progressed = true;
                            }
                            target_previous_lateral_distance =
                                lateral_distance;
                            allow_target_escape = true;
                        }
                    }
                }

                if (!allow_target_escape) {
                    collided = true;
                    path.collision_object_id = obj.id;
                    path.collision_sample_index = static_cast<int>(i);
                    break;
                }
            }
        }

        // 후보 전체가 margin 안에서 끝난 경우에도 종방향 이동만으로는
        // escape가 아니다. 실제 OBB 비충돌을 유지하면서 경로 법선 방향의
        // 중심 간격이 적어도 한 번은 증가해야 한다.
        if (!collided && target_escape_active &&
            !target_lateral_progressed) {
            collided = true;
            path.collision_object_id = avoid_escape_object_id;
            path.collision_sample_index =
                path.t.empty() ? -1 : static_cast<int>(path.t.size() - 1);
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
