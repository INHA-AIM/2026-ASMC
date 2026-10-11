#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Math/FrenetConverter.hpp"

#include <algorithm>
#include <cmath>

namespace {

struct CollisionSample {
    CartesianState ego{};
    double time = 0.0;
    bool reactive_coast = false;
};

struct MarginRecoveryState {
    bool initialized = false;
    bool started_inside_margin = false;
    bool exited_margin = false;
    bool clearance_progressed = false;
    bool use_physical_clearance = false;
    double initial_clearance = 0.0;
    double max_clearance = 0.0;
};

double EffectiveSafetyMargin(const CollisionCheckConfig& cfg,
                             const TrackedObject& object,
                             double prediction_time) {
    double margin = std::max(0.0, cfg.safety_margin);
    // LOW_THREAT는 perception이 고정 구조물/정적으로 확정한 상태다. 시간에
    // 따른 위치 불확실성이 누적되지 않으므로 고정 base margin만 적용한다.
    // UNKNOWN/PENDING/DYNAMIC은 전달 속도 예측의 불확실성을 반영해 성장시킨다.
    if (object.motion_state != ObjectMotionState::LOW_THREAT) {
        margin += std::max(0.0, cfg.margin_growth_rate) *
                  std::max(0.0, prediction_time);
    }
    return margin;
}

CartesianCollisionCheckResult EvaluateCollisionSamples(
    const std::vector<CollisionSample>& samples,
    const std::vector<TrackedObject>& obstacles,
    const VehicleShape& ego_shape,
    const CollisionCheckConfig& cfg,
    int margin_recovery_object_id,
    int reactive_coast_exempt_object_id) {
    CartesianCollisionCheckResult result;
    if (samples.empty() || obstacles.empty()) return result;

    const double recovery_tx = std::cos(samples.front().ego.yaw);
    const double recovery_ty = std::sin(samples.front().ego.yaw);
    const double recovery_nx = -recovery_ty;
    const double recovery_ny = recovery_tx;
    const double clearance_backtrack_tolerance =
        std::max(1e-4, 0.1 * std::max(0.0, cfg.safety_margin));
    MarginRecoveryState recovery;

    const auto reject = [&](int object_id, size_t sample_index) {
        CartesianCollisionCheckResult rejected = result;
        rejected.clear = false;
        rejected.blocking_object_id = object_id;
        rejected.blocking_sample_index = static_cast<int>(sample_index);
        return rejected;
    };

    for (size_t sample_index = 0; sample_index < samples.size();
         ++sample_index) {
        const CollisionSample& sample = samples[sample_index];
        for (const TrackedObject& object : obstacles) {
            if (sample.reactive_coast &&
                object.id == reactive_coast_exempt_object_id) {
                continue;
            }

            const double margin =
                EffectiveSafetyMargin(cfg, object, sample.time);
            const geometry::Polygon2D expanded_ego =
                MakeEgoFootprint(sample.ego, ego_shape, margin);
            const bool expanded_overlap = CheckObstacleOverlap(
                expanded_ego, {}, object, sample.time);

            if (object.id != margin_recovery_object_id) {
                // 회피 대상이 아닌 모든 객체는 항상 확장 margin을 강제한다.
                if (expanded_overlap) return reject(object.id, sample_index);
                continue;
            }

            // 회피 대상도 실제 차체 OBB 충돌은 어떤 phase/속도에서도 금지다.
            const geometry::Polygon2D physical_ego =
                MakeEgoFootprint(sample.ego, ego_shape, 0.0);
            if (CheckObstacleOverlap(physical_ego, {}, object, sample.time)) {
                return reject(object.id, sample_index);
            }

            const PredictedObjectPose predicted =
                PredictTrackedObjectPose(object, sample.time);
            if (!recovery.initialized) {
                const double longitudinal =
                    (predicted.x - sample.ego.x) * recovery_tx +
                    (predicted.y - sample.ego.y) * recovery_ty;
                recovery.use_physical_clearance = longitudinal <= 0.0;
            }
            const double clearance = recovery.use_physical_clearance
                ? geometry::ConvexPolygonDistance(
                      physical_ego,
                      PredictObstacleFootprint(object, sample.time))
                : std::fabs((predicted.x - sample.ego.x) * recovery_nx +
                            (predicted.y - sample.ego.y) * recovery_ny);
            if (!std::isfinite(clearance)) {
                return reject(object.id, sample_index);
            }

            if (!recovery.initialized) {
                recovery.initialized = true;
                recovery.started_inside_margin = expanded_overlap;
                recovery.initial_clearance = clearance;
                recovery.max_clearance = clearance;
                result.margin_recovery_used = expanded_overlap;
                continue;
            }

            if (!recovery.started_inside_margin) {
                // margin 밖에서 시작했다면 이후 진입은 recovery가 아니다.
                if (expanded_overlap) return reject(object.id, sample_index);
                continue;
            }

            if (recovery.exited_margin) {
                // 한 번 빠져나간 뒤 다시 margin으로 들어오는 경로는 금지한다.
                if (expanded_overlap) return reject(object.id, sample_index);
                continue;
            }

            if (clearance + clearance_backtrack_tolerance <
                recovery.max_clearance) {
                return reject(object.id, sample_index);
            }
            if (clearance > recovery.initial_clearance + 1e-6) {
                recovery.clearance_progressed = true;
            }
            recovery.max_clearance = std::max(recovery.max_clearance, clearance);

            if (!expanded_overlap) {
                if (!recovery.clearance_progressed) {
                    return reject(object.id, sample_index);
                }
                recovery.exited_margin = true;
            }
        }
    }

    // 경로가 margin 안에서 끝나더라도 실제 이격이 개선됐다면 다음 cycle에
    // 이어서 탈출할 수 있다. 전혀 개선되지 않은 정지/접근 후보만 거부한다.
    if (recovery.initialized && recovery.started_inside_margin &&
        !recovery.exited_margin && !recovery.clearance_progressed) {
        return reject(margin_recovery_object_id, samples.size() - 1);
    }
    return result;
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

CartesianCollisionCheckResult EvaluateCartesianTrajectorySafety(
    const CartesianPath& path,
    const std::vector<TrackedObject>& obstacles,
    const VehicleShape& ego_shape,
    const CollisionCheckConfig& cfg,
    double dt,
    int margin_recovery_object_id) {
    const size_t count =
        std::min({path.x.size(), path.y.size(), path.yaw.size()});
    std::vector<CollisionSample> samples;
    samples.reserve(count);
    for (size_t index = 0; index < count; ++index) {
        CollisionSample sample;
        sample.ego.x = path.x[index];
        sample.ego.y = path.y[index];
        sample.ego.yaw = path.yaw[index];
        sample.time = static_cast<double>(index) * dt;
        samples.push_back(sample);
    }
    return EvaluateCollisionSamples(samples, obstacles, ego_shape, cfg,
                                    margin_recovery_object_id, -1);
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
                        int margin_recovery_object_id) {
    // 이 값은 나눗셈이 아니라 "정지 상태면 coast 연장 추정을 할 필요가 없다"는
    // 최적화 판단에만 쓰인다 (아래 reactive-lookahead 섹션). 곡률/충돌 본 검사
    // 자체는 더 이상 이 임계값에 의존하지 않는다 (ComputeGeometricPath 참고).
    constexpr double kMinSpeedForCheck = 0.1;  // [m/s]

    for (auto& path : combined) {
        if (!path.valid) continue;
        GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
        const size_t count = std::min(
            {path.t.size(), geo.x.size(), geo.y.size(), geo.yaw.size()});
        std::vector<CollisionSample> samples;
        samples.reserve(count + 32);
        for (size_t index = 0; index < count; ++index) {
            CollisionSample sample;
            sample.ego.x = geo.x[index];
            sample.ego.y = geo.y[index];
            sample.ego.yaw = geo.yaw[index];
            sample.time = path.t[index];
            samples.push_back(sample);
        }

        if (!path.t.empty() && !path.s.empty() && !path.s_d.empty() &&
            !path.d.empty()) {
            const double dt = (path.t.size() >= 2) ? (path.t[1] - path.t[0]) : 0.1;
            const double last_t     = path.t.back();
            const double last_s     = path.s.back();
            const double last_s_dot = path.s_d.back();
            const double last_d     = path.d.back();

            if (std::abs(last_s_dot) >= kMinSpeedForCheck) {
                for (double t = last_t + dt;
                     t <= cfg.reactive_lookahead + 1e-9; t += dt) {
                    const double s_coast = last_s + last_s_dot * (t - last_t);

                    RefPoint rp = Interpolate(ref, s_coast);
                    // 등속 직진 가정이므로 s_ddot=0, d_dot=0, d_ddot=0 -> d_prime=d_pprime=0
                    CartesianState cs = FrenetToCartesian(rp, s_coast, last_s_dot, 0.0,
                                                           last_d, 0.0, 0.0);
                    CollisionSample sample;
                    sample.ego = cs;
                    sample.time = t;
                    sample.reactive_coast = true;
                    samples.push_back(sample);
                }
            }
        }

        const CartesianCollisionCheckResult collision =
            EvaluateCollisionSamples(samples, obstacles, ego_shape, cfg,
                                     margin_recovery_object_id,
                                     reactive_coast_exempt_object_id);
        path.margin_recovery_used = collision.margin_recovery_used;
        if (!collision.clear) {
            path.valid = false;
            path.rejection_reason = RejectionReason::COLLISION;
            path.collision_object_id = collision.blocking_object_id;
            path.collision_sample_index = collision.blocking_sample_index;
        }
    }
}
