#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Math/FrenetConverter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <stdexcept>

namespace {

// Speed-interval sweeps may contain many repeated measured corners. Avoid
// the general O(n^2) tolerance-deduplication pass in this bounded hot path.
geometry::Polygon2D LaneSweepHull(geometry::Polygon2D points) {
    if(!std::all_of(points.begin(),points.end(),[](const geometry::Point2D& p){return std::isfinite(p.x)&&std::isfinite(p.y);}))return {};
    std::sort(points.begin(),points.end(),[](const geometry::Point2D& a,const geometry::Point2D& b){
        return a.x<b.x||(a.x==b.x&&a.y<b.y);
    });
    if(points.size()<3)return points;
    auto cross=[](const geometry::Point2D& a,const geometry::Point2D& b,const geometry::Point2D& c){
        return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
    };
    geometry::Polygon2D hull(2*points.size());std::size_t count=0;
    for(const auto& p:points){while(count>=2&&cross(hull[count-2],hull[count-1],p)<=1e-9)--count;hull[count++]=p;}
    const auto lower=count;
    for(std::size_t i=points.size()-1;i>0;--i){const auto p=points[i-1];
        while(count>lower&&cross(hull[count-2],hull[count-1],p)<=1e-9)--count;hull[count++]=p;}
    if(count)--count;hull.resize(count);geometry::EnsureCounterClockwise(hull);return hull;
}

struct CollisionSample {
    CartesianState ego{};
    double time = 0.0;
    bool reactive_coast = false;
    double reference_s = 0.0;
};

// Local projection is bounded and starts at an already known station. Unlike
// a global nearest-waypoint search it cannot jump to a nearby parallel road.
bool FollowingLaneProjection(const RefLine& ref, const geometry::Point2D& point,
                             double hint, double& s, double& d) {
    s = hint;
    if (ref.points.size() < 2 || !std::isfinite(s) ||
        !std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    for (int iteration = 0; iteration < 20; ++iteration) {
        const auto rp = Interpolate(ref, s);
        const double tx = std::cos(rp.theta), ty = std::sin(rp.theta);
        const double dx = point.x-rp.x, dy = point.y-rp.y;
        const double along = dx*tx+dy*ty;
        d = -dx*ty+dy*tx;
        const double metric = 1-rp.kappa*d;
        if (!std::isfinite(d) || !std::isfinite(metric) || metric <= 0.0)
            return false;
        if (std::abs(along) < 1e-6) return true;
        s += along/metric;
        if (!std::isfinite(s)) return false;
        if (ref.closed_loop && ref.length > 0.0) {
            s = std::fmod(s, ref.length);
            if (s < 0.0) s += ref.length;
        } else {
            // A finite reference supplies no following-lane contract beyond
            // either end. Keep the original strict collision policy there.
            if (s < ref.points.front().s || s > ref.points.back().s)
                return false;
        }
    }
    return false;
}

bool InsideFollowingLane(const geometry::Polygon2D& footprint,
                         const RefLine& ref, double hint, double half_width) {
    if (footprint.size() < 3) return false;
    for (const auto& point : footprint) {
        double s, d;
        if (!FollowingLaneProjection(ref, point, hint, s, d) ||
            std::abs(d) > half_width) return false;
    }
    return true;
}

std::vector<bool> FollowingRearObjects(
    const CollisionSample& initial, const RefLine& ref,
    const std::vector<TrackedObject>& obstacles, const VehicleShape& shape,
    const CollisionCheckConfig& cfg, double half_width, double dt, double horizon) {
    std::vector<bool> following(obstacles.size(), false);
    if (!std::isfinite(half_width) || half_width <= 0.0 ||
        !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(horizon) ||
        horizon < 0.0 || horizon/dt > 1000 || ref.points.size() < 2)
        return following;
    const auto ego_body = MakeEgoFootprint(initial.ego, shape, 0.0);
    if (!InsideFollowingLane(ego_body, ref, initial.reference_s, half_width))
        return following;
    const double rear = shape.rear_overhang >= 0.0
        ? shape.rear_overhang : shape.length*0.5;
    const double tx = std::cos(initial.ego.yaw), ty = std::sin(initial.ego.yaw);
    for (size_t index = 0; index < obstacles.size(); ++index) {
        const auto& object = obstacles[index];
        if (!object.geometry_valid || object.motion_state == ObjectMotionState::LOW_THREAT ||
            !std::isfinite(object.vx) || !std::isfinite(object.vy) ||
            object.vx*tx+object.vy*ty <= 0.0 ||
            std::isfinite(object.prediction_min_speed) ||
            std::isfinite(object.prediction_max_speed)) continue;
        const auto body = PredictObstacleFootprint(object, 0.0);
        if (body.size() < 3 || geometry::ConvexPolygonDistance(ego_body, body) <=
            std::max(0.0, cfg.safety_margin)+1e-9) continue;
        bool behind = true;
        for (const auto& corner : body) {
            if ((corner.x-initial.ego.x)*tx+(corner.y-initial.ego.y)*ty >=
                -rear-std::max(0.0, cfg.safety_margin)) behind = false;
        }
        if (!behind) continue;
        const double window = std::hypot(object.x-initial.ego.x,
                                        object.y-initial.ego.y)+shape.length;
        double station = FindClosestS(ref, object.x, object.y,
                                      &initial.reference_s, window);
        bool follows_without_intrusion = true, departed = false;
        double last_departed_lateral = 0.0;
        const auto model_agrees=[&](double time,double hint) {
            if(!object.prediction_lane)return false;
            const double speed=std::hypot(object.vx,object.vy);
            const double end=std::min(object.prediction_lane->length,object.prediction_lane_end_s);
            const double s=object.prediction_lane_s+speed*time;
            if(s<0||s>end)return false;
            const auto point=Interpolate(*object.prediction_lane,s);double projected,lateral;
            if(!FollowingLaneProjection(ref,{point.x,point.y},hint,projected,lateral))return false;
            const auto own=Interpolate(ref,projected);
            return std::fabs(lateral)<=.35&&std::fabs(std::remainder(point.theta-own.theta,2*M_PI))<=.08;
        };
        const int count = static_cast<int>(std::ceil(horizon/dt));
        for (int sample = 0; sample <= count; ++sample) {
            const double time = std::min(sample*dt, horizon);
            const auto pose = PredictTrackedObjectPose(object, time);
            double projected, lateral;
            if (!FollowingLaneProjection(ref, {pose.x, pose.y}, station, projected, lateral)) {
                follows_without_intrusion = false; break;
            }
            // A confirmed same-lane follower can have an OBB straddling a
            // paint boundary. Lane assignment is not whole-box containment.
            // Preserve strict current clearance, wholly-behind and ego-lane
            // containment checks above. Adjacent lane models never qualify.
            const bool inside = object.prediction_lane?
                (std::fabs(lateral)<=half_width&&model_agrees(time,projected)):
                InsideFollowingLane(PredictObstacleFootprint(object, time),ref,projected,half_width);
            if (sample == 0 && !inside) {follows_without_intrusion = false;break;}
            // A follower departing away from this lane is not a cut-in. A
            // return toward it after departure IS an intrusion and restores
            // strict prediction for this object over the whole candidate.
            if (departed && (inside || std::abs(lateral)+1e-6 < last_departed_lateral)) {
                follows_without_intrusion = false;break;
            }
            if (!inside) {departed=true;last_departed_lateral=std::abs(lateral);}
            // Also reject a reversing/oncoming hypothesis. Position-supported
            // continuation and bounded-turn predictions use this same motion.
            const auto next = PredictTrackedObjectPose(object, time+dt);
            const auto rp = Interpolate(ref, projected);
            if ((next.x-pose.x)*std::cos(rp.theta)+
                (next.y-pose.y)*std::sin(rp.theta) < 0.0) {
                follows_without_intrusion = false; break;
            }
            station = projected;
        }
        following[index] = follows_without_intrusion;
    }
    return following;
}

bool FollowsLaneCorridor(const std::vector<CollisionSample>& samples,
                         const RefLine& ref, const VehicleShape& shape,
                         double half_width) {
    for (const auto& sample : samples)
        if (sample.ego.v < 0.0 ||
            !InsideFollowingLane(MakeEgoFootprint(sample.ego, shape, 0.0),
                                 ref, sample.reference_s, half_width)) return false;
    return true;
}

struct MarginRecoveryState {
    bool initialized = false;
    bool started_inside_margin = false;
    bool exited_margin = false;
    bool clearance_progressed = false;
    bool use_physical_clearance = false;
    double initial_clearance = 0.0;
    double max_clearance = 0.0;
};

// One FilterByCollision invocation owns one immutable obstacle vector.
// Index by vector position rather than track ID; two detections may share an
// ID and later observations must never inherit an earlier prediction.
class PredictionFootprintCache {
public:
    explicit PredictionFootprintCache(std::size_t objects) : values_(objects) {}
    const geometry::Polygon2D* get(std::size_t index,
        const TrackedObject& object, double time) {
        std::uint64_t key;
        static_assert(sizeof(key)==sizeof(time), "prediction time key");
        std::memcpy(&key,&time,sizeof(key));
        auto& values=values_[index];
        const auto found=values.find(key);
        if(found!=values.end())return &found->second;
        // Resource bounds affect reuse only. On exhaustion the caller runs
        // the original prediction; no sample or obstacle is skipped.
        if(entries_>=8192||vertices_>=65536)return nullptr;
        auto polygon=PredictObstacleFootprint(object,time);
        if(polygon.size()>65536-vertices_)return nullptr;
        vertices_+=polygon.size();++entries_;
        return &values.emplace(key,std::move(polygon)).first->second;
    }
private:
    std::vector<std::unordered_map<std::uint64_t,geometry::Polygon2D>> values_;
    std::size_t entries_=0,vertices_=0;
};

double EffectiveSafetyMargin(const CollisionCheckConfig& cfg,
                             const TrackedObject& object,
                             double prediction_time) {
    double margin = std::max(0.0, cfg.safety_margin);
    // LOW_THREAT는 perception이 고정 구조물/정적으로 확정한 상태다. 시간에
    // 따른 위치 불확실성이 누적되지 않으므로 고정 base margin만 적용한다.
    // UNKNOWN/PENDING/DYNAMIC은 전달 속도 예측의 불확실성을 반영해 성장시킨다.
    if (object.velocity_unavailable || object.motion_state != ObjectMotionState::LOW_THREAT) {
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
    int reactive_coast_exempt_object_id,
    const std::vector<bool>* following_rear = nullptr,
    PredictionFootprintCache* predictions = nullptr) {
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
        const geometry::Polygon2D physical_ego =
            MakeEgoFootprint(sample.ego, ego_shape, 0.0);
        for (size_t object_index = 0; object_index < obstacles.size(); ++object_index) {
            const TrackedObject& object = obstacles[object_index];
            // This is a normal-following responsibility policy, not permission
            // to hit an object: ALL current bodies/margins remain checked, and
            // a predicted lane intrusion or ego departure disables this policy.
            // A distant follower's constant-velocity rear-end extrapolation is
            // not a longitudinal stop obstacle for the vehicle ahead of it.
            if (sample.time > 0.0 && following_rear && (*following_rear)[object_index])
                continue;
            if (sample.reactive_coast &&
                object.id == reactive_coast_exempt_object_id) {
                continue;
            }

            const double margin =
                EffectiveSafetyMargin(cfg, object, sample.time);
            // Euclidean clearance keeps the same margin around edges and
            // corners. Expanding a rectangle by margin on both axes otherwise
            // reserves sqrt(2) * margin at its corners.
            const auto* cached=predictions
                ? predictions->get(object_index,object,sample.time) : nullptr;
            const auto uncached_polygon=cached ? geometry::Polygon2D{}
                : PredictObstacleFootprint(object,sample.time);
            const double physical_clearance = geometry::ConvexPolygonDistance(
                physical_ego, cached ? *cached : uncached_polygon);
            // Distance is conservatively zero for invalid geometry. Boundary
            // contact also remains blocked, including a zero configured margin.
            const bool inside_margin = physical_clearance <= margin + 1e-9;

            if (object.id != margin_recovery_object_id) {
                // 회피 대상이 아닌 모든 객체에는 최단거리 margin을 강제한다.
                if (inside_margin) return reject(object.id, sample_index);
                continue;
            }

            // 회피 대상도 실제 차체 OBB 충돌은 어떤 phase/속도에서도 금지다.
            if (physical_clearance <= 1e-9) {
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
                ? physical_clearance
                : std::fabs((predicted.x - sample.ego.x) * recovery_nx +
                            (predicted.y - sample.ego.y) * recovery_ny);
            if (!std::isfinite(clearance)) {
                return reject(object.id, sample_index);
            }

            if (!recovery.initialized) {
                recovery.initialized = true;
                recovery.started_inside_margin = inside_margin;
                recovery.initial_clearance = clearance;
                recovery.max_clearance = clearance;
                result.margin_recovery_used = inside_margin;
                continue;
            }

            if (!recovery.started_inside_margin) {
                // margin 밖에서 시작했다면 이후 진입은 recovery가 아니다.
                if (inside_margin) return reject(object.id, sample_index);
                continue;
            }

            if (recovery.exited_margin) {
                // 한 번 빠져나간 뒤 다시 margin으로 들어오는 경로는 금지한다.
                if (inside_margin) return reject(object.id, sample_index);
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

            if (!inside_margin) {
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
    if (object.velocity_unavailable || object.motion_state == ObjectMotionState::LOW_THREAT) {
        return {0.0, 0.0};
    }
    return {object.vx, object.vy};
}

std::pair<double, double> PredictTrackedObjectVelocity(
    const TrackedObject& object, double t) {
    if(object.prediction_origin)
        return PredictTrackedObjectVelocity(*object.prediction_origin,
            object.prediction_origin_offset_s+t);
    const auto velocity = EffectiveObjectVelocity(object);
    if(object.velocity_unavailable)return velocity;
    if(object.prediction_lane&&object.motion_state!=ObjectMotionState::LOW_THREAT&&std::isfinite(t)) {
        const double speed=std::hypot(velocity.first,velocity.second);
        const double end=std::min(object.prediction_lane->length,object.prediction_lane_end_s);
        const double station=object.prediction_lane_s+speed*t;
        const auto p=Interpolate(*object.prediction_lane,std::clamp(station,0.0,end));
        const double factor=station>end?1:1.0-p.kappa*object.prediction_lane_d;
        return {speed*factor*std::cos(p.theta),speed*factor*std::sin(p.theta)};
    }
    if (!std::isfinite(t) || !std::isfinite(object.yaw_rate)) {
        return velocity;
    }
    const double turn_time = std::min(t, std::max(0.0, object.turn_prediction_duration));
    if (t > turn_time && object.motion_state != ObjectMotionState::LOW_THREAT &&
        std::isfinite(object.continuation_vx) && std::isfinite(object.continuation_vy)) {
        return {object.continuation_vx, object.continuation_vy};
    }
    const double angle = object.yaw_rate * turn_time;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {cosine * velocity.first - sine * velocity.second,
            sine * velocity.first + cosine * velocity.second};
}

PredictedObjectPose PredictTrackedObjectPose(const TrackedObject& obj, double t) {
    if(obj.prediction_origin)
        return PredictTrackedObjectPose(*obj.prediction_origin,obj.prediction_origin_offset_s+t);
    if(obj.velocity_unavailable)return {obj.x,obj.y,obj.heading};
    if(t==0)return {obj.x,obj.y,obj.heading};
    PredictedObjectPose pose;
    const auto velocity = EffectiveObjectVelocity(obj);
    if(obj.prediction_lane&&obj.motion_state!=ObjectMotionState::LOW_THREAT&&std::isfinite(t)) {
        const auto& line=*obj.prediction_lane;
        const double station=obj.prediction_lane_s+std::hypot(velocity.first,velocity.second)*t;
        const double end=std::min(line.length,obj.prediction_lane_end_s);
        auto point=[&](double s) {
            const double bounded=std::clamp(s,0.0,end);
            const auto p=Interpolate(line,bounded);
            const double extra=s-bounded;
            return PredictedObjectPose{p.x+extra*std::cos(p.theta)-obj.prediction_lane_d*std::sin(p.theta),
                p.y+extra*std::sin(p.theta)+obj.prediction_lane_d*std::cos(p.theta),p.theta};
        };
        const auto initial=std::isfinite(obj.prediction_lane_anchor_x)&&std::isfinite(obj.prediction_lane_anchor_y)&&
            std::isfinite(obj.prediction_lane_heading)?
            PredictedObjectPose{obj.prediction_lane_anchor_x,obj.prediction_lane_anchor_y,obj.prediction_lane_heading}:
            point(obj.prediction_lane_s);
        const auto future=point(station);
        // The current measured center is not snapped to the HD-map line.
        return {obj.x+future.x-initial.x,obj.y+future.y-initial.y,
            obj.heading+std::remainder(future.heading-initial.heading,2*M_PI)};
    }
    // OBB heading은 형상의 방향이고 이동 방향이 아니다. 중심 이동은 항상
    // perception이 준 map-frame 절대속도 vx/vy로 시작한다. yaw_rate가
    // 있으면 그 속도 벡터를 회전시켜 constant-turn-rate로 적분한다.
    if (std::fabs(obj.yaw_rate) > 1e-3) {
        const double turn_time = std::min(t, std::max(0.0, obj.turn_prediction_duration));
        const double angle = obj.yaw_rate * turn_time;
        const double sine = std::sin(angle);
        const double cosine = std::cos(angle);
        pose.x = obj.x +
            (velocity.first * sine + velocity.second * (cosine - 1.0)) /
                obj.yaw_rate;
        pose.y = obj.y +
            (velocity.first * (1.0 - cosine) + velocity.second * sine) /
                obj.yaw_rate;
    } else {
        const double first_time = std::min(t, std::max(0.0, obj.turn_prediction_duration));
        const bool continuation = obj.motion_state != ObjectMotionState::LOW_THREAT && std::isfinite(obj.continuation_vx) && std::isfinite(obj.continuation_vy);
        pose.x = obj.x + velocity.first * first_time +
            (continuation ? obj.continuation_vx : velocity.first) * (t-first_time);
        pose.y = obj.y + velocity.second * first_time +
            (continuation ? obj.continuation_vy : velocity.second) * (t-first_time);
    }
    const double turn_time = std::min(t, std::max(0.0, obj.turn_prediction_duration));
    const double angle = obj.yaw_rate * turn_time;
    if (std::fabs(obj.yaw_rate) > 1e-3) {
        const double tangent_time = t - turn_time;
        const bool continuation = obj.motion_state != ObjectMotionState::LOW_THREAT && std::isfinite(obj.continuation_vx) && std::isfinite(obj.continuation_vy);
        pose.x += (continuation ? obj.continuation_vx :
            velocity.first * std::cos(angle) - velocity.second * std::sin(angle)) * tangent_time;
        pose.y += (continuation ? obj.continuation_vy :
            velocity.first * std::sin(angle) + velocity.second * std::cos(angle)) * tangent_time;
    }
    pose.heading = obj.heading + angle;
    return pose;
}

geometry::Corners8 PredictTrackedObjectCorners(const TrackedObject& object,double time) {
    if(object.prediction_origin)
        return PredictTrackedObjectCorners(*object.prediction_origin,
            object.prediction_origin_offset_s+time);
    if(object.velocity_unavailable)return object.corners;
    if(time==0)return object.corners;
    const auto pose=PredictTrackedObjectPose(object,time);
    if(!object.prediction_lane)return geometry::TranslateCorners(object.corners,pose.x-object.x,pose.y-object.y,1.0);
    auto result=object.corners;const double angle=pose.heading-object.heading;
    const double c=std::cos(angle),s=std::sin(angle);
    for(auto& p:result){const double x=p.x-object.x,y=p.y-object.y;p.x=pose.x+c*x-s*y;p.y=pose.y+s*x+c*y;}
    return result;
}

TrackedObject AdvanceObjectObservation(const TrackedObject& object, double elapsed) {
    if(!std::isfinite(elapsed)||elapsed<0)
        throw std::invalid_argument("invalid object observation alignment");
    if(elapsed==0)return object;
    auto advanced=object;
    advanced.prediction_origin=object.prediction_origin?object.prediction_origin:
        std::make_shared<const TrackedObject>(object);
    advanced.prediction_origin_offset_s=object.prediction_origin_offset_s+elapsed;
    const auto& origin=*advanced.prediction_origin;
    const double time=advanced.prediction_origin_offset_s;
    const auto pose=PredictTrackedObjectPose(origin,time);
    const auto velocity=PredictTrackedObjectVelocity(origin,time);
    advanced.corners=PredictTrackedObjectCorners(origin,time);
    advanced.x=pose.x;advanced.y=pose.y;advanced.heading=pose.heading;
    advanced.vx=velocity.first;advanced.vy=velocity.second;
    advanced.speed=std::hypot(advanced.vx,advanced.vy);
    if(origin.prediction_lane && origin.motion_state!=ObjectMotionState::LOW_THREAT) {
        advanced.prediction_lane_s=origin.prediction_lane_s+
            std::hypot(EffectiveObjectVelocity(origin).first,EffectiveObjectVelocity(origin).second)*time;
        const double end=std::min(origin.prediction_lane->length,origin.prediction_lane_end_s);
        const double bounded=std::clamp(advanced.prediction_lane_s,0.0,end);
        const auto point=Interpolate(*origin.prediction_lane,bounded);
        const double extra=advanced.prediction_lane_s-bounded;
        advanced.prediction_lane_anchor_x=point.x+extra*std::cos(point.theta)-origin.prediction_lane_d*std::sin(point.theta);
        advanced.prediction_lane_anchor_y=point.y+extra*std::sin(point.theta)+origin.prediction_lane_d*std::cos(point.theta);
        advanced.prediction_lane_heading=point.theta;
        advanced.prediction_lane_curvature=point.kappa;
    }
    return advanced;
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
    if(obstacle.prediction_origin)
        return PredictObstacleFootprint(*obstacle.prediction_origin,
            obstacle.prediction_origin_offset_s+prediction_time);
    if (!obstacle.geometry_valid || !std::isfinite(obstacle.vx) ||
        !std::isfinite(obstacle.vy) || !std::isfinite(prediction_time)) return {};
    const double speed=std::hypot(obstacle.vx,obstacle.vy);
    if(obstacle.motion_state!=ObjectMotionState::LOW_THREAT&&speed>1e-6&&
       (std::isfinite(obstacle.prediction_min_speed)||std::isfinite(obstacle.prediction_max_speed))) {
        const double low=obstacle.prediction_min_speed,high=obstacle.prediction_max_speed;
        if(!std::isfinite(low)||!std::isfinite(high)||low<0||low>speed||high<speed)return {};
        if(obstacle.prediction_lane) {
            // A curved reference is not affine in speed: endpoint rectangles
            // alone miss intermediate occupancy. Include every reference knot
            // in the speed interval, retaining the original measured corners.
            const double a=obstacle.prediction_lane_s+low*prediction_time;
            const double b=obstacle.prediction_lane_s+high*prediction_time;
            const double begin=std::min(a,b),end=std::max(a,b);
            std::vector<double> stations{begin};
            const auto& points=obstacle.prediction_lane->points;
            const double lane_end=std::min(obstacle.prediction_lane->length,obstacle.prediction_lane_end_s);
            auto it=std::upper_bound(points.begin(),points.end(),begin,
                [](double s,const RefPoint& p){return s<p.s;});
            for(;it!=points.end()&&it->s<end&&it->s<lane_end;++it)stations.push_back(it->s);
            if(lane_end>begin&&lane_end<end)stations.push_back(lane_end);
            stations.push_back(end);
            if(stations.size()>4096)return {};
            geometry::Polygon2D swept;double maximum_angle=0,previous_heading=0;
            for(std::size_t i=0;i<stations.size();++i) {
                auto bounded=obstacle;
                const double value=std::fabs(prediction_time)>1e-9?
                    (stations[i]-obstacle.prediction_lane_s)/prediction_time:speed;
                bounded.vx*=value/speed;bounded.vy*=value/speed;
                const auto corners=PredictTrackedObjectCorners(bounded,prediction_time);
                const auto xy=geometry::ProjectCornersToXY(corners);swept.insert(swept.end(),xy.begin(),xy.end());
                const double heading=Interpolate(*obstacle.prediction_lane,std::clamp(stations[i],0.0,lane_end)).theta;
                if(i)maximum_angle=std::max(maximum_angle,std::fabs(std::remainder(heading-previous_heading,2*M_PI)));
                previous_heading=heading;
            }
            auto hull=LaneSweepHull(std::move(swept));
            // Interpolate uses linear centers and rotating lateral/body
            // offsets. Bound their deviation from the endpoint chord.
            double radius=0;for(const auto& p:obstacle.corners)radius=std::max(radius,std::hypot(p.x-obstacle.x,p.y-obstacle.y));
            const double guard=(std::fabs(obstacle.prediction_lane_d)+radius)*
                maximum_angle*maximum_angle/8;
            if(guard>1e-12) {
                geometry::Polygon2D expanded;
                for(const auto& p:hull)for(double dx:{-guard,guard})for(double dy:{-guard,guard})
                    expanded.push_back({p.x+dx,p.y+dy});
                hull=LaneSweepHull(std::move(expanded));
            }
            return hull;
        }
        geometry::Polygon2D points;
        for(double bounded_speed:{low,high}) {
            auto bounded=obstacle;
            bounded.vx*=bounded_speed/speed;bounded.vy*=bounded_speed/speed;
            const auto pose=PredictTrackedObjectPose(bounded,prediction_time);
            const auto corners=geometry::TranslateCorners(obstacle.corners,
                pose.x-obstacle.x,pose.y-obstacle.y,1.0);
            const auto xy=geometry::ProjectCornersToXY(corners);
            points.insert(points.end(),xy.begin(),xy.end());
        }
        // Constant-turn translation (including fixed continuation velocity)
        // is affine in initial speed. The endpoint hull covers every speed
        // between them, avoiding sparse-speed sampling holes.
        return geometry::ComputeConvexHull(points);
    }
    const geometry::Corners8 predicted=PredictTrackedObjectCorners(obstacle,prediction_time);
    return geometry::ComputeConvexHull(
        geometry::ProjectCornersToXY(predicted));
}

bool CheckObstacleOverlap(const geometry::Polygon2D& ego_footprint,
                          const geometry::ZRange& ego_z,
                          const TrackedObject& obstacle,
                          double prediction_time) {
    if (!obstacle.geometry_valid || !std::isfinite(obstacle.vx) ||
        !std::isfinite(obstacle.vy) || !std::isfinite(prediction_time)) return true;
    const geometry::Corners8 predicted=PredictTrackedObjectCorners(obstacle,prediction_time);
    geometry::ZRange obstacle_z;
    if (obstacle.z_valid) obstacle_z = geometry::ComputeZRange(predicted);
    if (!geometry::CheckZOverlap(ego_z, obstacle_z)) return false;
    const bool speed_interval=obstacle.motion_state!=ObjectMotionState::LOW_THREAT&&
        (std::isfinite(obstacle.prediction_min_speed)||std::isfinite(obstacle.prediction_max_speed));
    const geometry::Polygon2D footprint = speed_interval
        ? PredictObstacleFootprint(obstacle,prediction_time)
        : geometry::ComputeConvexHull(geometry::ProjectCornersToXY(predicted));
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
                        int margin_recovery_object_id,
                        double following_lane_half_width,
                        const StoppingContinuationPolicy* stopping,
                        bool reuse_object_predictions) {
    if(stopping&&stopping->allow_coast_alternative) {
        // Preserve every existing safe plan. Only a conflict in the speculative
        // COAST extension may be rescued by a checked braking alternative;
        // collision anywhere in the executable prefix remains a hard reject.
        std::vector<bool> originally_valid;
        for(const auto& path:combined)originally_valid.push_back(path.valid);
        FilterByCollision(combined,ref,obstacles,ego_shape,cfg,
            reactive_coast_exempt_object_id,margin_recovery_object_id,following_lane_half_width,
            nullptr,reuse_object_predictions);
        auto braking=*stopping;braking.allow_coast_alternative=false;
        for(std::size_t i=0;i<combined.size();++i) {
            auto& path=combined[i];
            if(!originally_valid[i]||path.valid||path.rejection_reason!=RejectionReason::COLLISION||
               path.collision_sample_index<static_cast<int>(path.t.size()))continue;
            std::vector<FrenetPath> alternative{path};alternative.front().valid=true;
            FilterByCollision(alternative,ref,obstacles,ego_shape,cfg,
                reactive_coast_exempt_object_id,margin_recovery_object_id,following_lane_half_width,
                &braking,reuse_object_predictions);
            if(alternative.front().valid) {
                path=std::move(alternative.front());path.rejection_reason=RejectionReason::NONE;
                path.collision_object_id=path.collision_sample_index=-1;
            }
        }
        return;
    }
    // 이 값은 나눗셈이 아니라 "정지 상태면 coast 연장 추정을 할 필요가 없다"는
    // 최적화 판단에만 쓰인다 (아래 reactive-lookahead 섹션). 곡률/충돌 본 검사
    // 자체는 더 이상 이 임계값에 의존하지 않는다 (ComputeGeometricPath 참고).
    constexpr double kMinSpeedForCheck = 0.1;  // [m/s]
    PredictionFootprintCache predictions(reuse_object_predictions?obstacles.size():0);

    double horizon = std::max(0.0, cfg.reactive_lookahead), prediction_dt = 0.1;
    for (const auto& path : combined) {
        if (!path.valid || path.t.empty()) continue;
        horizon = std::max(horizon, path.t.back());
        if (path.t.size() > 1 && path.t[1] > path.t[0])
            prediction_dt = std::min(prediction_dt, path.t[1]-path.t[0]);
    }
    std::vector<bool> following_rear;
    CollisionSample following_origin;
    bool following_prepared = false;

    for (auto& path : combined) {
        if (!path.valid) continue;
        FrenetPath safety_path;
        if(stopping && !BuildStoppingSafetyPath(path,ref,*stopping,cfg.reactive_lookahead,safety_path)) {
            path.valid=false;path.rejection_reason=RejectionReason::CURVATURE;continue;
        }
        const auto& evaluated=stopping?safety_path:path;
        if(stopping&&evaluated.t.back()>horizon) {
            horizon=evaluated.t.back();following_prepared=false;
        }
        GeometricPath geo;
        if(stopping) {
            const auto cart=ConvertToCartesianPath(evaluated,ref);
            geo.x=cart.x;geo.y=cart.y;geo.yaw=cart.yaw;geo.kappa=cart.kappa;
        } else geo=ComputeGeometricPath(path.s,path.d,ref);
        const size_t count = std::min(
            {evaluated.t.size(), geo.x.size(), geo.y.size(), geo.yaw.size()});
        std::vector<CollisionSample> samples;
        samples.reserve(count + 32);
        for (size_t index = 0; index < count; ++index) {
            CollisionSample sample;
            sample.ego.x = geo.x[index];
            sample.ego.y = geo.y[index];
            sample.ego.yaw = geo.yaw[index];
            if (index < evaluated.s_d.size()) sample.ego.v = evaluated.s_d[index];
            sample.time = evaluated.t[index];
            sample.reference_s = evaluated.s[index];
            samples.push_back(sample);
        }

        if (!stopping && !path.t.empty() && !path.s.empty() && !path.s_d.empty() &&
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
                    sample.reference_s = s_coast;
                    samples.push_back(sample);
                }
            }
        }

        const std::vector<bool>* rear_policy = nullptr;
        if (following_lane_half_width > 0.0 && !samples.empty()) {
            const auto& initial = samples.front();
            if (!following_prepared || initial.ego.x != following_origin.ego.x ||
                initial.ego.y != following_origin.ego.y ||
                initial.reference_s != following_origin.reference_s) {
                following_origin = initial;
                // Rear/front traffic roles belong to the reference corridor,
                // not each lateral polynomial's numerical first-step yaw.
                // Resolve once per common origin; the actual body/yaw of EACH
                // candidate still receives the corridor and current checks.
                following_origin.ego.yaw = Interpolate(ref, initial.reference_s).theta;
                following_rear = FollowingRearObjects(following_origin, ref, obstacles,
                    ego_shape, cfg, following_lane_half_width, prediction_dt, horizon);
                following_prepared = true;
            }
            if (std::any_of(following_rear.begin(), following_rear.end(), [](bool value) {return value;}) &&
                FollowsLaneCorridor(samples, ref, ego_shape, following_lane_half_width))
                rear_policy = &following_rear;
        }
        const CartesianCollisionCheckResult collision =
            EvaluateCollisionSamples(samples, obstacles, ego_shape, cfg,
                                     margin_recovery_object_id,
                                     reactive_coast_exempt_object_id, rear_policy,
                                     reuse_object_predictions?&predictions:nullptr);
        path.margin_recovery_used = collision.margin_recovery_used;
        if (!collision.clear) {
            path.valid = false;
            path.rejection_reason = RejectionReason::COLLISION;
            path.collision_object_id = collision.blocking_object_id;
            path.collision_sample_index = collision.blocking_sample_index;
        }
    }
}

