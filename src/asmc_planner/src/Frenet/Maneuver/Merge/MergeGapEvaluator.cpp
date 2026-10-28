#include "Frenet/Maneuver/Merge/MergeGapEvaluator.hpp"
#include "Frenet/Maneuver/Merge/HighwayMerge.hpp"
#include "Frenet/Maneuver/Merge/HighwayPrediction.hpp"
#include "Frenet/Maneuver/Merge/HighwayRearResponse.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// Retain main's observed fast-UNKNOWN fragment protection at admission.
// This is a derived vehicle-size hypothesis; raw observations and committed
// rear-response/continuation geometry remain unchanged.
TrackedObject HighwayAdmissionEnvelope(const TrackedObject& object,const VehicleShape& shape) {
    const double speed=std::hypot(object.vx,object.vy);
    if(!object.geometry_valid || object.type!=0 || object.velocity_unavailable ||
       object.motion_state==ObjectMotionState::LOW_THREAT || !std::isfinite(speed) || speed<7 ||
       (object.width>=shape.width && object.length>=shape.length))return object;
    auto envelope=object;
    const double heading=std::atan2(object.vy,object.vx),c=std::cos(heading),s=std::sin(heading);
    const double hl=.5*std::max(object.length,shape.length),hw=.5*std::max(object.width,shape.width);
    const double xy[4][2]={{-hl,-hw},{hl,-hw},{hl,hw},{-hl,hw}};
    for(int i=0;i<4;++i){
        const double x=object.x+c*xy[i][0]-s*xy[i][1],y=object.y+s*xy[i][0]+c*xy[i][1];
        envelope.corners[i]={x,y,object.z};
        envelope.corners[i+4]={x,y,object.z+std::max(object.height,1.0)};
    }
    envelope.width=2*hw;envelope.length=2*hl;envelope.height=std::max(object.height,1.0);
    envelope.heading=heading;envelope.z_valid=false;
    return envelope;
}

struct Arrival { int id; double time; };

// 현재 속도에서 accel로 desired_speed까지 가속한 뒤 정속 주행할 때의
// 최소 도달시간. MERGE gap 판정과 실제 종방향 후보가 서로 다른 속도를
// 가정하지 않도록 정지 상태에서도 가속 시간을 명시적으로 포함한다.
double MinimumTravelTime(double distance, double initial_speed,
                         double desired_speed, double accel,
                         double min_travel_speed, double min_travel_accel) {
    if (distance <= 0.0) return 0.0;
    const double v0 = std::max(0.0, initial_speed);
    const double target = std::max(desired_speed, min_travel_speed);
    const double a = std::max(accel, min_travel_accel);
    if (v0 >= target) return distance / std::max(v0, min_travel_speed);

    const double accel_time = (target - v0) / a;
    const double accel_distance = v0 * accel_time + 0.5 * a * accel_time * accel_time;
    if (distance <= accel_distance) {
        return (-v0 + std::sqrt(v0 * v0 + 2.0 * a * distance)) / a;
    }
    return accel_time + (distance - accel_distance) / target;
}
}

RoundaboutGap FindRoundaboutGap(const RefLine& ref, const FrenetState& ego,
                                const std::vector<TrackedObject>& obstacles,
                                const MergeConfig& cfg, double desired_speed,
                                double max_ego_accel,
                                const std::unordered_map<int, double>* yaw_rates) {
    RoundaboutGap result;
    if (cfg.conflict_s < 0.0 || ref.points.empty()) return result;

    const RefPoint conflict = Interpolate(ref, cfg.conflict_s);
    std::vector<Arrival> arrivals;
    for (const auto& obj : obstacles) {
        const auto effective_velocity = EffectiveObjectVelocity(obj);
        if (std::hypot(effective_velocity.first, effective_velocity.second) <
            cfg.min_object_speed) {
            continue;
        }
        double yaw_rate = obj.yaw_rate;
        if (yaw_rates) {
            const auto it = yaw_rates->find(obj.id);
            if (it != yaw_rates->end()) yaw_rate = it->second;
        }

        // OBB heading이 아니라 map-frame vx/vy로 시작하고, yaw-rate가 있으면
        // 그 이동 벡터를 회전시키는 공용 예측을 사용한다.
        const double t_end = cfg.max_wait_time + cfg.ego_clear_time;
        double best_t = 0.0;
        double best_miss = std::hypot(conflict.x - obj.x, conflict.y - obj.y);
        TrackedObject predicted_object = obj;
        predicted_object.yaw_rate = yaw_rate;
        for (double t = cfg.prediction_dt; t <= t_end + 1e-9; t += cfg.prediction_dt) {
            const PredictedObjectPose predicted =
                PredictTrackedObjectPose(predicted_object, t);
            const double miss = std::hypot(
                conflict.x - predicted.x, conflict.y - predicted.y);
            if (miss < best_miss) {
                best_miss = miss;
                best_t = t;
            }
        }
        if (best_miss > cfg.conflict_radius) continue;
        arrivals.push_back({obj.id, best_t});
    }
    std::sort(arrivals.begin(), arrivals.end(),
              [](const Arrival& a, const Arrival& b) { return a.time < b.time; });
    result.crossing_vehicle_count = arrivals.size();

    const double distance = std::max(0.0, cfg.conflict_s - ego.s);
    const double earliest = MinimumTravelTime(
        distance, ego.s_d, desired_speed, max_ego_accel,
        cfg.min_travel_speed, cfg.min_travel_accel);
    // rear gap은 고정 ego_clear_time이 아니라, 현재 가속 능력으로 conflict
    // point부터 completion_distance까지 실제로 빠져나가는 시간을 사용한다.
    // 설정값은 측정/모델 오차에 대한 최소 하한으로만 유지한다.
    const double entry_speed = std::min(
        std::max(desired_speed, cfg.min_travel_speed),
        std::sqrt(std::max(0.0, ego.s_d * ego.s_d +
                                   2.0 * std::max(max_ego_accel, cfg.min_travel_accel) *
                                       distance)));
    const double clear_duration = std::max(
        cfg.ego_clear_time,
        MinimumTravelTime(cfg.completion_distance, entry_speed,
                          desired_speed, max_ego_accel,
                          cfg.min_travel_speed, cfg.min_travel_accel));
    result.clear_time = clear_duration;

    // Ego가 너무 멀어 도착 자체가 예측 시간범위 밖이면 현재 관측 차량으로
    // gap을 확정할 단계가 아니다. 이를 unsafe로 반환하면 정지 상태에서
    // earliest>max_wait가 영원히 유지되어 출발조차 못 하는 자기잠금이 된다.
    // 이 구간은 global path로 접근하고, max_wait 범위에 들어온 뒤 매 cycle
    // 실제 순환 차량을 다시 관측해 WAIT/ENTER를 결정한다.
    if (earliest > cfg.max_wait_time) {
        // 안전하다는 뜻이 아니라 아직 Ego 도착시각이 예측창 밖이라는 뜻이다.
        // APPROACH는 계속할 수 있지만 COMMIT 근거로 사용할 수 없다.
        result.confirmed = false;
        result.entry_time = earliest;
        return result;
    }
    result.confirmed = true;

    for (double entry = earliest; entry <= cfg.max_wait_time + 1e-9;
         entry += cfg.entry_search_dt) {
        int preceding_id = -1, following_id = -1;
        double preceding_time = -1.0, following_time = -1.0;
        for (const auto& arrival : arrivals) {
            if (arrival.time <= entry) {
                preceding_id = arrival.id;
                preceding_time = arrival.time;
            } else {
                following_id = arrival.id;
                following_time = arrival.time;
                break;
            }
        }
        const bool front_clear = preceding_id < 0 ||
            entry - preceding_time >= cfg.min_front_time_gap;
        const bool rear_clear = following_id < 0 ||
            following_time - entry >= clear_duration + cfg.min_rear_time_gap;
        if (front_clear && rear_clear) {
            result.safe = true;
            result.entry_time = entry;
            result.preceding_id = preceding_id;
            result.preceding_time = preceding_time;
            result.following_id = following_id;
            result.following_time = following_time;
            return result;
        }
    }
    result.entry_time = cfg.max_wait_time;
    return result;
}

HighwayCandidateCheck EvaluateHighwayMergeCandidate(
    const CartesianPath& path,const HighwayMergeRoute::Zone& zone,const RefLine& route,
    const std::vector<TrackedObject>& observations,const HighwayMergeConfig& cfg,
    const VehicleShape& shape,const CollisionCheckConfig& collision,double dt,bool entering,double object_time_offset_s,
    std::chrono::steady_clock::time_point deadline,bool reference_transition) {
    HighwayCandidateCheck result;
    result.reason="invalid_candidate";
    const std::size_t n=path.x.size();
    if(!cfg.enabled||n<2||path.y.size()!=n||path.yaw.size()!=n||path.kappa.size()!=n||
       path.v.size()!=n||path.a.size()!=n||!std::isfinite(dt)||dt<=0||!std::isfinite(object_time_offset_s)||
       !zone.target_region||zone.target_reference.points.size()<3||
       zone.config.checkpoints.empty()||route.points.size()<3||
       !std::isfinite(cfg.collision_check_dt)||cfg.collision_check_dt<=0||
       !std::isfinite(cfg.collision_max_step_m)||cfg.collision_max_step_m<=0||
       !std::isfinite(cfg.post_merge_horizon_s)||cfg.post_merge_horizon_s<0||
       !std::isfinite(cfg.speed_error_mps)||cfg.speed_error_mps<0||
       !std::isfinite(cfg.speed_error_floor_mps)||cfg.speed_error_floor_mps<0||
       !std::isfinite(cfg.speed_error_correlation_s)||cfg.speed_error_correlation_s<=0||
       cfg.max_collision_samples<2)return result;
    for(std::size_t i=0;i<n;++i)
        if(!std::isfinite(path.x[i])||!std::isfinite(path.y[i])||
           !std::isfinite(path.yaw[i])||!std::isfinite(path.kappa[i])||
           !std::isfinite(path.v[i])||!std::isfinite(path.a[i])||path.v[i]<0)return result;
    if(reference_transition&&(!std::isfinite(cfg.completion_lateral_m)||cfg.completion_lateral_m<=0||
       !std::isfinite(cfg.completion_heading_rad)||cfg.completion_heading_rad<=0||cfg.completion_heading_rad>=M_PI/2))return result;
    std::vector<TrackedObject> objects=observations;
    std::vector<double> radii,speed_errors;
    double maximum_object_speed=0;
    for(auto& obj:objects) {
        if(entering)obj=HighwayAdmissionEnvelope(obj,shape);
        if(!obj.geometry_valid||!geometry::ValidateCorners8(obj.corners).valid||
           !std::isfinite(obj.x)||!std::isfinite(obj.y)||!std::isfinite(obj.heading)||
           !std::isfinite(obj.vx)||!std::isfinite(obj.vy)||!std::isfinite(obj.yaw_rate)) {
            result.blocking_object_id=obj.id;result.reason="invalid_object_geometry_or_motion";return result;
        }
        const auto velocity=EffectiveObjectVelocity(obj);
        const double speed=std::hypot(velocity.first,velocity.second);
        double speed_error=obj.motion_state==ObjectMotionState::LOW_THREAT?0:cfg.speed_error_mps;
        if(std::isfinite(obj.prediction_min_speed)||std::isfinite(obj.prediction_max_speed)) {
            if(!std::isfinite(obj.prediction_min_speed)||!std::isfinite(obj.prediction_max_speed)||
               obj.prediction_min_speed<0||obj.prediction_min_speed>speed||obj.prediction_max_speed<speed) {
                result.reason="invalid_object_speed_interval";result.blocking_object_id=obj.id;return result;
            }
            speed_error=std::max(speed-obj.prediction_min_speed,obj.prediction_max_speed-speed);
            // An observation-derived interval may exceed the initial prior.
            // It is evidence of noise, not an invalid message or a reason to
            // silently clamp uncertainty to a guessed global sensor bound.
        }
        speed_errors.push_back(speed_error);
        double radius=0;
        for(const auto& p:obj.corners)radius=std::max(radius,std::hypot(p.x-obj.x,p.y-obj.y));
        if(!std::isfinite(radius)){result.reason="invalid_object_center";return result;}
        radii.push_back(radius);
        maximum_object_speed=std::max(maximum_object_speed,
            (speed+speed_error)*(1+(std::fabs(obj.prediction_lane_d)+radius)*obj.prediction_lane_curvature)+
            std::fabs(obj.yaw_rate)*radius);
        if(obj.motion_state!=ObjectMotionState::LOW_THREAT&&speed>1e-6) {
            obj.prediction_min_speed=std::max(0.0,speed-speed_error);
            obj.prediction_max_speed=speed+speed_error;
        }
    }
    const double maximum_ego_speed=*std::max_element(path.v.begin(),path.v.end());
    double ego_rotation_bound=0;
    const double ego_radius=std::hypot(shape.length,shape.width*.5);
    for(std::size_t i=1;i<n;++i)ego_rotation_bound=std::max(ego_rotation_bound,
        ego_radius*std::fabs(std::remainder(path.yaw[i]-path.yaw[i-1],2*M_PI))/dt);
    const double bound=std::max(1.0,maximum_ego_speed+maximum_object_speed+ego_rotation_bound);
    const double check_dt=objects.empty()?std::min(dt,cfg.collision_check_dt):
        std::min({dt,cfg.collision_check_dt,cfg.collision_max_step_m/bound});
    double duration=(n-1)*dt;
    std::size_t end_sample=n-1;
    const double requested_samples=std::ceil(duration/check_dt)+1;
    if(!std::isfinite(bound)||!std::isfinite(requested_samples)) {
        result.reason="collision_sample_budget";return result;
    }
    const double rear=shape.rear_overhang<0?shape.length*.5:shape.rear_overhang;
    // Once the ACTUAL ego body is established in the target lane, use the
    // same rear-following responsibility as ordinary lane following. Entry
    // approval remains strict. Current contact, cut-ins, oncoming traffic and
    // ego leaving the corridor never receive this continuation policy.
    const CartesianState initial{path.x[0],path.y[0],path.yaw[0],path.kappa[0],path.v[0],path.a[0]};
    const auto initial_body=MakeEgoFootprint(initial,shape,0);
    const bool lane_established=!entering&&reference_transition&&
        zone.target_region->coversPolygon(initial_body);
    auto interval=[&](const geometry::Polygon2D& polygon,double hint) {
        std::pair<double,double> extent{INFINITY,-INFINITY};
        for(const auto& p:polygon) {
            const double s=FindClosestS(zone.target_reference,p.x,p.y,&hint,15);
            extent.first=std::min(extent.first,s);extent.second=std::max(extent.second,s);
        }
        return extent;
    };
    std::vector<bool> followers(objects.size(),false);
    if(!entering&&reference_transition) {
        const auto rp=Interpolate(zone.target_reference,FindClosestS(zone.target_reference,initial.x,initial.y,nullptr,100));
        const double tx=std::cos(rp.theta),ty=std::sin(rp.theta);
        for(std::size_t j=0;j<objects.size();++j) {
            const auto& obj=objects[j];auto nominal=obj;
            nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
            const auto polygon=PredictObstacleFootprint(nominal,object_time_offset_s);
            // A mapped follower on a bend has a different local tangent from
            // ego. Compare its velocity in ITS lane frame, then classify rear
            // position by lane station, never by ego's global tangent alone.
            const auto actor_pose=PredictTrackedObjectPose(nominal,object_time_offset_s);
            const double object_s=obj.prediction_lane?FindClosestS(zone.target_reference,
                actor_pose.x,actor_pose.y,nullptr,100):0;
            const auto actor_frame=obj.prediction_lane?Interpolate(zone.target_reference,object_s):rp;
            const double ax=std::cos(actor_frame.theta),ay=std::sin(actor_frame.theta);
            const auto actual_velocity=obj.prediction_lane?
                PredictTrackedObjectVelocity(obj,object_time_offset_s):std::make_pair(obj.vx,obj.vy);
            if(InspectPredictionVelocity(obj)!=PredictionInputStatus::MEASURED||
               obj.motion_state==ObjectMotionState::LOW_THREAT||std::fabs(obj.yaw_rate)>1e-3||
               ((std::isfinite(obj.continuation_vx)||std::isfinite(obj.continuation_vy))&&
                (!std::isfinite(obj.continuation_vx)||!std::isfinite(obj.continuation_vy)||
                 obj.continuation_vx*tx+obj.continuation_vy*ty<=0||
                 std::fabs(-obj.continuation_vx*ty+obj.continuation_vy*tx)>cfg.provisional_lateral_speed_mps))||
               actual_velocity.first*ax+actual_velocity.second*ay<=0||
               std::fabs(-actual_velocity.first*ay+actual_velocity.second*ax)>cfg.provisional_lateral_speed_mps||
               !zone.target_region->coversPolygon(polygon)||
               geometry::ConvexPolygonDistance(initial_body,polygon)<=collision.safety_margin)continue;
            const bool behind=obj.prediction_lane?
                interval(polygon,object_s).second <
                    interval(initial_body,FindClosestS(zone.target_reference,initial.x,initial.y,nullptr,100)).first-collision.safety_margin:
                std::all_of(polygon.begin(),polygon.end(),[&](const auto& p){
                    return (p.x-initial.x)*tx+(p.y-initial.y)*ty < -rear-collision.safety_margin;
                });
            if(!behind)continue;
            double own_rear=INFINITY,other_front=-INFINITY;
            for(const auto& p:initial_body)own_rear=std::min(own_rear,p.x*tx+p.y*ty);
            for(const auto& p:polygon)other_front=std::max(other_front,p.x*tx+p.y*ty);
            const double closing=std::max(0.0,actual_velocity.first*ax+actual_velocity.second*ay+speed_errors[j]-
                initial.v*std::cos(initial.yaw-rp.theta));
            const double available=own_rear-other_front-collision.safety_margin-
                closing*highway_rear_response::reaction_s;
            const double response_decel=closing<1e-3?0:
                (available>0?closing*closing/(2*available):INFINITY);
            // During an already committed crossing, a separated target-lane
            // follower may reasonably respond with the SAME bounded braking
            // used by admission. No new approval or increased rear braking
            // assumption is introduced. Before full lane occupancy, all
            // physical crossing samples remain strict below.
            followers[j]=lane_established||response_decel<=cfg.max_rear_decel_mps2;
        }
    }
    // Current/source-lane followers retain ordinary following responsibility
    // while ego remains wholly in that corridor. Target-lane ENTRY admission
    // stays strict; this role ends permanently when this candidate leaves source.
    std::vector<bool> source_followers(objects.size(),false);
    std::vector<double> source_following_start(objects.size(),0.0);
    const auto& source_following_region=zone.source_following_region?
        zone.source_following_region:zone.source_region;
    if(reference_transition&&source_following_region&&zone.source_reference.points.size()>=3&&
       source_following_region->coversPolygon(initial_body)) {
        const double own_s=FindClosestS(zone.source_reference,initial.x,initial.y,nullptr,100);
        auto extent=[&](const geometry::Polygon2D& polygon,double hint) {
            std::pair<double,double> value{INFINITY,-INFINITY};
            for(const auto& p:polygon) {
                const double s=FindClosestS(zone.source_reference,p.x,p.y,&hint,15);
                value.first=std::min(value.first,s);value.second=std::max(value.second,s);
            }
            return value;
        };
        const auto own_extent=extent(initial_body,own_s);
        for(std::size_t j=0;j<objects.size();++j) {
            const auto& obj=objects[j];
            if(obj.type!=static_cast<int>(asmc_planner::ObjectType::VEHICLE)&&
               obj.type!=static_cast<int>(asmc_planner::ObjectType::UNKNOWN))continue;
            const bool provisional=std::isfinite(obj.continuation_vx)&&std::isfinite(obj.continuation_vy)&&
                std::isfinite(obj.turn_prediction_duration)&&obj.turn_prediction_duration>=0&&
                obj.turn_prediction_duration<=cfg.speed_error_correlation_s;
            if(InspectPredictionVelocity(obj)!=PredictionInputStatus::MEASURED||
               obj.motion_state==ObjectMotionState::LOW_THREAT||
               (!provisional&&std::fabs(obj.yaw_rate)>1e-3))continue;
            auto nominal=obj;nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
            const auto pose=PredictTrackedObjectPose(nominal,object_time_offset_s);
            const double at_s=FindClosestS(zone.source_reference,pose.x,pose.y,nullptr,100);
            const auto frame=Interpolate(zone.source_reference,at_s);
            const double response_time=provisional?
                std::max(object_time_offset_s,obj.turn_prediction_duration+1e-6):object_time_offset_s;
            const auto velocity=PredictTrackedObjectVelocity(nominal,response_time);
            const double tx=std::cos(frame.theta),ty=std::sin(frame.theta);
            if(velocity.first*tx+velocity.second*ty<=0||
               std::fabs(-velocity.first*ty+velocity.second*tx)>cfg.provisional_lateral_speed_mps)continue;
            const auto polygon=PredictObstacleFootprint(nominal,object_time_offset_s);
            source_followers[j]=source_following_region->coversPolygon(polygon)&&
                extent(polygon,at_s).second<own_extent.first-collision.safety_margin&&
                geometry::ConvexPolygonDistance(initial_body,polygon)>collision.safety_margin;
            source_following_start[j]=std::max(0.0,response_time-object_time_offset_s);
        }
    }
    const bool have_source_followers=std::any_of(source_followers.begin(),source_followers.end(),
        [](bool value){return value;});
    bool source_follower_departed=false;
    std::size_t source_certified_segments=0;
    const auto source_segment_inside=[&](std::size_t i) {
        // Certify each ORIGINAL trajectory segment once, not again at every
        // dense object-collision sample. Position and yaw are linearly
        // interpolated below. A rotating corner differs from its endpoint
        // chord by at most radius * delta_yaw^2 / 8; expanded endpoint hulls
        // therefore cover the whole intervening footprint. Failure only
        // revokes the following role; physical collision checks still run.
        const double angle=std::remainder(path.yaw[i+1]-path.yaw[i],2*M_PI);
        const double guard=ego_radius*angle*angle/8+1e-9;
        CartesianState a{path.x[i],path.y[i],path.yaw[i],0,0,0};
        CartesianState b{path.x[i+1],path.y[i+1],path.yaw[i+1],0,0,0};
        auto points=MakeEgoFootprint(a,shape,guard);
        const auto end=MakeEgoFootprint(b,shape,guard);
        points.insert(points.end(),end.begin(),end.end());
        return source_following_region->coversPolygon(geometry::ComputeConvexHull(points));
    };
    // A committed join may straddle source and target while both ego and its
    // already mapped rear traffic are still upstream of target_region. That
    // region is a HANDOVER condition, not the beginning of following traffic.
    // Match the exact prepared prediction lane; never grant this role to an
    // unresolved/crossing actor, a pedestrian, or a newly requested entry.
    std::vector<const HighwayMergeRoute::Zone::FollowingLane*> joining_followers(objects.size(),nullptr);
    std::vector<highway_rear_response::State> joining_responses(objects.size());
    std::vector<TrackedObject> joining_response_origins(objects.size());
    std::vector<double> joining_origin_offsets(objects.size(),0);
    std::vector<std::size_t> joining_response_segments(objects.size(),n);
    std::vector<geometry::Polygon2D> joining_response_sweeps(objects.size());
    std::vector<std::size_t> joining_ordinary_segments(objects.size(),n);
    std::vector<geometry::Polygon2D> joining_ordinary_sweeps(objects.size());
    bool joining_departed=false;
    std::vector<int> joining_scope_segments(n-1,-1);
    std::size_t joining_certified_segments=0;
    const asmc_planner::lane_reference::FootprintShape joining_body{shape.length-rear,rear,shape.width*.5};
    const auto following_lane_contains=[&](const HighwayMergeRoute::Zone::FollowingLane& lane,double x,double y) {
        if(!lane.reference||!lane.region)return false;
        if(lane.region->coversPoint(x,y))return true;
        if(!zone.following_road_footprint||lane.reference->points.size()<3)return false;
        const double station=FindClosestS(*lane.reference,x,y,nullptr,100);
        if(station<=0||station>=lane.reference->length)return false;
        const auto pose=Interpolate(*lane.reference,station);
        // Prepared reference geometry bridges known vector seam gaps. This
        // proves membership only; the physical swept body still needs the
        // runtime road certificate below. Reuse the existing alignment bound.
        return std::hypot(x-pose.x,y-pose.y)<=cfg.completion_lateral_m;
    };
    const auto following_scope_contains=[&](double x,double y) {
        if(zone.following_region&&zone.following_region->coversPoint(x,y))return true;
        return std::any_of(zone.following_lanes.begin(),zone.following_lanes.end(),
            [&](const auto& lane){return following_lane_contains(lane,x,y);});
    };
    const auto joining_footprint_clear=[&](double x,double y,double yaw,
        const asmc_planner::lane_reference::FootprintShape& body) {
        if(!zone.following_region)return false;
        const auto& base=zone.following_road_body;
        if(!zone.following_road_footprint)
            return zone.following_region->coversFootprint(x,y,yaw,body,0);
        if(!asmc_planner::lane_reference::ValidFootprint(base)||!following_scope_contains(x,y))return false;
        // The four translated physical rectangles COVER the whole enlarged
        // rectangle when their opposing spans overlap. No map buffer, gap
        // filling or point-only road test is used. This is the SAME raster
        // contract already used by prepared references and final road checks.
        const double left=std::min(0.0,base.rear_m-body.rear_m);
        const double right=std::max(0.0,body.front_m-base.front_m);
        const double side=std::max(0.0,body.half_width_m-base.half_width_m);
        if(right-left>base.front_m+base.rear_m||side>base.half_width_m)return false;
        const double c=std::cos(yaw),s=std::sin(yaw);
        for(double along:{left,right})for(double lateral:{-side,side})
            if(!zone.following_road_footprint(x+c*along-s*lateral,y+s*along+c*lateral,yaw))return false;
        return true;
    };
    if(!entering&&reference_transition&&object_time_offset_s>=0&&zone.following_region&&
       std::isfinite(cfg.max_rear_decel_mps2)&&cfg.max_rear_decel_mps2>0&&
       std::isfinite(cfg.min_heading_alignment)&&cfg.min_heading_alignment>=0&&cfg.min_heading_alignment<=1&&
       joining_footprint_clear(initial.x,initial.y,initial.yaw,joining_body)) {
        for(std::size_t j=0;j<objects.size();++j) {
            const auto& obj=objects[j];
            const auto& original=obj.prediction_origin?*obj.prediction_origin:obj;
            if(original.prediction_origin||original.prediction_lane.get()!=obj.prediction_lane.get()||
               (obj.prediction_origin&&obj.prediction_origin_offset_s<0))continue;
            if(!obj.prediction_lane||InspectPredictionVelocity(obj)!=PredictionInputStatus::MEASURED||
               (obj.type!=static_cast<int>(asmc_planner::ObjectType::VEHICLE)&&
                obj.type!=static_cast<int>(asmc_planner::ObjectType::UNKNOWN))||
               obj.motion_state==ObjectMotionState::LOW_THREAT||std::fabs(obj.yaw_rate)>1e-3)continue;
            const auto found=std::find_if(zone.following_lanes.begin(),zone.following_lanes.end(),
                [&](const auto& lane){return lane.reference.get()==obj.prediction_lane.get()&&lane.region&&
                    lane.reference->points.size()>=3&&std::isfinite(lane.reference->length)&&lane.reference->length>0;});
            if(found==zone.following_lanes.end()||!found->region->intersectsPolygon(initial_body))continue;
            auto nominal=obj;nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
            const auto pose=PredictTrackedObjectPose(nominal,object_time_offset_s);
            const auto polygon=PredictObstacleFootprint(nominal,object_time_offset_s);
            if(!following_lane_contains(*found,pose.x,pose.y)||
               geometry::ConvexPolygonDistance(initial_body,polygon)<=collision.safety_margin)continue;
            const auto& lane=*found->reference;
            const double station=FindClosestS(lane,pose.x,pose.y,nullptr,100);
            const auto frame=Interpolate(lane,station);
            const auto velocity=PredictTrackedObjectVelocity(nominal,object_time_offset_s);
            const double tx=std::cos(frame.theta),ty=std::sin(frame.theta);
            const double along=velocity.first*tx+velocity.second*ty;
            if(along<=0||std::fabs(-velocity.first*ty+velocity.second*tx)>cfg.provisional_lateral_speed_mps||
               std::fabs(-obj.vx*ty+obj.vy*tx)>cfg.provisional_lateral_speed_mps||
               std::cos(initial.yaw-frame.theta)<cfg.min_heading_alignment)continue;
            auto extent=[&](const geometry::Polygon2D& body,double hint) {
                std::pair<double,double> value{INFINITY,-INFINITY};
                for(const auto& p:body) {
                    const double s=FindClosestS(lane,p.x,p.y,&hint,15);
                    value.first=std::min(value.first,s);value.second=std::max(value.second,s);
                }
                return value;
            };
            const auto own=extent(initial_body,FindClosestS(lane,initial.x,initial.y,nullptr,100));
            const double gap=own.first-extent(polygon,station).second;
            const double closing=std::max(0.0,along+speed_errors[j]-
                initial.v*std::cos(initial.yaw-frame.theta));
            const double available=gap-collision.safety_margin-closing*highway_rear_response::reaction_s;
            const double braking=closing<1e-3?0:(available>0?closing*closing/(2*available):INFINITY);
            if(gap<=collision.safety_margin||braking>cfg.max_rear_decel_mps2)continue;
            joining_followers[j]=&*found;
            // Alignment preserves the original observation for ordinary
            // prediction. A candidate-specific response must instead modify
            // that origin explicitly, or its velocity overrides are ignored.
            // Keep the whole pre-alignment time and its speed uncertainty;
            // never pretend the follower braked before the current reaction.
            joining_response_origins[j]=obj.prediction_origin?*obj.prediction_origin:obj;
            joining_response_origins[j].prediction_origin.reset();
            joining_origin_offsets[j]=obj.prediction_origin?obj.prediction_origin_offset_s:0;
            auto& response=joining_responses[j];response.enabled=true;
            response.start=joining_origin_offsets[j]+std::max(0.0,object_time_offset_s)+highway_rear_response::reaction_s;
            const auto& origin=joining_response_origins[j];
            const double speed=std::hypot(origin.vx,origin.vy);
            response.low=std::max(0.0,speed-speed_errors[j]);response.high=speed+speed_errors[j];
            response.floor=0;response.required_decel=response.maximum_decel=cfg.max_rear_decel_mps2;
        }
    }
    const auto joining_segment_inside=[&](std::size_t i) {
        const double dx=path.x[i+1]-path.x[i],dy=path.y[i+1]-path.y[i];
        if(dx*std::cos(path.yaw[i])+dy*std::sin(path.yaw[i]) < -1e-6)return false;
        const double angle=std::remainder(path.yaw[i+1]-path.yaw[i],2*M_PI);
        const double yaw=path.yaw[i]+angle*.5,c=std::cos(yaw),s=std::sin(yaw);
        const double turn=std::fabs(angle)*.5;
        const double radius=std::max(joining_body.front_m,joining_body.rear_m);
        // An enclosing rectangle covers EVERY interpolated pose, including
        // corner rotation. coversFootprint uses the existing exact local union
        // rather than repeated polygon differences across a lane seam.
        const double longitudinal=.5*std::fabs(dx*c+dy*s)+
            radius*(1-std::cos(turn))+joining_body.half_width_m*std::sin(turn)+1e-9;
        const double lateral=.5*std::fabs(-dx*s+dy*c)+
            radius*std::sin(turn)+joining_body.half_width_m*(1-std::cos(turn))+1e-9;
        const asmc_planner::lane_reference::FootprintShape swept{
            joining_body.front_m+longitudinal,joining_body.rear_m+longitudinal,
            joining_body.half_width_m+lateral};
        return joining_footprint_clear(.5*(path.x[i]+path.x[i+1]),
            .5*(path.y[i]+path.y[i+1]),yaw,swept);
    };
    const bool have_joining_followers=std::any_of(joining_followers.begin(),joining_followers.end(),
        [](const auto* lane){return lane!=nullptr;});
    double station_hint=zone.config.start_s,target_hint=0,last_gap_time=-1;
    bool follower_entered=lane_established,follower_departed=false;
    bool have_station=false,have_target=false;
    std::vector<highway_rear_response::State> responses(objects.size());
    std::vector<std::size_t> rear_candidates;
    if(entering&&reference_transition&&std::isfinite(cfg.max_rear_decel_mps2)&&cfg.max_rear_decel_mps2>0) {
        const double ego_hint=FindClosestS(zone.target_reference,initial.x,initial.y,nullptr,100);
        const auto initial_extent=interval(initial_body,ego_hint);
        for(std::size_t j=0;j<objects.size();++j) {
            const auto& obj=objects[j];
            if(!obj.prediction_lane||obj.motion_state==ObjectMotionState::LOW_THREAT||
               obj.motion_state==ObjectMotionState::UNKNOWN||std::hypot(obj.vx,obj.vy)<1)continue;
            const auto prediction=PredictTrackedObjectPose(obj,object_time_offset_s);
            const double station=FindClosestS(zone.target_reference,prediction.x,prediction.y,nullptr,100);
            const auto rp=Interpolate(zone.target_reference,station);
            const auto velocity=PredictTrackedObjectVelocity(obj,object_time_offset_s);
            const double along=velocity.first*std::cos(rp.theta)+velocity.second*std::sin(rp.theta);
            const double lateral=-velocity.first*std::sin(rp.theta)+velocity.second*std::cos(rp.theta);
            if(along<=0||std::fabs(lateral)>cfg.provisional_lateral_speed_mps||
               std::hypot(prediction.x-rp.x,prediction.y-rp.y)>.35)continue;
            auto nominal=obj;nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
            const auto current=PredictObstacleFootprint(nominal,object_time_offset_s);
            if(!zone.target_region->coversPolygon(current)||
               geometry::ConvexPolygonDistance(initial_body,current)<=collision.safety_margin||
               interval(PredictHighwayObstacleFootprint(obj,object_time_offset_s,speed_errors[j],cfg),station).second>=
                   initial_extent.first-collision.safety_margin)continue;
            rear_candidates.push_back(j);
        }
    }
    // Do not walk the candidate a second time when no confirmed target-lane
    // follower can qualify. Most ordinary front/adjacent cases take this path.
    if(!rear_candidates.empty()) {
        // Coarse entry is at or AFTER the first physical overlap; never grant
        // an earlier reaction than the dense checker. Use only the candidate's
        // completion + retained horizon to derive a conservative ego floor.
        const double tail=std::ceil(cfg.post_merge_horizon_s/dt);
        const std::size_t tail_samples=tail>=n?n:static_cast<std::size_t>(tail);
        const double reaction=std::ceil(highway_rear_response::reaction_s/dt);
        const std::size_t reaction_samples=reaction>=n?n:static_cast<std::size_t>(reaction);
        std::size_t entry=n,clear=n;
        std::vector<double> rear_stations(n);
        double hint=0;
        for(std::size_t i=0;i<n;++i) {
            if((i%16)==0&&std::chrono::steady_clock::now()>=deadline) {
                result.reason="planning_time_budget";return result;
            }
            const CartesianState pose{path.x[i],path.y[i],path.yaw[i],path.kappa[i],path.v[i],path.a[i]};
            const auto body=MakeEgoFootprint(pose,shape,0);
            hint=FindClosestS(zone.target_reference,pose.x,pose.y,i?&hint:nullptr,100);
            rear_stations[i]=interval(body,hint).first;
            if(entry==n&&zone.target_region->intersectsPolygon(body))entry=i;
            const auto rp=Interpolate(zone.target_reference,hint);
            if(entry<n&&clear==n&&hint>=rear&&hint<=zone.target_reference.length-(shape.length-rear)&&
               std::hypot(pose.x-rp.x,pose.y-rp.y)<=cfg.completion_lateral_m&&
               std::fabs(std::remainder(pose.yaw-rp.theta,2*M_PI))<=cfg.completion_heading_rad&&
               zone.target_region->coversPolygon(body))clear=i;
            if(clear<n&&i>=clear+tail_samples+2)break;
        }
        const std::size_t response=entry<n&&reaction_samples<n-entry?entry+reaction_samples:n;
        const std::size_t end=clear<n?std::min(n-1,clear+tail_samples+2):n-1;
        if(response<end&&clear<n) {
            double floor=INFINITY;
            for(std::size_t i=response;i<end;++i)
                floor=std::min(floor,(rear_stations[i+1]-rear_stations[i])/dt);
            const CartesianState pose{path.x[response],path.y[response],path.yaw[response],path.kappa[response],path.v[response],path.a[response]};
            const auto own=interval(MakeEgoFootprint(pose,shape,0),rear_stations[response]);
            const double start=response*dt+object_time_offset_s;
            for(const auto j:rear_candidates) {
                if(std::chrono::steady_clock::now()>=deadline) {
                    result.reason="planning_time_budget";return result;
                }
                const auto& obj=objects[j];
                if(start<=0||!std::isfinite(floor)||floor<=0)continue;
                const double speed=std::hypot(obj.vx,obj.vy);
                const double low=std::max(0.0,speed-speed_errors[j]),high=speed+speed_errors[j];
                const double lane_end=std::min(obj.prediction_lane->length,obj.prediction_lane_end_s);
                // Never invent a response on an unresolved successor or beyond
                // a prepared lane. Unknown/crossing motion retains strict CV.
                if(obj.prediction_lane_s+high*(end*dt+object_time_offset_s)>lane_end)continue;
                const auto at_response=PredictHighwayObstacleFootprint(obj,start,speed_errors[j],cfg);
                if(!zone.target_region->coversPolygon(at_response))continue;
                const double gap=own.first-interval(at_response,own.first).second;
                const double closing=std::max(0.0,high-floor),available=gap-cfg.min_rear_gap;
                const double required=closing<1e-3?0:(available>0?closing*closing/(2*available):INFINITY);
                if(!std::isfinite(required)||required>cfg.max_rear_decel_mps2||gap<cfg.min_rear_gap)continue;
                highway_rear_response::State response_model;
                response_model.enabled=true;response_model.start=start;
                response_model.low=low;response_model.high=high;response_model.floor=floor;
                response_model.required_decel=required;response_model.maximum_decel=cfg.max_rear_decel_mps2;
                // Confirm the WHOLE bounded response remains target-lane
                // traffic. Failure restores the original strict hypothesis.
                bool contained=true;
                for(std::size_t i=response;i<=end;++i) {
                    if((i%16)==0&&std::chrono::steady_clock::now()>=deadline) {
                        result.reason="planning_time_budget";return result;
                    }
                    const auto polygon=response_model.footprint(obj,i*dt+object_time_offset_s);
                    if(polygon.size()<3||!zone.target_region->coversPolygon(polygon)){contained=false;break;}
                }
                if(contained)responses[j]=response_model;
            }
        }
    }
    for(std::size_t k=0;;++k) {
        if(k>=cfg.max_collision_samples) {
            result.reason="collision_sample_budget";return result;
        }
        if((k%16)==0&&std::chrono::steady_clock::now()>=deadline) {
            result.reason="planning_time_budget";return result;
        }
        const double t=std::min(duration,k*check_dt),index=t/dt;
        const std::size_t lo=std::min(n-2,static_cast<std::size_t>(index));
        const double w=std::clamp(index-lo,0.0,1.0);
        auto lerp=[&](const std::vector<double>& a){return a[lo]+w*(a[lo+1]-a[lo]);};
        CartesianState ego{lerp(path.x),lerp(path.y),
            path.yaw[lo]+w*std::remainder(path.yaw[lo+1]-path.yaw[lo],2*M_PI),
            lerp(path.kappa),lerp(path.v),lerp(path.a)};
        const auto ego_polygon=objects.empty()&&!reference_transition?geometry::Polygon2D{}:MakeEgoFootprint(ego,shape,0);
        if(have_joining_followers&&!joining_departed) {
            auto& contained=joining_scope_segments[lo];
            if(contained<0) {
                // The rectangle encloses EVERY interpolated center in this
                // segment. A failed certificate falls back to the original
                // dense membership queries, including vector seams. It is
                // never used as a substitute for physical body containment.
                const double left=std::min(path.x[lo],path.x[lo+1])-1e-7;
                const double right=std::max(path.x[lo],path.x[lo+1])+1e-7;
                const double bottom=std::min(path.y[lo],path.y[lo+1])-1e-7;
                const double top=std::max(path.y[lo],path.y[lo+1])+1e-7;
                contained=zone.following_region->coversPolygon({{left,bottom},{right,bottom},
                    {right,top},{left,top}})?1:0;
            }
            if(!contained&&!following_scope_contains(ego.x,ego.y))joining_departed=true;
        }
        // The distance guard covers motion between neighbouring sample times;
        // the finer of time and relative-travel bounds is used for all objects.
        const double between_samples_guard=bound*check_dt*.5;
        bool follower_corridor=false;
        if(std::any_of(followers.begin(),followers.end(),[](bool value){return value;})) {
            const bool inside=zone.target_region->coversPolygon(ego_polygon);
            if(follower_entered&&!inside)follower_departed=true;
            follower_entered|=inside;
            follower_corridor=inside&&!follower_departed;
        }
        // Visit each segment only when the physical collision scan reaches it;
        // an earlier completed certificate need not inspect the unused tail.
        while(have_source_followers&&!source_follower_departed&&source_certified_segments<=lo) {
            source_follower_departed=!source_segment_inside(source_certified_segments++);
        }
        const bool source_follower_inside=have_source_followers&&!source_follower_departed;
        for(const auto& obj:objects) {
            const auto predicted=PredictTrackedObjectPose(obj,t+object_time_offset_s);
            const double reach=ego_radius+radii[&obj-objects.data()]+collision.safety_margin+
                collision.margin_growth_rate*std::fabs(t+object_time_offset_s)+
                HighwayPositionError(speed_errors[&obj-objects.data()],t+object_time_offset_s,cfg)+between_samples_guard;
            const auto j=static_cast<std::size_t>(&obj-objects.data());
            const auto& response=responses[&obj-objects.data()];
            const bool reacting=response.enabled&&t+object_time_offset_s>response.start;
            if(!reacting&&std::hypot(predicted.x-ego.x,predicted.y-ego.y)>reach)continue;
            const auto& original=obj.prediction_origin?*obj.prediction_origin:obj;
            const double original_speed=std::hypot(original.vx,original.vy);
            const bool valid_interval=(!std::isfinite(original.prediction_min_speed)&&
                !std::isfinite(original.prediction_max_speed))||
                (std::isfinite(original.prediction_min_speed)&&std::isfinite(original.prediction_max_speed)&&
                 original.prediction_min_speed>=0&&original.prediction_min_speed<=original_speed&&
                 original.prediction_max_speed>=original_speed);
            if(reference_transition&&object_time_offset_s>=0&&obj.prediction_lane&&!reacting&&
               obj.motion_state!=ObjectMotionState::LOW_THREAT&&!original.prediction_origin&&
               original.prediction_lane.get()==obj.prediction_lane.get()&&valid_interval&&
               (!obj.prediction_origin||obj.prediction_origin_offset_s>=0)) {
                // A segment-wide lane sweep is a conservative broad phase,
                // not fewer collision samples. Near occupancy still uses the
                // exact original prediction at this dense sample below.
                if(joining_ordinary_segments[j]!=lo) {
                    auto bounded=original;
                    const double speed=original_speed;
                    double low=std::max(0.0,speed-speed_errors[j]),high=speed+speed_errors[j];
                    if(obj.prediction_origin) {
                        low=std::isfinite(original.prediction_min_speed)?original.prediction_min_speed:speed;
                        high=std::isfinite(original.prediction_max_speed)?original.prediction_max_speed:speed;
                    }
                    const double age=obj.prediction_origin?obj.prediction_origin_offset_s:0;
                    low*=std::max(0.0,lo*dt+object_time_offset_s+age);
                    high*=std::max(0.0,(lo+1)*dt+object_time_offset_s+age);
                    bounded.vx=(low+high)*.5;bounded.vy=0;
                    bounded.prediction_min_speed=low;bounded.prediction_max_speed=high;
                    joining_ordinary_sweeps[j]=PredictObstacleFootprint(bounded,1.0);
                    joining_ordinary_segments[j]=lo;
                }
                const auto& broad=joining_ordinary_sweeps[j];
                const double margin=collision.safety_margin+
                    collision.margin_growth_rate*std::fabs(t+object_time_offset_s)+between_samples_guard;
                if(broad.size()>=3&&geometry::ConvexPolygonDistance(ego_polygon,broad)>margin)continue;
            }
            const auto footprint=reacting?response.footprint(obj,t+object_time_offset_s):
                PredictHighwayObstacleFootprint(obj,t+object_time_offset_s,speed_errors[j],cfg);
            if(footprint.size()<3) {
                result.reason="invalid_predicted_footprint";result.blocking_object_id=obj.id;return result;
            }
            const double uncertainty=obj.motion_state==ObjectMotionState::LOW_THREAT?0:
                collision.margin_growth_rate*std::fabs(t+object_time_offset_s);
            if(geometry::ConvexPolygonDistance(ego_polygon,footprint)<=
                collision.safety_margin+uncertainty+between_samples_guard) {
                const auto j=static_cast<std::size_t>(&obj-objects.data());
                if(t>source_following_start[j]&&source_followers[j]&&source_follower_inside) {
                    auto nominal=obj;nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
                    if(source_following_region->coversPolygon(PredictObstacleFootprint(nominal,t+object_time_offset_s)))
                        continue;
                }
                if(t>0&&followers[j]&&follower_corridor) {
                    auto nominal=obj;nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
                    if(zone.target_region->coversPolygon(PredictObstacleFootprint(nominal,t+object_time_offset_s)))
                        continue;
                }
                if(joining_followers[j]&&t>highway_rear_response::reaction_s&&!joining_departed) {
                    // Certify ORIGINAL segments lazily, at most once each.
                    // Current contact and the entire reaction interval above
                    // remain strict; departure permanently revokes this role.
                    while(!joining_departed&&joining_certified_segments<=lo) {
                        if(std::chrono::steady_clock::now()>=deadline) {
                            result.reason="planning_time_budget";return result;
                        }
                        joining_departed=!joining_segment_inside(joining_certified_segments++);
                    }
                    auto nominal=obj;nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
                    const auto pose=PredictTrackedObjectPose(nominal,t+object_time_offset_s);
                    const double station=obj.prediction_lane_s+std::hypot(obj.vx,obj.vy)*
                        std::max(0.0,t+object_time_offset_s);
                    if(!joining_departed&&station+radii[j]<=std::min(obj.prediction_lane->length,obj.prediction_lane_end_s)&&
                       following_lane_contains(*joining_followers[j],pose.x,pose.y)) {
                        // Following responsibility is not a promise that ANY
                        // ego schedule can be saved. The full original speed
                        // interval must still have a collision-free bounded
                        // braking response to THIS path, after the reaction
                        // allowance. Excessive ego braking remains rejected.
                        if(joining_response_segments[j]!=lo) {
                            // Cache a CONSERVATIVE occupancy for this whole
                            // original segment. Braking travel is monotone;
                            // the earliest low station and latest high station
                            // contain every dense sample, including lane knots.
                            // Dense ego/obstacle checks and margins stay intact.
                            const double begin=std::max(0.0,lo*dt+object_time_offset_s+joining_origin_offsets[j]);
                            const double end=std::max(begin,(lo+1)*dt+object_time_offset_s+joining_origin_offsets[j]);
                            const auto low=joining_responses[j].distances(begin).first;
                            const auto high=joining_responses[j].distances(end).second;
                            auto bounded=joining_response_origins[j];bounded.vx=(low+high)*.5;bounded.vy=0;
                            bounded.prediction_min_speed=low;bounded.prediction_max_speed=high;
                            joining_response_sweeps[j]=PredictObstacleFootprint(bounded,1.0);
                            joining_response_segments[j]=lo;
                        }
                        const auto& broad=joining_response_sweeps[j];
                        if(broad.size()>=3) {
                            const double margin=collision.safety_margin+uncertainty+between_samples_guard;
                            if(geometry::ConvexPolygonDistance(ego_polygon,broad)>margin)continue;
                            // The cached sweep can only accept separation. Refine
                            // overlapping broad phases at the actual dense time;
                            // temporal enclosure must not create a false collision.
                            const auto exact=joining_responses[j].footprint(joining_response_origins[j],
                                t+object_time_offset_s+joining_origin_offsets[j]);
                            if(exact.size()>=3&&geometry::ConvexPolygonDistance(ego_polygon,exact)>margin)continue;
                        }
                    }
                }
                result.reason="candidate_collision";result.blocking_object_id=obj.id;return result;
            }
        }
        ++result.evaluated_samples;
        // Station queries and gap metrics need not run at the dense collision
        // rate. All actual motion, including the final sample, is still checked.
        if(last_gap_time>=0&&t<duration&&t-last_gap_time<cfg.collision_check_dt)continue;
        last_gap_time=t;
        station_hint=FindClosestS(route,ego.x,ego.y,have_station?&station_hint:nullptr,50);
        have_station=true;
        const bool entered=reference_transition?zone.target_region->intersectsPolygon(ego_polygon):station_hint>=zone.entry_s;
        bool cleared=station_hint-rear>=zone.config.completion_s;
        if(reference_transition) {
            const double lane_s=FindClosestS(zone.target_reference,ego.x,ego.y,nullptr,100);
            const auto p=Interpolate(zone.target_reference,lane_s);
            const asmc_planner::lane_reference::FootprintShape body{
                shape.length-rear,rear,shape.width*.5};
            cleared=lane_s>=rear&&lane_s<=zone.target_reference.length-body.front_m&&
                std::hypot(ego.x-p.x,ego.y-p.y)<=cfg.completion_lateral_m&&
                std::fabs(std::remainder(ego.yaw-p.theta,2*M_PI))<=cfg.completion_heading_rad&&
                zone.target_region->coversFootprint(ego.x,ego.y,ego.yaw,body,0);
        }
        if(result.entry_time_s<0&&entered)
            result.entry_time_s=t;
        if(result.clear_time_s<0&&cleared) {
            result.clear_time_s=t;
            // One owner defines both completion and the retained prefix. The
            // dense checker may observe clearance between trajectory samples;
            // retain the first sample at/after clearance + the full horizon.
            const double required=t+cfg.post_merge_horizon_s;
            if(required<=duration) {
                end_sample=std::min(n-1,static_cast<std::size_t>(std::ceil(required/dt)));
                duration=end_sample*dt;
            }
        }
        if(result.entry_time_s<0||objects.empty()) {
            if(t>=duration)break;
            continue;
        }
        RefPoint rp{};std::pair<double,double> own{};
        double ego_speed=0;bool own_projected=false;
        for(const auto& obj:objects) {
            const auto predicted=PredictTrackedObjectPose(obj,t+object_time_offset_s);
            const auto& response=responses[&obj-objects.data()];
            const bool reacting=response.enabled&&t+object_time_offset_s>response.start;
            if(!reacting&&std::hypot(predicted.x-ego.x,predicted.y-ego.y)>
                cfg.object_search_distance+HighwayPositionError(speed_errors[&obj-objects.data()],t+object_time_offset_s,cfg))continue;
            // Speed uncertainty is longitudinal occupancy, not evidence that a
            // vehicle in another lane is entering our target lane. Classify
            // target traffic with the nominal physical footprint. The dense
            // collision pass above still checks every object's full interval,
            // including off-corridor cut-ins and crossing actors.
            auto nominal=obj;nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
            if(!reacting&&!zone.target_region->intersectsPolygon(
                PredictObstacleFootprint(nominal,t+object_time_offset_s)))continue;
            const auto footprint=reacting?response.footprint(obj,t+object_time_offset_s):
                PredictHighwayObstacleFootprint(obj,t+object_time_offset_s,speed_errors[&obj-objects.data()],cfg);
            // Project the ego footprint only when a relevant target-corridor
            // actor actually exists. The dense physical collision pass above
            // still checks every object, including crossing/off-corridor ones.
            if(!own_projected) {
                target_hint=FindClosestS(zone.target_reference,ego.x,ego.y,have_target?&target_hint:nullptr,100);
                have_target=true;rp=Interpolate(zone.target_reference,target_hint);
                own=interval(ego_polygon,target_hint);
                ego_speed=std::max(0.0,ego.v*std::cos(ego.yaw-rp.theta));
                own_projected=true;
            }
            const double prediction_time=t+object_time_offset_s;
            const double turn_end=std::max(0.0,obj.turn_prediction_duration);
            const bool continuation=prediction_time>turn_end&&
                obj.motion_state!=ObjectMotionState::LOW_THREAT&&
                std::isfinite(obj.continuation_vx)&&std::isfinite(obj.continuation_vy);
            const auto v=continuation?std::make_pair(obj.continuation_vx,obj.continuation_vy):
                PredictTrackedObjectVelocity(obj,std::min(prediction_time,turn_end));
            const double speed=std::hypot(v.first,v.second);
            const double along=v.first*std::cos(rp.theta)+v.second*std::sin(rp.theta);
            // Crossing/reverse traffic remains in the full collision check.
            if(speed>.1&&along/speed<cfg.min_heading_alignment)continue;
            const auto other=interval(footprint,target_hint);
            const double uncertainty=HighwayVelocityError(speed_errors[&obj-objects.data()],prediction_time,cfg);
            double minimum_speed=std::max(0.0,along-uncertainty),maximum_speed=along+uncertainty;
            if(reacting) {
                const auto limits=response.speeds(prediction_time);
                // Bound ALL lane positions in the interval, including a bend
                // between its endpoints. OBB heading is not travel direction.
                minimum_speed=0;
                maximum_speed=limits.second*(1+std::fabs(obj.prediction_lane_d)*obj.prediction_lane_curvature);
            }
            if(other.first>=own.second) {
                const double gap=other.first-own.second;
                const double closing=std::max(0.0,ego_speed-minimum_speed);
                const double ttc=closing>1e-3?gap/closing:INFINITY;
                if(gap<result.minimum_front_gap_m){result.minimum_front_gap_m=gap;result.front_id=obj.id;}
                result.minimum_front_ttc_s=std::min(result.minimum_front_ttc_s,ttc);
                if(entering&&(gap<cfg.min_front_gap||ttc<cfg.min_front_ttc)) {
                    result.reason="front_gap_or_ttc";result.blocking_object_id=obj.id;return result;
                }
            } else if(other.second<=own.first) {
                const double gap=own.first-other.second;
                const double closing=std::max(0.0,maximum_speed-ego_speed);
                const double ttc=closing>1e-3?gap/closing:INFINITY;
                // Do not restart a second one-second reaction at every future
                // sample after already predicting that same bounded response.
                const double remaining_response=response.enabled?
                    std::max(0.0,response.start-prediction_time):highway_rear_response::reaction_s;
                const double available=gap-cfg.min_rear_gap-closing*remaining_response;
                const double residual=closing<1e-3?0:(available>0?closing*closing/(2*available):INFINITY);
                const double decel=response.enabled?std::max(response.required_decel,residual):residual;
                if(gap<result.minimum_rear_gap_m){result.minimum_rear_gap_m=gap;result.rear_id=obj.id;}
                result.minimum_rear_ttc_s=std::min(result.minimum_rear_ttc_s,ttc);
                result.maximum_rear_decel_mps2=std::max(result.maximum_rear_decel_mps2,decel);
                if(entering&&(gap<cfg.min_rear_gap||ttc<cfg.min_rear_ttc||decel>cfg.max_rear_decel_mps2)) {
                    result.reason="rear_gap_ttc_or_required_braking";result.blocking_object_id=obj.id;return result;
                }
            } else if(entering) {
                result.reason="alongside_target_traffic";result.blocking_object_id=obj.id;return result;
            }
        }
        if(t>=duration)break;
    }
    result.covers_completion=result.clear_time_s>=0&&duration-result.clear_time_s+1e-9>=cfg.post_merge_horizon_s;
    if(entering&&!result.covers_completion){result.reason="insufficient_completion_horizon";return result;}
    if(std::chrono::steady_clock::now()>=deadline) {
        result.reason="planning_time_budget";return result;
    }
    result.certified_path_samples=end_sample+1;
    result.safe=true;result.reason="safe_candidate";return result;
}

