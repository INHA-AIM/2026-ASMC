#include "Behavior/Feature/DynamicObjectPolicy.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include <algorithm>
#include <cmath>

namespace behavior_planner {
namespace {
bool IntersectsRoad(const ::TrackedObject& object, const AvoidRoadMap& road) {
  geometry::Polygon2D polygon;
  for (const auto& p : object.corners) polygon.push_back({p.x,p.y});
  return road.IntersectsDrivable(polygon);
}
}
bool DynamicObjectPolicy::inScope(const std::string& link) const {
  return enabled() && !link.empty() &&
      std::find(config_.route_links.begin(),config_.route_links.end(),link)!=config_.route_links.end();
}
bool DynamicObjectPolicy::allowsStaticAvoid(const ObjectFeature& object,
    const RouteManager& route) const {
  if (!enabled() || !object.valid || object.type != ObjectType::UNKNOWN) return true;
  double s=0,d=0;std::string link;
  if (!route.projectEgo(object.x,object.y,&s,&d,&link)) return true;
  // Standstill does not establish static semantics. Prevent an adjacent-lane
  // static bypass from preempting the configured dynamic section. New actions
  // still require the EGO to enter that section; native collision safety sees
  // the unchanged complete object list throughout the approach.
  return !inScope(link);
}
PerceptionObjects DynamicObjectPolicy::legacyObjects(const PerceptionObjects& source,bool active,
    bool validated_native_avoid,const std::vector<int>& evaluated_provisional_ids,int nominated_in_lane_target) {
  auto result=source;
  if(active) for(auto* group:{&result.vehicles,&result.pedestrians,&result.obstacles}) {
    group->erase(std::remove_if(group->begin(),group->end(),
        [validated_native_avoid,&evaluated_provisional_ids,nominated_in_lane_target](const TrackedObject& o){
          const bool evaluated=std::find(evaluated_provisional_ids.begin(),evaluated_provisional_ids.end(),o.id)!=evaluated_provisional_ids.end();
          return o.motion_state==MotionState::DYNAMIC ||
              ((o.motion_state==MotionState::LOW_THREAT || o.motion_state==MotionState::DYNAMIC_PENDING) &&
               o.type==ObjectType::PEDESTRIAN &&
               ((o.motion_state==MotionState::LOW_THREAT && validated_native_avoid) ||
                evaluated)) ||
              (o.type==ObjectType::UNKNOWN && o.motion_state==MotionState::LOW_THREAT &&
               (validated_native_avoid || (evaluated && o.id==nominated_in_lane_target)));
        }),group->end());
  }
  return result;
}
bool DynamicObjectPolicy::usesNativePathSafety(BehaviorType active,PlannerExecutionState state) {
  if(state==PlannerExecutionState::COMPLETED || state==PlannerExecutionState::FAILED)return false;
  if(active==BehaviorType::AVOID)
    return state==PlannerExecutionState::COMMITTED || state==PlannerExecutionState::EXECUTING;
  return active==BehaviorType::MERGE || active==BehaviorType::LEFT_CHANGE || active==BehaviorType::RIGHT_CHANGE;
}
const char* DynamicActionName(DynamicAction action) {
  switch(action) {
    case DynamicAction::FOLLOW:return "follow";
    case DynamicAction::AVOID:return "avoid";
    case DynamicAction::STOP:return "stop";
    case DynamicAction::EMERGENCY:return "emergency";
    default:return "keep";
  }
}
DynamicDecision DynamicObjectPolicy::evaluate(const UrbanFeature& feature,const RouteManager& route,
    const std::vector<::TrackedObject>& objects,double age,bool ordinary,bool previous_validated_stop) const {
  DynamicDecision result;
  if(!inScope(feature.route.route_required_link_id)) {
    result.reason="outside_dynamic_section";return result;
  }
  result.in_scope=true;
  result.section_speed_limit=config_.section_speed_limit;
  if(!ordinary) { result.reason="native_maneuver_owns_path_safety";return result; }
  if(!feature.ego.valid || !std::isfinite(age) || age<0 || age>.3) {
    result.reason="dynamic_observation_unavailable";return result;
  }
  // Predict intended ordinary motion, including a bounded launch from rest.
  // Execution always checks the actual generated path independently.
  CartesianPath nominal;
  double distance=0,v=feature.ego.speed;
  const int samples=int(std::ceil(config_.horizon_s/config_.step_s));
  for(int i=0;i<=samples;++i) {
    const double t=i*config_.step_s;
    Point2D center;double yaw=0;
    if(!route.getRoutePose(feature.ego.s+distance,&center,&yaw)) break;
    nominal.x.push_back(i?center.x-std::sin(yaw)*feature.ego.d:feature.ego.x);
    nominal.y.push_back(i?center.y+std::cos(yaw)*feature.ego.d:feature.ego.y);
    nominal.yaw.push_back(i?yaw:feature.ego.yaw);
    nominal.v.push_back(v);nominal.a.push_back(0);nominal.kappa.push_back(0);
    if(t>=config_.reaction_s) v=std::min(std::max(std::min(feature.route.speed_limit,
        config_.section_speed_limit),feature.ego.speed),
        v+config_.restart_accel*config_.step_s);
    distance+=v*config_.step_s;
  }
  if(nominal.x.size()<2) {result.reason="route_prediction_unavailable";return result;}
  auto collision=environment_.collision;
  // Forecast exactly this supplied horizon; the shared checker must not append
  // a coast extension that this decision did not specify.
  collision.reactive_lookahead=0;
  std::vector<int> evaluated_provisional_ids;
  std::vector<DynamicAssessment> assessments;
  for(const auto& source:objects) {
    DynamicAssessment assessment;
    assessment.id=source.id;assessment.type=source.type;
    assessment.motion=static_cast<int>(source.motion_state);
    const bool confirmed=source.motion_state==ObjectMotionState::DYNAMIC;
    const bool pending=source.motion_state==ObjectMotionState::DYNAMIC_PENDING;
    const bool known_mobile=source.type==static_cast<int>(ObjectType::PEDESTRIAN) ||
        source.type==static_cast<int>(ObjectType::VEHICLE);
    // A new track can already occupy our lane before perception has enough
    // samples to settle its motion flag. Waiting for PENDING lost ~0.7 s in
    // the captured deer collision. Evaluate the observed occupancy now, using
    // the same shared motion model, but grant no lead/bypass ownership.
    const bool unmeasured=source.velocity_unavailable ||
        (source.motion_state==ObjectMotionState::UNKNOWN &&
         (known_mobile || source.type==static_cast<int>(ObjectType::UNKNOWN)));
    // LOW_THREAT describes measured motion, not semantics or proven rest.
    // A late camera label must not postpone an occupancy brake. Unclassified
    // LOW objects may nominate a bounded in-lane path; the native planner must
    // validate its entire occupancy before granting motion. Semantic names do
    // not change the motion model or confer permission to leave the lane.
    const bool low_motion=source.motion_state==ObjectMotionState::LOW_THREAT &&
        (known_mobile || source.type==static_cast<int>(ObjectType::UNKNOWN));
    if((!confirmed && !pending && !low_motion && !unmeasured) || !source.geometry_valid ||
       !std::isfinite(source.vx) || !std::isfinite(source.vy) ||
       !geometry::ValidateCorners8(source.corners).valid) {
      assessment.gate="invalid_or_unmeasured_motion";assessments.push_back(assessment);continue;
    }
    if(!IntersectsRoad(source,*environment_.road)) {
      ++result.outside;assessment.gate="outside_drivable";assessments.push_back(assessment);continue;
    }
    auto object=AdvanceObjectObservation(source,age);
    const auto footprint=PredictObstacleFootprint(source,age);
    if(footprint.size()<3) continue;
    double s=0,d=0;std::string link;
    if(!route.projectEgo(object.x,object.y,&s,&d,&link)) continue;
    Point2D center;double route_yaw=feature.ego.yaw;route.getRoutePose(s,&center,&route_yaw);
    const double c=std::cos(route_yaw),sn=std::sin(route_yaw);
    const double along=object.vx*c+object.vy*sn;
    const double lateral=-object.vx*sn+object.vy*c;
    // The canonical footprint is authoritative even when scalar size metadata
    // disagrees. Use the same predicted corners as the shared collision check.
    double half_length=0,half_width=0;
    for(const auto& p:footprint) {
      const double dx=p.x-object.x,dy=p.y-object.y;
      half_length=std::max(half_length,std::abs(dx*c+dy*sn));
      half_width=std::max(half_width,std::abs(-dx*sn+dy*c));
    }
    const double front=environment_.shape.length-environment_.shape.rear_overhang;
    const double gap=s-feature.ego.s-half_length-front;
    const bool ahead=s>feature.ego.s && s-feature.ego.s<60;
    const bool corridor=std::fabs(d-feature.ego.d)<half_width+environment_.shape.width*.5;
    const bool aligned=along>0 && std::fabs(lateral)<std::max(.3,.5*along);
    const bool low_lead=low_motion && source.type==static_cast<int>(ObjectType::VEHICLE) &&
        source.vx*c+source.vy*sn>=0 && std::fabs(-source.vx*sn+source.vy*c)<.3;
    // Motion confirmation/direction is not a prerequisite for collision
    // braking. A PENDING object can occupy the lane before its lateral speed
    // settles. It may request only STOP/E-Stop, never FOLLOW or AVOID.
    // Use the unchanged shared provisional-motion model and final geometry.
    if(low_motion || pending)evaluated_provisional_ids.push_back(source.id);
    ++result.eligible;
    auto safety=EvaluateCartesianTrajectorySafety(nominal,{object},environment_.shape,collision,config_.step_s);
    bool observed_restart_conflict=false;
    if(previous_validated_stop && feature.ego.speed<1.) {
      // Releasing a real stop requires clearing the presently observed body,
      // too. A crossing track's optimistic longitudinal velocity made its
      // forecast clear while the native walker still occupied our lane.
      // Reuse the shared checker with this SAME fresh observation held in
      // place; no extra track, lost-object memory or timer is introduced.
      auto occupied=source;
      occupied.vx=occupied.vy=occupied.yaw_rate=occupied.speed=0.;
      const auto observed=EvaluateCartesianTrajectorySafety(nominal,{occupied},
          environment_.shape,collision,config_.step_s);
      observed_restart_conflict=!observed.clear;
      if(observed_restart_conflict && (safety.clear || observed.blocking_sample_index<safety.blocking_sample_index))
        safety=observed;
    }
    DynamicDecision candidate;
    candidate.in_scope=true;candidate.section_speed_limit=result.section_speed_limit;
    candidate.object_id=object.id;
    candidate.ttc=safety.clear?1e9:std::max(0,safety.blocking_sample_index)*config_.step_s;
    assessment.known_mobile=known_mobile;assessment.ttc=candidate.ttc;
    assessment.object_speed=std::hypot(object.vx,object.vy);
    assessment.lateral=d;assessment.half_width=half_width;
    assessment.gate=safety.clear?"nominal_path_clear":"occupancy_conflict";
    const double closing=std::max(0.0,feature.ego.speed-along);
    const double braking_gap=gap-closing*config_.reaction_s-environment_.collision.safety_margin;
    const double required=closing*closing/(2*std::max(.1,braking_gap));
    if(!observed_restart_conflict && ahead && corridor && ((confirmed && aligned) || low_lead)) {
      assessment.gate="lead_follow_owns_response";
      candidate.action=(braking_gap<=0 || required>config_.emergency_decel)
          ?DynamicAction::EMERGENCY:DynamicAction::FOLLOW;
      candidate.reason=candidate.action==DynamicAction::FOLLOW
          ?(low_lead?"low_motion_vehicle_gap_follow":"confirmed_in_road_lead")
          :"lead_braking_margin_exhausted";
      candidate.lead.id=object.id;candidate.lead.type=static_cast<ObjectType>(object.type);
      candidate.lead.motion_state=static_cast<MotionState>(source.motion_state);candidate.lead.valid=true;
      candidate.lead.x=object.x;candidate.lead.y=object.y;candidate.lead.yaw=object.heading;
      candidate.lead.vx=object.vx;candidate.lead.vy=object.vy;candidate.lead.speed=std::hypot(object.vx,object.vy);
      candidate.lead.length=object.length;candidate.lead.width=object.width;
      candidate.lead.s=s;candidate.lead.d=d;candidate.lead.gap=std::max(0.0,s-feature.ego.s);
      candidate.lead.relative_speed=closing;
      candidate.lead.ttc=closing>1e-3?std::max(0.0,gap)/closing:1e9;
    } else if(!safety.clear) {
      const double stop_time=config_.reaction_s+feature.ego.speed/config_.service_decel;
      candidate.action=candidate.ttc<=config_.reaction_s ||
          (feature.ego.speed>1 && candidate.ttc<config_.reaction_s+feature.ego.speed/config_.emergency_decel)
          ?DynamicAction::EMERGENCY:DynamicAction::STOP;
      candidate.reason=candidate.action==DynamicAction::EMERGENCY?"imminent_dynamic_occupancy":"yield_to_dynamic_occupancy";
      if(pending) candidate.reason=candidate.action==DynamicAction::EMERGENCY
          ?"pending_crossing_imminent":"pending_crossing_occupancy";
      if(unmeasured) candidate.reason=candidate.action==DynamicAction::EMERGENCY
          ?"unmeasured_occupancy_imminent":"unmeasured_observed_occupancy";
      if(observed_restart_conflict)candidate.reason="observed_restart_occupancy";
      // Only a slow object with time to brake can request an in-lane bypass.
      // Frenet PREPARE/READY still must generate a valid all-object path.
      // The timed shared collision check already established a conflict.
      // Requiring physical body-tube overlap here excluded edge objects
      // which need only a small, feasible in-lane offset. Nomination is not
      // permission to pass: native PREPARE still checks the whole swept body,
      // lane, all objects and unchanged uncertainty margins before COMMIT.
      const double half_ego=.5*environment_.shape.width;
      const double bound=.5*environment_.lane_width-half_ego-environment_.collision.safety_margin;
      // Necessary current cross-section room only. Native swept checks remain
      // final; a lane-filling object must request a stop, not an impossible bypass.
      const bool bypass_room=bound>0 &&
          (d-half_width-half_ego-environment_.collision.safety_margin>=-bound ||
           d+half_width+half_ego+environment_.collision.safety_margin<=bound);
      assessment.bypass_room=bypass_room;assessment.stop_time=stop_time;
      assessment.gate=!(confirmed || low_motion)?"bypass_motion_unconfirmed"
          :!ahead?"bypass_not_ahead":!bypass_room?"bypass_no_cross_section_room"
          :assessment.object_speed>config_.avoid_max_speed?"bypass_object_too_fast"
          :candidate.ttc<=stop_time+1?"bypass_insufficient_braking_time"
          :feature.ego.speed>=4?"bypass_ego_too_fast":"bypass_nomination";
      if(unmeasured)assessment.gate="unmeasured_observed_occupancy";
      if(observed_restart_conflict)assessment.gate="observed_restart_occupancy";
      if((confirmed || low_motion) && ahead && bypass_room &&
          std::hypot(object.vx,object.vy)<=config_.avoid_max_speed &&
          candidate.ttc>stop_time+1 && feature.ego.speed<4) {
        candidate.action=DynamicAction::AVOID;candidate.reason="slow_dynamic_bypass_prepare";
      }
      // A stationary conflict must have a stationary stop target. An endpoint
      // based on current ego + current braking distance moved every cycle and
      // restarted the STOP transaction during braking. Retain the object-front
      // and vehicle-front clearance; native STOP adds its own existing buffer.
      candidate.stop_s=feature.ego.s+std::max(0.0,gap-environment_.collision.safety_margin);
      Point2D stop;route.getRoutePose(candidate.stop_s,&stop,&candidate.stop_yaw);
      candidate.stop_x=stop.x;candidate.stop_y=stop.y;
    }
    assessments.push_back(assessment);
    if(int(candidate.action)>int(result.action) ||
       (candidate.action==result.action && candidate.action!=DynamicAction::KEEP &&
        (candidate.ttc<result.ttc || (candidate.action==DynamicAction::FOLLOW &&
         candidate.ttc==result.ttc && candidate.lead.gap<result.lead.gap)))) {
      const auto eligible=result.eligible,outside=result.outside;
      result=candidate;result.eligible=eligible;result.outside=outside;
    }
  }
  result.evaluated_provisional_ids=std::move(evaluated_provisional_ids);
  result.assessments=std::move(assessments);
  return result;
}
}
