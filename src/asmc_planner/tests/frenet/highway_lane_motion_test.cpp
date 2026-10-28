#include "Execution/HighwayReferenceExecution.hpp"
#include "Frenet/Geometry/HighwayLaneMotion.hpp"
#include "Frenet/Maneuver/Merge/HighwayPrediction.hpp"
#include <iostream>
using namespace asmc_planner::lane_reference;
namespace {
int checks=0,failures=0;
void expect(bool ok,const char* text){++checks;if(!ok){++failures;std::cerr<<"FAIL "<<text<<'\n';}}
TrackedObject actor(const LaneReference& lane,double s,double d,double speed,double lateral) {
 const auto p=Interpolate(lane.line(),s);TrackedObject o;
 o.id=12;o.type=int(asmc_planner::ObjectType::VEHICLE);o.motion_state=ObjectMotionState::DYNAMIC;
 o.x=p.x-d*std::sin(p.theta);o.y=p.y+d*std::cos(p.theta);o.heading=p.theta;
 o.vx=speed*std::cos(p.theta)-lateral*std::sin(p.theta);
 o.vy=speed*std::sin(p.theta)+lateral*std::cos(p.theta);
 o.width=1.8;o.length=4.5;o.height=1.5;o.geometry_valid=o.z_valid=true;
 for(int k=0;k<8;++k){const double x=k%4<2?-2.25:2.25,y=k%2?-.9:.9;
  o.corners[k]={o.x+std::cos(p.theta)*x-std::sin(p.theta)*y,o.y+std::sin(p.theta)*x+std::cos(p.theta)*y,k<4?0.0:1.5};}
 return o;
}
bool inside(const geometry::Polygon2D& polygon,geometry::Point2D p) {
 if(polygon.size()<3)return false;
 for(std::size_t k=0;k<polygon.size();++k){const auto a=polygon[k],b=polygon[(k+1)%polygon.size()];
  if((b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x)<-1e-7)return false;}
 return true;
}
void run() {
 FrenetPlanner mission;asmc_planner::execution::ManeuverState state;std::string detail;
 if(!mission.Init(FRENET_TEST_CONFIG)||!mission.EnableHighwayMerge(detail))throw std::runtime_error(detail);
 HighwayReferenceExecution execution(mission,FRENET_TEST_CONFIG,state);const auto network=execution.status().network;
 const auto& lane=*network->lanes[0];ObservedObjectMotion track;TrackedObject measured;
 for(int k=0;k<8;++k){measured=actor(lane,160+k*1.4,.10+k*.07,14,.7);track.Observe(measured,100+k*.1,1,.05,12,.8);}
 auto predicted=PrepareHighwayLaneMotion(measured,&track,network->lanes,.2,.7);
 expect(bool(predicted.prediction_lane),"position-supported lateral noise still uses the confirmed highway lane");
 bool same=true;for(int k=0;k<8;++k)same&=predicted.corners[k].x==measured.corners[k].x&&predicted.corners[k].y==measured.corners[k].y&&predicted.corners[k].z==measured.corners[k].z;
 expect(same,"lane prediction never rebuilds or shrinks measured corners");
 const auto zero=PredictTrackedObjectPose(predicted,0);
 expect(std::hypot(zero.x-measured.x,zero.y-measured.y)<1e-10,"current measured center is preserved exactly");
 const auto initial=ProjectOnLaneReference(lane,measured.x,measured.y,0,0);
 const auto future=PredictTrackedObjectPose(predicted,4);
 const auto fp=ProjectOnLaneReference(lane,future.x,future.y,0,0);
 expect(std::abs(fp.d-initial.d)<.02,"four-second prediction holds observed lateral lane offset");
 expect(std::abs(fp.s-initial.s-56)<.1,"observed along-lane speed determines forward progress");
 expect(predicted.prediction_lane_end_s<predicted.prediction_lane->length-10,
     "NPC prediction does not inherit ego's selected successor at the final fork");
 auto unknown=measured;unknown.type=int(asmc_planner::ObjectType::UNKNOWN);
 expect(bool(PrepareHighwayLaneMotion(unknown,&track,network->lanes,.2,.7).prediction_lane),
     "moving UNKNOWN vehicle fragments can use independently observed along-road displacement");
 auto stationary=track;for(auto& s:stationary.samples){s.x=measured.x;s.y=measured.y;}
 expect(!PrepareHighwayLaneMotion(unknown,&stationary,network->lanes,.2,.7).prediction_lane,
     "stationary UNKNOWN with a fictitious velocity is not promoted to traffic");
 for(auto type:{asmc_planner::ObjectType::PEDESTRIAN,asmc_planner::ObjectType::STATIC_OBSTACLE}) {
   auto other=measured;other.type=int(type);
   expect(!PrepareHighwayLaneMotion(other,&track,network->lanes,.2,.7).prediction_lane,"pedestrians and static objects retain original prediction");
 }
 auto low=measured;low.motion_state=ObjectMotionState::LOW_THREAT;
 expect(!PrepareHighwayLaneMotion(low,&track,network->lanes,.2,.7).prediction_lane,"confirmed stationary objects retain original prediction");
 auto pending=unknown;pending.motion_state=ObjectMotionState::UNKNOWN;
 expect(bool(PrepareHighwayLaneMotion(pending,&track,network->lanes,.2,.7).prediction_lane),
     "measured position history can establish a lane before the motion-state label settles");
 pending.type=int(asmc_planner::ObjectType::VEHICLE);
 expect(!PrepareHighwayLaneMotion(pending,&stationary,network->lanes,.2,.7).prediction_lane,
     "a camera vehicle label alone cannot establish unknown motion without position support");
 // Mature, unresolved lane association is a distinct fallback from initial grace.
 HighwayMergeConfig cfg;cfg.motion_fit_min_span_s=.2;cfg.min_rear_gap=8;
 VehicleShape test_shape;test_shape.length=4.635;test_shape.width=1.9;test_shape.rear_overhang=.8;
 CartesianState own{measured.x+60*std::cos(measured.heading),
     measured.y+60*std::sin(measured.heading),measured.heading,0,14,0};
 const auto unresolved=[&](const TrackedObject& object,const ObservedObjectMotion& history) {
   return PrepareHighwayUnresolvedMotion(object,&history,cfg,measured.heading,own,test_shape);
 };
 ObservedObjectMotion first;first.Observe(measured,101,1);
 const auto grace=unresolved(measured,first);
 expect(std::isfinite(grace.continuation_vx)&&grace.turn_prediction_duration==.2,
     "far newborn rear actor retains short measured motion but not a long lateral extrapolation");
 expect(grace.x==measured.x&&grace.y==measured.y&&grace.corners[0].x==measured.corners[0].x,
     "provisional rear treatment preserves the current measured footprint");
 auto no_lateral=track;
 const double tx=std::cos(measured.heading),ty=std::sin(measured.heading);
 for(auto& p:no_lateral.samples){const double along=(p.x-measured.x)*tx+(p.y-measured.y)*ty;
   p.x=measured.x+along*tx;p.y=measured.y+along*ty;}
 expect(std::isfinite(unresolved(measured,no_lateral).continuation_vx),
     "after grace, unmapped but position-unsupported transverse speed remains provisional");
 auto crossing_history=no_lateral;
 for(auto& p:crossing_history.samples){const double d=2*(p.time-crossing_history.stamp);
   p.x-=d*ty;p.y+=d*tx;}
 expect(!std::isfinite(unresolved(measured,crossing_history).continuation_vx),
     "after grace, position-supported lateral motion retains general collision prediction");
 auto nearby_ego=own;nearby_ego.x=measured.x+5*tx;nearby_ego.y=measured.y+5*ty;
 expect(!std::isfinite(PrepareHighwayUnresolvedMotion(measured,&first,cfg,measured.heading,
     nearby_ego,test_shape).continuation_vx),"near rear hazards do not receive provisional lateral grace");
 expect(!std::isfinite(PrepareHighwayUnresolvedMotion(measured,&first,cfg,NAN,own,test_shape).continuation_vx),
     "missing map direction cannot grant a straight-road prediction");
 auto ahead_ego=own;ahead_ego.x=measured.x-60*tx;ahead_ego.y=measured.y-60*ty;
 expect(!std::isfinite(PrepareHighwayUnresolvedMotion(measured,&first,cfg,measured.heading,
     ahead_ego,test_shape).continuation_vx),"front actors never receive the newborn rear policy");
 auto crossing=track;crossing.samples.back().x+=20;
 expect(!PrepareHighwayLaneMotion(measured,&crossing,network->lanes,.2,.7).prediction_lane,
     "inconsistent recent lane membership cannot certify a traffic hypothesis");
 ObservedObjectMotion newborn;newborn.Observe(measured,101,1);
 expect(!PrepareHighwayLaneMotion(measured,&newborn,network->lanes,.2,.7).prediction_lane,
     "new IDs must provide unique observations before map-constrained prediction");
 expect(!PrepareHighwayLaneMotion(measured,&track,{},.2,.7).prediction_lane,"no prepared highway catalog leaves legacy prediction unchanged");
 const auto& curve=*network->lanes[1];double station=0,maximum=0;
 for(const auto& p:curve.line().points)if(p.s>20&&p.s<curve.line().length-50&&std::abs(p.kappa)>maximum){maximum=std::abs(p.kappa);station=p.s;}
 ObservedObjectMotion curved;TrackedObject observation;
 for(int k=0;k<7;++k){observation=actor(curve,station-6+k*.8,.1,8,0);curved.Observe(observation,200+k*.1,1,.05,12,.8);}
 auto moving=PrepareHighwayLaneMotion(observation,&curved,network->lanes,.2,.7);
 expect(bool(moving.prediction_lane),"a confirmed vehicle on a curved lane uses its successor reference");
 moving.prediction_min_speed=4;moving.prediction_max_speed=12;
 const auto sweep=PredictObstacleFootprint(moving,3);
 bool covered=true;
 const double nominal=std::hypot(moving.vx,moving.vy);
 for(int k=0;k<=80;++k){auto middle=moving;const double speed=4+k*.1;middle.vx*=speed/nominal;middle.vy*=speed/nominal;
   const auto corners=PredictTrackedObjectCorners(middle,3);
   for(const auto& corner:corners)covered&=inside(sweep,{corner.x,corner.y});}
 expect(covered,"curved speed-interval sweep covers intermediate speeds rather than just endpoint boxes");
 const auto at=PredictTrackedObjectPose(moving,0);CartesianState ego{at.x,at.y,moving.heading,0,0,0};VehicleShape shape;
 shape.length=4.635;shape.width=1.9;shape.rear_overhang=.8;
 expect(geometry::CheckConvexPolygonOverlapSAT(MakeEgoFootprint(ego,shape,0),PredictObstacleFootprint(moving,0)),
     "map prediction cannot hide a current physical collision");
}
}
int main(){try{run();}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
 std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
