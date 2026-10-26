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
