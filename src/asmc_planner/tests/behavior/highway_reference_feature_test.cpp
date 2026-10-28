#include "Behavior/Feature/feature_builder.hpp"
#include "Behavior/Map/mgeo_loader.hpp"
#include "Output/Behavior/utils.hpp"
#include "Frenet/frenet_planner.hpp"
#include <cmath>
#include <iostream>
namespace {
struct Green final:behavior_planner::TrafficLightProvider {
  behavior_planner::TrafficLightState getState(const std::string&,behavior_planner::SignalMovement)const override {
    return behavior_planner::TrafficLightState::GREEN;
  }
  bool hasRecentDetection(const std::string&)const override{return true;}
};
}
int main() {
  using namespace asmc_planner::lane_reference;
  using namespace behavior_planner;
  int checks=0,failures=0;
  const auto expect=[&](bool yes,const char* what){++checks;if(!yes){++failures;std::cerr<<"FAIL "<<what<<'\n';}};
  const std::string root=BEHAVIOR_PLANNER_SOURCE_DIR;
  MGeoLoader map;RouteManager route;
  if(!map.load(root+"/config/behavior/link_set.json",root+"/config/behavior/node_set.json")||
     !route.initialize(map,loadRouteLinks(root+"/config/behavior/route_links_1.yaml")))return 1;
  FrenetPlanner core;std::string error;
  if(!core.Init(root+"/config/frenet/params.yaml")||!core.EnableHighwayMerge(error))return 1;
  const auto highway=core.highway_route();
  LaneReferenceOptions o;o.reference=core.reference_config();
  o.vehicle={highway->vehicle.length-highway->vehicle.rear_overhang,highway->vehicle.rear_overhang,highway->vehicle.width*.5};
  o.max_curvature=.23;o.max_lateral_accel=2.6;o.max_join_gap_m=.05;o.sample_distance_m=.2;o.sample_yaw_rad=.02;o.max_samples=30000;
  const auto built=BuildLaneReference(highway->map,highway->map->identity(),highway->zones[1].config.target_lanes,o);
  if(!built){std::cerr<<built.detail;return 1;}
  const auto lane=built.candidate;const auto p=Interpolate(lane->line(),65);
  EgoState ego{p.x,p.y,p.theta,10,100,1,true};
  PerceptionObjects objects;
  auto add=[&](int id,const RefLine& ref,double s){
    const auto q=Interpolate(ref,s);behavior_planner::TrackedObject object;
    object.id=id;object.type=ObjectType::VEHICLE;object.motion_state=MotionState::DYNAMIC;
    object.x=q.x;object.y=q.y;object.yaw=q.theta;object.speed=8;
    object.vx=8*std::cos(q.theta);object.vy=8*std::sin(q.theta);
    objects.vehicles.push_back(object);
  };
  add(2,lane->line(),95);
  const double mission_s=FindClosestS(core.reference(),p.x,p.y,nullptr,100);
  add(1,core.reference(),mission_s+15);
  FeatureBuilder builder;builder.setHighwayRoute(highway);Green green;
  HighwayReferenceStatus status;status.lane=lane;status.completed={true,true};status.epoch=3;
  builder.setHighwayReference(status);
  const auto feature=builder.build(ego,objects,route,green);
  std::cerr<<"highway valid="<<feature.highway.valid<<" approach="<<feature.highway.approach
      <<" mission_s="<<mission_s<<" feature_route_s="<<feature.highway.route_s
      <<" lane_s="<<feature.ego.s<<'\n';
  expect(feature.ego.valid,"off-route adopted lane still has a valid ego feature");
  expect(std::abs(feature.ego.d)<.1&&std::abs(feature.ego.s-65)<.5,"ego uses adopted lane coordinates");
  expect(feature.surrounding.has_front_vehicle&&feature.surrounding.front_vehicle.id==2,
      "leader selection follows adopted lane rather than closer former-lane traffic");
  expect(feature.surrounding.front_vehicle.gap>29&&feature.surrounding.front_vehicle.gap<31,
      "leader gap and ego use the same reference");
  expect(feature.highway.valid&&!feature.highway.approach,"completed zone never requests another MERGE");
  expect(feature.highway.cruising&&std::abs(feature.highway.speed_cap_mps-highway->config.speed_cap_mps)<1e-9,
      "active prepared reference carries highway cruise policy after MERGE completion");
  expect(feature.highway.route_s>1400,"mission progress is retained separately from local lane station");
  std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
