// Regression against the exact committed compact-map coordinates. This used
// to reject the complete map on Boost 1.71 despite passing on Boost 1.89.
#include "Frenet/Reference/LaneReference.hpp"
#include "Execution/ReferenceTrialObjects.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>

using namespace asmc_planner;
using namespace asmc_planner::lane_reference;
namespace pt=boost::property_tree;
namespace {
int checks=0,failures=0;
void expect(bool yes,const char* name){++checks;if(!yes){++failures;std::cerr<<"FAIL: "<<name<<'\n';}}
std::vector<Point2D> points(const pt::ptree& rows){
  std::vector<Point2D> out;
  for(const auto& row:rows){auto p=row.second.begin();const double x=p++->second.get_value<double>();out.push_back({x,p->second.get_value<double>()});}
  return out;
}
std::vector<std::string> ids(const pt::ptree& rows){std::vector<std::string> out;for(const auto& row:rows)out.push_back(row.second.get_value<std::string>());return out;}
LaneMapData load(){
  pt::ptree root;pt::read_json(HD_LANE_MAP_PRODUCT,root);LaneMapData d;
  d.identity={root.get<std::string>("frame_id"),root.get<std::string>("build_id"),root.get<std::string>("map_id"),root.get<std::string>("reviewed_geometry_sha256")};d.complete=true;
  for(const auto& row:root.get_child("lanes")){
    const auto& p=row.second;LaneData l;l.id=p.get<std::string>("id");l.link_id=p.get<std::string>("link_id");
    l.left_adjacent=p.get<std::string>("left_adjacent_lane_id");l.right_adjacent=p.get<std::string>("right_adjacent_lane_id");
    l.predecessors=ids(p.get_child("predecessor_lane_ids"));l.successors=ids(p.get_child("successor_lane_ids"));
    l.centerline=points(p.get_child("centerline"));l.polygon=points(p.get_child("polygon"));
    const auto type=p.get<std::string>("lane_type");l.traversable=type!="unknown"&&type!="shoulder"&&p.get<std::string>("direction")!="unknown";d.lanes.push_back(std::move(l));
  }
  for(const auto& row:root.get_child("drivable_areas")){const auto& p=row.second;d.areas.push_back({p.get<std::string>("id"),points(p.get_child("polygon")),p.get<std::string>("validity")=="validated"});}
  return d;
}
LaneReferenceOptions options(){
  const auto p=YAML::LoadFile(FRENET_TEST_CONFIG)["planner"];const auto r=p["reference"],v=p["vehicle_shape"];
  LaneReferenceOptions o;o.reference={r["closure_tolerance_m"].as<double>(),r["smoothing_window_m"].as<double>(),r["resample_ds"].as<double>(),r["relax_max_iter"].as<int>(),r["relax_step"].as<double>(),r["ego_projection_search_window"].as<double>()};
  o.vehicle={v["length"].as<double>()-v["rear_overhang"].as<double>(),v["rear_overhang"].as<double>(),v["width"].as<double>()/2};
  o.max_curvature=p["kinematic_limits"]["max_curvature"].as<double>();o.max_lateral_accel=p["kinematic_limits"]["max_lateral_accel"].as<double>();
  o.max_join_gap_m=.05;o.sample_distance_m=.2;o.sample_yaw_rad=.02;o.max_samples=30000;return o;
}
void run(){
  auto data=load();std::string error;const auto map=HdLaneMap::create(data,error);
  expect(bool(map),"complete compact map accepted on supported Boost versions");
  if(!map){std::cerr<<error<<'\n';return;}
  expect(map->laneCount()==data.lanes.size(),"all source lanes retained");
  bool vertices=true;
  for(const auto& a:data.areas)for(std::size_t i=0;i<a.polygon.size();++i) {
    const auto& p=a.polygon[i];const auto& before=a.polygon[(i+a.polygon.size()-1)%a.polygon.size()];
    const auto& after=a.polygon[(i+1)%a.polygon.size()];
    // A zero-area retraced spur does not define traversable surface.
    if(before.x==after.x && before.y==after.y)continue;
    vertices=vertices&&map->road()->coversPoint(p.x,p.y);
  }
  expect(vertices,"all validated source boundary vertices remain covered");
  expect(!map->road()->coversPoint(1e6,1e6),"outside point is not made drivable");
  const auto trial=map->corridor({"lane:A2256W000408","lane:A2256W000434"},error);
  TrackedObject roadside;roadside.id=1104;roadside.motion_state=MotionState::LOW_THREAT;
  roadside.x=81.65422163963319;roadside.y=-49.60868804931641;roadside.z=-.4772889911785572;
  roadside.heading=-1.5291538063214336;roadside.width=.19986118392752608;
  roadside.length=6.28458216918049;roadside.height=.2428213148180295;
  roadside.geometry_valid=roadside.z_valid=true;
  const std::vector<Point2D> observed={{81.62325050354002,-46.464960937499995},
    {81.4235625839234,-46.473281250000014},{81.68519279479983,-52.75241516113283},
    {81.88488067626952,-52.74409484863281}};
  int corner=0;for(double z:{-.598699648587572,-.3558783337695425})
    for(const auto& p:observed)roadside.corners[corner++]={p.x,p.y,z};
  expect(trial&&TrialObjectsAllowStart(*trial,{roadside},error),"reported ID1104 outside actual A408-A434 no longer blocks start");
  auto moved=roadside;moved.id=1105;moved.x-=10;for(auto& p:moved.corners)p.x-=10;
  expect(trial&&!TrialObjectsAllowStart(*trial,{roadside,moved},error),"same observed OBB moved into actual target lane still blocks");
  auto dynamic=roadside;dynamic.motion_state=MotionState::DYNAMIC_PENDING;
  expect(trial&&!TrialObjectsAllowStart(*trial,{dynamic},error),"off-corridor pending motion still withholds this empty-traffic trial");
  const auto o=options();
  const auto target=BuildLaneReference(map,map->identity(),{"lane:A2256W000434"},o);
  expect(bool(target),"actual A434 target footprint remains drivable");
  if(!target)std::cerr<<target.detail<<'\n';
  const auto chain=BuildLaneReference(map,map->identity(),{"lane:A2256W000445","lane:A2256W000153"},o);
  expect(bool(chain),"actual A445-A153 seam remains drivable");
  if(!chain)std::cerr<<chain.detail<<'\n';
  std::reverse(data.areas.begin(),data.areas.end());const auto reverse=HdLaneMap::create(data,error);
  expect(bool(reverse),"complete map also loads in reversed polygon order");
  expect(reverse&&bool(BuildLaneReference(reverse,reverse->identity(),{"lane:A2256W000434"},o)),"target coverage survives reversed polygon order");
}
}
int main(){try{run();}catch(const std::exception& e){++failures;std::cerr<<e.what()<<'\n';}
std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
