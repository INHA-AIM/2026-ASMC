#include "frenet_planner/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>
int failures=0;
void Expect(bool value,const char* name){std::cout<<(value?"[PASS] ":"[FAIL] ")<<name<<'\n';if(!value)++failures;}
CartesianState Ego(const YAML::Node& v){return {v["x"].as<double>(),v["y"].as<double>(),v["yaw"].as<double>(),0,v["speed"].as<double>(),v["acceleration"].as<double>()};}
std::vector<TrackedObject> Objects(const YAML::Node& nodes){
 std::vector<TrackedObject> objects;
 for(auto o:nodes){TrackedObject obj;obj.id=o["id"].as<int>();obj.type=o["type"].as<int>();obj.motion_state=static_cast<ObjectMotionState>(o["motion_state"].as<int>());
 obj.x=o["center"]["x"].as<double>();obj.y=o["center"]["y"].as<double>();obj.z=o["center"]["z"].as<double>();obj.heading=o["yaw"].as<double>();
 obj.vx=o["vx"].as<double>();obj.vy=o["vy"].as<double>();obj.speed=std::hypot(obj.vx,obj.vy);obj.length=o["length"].as<double>();obj.width=o["width"].as<double>();obj.height=o["height"].as<double>();obj.geometry_valid=true;obj.z_valid=true;
 for(int i=0;i<8;++i){auto p=o["obb_corners"][i];obj.corners[i]={p["x"].as<double>(),p["y"].as<double>(),p["z"].as<double>()};}objects.push_back(obj);}
 return objects;
}
int main(){
 const std::string root=std::string(FRENET_PACKAGE_SRC_DIR)+"/../";
 const auto fixture=YAML::LoadFile(root+"tests/data/avoid_road_constraints.json");
 auto config=YAML::LoadFile(root+"config/params.yaml");config["waypoint_file"]=FRENET_TEST_WAYPOINT_FILE;config["avoid_hd_map"]["enabled"]=true;
 {std::ofstream out("avoid_road_constraints_params.yaml");out<<config;}
 AvoidRoadMap map;std::string error;Expect(map.Load(root+"config/hd_map/drivable.json",root+"config/hd_map/lines.json",error),"pinned map loads");
 if(!error.empty())std::cerr<<error<<'\n';
 VehicleShape shape{};shape.width=1.9;shape.length=4.635;shape.rear_overhang=.79;
 for(auto old:fixture["unsafe"]){CartesianPath path;for(auto p:old){path.x.push_back(p["x"].as<double>());path.y.push_back(p["y"].as<double>());path.yaw.push_back(p["yaw"].as<double>());}Expect(!map.PathClear(path,shape),"recorded departure trajectory rejected");}
 // Build synthetic paint contact poses from the public map geometry, not a
 // driving log. A footprint already touching paint must be able to recover.
 const auto lines=YAML::LoadFile(root+"config/hd_map/lines.json");
 bool found_paint=false;CartesianPath painted;
 for(auto feature:lines["features"]){
  const auto attr=feature["attributes"];
  if(feature["feature_type"].as<std::string>()!="lane_marking" ||
     attr["pattern"].as<std::string>()!="solid" ||
     attr["functional_role"].as<std::string>()!="lane_boundary_paint")continue;
  const auto coordinates=feature["geometry"]["coordinates"];
  for(std::size_t i=1;i<coordinates.size()&&!found_paint;++i){
   const double x=coordinates[i][0].as<double>(),y=coordinates[i][1].as<double>();
   const double yaw=std::atan2(y-coordinates[i-1][1].as<double>(),x-coordinates[i-1][0].as<double>());
   painted={};painted.x={x,x};painted.y={y,y};painted.yaw={yaw,yaw};
   const auto evaluation=map.EvaluatePath(painted,shape,.1);
   if(evaluation.drivable&&evaluation.paint_contact_seconds>0)found_paint=true;
  }
  if(found_paint)break;
 }
 Expect(found_paint,"public map contains paint inside drivable pavement");
 if(found_paint){
  const auto brief=map.EvaluatePath(painted,shape,.1);
  Expect(!map.PoseClear(painted.x[0],painted.y[0],painted.yaw[0],shape),"strict diagnostic detects initial paint contact");
  Expect(map.PathClear(painted,shape),"initial paint contact alone does not kill recovery candidates");
  CartesianPath long_contact=painted;
  long_contact.x.assign(41,painted.x[0]);long_contact.y.assign(41,painted.y[0]);long_contact.yaw.assign(41,painted.yaw[0]);
  const auto prolonged=map.EvaluatePath(long_contact,shape,.1);
  Expect(prolonged.drivable&&prolonged.longest_paint_contact_seconds>3,"prolonged paint contact is a scored fallback, not a wall");
  Expect(AvoidRoadMap::PaintCost(prolonged)>AvoidRoadMap::PaintCost(brief)+200,"long contact costs more than brief contact");
  painted.x.assign(2,10000);painted.y.assign(2,10000);
  Expect(!map.PathClear(painted,shape),"paint policy does not relax physical road containment");
 }
 FrenetPlanner planner;Expect(planner.Init("avoid_road_constraints_params.yaml"),"planner loads map");
 auto ego=Ego(fixture["approach"]["ego"]);auto objects=Objects(fixture["approach"]["objects"]);
 PlannerCommand command;command.mode=AVOID;command.target_object_id=83;command.has_target_speed=true;command.target_speed=50./3.6;
 CartesianPath path;FrenetPlanner::AvoidPrepareResult result;
 Expect(planner.PrepareAvoid(ego,objects,command,path,result),"unknown target has a maintenance trajectory");
 Expect(!result.candidate_valid&&!planner.CanCommitAvoid(result),"unknown motion cannot commit a transient bypass");
 Expect(path.x.size()>2&&std::fabs(path.v.front()-ego.v)<1e-9&&path.v.back()<ego.v-2,"moving approach brakes from measured speed");
 Expect(map.PathClear(path,shape),"braking approach stays on road");
 // Counterfactual ideal tracking: same fixed OBB, static confirmation after 2s.
 // This tests planner continuity, not MORAI vehicle dynamics or detector recall.
 bool committed=false,complete=false;double locked=0;int stalls=0;double max_ms=0;
 for(int step=0;step<350;++step){
   if(step>=20)for(auto& object:objects)object.motion_state=ObjectMotionState::LOW_THREAT;
   result={};path={};auto before=std::chrono::steady_clock::now();
   const bool ok=committed?planner.PlanCommittedAvoid(ego,objects,command,path,result):planner.PrepareAvoid(ego,objects,command,path,result);
   max_ms=std::max(max_ms,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count());
   double current_s=0;planner.ProjectEgoToS(ego,current_s);
   if(!ok||path.x.size()<2){std::cout<<"STALL step="<<step<<" ok="<<ok<<" n="<<path.x.size()<<" v="<<ego.v<<" d="<<planner.last_d()<<" s="<<current_s<<" gap="<<planner.avoid_target_s()-current_s<<" phase="<<planner.avoid_phase_name()<<" candidate="<<result.candidate_valid<<" gen="<<result.generated_candidate_count<<" valid="<<result.valid_candidate_count<<"\n";++stalls;break;}
   if(!committed&&result.candidate_valid){committed=planner.CommitAvoid(result);locked=result.selected_target_d;path=result.candidate_path;}
   if(committed&&std::fabs(planner.avoid_locked_target_d()-locked)>1e-8){++stalls;break;}
   if(!map.PathClear(path,shape)){++stalls;break;}
   for(std::size_t i=1;i<path.v.size();++i){if(std::fabs((path.v[i]-path.v[i-1])/planner.sample_dt())>3.001||std::fabs(path.a[i]-path.a[i-1])/planner.sample_dt()>6.001){++stalls;break;}}
   if(stalls){std::cout<<"KINSTALL step="<<step<<"\n";break;}
   ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
   if(planner.avoid_completed()){complete=true;break;}
 }
 std::cout<<"ideal_tracking committed="<<committed<<" complete="<<complete<<" stalls="<<stalls<<" max_planning_ms="<<max_ms<<'\n';
 Expect(committed,"static obstacle gets a complete map-constrained bypass");
 Expect(stalls==0&&complete,"ideal tracking completes avoidance and return without STOP gaps");
 FrenetPlanner junction;
 Expect(junction.Init("avoid_road_constraints_params.yaml"),"junction planner loads");
 auto junction_ego=Ego(fixture["junction"]["ego"]);
 auto junction_objects=Objects(fixture["junction"]["objects"]);
 command.target_object_id=99;command.requested_direction=LateralDirection::RIGHT;
 path={};result={};
 const bool junction_ok=junction.PrepareAvoid(junction_ego,junction_objects,command,path,result);
 Expect(junction_ok&&result.candidate_valid&&result.selected_target_d<0,"recorded junction state has a right bypass");
 Expect(result.candidate_valid&&map.PathClear(result.candidate_path,shape),"right bypass includes a road-constrained return");
 // A paint-scored fallback must never override a physical OBB collision.
 auto barrier=junction_objects.front();barrier.id=900;
 barrier.motion_state=ObjectMotionState::LOW_THREAT;
 barrier.x=junction_ego.x;barrier.y=junction_ego.y;
 barrier.vx=barrier.vy=barrier.speed=0;barrier.width=barrier.length=20;
 for(int i=0;i<8;++i){const double xs[4]={-10,10,10,-10},ys[4]={-10,-10,10,10};
  barrier.corners[i].x=barrier.x+xs[i%4];barrier.corners[i].y=barrier.y+ys[i%4];}
 junction_objects.push_back(barrier);path={};result={};
 junction.PrepareAvoid(junction_ego,junction_objects,command,path,result);
 Expect(!result.candidate_valid,"paint fallback cannot select a colliding trajectory");
 return failures?1:0;
}
