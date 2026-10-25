#include "Frenet/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <filesystem>
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

// A bounded search can report pending before its later candidate slice is
// evaluated. These geometry assertions wait on the same input snapshot; the
// separate side-search test checks braking output and per-call latency.
bool AwaitPrepare(FrenetPlanner& planner,const CartesianState& ego,
                  const std::vector<TrackedObject>& objects,const PlannerCommand& command,
                  CartesianPath& path,FrenetPlanner::AvoidPrepareResult& result){
    bool ok=false;
    for(int i=0;i<100;++i){path={};result={};ok=planner.PrepareAvoid(ego,objects,command,path,result);if(result.candidate_valid)break;}
    return ok;
}
bool AwaitCommitted(FrenetPlanner& planner,const CartesianState& ego,
                    const std::vector<TrackedObject>& objects,const PlannerCommand& command,
                    CartesianPath& path,FrenetPlanner::AvoidPrepareResult& result){
    bool ok=false;
    for(int i=0;i<100;++i){path={};result={};ok=planner.PlanCommittedAvoid(ego,objects,command,path,result);if(result.candidate_valid)break;}
    return ok;
}

// Synthetic geometry only: a road widens after x=45, then several independently
// tracked boxes occupy the same bypass corridor. No recorded bag is needed.
void SyntheticReplanning(const YAML::Node& original) {
 const auto dir=std::filesystem::absolute("avoid_replanning_fixture");
 std::filesystem::create_directories(dir);
 {std::ofstream f(dir/"waypoints.txt");for(int x=0;x<=200;++x)f<<x<<" 0\n";}
 {std::ofstream f(dir/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-4],[200,-4],[200,4],[0,4]]}]})";}
 {std::ofstream f(dir/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[200,5]]}}]})";}
 auto grid=[&](bool narrow){std::ofstream f(dir/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int row=10;row<50;++row)f<<row<<' '<<(narrow&&(row<21||row>=39)?225:0)<<" 1000\n";};
 auto cfg=YAML::Clone(original);cfg["roundabout_merge"]["enabled"]=false;cfg["waypoint_file"]=(dir/"waypoints.txt").string();cfg["avoid_hd_map"]["directory"]=dir.string();
 cfg["planner"]["collision_check"]["margin_growth_rate"]=0.0;
 {std::ofstream f(dir/"params.yaml");f<<cfg;}
 auto box=[](int id,double x,double y,double width){
  TrackedObject o{};o.id=id;o.x=x;o.y=y;o.z=0;o.width=width;o.length=1;o.height=1;o.heading=0;
  o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;
  for(int i=0;i<8;++i){const double xs[4]={-.5,.5,.5,-.5},ys[4]={-.5,-.5,.5,.5};o.corners[i]={x+xs[i%4],y+width*ys[i%4],double(i/4)};}
  return o;
 };
 PlannerCommand cmd{};cmd.mode=AVOID;cmd.target_object_id=1;cmd.requested_direction=LateralDirection::RIGHT;cmd.has_target_speed=true;cmd.target_speed=2;
 CartesianPath p;FrenetPlanner::AvoidPrepareResult r;
 grid(true);FrenetPlanner retime;Expect(retime.Init((dir/"params.yaml").string()),"synthetic widening road loads");
 CartesianState ego{};ego.x=30;ego.y=-.7;ego.v=2;
 std::vector<TrackedObject> objects{box(1,80,1.1,.1)};
 retime.PrepareAvoid(ego,objects,cmd,p,r);
 const bool committed=retime.CommitAvoid(r);const double original_entry=r.map_entry_s;
 Expect(committed&&original_entry<45,"small initial box commits before road widens");
 objects[0]=box(1,80,1.1,2.0);p={};r={};
 const bool resumed=retime.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(resumed&&r.candidate_valid&&r.map_entry_s>original_entry+2&&retime.avoid_phase_name()==std::string("TRACKING"),"changed box replans entry before measured lateral motion");
 const double entry=r.map_entry_s;
 p={};r={};retime.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(r.candidate_valid&&std::fabs(r.map_entry_s-entry)<1e-6,"valid retimed bypass keeps its entry station");
 grid(false);FrenetPlanner multi;Expect(multi.Init((dir/"params.yaml").string()),"synthetic wide road loads");
 ego={};ego.x=30;ego.v=2;objects={box(1,80,.5,.3)};p={};r={};
 multi.PrepareAvoid(ego,objects,cmd,p,r);const bool first=multi.CommitAvoid(r);const double nominal=r.selected_target_d;
 Expect(first,"one box commits a nominal bypass");
 // Advance along accepted samples until lateral motion is actually underway.
 bool moved=false;
 for(int i=0;i<150&&first;++i){
  p={};r={};if(!multi.PlanCommittedAvoid(ego,objects,cmd,p,r)||p.x.size()<2)break;
  ego={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};
  if(ego.y<-.35){moved=true;break;}
 }
 Expect(moved,"synthetic vehicle starts the lateral bypass");
 objects.push_back(box(2,81,-.4,.4));p={};r={};
 const bool widened=AwaitCommitted(multi,ego,objects,cmd,p,r);
 double s=0;multi.ProjectEgoToS(ego,s);
 Expect(widened&&r.candidate_valid&&r.selected_target_d<nominal-.25,"second independent box widens the same-side bypass");
 Expect(widened&&r.map_entry_s<=s+1e-6,"actual lateral motion prevents entry postponement");
 const double wide_target=r.selected_target_d;p={};r={};multi.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(r.candidate_valid&&std::fabs(r.selected_target_d-wide_target)<1e-6,"valid widened target is retained on the next cycle");
 // A third box fills the entire remaining road; expansion must fail safely.
 objects.push_back(box(3,80,0,20));p={};r={};
 multi.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(!r.candidate_valid,"widening cannot bypass a full-road physical barrier");
 objects.pop_back();bool completed=false;int stalls=0;
 for(int i=0;i<800;++i){
  p={};r={};if(!AwaitCommitted(multi,ego,objects,cmd,p,r)||p.x.size()<2){++stalls;break;}
  if(multi.avoid_completed()){completed=true;break;}
  ego={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};
 }
 Expect(completed&&stalls==0,"two-box bypass completes its return under ideal tracking");
 std::filesystem::remove_all(dir);
}

int main(){
 const std::string root=std::string(FRENET_PACKAGE_SRC_DIR)+"/../";
 const auto fixture=YAML::LoadFile(root+"tests/frenet/data/avoid_road_constraints.json");
 auto config=YAML::LoadFile(root+"config/frenet/params.yaml");config["waypoint_file"]=FRENET_TEST_WAYPOINT_FILE;config["avoid_hd_map"]["enabled"]=true;
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
 // Synthetic braking disturbance near the locked bypass offset. A small
 // remaining lateral displacement must not imply a short heading transition.
 FrenetPlanner disturbed;
 Expect(disturbed.Init("avoid_road_constraints_params.yaml"),"disturbed planner loads");
 auto disturbed_ego=Ego(fixture["approach"]["ego"]);
 bool disturbed_committed=false,heading_injected=false,heading_resume_valid=false;
 int disturbed_stalls=0;
 for(int step=0;step<500;++step){
   if(disturbed_committed&&!heading_injected&&
      std::fabs(disturbed.last_d()-disturbed.avoid_locked_target_d())<.06&&
      disturbed.avoid_phase_name()==std::string("PASS")){
     disturbed_ego.yaw-=.2;disturbed_ego.v=0;disturbed_ego.a=0;
     heading_injected=true;
   }
   path={};result={};
   const bool ok=disturbed_committed
     ? disturbed.PlanCommittedAvoid(disturbed_ego,objects,command,path,result)
     : disturbed.PrepareAvoid(disturbed_ego,objects,command,path,result);
   if(!ok||path.x.size()<2){++disturbed_stalls;break;}
   if(!disturbed_committed&&result.candidate_valid){
     disturbed_committed=disturbed.CommitAvoid(result);path=result.candidate_path;
   }
   // The injected yaw can put a previously valid body slightly outside the
   // raster. Require the same bounded recovery contract as the production plan.
   if(!(heading_injected?map.EvaluateRecoveryPath(path,shape,.1).drivable:map.PathClear(path,shape))){++disturbed_stalls;break;}
   if(heading_injected){
     heading_resume_valid=result.candidate_valid&&map.EvaluateRecoveryPath(result.candidate_path,shape,.1).drivable;
     break;
   }
   disturbed_ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
 }
 Expect(heading_injected,"synthetic heading disturbance occurs near bypass offset");
 Expect(heading_resume_valid&&disturbed_stalls==0,"stopped bypass generates a complete road-constrained heading recovery");
 FrenetPlanner junction;
 Expect(junction.Init("avoid_road_constraints_params.yaml"),"junction planner loads");
 auto junction_ego=Ego(fixture["junction"]["ego"]);
 auto junction_objects=Objects(fixture["junction"]["objects"]);
 command.target_object_id=99;command.requested_direction=LateralDirection::RIGHT;
 path={};result={};
 const bool junction_ok=AwaitPrepare(junction,junction_ego,junction_objects,command,path,result);
 Expect(junction_ok&&result.candidate_valid&&result.selected_target_d<0,"recorded junction state has a right bypass");
 Expect(result.candidate_valid&&map.PathClear(result.candidate_path,shape),"right bypass includes a road-constrained return");
 FrenetPlanner recovery_avoid;
 Expect(recovery_avoid.Init("avoid_road_constraints_params.yaml"),"recovery AVOID planner loads");
 double recovery_start_s=0,recovery_end_s=0;
 Expect(recovery_avoid.ProjectEgoToS(junction_ego,recovery_start_s),"recovery start projects");
 PlannerCommand recovery_command=command;
 recovery_command.recovery_clear=true;
 recovery_command.recovery_min_end_s=recovery_start_s+32.0;
 CartesianPath recovery_maintenance;
 FrenetPlanner::AvoidPrepareResult recovery_result;
 const bool recovery_ok=AwaitPrepare(recovery_avoid,junction_ego,junction_objects,
                                     recovery_command,recovery_maintenance,recovery_result);
 const bool recovery_reaches=recovery_result.candidate_valid &&
     recovery_avoid.ProjectPointToS(recovery_result.candidate_path.x.back(),
         recovery_result.candidate_path.y.back(),recovery_command.recovery_min_end_s,
         recovery_end_s) && recovery_end_s>=recovery_command.recovery_min_end_s;
 Expect(recovery_ok&&recovery_reaches,"recovery AVOID reaches the conflict exit");
 Expect(recovery_reaches&&map.EvaluateRecoveryPath(recovery_result.candidate_path,
     shape,recovery_avoid.sample_dt()).drivable,
     "recovery AVOID complete exit remains on road");
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
 SyntheticReplanning(config);
 return failures?1:0;
}

