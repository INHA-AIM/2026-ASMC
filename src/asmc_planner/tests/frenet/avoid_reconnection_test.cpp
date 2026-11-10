#include "Frenet/frenet_planner.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "avoid_test_braking.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <filesystem>
int failures=0;
void Expect(bool value,const char* name){std::cout<<(value?"[PASS] ":"[FAIL] ")<<name<<'\n';if(!value)++failures;}
int main(){
 auto original=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
 const auto dir=std::filesystem::absolute("avoid_reconnection_fixture");
 std::filesystem::create_directories(dir);
 {std::ofstream f(dir/"waypoints.txt");for(int x=0;x<=200;++x)f<<x<<" 0\n";}
 {std::ofstream f(dir/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-6],[200,-6],[200,6],[0,6]]}]})";}
 {std::ofstream f(dir/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[200,5]]}}]})";}
 auto grid=[&](){std::ofstream f(dir/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int row=0;row<60;++row)f<<row<<' '<<0<<" 1000\n";};
 auto cfg=YAML::Clone(original);cfg["roundabout_merge"]["enabled"]=false;cfg["waypoint_file"]=(dir/"waypoints.txt").string();cfg["avoid_hd_map"]["directory"]=dir.string();
 cfg["planner"]["collision_check"]["margin_growth_rate"]=0.0;
 {std::ofstream f(dir/"params.yaml");f<<cfg;}
 auto box=[](int id,double x,double y,double width){
  TrackedObject o{};o.id=id;o.x=x;o.y=y;o.z=0;o.width=width;o.length=1;o.height=1;o.heading=0;
  o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;
  for(int i=0;i<8;++i){const double xs[4]={-.5,.5,.5,-.5},ys[4]={-.5,-.5,.5,.5};o.corners[i]={x+xs[i%4],y+width*ys[i%4],double(i/4)};}
  return o;
 };

 grid();cfg["planner"]["avoid"]["map_search_budget_s"]=.08;
 {std::ofstream f(dir/"params.yaml");f<<cfg;}
 PlannerCommand cmd{};cmd.mode=AVOID;cmd.target_object_id=1;cmd.has_target_speed=true;cmd.target_speed=4;
 for(int sign:{1,-1}){
  cmd.requested_direction=sign>0?LateralDirection::LEFT:LateralDirection::RIGHT;
  FrenetPlanner planner;Expect(planner.Init((dir/"params.yaml").string()),"shared reconnection synthetic road loads");
  CartesianState ego{};ego.x=50;ego.v=4;std::vector<TrackedObject> objects={box(1,80,0,1)};
  CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
  for(int i=0;i<100;++i){planner.PrepareAvoid(ego,objects,cmd,path,r);if(r.candidate_valid)break;}
  Expect(r.candidate_valid&&planner.CommitAvoid(r),"explicit bypass side commits before return test");
  bool reached=false;
  for(int i=0;i<500;++i){if(!planner.PlanCommittedAvoid(ego,objects,cmd,path,r)||path.x.size()<2)break;
   if(std::string(planner.avoid_phase_name())=="RETURN"){reached=true;break;}
   ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
  }
  Expect(reached,"ideal tracking reaches RETURN");
  objects.push_back(box(2,89,sign*.8,1));
  objects.push_back(box(3,99,-sign*2.5,1));
  bool ready=false;
  bool braking_safe=true;
  for(int i=0;i<100&&!ready;++i){
   planner.PlanCommittedAvoid(ego,objects,cmd,path,r);ready=r.candidate_valid;
   if(!ready)braking_safe=braking_safe&&BrakeAvoidTestVehicle(ego,planner,objects);
  }
  Expect(braking_safe,"late blocker permits collision-free braking before reconnection");
  if(!ready)std::cout<<"reconnection stalled: x="<<ego.x<<" y="<<ego.y<<" yaw="<<ego.yaw<<" k="<<ego.kappa<<" v="<<ego.v<<" a="<<ego.a<<" phase="<<planner.avoid_phase_name()<<" generated="<<r.generated_candidate_count<<" stage="<<r.rejection_stage<<" blocker="<<r.blocking_object_id<<'\n';
  Expect(ready,"RETURN finds a checked route around a downstream obstacle");
  Expect(ready&&std::fabs(r.selected_target_d)>.1,"blocked RETURN selects an intermediate lateral goal");
  Expect(ready&&std::fabs(path.y.back())<.02,"intermediate route still ends on the global path");
  Expect(ready&&CartesianAvoidTrajectoryClear(path,objects,{1.9,4.635,.79},{.45,0,8},.1),"downstream route checks all three obstacle bodies");
  bool completed=false,valid=true;
  for(int i=0;i<1000;++i){if(planner.avoid_completed()){completed=true;break;}if(path.x.size()<2){valid=false;break;}
   ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
   if(!planner.PlanCommittedAvoid(ego,objects,cmd,path,r)){valid=false;break;}
  }
  Expect(valid&&completed,"shared downstream bypass returns under ideal tracking");
 }
 std::filesystem::remove_all(dir);return failures?1:0;
}
