#include "Frenet/frenet_planner.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <filesystem>
int failures=0;
void Expect(bool value,const char* name){std::cout<<(value?"[PASS] ":"[FAIL] ")<<name<<'\n';if(!value)++failures;}
int main(){
 auto original=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
 const auto dir=std::filesystem::absolute("avoid_completion_fixture");
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
 auto setup=[&](FrenetPlanner& planner){
  Expect(planner.Init((dir/"params.yaml").string()),"completion fixture loads");
  CartesianState e{};e.x=50;e.v=4;CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
  for(int i=0;i<100;++i){planner.PrepareAvoid(e,{box(1,80,0,1)},cmd,path,r);if(r.candidate_valid)break;}
  Expect(r.candidate_valid&&planner.CommitAvoid(r),"completion fixture commits primary bypass");
 };
 CartesianState ego{};ego.x=90;ego.v=4;
 for(int motion:{int(ObjectMotionState::LOW_THREAT),int(ObjectMotionState::UNKNOWN),int(ObjectMotionState::DYNAMIC_PENDING)}){
  asmc_planner::execution::ManeuverState execution;
  FrenetPlanner planner;planner.BindExecutionState(execution);setup(planner);CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
  auto next=box(2,180,0,1);next.motion_state=static_cast<ObjectMotionState>(motion);
  next.vy=15;next.speed=15;
  planner.PlanCommittedAvoid(ego,{box(1,80,0,1),next},cmd,path,r);
  Expect(r.candidate_valid,"safe AVOID can remain valid while a later obstacle still needs avoidance");
  Expect(execution.avoid.committed && execution.avoid.completed,"safe finite handoff completes the first transaction");
  Expect(planner.avoid_completed(),"distant next obstacle does not retain the old target lock");
  auto aside=box(3,180,4,1);
  planner.PlanCommittedAvoid(ego,{box(1,80,0,1),aside},cmd,path,r);
  Expect(execution.avoid.completed,"safe KEEP handoff updates the bound Execution owner");
  asmc_planner::execution::ManeuverExecution bound;bound.bind(execution);
  auto snapshot=bound;snapshot.resetAvoid();
  Expect(execution.avoid.completed && planner.avoid_committed(),"preview snapshot reset does not mutate the bound production owner");
  Expect(planner.avoid_completed(),"off-corridor object does not keep a finished avoidance locked");
 }
 {
  asmc_planner::execution::ManeuverState execution;
  FrenetPlanner planner;planner.BindExecutionState(execution);setup(planner);CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
  planner.PlanCommittedAvoid(ego,{box(1,80,0,1),box(2,96,0,1)},cmd,path,r);
  Expect(!planner.avoid_completed(),"next obstacle inside the response-and-stop corridor prevents handoff");
 }
 {
  asmc_planner::execution::ManeuverState execution;
  FrenetPlanner planner;planner.BindExecutionState(execution);setup(planner);CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
  ego.yaw=.2;planner.PlanCommittedAvoid(ego,{box(1,80,0,1)},cmd,path,r);
  Expect(!planner.avoid_completed(),"temporary centerline crossing with unsettled heading stays AVOID");
  ego.yaw=0;ego.y=.1;
  planner.PlanCommittedAvoid(ego,{box(1,80,0,1)},cmd,path,r);
  Expect(planner.avoid_completed(),"small tracking offset completes without chasing the generated return endpoint");
  Expect(planner.last_mode()==AVOID,"KEEP preview does not mutate the executing mode");
  ego.y=0;
 }
 {
  asmc_planner::execution::ManeuverState execution;
  FrenetPlanner planner;planner.BindExecutionState(execution);setup(planner);CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
  ego.x=81;planner.PlanCommittedAvoid(ego,{box(1,80,0,1)},cmd,path,r);
  Expect(!planner.avoid_completed(),"primary body must be fully passed before completion");ego.x=90;
 }
 {
  asmc_planner::execution::ManeuverState execution;
  FrenetPlanner planner;planner.BindExecutionState(execution);setup(planner);CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
  auto crossing=box(2,95,3,1);crossing.motion_state=ObjectMotionState::DYNAMIC;crossing.vy=-1;crossing.speed=1;
  planner.PlanCommittedAvoid(ego,{box(1,80,0,1),crossing},cmd,path,r);
  Expect(!planner.avoid_completed(),"moving collision in the actual KEEP preview blocks handoff");
 }
 std::filesystem::remove_all(dir);return failures?1:0;
}
