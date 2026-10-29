#include "Frenet/frenet_planner.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
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
 const auto dir=std::filesystem::absolute("avoid_return_progress_fixture");
 std::filesystem::create_directories(dir);
 {std::ofstream f(dir/"waypoints.txt");for(int x=0;x<=200;++x)f<<x<<" 0\n";}
 {std::ofstream f(dir/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-4],[200,-4],[200,4],[0,4]]}]})";}
 {std::ofstream f(dir/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[200,5]]}}]})";}
 auto grid=[&](){std::ofstream f(dir/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int row=10;row<50;++row)f<<row<<' '<<0<<" 1000\n";};
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



 grid();
 auto prepare_pass=[&](FrenetPlanner& planner,CartesianState& ego,std::vector<TrackedObject>& objects){
  Expect(planner.Init((dir/"params.yaml").string()),"synthetic return-progress road loads");
  ego={};ego.x=65;ego.v=2;objects={box(1,80,0,1)};
  CartesianPath p;FrenetPlanner::AvoidPrepareResult r;
  planner.PrepareAvoid(ego,objects,cmd,p,r);
  Expect(r.candidate_valid&&planner.CommitAvoid(r),"initial bypass commits before the downstream blocker appears");
  const double target=r.selected_target_d;
  planner.PlanCommittedAvoid(ego,objects,cmd,p,r);
  if(p.x.size()>3){
   const auto c=p;
   auto mid=[](double a,double b){return .5*(a+b);};
   ego={mid(c.x[1],c.x[2]),mid(c.y[1],c.y[2]),mid(c.yaw[1],c.yaw[2]),
        mid(c.kappa[1],c.kappa[2]),mid(c.v[1],c.v[2]),mid(c.a[1],c.a[2])};
   planner.PlanCommittedAvoid(ego,objects,cmd,p,r);
   Expect(r.candidate_valid&&r.generated_candidate_count==1&&r.selected_cost<1.0,
          "segment projection retains the cached suffix between temporal samples");
  }
  ego.x=79;ego.y=target;
  planner.PlanCommittedAvoid(ego,objects,cmd,p,r);
  Expect(std::string(planner.avoid_phase_name())=="PASS","synthetic vehicle reaches the bypass offset before the target rear edge");
  objects.push_back(box(2,87,-2,1));ego.yaw=.05;
 };
 FrenetPlanner planner;CartesianState ego{};std::vector<TrackedObject> objects;
 prepare_pass(planner,ego,objects);
 CartesianPath p;FrenetPlanner::AvoidPrepareResult r;
 bool braking_safe=true;
 for(int i=0;i<100&&!r.candidate_valid;++i) {
  planner.PlanCommittedAvoid(ego,objects,cmd,p,r);
  if(!r.candidate_valid)braking_safe=braking_safe&&BrakeAvoidTestVehicle(ego,planner,objects);
 }
 Expect(braking_safe,"new blocker permits safe braking before a different return");
 if(!r.candidate_valid)std::cout<<"return stalled: x="<<ego.x<<" y="<<ego.y<<" yaw="<<ego.yaw<<" k="<<ego.kappa<<" v="<<ego.v<<" a="<<ego.a<<" stage="<<r.rejection_stage<<" blocker="<<r.blocking_object_id<<'\n';
 Expect(r.candidate_valid,"measured-pose return reconnects around the new downstream blocker");
 Expect(std::string(planner.avoid_phase_name())=="RETURN","a validated direct return enters RETURN immediately");
 double clearance=1e9;
 for(std::size_t i=0;i<p.x.size();++i){
  CartesianState e{p.x[i],p.y[i],p.yaw[i],p.kappa[i],p.v[i],p.a[i]};
  for(const auto& o:objects)clearance=std::min(clearance,geometry::ConvexPolygonDistance(
   MakeEgoFootprint(e,VehicleShape{1.9,4.635,.79},0),PredictObstacleFootprint(o,0)));
 }
 Expect(r.candidate_valid&&clearance>=.45-1e-6,"direct return preserves physical clearance to both obstacles");
 bool completed=false,stayed_return=true;int stalls=0;
 for(int i=0;i<600;++i){
  p={};r={};
  if(!planner.PlanCommittedAvoid(ego,objects,cmd,p,r)||p.x.size()<2){++stalls;break;}
  if(planner.avoid_completed()){completed=true;break;}
  stayed_return=stayed_return&&std::string(planner.avoid_phase_name())=="RETURN";
  ego={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};
 }
 Expect(stayed_return,"subsequent replans do not reset the accepted return to PASS or SHIFT");
 Expect(completed&&stalls==0,"measured-pose return completes under ideal tracking");
 FrenetPlanner blocked;prepare_pass(blocked,ego,objects);
 blocked.PlanCommittedAvoid(ego,objects,cmd,p,r);
 objects.push_back(box(999,ego.x,ego.y,20));p={};r={};
 blocked.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(!r.candidate_valid,"cached continuation and direct return reject a fresh physical blocker");
 std::filesystem::remove_all(dir);return failures?1:0;
}
