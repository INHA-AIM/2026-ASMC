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
 const auto dir=std::filesystem::absolute("avoid_side_search_fixture");
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

 auto write_config=[&](double budget){cfg["planner"]["avoid"]["prefer_right_when_equal"]=false;cfg["planner"]["avoid"]["map_search_budget_s"]=budget;std::ofstream f(dir/"params.yaml");f<<cfg;};
 grid();write_config(.08);
 PlannerCommand cmd{};cmd.mode=AVOID;cmd.target_object_id=1;cmd.has_target_speed=true;cmd.target_speed=4;
 auto setup=[&](FrenetPlanner& planner,CartesianState& ego,std::vector<TrackedObject>& objects,PlannerCommand request){
  Expect(planner.Init((dir/"params.yaml").string()),"synthetic side-search road loads");
  ego={};ego.x=50;ego.v=4;objects={box(1,80,0,1)};
  CartesianPath p;FrenetPlanner::AvoidPrepareResult r;
  for(int i=0;i<80;++i){planner.PrepareAvoid(ego,objects,request,p,r);if(r.candidate_valid)break;}
  Expect(r.candidate_valid&&r.selected_target_d>0&&planner.CommitAvoid(r),"left bypass commits on the initially clear road");
 };
 FrenetPlanner planner;CartesianState ego;std::vector<TrackedObject> objects;CartesianPath p;FrenetPlanner::AvoidPrepareResult r;
 setup(planner,ego,objects,cmd);
 planner.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(r.candidate_valid&&r.selected_target_d>0,"a safe committed side remains unchanged");
 objects.push_back(box(2,80,2.5,5));
 planner.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(r.candidate_valid&&r.selected_target_d<0,"a blocked straight approach can select the checked opposite side");
 Expect(planner.avoid_committed(),"side reselection preserves the AVOID transaction");
 Expect(EvaluateCartesianTrajectorySafety(p,objects,VehicleShape{1.9,4.635,.79},CollisionCheckConfig{.45,0,8},.1).clear,
        "the opposite path passes full swept-body collision checks");
 const double accepted=r.selected_target_d;
 planner.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(r.candidate_valid&&std::fabs(r.selected_target_d-accepted)<1e-6,"a safe opposite path does not oscillate back");
 bool completed=false,no_reversal=true;
 for(int i=0;i<600;++i){
  p={};r={};if(!planner.PlanCommittedAvoid(ego,objects,cmd,p,r)||p.x.size()<2)break;
  no_reversal=no_reversal&&(r.selected_target_d<=0||std::string(planner.avoid_phase_name())=="RETURN");
  if(planner.avoid_completed()){completed=true;break;}
  ego={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};
 }
 Expect(completed&&no_reversal,"the reselected bypass completes its return under ideal tracking");
 cfg["planner"]["avoid"]["prefer_right_when_equal"]=true;
 {std::ofstream f(dir/"params.yaml");f<<cfg;}
 FrenetPlanner mirror;mirror.Init((dir/"params.yaml").string());ego={};ego.x=50;ego.v=4;objects={box(1,80,0,1)};
 mirror.PrepareAvoid(ego,objects,cmd,p,r);const bool right_committed=r.candidate_valid&&r.selected_target_d<0&&mirror.CommitAvoid(r);
 objects.push_back(box(2,80,-2.5,5));mirror.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(right_committed&&r.candidate_valid&&r.selected_target_d>0,"opposite-side reselection is symmetric for a blocked right approach");
 write_config(.08);
 auto restricted=[&](PlannerCommand request,bool moved,bool heading,const char* name){
  FrenetPlanner q;setup(q,ego,objects,request);
  if(moved){ego.x=74;ego.y=1;ego.v=2;}
  if(heading)ego.yaw=.15;
  objects.push_back(box(2,80,2.5,5));
  q.PlanCommittedAvoid(ego,objects,request,p,r);
  Expect(!r.candidate_valid||r.selected_target_d>=0,name);
 };
 PlannerCommand left=cmd;left.requested_direction=LateralDirection::LEFT;
 restricted(left,false,false,"explicit LEFT never becomes a right bypass");
 PlannerCommand pose=cmd;pose.has_avoid_target_d=true;pose.avoid_target_d=2.2;
 restricted(pose,false,false,"explicit target pose remains authoritative");
 restricted(cmd,true,false,"measured lateral motion prevents opposite-side reselection");
 restricted(cmd,false,true,"an ongoing heading change prevents opposite-side reselection");
 write_config(.001);
 FrenetPlanner sliced;setup(sliced,ego,objects,cmd);
 objects.push_back(box(3,80,0,20));
 const auto begin=std::chrono::steady_clock::now();
 sliced.PlanCommittedAvoid(ego,objects,cmd,p,r);
 const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
 Expect(!r.candidate_valid&&elapsed<.15,"a failed search yields instead of exhausting thousands of candidates");
 Expect(p.x.size()>1&&p.v.back()==0&&EvaluateCartesianTrajectorySafety(p,objects,VehicleShape{1.9,4.635,.79},CollisionCheckConfig{.45,0,8},.1).clear,
        "an unfinished search returns a fresh checked braking path");
 // Success on the same small slice also requires continuation: repeatedly
 // restarting at the first invalid candidate would never reach this bypass.
 objects={box(1,80,0,1),box(2,80,2.5,5)};bool ready=false;
 for(int i=0;i<300&&!ready;++i){sliced.PlanCommittedAvoid(ego,objects,cmd,p,r);ready=r.candidate_valid;}
 Expect(ready&&r.selected_target_d<0,"unfinished candidate enumeration resumes with fresh safety checks");
 objects.push_back(box(999,ego.x,ego.y,20));
 sliced.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(!r.candidate_valid&&p.x.empty(),"an unsafe braking path is not supplied to the runtime");
 // An uncertain object moving away cannot authorize traversing its
 // present footprint. Confirmed dynamic motion still uses normal prediction.
 CartesianPath crossing;crossing.x={0,5,10};crossing.y={0,0,0};
 crossing.yaw={0,0,0};crossing.kappa={0,0,0};crossing.v={5,5,5};crossing.a={0,0,0};
 auto uncertain=box(7,7,0,1);uncertain.motion_state=ObjectMotionState::UNKNOWN;uncertain.vy=8;
 std::vector<TrackedObject> hypothesis={uncertain};
 const VehicleShape shape{1.9,4.635,.79};const CollisionCheckConfig margin{.45,0,8};
 Expect(CartesianTrajectoryClear(crossing,hypothesis,shape,margin,1),"moving-only prediction vacates the synthetic crossing");
 Expect(!CartesianAvoidTrajectoryClear(crossing,hypothesis,shape,margin,1),"UNKNOWN current occupancy also blocks AVOID");
 hypothesis[0].motion_state=ObjectMotionState::DYNAMIC_PENDING;
 Expect(!CartesianAvoidTrajectoryClear(crossing,hypothesis,shape,margin,1),"pending dynamic motion cannot promise a vacated corridor");
 hypothesis[0].motion_state=ObjectMotionState::DYNAMIC;
 Expect(CartesianAvoidTrajectoryClear(crossing,hypothesis,shape,margin,1),"confirmed dynamic objects retain moving prediction");
 hypothesis[0]=box(7,7,8,1);hypothesis[0].motion_state=ObjectMotionState::UNKNOWN;hypothesis[0].vy=-8;
 Expect(!CartesianAvoidTrajectoryClear(crossing,hypothesis,shape,margin,1),"the moving hypothesis remains checked alongside stationary occupancy");
 // Contract a blocked same-side goal after actual lateral execution starts.
 write_config(.08);FrenetPlanner narrow;narrow.Init((dir/"params.yaml").string());
 ego={};ego.x=50;ego.v=4;objects={box(1,80,0,2)};
 bool wide_ready=false;double wide_goal=0;
 for(int i=0;i<100&&!wide_ready;++i){narrow.PrepareAvoid(ego,objects,cmd,p,r);wide_ready=r.candidate_valid;}
 wide_goal=r.selected_target_d;const bool wide_commit=wide_ready&&narrow.CommitAvoid(r);
 ego.x=70;ego.y=1;ego.v=2;objects={box(1,80,0,1),box(8,84,4.3,1)};
 bool contracted=false;
 for(int i=0;i<100&&!contracted;++i){narrow.PlanCommittedAvoid(ego,objects,cmd,p,r);contracted=r.candidate_valid&&r.selected_target_d>0&&r.selected_target_d<wide_goal-.1;}
 Expect(wide_commit&&contracted,"blocked same-side goal contracts after measured lateral motion");
 Expect(contracted&&CartesianAvoidTrajectoryClear(p,objects,shape,margin,.1),"narrow reconnection passes full-body checks for all objects");
 const double narrow_goal=r.selected_target_d;
 narrow.PlanCommittedAvoid(ego,objects,cmd,p,r);
 Expect(r.candidate_valid&&std::fabs(r.selected_target_d-narrow_goal)<1e-6,"an accepted safe narrower goal remains stable");
 bool narrow_completed=false,narrow_same_side=true;
 for(int i=0;i<600;++i){
  p={};r={};if(!narrow.PlanCommittedAvoid(ego,objects,cmd,p,r)||p.x.size()<2)break;
  narrow_same_side=narrow_same_side&&(r.selected_target_d>=0||std::string(narrow.avoid_phase_name())=="RETURN");
  if(narrow.avoid_completed()){narrow_completed=true;break;}
  ego={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};
 }
 Expect(narrow_completed&&narrow_same_side,"contracted bypass completes return under ideal tracking without reversing side");
 cfg["planner"]["avoid"]["prefer_right_when_equal"]=true;
 {std::ofstream f(dir/"params.yaml");f<<cfg;}
 FrenetPlanner narrow_right;narrow_right.Init((dir/"params.yaml").string());
 ego={};ego.x=50;ego.v=4;objects={box(1,80,0,2)};
 narrow_right.PrepareAvoid(ego,objects,cmd,p,r);const double right_wide=r.selected_target_d;
 const bool right_commit=r.candidate_valid&&narrow_right.CommitAvoid(r);
 ego.x=70;ego.y=-1;ego.v=2;objects={box(1,80,0,1),box(8,84,-4.3,1)};
 bool right_contracted=false;
 for(int i=0;i<100&&!right_contracted;++i){narrow_right.PlanCommittedAvoid(ego,objects,cmd,p,r);right_contracted=r.candidate_valid&&r.selected_target_d<0&&r.selected_target_d>right_wide+.1;}
 Expect(right_commit&&right_contracted,"same-side contraction is symmetric on the right");
 FrenetPlanner fixed;fixed.Init((dir/"params.yaml").string());
 PlannerCommand fixed_cmd=cmd;fixed_cmd.has_avoid_target_d=true;fixed_cmd.avoid_target_d=-2.7;
 ego={};ego.x=50;ego.v=4;objects={box(1,80,0,1)};
 fixed.PrepareAvoid(ego,objects,fixed_cmd,p,r);const bool fixed_commit=r.candidate_valid&&fixed.CommitAvoid(r);
 ego.x=70;ego.y=-1;ego.v=2;objects.push_back(box(8,84,-4.3,1));
 fixed.PlanCommittedAvoid(ego,objects,fixed_cmd,p,r);
 Expect(fixed_commit&&(!r.candidate_valid||std::fabs(r.selected_target_d+2.7)<1e-6),"same-side reconnection preserves explicit target poses");
 std::filesystem::remove_all(dir);return failures?1:0;
}
