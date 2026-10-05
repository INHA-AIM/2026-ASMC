#include "frenet_planner/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
int failures=0;
void Expect(bool ok,const std::string& name){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<"\n"; if(!ok)++failures;}
int main(){
  // Recorded ego states and authoritative OBBs from the stalled AVOID run.
  // This replays planner inputs, not a closed-loop vehicle simulation.
  auto frames=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../tests/data/avoid_tracking_handoff.json");
  auto config=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/params.yaml");
  config["roundabout_merge"]["enabled"]=false; // This test isolates legacy AVOID.
  config["avoid_hd_map"]["enabled"]=false; // Legacy handoff regression; map policy has separate tests.
  config["waypoint_file"]=FRENET_TEST_WAYPOINT_FILE;
  const std::string config_path="avoid_tracking_handoff_test_params.yaml";
  {std::ofstream out(config_path); out<<config;}
  FrenetPlanner planner;
  if(!planner.Init(config_path)) return 2;
  bool committed=false;
  CartesianState initial_ego{};
  std::vector<TrackedObject> initial_objects;
  for(auto frame:frames){
    double t=frame["t"].as<double>(); if(t<169.6)continue;
    auto v=frame["ego"]; CartesianState ego{};
    ego.x=v["x"].as<double>();ego.y=v["y"].as<double>();ego.yaw=v["yaw"].as<double>();ego.v=v["speed"].as<double>();ego.a=v["acceleration"].as<double>();
    std::vector<TrackedObject> objects;
    for(auto o:frame["objects"]){
      TrackedObject obj;obj.id=o["id"].as<int>();obj.type=o["type"].as<int>();obj.motion_state=static_cast<ObjectMotionState>(o["motion_state"].as<int>());
      obj.x=o["center"]["x"].as<double>();obj.y=o["center"]["y"].as<double>();obj.z=o["center"]["z"].as<double>();obj.heading=o["yaw"].as<double>();
      obj.vx=o["vx"].as<double>();obj.vy=o["vy"].as<double>();obj.speed=std::hypot(obj.vx,obj.vy);obj.length=o["length"].as<double>();obj.width=o["width"].as<double>();obj.height=o["height"].as<double>();obj.geometry_valid=true;obj.z_valid=true;
      for(int i=0;i<8;i++){auto c=o["obb_corners"][i];obj.corners[i]={c["x"].as<double>(),c["y"].as<double>(),c["z"].as<double>()};}objects.push_back(obj);
    }
    PlannerCommand cmd;cmd.mode=AVOID;cmd.target_object_id=99;cmd.has_target_speed=true;cmd.target_speed=50./3.6;
    CartesianPath path;FrenetPlanner::AvoidPrepareResult result;
    bool ok=committed?planner.PlanCommittedAvoid(ego,objects,cmd,path,result):planner.PrepareAvoid(ego,objects,cmd,path,result);
    if(!committed && result.candidate_valid){
      Expect(!path.x.empty(),"PREPARE provides maintenance trajectory");
      committed=planner.CommitAvoid(result);
      initial_ego=ego;initial_objects=objects;
      Expect(committed && std::string(planner.avoid_phase_name())=="TRACKING","far low-speed commit keeps TRACKING");
    }
    Expect(ok && !path.x.empty(),"trajectory available at "+std::to_string(t));
    if(t>=172.0){
      Expect(std::string(planner.avoid_phase_name())=="SHIFT","blocked approach enters SHIFT");
      Expect(result.selected_target_d < -0.5,"committed right corridor retained");
      for(std::size_t i=0;i<path.kappa.size();++i){
        if(!std::isfinite(path.kappa[i]) || std::fabs(path.kappa[i])>0.230001){Expect(false,"curvature limit");break;}
        if(i<path.v.size() && path.v[i]*path.v[i]*std::fabs(path.kappa[i])>2.600001){Expect(false,"lateral acceleration limit");break;}
      }
    }
    if(t==179.0){
      ego.v=1.7; ego.a=0.0;
      Expect(planner.PlanCommittedAvoid(ego,objects,cmd,path,result) && result.spatial_candidate_selected,
             "spatial SHIFT survives speed boundary crossing");
      // An additional, very wide physical barrier must invalidate both corridors.
      TrackedObject barrier=objects.back();barrier.id=100;barrier.x=ego.x+4*std::cos(ego.yaw);barrier.y=ego.y+4*std::sin(ego.yaw);
      barrier.length=20;barrier.width=20;barrier.heading=ego.yaw;barrier.vx=barrier.vy=barrier.speed=0;
      for(int i=0;i<8;++i){const double xs[4]={-10,10,10,-10},ys[4]={-10,-10,10,10};double dx=xs[i%4],dy=ys[i%4];barrier.corners[i]={barrier.x+dx,barrier.y+dy,barrier.z+(i<4?-0.2:0.2)};}
      objects.push_back(barrier);
      Expect(!planner.PlanCommittedAvoid(ego,objects,cmd,path,result),"blocked SHIFT remains failed");
      Expect(std::string(planner.avoid_phase_name())=="SHIFT","failed candidate retains committed phase");
      FrenetPlanner blocked_approach;
      Expect(blocked_approach.Init(config_path),"fresh approach initialization");
      FrenetPlanner::AvoidPrepareResult prepared;
      Expect(blocked_approach.PrepareAvoid(initial_ego,initial_objects,cmd,path,prepared) &&
             blocked_approach.CommitAvoid(prepared),"fresh approach committed");
      Expect(!blocked_approach.PlanCommittedAvoid(ego,objects,cmd,path,result),"unrelated barrier blocks approach");
      Expect(std::string(blocked_approach.avoid_phase_name())=="TRACKING","unrelated barrier does not promote SHIFT");
    }
    double s=0;planner.ProjectEgoToS(ego,s);
    std::cout<<"REPLAY "<<t<<" ok="<<ok<<" phase="<<planner.avoid_phase_name()<<" gap="<<planner.avoid_target_s()-s<<" v="<<ego.v<<" d="<<planner.last_d()<<" target="<<result.selected_target_d<<" gen="<<result.generated_candidate_count<<" valid="<<result.valid_candidate_count<<" block="<<result.blocking_object_id<<" fail="<<int(planner.last_failure_reason())<<" n="<<path.x.size()<<" k0="<<(path.kappa.empty()?0:path.kappa[0])<<"\n";
  }
  return failures?1:0;
}
