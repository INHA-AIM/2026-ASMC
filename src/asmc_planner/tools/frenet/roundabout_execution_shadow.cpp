// Offline PREPARE/READY replay. Never publishes or commits a maneuver.
#include "Frenet/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cmath>
#include <algorithm>
int main(int argc,char** argv){
 if(argc<3){std::cerr<<"Usage: roundabout_execution_shadow frames.json resolved_params.yaml [empty]\n";return 1;}
  auto frames=YAML::LoadFile(argv[1]);

  FrenetPlanner planner;if(!planner.Init(argc>2?argv[2]:"speed-fix-build/avoid_road_constraints_params.yaml"))return 2;

  for(auto frame:frames){
    double t=frame["t"].as<double>();
    auto v=frame["ego"]; CartesianState ego{};
    ego.x=v["x"].as<double>();ego.y=v["y"].as<double>();ego.yaw=v["yaw"].as<double>();ego.v=v["speed"].as<double>();ego.a=v["acceleration"].as<double>();
    std::vector<TrackedObject> objects;
    for(auto o:frame["objects"]){
      TrackedObject obj;obj.id=o["id"].as<int>();obj.type=o["type"].as<int>();obj.motion_state=static_cast<ObjectMotionState>(o["motion_state"].as<int>());
      obj.x=o["center"]["x"].as<double>();obj.y=o["center"]["y"].as<double>();obj.z=o["center"]["z"].as<double>();obj.heading=o["yaw"].as<double>();
      obj.vx=o["vx"].as<double>();obj.vy=o["vy"].as<double>();obj.speed=std::hypot(obj.vx,obj.vy);obj.length=o["length"].as<double>();obj.width=o["width"].as<double>();obj.height=o["height"].as<double>();obj.geometry_valid=true;obj.z_valid=true;
      for(int i=0;i<8;i++){auto c=o["obb_corners"][i];obj.corners[i]={c["x"].as<double>(),c["y"].as<double>(),c["z"].as<double>()};}objects.push_back(obj);
    }
    PlannerCommand cmd;cmd.mode=MERGE;cmd.merge_type=MergeType::ROUNDABOUT;
    cmd.source_link_id="A2256W000148";cmd.target_link_id="A2256W000146";
    cmd.has_target_speed=true;cmd.target_speed=25.0/3.6;
    double s=0;planner.ProjectEgoToS(ego,s);
    if(s<planner.roundabout_zone().stop_s-45 || s>planner.roundabout_zone().stop_s+.5)continue;
    if(argc>3)objects.clear();
    CartesianPath path;FrenetPlanner::MergePrepareResult result;
    double stamp=v["header"]["stamp"].as<double>();
    bool ok=planner.PlanRoundaboutMerge(ego,objects,cmd,stamp,
        stamp-frame["object_stamp_age"].as<double>(),path,result);
    std::cout<<"AUDIT "<<t<<" s="<<s<<" v="<<ego.v<<" maintenance="<<ok<<" candidate="<<result.candidate_valid<<" ready="<<result.ready<<" valid="<<result.valid<<" entry="<<result.entry_time<<" exit="<<result.exit_time<<" blocker="<<result.blocking_object_id<<"\n";
  }
}
