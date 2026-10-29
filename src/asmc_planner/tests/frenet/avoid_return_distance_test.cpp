#include "Frenet/frenet_planner.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <filesystem>
int failures=0;
void Expect(bool value,const char* name){std::cout<<(value?"[PASS] ":"[FAIL] ")<<name<<'\n';if(!value)++failures;}
int main(){
 auto original=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
 const auto dir=std::filesystem::absolute("avoid_return_fixture");
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
 FrenetPlanner planner;
 Expect(planner.Init((dir/"params.yaml").string()),"synthetic road loads");
 CartesianState ego{};ego.x=65;ego.v=2;
 auto objects=std::vector<TrackedObject>{box(1,80,0,1),box(2,90,-2,1)};
 CartesianPath p;FrenetPlanner::AvoidPrepareResult r;
 planner.PrepareAvoid(ego,objects,cmd,p,r);
 Expect(r.candidate_valid,"short return fits before the downstream obstacle");
 const auto& candidate=r.candidate_path;
 bool centered_before_next=false;
 for(std::size_t i=0;i<candidate.x.size();++i)
  centered_before_next|=candidate.x[i]>82&&candidate.x[i]<90&&std::fabs(candidate.y[i])<.05;
 Expect(centered_before_next,
        "selected bypass returns to the centerline before the second obstacle");
 bool kinematic=!candidate.x.empty();
 const auto limits=cfg["planner"]["kinematic_limits"];
 for(std::size_t i=0;i<candidate.kappa.size();++i){
  kinematic=kinematic&&std::fabs(candidate.kappa[i])<=limits["max_curvature"].as<double>()+1e-6&&
    candidate.v[i]*candidate.v[i]*std::fabs(candidate.kappa[i])<=limits["max_lateral_accel"].as<double>()+1e-6;
 }
 Expect(kinematic,"short return retains curvature and lateral acceleration limits");
 const bool committed=planner.CommitAvoid(r);
 Expect(committed,"short return can be committed");
 bool completed=false;int stalls=0;
 for(int i=0;i<800&&committed;++i){
  p={};r={};
  if(!planner.PlanCommittedAvoid(ego,objects,cmd,p,r)||p.x.size()<2){++stalls;break;}
  if(planner.avoid_completed()){completed=true;break;}
  ego={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};
 }
 Expect(completed&&stalls==0,"short return completes under ideal tracking without candidate loss");
 FrenetPlanner blocked;
 Expect(blocked.Init((dir/"params.yaml").string()),"blocked road loads");
 ego={};ego.x=65;ego.v=2;
 objects.push_back(box(3,84,0,20));p={};r={};
 blocked.PrepareAvoid(ego,objects,cmd,p,r);
 Expect(!r.candidate_valid,"short returns cannot bypass a full-road obstacle");

 // Recorded stationary multi-obstacle geometry: the old pass-end floor sends
 // the return into the downstream barrier. Keep the actual map and OBB corners.
 auto recorded=YAML::Clone(original);
 recorded["waypoint_file"]=FRENET_TEST_WAYPOINT_FILE;
 recorded["avoid_hd_map"]["directory"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/hd_map";
 recorded["roundabout_merge"]["enabled"]=false;
 {std::ofstream f(dir/"recorded.yaml");f<<recorded;}
 std::vector<TrackedObject> recorded_objects;
 {TrackedObject o{};o.id=375;o.type=0;o.x=-62.315461541596257;o.y=-146.31310315575601;o.z=-0.95831350736105791;o.width=0.10509679466485977;o.length=1.117840051651001;o.height=0.29524576663970947;o.heading=0.030125888671314858;o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;
o.corners[0]={-62.873080055814711,-146.3823942736,-1.1039359034057064};
o.corners[1]={-61.755754542718797,-146.34872372981067,-1.1079952051726962};
o.corners[2]={-61.758920044632539,-146.24367460804561,-1.1079347623464202};
o.corners[3]={-62.876245557728453,-146.27734515183496,-1.1038754605794299};
o.corners[4]={-62.872003278396804,-146.38253170289735,-0.80869213229063208};
o.corners[5]={-61.75467776530089,-146.348861159108,-0.81275143405762229};
o.corners[6]={-61.757843267214632,-146.24381203734293,-0.81269099123134581};
o.corners[7]={-62.875168780310545,-146.27748258113229,-0.80863168946435571};
recorded_objects.push_back(o);}
 {TrackedObject o{};o.id=381;o.type=0;o.x=-60.682714890065739;o.y=-142.38559393591945;o.z=-0.79351951122002795;o.width=1.4458953142166138;o.length=1.4489490985870361;o.height=0.92270416021347046;o.heading=0.17279275053375834;o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;
o.corners[0]={-61.273782449384782,-143.2221224236618,-1.2531084542497419};
o.corners[1]={-59.846419258468345,-142.97300008055223,-1.2581982000448622};
o.corners[2]={-60.095012543748446,-141.54863642613591,-1.256628551580288};
o.corners[3]={-61.522375734664884,-141.79775876924549,-1.2515388057851682};
o.corners[4]={-61.270417296974856,-143.22255191867481,-0.33041053046398999};
o.corners[5]={-59.843054106058425,-142.97342957556523,-0.33550027625911016};
o.corners[6]={-60.091647391338526,-141.54906592114892,-0.33393062779453597};
o.corners[7]={-61.519010582254957,-141.7981882642585,-0.3288408819994158};
recorded_objects.push_back(o);}
 {TrackedObject o{};o.id=389;o.type=0;o.x=-57.093738886679319;o.y=-134.74307479651338;o.z=-1.1119619215818353;o.width=0.097037635743618011;o.length=1.7819705009460449;o.height=0.43360710144042969;o.heading=0.012672467468004128;o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;
o.corners[0]={-57.984822620097312,-134.8027788881806,-1.325544875607237};
o.corners[1]={-56.203006953217397,-134.78019766624175,-1.3320327874242159};
o.corners[2]={-56.204236450828859,-134.68316792213986,-1.3319831381978824};
o.corners[3]={-57.986052162052182,-134.70574913841864,-1.3254952262165447};
o.corners[4]={-57.983241231343158,-134.80298072111438,-0.89194070485625565};
o.corners[5]={-56.201425564463243,-134.78039949917553,-0.89842861667323448};
o.corners[6]={-56.202655062074705,-134.68336975507364,-0.89837896744690082};
o.corners[7]={-57.984470773298028,-134.70595097135242,-0.89189105546556346};
recorded_objects.push_back(o);}
 {TrackedObject o{};o.id=407;o.type=0;o.x=-70.418987880965318;o.y=-127.04355263808498;o.z=2.9662750421022706;o.width=0.30152437090873718;o.length=1.4052398204803467;o.height=0.090570211410522461;o.heading=0.012672467468004128;o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;
o.corners[0]={-71.119801758902526,-127.20318599685451,2.9234713675321005};
o.corners[1]={-69.714683794388293,-127.18537895130338,2.9183550798007012};
o.corners[2]={-69.718504558158401,-126.88387901332484,2.9185093558302926};
o.corners[3]={-71.123622522672633,-126.90168605887598,2.9236256435616914};
o.corners[4]={-71.119471444400503,-127.20322815495437,3.0140409667912951};
o.corners[5]={-69.714353479886256,-127.18542110940325,3.0089246790598958};
o.corners[6]={-69.718174243656378,-126.88392117142472,3.0090789550894868};
o.corners[7]={-71.123292208170611,-126.90172821697584,3.014195242820886};
recorded_objects.push_back(o);}

 FrenetPlanner early;
 Expect(early.Init((dir/"recorded.yaml").string()),"recorded early-return road loads");
 ego={};ego.x=-57.98996303626336;ego.y=-146.9062636466697;ego.yaw=1.4438636218198784;
 cmd={};cmd.mode=AVOID;cmd.target_object_id=381;cmd.has_target_speed=true;cmd.target_speed=25.0/3.6;
 p={};r={};
 for(int i=0;i<100&&!r.candidate_valid;++i)early.PrepareAvoid(ego,recorded_objects,cmd,p,r);
 Expect(r.candidate_valid,"early return clears the recorded downstream barrier");
 double minimum=1e9;bool within_limits=!r.candidate_path.x.empty();
 for(std::size_t i=0;i<r.candidate_path.x.size();++i){
  const auto& path=r.candidate_path;
  CartesianState sample{path.x[i],path.y[i],path.yaw[i],path.kappa[i],path.v[i],path.a[i]};
  for(const auto& o:recorded_objects)
   minimum=std::min(minimum,geometry::ConvexPolygonDistance(
     MakeEgoFootprint(sample,VehicleShape{1.9,4.635,.79},0),PredictObstacleFootprint(o,0)));
  within_limits=within_limits&&std::fabs(path.kappa[i])<=limits["max_curvature"].as<double>()+1e-6&&
   path.v[i]*path.v[i]*std::fabs(path.kappa[i])<=limits["max_lateral_accel"].as<double>()+1e-6;
 }
 Expect(minimum>=.45-1e-6,"early return preserves 0.45m physical clearance to every object");
 Expect(within_limits,"early return preserves curvature and lateral acceleration limits");
 const bool early_committed=early.CommitAvoid(r);
 Expect(early_committed,"early return can be committed");
 completed=false;stalls=0;
 for(int i=0;i<800&&early_committed;++i){
  p={};r={};
  if(!early.PlanCommittedAvoid(ego,recorded_objects,cmd,p,r)||p.x.size()<2){++stalls;break;}
  if(early.avoid_completed()){completed=true;break;}
  ego={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};
 }
 Expect(completed&&stalls==0,"recorded early return completes under ideal tracking");

 std::filesystem::remove_all(dir);return failures?1:0;
}
