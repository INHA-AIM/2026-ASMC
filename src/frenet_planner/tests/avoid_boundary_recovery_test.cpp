#include "frenet_planner/Map/AvoidRoadMap.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
int failures=0;
void Expect(bool value,const char* message){std::cout<<(value?"[PASS] ":"[FAIL] ")<<message<<'\n';if(!value)++failures;}
CartesianPath Line(double x0,double y0,double x1,double y1,int count=21){
 CartesianPath p;for(int i=0;i<count;++i){double t=double(i)/(count-1);p.x.push_back(x0+t*(x1-x0));p.y.push_back(y0+t*(y1-y0));p.yaw.push_back(0);}return p;
}
int main(){
 // Synthetic straight road: original free space is x=[0,20), y=[-2,2).
 // No real driving log is needed to verify bounded initial recovery.
 const std::string dir="avoid_boundary_recovery_fixture";std::filesystem::create_directories(dir);
 {std::ofstream f(dir+"/drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-2},"drivable_areas":[{"validity":"validated","polygon":[[0,-2],[20,-2],[20,2],[0,2]]}]})";}
 {std::ofstream f(dir+"/lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,20],[20,20]]}}]})";}
 {std::ofstream f(dir+"/drivable_grid.rle");f<<"3085 7232 0.2 0 -2\n";for(int row=0;row<20;++row)f<<row<<" 0 100\n";}
 AvoidRoadMap map;std::string error;Expect(map.Load(dir+"/drivable.json",dir+"/lines.json",error),"synthetic original map loads");
 VehicleShape shape{1.9,4.635,.79};auto recovery=Line(2,-1.10,4,0);
 Expect(!map.EvaluatePath(recovery,shape,.1).drivable,"strict map still rejects initial overlap");
 auto accepted=map.EvaluateRecoveryPath(recovery,shape,.1);
 Expect(accepted.drivable&&accepted.initial_overlap_m>0&&accepted.recovery_seconds>0,"small initial overlap recovers into unchanged map");
 Expect(!map.EvaluateRecoveryPath(Line(2,-1.10,4,-1.10),shape,.1).drivable,"outside hold cannot masquerade as recovery");
 Expect(!map.EvaluateRecoveryPath(Line(2,-1.10,4,-1.25),shape,.1).drivable,"increasing departure rejected");
 Expect(!map.EvaluateRecoveryPath(Line(2,-1.50,4,0),shape,.1).drivable,"large initial departure rejected");
 Expect(!map.EvaluateRecoveryPath(Line(2,0,4,-1.10),shape,.1).drivable,"new departure from inside remains rejected");
 auto near_edge=Line(2,-1.04,2.5,-1.10);auto finish=Line(2.5,-1.10,4,0);
 near_edge.x.insert(near_edge.x.end(),finish.x.begin(),finish.x.end());near_edge.y.insert(near_edge.y.end(),finish.y.begin(),finish.y.end());near_edge.yaw.insert(near_edge.yaw.end(),finish.yaw.begin(),finish.yaw.end());
 Expect(!map.EvaluateRecoveryPath(near_edge,shape,.1).drivable,"inside start cannot initiate a fresh boundary escape");
 Expect(map.EvaluateRecoveryPath(near_edge,shape,.1,6,.10,true).drivable,"ongoing recovery survives near-edge grid reentry without renewing limits");
 auto leave_again=Line(2,-1.10,4,0);auto tail=Line(4,0,6,-1.10);leave_again.x.insert(leave_again.x.end(),tail.x.begin(),tail.x.end());leave_again.y.insert(leave_again.y.end(),tail.y.begin(),tail.y.end());leave_again.yaw.insert(leave_again.yaw.end(),tail.yaw.begin(),tail.yaw.end());
 Expect(!map.EvaluateRecoveryPath(leave_again,shape,.1).drivable,"departure after settled recovery rejected");
 Expect(!map.EvaluateRecoveryPath(recovery,shape,.1,.02).drivable,"remaining travel budget enforced");
 Expect(!map.EvaluateRecoveryPath(recovery,shape,.1,6,.01).drivable,"episode overlap limit cannot be renewed each plan");
 Expect(!map.EvaluateRecoveryPath(Line(2,-1.10,4,0,201),shape,1.0).drivable,"slow recovery exceeds predicted five-second budget");
 recovery.x[0]=std::numeric_limits<double>::quiet_NaN();Expect(!map.EvaluateRecoveryPath(recovery,shape,.1).drivable,"nonfinite initial pose rejected");


 std::filesystem::remove_all(dir);return failures?1:0;
}
