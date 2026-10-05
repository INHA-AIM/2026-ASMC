// Offline legacy MERGE preview only. No commands or feedback are published.
// variant 0 reproduces planner heading-rate smoothing; variant 1 uses CV prediction.
#include "frenet_planner/Config/ConfigLoader.hpp"
#include "frenet_planner/Maneuver/Merge/MergeGapEvaluator.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Generator/BoundaryConditionPolicy.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"
#include "frenet_planner/Speed/TargetSpeedResolver.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <algorithm>
int main(int argc,char**argv){
 if(argc!=3){std::cerr<<"Usage: roundabout_merge_shadow frames.json resolved_params.yaml\n";return 1;}
 ReferenceConfig rc; ObjectTrackingConfig ot;PathGeneratorConfig pc;KinematicLimits lim;CostWeights cw;VehicleShape shape;CollisionCheckConfig cc;CurveSpeedConfig cs;FollowingConfig fc;AvoidConfig ac;LaneChangeConfig lc;StopConfig sc;EmergencyStopConfig ec;MergeConfig mc;HighwayMergeConfig hc;PlannerVisualizationConfig vc;double wb,lw,period,maxa;std::string wp,err;
 if(!LoadParams(argv[2],rc,ot,pc,lim,cw,shape,cc,cs,fc,ac,lc,sc,ec,mc,hc,vc,wb,lw,period,maxa,wp,err)){std::cerr<<err;return 2;}
 RefLine ref;if(!LoadReferenceLine(wp,ref,rc,lim.max_curvature))return 3;
 struct Track{double heading=0,rate=0;int missing=0;};std::unordered_map<int,Track> tracks;
 bool hashint=false;double hint=0;
 std::cout<<"t,s,v,object_age,stamp_age,variant,confirmed,gap_safe,crossing,entry,clear_duration,front,rear,lon_total,lon_valid,combined,curvature_valid,collision_valid,final_valid,arrival_match,exit_covered,joint_covered,collision_rejected\n";
 for(auto f:YAML::LoadFile(argv[1])){
  auto v=f["ego"];CartesianState ego{};ego.x=v["x"].as<double>();ego.y=v["y"].as<double>();ego.yaw=v["yaw"].as<double>();ego.v=v["speed"].as<double>();ego.a=v["acceleration"].as<double>();
  double s,sd,sdd,d,dp,dpp,dd,ddd;CartesianToFrenet(ref,ego,s,sd,sdd,d,dp,dpp,hashint?&hint:nullptr,rc.ego_projection_search_window);hint=s;hashint=true;ArcDerivToTimeDeriv(sd,sdd,dp,dpp,dd,ddd);auto start=ClampInitialBoundaryState(s,sd,sdd,d,dd,ddd,lim);
  for(auto& p:tracks)++p.second.missing;
  std::vector<TrackedObject> obs;
  for(auto o:f["objects"]){TrackedObject q;q.id=o["id"].as<int>();q.type=o["type"].as<int>();q.motion_state=static_cast<ObjectMotionState>(o["motion_state"].as<int>());q.x=o["center"]["x"].as<double>();q.y=o["center"]["y"].as<double>();q.z=o["center"]["z"].as<double>();q.heading=o["yaw"].as<double>();q.vx=o["vx"].as<double>();q.vy=o["vy"].as<double>();q.speed=std::hypot(q.vx,q.vy);q.length=o["length"].as<double>();q.width=o["width"].as<double>();q.height=o["height"].as<double>();q.geometry_valid=true;q.z_valid=true;for(int i=0;i<8;++i){auto c=o["obb_corners"][i];q.corners[i]={c["x"].as<double>(),c["y"].as<double>(),c["z"].as<double>()};}
   auto it=tracks.find(q.id);if(it==tracks.end())tracks[q.id]={q.heading,0,0};else{double measured=std::remainder(q.heading-it->second.heading,2*M_PI)/period;measured=std::clamp(measured,-ot.yaw_rate_clamp_rad_s,ot.yaw_rate_clamp_rad_s);it->second.rate=(1-ot.yaw_rate_smoothing_alpha)*it->second.rate+ot.yaw_rate_smoothing_alpha*measured;it->second.heading=q.heading;it->second.missing=0;}q.yaw_rate=tracks[q.id].rate;obs.push_back(q);
  }
  for(auto it=tracks.begin();it!=tracks.end();)if(it->second.missing>ot.missing_cycles_eviction)it=tracks.erase(it);else ++it;
  if(s<mc.conflict_s-mc.approach_distance ||s>=mc.conflict_s)continue;
  for(int variant=0;variant<2;++variant){auto objects=obs;if(variant)for(auto&q:objects)q.yaw_rate=0;
   TargetSpeedInput speed_input;speed_input.ego_s=s;speed_input.ego_speed=ego.v;speed_input.has_external_target_speed=true;speed_input.external_target_speed=30.0/3.6;double desired=ResolveTargetSpeed(ref,speed_input,cs,lim);auto gap=FindRoundaboutGap(ref,start,objects,mc,desired,lim.max_longitudinal_accel);
   PlannerCommand cmd;cmd.mode=MERGE;cmd.has_target_speed=true;cmd.target_speed=desired;cmd.merge_gap_safe=gap.safe&&gap.confirmed;cmd.merge_conflict_s=mc.conflict_s;cmd.merge_stop_s=mc.conflict_s-mc.stop_buffer;cmd.merge_entry_time=gap.entry_time;cmd.merge_committed=true;
   PlannerDebugStats stats;auto candidates=ResolveManeuver(start,cmd,ref,pc,lim,lw,objects,shape,cc,sc,mc,&stats);FilterByCartesianLateralAcceleration(candidates,ref,lim.max_lateral_accel);
   int valid=0,match=0,exit=0,joint=0,coll=0;for(auto&p:candidates){if(p.rejection_reason==RejectionReason::COLLISION)++coll;if(!p.valid)continue;++valid;double arrival=-1;for(size_t i=0;i<p.s.size();++i)if(p.s[i]>=mc.conflict_s-1e-3){arrival=p.t[i];break;}if(arrival>=0&&std::abs(arrival-gap.entry_time)<=mc.arrival_time_match_tolerance+1e-6)++match;if(!p.s.empty()&&p.s.back()>=mc.conflict_s+mc.completion_distance&&p.t.back()>=gap.entry_time+gap.clear_time)++exit;if(arrival>=0&&std::abs(arrival-gap.entry_time)<=mc.arrival_time_match_tolerance+1e-6&&!p.s.empty()&&p.s.back()>=mc.conflict_s+mc.completion_distance&&p.t.back()>=gap.entry_time+gap.clear_time)++joint;}
   std::cout<<f["t"].as<double>()<<','<<s<<','<<ego.v<<','<<f["object_age"].as<double>()<<','<<f["object_stamp_age"].as<double>()<<','<<variant<<','<<gap.confirmed<<','<<gap.safe<<','<<gap.crossing_vehicle_count<<','<<gap.entry_time<<','<<gap.clear_time<<','<<gap.preceding_id<<','<<gap.following_id<<','<<stats.longitudinal_total<<','<<stats.longitudinal_valid<<','<<stats.combined_total<<','<<stats.combined_valid_after_curvature<<','<<stats.combined_valid_after_collision<<','<<valid<<','<<match<<','<<exit<<','<<joint<<','<<coll<<'\n';
  }
 }
}
