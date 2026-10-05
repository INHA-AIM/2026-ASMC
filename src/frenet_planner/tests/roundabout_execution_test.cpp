#include "frenet_planner/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
int main(){
 int failures=0;auto expect=[&](bool ok,const char* name){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';failures+=!ok;};
 const auto root=std::filesystem::path(FRENET_PACKAGE_SRC_DIR).parent_path();
 auto cfg=YAML::LoadFile((root/"config/params.yaml").string());
 cfg["waypoint_file"]=FRENET_TEST_WAYPOINT_FILE;
 cfg["avoid_hd_map"]["directory"]=(root/"config/hd_map").string();
 cfg["roundabout_merge"]["enabled"]=true;
 auto dir=std::filesystem::temp_directory_path()/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
 std::filesystem::create_directories(dir);auto file=dir/"params.yaml";{std::ofstream f(file);f<<cfg;}
 FrenetPlanner planner;bool initialized=planner.Init(file.string());expect(initialized,"map-derived rear-axle waiting pose loads and is drivable");
 if(!initialized){std::filesystem::remove_all(dir);return 1;}
 auto zone=planner.roundabout_zone();expect(zone.conflict_start_s>zone.stop_s&&zone.rear_clear_s>zone.conflict_start_s,"conflict start and rear-clear bounds are separate from stopline");
 CartesianState ego=zone.waiting_pose;
 PlannerCommand cmd;cmd.mode=MERGE;cmd.merge_type=MergeType::ROUNDABOUT;cmd.source_link_id="A2256W000148";cmd.target_link_id="A2256W000146";cmd.has_target_speed=true;cmd.target_speed=25.0/3.6;
 CartesianPath out;FrenetPlanner::MergePrepareResult r;
 auto tick=[&](double time,double observation,const std::vector<TrackedObject>& objects=std::vector<TrackedObject>{}){out={};return planner.PlanRoundaboutMerge(ego,objects,cmd,time,observation,out,r);};
 expect(tick(100,100)&&r.candidate_valid&&!r.ready,"resting ego has full departure candidate and valid hold output");
 tick(100.05,100);expect(!r.ready,"same observation cannot satisfy confirmation count");
 tick(100.1,100.1);tick(100.2,100.2);expect(r.ready&&planner.CanCommitRoundaboutMerge(r),"three fresh safe observations authorize current candidate");
 auto old=r;
 planner.RevokeRoundaboutReadiness();expect(!planner.CanCommitRoundaboutMerge(old),"stale input revokes READY without accepting an old commit");
 tick(100.21,100.21);expect(!r.ready,"fresh input must rebuild confirmations after staleness");
 tick(100.22,100.22);tick(100.23,100.23);
 auto candidate=r.candidate_path;
 if(candidate.x.empty()){std::filesystem::remove_all(dir);return 1;}
 old=r;
 TrackedObject o{};o.id=1;o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;o.length=o.width=2;o.height=1;
 const size_t index=std::min(candidate.x.size()-1,static_cast<size_t>((r.entry_time+1)/planner.sample_dt()));o.x=candidate.x[index];o.y=candidate.y[index];
 for(int i=0;i<8;++i){double dx[4]={-1,1,1,-1},dy[4]={-1,-1,1,1};o.corners[i]={o.x+dx[i%4],o.y+dy[i%4],double(i/4)};}
 tick(100.3,100.3,{o});expect(!r.ready&&!r.candidate_valid&&!planner.CanCommitRoundaboutMerge(old),"new occupying object revokes READY and invalidates old commit token");
 o.id=2;tick(100.4,100.4,{o});expect(!r.ready,"new ID of same occupying object remains blocked");
 planner.ResetRoundaboutMerge();tick(101,101);tick(101.1,101.1);tick(101.2,101.2);expect(r.ready,"fresh empty observations recover from WAITING");
 if(r.ready)planner.ApplyRoundaboutMergeCommit(r);
 expect(planner.roundabout_merge_committed()&&!planner.CanCommitRoundaboutMerge(r),"commit is one-way and cannot be repeated");
 bool complete=false;double t=102;
 for(int i=0;i<200&&planner.roundabout_merge_committed();++i){
  if(!tick(t,t)||out.x.size()<2)break;
  if(r.maneuver_completed){complete=true;break;}
  ego={out.x[1],out.y[1],out.yaw[1],0,out.v[1],out.a[1]};t+=planner.sample_dt();
 }
 expect(complete&&r.progress==1,"ideal tracking clears rear footprint and makes KEEP handoff");
 planner.ResetRoundaboutMerge();expect(!planner.roundabout_merge_committed()&&!planner.CanCommitRoundaboutMerge(old),"transaction reset clears merge commitment and READY token");
 cmd.merge_type=MergeType::HIGHWAY;expect(!tick(200,200)&&planner.last_failure_reason()==FrenetPlanner::PlanFailureReason::INVALID_COMMAND,"roundabout implementation cannot silently execute highway merge");
 std::filesystem::remove_all(dir);return failures?1:0;
}
