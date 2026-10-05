#include "frenet_planner/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <algorithm>
#include "frenet_planner/Maneuver/ManeuverUtils.hpp"
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

 auto ref_cfg=cfg["planner"]["reference"];
 ReferenceConfig rc{ref_cfg["closure_tolerance_m"].as<double>(),ref_cfg["smoothing_window_m"].as<double>(),ref_cfg["resample_ds"].as<double>(),ref_cfg["relax_max_iter"].as<int>(),ref_cfg["relax_step"].as<double>(),ref_cfg["ego_projection_search_window"].as<double>()};
 RefLine ref;expect(LoadReferenceLine(FRENET_TEST_WAYPOINT_FILE,ref,rc,cfg["planner"]["kinematic_limits"]["max_curvature"].as<double>()),"public map reference loads for boundary regressions");
 auto pose_at=[&](double s){auto p=Interpolate(ref,s);return CartesianState{p.x,p.y,p.theta,0,0,0};};
 ego=zone.waiting_pose;ego.v=.1;ego.a=-3;
 planner.ResetRoundaboutMerge();expect(tick(300,300)&&r.candidate_valid,"low-speed braking boundary can reach zero and depart without reverse rejection");
 ego=pose_at(zone.stop_s-6);planner.ResetRoundaboutMerge();tick(301,301);tick(301.1,301.1);tick(301.2,301.2);
 expect(r.candidate_valid&&!r.ready,"safe distant candidate cannot commit six metres before the stopline");
 ego=pose_at(zone.rear_clear_s-6);ego.v=5;ego.a=2.5;
 planner.ResetRoundaboutMerge();planner.ApplyRoundaboutMergeCommit({});
 expect(tick(302,302)&&r.candidate_valid,"positive acceleration entering exit curve does not force an empty trajectory");
 bool lateral_safe=!out.v.empty();for(size_t i=0;i<out.v.size();++i)lateral_safe&=out.v[i]*out.v[i]*std::fabs(out.kappa[i])<=cfg["planner"]["kinematic_limits"]["max_lateral_accel"].as<double>()+1e-6;
 expect(lateral_safe,"rescued exit candidate keeps the physical lateral acceleration limit");
 AvoidRoadMap road;std::string error;VehicleShape shape{1.9,4.635,.79};
 expect(road.Load((root/"config/hd_map/drivable.json").string(),(root/"config/hd_map/lines.json").string(),error),"original road grid loads without expansion");
 auto centre=pose_at(zone.stop_s-15);CartesianPath bridge;
 bool found=false;
 for(double d=.1;d<2&&!found;d+=.025)for(double side:{-1.0,1.0}){
  auto edge=centre;edge.x-=std::sin(centre.yaw)*d*side;edge.y+=std::cos(centre.yaw)*d*side;
  double overlap=road.FootprintOverlapDepth(edge.x,edge.y,edge.yaw,shape);
  if(overlap>0&&overlap<.1){
   for(int i=0;i<=20;++i){double f=double(i)/20;bridge.x.push_back(edge.x+(centre.x-edge.x)*f);bridge.y.push_back(edge.y+(centre.y-edge.y)*f);bridge.yaw.push_back(centre.yaw);}found=true;break;
  }
 }
 expect(found&&!road.EvaluatePath(bridge,shape,.1).drivable&&road.EvaluateMergeBoundaryBridge(bridge,shape,.1,6,5).drivable,"small map boundary contact can return to original grid within bounded bridge");
 auto deep=bridge;if(!deep.x.empty()){double dx=deep.x.front()-centre.x,dy=deep.y.front()-centre.y,len=std::hypot(dx,dy);deep.x.front()+=.5*dx/len;deep.y.front()+=.5*dy/len;}
 expect(!road.EvaluateMergeBoundaryBridge(deep,shape,.1,6,5).drivable,"bridge cannot excuse a substantial road departure");
 auto no_return=bridge;if(!no_return.x.empty()){std::reverse(no_return.x.begin(),no_return.x.end());std::reverse(no_return.y.begin(),no_return.y.end());}
 expect(!road.EvaluateMergeBoundaryBridge(no_return,shape,.1,6,5).drivable,"bridge must settle inside the original road rather than end outside");
 expect(!road.EvaluateMergeBoundaryBridge(bridge,shape,.1,.01,5).drivable,"bridge cannot renew an exhausted distance budget");
 expect(!road.EvaluateMergeBoundaryBridge(bridge,shape,.1,6,.01).drivable,"bridge cannot renew an exhausted time budget");

 // Synthetic offsets from the public map, independent of recorded vehicle poses.
 bool waiting_recovered=false;
 CartesianState displaced{};
 for(double offset=.4;offset<=1.2&&!waiting_recovered;offset+=.025){
  ego=pose_at(zone.stop_s);ego.x+=std::sin(ego.yaw)*offset;ego.y-=std::cos(ego.yaw)*offset;
  const double overlap=road.FootprintOverlapDepth(ego.x,ego.y,ego.yaw,shape);
  if(overlap<=0||overlap>.13)continue;
  planner.ResetRoundaboutMerge();tick(400,400);tick(400.1,400.1);tick(400.2,400.2);
  if(r.ready){waiting_recovered=true;displaced=ego;}
 }
 expect(waiting_recovered&&r.candidate_valid&&planner.CanCommitRoundaboutMerge(r),"slightly displaced waiting pose can certify bounded departure before commit");
 expect(!planner.roundabout_merge_committed()&&!out.v.empty()&&out.v.back()==0,"boundary recovery certificate keeps PREPARE output stopped until commit");
 if(waiting_recovered){
  auto departure=r.candidate_path;old=r;
  auto blocker=o;blocker.id=42;
  const size_t j=std::min(departure.x.size()-1,static_cast<size_t>((r.entry_time+1)/planner.sample_dt()));
  blocker.x=departure.x[j];blocker.y=departure.y[j];
  for(int i=0;i<8;++i){double dx[4]={-1,1,1,-1},dy[4]={-1,-1,1,1};blocker.corners[i]={blocker.x+dx[i%4],blocker.y+dy[i%4],double(i/4)};}
  tick(400.3,400.3,{blocker});
  expect(!r.ready&&!r.candidate_valid&&!planner.CanCommitRoundaboutMerge(old),"pre-commit boundary recovery still rejects an occupying vehicle and stale token");
  ego=displaced;ego.x+=std::sin(ego.yaw)*.5;ego.y-=std::cos(ego.yaw)*.5;
  planner.ResetRoundaboutMerge();tick(401,401);
  expect(!r.candidate_valid&&!r.ready,"pre-commit recovery cannot excuse substantial road departure");
 }
 // The waiting STOP uses a centre target, rather than ordinary STOP's lateral
 // search or low-speed hold. Check the actual output, not just a request flag.
 ego=pose_at(zone.stop_s-20);ego.x+=std::sin(ego.yaw)*.6;ego.y-=std::cos(ego.yaw)*.6;ego.v=5;ego.a=-1;
 FrenetPlanner approach_planner;bool approaching=approach_planner.Init(file.string())&&approach_planner.PlanRoundaboutMerge(ego,{},cmd,402,402,out,r);
 double terminal_d=1e9;
 if(!out.x.empty()){auto rp=Interpolate(ref,FindClosestS(ref,out.x.back(),out.y.back(),nullptr,1e6));terminal_d=(out.y.back()-rp.y)*std::cos(rp.theta)-(out.x.back()-rp.x)*std::sin(rp.theta);}
 expect(approaching&&out.x.size()>1&&std::fabs(terminal_d)<.05,"roundabout braking approach terminates on the reference centre rather than a side offset");
 ego=pose_at(zone.stop_s-37);ego.x+=std::sin(ego.yaw)*.4;ego.y-=std::cos(ego.yaw)*.4;ego.yaw+=.01;ego.v=11;ego.a=-1.4;
 FrenetPlanner fallback_planner;
 bool fallback_ok=fallback_planner.Init(file.string())&&fallback_planner.PlanRoundaboutMerge(ego,{},cmd,403,403,out,r);
 terminal_d=1e9;
 if(!out.x.empty()){auto rp=Interpolate(ref,FindClosestS(ref,out.x.back(),out.y.back(),nullptr,1e6));terminal_d=(out.y.back()-rp.y)*std::cos(rp.theta)-(out.x.back()-rp.x)*std::sin(rp.theta);}
 expect(fallback_ok&&out.x.size()>1&&std::fabs(terminal_d)<.05&&out.v.back()==0,"long waiting braking fallback recentres instead of freezing lateral error");
 expect(fallback_ok&&!out.yaw.empty()&&std::fabs(std::remainder(out.yaw.front()-ego.yaw,2*M_PI))<.01,"waiting fallback centering preserves observed initial heading");

 // Heading alignment must keep the selected rear-axle endpoint and preserve
 // the observed starting direction. Compare with the same STOP request.
 ego=pose_at(zone.stop_s-7);ego.v=5;ego.yaw+=.08;ego.a=-1;
 FrenetPlanner plain_stop,heading_stop;
 bool stop_init=plain_stop.Init(file.string())&&heading_stop.Init(file.string());
 auto waiting_cmd=cmd;waiting_cmd.mode=STOP;waiting_cmd.stop_position=zone.stop_s;
 CartesianPath plain,aligned;
 bool plain_ok=stop_init&&plain_stop.Plan(ego,{},waiting_cmd,plain);
 bool aligned_ok=stop_init&&heading_stop.PlanRoundaboutMerge(ego,{},cmd,404,404,aligned,r);
 expect(plain_ok&&aligned_ok&&!plain.x.empty()&&!aligned.x.empty()&&
  std::hypot(zone.waiting_pose.x-aligned.x.back(),zone.waiting_pose.y-aligned.y.back())<.001,
  "waiting heading alignment preserves the selected rear-axle stop position");
 expect(aligned_ok&&aligned.x.size()>2&&
  std::fabs(std::remainder(aligned.yaw.front()-ego.yaw,2*M_PI))<1e-6,
  "waiting heading profile starts at observed yaw without an instantaneous turn");
 expect(aligned_ok&&!aligned.x.empty()&&road.PoseClear(aligned.x.back(),aligned.y.back(),aligned.yaw.back(),shape)&&
  road.EvaluatePath(aligned,shape,.1).drivable,
  "waiting heading checks the complete body and the whole approach against the map");
 bool waiting_bounds=!aligned.v.empty();
 for(size_t i=0;i<aligned.v.size();++i)waiting_bounds&=aligned.v[i]>=-1e-6&&
  aligned.v[i]<=std::max(cmd.target_speed,ego.v)+.05&&
  aligned.a[i]<=cfg["planner"]["kinematic_limits"]["max_longitudinal_accel"].as<double>()&&
  aligned.a[i]>=-cfg["planner"]["stop"]["max_service_decel"].as<double>();
 expect(aligned_ok&&waiting_bounds&&aligned.v.back()<.001,
  "frontier approach preserves longitudinal bounds and ends stopped at the limit");
 ego=pose_at(zone.stop_s);ego.yaw+=.05;
 heading_stop.ResetRoundaboutMerge();aligned={};
 bool hold_ok=heading_stop.PlanRoundaboutMerge(ego,{},cmd,405,405,aligned,r);
 expect(hold_ok&&aligned.x.size()==1&&std::fabs(std::remainder(aligned.yaw.front()-ego.yaw,2*M_PI))<1e-6,
  "stationary waiting keeps observed yaw instead of rotating in place");

 // PREPARE never crosses the rear-axle frontier, including an approach at
 // cruise speed. A complete safe traversal can authorize a rolling departure.
 ego=pose_at(zone.stop_s-12);ego.v=5;ego.a=0;
 FrenetPlanner frontier_planner;bool frontier_ok=frontier_planner.Init(file.string());
 CartesianPath frontier;
 frontier_ok=frontier_ok&&frontier_planner.PlanRoundaboutMerge(ego,{},cmd,406,406,frontier,r);
 bool behind_line=frontier_ok&&!frontier.x.empty();double hint=zone.stop_s-12;
 for(size_t i=0;i<frontier.x.size();++i){hint=FindClosestS(ref,frontier.x[i],frontier.y[i],&hint,5);behind_line&=hint<=zone.stop_s+.02;}
 expect(behind_line&&frontier.v.back()==0,"PREPARE approach stays behind the stopline and includes a complete stop");
 ego=pose_at(zone.stop_s-3);ego.v=2;ego.a=0;
 FrenetPlanner rolling_entry;bool rolling_ready=rolling_entry.Init(file.string());
 for(int i=0;i<3&&rolling_ready;++i)rolling_ready=rolling_entry.PlanRoundaboutMerge(ego,{},cmd,407+i*.1,407+i*.1,frontier,r);
 expect(rolling_ready&&r.ready&&rolling_entry.CanCommitRoundaboutMerge(r)&&ego.v>0,
  "full fresh clearance authorizes departure without requiring zero measured speed");
 auto early_token=r;auto early_candidate=r.candidate_path;
 // Empty traffic does not justify committing while a strong measured brake
 // is still changing the planned arrival schedule.
 FrenetPlanner braking_entry;bool braking_ready=braking_entry.Init(file.string());
 ego.a=-2;
 for(int i=0;i<3&&braking_ready;++i)braking_ready=braking_entry.PlanRoundaboutMerge(ego,{},cmd,407+i*.1,407+i*.1,frontier,r);
 expect(braking_ready&&!r.ready,"strong braking approach retains its stop option instead of authorizing rolling entry");
 ego.a=0;
 // A partial moving vehicle close to the conflict is uncertain even when its
 // instantaneous small box is outside the candidate footprint.
 FrenetPlanner uncertain_entry;bool uncertain_ok=uncertain_entry.Init(file.string());
 auto fragment=o;fragment.id=901;fragment.length=.8;fragment.width=.4;
 fragment.motion_state=ObjectMotionState::DYNAMIC;fragment.vx=0;fragment.vy=3;
 const auto conflict_pose=pose_at(zone.conflict_start_s);
 fragment.x=conflict_pose.x-15;fragment.y=conflict_pose.y;
 for(int i=0;i<8;++i)fragment.corners[i]={fragment.x+(i%4<2?-.4:.4),fragment.y+(i%2?-.2:.2),double(i/4)};
 for(int i=0;i<4&&uncertain_ok;++i)uncertain_ok=uncertain_entry.PlanRoundaboutMerge(ego,{fragment},cmd,410+i*.1,410+i*.1,frontier,r);
 expect(uncertain_ok&&r.candidate_valid&&!r.ready&&r.blocking_object_id==901,"nearby partial moving vehicle prevents a departure certificate");
 ego.v=0;
 uncertain_ok=uncertain_ok&&uncertain_entry.PlanRoundaboutMerge(ego,{fragment},cmd,410.5,410.5,frontier,r);
 expect(uncertain_ok&&r.candidate_valid&&!r.ready,"having stopped once does not bypass uncertain nearby vehicle checks");
 ego.v=2;
 for(int i=0;i<3&&uncertain_ok;++i)uncertain_ok=uncertain_entry.PlanRoundaboutMerge(ego,{},cmd,412+i*.1,412+i*.1,frontier,r);
 expect(uncertain_ok&&r.ready,"fresh clearance recovers departure after uncertain traffic has cleared");
 FrenetPlanner stale_entry;bool stale_ok=stale_entry.Init(file.string());
 for(int i=0;i<3&&stale_ok;++i)stale_ok=stale_entry.PlanRoundaboutMerge(ego,{},cmd,413+i*.1,412+i*.1,frontier,r);
 expect(stale_ok&&!r.ready,"fresh confirmation numbers cannot authorize stale sensor observations");
 auto crossing=o;crossing.id=88;
 if(!early_candidate.x.empty()){
  const size_t j=std::min(early_candidate.x.size()-1,static_cast<size_t>((r.entry_time+1)/.1));
  crossing.x=early_candidate.x[j];crossing.y=early_candidate.y[j];
  for(int i=0;i<8;++i){double dx[4]={-1,1,1,-1},dy[4]={-1,-1,1,1};crossing.corners[i]={crossing.x+dx[i%4],crossing.y+dy[i%4],double(i/4)};}
 }
 rolling_entry.PlanRoundaboutMerge(ego,{crossing},cmd,407.3,407.3,frontier,r);
 expect(!r.ready&&!rolling_entry.CanCommitRoundaboutMerge(early_token),
  "new traffic revokes an earlier rolling-departure certificate");

 // Once stopped upstream with traffic blocking the complete departure,
 // remain at that measured pose instead of creeping to a second stop target.
 ego=pose_at(zone.stop_s-4);ego.v=0;ego.a=0;
 FrenetPlanner latched_wait;bool latch_ok=latched_wait.Init(file.string());
 for(int i=0;i<3&&latch_ok;++i)latch_ok=latched_wait.PlanRoundaboutMerge(ego,{crossing},cmd,408+i*.1,408+i*.1,frontier,r);
 expect(latch_ok&&!r.ready&&frontier.x.size()==1&&std::hypot(frontier.x[0]-ego.x,frontier.y[0]-ego.y)<1e-6,
  "a blocked vehicle which already stopped upstream holds that pose until departure is safe");
 for(int i=0;i<3&&latch_ok;++i)latch_ok=latched_wait.PlanRoundaboutMerge(ego,{},cmd,408.4+i*.1,408.4+i*.1,frontier,r);
 expect(latch_ok&&r.ready&&latched_wait.CanCommitRoundaboutMerge(r)&&frontier.x.size()==1,
  "fresh full clearance releases upstream waiting through the commit handshake");

 // A vehicle which already waited upstream can execute its certified
 // departure directly. Ideal prefix tracking must not re-enter STOP at the line.
 ego=pose_at(zone.stop_s-4);ego.v=0;ego.a=0;
 FrenetPlanner once_waited;bool once_ok=once_waited.Init(file.string());
 for(int i=0;i<3&&once_ok;++i)once_ok=once_waited.PlanRoundaboutMerge(ego,{},cmd,408+i*.1,408+i*.1,frontier,r);
 bool once_done=false,second_wait=false;
 if(once_ok&&r.ready)once_waited.ApplyRoundaboutMergeCommit(r);else once_ok=false;
 for(int i=0;i<200&&once_ok;++i){
  frontier={};once_ok=once_waited.PlanRoundaboutMerge(ego,{},cmd,409+i*.1,409+i*.1,frontier,r);
  if(r.maneuver_completed){once_done=true;break;}
  if(!once_ok||frontier.x.size()<2){once_ok=false;break;}
  second_wait|=i>5&&frontier.v[1]<.01;
  ego={frontier.x[1],frontier.y[1],frontier.yaw[1],frontier.kappa[1],frontier.v[1],frontier.a[1]};
 }
 expect(once_ok&&once_done&&!second_wait,
  "an earlier safe waiting pose departs and reaches KEEP without a second mandatory stop");

 // Execute the generated prefix repeatedly, rather than treating a fixed
 // historical overspeed pose as the result of the corrected controller.
 FrenetPlanner curve_planner;bool rolling=curve_planner.Init(file.string());
 ego=pose_at(zone.rear_clear_s-15);ego.x+=std::sin(ego.yaw)*.2;ego.y-=std::cos(ego.yaw)*.2;ego.yaw+=.05;ego.v=25.0/3.6;ego.a=.4;
 curve_planner.ApplyRoundaboutMergeCommit({});
 bool early_braking=false,rolling_limits=true,handed_off=false;
 for(int i=0;i<250&&rolling;++i){
  out={};rolling=curve_planner.PlanRoundaboutMerge(ego,{},cmd,500+i*.1,500+i*.1,out,r);
  if(i==0)early_braking=out.v.size()>1&&out.a[1]<ego.a;
  if(r.maneuver_completed){handed_off=true;break;}
  if(!rolling||out.x.size()<2){rolling=false;break;}
  for(size_t j=0;j<out.v.size();++j)rolling_limits&=out.v[j]*out.v[j]*std::fabs(out.kappa[j])<=cfg["planner"]["kinematic_limits"]["max_lateral_accel"].as<double>()+1e-6;
  ego={out.x[1],out.y[1],out.yaw[1],out.kappa[1],out.v[1],out.a[1]};
 }
 expect(early_braking,"actual exit geometry starts reducing acceleration before the bend");
 expect(rolling&&rolling_limits&&handed_off,"receding-horizon exit traversal keeps steering paths and reaches KEEP handoff");
 FrenetPlanner brake_planner;bool brake_init=brake_planner.Init(file.string());
 ego=pose_at(zone.conflict_start_s+6);ego.v=2;ego.a=-4;
 brake_planner.ApplyRoundaboutMergeCommit({});
 auto occupying=o;occupying.id=77;auto future=pose_at(zone.conflict_start_s+14);occupying.x=future.x;occupying.y=future.y;
 for(int i=0;i<8;++i){double dx[4]={-1,1,1,-1},dy[4]={-1,-1,1,1};occupying.corners[i]={occupying.x+dx[i%4],occupying.y+dy[i%4],double(i/4)};}
 out={};bool braked=brake_init&&brake_planner.PlanRoundaboutMerge(ego,{occupying},cmd,600,600,out,r);
 expect(braked&&!r.candidate_valid&&out.x.size()>2&&out.v.back()<.05,"blocked traversal under service braking retains a multi-point stop trajectory");
 expect(braked&&CartesianTrajectoryClear(out,{occupying},shape,{.5,.1,3},.1),"steerable service stop still checks the occupying object's physical clearance");

 MergeAccelerationMemory memory;
 expect(memory.Seed(-2.4,1)==-2.4,"first traversal keeps measured braking seed");
 memory.Accept({-2.4,-1.92,-1.44},1,.1);
 expect(std::fabs(memory.Seed(-2.58,1.05)+2.16)<1e-9,"fresh replan advances accepted brake release rather than measured restart");
 expect(memory.Seed(-2.58,1)==-2.58&&memory.Seed(-2.58,.9)==-2.58,"duplicate or reversed planning time does not advance memory");
 expect(memory.Seed(-2.58,1.6)==-2.58,"stale traversal memory cannot seed a new plan");
 expect(memory.Seed(.5,1.05)==.5,"positive measured acceleration remains authoritative");
 memory.Reset();expect(memory.Seed(-2.58,1.05)==-2.58,"safety braking reset restores measured acceleration");
 FrenetPlanner release_planner;bool release_ok=release_planner.Init(file.string());
 ego=pose_at(zone.rear_clear_s-9);ego.v=2;ego.a=-2.58;
 release_planner.ApplyRoundaboutMergeCommit({});
 double minimum_speed=ego.v,previous_a=-10;bool advances=true;
 for(int i=0;i<12&&release_ok;++i){
  out={};release_ok=release_planner.PlanRoundaboutMerge(ego,{},cmd,700+i*.1,700+i*.1,out,r)&&r.candidate_valid&&out.x.size()>1;
  if(!release_ok)break;
  advances&=out.a.front()>=previous_a-1e-6;
  previous_a=out.a.front();minimum_speed=std::min(minimum_speed,out.v[1]);
  ego={out.x[1],out.y[1],out.yaw[1],out.kappa[1],out.v[1],-2.58};
 }
 expect(release_ok&&advances&&previous_a>0&&minimum_speed>.8,"persistent negative measurement cannot restart safe merge brake release every cycle");

 occupying.x=ego.x;occupying.y=ego.y;
 for(int i=0;i<8;++i){double dx[4]={-1,1,1,-1},dy[4]={-1,-1,1,1};occupying.corners[i]={ego.x+dx[i%4],ego.y+dy[i%4],double(i/4)};}
 out={};bool physical_blocked=!release_planner.PlanRoundaboutMerge(ego,{occupying},cmd,701.3,701.3,out,r);
 expect(physical_blocked,"accepted acceleration history cannot bypass a new physical obstacle");
 out={};bool restored=release_planner.PlanRoundaboutMerge(ego,{},cmd,701.4,701.4,out,r);
 expect(restored&&!out.a.empty()&&out.a.front()<0,"after safety rejection the next traversal starts from measured braking");
 cmd.merge_type=MergeType::HIGHWAY;expect(!tick(200,200)&&planner.last_failure_reason()==FrenetPlanner::PlanFailureReason::INVALID_COMMAND,"roundabout implementation cannot silently execute highway merge");
 std::filesystem::remove_all(dir);return failures?1:0;
}
