// Offline probe invokes the actual node's candidate/road math. No control node,
// subscribers, publications or alternate Python planning implementation.
#define main shadow_controller_runtime_main
#include "../src/camera_bbox_lane_change_node.cpp"
#undef main
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <ros/serialization.h>
template<class T> boost::shared_ptr<T> read(const char* file){
 std::ifstream f(file,std::ios::binary);std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)),{});
 if(b.empty())throw std::runtime_error("empty serialized fixture");
 auto m=boost::make_shared<T>();ros::serialization::IStream s(b.data(),b.size());ros::serialization::deserialize(s,*m);return m;
}
// Exercise the previous linear selection exactly, including its epsilon
// boundary convention. The runtime geometry is monotonic by construction.
int interpolationRegression(){
 LaneChain lane;lane.monotonic_x=true;for(int i=0;i<241;++i){double x=i*.083;lane.centerline.push_back({x,std::sin(x)});lane.left_boundary.push_back({x,std::sin(x)+2});lane.right_boundary.push_back({x,std::sin(x)-2});}
 for(int k=-20;k<24200;++k){double x=k*.00083;LaneSample old,now;bool expected=false;
  if(x>=lane.centerline.front().x-params.epsilon && x<=lane.centerline.back().x+params.epsilon)
   for(size_t i=1;i<lane.centerline.size();++i){if(x>lane.centerline[i].x+params.epsilon)continue;
    double d=lane.centerline[i].x-lane.centerline[i-1].x;if(d<=params.epsilon)break;
    double r=std::clamp((x-lane.centerline[i-1].x)/d,0.,1.);
    old.center=interpolatePoint(lane.centerline[i-1],lane.centerline[i],r);old.left=interpolatePoint(lane.left_boundary[i-1],lane.left_boundary[i],r);old.right=interpolatePoint(lane.right_boundary[i-1],lane.right_boundary[i],r);expected=true;break;}
  bool actual=interpolateLaneAtX(lane,x,now);
  if(expected!=actual || (actual&&(old.center.x!=now.center.x || old.center.y!=now.center.y || old.left.y!=now.left.y || old.right.y!=now.right.y)))return 6;
 }
 std::cout<<"interpolation_equivalent samples=24220\n";return 0;
}
int bevRegression(){
 ros::Time::init();ros::Time::setNow(ros::Time(100));
 bev_spec={250,160,20,8,.1};
 auto status=boost::make_shared<asmc_msgs::InferenceStatus>();
 status->model="twinlite";status->producer_session_id="one";status->generation=2;status->mode=status->ACTIVE;
 status->header.stamp=ros::Time(100);status->source_cutoff=ros::Time(99);bevEpoch(status);
 auto image=boost::make_shared<sensor_msgs::Image>();image->header.stamp=ros::Time(100);image->header.frame_id="base_link";
 image->encoding="mono8";image->height=250;image->width=image->step=160;image->data.assign(40000,255);
 bevInput(image,true);if(selectBev(ros::Time(100),ros::Time(100)))return 10;
 bevInput(image,false);if(!selectBev(ros::Time(100),ros::Time(100)))return 11;
 if(selectBev(ros::Time(100.051),ros::Time(100)))return 12;
 auto duplicate=boost::make_shared<sensor_msgs::Image>(*image);duplicate->header.stamp=ros::Time(99.99);
 bevInput(duplicate,false);if(!selectBev(ros::Time(100),ros::Time(100)))return 13;
 ros::Time::setNow(ros::Time(100.36));if(selectBev(ros::Time(100),ros::Time(100)))return 14;
 status->generation++;status->header.stamp=ros::Time(100.36);status->source_cutoff=ros::Time(100.3);bevEpoch(status);
 if(!bev_frames.empty())return 15;
 image->header.stamp=ros::Time(100.36);bevInput(image,true);bevInput(image,false);
 if(!selectBev(ros::Time(100.36),ros::Time(100.36)))return 16;
 status->producer_session_id="two";status->header.stamp=ros::Time(100.37);bevEpoch(status);
 if(!bev_frames.empty())return 17;
 status->producer_session_id="one";status->header.stamp=ros::Time(100.38);bevEpoch(status);
 if(bev_session!="two")return 18;
 std::cout<<"bev_epoch_pairing_freshness_passed\n";return 0;
}
int semanticRegression(){
 ros::Time::init();ros::Time::setNow(ros::Time(100));
 params.contact_clearance_enabled=true;params.contact_local_required=true;params.bev_mode="semantic_trial";
 params.path_horizon=15.;params.expected_config_sha256="fixture";
 bev_spec={250,160,20,8,.1};
 auto status=boost::make_shared<asmc_msgs::InferenceStatus>();status->model="twinlite";
 status->producer_session_id="one";status->generation=2;status->mode=status->ACTIVE;
 status->header.stamp=ros::Time(100);status->source_cutoff=ros::Time(99);bevEpoch(status);
 std::vector<LaneChain> lanes;
 for(int index=0;index<2;++index){LaneChain l;l.lane_index=index;l.monotonic_x=true;
  for(int i=4;i<=80;++i){double x=i*.25,y=index==1?0.:3.5;
   l.centerline.push_back({x,y});l.left_boundary.push_back({x,y+1.75});l.right_boundary.push_back({x,y-1.75});}lanes.push_back(l);}
 follower_state.selected_lane=lanes[1];follower_state.locked_lane=1;
 contact_lane_history.push_back({ros::Time(100),lanes});
 auto local=boost::make_shared<asmc_msgs::LidarTrackingObservation>();auto& o=local->observation;
 o.producer_session_id="lidar";o.config_id="c";o.calibration_id="k";o.scan_sequence=1;
 o.objects.header.frame_id="base_link";o.objects.header.stamp=ros::Time(100);o.objects.status=o.objects.OBSERVED_EMPTY;
 semantic_local.accept(local,ros::Time(100),ros::SteadyTime::now());
 auto context=boost::make_shared<asmc_msgs::CameraGroundProjection>();context->header.stamp=ros::Time(100);
 context->header.seq=900;context->producer_session_id="one";context->generation=2;context->config_id="fixture";context->imu_corrected=true;
 auto image=boost::make_shared<sensor_msgs::Image>();image->header.stamp=ros::Time(100);image->header.frame_id="base_link";
 image->encoding="mono8";image->height=250;image->width=image->step=160;image->data.assign(40000,255);
 bevInput(image,true);bevInput(image,false);
 if(selectBev(ros::Time(100),ros::Time(100)))return 30; // no context
 semantic_contexts.push_back(context);bevInput(image,true);
 if(selectBev(ros::Time(100.01),ros::Time(100.01)))return 31; // no nearest family
 auto result=updateSemanticPathPlan();if(!result.empty()){std::cerr<<result;return 32;}
 if(!contact_geometry_ready || bool(contact_result) || semantic_target!=1)return 33;
 if(semantic_safe_length<14.)return 34;
 // A newer semantic pair without <=50ms LiDAR cannot discard an older
 // complete tuple that is still fresh. No individual timestamp is refreshed.
 ros::Time::setNow(ros::Time(100.15));contact_lane_history.push_back({ros::Time(100.12),lanes});
 auto newest=bev_frames.back();newest.stamp=ros::Time(100.12);bev_frames.push_back(newest);
 if(!updateSemanticPathPlan().empty() || semantic_selected_stamp!=ros::Time(100))return 38;
 contact_lane_history.pop_back();bev_frames.pop_back();ros::Time::setNow(ros::Time(100));
 // A measured local collision vetoes semantic free space, including blind
 // assistance. An empty array is never allowed to overwrite this geometry.
 auto occupied=boost::make_shared<asmc_msgs::LidarTrackingObservation>(*local);
 occupied->observation.objects.status=occupied->observation.objects.OBSERVED;
 asmc_msgs::LidarObject obstacle;obstacle.observation_state=obstacle.OBSERVED;
 obstacle.source_point_count=8;obstacle.length=1.;obstacle.width=9.;obstacle.height=1.;
 unsigned k=0;for(double x:{6.,7.})for(double y:{-3.,6.})for(double z:{0.,1.}){
  obstacle.obb_corners[k].x=x;obstacle.obb_corners[k].y=y;obstacle.obb_corners[k++].z=z;}
 occupied->observation.objects.objects.push_back(obstacle);
 semantic_local=lane_boundary_adapter::LocalObstacleSupport();semantic_local.accept(occupied,ros::Time(100),ros::SteadyTime::now());
 if(updateSemanticPathPlan()!="semantic_all_paths_blocked_or_unobserved" || !contact_collision_rejected)return 39;
 semantic_local=lane_boundary_adapter::LocalObstacleSupport();semantic_local.accept(local,ros::Time(100),ros::SteadyTime::now());
 // The same algorithm must refuse an all-unknown grid, even with EMPTY LiDAR.
 auto grid=std::make_shared<lane_boundary_adapter::clearance::DrivableClearance>();
 std::vector<uint8_t> zeros(40000,0),free(40000,255);
 grid->prepare(bev_spec,free,zeros,{});bev_frames.back().grid=grid;
 if(updateSemanticPathPlan()!="semantic_all_paths_blocked_or_unobserved")return 35;
 // An own-lane semantic shadow begins at12m and extends indefinitely; the
 // adjacent observed corridor permits a prefix, without requiring a return.
 for(int r=0;r<81;++r)for(int c=70;c<90;++c)free[r*160+c]=0;
 grid->prepare(bev_spec,free,image->data,{});bev_frames.back().grid=grid;
 result=updateSemanticPathPlan();if(!result.empty() || semantic_target!=0){std::cerr<<result<<" target="<<semantic_target;return 36;}
 // Receding horizon trap: home is free for8m, but an observed hole at16m
 // blocks its long plan. Far unknown truncates both full horizons. Start a
 // checked adjacent prefix rather than repeatedly choosing the home prefix.
 free.assign(40000,255);std::vector<uint8_t> coverage(40000,255);
 for(int r=0;r<10;++r)for(int c=0;c<160;++c)coverage[r*160+c]=0;
 for(int r=0;r<=40;++r)for(int c=70;c<90;++c)free[r*160+c]=0;
 grid->prepare(bev_spec,free,coverage,{});bev_frames.back().grid=grid;
 semantic_home=semantic_target=1;semantic_confirm=0;semantic_last_decision=ros::Time();
 result=updateSemanticPathPlan();if(!result.empty() || semantic_target!=0)return 43;
 result=updateSemanticPathPlan();if(!result.empty() || semantic_target!=0 || semantic_confirm)return 44;
 ros::Time::setNow(ros::Time(100.36));
 if(updateSemanticPathPlan()!="semantic_exact_family_space_unavailable")return 37;
 std::cout<<"semantic_primary_no_yolo_exact_epoch_unknown_and_partial_avoidance_passed\n";return 0;
}
// Exact recorded source pair; invokes production candidate, projection context
// and local footprint checks. This is offline decision replay, not wall latency.
int semanticFrame(const char* directory,bool assist){
 const std::string d=directory;
 auto family=read<asmc_msgs::LaneBoundaryFamily>((d+"/family.rosbin").c_str());
 auto probability=read<sensor_msgs::Image>((d+"/probability.rosbin").c_str());
 auto coverage=read<sensor_msgs::Image>((d+"/coverage.rosbin").c_str());
 auto projection=read<asmc_msgs::CameraGroundProjection>((d+"/context.rosbin").c_str());
 auto local=read<asmc_msgs::LidarTrackingObservation>((d+"/local.rosbin").c_str());
 const auto stamp=family->header.stamp;
 ros::Time::init();ros::Time::setNow(stamp+ros::Duration(.05));
 params.bev_mode="semantic_trial";params.semantic_near_ego_lidar_assist=assist;
 params.expected_config_sha256=projection->config_id;
 params.contact_clearance_enabled=true;params.path_horizon=15.;
 params.consumer_x_max=adapter_config.consumer_x_max_m=20.;
 bev_spec={250,160,20,8,.1};
 auto context=lane_boundary_adapter::BuildBoundaryFamilyStripContext(*family,adapter_config);
 if(!context.usable()){std::cout<<context.reason;return 40;}
 auto lanes=buildContinuousLanes(context);auto candidates=findOwnLaneCandidates(lanes);
 if(candidates.size()!=1){std::cout<<"own_lane_candidates="<<candidates.size();return 41;}
 follower_state.selected_lane=candidates[0].lane;follower_state.locked_lane=candidates[0].lane.lane_index;
 contact_lane_history.push_back({stamp,lanes});
 auto status=boost::make_shared<asmc_msgs::InferenceStatus>();status->model="twinlite";
 status->producer_session_id=projection->producer_session_id;status->generation=projection->generation;
 status->mode=status->ACTIVE;status->header.stamp=stamp;status->source_cutoff=stamp-ros::Duration(1.);
 bevEpoch(status);semantic_contexts.push_back(projection);
 bevInput(probability,true);bevInput(coverage,false);
 semantic_local.accept(local,ros::Time::now(),ros::SteadyTime::now());
 const auto decision=updateSemanticPathPlan();
 std::cout<<"decision="<<decision<<" candidates="<<contact_candidates<<" road="<<contact_road_rejected
   <<" grid="<<bev_rejected<<" local="<<contact_collision_rejected<<" target="<<semantic_target<<'\n';
 auto grid=selectBev(stamp,stamp);if(!grid)return 42;
 const lane_boundary_adapter::clearance::Vehicle vehicle{params.vehicle_front,params.vehicle_rear,params.vehicle_width/2,
  params.obstacle_lateral_margin,params.obstacle_longitudinal_margin};
 for(const auto& original:lanes)for(double offset:{0.,-.3,.3})for(double length:{6.,10.,15.}){
  auto lane=original;for(auto& p:lane.centerline)p.y+=offset;
  std::vector<FamilyPoint2d> path;if(!buildPath(lane,true,path,1.,length,true,8.)||!pathInsideRoad(path,&lanes))continue;
  std::vector<lane_boundary_adapter::clearance::Point> pts;for(const auto& p:path)pts.push_back({p.x,p.y});
  const auto result=grid->check(pts,vehicle,params.path_sample_step);
  std::cout<<"lane="<<lane.lane_index<<" offset="<<offset<<" length="<<length<<" clear="<<result.clear
   <<" reason="<<result.reason<<" blocked="<<result.blocked_x<<','<<result.blocked_y<<" assisted="<<result.assisted<<'\n';
 }
 // Offline attribution only. Do not publish these counterfactual inputs.
 // Preserve actual geometry and compare which layer rejects all candidates.
 const auto original_grid=bev_frames.back().grid;
 for(int mask=1;mask<4;++mask){
  semantic_home=semantic_target=follower_state.locked_lane;semantic_confirm=0;semantic_last_decision=ros::Time();
  bev_frames.back().grid=original_grid;
  if(mask&1){auto free_grid=std::make_shared<lane_boundary_adapter::clearance::DrivableClearance>();
   std::vector<uint8_t> free_cells(40000,255);free_grid->prepare(bev_spec,free_cells,free_cells,vehicle,166,true,assist);bev_frames.back().grid=free_grid;}
  auto scan=boost::make_shared<asmc_msgs::LidarTrackingObservation>(*local);
  if(mask&2){scan->observation.objects.objects.clear();scan->observation.objects.status=scan->observation.objects.OBSERVED_EMPTY;}
  semantic_local=lane_boundary_adapter::LocalObstacleSupport();semantic_local.accept(scan,ros::Time::now(),ros::SteadyTime::now());
  const auto why=updateSemanticPathPlan();
  std::cout<<"COUNTERFACTUAL_NOT_EXECUTION semantic_free="<<bool(mask&1)<<" local_empty="<<bool(mask&2)
   <<" decision="<<why<<" target="<<semantic_target<<" end="<<semantic_safe_length
   <<" candidates="<<contact_candidates<<" road="<<contact_road_rejected<<" grid="<<bev_rejected<<" local="<<contact_collision_rejected<<'\n';
 }
 return 0;
}
int main(int argc,char**argv){
 if(argc==4 && std::string(argv[1])=="--semantic-frame")return semanticFrame(argv[2],std::string(argv[3])=="assist");
 if(argc==2 && std::string(argv[1])=="--semantic-regression")return semanticRegression();
 if(argc==2 && std::string(argv[1])=="--bev-regression")return bevRegression();
 if(argc==2 && std::string(argv[1])=="--interpolation-regression")return interpolationRegression();
 const bool wide=argc==4 && std::string(argv[3])=="--wide-search";
 const bool lengths=argc==4 && std::string(argv[3])=="--length-search";
 const bool inspect=argc==4 && std::string(argv[3])=="--segments";
 const bool support_replay=argc==4 && std::string(argv[3])=="--support-replay";
 if(argc!=3 && argc!=5 && argc!=6 && !inspect && !support_replay && !wide && !lengths)return 2;
 if(inspect || support_replay || wide || lengths){adapter_config.consumer_x_max_m=params.consumer_x_max=20.;params.path_horizon=15.;}
 if(argc>=5){adapter_config.consumer_x_max_m=std::stod(argv[3]);params.consumer_x_max=std::stod(argv[3]);params.path_horizon=std::stod(argv[4]);}
 ros::Time::init();
 auto family=read<asmc_msgs::LaneBoundaryFamily>(argv[1]);
 auto bbox=read<BboxResult>(argv[2]);
 const auto context=lane_boundary_adapter::BuildBoundaryFamilyStripContext(*family,adapter_config);
 if(!context.usable()){std::cerr<<context.reason<<'\n';return 3;}
 if(inspect){for(const auto& segment:context.segments)std::cout<<"lane="<<segment.lateral_index<<" begin="<<segment.valid_x_min_m<<" end="<<segment.valid_x_max_m<<" confidence="<<segment.effective_confidence<<'\n';return 0;}

 if(support_replay){double support[2]={};for(unsigned lane=0;lane<2;++lane){
  std::vector<std::pair<double,double>> spans;for(const auto& segment:context.segments)
   if(segment.lateral_index==lane && segment.effective_confidence>=params.min_confidence)spans.emplace_back(segment.valid_x_min_m,segment.valid_x_max_m);
  support[lane]=lane_boundary_adapter::clearance::contiguousSupport(std::move(spans));}
  bbox->contact_support_x_max_m=std::min(support[0],support[1]);std::cout<<"recomputed_support="<<bbox->contact_support_x_max_m<<'\n';
 }
 change_state.lanes=buildContinuousLanes(context);params.contact_clearance_enabled=true;
 follower_state.locked_lane=bbox->own_lane;follower_state.source_stamp=bbox->header.stamp;contact_result=bbox;
 contact_lane_history.push_back({bbox->lane_stamp,change_state.lanes});
 change_state.changing=true;change_state.target_lane=bbox->own_lane;
 if(argc==6){
  const int n=std::stoi(argv[5]);if(n<1 || n>10000)return 2;
  std::vector<double> ms;ms.reserve(n);std::string first;std::vector<FamilyPoint2d> path;
  for(int i=0;i<n;++i){const auto start=std::chrono::steady_clock::now();const auto decision=updateContactPathPlan();
   ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
   if(i==0){first=decision;path=change_state.path;}else{
    if(first!=decision || path.size()!=change_state.path.size())return 5;
    for(size_t j=0;j<path.size();++j)if(path[j].x!=change_state.path[j].x || path[j].y!=change_state.path[j].y)return 5;
   }}
  std::sort(ms.begin(),ms.end());std::cout<<std::setprecision(17)<<"iterations="<<n<<" decision="<<first<<" candidates="<<contact_candidates
   <<" p50_ms="<<ms[(n-1)*50/100]<<" p95_ms="<<ms[(n-1)*95/100]<<" p99_ms="<<ms[(n-1)*99/100]<<" max_ms="<<ms.back()<<'\n';
  for(const auto&p:path)std::cout<<p.x<<','<<p.y<<';';
  std::cout<<'\n';return 0;
 }
 std::cout<<"runtime_decision="<<updateContactPathPlan()<<" candidates="<<contact_candidates<<" road_rejected="<<contact_road_rejected<<" collision_rejected="<<contact_collision_rejected<<'\n';
 std::vector<lane_boundary_adapter::clearance::Polygon> local_occupied;
 std::vector<lane_boundary_adapter::clearance::Polygon> contacts;
 unsigned contact_index=0;
 for(const auto& poly:bbox->ground_contacts){
  lane_boundary_adapter::clearance::Bounds b{1e100,-1e100,1e100,-1e100};
  for(const auto&p:poly.points){b.xmin=std::min(b.xmin,double(p.x));b.xmax=std::max(b.xmax,double(p.x));b.ymin=std::min(b.ymin,double(p.y));b.ymax=std::max(b.ymax,double(p.y));}
  lane_boundary_adapter::clearance::Polygon q;
  for(const auto& p:poly.points)q.push_back({p.x,p.y});
  if(bbox->ground_contact_sources.size()==bbox->boxes.size() && bbox->ground_contact_sources[contact_index]=="lidar_observed")
   local_occupied.push_back(lane_boundary_adapter::clearance::paddedHull(q,0.));
  else contacts.push_back(q);
  ++contact_index;
  std::cout<<"contact "<<b.xmin<<','<<b.xmax<<','<<b.ymin<<','<<b.ymax<<'\n';
 }
 double horizon=bbox->contact_support_x_max_m;
 for(const auto& lane:change_state.lanes) horizon=std::min({horizon,lane.left_boundary.back().x,lane.right_boundary.back().x});
 std::vector<lane_boundary_adapter::clearance::Point> reference;
 for(const auto& lane:change_state.lanes)if(lane.lane_index==1)for(const auto& p:lane.left_boundary)reference.push_back({p.x,p.y});
 std::vector<lane_boundary_adapter::clearance::Polygon> occupied;
 if(!lane_boundary_adapter::clearance::projectedContactOccupancy(contacts,reference,horizon,occupied))return 4;
 for(const auto& p:bbox->local_obstacle_footprints){lane_boundary_adapter::clearance::Polygon o;for(const auto& point:p.points)o.push_back({point.x,point.y});local_occupied.push_back(lane_boundary_adapter::clearance::paddedHull(o,0.));}
 occupied.insert(occupied.end(),local_occupied.begin(),local_occupied.end());
 const lane_boundary_adapter::clearance::Vehicle shape{params.vehicle_front,params.vehicle_rear,params.vehicle_width/2,params.obstacle_lateral_margin,params.obstacle_longitudinal_margin};
 std::vector<double> offsets{0.,-.05,.05,-.1,.1,-.15,.15,-.2,.2,-.25,.25,-.3,.3,-.35,.35,-.4,.4,-.45,.45,-.5,.5};
 if(wide)for(double x:{.55,.60,.65,.70,.75}){offsets.push_back(-x);offsets.push_back(x);}
 std::vector<double> maneuver_lengths{3.,4.,5.,6.,8.,10.,12.,15.};
 // Offline sensitivity only: do not silently widen the operational grid.
 if(lengths){maneuver_lengths.clear();for(double x=3.;x<=20.;x+=.5)maneuver_lengths.push_back(x);}
 for(const auto& original:change_state.lanes)for(double offset:offsets)for(double length:maneuver_lengths){
  auto lane=original;for(auto&p:lane.centerline)p.y+=offset;
  std::vector<FamilyPoint2d> path;
  const double end=horizon-std::hypot(shape.front+shape.longitudinal_margin+params.path_sample_step,shape.half_width+shape.lateral_margin)-.1;
  const double path_end=std::floor(std::min(params.path_horizon,end)/params.path_sample_step)*params.path_sample_step;
  if(!buildPath(lane,true,path,1.,length,true,path_end))continue;
  RoadCheckFailure failure;
  const bool road=path.size()>1 && path.back().x>=params.lookahead_distance && pathInsideRoad(path,nullptr,&failure);
  std::vector<lane_boundary_adapter::clearance::Point> pts;for(const auto&p:path)pts.push_back({p.x,p.y});
  const bool clear=lane_boundary_adapter::clearance::pathClearOfPolygons(pts,occupied,shape,horizon,params.path_sample_step);
  std::cout<<"lane="<<lane.lane_index<<" offset="<<offset<<" length="<<length<<" road="<<road<<" clear="<<clear<<" curvature="<<change_state.maximum_path_curvature
   <<" road_failure="<<failure.reason<<" corner="<<failure.x<<','<<failure.y<<" bounds="<<failure.right<<','<<failure.left
   <<" curvature_limit="<<failure.curvature_limit<<" path=";
  for(unsigned i=0;i<path.size();i+=8) { std::cout<<path[i].x<<','<<path[i].y<<';'; }
  std::cout<<'\n';
 }
}
