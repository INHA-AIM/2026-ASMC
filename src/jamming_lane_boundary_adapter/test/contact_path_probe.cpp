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
int main(int argc,char**argv){
 if(argc==2 && std::string(argv[1])=="--interpolation-regression")return interpolationRegression();
 const bool inspect=argc==4 && std::string(argv[3])=="--segments";
 const bool support_replay=argc==4 && std::string(argv[3])=="--support-replay";
 if(argc!=3 && argc!=5 && argc!=6 && !inspect && !support_replay)return 2;
 if(inspect || support_replay){adapter_config.consumer_x_max_m=params.consumer_x_max=20.;params.path_horizon=15.;}
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
  for(const auto&p:path)std::cout<<p.x<<','<<p.y<<';';std::cout<<'\n';return 0;
 }
 std::cout<<"runtime_decision="<<updateContactPathPlan()<<" candidates="<<contact_candidates<<" road_rejected="<<contact_road_rejected<<" collision_rejected="<<contact_collision_rejected<<'\n';
 std::vector<lane_boundary_adapter::clearance::Polygon> local_occupied;
 std::vector<lane_boundary_adapter::clearance::Bounds> contacts;
 unsigned contact_index=0;
 for(const auto& poly:bbox->ground_contacts){
  lane_boundary_adapter::clearance::Bounds b{1e100,-1e100,1e100,-1e100};
  for(const auto&p:poly.points){b.xmin=std::min(b.xmin,double(p.x));b.xmax=std::max(b.xmax,double(p.x));b.ymin=std::min(b.ymin,double(p.y));b.ymax=std::max(b.ymax,double(p.y));}
  if(bbox->ground_contact_sources.size()==bbox->boxes.size() && bbox->ground_contact_sources[contact_index]=="lidar_observed")
   local_occupied.push_back({{{b.xmin,b.ymin},{b.xmax,b.ymin},{b.xmax,b.ymax},{b.xmin,b.ymax}}});
  else contacts.push_back(b);
  ++contact_index;
  std::cout<<"contact "<<b.xmin<<','<<b.xmax<<','<<b.ymin<<','<<b.ymax<<'\n';
 }
 double horizon=bbox->contact_support_x_max_m;
 for(const auto& lane:change_state.lanes) horizon=std::min({horizon,lane.left_boundary.back().x,lane.right_boundary.back().x});
 std::vector<lane_boundary_adapter::clearance::Point> reference;
 for(const auto& lane:change_state.lanes)if(lane.lane_index==1)for(const auto& p:lane.left_boundary)reference.push_back({p.x,p.y});
 std::vector<lane_boundary_adapter::clearance::Polygon> occupied;
 if(!lane_boundary_adapter::clearance::contactOccupancy(contacts,reference,horizon,occupied))return 4;
 for(const auto& p:bbox->local_obstacle_footprints){lane_boundary_adapter::clearance::Polygon o;for(unsigned i=0;i<4;++i)o[i]={p.points[i].x,p.points[i].y};local_occupied.push_back(o);}
 occupied.insert(occupied.end(),local_occupied.begin(),local_occupied.end());
 const lane_boundary_adapter::clearance::Vehicle shape{params.vehicle_front,params.vehicle_rear,params.vehicle_width/2,params.obstacle_lateral_margin,params.obstacle_longitudinal_margin};
 for(const auto& original:change_state.lanes)for(double offset:{0.,-.05,.05,-.1,.1,-.15,.15,-.2,.2,-.25,.25,-.3,.3,-.35,.35,-.4,.4,-.45,.45,-.5,.5})for(double length:{3.,4.,5.,6.,8.,10.,12.}){
  auto lane=original;for(auto&p:lane.centerline)p.y+=offset;
  std::vector<FamilyPoint2d> path;if(!buildPath(lane,true,path,1.,length,true))continue;
  const double end=horizon-std::hypot(shape.front+shape.longitudinal_margin+params.path_sample_step,shape.half_width+shape.lateral_margin)-.1;
  while(!path.empty() && path.back().x>end)path.pop_back();
  const bool road=path.size()>1 && path.back().x>=params.lookahead_distance && pathInsideRoad(path);
  std::vector<lane_boundary_adapter::clearance::Point> pts;for(const auto&p:path)pts.push_back({p.x,p.y});
  const bool clear=lane_boundary_adapter::clearance::pathClearOfPolygons(pts,occupied,shape,horizon,params.path_sample_step);
  std::cout<<"lane="<<lane.lane_index<<" offset="<<offset<<" length="<<length<<" road="<<road<<" clear="<<clear<<" curvature="<<change_state.maximum_path_curvature<<" path=";
  for(unsigned i=0;i<path.size();i+=8) { std::cout<<path[i].x<<','<<path[i].y<<';'; }
  std::cout<<'\n';
 }
}
