// Offline probe invokes the actual node's candidate/road math. No control node,
// subscribers, publications or alternate Python planning implementation.
#define main shadow_controller_runtime_main
#include "../src/camera_bbox_lane_change_node.cpp"
#undef main
#include <fstream>
#include <iostream>
#include <ros/serialization.h>
template<class T> boost::shared_ptr<T> read(const char* file){
 std::ifstream f(file,std::ios::binary);std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)),{});
 if(b.empty())throw std::runtime_error("empty serialized fixture");
 auto m=boost::make_shared<T>();ros::serialization::IStream s(b.data(),b.size());ros::serialization::deserialize(s,*m);return m;
}
int main(int argc,char**argv){
 if(argc!=3 && argc!=5)return 2;
 if(argc==5){adapter_config.consumer_x_max_m=std::stod(argv[3]);params.consumer_x_max=std::stod(argv[3]);params.path_horizon=std::stod(argv[4]);}
 ros::Time::init();
 auto family=read<asmc_msgs::LaneBoundaryFamily>(argv[1]);
 auto bbox=read<BboxResult>(argv[2]);
 const auto context=lane_boundary_adapter::BuildBoundaryFamilyStripContext(*family,adapter_config);
 if(!context.usable()){std::cerr<<context.reason<<'\n';return 3;}
 change_state.lanes=buildContinuousLanes(context);params.contact_clearance_enabled=true;
 follower_state.locked_lane=bbox->own_lane;follower_state.source_stamp=bbox->header.stamp;contact_result=bbox;
 contact_lane_history.push_back({bbox->lane_stamp,change_state.lanes});
 change_state.changing=true;change_state.target_lane=bbox->own_lane;
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
 for(const auto& original:change_state.lanes)for(double offset:{0.,-.1,.1,-.2,.2,-.3,.3,-.4,.4,-.5,.5})for(double length:{3.,4.,5.,6.,8.,10.,12.}){
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
