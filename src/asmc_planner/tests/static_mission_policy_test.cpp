#include "Parser/StaticMissionPolicy.hpp"
#include <cassert>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <openssl/sha.h>
#include <unistd.h>
#include <iostream>
#include <boost/make_shared.hpp>
int main() {
  char tmp[]="/tmp/asmc-static-mission-XXXXXX";
  const std::string dir=mkdtemp(tmp), route="0 0 0\n100 0 0\n";
  std::ofstream(dir+"/route.txt")<<route;
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(route.data()),route.size(),digest);
  std::ostringstream sha;for(auto b:digest)sha<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);
  std::ofstream(dir+"/map.yaml")<<"schema_version: asmc.competition_metadata/v1\nframe_id: map\n"
    <<"mission_zones:\n- {id: static_obstacle_candidate, mission_type: static_obstacle, planning_enforcement_allowed: true, route_station_start_m: 20, route_station_end_m: 40}\n"
    <<"sources:\n  official_global_path: {path: '"<<dir<<"/route.txt', sha256: '"<<sha.str()<<"'}\n"
    <<"route: {length_xy_m: 100, point_count: 2}\n";
  asmc_planner::StaticMissionPolicy p(dir+"/map.yaml",dir);
  planning_inputs::DetectedObjectArray raw;raw.header.frame_id="map";raw.header.stamp=ros::Time(1);
  planning_inputs::DetectedObject o;o.id=42;o.type=1;o.classification="vehicle";o.camera_class="car";
  o.center.x=30;o.vx=12;o.vy=-4;o.motion_state=3;raw.objects.push_back(o);
  auto a=raw;assert(p.apply(30,0,0,true,a)==1);assert(a.objects[0].type==3&&a.objects[0].motion_state==1);
  assert(a.objects[0].vx==12&&a.objects[0].vy==-4&&!a.objects[0].velocity_unavailable);
  assert(raw.objects[0].motion_state==3&&a.objects[0].classification=="vehicle"&&a.objects[0].id==42);
  for(double x:{19.,41.}) {a=raw;assert(p.apply(x,0,0,true,a)==0&&a.objects[0].motion_state==3);}
  for(double x:{20.,40.}) {a=raw;a.objects[0].center.x=x;assert(p.apply(x,0,0,true,a)==1);}
  a=raw;a.objects[0].center.x=50;assert(p.apply(30,0,0,true,a)==0);
  a=raw;assert(p.apply(30,0,3.14,true,a)==0);assert(p.apply(30,0,0,false,a)==0);
  a=raw;a.header.frame_id="base_link";assert(p.apply(30,0,0,true,a)==0);
  a=raw;a.objects[0].velocity_unavailable=true;a.objects[0].vx=a.objects[0].vy=0;a.objects[0].motion_state=0;
  assert(p.apply(30,0,0,true,a)==1&&a.objects[0].velocity_unavailable&&a.objects[0].motion_state==0);
  a=raw;a.objects[0].velocity_unavailable=true;assert(p.apply(30,0,0,true,a)==0&&a.objects[0].motion_state==3);
  auto source=boost::make_shared<planning_inputs::DetectedObjectArray>(raw);
  asmc_planner::StaticMissionPolicy::Observation cache;
  assert(cache.update(p,source,30,0,0,true)&&cache.objects().objects[0].motion_state==1);
  assert(cache.update(p,source,50,0,0,true)&&cache.objects().objects[0].motion_state==3);
  unlink((dir+"/map.yaml").c_str());unlink((dir+"/route.txt").c_str());rmdir(dir.c_str());
  std::cout<<"PASS static mission scope, boundaries, motion prior, raw preservation, invalid inputs, exit restoration\n";
}
