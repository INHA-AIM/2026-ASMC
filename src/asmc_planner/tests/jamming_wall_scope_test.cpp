#include "shadow_localization/static_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <cmath>
int main(){
 const std::string root=JAMMING_SOURCE_DIR;
 shadow_localization::StaticPlanner scoped(root+"/config/frenet/params.yaml");
 const std::string file=root+"/config/shadow_localization/tunnel_wall_corridor_molitcomp03.json";
 scoped.load_wall_corridor(file);if(!scoped.wall_loaded())return 1;
 FrenetPlanner core;if(!core.Init(root+"/config/frenet/params.yaml"))return 2;
 const auto data=YAML::LoadFile(file);std::vector<geometry::Polygon2D> polygons;
 for(auto item:data["segments"]){geometry::Polygon2D p;for(auto xy:item["polygon"])p.push_back({xy[0].as<double>(),xy[1].as<double>()});polygons.push_back(p);}
 if(!core.SetShadowWallOverlay(polygons))return 3;
 const VehicleShape body{2.160835,4.635,.79};bool found=false;double fx=0,fy=0,heading=0;
 core.SetShadowPaintExemption(true);
 // Sample actual measured strips. The same full body must fit the union of
 // adjacent strip polygons, not merely a centre inside one polygon.
 for(size_t i=2;i+2<polygons.size()&&!found;++i){
  const auto& p=polygons[i];const double ax=(p[0].x+p[3].x)/2,ay=(p[0].y+p[3].y)/2;
  const double bx=(p[1].x+p[2].x)/2,by=(p[1].y+p[2].y)/2;
  const double yaw=std::atan2(by-ay,bx-ax);
  for(double t=.05;t<.96;t+=.05){
   const double x=(1-t)*(p[0].x+p[1].x)/2+t*(p[3].x+p[2].x)/2;
   const double y=(1-t)*(p[0].y+p[1].y)/2+t*(p[3].y+p[2].y)/2;
   core.SetShadowWallPermission(false);const bool before=core.road_map().FootprintInside(x,y,yaw,body);
   core.SetShadowWallPermission(true);const bool after=core.road_map().FootprintInside(x,y,yaw,body);
   if(!before&&after){found=true;fx=x;fy=y;heading=yaw;break;}
  }
 }
 if(!found){std::cerr<<"no qualified full-body shoulder extension\n";return 4;}
 core.SetShadowWallPermission(false);
 if(core.road_map().FootprintInside(fx,fy,heading,body))return 5;
 core.SetShadowWallPermission(true);
 if(core.road_map().FootprintInside(fx+30,fy+30,heading,body))return 6;
 std::cout<<"PASS measured shoulder permits full body only with wall permission; paint alone stays bounded\n";
 return 0;
}
