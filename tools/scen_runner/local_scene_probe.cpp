// Exact recorded scene replay through the native LOCAL core. No ROS control.
#include "local/planner.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
using namespace local_planning;
int main(int argc,char**argv) {
  if(argc<2)return 2;
  const auto root=YAML::LoadFile(argv[1]);const auto g=root["grid"];
  Scene s;Config c;c.recovery=true;c.speed_cap=10./3.6;
  c.budget_ms=argc>2?std::stod(argv[2]):40.;
  if(argc>3)c.blind_assist=std::stoi(argv[3]);
  if(argc>4)c.recovery=std::stoi(argv[4]);
  s.speed=root["speed"].as<double>(0.);s.acceleration=root["acceleration"].as<double>(0.);
  s.yaw_rate=root["yaw_rate"].as<double>(0.);s.age=root["age"].as<double>(0.);s.aligned_to_plan=true;
  s.grid.rows=g["rows"].as<int>();s.grid.cols=g["cols"].as<int>();
  s.grid.resolution=g["resolution"].as<double>(.1);
  s.grid.ground_z=g["ground_z"].as<double>();
  s.grid.rotation=g["rotation"].as<std::array<double,9>>();
  s.grid.source_in_plan=g["source_in_plan"].as<std::array<double,3>>();
  for(auto x:g["probability"])s.grid.probability.push_back(x.as<unsigned>());
  for(auto x:g["coverage"])s.grid.coverage.push_back(x.as<unsigned>());
  for(auto x:root["home"])s.home.push_back(x.as<std::array<double,2>>());
  for(auto x:root["obstacles"]) {Obstacle o;o.id=x["id"].as<int>();
    o.uncertainty=x["motion_uncertainty_m"].as<double>();
    for(auto p:x["polygon"])o.polygon.push_back({p[0].as<double>(),p[1].as<double>()});
    s.obstacles.push_back(o);}
  const auto r=Plan(s,c);
  std::cout<<"{\"valid\":"<<(r.valid?"true":"false")<<",\"reason\":\""<<r.reason<<"\",\"length\":"<<(r.path.empty()?0:r.path.back().s)<<",\"chosen_kph\":"<<r.chosen_cap*3.6<<",\"elapsed_ms\":"<<r.elapsed_ms<<",\"rejected\":{";
  bool first=true;for(auto p:r.rejected){if(!first)std::cout<<',';first=false;std::cout<<'"'<<p.first<<"\":"<<p.second;}std::cout<<"},\"path\":[";
  first=true;for(auto p:r.path){if(!first)std::cout<<',';first=false;std::cout<<'['<<p.x<<','<<p.y<<','<<p.yaw<<']';}std::cout<<"]}\n";
}
