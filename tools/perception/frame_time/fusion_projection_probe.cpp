// Offline F1 probe: production calibration/project(), no map pose or new math.
#include <asmc_lidar/geometry/camera_calibration.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <iomanip>
using namespace asmc::lidar;
int main(int argc,char** argv) { try {
  if(argc!=2) throw std::runtime_error("usage: fusion_projection_probe camera.yaml < requests.jsonl");
  const auto cameras=camera::load_calibrations(argv[1]);
  std::string line; std::cout<<std::setprecision(17);
  while(std::getline(std::cin,line)) {
    auto n=YAML::Load(line);const auto& cal=cameras.at(n["camera_id"].as<std::string>());
    Context c;c.frame=Frame::Base;
    const Ns stamp=n["scan_stamp_ns"].as<Ns>(),arrival=n["scan_arrival_ns"].as<Ns>();
    // Geometry-only query at recorded ROS receipt; no scheduler timing claim.
    c.time={stamp,arrival,arrival,"ros","bridge_receive_ns","ros"};
    c.calibration_source="F1 same-scan base_link corners";
    std::cout<<"{\"calibration_id\":\""<<cal.id<<"\",\"width\":"<<cal.width
             <<",\"height\":"<<cal.height<<",\"points\":[";
    bool comma=false;
    for(const auto& p:n["points"]) {
      if(comma)std::cout<<',';
      comma=true;
      const auto q=camera::project(cal,c,n["image_stamp_ns"].as<Ns>(),
                                  Vec(p[0].as<double>(),p[1].as<double>(),p[2].as<double>()));
      std::cout<<"{\"valid\":"<<(q.valid?"true":"false")<<",\"reason\":\""<<q.reason
               <<"\",\"u\":"<<(std::isfinite(q.u)?q.u:0)<<",\"v\":"<<(std::isfinite(q.v)?q.v:0)
               <<",\"depth\":"<<(std::isfinite(q.depth)?q.depth:0)<<'}';
    }
    std::cout<<"]}\n";
  }return 0;
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
