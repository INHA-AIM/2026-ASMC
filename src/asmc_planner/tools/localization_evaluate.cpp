// Offline native-pose audit. Does not consume planner decisions or estimated
// localization, and cannot publish ROS or control a vehicle.
#include "Frenet/frenet_planner.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

int main(int argc, char** argv) {
  try {
    if (argc!=4) throw std::runtime_error("usage: localization_evaluate params.yaml native.csv output.csv");
    FrenetPlanner core;
    if (!core.Init(argv[1]) || !core.road_map().enabled())
      throw std::runtime_error("fixed reference and drivable map must be loaded");
    std::ifstream input(argv[2]);
    if (!input) throw std::runtime_error("missing native input");
    std::ifstream exists(argv[3]);
    if (exists.good()) throw std::runtime_error("refusing to overwrite evaluation");
    std::ofstream out(argv[3]);
    if (!out) throw std::runtime_error("cannot create evaluation");
    out<<"index,stamp,s,x,y,yaw,lateral_m,heading_deg,map_inside,projected\n"<<std::setprecision(17);
    std::string line; std::getline(input,line); double prior=NAN;
    const VehicleShape shape{2.160835,4.635,.79}; // conservative SDK physical envelope, same as local Config
    while (std::getline(input,line)) {
      std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
      unsigned index;double t,x,y,yaw;
      if (!(row>>index>>t>>x>>y>>yaw) || !std::isfinite(t+x+y+yaw))
        throw std::runtime_error("malformed native pose row");
      CartesianState ego{x,y,yaw,0,0,0}, guide;double s=0,d=0;
      const bool projected=(std::isfinite(prior)?core.ProjectPointToFrenet(x,y,prior,30.,4.,s,d):
          core.ProjectEgoToS(ego,s)) && core.ReferencePoseAt(s,guide);
      if (!projected) {out<<index<<','<<t<<",nan,nan,nan,nan,nan,nan,0,0\n";continue;}
      prior=s;
      const double lateral=-std::sin(guide.yaw)*(x-guide.x)+std::cos(guide.yaw)*(y-guide.y);
      const double heading=std::remainder(yaw-guide.yaw,2*M_PI)*180/M_PI;
      out<<index<<','<<t<<','<<s<<','<<guide.x<<','<<guide.y<<','<<guide.yaw<<','
         <<lateral<<','<<heading<<','<<core.road_map().FootprintInside(x,y,yaw,shape)<<",1\n";
    }
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
