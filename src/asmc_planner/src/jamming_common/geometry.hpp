#pragma once
#include "jamming_common/types.hpp"
namespace local_planning {
void BuildGridIndex(Grid &);
bool CheckFootprint(const Scene &,const Config &,const Pose &,Result &);
bool CheckSweep(const Scene &,const Config &,const Pose &,const Pose &,Result &);
std::array<double,3> Rotate(const std::array<double,9>&,double,double,double);
std::array<double,9> GroundRotation(double,double);
std::array<double,9> AttitudeRotation(double,double);
std::array<double,3> GridToPlan(const Grid&,double,double);
std::array<double,3> PlanToGrid(const Grid&,double,double);
} // namespace local_planning
