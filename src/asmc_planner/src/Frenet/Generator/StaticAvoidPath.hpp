#pragma once

#include "Frenet/Geometry/CartesianTypes.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include <string>

// Retain the spatial maneuver while updating its speed from observed v/a.
// The complete geometry has a settled tail where a stop can be certified.
// Curvature, speed-dependent steering slew and jerk use the production limits.
bool RetimeStaticAvoidGeometry(const CartesianPath& geometry,double speed,
    double acceleration,double target_speed,double dt,double wheelbase,
    const KinematicLimits& limits,CartesianPath& output,std::string* rejection=nullptr);

// Compare already checked shapes at a common speed ceiling. This estimates
// travel time with the common retimer from a fixed rest state; it never authorizes speed
// or replaces the timed collision, road, acceleration and stopping checks.
double StaticAvoidGeometryTravelTime(const CartesianPath& geometry,
    double target_speed,double dt,double wheelbase,const KinematicLimits& limits);
