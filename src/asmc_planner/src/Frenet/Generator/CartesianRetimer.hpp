#pragma once
#include "Frenet/Geometry/CartesianTypes.hpp"
#include "Frenet/Geometry/FrenetTypes.hpp"
#include "Frenet/Generator/GeneratorConfig.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Geometry/ObjectTypes.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"

// Geometry is an input, independent of the maneuver that produced it. Retiming
// uses physical arc length, the shared speed envelope and longitudinal generator.
// It must not treat a former reference's station as distance on a new lane.
bool RetimeCartesianGeometry(const CartesianPath& geometry,
    double initial_speed,double initial_acceleration,double target_speed,
    const PathGeneratorConfig&,const KinematicLimits&,const CurveSpeedConfig&,
    CartesianPath& output,FrenetPath& physical_motion);

struct CartesianFollowingCandidate {
    CartesianPath path;
    FrenetPath motion;
    int leader_id=-1;
};
// Longitudinal FOLLOW on the actual connector, before reference adoption.
// Reuses the ordinary following generators/limits. Selection and the complete
// collision/road certificate remain with the calling maneuver.
std::vector<CartesianFollowingCandidate> RetimeCartesianFollowingGeometry(
    const CartesianPath& geometry,double initial_speed,double initial_acceleration,
    double cruise_speed,const std::vector<TrackedObject>& objects,
    const VehicleShape&,const FollowingConfig&,const PathGeneratorConfig&,
    const KinematicLimits&,const CurveSpeedConfig&,double observation_age_s=0);
