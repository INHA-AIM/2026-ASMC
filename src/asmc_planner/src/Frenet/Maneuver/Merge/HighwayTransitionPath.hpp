#pragma once
#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Common/CartesianTypes.hpp"
#include "Frenet/Common/FrenetTypes.hpp"
#include "Frenet/Reference/Reference.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"

// Spatial geometry survives replanning; the longitudinal schedule does not.
// Retiming this curve must not silently re-fit its lateral polynomial.
struct HighwayTransitionGeometry {
    bool valid=false;
    double begin_s=0,terminal_s=0;
    QuinticPolynomial lateral{};
};

HighwayTransitionGeometry MakeHighwayTransitionGeometry(const RefLine& mission,
    const RefLine& target,double start_s,double d,double slope,double second,double terminal_s);
bool SampleHighwayTransitionPath(const RefLine& mission,const RefLine& target,
    const FrenetPath& motion,const HighwayTransitionGeometry&,CartesianPath& output);
bool HighwayTransitionKinematicsValid(const CartesianPath&,const FrenetPath&,
    const KinematicLimits&,std::size_t samples);
// Bounds the current body relative to the retained reference curve. Callers
// use the returned displacement as an extra road/collision footprint guard.
bool HighwayTransitionTrackingGuard(const RefLine& mission,const RefLine& target,
    const HighwayTransitionGeometry&,double station,const CartesianState& ego,
    const VehicleShape&,double position_limit,double heading_limit,double curvature_limit,
    double& footprint_guard);

// The longitudinal schedule and lateral connector describe ONE physical path.
// The endpoint is a station on the mission line, not a constant lane offset:
// after the connector the path follows the prepared target through its joins.
bool BuildHighwayTransitionPath(const RefLine& mission,const RefLine& target,
    const FrenetPath& motion,double initial_d,double initial_slope,
    double initial_second,double terminal_s,const KinematicLimits&,
    CartesianPath& output);
