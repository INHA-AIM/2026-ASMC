#pragma once

#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <limits>
#include <vector>

// A per-candidate policy, never a mutation of the global collision settings.
// IDs must name currently confirmed fixed occupancy. Other actors keep the
// ordinary timed/stationary hypotheses and nominal uncertainty margin.
struct StaticAvoidPolicy {
    enum Mode { NORMAL=0, REDUCED_MARGIN=1, CONTACT_PROGRESS=2 };
    Mode mode=NORMAL;
    double margin=.45;
    std::vector<int> object_ids;
    // Cooperative search state only; never changes collision eligibility.
    int search_level=0;
};

bool IsStaticAvoidProgressObject(const TrackedObject& object);
bool StaticAvoidTrajectoryClear(const CartesianPath& path,
    const std::vector<TrackedObject>& objects,const VehicleShape& shape,
    const CollisionCheckConfig& nominal,double dt,const StaticAvoidPolicy* policy,
    int* blocker=nullptr,bool* recovery=nullptr,
    const std::vector<int>* initial_recovery_ids=nullptr);

struct StaticAvoidContact {
    double minimum_clearance=std::numeric_limits<double>::infinity();
    double cost=0;
    bool physical_overlap=false;
};
StaticAvoidContact MeasureStaticAvoidContact(const CartesianPath& geometry,
    const std::vector<TrackedObject>& objects,const VehicleShape& shape,
    const StaticAvoidPolicy& policy);
