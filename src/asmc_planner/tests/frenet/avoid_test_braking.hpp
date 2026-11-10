#pragma once
#include "Frenet/frenet_planner.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"

// A newly appearing blocker may require braking before changing the current
// steering direction. Advance an independent conservative braking plant rather
// than freezing a moving input for hundreds of planning cycles. Keep steering
// curvature during braking and check actual body/road at 20 ms substeps.
inline bool BrakeAvoidTestVehicle(CartesianState& ego,const FrenetPlanner& planner,
    const std::vector<TrackedObject>& objects) {
    constexpr double dt=.02,decel=5.0;
    for(int tick=0;tick<5;++tick) {
        const double next=std::max(0.0,ego.v-decel*dt);
        const double ds=(ego.v+next)*.5*dt;
        ego.x+=ds*std::cos(ego.yaw+ego.kappa*ds*.5);
        ego.y+=ds*std::sin(ego.yaw+ego.kappa*ds*.5);
        ego.yaw+=ego.kappa*ds;ego.v=next;ego.a=next>0?-decel:0;
        if(!planner.road_map().FootprintInside(ego.x,ego.y,ego.yaw,{2.161,4.635,.79}))return false;
        const auto body=MakeEgoFootprint(ego,{1.9,4.635,.79},0);
        for(const auto& object:objects)
            if(geometry::ConvexPolygonDistance(body,PredictObstacleFootprint(object,0))<=1e-9)return false;
    }
    return true;
}
