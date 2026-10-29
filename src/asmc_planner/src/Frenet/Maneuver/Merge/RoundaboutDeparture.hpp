#pragma once
#include "Frenet/Maneuver/Merge/RoundaboutMerge.hpp"
#include "Frenet/Maneuver/Merge/RoundaboutPrediction.hpp"

struct MissingDepartureCheck {
    bool safe=true;
    const char* status="none";
    const char* reason="none";
    int blocking_id=-1;
    double age=-1, time=-1, shift=0;
    std::size_t models_checked=0, non_interference_proven=0;
};

// No cached exemptions: every call checks this complete candidate, including
// rear clearance and both temporal gaps, against every retained observation.
inline MissingDepartureCheck CheckMissingRoundaboutDeparture(
    const CartesianPath& path,const RoundaboutMergeZone& zone,
    const std::vector<RoundaboutPrediction::MissingDepartureObservation>& missing,
    const VehicleShape& shape,const CollisionCheckConfig& collision,double dt,
    double front_gap,double rear_gap) {
    MissingDepartureCheck result;
    for(const auto& object:missing) {
        const bool supported=!object.models.empty();
        const auto& models=supported?object.models:object.relevance_models;
        if(models.empty()) {
            result.safe=false;result.status="unverifiable";result.reason=object.reason;
        } else {
            const auto check=CheckRoundaboutCandidate(path,zone,models,shape,collision,
                dt,front_gap,rear_gap,true);
            if(!check.safe) {
                result.safe=false;
                result.status=check.blocking_object_id>=0?"potential_conflict":"unverifiable";
                result.reason=check.rejection_reason;result.time=check.blocking_time;
                result.shift=check.blocking_shift;
            } else if(supported)++result.models_checked;
            else ++result.non_interference_proven;
        }
        if(!result.safe) {
            result.blocking_id=object.id;result.age=object.age;return result;
        }
    }
    result.status=result.non_interference_proven?"non_interference_proven":
        (result.models_checked?"models_checked":"none");
    return result;
}
