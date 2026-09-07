#pragma once
#include "behavior_planner/context/behavior_types.hpp"
namespace behavior_planner { class Fsm { public:BehaviorState update(BehaviorType); bool updateEmergency(bool requested,double min_ttc,double timestamp); void setEmergencyRelease(double exit_ttc,double hold_s); bool emergencyLatched()const{return emergency_latched_;} private:bool emergency_latched_=false;double emergency_safe_since_=-1.0;double emergency_last_timestamp_=-1.0;double emergency_exit_ttc_=3.0;double emergency_release_hold_s_=0.5; }; }
