#pragma once
#include <string>
#include "behavior_planner/context/behavior_types.hpp"
namespace behavior_planner { class TrafficLightProvider { public: virtual ~TrafficLightProvider()=default; virtual TrafficLightState getState(const std::string&, SignalMovement movement=SignalMovement::UNKNOWN)const=0; virtual std::string getRawLabel()const{return {};} }; }
