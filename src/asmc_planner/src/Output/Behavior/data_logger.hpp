#pragma once
#include "Behavior/Context/behavior_context.hpp"
#include "Behavior/Feature/urban_feature.hpp"
namespace behavior_planner { class DataLogger { public:void write(const UrbanFeature&,const BehaviorContextData&); }; }
