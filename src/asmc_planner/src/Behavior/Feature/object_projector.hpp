#pragma once
#include "Behavior/Feature/urban_feature.hpp"
#include "Behavior/Map/route_manager.hpp"
namespace behavior_planner { class ObjectProjector { public: ObjectFeature project(const TrackedObject&,const EgoFeature&,const RouteManager&)const; }; }
