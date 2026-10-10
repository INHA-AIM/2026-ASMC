#pragma once
#include "Behavior/Context/behavior_context.hpp"
#include "Behavior/Feature/urban_feature.hpp"
namespace behavior_planner { class BehaviorScorer { public: BehaviorScore score(const UrbanFeature&,const HardConstraint&,bool persisted_follow_target=false)const; }; }
