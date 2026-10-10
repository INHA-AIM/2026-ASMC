#pragma once
#include <cstdint>
#include "asmc_msgs/BehaviorContext.h"
#include "asmc_msgs/UrbanFeatureDebug.h"
#include "Behavior/Context/behavior_context.hpp"
#include "Behavior/Feature/urban_feature.hpp"
namespace behavior_planner {
using UrbanFeatureDebug = asmc_msgs::UrbanFeatureDebug;
bool SameCommandIdentity(const asmc_msgs::BehaviorContext& lhs,
                         const asmc_msgs::BehaviorContext& rhs);
class MsgConverter { public: static asmc_msgs::BehaviorContext toMsg(const BehaviorContextData&, uint32_t command_id); static UrbanFeatureDebug toDebugMsg(const UrbanFeature&, const BehaviorContextData&); };
}
