#pragma once
#include <cstdint>
#include "asmc_msgs/BehaviorContext.h"
#include "behavior_planner/UrbanFeatureDebug.h"
#include "behavior_planner/context/behavior_context.hpp"
#include "behavior_planner/feature/urban_feature.hpp"
namespace behavior_planner {
bool SameCommandIdentity(const asmc_msgs::BehaviorContext& lhs,
                         const asmc_msgs::BehaviorContext& rhs);
class MsgConverter { public: static asmc_msgs::BehaviorContext toMsg(const BehaviorContextData&, uint32_t command_id); static UrbanFeatureDebug toDebugMsg(const UrbanFeature&, const BehaviorContextData&); };
}
