#pragma once
#include "Map/HdLaneMap.hpp"
#include <asmc_msgs/HdMapMeta.h>
#include <asmc_msgs/LaneArray.h>
#include <asmc_msgs/LaneMarkingArray.h>
#include <asmc_msgs/DrivableAreaArray.h>

namespace asmc_planner::lane_reference {
// All four global layers must describe one build. Returns a new immutable
// snapshot or nullptr; an existing snapshot is never partially overwritten.
std::shared_ptr<const HdLaneMap> ConvertHdMapLaneSnapshot(
    const asmc_msgs::HdMapMeta& meta,const asmc_msgs::LaneArray& lanes,
    const asmc_msgs::LaneMarkingArray& markings,const asmc_msgs::DrivableAreaArray& areas,
    std::string& error);
}
