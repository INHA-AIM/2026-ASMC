#pragma once
#include "Execution/HighwayLaneChange.hpp"
#include <cstdint>
namespace asmc_planner::lane_reference {
struct HighwayReferenceStatus {
  std::shared_ptr<const LaneReference> lane;
  std::vector<bool> completed;
  std::shared_ptr<const HighwayLaneChangeNetwork> network;
  std::uint64_t epoch=1;
  bool transition_interrupted=false;
};
}
