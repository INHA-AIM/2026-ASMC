#pragma once
#include "Planning/V1Cycle.hpp"
#include "Planning/V2Cycle.hpp"
namespace asmc_planner {
template<class RuntimePort>
void Planner::RunCycle(RuntimePort& port) {
    // Keep the original v2-priority arbitration and ROS-time sample. The
    // separate Behavior SteadyTimer continues updating intent at 10 Hz; each
    // 20 Hz planning cycle consumes/reuses the latest delivered intent.
    if (RunV2(port, ros::Time::now())) return;
    RunV1(port);
}
}
