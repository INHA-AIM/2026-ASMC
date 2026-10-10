#include "Planning/PlanningPort.hpp"
#include "Planning/RuntimeCycle.hpp"
namespace frenet_planner_ros {
bool PlanningPort::runV2PlanningCycle(const ros::Time& now) {
    return coordinator_.RunV2(*this, now);
}
}
