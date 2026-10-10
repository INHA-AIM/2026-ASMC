#pragma once
namespace ros { class Time; }
namespace asmc_planner {
struct RequestedIntent;
// Top-level scheduling/validation/output policy. A legacy ROS adapter supplies
// transport resources; it never starts a second timer or publisher here.
class Planner final {
 public:
  template<class RuntimePort> void ReceiveIntent(RuntimePort&, const RequestedIntent&);
  template<class RuntimePort> void RunCycle(RuntimePort& port);
  template<class RuntimePort> void RunV1(RuntimePort& port);
  template<class RuntimePort> bool RunV2(RuntimePort& port, const ros::Time& now);
};
}
