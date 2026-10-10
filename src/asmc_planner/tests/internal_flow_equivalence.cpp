#include "Planning/IntentInput.hpp"
#include "Execution/Acceptance/CommandExecutionManager.hpp"
#include "Parser/BehaviorCommandAdapter.hpp"
#include <ros/serialization.h>
#include <random>
#include <iostream>
#include <stdexcept>
#include <limits>

using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;
struct InputPort {
  std::mutex input_mutex_;
  Command latest_v2_command_;
  bool has_v2_command_ = false;
};
template<class T> std::vector<uint8_t> bytes(const T& value) {
  std::vector<uint8_t> data(ros::serialization::serializationLength(value));
  ros::serialization::OStream stream(data.data(), data.size());
  ros::serialization::serialize(stream, value);
  return data;
}
Command wireRoundTrip(const Command& command) {
  auto data = bytes(command);
  ros::serialization::IStream stream(data.data(), data.size());
  Command decoded;
  ros::serialization::deserialize(stream, decoded);
  return decoded;
}
void require(bool condition) {
  if (!condition) throw std::runtime_error("direct input/transaction equivalence mismatch");
}
void compare(const frenet_planner_ros::CommandExecutionManager& a,
             const frenet_planner_ros::CommandExecutionManager& b) {
  auto x=a.snapshot(), y=b.snapshot();
  require(x.has_requested_command==y.has_requested_command &&
          x.requested_command_id==y.requested_command_id &&
          x.has_active_command==y.has_active_command &&
          x.active_command_id==y.active_command_id &&
          x.execution_phase==y.execution_phase && x.locked==y.locked &&
          x.terminal==y.terminal && bytes(a.active_command())==bytes(b.active_command()));
}
int main() {
  ros::Time::init();
  std::mt19937 random(20261010);
  asmc_planner::Planner coordinator;
  behavior_planner::BehaviorContextData decision;
  size_t cycles=0;
  for (int scenario=0;scenario<1000;++scenario) {
    InputPort legacy, direct;
    frenet_planner_ros::CommandExecutionManager a,b;
    for (uint8_t type=0;type<=6;++type) {
      a.SetImplementedManeuver(type,true); b.SetImplementedManeuver(type,true);
    }
    for (int cycle=0;cycle<200;++cycle,++cycles) {
      // Several Behavior events may precede one planning event: both paths
      // consume/reuse only the latest value (old subscriber queue size one).
      for (unsigned n=0,count=random()%4;n<count;++n) {
        Command command;
        command.command_id=random()%17;
        command.maneuver_type=random()%7;
        command.execution_request=random()%3;
        command.header.stamp=ros::Time(10+cycle,random()%1000000000);
        command.valid_until=command.header.stamp+ros::Duration(.5);
        command.header.frame_id="map";
        command.desired_speed_mps=double(random()%100)/10;
        if (random()%47==0) command.desired_speed_mps=std::numeric_limits<double>::quiet_NaN();
        command.primary_object_id=int(random()%100)-1;
        command.reason=std::string(random()%100,'x');
        command.source_link_id=std::to_string(random()%3);
        command.target_link_id=std::to_string(random()%4);
        command.has_target_pose=random()%2;
        command.target_pose_map.orientation.w=1;
        command.target_pose_map.position.x=double(random()%100)-50;
        // This is precisely the former ROS callback's value-copy operation.
        legacy.latest_v2_command_=wireRoundTrip(command);
        legacy.has_v2_command_=true;
        coordinator.ReceiveIntent(direct,{decision,command});
      }
      require(legacy.has_v2_command_==direct.has_v2_command_);
      require(bytes(legacy.latest_v2_command_)==bytes(direct.latest_v2_command_));
      const bool tracking=random()%2;
      if (legacy.has_v2_command_) {
        auto x=a.AcceptRequest({legacy.latest_v2_command_,tracking});
        auto y=b.AcceptRequest({direct.latest_v2_command_,tracking});
        require(x.command_accepted==y.command_accepted &&
                x.planning_target==y.planning_target &&
                x.execution_state==y.execution_state &&
                x.result_code==y.result_code && x.detail==y.detail && x.transaction_started==y.transaction_started);
      }
      switch (random()%6) {
        case 0:a.NotifyPlanSucceeded();b.NotifyPlanSucceeded();break;
        case 1:require(a.NotifyCommitted()==b.NotifyCommitted());break;
        case 2:require(a.NotifyExecutionStarted()==b.NotifyExecutionStarted());break;
        case 3:require(a.NotifyManeuverCompleted()==b.NotifyManeuverCompleted());break;
        case 4:a.NotifyPlanFailed(false);b.NotifyPlanFailed(false);break;
        default:break;
      }
      compare(a,b);
    }
  }
  std::cout<<cycles<<" direct-call cycles identical: input bytes, requested/accepted IDs, lifecycle, locks, terminal state\n";
}
