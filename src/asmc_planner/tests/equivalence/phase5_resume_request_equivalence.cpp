#include "phase5_legacy_request.hpp"
#include "Execution/IntentEncoder.hpp"
#include <ros/serialization.h>
#include <random>
#include <iostream>
#include <stdexcept>
using namespace behavior_planner;
template<class M> std::vector<uint8_t> bytes(const M& m){std::vector<uint8_t> v(ros::serialization::serializationLength(m));ros::serialization::OStream s(v.data(),v.size());ros::serialization::serialize(s,m);return v;}
int main(){std::mt19937 rng(20261010);for(int scenario=0;scenario<200;++scenario){
  LegacyV2CommandManager a;V2CommandManager b;
  for(int cycle=0;cycle<1000;++cycle){BehaviorContextData c;
    c.selected_behavior=static_cast<BehaviorType>(rng()%9);c.maneuver_phase=static_cast<ManeuverPhase>(rng()%7);
    c.hard.force_stop=rng()%31==0;c.target.current_link_id=std::to_string(rng()%3);
    c.target.target_link_id=std::to_string(rng()%4);c.target.has_target_pose=rng()%2;
    c.target.target_x=(rng()%6)*1e-4;c.target.target_y=rng()%3;c.target.target_yaw=(rng()%3)*.1;
    c.target.desired_speed=rng()%12;c.target.avoidance_object_id=rng()%4;c.target.lead_vehicle_id=rng()%4;
    c.target.follow_time_gap=1+double(rng()%8)/10;c.target.follow_min_gap=rng()%8;
    bool approach=rng()%2;c.extra_flags["intersection_approach_speed_limit"]=approach;
    asmc_msgs::FrenetFeedback f;f.command_id=rng()%2?a.activeCommandId():rng()%15;f.execution_state=rng()%8;
    const auto* feedback=rng()%5?&f:nullptr;ros::Time now(1+cycle,scenario);double validity=.1*(rng()%6);
    auto x=a.update(c,feedback,now,validity);
    auto y=b.update(c,feedback,now,validity);
    if(bytes(x)!=bytes(y)||a.activeCommandId()!=b.activeCommandId())throw std::runtime_error("request transaction mismatch");
  }
}std::cout<<"200000 request transaction cycles identical (serialized payload + command_id)\n";}
