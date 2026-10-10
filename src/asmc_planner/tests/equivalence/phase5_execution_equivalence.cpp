#include "phase5_legacy_manager.hpp"
#include "Execution/Acceptance/CommandExecutionManager.hpp"
#include <ros/serialization.h>
#include <random>
#include <iostream>
#include <stdexcept>
using C=asmc_msgs::BehaviorCommand;
using F=asmc_msgs::FrenetFeedback;
using New=frenet_planner_ros::CommandExecutionManager;
using Old=legacy_execution::CommandExecutionManager;
void require(bool x){if(!x)throw std::runtime_error("transition mismatch");}
template<class M> std::vector<uint8_t> bytes(const M& m){
  std::vector<uint8_t> out(ros::serialization::serializationLength(m));
  ros::serialization::OStream stream(out.data(),out.size());ros::serialization::serialize(stream,m);return out;
}
void state(const Old& a,const New& b){
  require(a.has_active_command()==b.has_active_command()&&a.command_id()==b.command_id()&&
    a.maneuver_type()==b.maneuver_type()&&a.execution_state()==b.execution_state()&&
    int(a.runtime_failure())==int(b.runtime_failure())&&a.terminal_latched()==b.terminal_latched()&&
    a.terminal_result_code()==b.terminal_result_code()&&a.terminal_result_detail()==b.terminal_result_detail()&&
    a.is_non_preemptible()==b.is_non_preemptible()&&a.completed_normally()==b.completed_normally()&&
    bytes(a.active_command())==bytes(b.active_command()));
  const auto& x=a.fingerprint(); const auto& y=b.fingerprint();
  require(frenet_planner_ros::SameCommandFingerprint(x,y));
  auto s=b.snapshot();require(s.active_command_id==a.command_id()&&s.execution_phase==a.execution_state());
}
void accept(Old& a,New& b,const C& c,bool tracking){
  bool allow=a.has_active_command()&&a.maneuver_type()==C::AVOID&&a.command_id()!=c.command_id&&tracking;
  auto x=a.Accept(c,allow);auto y=b.AcceptRequest({c,tracking});
  require(x.command_accepted==y.command_accepted&&int(x.planning_target)==int(y.planning_target)&&
    x.execution_state==y.execution_state&&x.result_code==y.result_code&&x.detail==y.detail&&
    x.transaction_started==y.transaction_started&&x.transaction_cancelled==y.transaction_cancelled);
  auto s=b.snapshot();require(s.has_requested_command&&s.requested_command_id==c.command_id);state(a,b);
}
void init(Old& a,New& b){for(uint8_t i=0;i<=6;++i){a.SetImplementedManeuver(i,true);b.SetImplementedManeuver(i,true);}}
C command(uint32_t id,uint8_t type){C c;c.command_id=id;c.maneuver_type=type;c.execution_request=C::PREPARE;
 c.desired_speed_mps=4;c.primary_object_id=42;c.target_pose_map.orientation.w=1;c.reason="request";return c;}
void ready_commit(Old& a,New& b){a.NotifyPlanSucceeded();b.NotifyPlanSucceeded();require(a.NotifyCommitted()==b.NotifyCommitted());}
int main(){
  // Enumerate request behavior, command identity, payload changes, every handshake
  // phase and the original TRACKING permission. Explicitly cover release/preemption.
  size_t exhaustive=0;
  for(int phase=0;phase<6;++phase)for(int tracking=0;tracking<2;++tracking)
    for(uint8_t type=0;type<=6;++type)for(int same=0;same<2;++same)for(int payload=0;payload<2;++payload)
      for(uint8_t req=0;req<=2;++req){
    Old a;New b;init(a,b);accept(a,b,command(100,C::AVOID),false);
    if(phase>=1){a.NotifyPlanSucceeded();b.NotifyPlanSucceeded();}
    if(phase>=2)require(a.NotifyCommitted()==b.NotifyCommitted());
    if(phase>=3)require(a.NotifyExecutionStarted()==b.NotifyExecutionStarted());
    if(phase==4)require(a.NotifyManeuverCompleted()==b.NotifyManeuverCompleted());
    if(phase==5){a.NotifyPlanFailed(true,F::INVALID_COMMAND,"lost");b.NotifyPlanFailed(true,F::INVALID_COMMAND,"lost");}
    auto c=command(same?100:101,type);c.execution_request=req;if(payload)c.desired_speed_mps+=1;
    accept(a,b,c,tracking);++exhaustive;
  }
  for(auto type:{C::FOLLOW,C::KEEP}){Old a;New b;init(a,b);accept(a,b,command(100,C::AVOID),false);ready_commit(a,b);
    require(a.NotifyExecutionStarted()==b.NotifyExecutionStarted());accept(a,b,command(101,type),false);
    require(b.command_id()==100&&b.snapshot().requested_command_id==101&&b.maneuver_type()==C::AVOID);
    require(a.NotifyManeuverCompleted()==b.NotifyManeuverCompleted());accept(a,b,command(101,type),false);require(b.command_id()==101);
  }
  std::mt19937 rng(20261010);size_t n=0;
  for(int scenario=0;scenario<1000;++scenario){Old a;New b;init(a,b);
    for(int cycle=0;cycle<200;++cycle,++n){uint32_t event=rng()%16;
      if(event<8){uint32_t id=(rng()%3)?a.command_id():rng();auto c=command(id,rng()%7);
        if(a.has_active_command()&&rng()%3==0)c=a.active_command();
        c.execution_request=rng()%3;c.header.stamp=ros::Time(1+cycle,0);c.valid_until=ros::Time(1000+cycle,0);
        if(rng()%5==0)c.desired_speed_mps+=double(int(rng()%3)-1)*1e-6;
        if(rng()%5==0)c.primary_object_id=rng()%100;
        accept(a,b,c,rng()%2);
      }else if(event==8){a.NotifyPlanSucceeded();b.NotifyPlanSucceeded();}
      else if(event==9){bool terminal=rng()%2;uint8_t code=rng()%10;std::string d=std::to_string(rng()%4);
        a.NotifyPlanFailed(terminal,code,d);b.NotifyPlanFailed(terminal,code,d);}
      else if(event==10)require(a.NotifyCommitted()==b.NotifyCommitted());
      else if(event==11)require(a.NotifyExecutionStarted()==b.NotifyExecutionStarted());
      else if(event==12)require(a.NotifyManeuverCompleted()==b.NotifyManeuverCompleted());
      else if(event==13){a.NotifyInputDataStale();b.NotifyInputDataStale();}
      else {uint8_t type=rng()%7;bool enabled=rng()%2;a.SetImplementedManeuver(type,enabled);b.SetImplementedManeuver(type,enabled);}
      state(a,b);
    }
  }
  std::cout<<n<<" random transition cycles + "<<exhaustive<<" exhaustive cases identical\n";
}
