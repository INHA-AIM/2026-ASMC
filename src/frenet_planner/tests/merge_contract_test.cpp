// Contract-only tests: enabling MERGE here does not enable its ROS runtime.
#include "command_execution_manager.hpp"
#include <iostream>

using namespace frenet_planner_ros;
using C=asmc_msgs::BehaviorCommand;
using F=asmc_msgs::FrenetFeedback;

C Merge(uint32_t id) {
    C c; c.header.frame_id="map";c.command_id=id;
    c.maneuver_type=C::MERGE;c.merge_type=C::MERGE_ROUNDABOUT;
    c.direction=C::DIRECTION_NONE;c.execution_request=C::PREPARE;
    c.desired_speed_mps=8;c.primary_object_id=-1;
    c.preferred_front_object_id=-1;c.preferred_rear_object_id=-1;
    c.desired_time_gap_s=-1;c.minimum_gap_m=-1;
    c.valid_until=ros::Time(1001);return c;
}
int main() {
    int failures=0;
    auto check=[&](bool ok,const char* name){
        std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';if(!ok)++failures;
    };
    const ros::Time now(1000);
    auto c=Merge(1);
    CommandExecutionManager closed;
    auto a=closed.Accept(c);
    check(a.planning_target==PlanningTarget::NONE,
          "production capability gate prevents unimplemented MERGE planning");
    CommandExecutionManager m;
    m.SetImplementedManeuver(C::MERGE,true);
    m.SetImplementedManeuver(C::KEEP,true);
    m.SetImplementedManeuver(C::EMERGENCY_STOP,true);
    a=m.Accept(c);
    check(a.command_accepted&&a.execution_state==F::PREPARING,
          "valid MERGE enters PREPARING");
    check(!m.NotifyCommitted(),"PREPARING cannot commit");
    m.NotifyPlanFailed(false);
    check(m.execution_state()==F::WAITING,"no safe plan is WAITING, not FAILED");
    m.NotifyPlanSucceeded();
    check(m.execution_state()==F::READY,"safe plan promotes WAITING to READY");
    m.NotifyPlanFailed(false);
    check(!m.NotifyCommitted(),"unsafe recheck revokes READY and blocks commit");
    m.NotifyPlanSucceeded();
    auto wrong=c;wrong.merge_type=C::MERGE_HIGHWAY;
    check(!m.Accept(wrong).command_accepted,
          "same ID cannot change merge geometry/type");
    c.execution_request=C::EXECUTE;
    check(m.Accept(c).command_accepted,
          "PREPARE and EXECUTE retain the transaction ID");
    check(m.NotifyCommitted()&&m.NotifyExecutionStarted(),
          "READY advances through COMMITTED to EXECUTING");
    auto keep=Merge(2);keep.maneuver_type=C::KEEP;keep.merge_type=C::MERGE_NONE;
    keep.execution_request=C::EXECUTE;
    a=m.Accept(keep);
    check(a.result_code==F::TOO_LATE_TO_CANCEL&&a.planning_target==PlanningTarget::ACTIVE_TRANSACTION,
          "KEEP cannot replace executing MERGE and rejected payload is not planned");
    auto cancel=c;cancel.execution_request=C::CANCEL;
    check(m.Accept(cancel).result_code==F::TOO_LATE_TO_CANCEL,"CANCEL is rejected after COMMIT");
    m.NotifyPlanFailed(false);
    check(m.execution_state()==F::EXECUTING,
          "temporary collision failure does not erase committed maneuver");
    check(m.NotifyManeuverCompleted(),"EXECUTING can complete");
    check(m.Accept(c).planning_target==PlanningTarget::NONE,
          "completed transaction does not execute again under same ID");
    c=Merge(3);m.Accept(c);m.NotifyPlanSucceeded();
    c.execution_request=C::EXECUTE;m.Accept(c);
    m.NotifyCommitted();m.NotifyExecutionStarted();
    auto emergency=keep;emergency.command_id=4;emergency.maneuver_type=C::EMERGENCY_STOP;
    check(m.Accept(emergency).command_accepted,
          "emergency STOP preempts executing MERGE");
    auto invalid=Merge(5);invalid.primary_object_id=7;
    check(ValidateBehaviorCommand(invalid,now).code==CommandValidationCode::INVALID_COMMAND,
          "MERGE does not consume a FOLLOW primary target ID");
    return failures?1:0;
}
