#include "Planning/PlanningPort.hpp"
#include "Parser/PlanningInput.hpp"
#include <algorithm>
#include <cmath>

namespace frenet_planner_ros {
using namespace asmc_planner::lane_reference;
namespace {
std::string occupiedHighwayLink(const HighwayReferenceStatus& status,const CartesianState& ego) {
    if(!status.lane||!status.lane->coversFootprint(ego.x,ego.y,ego.yaw))return {};
    const auto p=ProjectOnLaneReference(*status.lane,ego.x,ego.y,ego.yaw,ego.v);
    if(!p.valid||std::abs(p.d)>.5||std::abs(p.heading_error_rad)>.12)return {};
    double nearest=INFINITY;std::string link;
    for(const auto& id:status.lane->lanes()) {
        const auto* road=status.lane->map()->lane(id);if(!road)continue;
        for(const auto& q:road->centerline) {
            const double d=std::hypot(q.x-ego.x,q.y-ego.y);
            if(d<nearest){nearest=d;link=road->link_id;}
        }
    }
    return link;
}
}
ReferenceExecutionInput PlanningPort::referenceInput(const InputSnapshot& snapshot,
    const CartesianState& ego,const std::vector<TrackedObject>& objects,double now) {
    ReferenceExecutionInput i;
    i.observation={now,snapshot.vehicle.header.stamp.toSec(),snapshot.objects.header.stamp.toSec(),
        snapshot.has_vehicle&&snapshot.vehicle.valid,snapshot.has_objects,ego};
    i.objects=objects;i.sequence=++reference_input_sequence_;
    if(planner_.highway_route())i.map=planner_.highway_route()->map->identity();
    return i;
}

bool PlanningPort::runHighwayMerge(const asmc_msgs::BehaviorCommand& command,uint32_t feedback_id,
    const CommandAcceptance& acceptance,const InputSnapshot& snapshot,const CartesianState& ego,
    const std::vector<TrackedObject>& objects,const PlannerCommand& request,double now) {
    using Feedback=asmc_msgs::FrenetFeedback;
    const auto route=planner_.highway_route();
    std::size_t zone=route?route->zones.size():0;
    if(route)for(std::size_t z=0;z<route->zones.size();++z)
        if(route->zones[z].config.source_link==request.source_link_id&&
           route->map->lane(route->zones[z].config.target_lanes.back())->link_id==request.target_link_id)zone=z;
    const bool optional=command.maneuver_type==asmc_msgs::BehaviorCommand::LANE_CHANGE;
    const auto status=highway_reference_?highway_reference_->status():HighwayReferenceStatus{};
    const auto source_index=status.network?status.network->indexForLink(request.source_link_id):0;
    const bool dynamic=optional||(status.network&&status.lane&&source_index<status.network->lanes.size()&&
        status.lane.get()==status.network->lanes[source_index].get());
    std::string binding_detail;
    const bool bound=highway_reference_&&route&&(dynamic?
        highway_reference_->beginLaneTransition(command.command_id,request,ego,optional,binding_detail):
        (zone<route->zones.size()&&highway_reference_->begin(command.command_id,zone)));
    auto& merge_core=bound&&dynamic?highway_reference_->transitionPlanner():planner_;

    // The current feedback can be WAITING after a computation-only timeout.
    // A recent READY for this bound command remains handshake evidence, never
    // a safety certificate. Planning below must certify the CURRENT snapshot.
    const bool was_ready=bound&&merge_core.HighwayReadyWasIssued(now);
    const bool execute_requested=command.execution_request==asmc_msgs::BehaviorCommand::EXECUTE;
    if(command_manager_.execution_state()==Feedback::COMMITTED)command_manager_.NotifyExecutionStarted();
    FrenetPlanner::HighwayMergeResult result;
    CartesianPath path;
    std::vector<HighwayObservedBox> boxes;
    if(optional)for(const auto& object:objects)
        boxes.push_back({object.x,object.y,object.heading,object.length,object.width});
    bool target_pose_valid=true;
    if(optional&&bound&&!merge_core.highway_merge_committed()) {
        const auto& q=command.target_pose_map.orientation;
        const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        const auto p=ProjectOnLaneReference(highway_reference_->transitionTarget(),
            command.target_pose_map.position.x,command.target_pose_map.position.y,yaw,0);
        target_pose_valid=p.valid&&std::fabs(p.d)<=route->config.completion_lateral_m&&
            std::fabs(p.heading_error_rad)<=runtime_.lane_change_max_target_heading_error_rad;
    }
    const bool window_observed=!optional||merge_core.highway_merge_committed()||
        (bound&&target_pose_valid&&status.network&&HighwayTargetWindowObserved(highway_reference_->transitionTarget(),
            ego,boxes,status.network->policy));
    bool ok=bound&&window_observed&&(dynamic?
        merge_core.PlanHighwayTransition(*route,highway_reference_->transitionZone(),optional,
            ego,objects,request,now,snapshot.objects.header.stamp.toSec(),path,result,
            snapshot.vehicle.header.stamp.toSec(),highway_reference_->transitionTarget().line(),execute_requested):
        merge_core.PlanHighwayMerge(ego,objects,request,now,
            snapshot.objects.header.stamp.toSec(),path,result,snapshot.vehicle.header.stamp.toSec(),
            &highway_reference_->target(zone).line(),&planner().reference(),execute_requested));
    if(optional&&(!bound||!window_observed)) {
        // Optional eligibility can expire between Behavior and this queue.
        // Its rejection is not a failure of ordinary source-lane driving.
        const auto source=occupiedHighwayLink(status,ego);
        ok=!source.empty()&&planner().PlanHighwayLaneContinuation(ego,objects,request.target_speed,
            source,path);
        merge_core.RevokeHighwayMergeReadiness();
        result.detail=!target_pose_valid?"optional_target_pose_not_on_requested_lane":
            bound?"optional_target_window_not_observed":
            "optional_change_not_bound: "+binding_detail;
    }
    asmc_msgs::Trajectory trajectory;
    bool converted=ok&&ConvertCartesianPath(path,planner_.sample_dt(),snapshot.vehicle,trajectory)==ConversionStatus::OK;
    const auto fresh=[&] {
        const double finished=ros::Time::now().toSec();
        const double a=finished-snapshot.vehicle.header.stamp.toSec();
        const double b=finished-snapshot.objects.header.stamp.toSec();
        return route&&a>=0&&b>=0&&a<=route->config.source_max_age_s&&b<=route->config.source_max_age_s;
    };
    if(converted&&!fresh()) {
        merge_core.RevokeHighwayMergeReadiness();converted=false;result.ready=false;
        result.detail="inputs_expired_during_highway_planning";
    }
    bool committed_now=false,completed_now=false,recovered_now=false;
    if(converted&&!merge_core.highway_merge_committed()) {
        if(result.ready)command_manager_.NotifyPlanSucceeded();
        else command_manager_.NotifyPlanFailed(false);
        if(execute_requested&&was_ready&&merge_core.HighwayReadyWasIssued(now)&&
           result.ready&&merge_core.CanCommitHighwayMerge(result)) {
            asmc_msgs::Trajectory executing;
            if(ConvertCartesianPath(result.candidate_path,planner_.sample_dt(),snapshot.vehicle,executing)==ConversionStatus::OK&&
               fresh()&&command_manager_.NotifyCommitted()) {
                merge_core.ApplyHighwayMergeCommit(result);trajectory=std::move(executing);committed_now=true;
            }
        }
        if(result.output_requires_commit&&!committed_now) {
            // The fast-path output is lateral motion. Never publish it when
            // the command/conversion/freshness gate failed to take ownership.
            converted=false;result.ready=false;merge_core.RevokeHighwayMergeReadiness();
            result.detail="highway_commit_gate_rejected";
        }
    } else if(converted&&optional&&result.returning_to_source) {
        if(result.source_return_completed&&fresh()) {
            // The original reference/core never changed. Release the lateral
            // lock only after actual source footprint/alignment/settling and
            // a fresh source continuation were certified. No target adoption.
            command_manager_.NotifyPlanFailed(true,Feedback::COLLISION_BLOCKED,
                "optional_change_aborted_on_source_lane");
            highway_recovered_command_id_=command.command_id;recovered_now=true;
        }
    } else if(converted&&merge_core.highway_merge_committed()&&result.candidate_valid) {
        auto input=referenceInput(snapshot,ego,objects,ros::Time::now().toSec());
        std::string detail;
        if(highway_reference_->prepare(input,path,request.target_speed,detail)) {
            // Conversion and freshness precede COMPLETED. Commit adopts the
            // core, reference, ego/object projections and checked continuation
            // together on the same planning queue.
            const auto* frame=highway_reference_->preparedFrame();
            asmc_msgs::Trajectory next;
            if(frame&&ConvertCartesianPath(frame->path,planner_.sample_dt(),snapshot.vehicle,next)==ConversionStatus::OK&&fresh()) {
                input.observation.now_s=ros::Time::now().toSec();
                if(highway_reference_->commit(input,[&]{return command_manager_.NotifyManeuverCompleted();},detail)) {
                    trajectory=std::move(next);completed_now=true;merge_core.MarkHighwayReferenceCompleted();
                    ROS_INFO_STREAM("Highway reference adopted: zone="<<(dynamic?highway_reference_->transitionZone().config.name:route->zones[zone].config.name)
                        <<" epoch="<<highway_reference_->status().epoch);
                }
            }
        }
        if(!completed_now)result.detail+="; "+detail;
    }
    if(converted&&!fresh()) {
        converted=false;result.ready=false;merge_core.RevokeHighwayMergeReadiness();
        result.detail="inputs_expired_during_reference_preparation";
    }
    if(converted&&result.ready&&!merge_core.highway_merge_committed()&&
       command_manager_.execution_state()==Feedback::READY)
        merge_core.MarkHighwayReadyIssued(ros::Time::now().toSec());
    CommandAcceptance reply=acceptance;reply.execution_state=command_manager_.execution_state();
    if(recovered_now) {
        reply.result_code=Feedback::COLLISION_BLOCKED;
        reply.detail="optional_change_aborted_on_source_lane";
    }
    if(reply.result_code==Feedback::RESULT_NONE)reply.detail=completed_now?
        "highway_reference_and_continuation_adopted":result.detail;
    double ego_s=0;const bool projected=planner().ProjectEgoToS(ego,ego_s);
    if(!converted) {
        const bool terminal=(!bound&&!optional)||merge_core.last_failure_reason()==FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        command_manager_.NotifyPlanFailed(terminal);has_last_v2_trajectory_=false;
        reply.execution_state=command_manager_.execution_state();
        if(reply.result_code==Feedback::RESULT_NONE)
            reply.result_code=terminal?Feedback::INVALID_COMMAND:Feedback::TRANSIENT_GENERATION_FAILURE;
        const auto stop=MakeStopTrajectory(stopReferenceVehicle(snapshot),trajectoryDt(),asmc_msgs::Trajectory::FAILURE_GENERATION);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_id,reply,&stop,Feedback::TRAJECTORY_SAFE_STOP,false,projected,ego_s);return true;
    }
    last_v2_trajectory_=trajectory;has_last_v2_trajectory_=true;
    last_v2_trajectory_command_id_=command.command_id;v2_plan_fail_streak_=0;
    trajectory_pub_.publish(trajectory);publishDebugMarkersV2(trajectory,command);
    publishFeedbackV2(feedback_id,reply,&trajectory,(committed_now||merge_core.highway_merge_committed())?
        (result.candidate_valid?Feedback::TRAJECTORY_LOCKED:Feedback::TRAJECTORY_SAFE_STOP):Feedback::TRAJECTORY_NEW,
        (!recovered_now&&result.candidate_valid)||completed_now,projected,ego_s);return true;
}

bool PlanningPort::runInterruptedHighway(const asmc_msgs::BehaviorCommand& command,uint32_t feedback_id,
    const CommandAcceptance& acceptance,const InputSnapshot& snapshot,const CartesianState& ego,
    const std::vector<TrackedObject>& objects,double now) {
    using Feedback=asmc_msgs::FrenetFeedback;
    auto& refs=*highway_reference_;auto& core=refs.transitionPlanner();
    const auto route=planner_.highway_route();auto request=refs.interruptedRequest();
    if(command.desired_speed_mps>0)request.target_speed=std::min(request.target_speed,command.desired_speed_mps);
    CartesianPath path;FrenetPlanner::HighwayMergeResult result;
    bool ok=core.PlanHighwayTransition(*route,refs.transitionZone(),true,ego,objects,request,
        now,snapshot.objects.header.stamp.toSec(),path,result,snapshot.vehicle.header.stamp.toSec(),
        refs.transitionTarget().line());
    const auto fresh=[&] {
        const double t=ros::Time::now().toSec();
        const double a=t-snapshot.vehicle.header.stamp.toSec(),b=t-snapshot.objects.header.stamp.toSec();
        return a>=0&&b>=0&&a<=route->config.source_max_age_s&&b<=route->config.source_max_age_s;
    };
    asmc_msgs::Trajectory trajectory;
    bool converted=ok&&ConvertCartesianPath(path,core.sample_dt(),snapshot.vehicle,trajectory)==ConversionStatus::OK&&fresh();
    bool recovered=false;
    if(converted&&result.candidate_valid) {
        if(result.returning_to_source&&result.source_return_completed) {
            refs.finishInterruptedOnSource();recovered=true;
        } else if(!result.returning_to_source) {
            auto input=referenceInput(snapshot,ego,objects,ros::Time::now().toSec());std::string detail;
            if(refs.prepare(input,path,request.target_speed,detail)) {
                const auto* frame=refs.preparedFrame();asmc_msgs::Trajectory next;
                if(frame&&ConvertCartesianPath(frame->path,core.sample_dt(),snapshot.vehicle,next)==ConversionStatus::OK&&fresh()) {
                    input.observation.now_s=ros::Time::now().toSec();
                    // This restores physical reference ownership; it does not
                    // complete the new command or resurrect the preempted ID.
                    if(refs.commit(input,[]{return true;},detail)) {
                        trajectory=std::move(next);core.MarkHighwayReferenceCompleted();recovered=true;
                    }
                }
            }
            if(!recovered)result.detail+="; "+detail;
        }
    }
    auto reply=acceptance;
    if(!converted||!fresh()) {
        command_manager_.NotifyPlanFailed(false);has_last_v2_trajectory_=false;
        reply.execution_state=command_manager_.execution_state();
        reply.detail="interrupted_highway_recovery: "+result.detail;
        const auto stop=MakeStopTrajectory(stopReferenceVehicle(snapshot),trajectoryDt(),asmc_msgs::Trajectory::FAILURE_GENERATION);
        trajectory_pub_.publish(stop);publishFeedbackV2(feedback_id,reply,&stop,Feedback::TRAJECTORY_SAFE_STOP,false);return true;
    }
    if(recovered&&(command.maneuver_type==command.LANE_CHANGE||command.maneuver_type==command.MERGE)) {
        command_manager_.NotifyPlanFailed(true,Feedback::COLLISION_BLOCKED,"highway_reference_recovered_replan_request");
        highway_recovered_command_id_=command.command_id;
        reply.result_code=Feedback::COLLISION_BLOCKED;
    } else if(command.maneuver_type==command.KEEP||command.maneuver_type==command.FOLLOW)
        command_manager_.NotifyPlanSucceeded();
    else command_manager_.NotifyPlanFailed(false);
    reply.execution_state=command_manager_.execution_state();
    reply.detail=recovered?"interrupted_highway_reference_recovered":"interrupted_highway_recovery: "+result.detail;
    last_v2_trajectory_=trajectory;has_last_v2_trajectory_=true;last_v2_trajectory_command_id_=command.command_id;
    trajectory_pub_.publish(trajectory);publishDebugMarkersV2(trajectory,command);
    publishFeedbackV2(feedback_id,reply,&trajectory,Feedback::TRAJECTORY_NEW,
        recovered?command.maneuver_type!=command.LANE_CHANGE&&command.maneuver_type!=command.MERGE:result.candidate_valid);
    return true;
}

bool PlanningPort::continueRecoveredHighwayLane(const asmc_msgs::BehaviorCommand& command,
    uint32_t feedback_id,const CommandAcceptance& acceptance,const InputSnapshot& snapshot) {
    using Feedback=asmc_msgs::FrenetFeedback;
    if(!highway_reference_||highway_recovered_command_id_!=command.command_id||
       (command.maneuver_type!=asmc_msgs::BehaviorCommand::LANE_CHANGE&&command.maneuver_type!=asmc_msgs::BehaviorCommand::MERGE)||
       command_manager_.execution_state()!=Feedback::FAILED||
       !command_manager_.has_active_command()||command_manager_.command_id()!=command.command_id)return false;
    const auto& recovered=command_manager_.active_command();
    const auto route=planner_.highway_route();
    const auto fresh=[&] {
        const double now=ros::Time::now().toSec();
        const double ego_age=now-snapshot.vehicle.header.stamp.toSec();
        const double object_age=now-snapshot.objects.header.stamp.toSec();
        return route&&snapshot.has_vehicle&&snapshot.vehicle.valid&&snapshot.has_objects&&
            ego_age>=0&&ego_age<=route->config.source_max_age_s&&
            object_age>=0&&object_age<=route->config.source_max_age_s;
    };
    CartesianState ego{};std::vector<TrackedObject> objects;CartesianPath path;
    asmc_msgs::Trajectory trajectory;
    if(!fresh()||ConvertVehicleState(snapshot.vehicle,ego)!=ConversionStatus::OK||
       ConvertDetectedObjects(snapshot.objects,objects)!=ConversionStatus::OK)return false;
    const auto source=occupiedHighwayLink(highway_reference_->status(),ego);
    if(source.empty())return false;
    planner().SetObjectObservationTime(snapshot.objects.header.stamp.toSec());
    if(!planner().PlanHighwayLaneContinuation(ego,objects,recovered.desired_speed_mps,
            source,path)||
       convertReferencePath(path,planner_.sample_dt(),snapshot.vehicle,trajectory)!=ConversionStatus::OK||
       !fresh())return false;
    last_v2_trajectory_=trajectory;has_last_v2_trajectory_=true;
    last_v2_trajectory_command_id_=command.command_id;
    trajectory_pub_.publish(trajectory);
    auto reply=acceptance;reply.detail="optional_change_aborted_on_source_lane; source_continuation";
    // The requested lane change failed; the NEW source trajectory is valid.
    // These are independent axes of the existing feedback contract.
    publishFeedbackV2(feedback_id,reply,&trajectory,Feedback::TRAJECTORY_NEW,false);
    return true;
}

bool PlanningPort::tryMissionReferenceReturn(const InputSnapshot& snapshot,const CartesianState& ego,
    const std::vector<TrackedObject>& objects,const PlannerCommand& request,CartesianPath& path,
    asmc_msgs::Trajectory& trajectory,double) {
    if(!highway_reference_||&planner()==&planner_||
       (request.mode!=LANE_KEEPING&&request.mode!=FOLLOWING))return true;
    const auto fresh=[&] {
        const double now=ros::Time::now().toSec(),limit=planner_.highway_route()->config.source_max_age_s;
        const double a=now-snapshot.vehicle.header.stamp.toSec(),b=now-snapshot.objects.header.stamp.toSec();
        return a>=0&&b>=0&&a<=limit&&b<=limit;
    };
    auto input=referenceInput(snapshot,ego,objects,ros::Time::now().toSec());
    std::string detail;
    if(!highway_reference_->prepareMissionReturn(input,path,request.target_speed,detail))return fresh();
    const auto* frame=highway_reference_->preparedFrame();asmc_msgs::Trajectory next;
    if(!frame||ConvertCartesianPath(frame->path,planner_.sample_dt(),snapshot.vehicle,next)!=ConversionStatus::OK)return fresh();
    CartesianPath next_path=frame->path;input.observation.now_s=ros::Time::now().toSec();
    if(highway_reference_->commitMissionReturn(input,detail)) {
        path=std::move(next_path);trajectory=std::move(next);
        has_last_v2_trajectory_=false;
        ROS_INFO_STREAM("Official mission reference restored: epoch="<<highway_reference_->status().epoch);
    }
    return fresh();
}
}  // namespace frenet_planner_ros
