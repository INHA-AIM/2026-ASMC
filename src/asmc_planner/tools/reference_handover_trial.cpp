// Opt-in, single-maneuver execution trial. Does not instantiate BehaviorInput.
// Normal ProductionRuntime and its command/route policy are unchanged.
#include "Execution/LaneReferenceExecution.hpp"
#include "Execution/ReferenceTrialObjects.hpp"
#include "Parser/HdMapLaneInput.hpp"
#include "Parser/PlanningInput.hpp"
#include "Planning/GpsModeSource.hpp"
#include "Planning/Compatibility/lane_change_commit_coordinator.hpp"
#include <asmc_msgs/HdMapMeta.h>
#include <asmc_msgs/LaneArray.h>
#include <asmc_msgs/LaneMarkingArray.h>
#include <asmc_msgs/DrivableAreaArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <future>
#include <sstream>
#include <algorithm>
#include <cmath>

using namespace asmc_planner::lane_reference;
using namespace frenet_planner_ros;
using Command=asmc_msgs::BehaviorCommand;
using Feedback=asmc_msgs::FrenetFeedback;
namespace {
struct Prepared {
  std::shared_ptr<const HdLaneMap> map;
  std::shared_ptr<const LaneReference> target;
  std::shared_ptr<const LaneRegion> source,trial_corridor;
  std::unique_ptr<FrenetPlanner> source_core,target_core;
};
class Trial {
 public:
  Trial():pnh_("~") {
    std::string config,planner_config;
    if(!pnh_.getParam("config",config)||!pnh_.getParam("planner_config",planner_config))
      throw std::runtime_error("trial config and planner_config are required");
    settings_=YAML::LoadFile(config);planner_config_=planner_config;
    const auto planner_settings=YAML::LoadFile(planner_config);
    const double planning_hz=planner_settings["runtime"]["planning_frequency_hz"].as<double>();
    command_validity_s_=planner_settings["runtime"]["timeouts"]["behavior"].as<double>();
    const auto main=planner_settings["planner"];
    max_target_lateral_=planner_settings["runtime"]["interface_validation"]["lane_change"]["max_target_lateral_offset"].as<double>();
    target_projection_window_=planner_settings["runtime"]["interface_validation"]["lane_change"]["projection_search_window"].as<double>();
    auto r=main["reference"];
    options_.reference={r["closure_tolerance_m"].as<double>(),r["smoothing_window_m"].as<double>(),
      r["resample_ds"].as<double>(),r["relax_max_iter"].as<int>(),r["relax_step"].as<double>(),r["ego_projection_search_window"].as<double>()};
    auto v=main["vehicle_shape"];options_.vehicle={v["length"].as<double>()-v["rear_overhang"].as<double>(),
      v["rear_overhang"].as<double>(),v["width"].as<double>()/2};
    options_.max_curvature=main["kinematic_limits"]["max_curvature"].as<double>();
    options_.max_lateral_accel=main["kinematic_limits"]["max_lateral_accel"].as<double>();
    const auto trial_reference=settings_["reference"];
    options_.max_join_gap_m=trial_reference["max_join_gap_m"].as<double>();
    options_.sample_distance_m=trial_reference["sample_distance_m"].as<double>();options_.sample_yaw_rad=trial_reference["sample_yaw_rad"].as<double>();
    options_.footprint_margin_m=trial_reference["footprint_margin_m"].as<double>();options_.max_samples=trial_reference["max_samples"].as<std::size_t>();
    max_age_=settings_["input_max_age_s"].as<double>();speed_=settings_["speed_mps"].as<double>();
    arrival_ahead_=settings_["arrival_ahead_m"].as<double>();
    if(!std::isfinite(speed_)||speed_<=0||speed_>25.0/3.6||!std::isfinite(max_age_)||max_age_<=0||
       !std::isfinite(arrival_ahead_)||arrival_ahead_<=0||!std::isfinite(max_target_lateral_)||max_target_lateral_<=0||
       !std::isfinite(planning_hz)||planning_hz<=0||!std::isfinite(command_validity_s_)||command_validity_s_<=0||
       !std::isfinite(target_projection_window_)||target_projection_window_<=0)
      throw std::runtime_error("invalid bounded trial settings");
    auto lc=main["lane_change"];
    alignment_={lc["lateral_position_tolerance"].as<double>(),lc["heading_tolerance"].as<double>(),
      lc["lateral_speed_tolerance"].as<double>(),settings_["alignment_settle_s"].as<double>(),max_age_,max_age_,settings_["max_pose_gap_s"].as<double>()};
    auto c=settings_["continuity"];continuity_={c["overlap_s"].as<double>(),c["position_m"].as<double>(),
      c["heading_rad"].as<double>(),c["curvature_inv_m"].as<double>(),c["speed_mps"].as<double>(),c["acceleration_mps2"].as<double>()};
    pnh_.param("enable_control",control_,false);
    std::vector<std::string> nodes;ros::master::getNodes(nodes);
    if(std::find(nodes.begin(),nodes.end(),"/asmc_planner")!=nodes.end())
      throw std::runtime_error("stop the normal /asmc_planner before running the reference trial");
    if(control_)mode_source_.reset(new asmc_planner::GpsModeSource(nh_));
    std::string state_topic,objects_topic;
    pnh_.param<std::string>("state_topic",state_topic,"/planning/vehicle/state");
    pnh_.param<std::string>("objects_topic",objects_topic,"/planning/objects");
    state_sub_=nh_.subscribe<asmc_msgs::VehicleState>(state_topic,1,[this](const asmc_msgs::VehicleState::ConstPtr& m){vehicle_=*m;vehicle_seen_=ros::SteadyTime::now();have_vehicle_=true;++sequence_;});
    objects_sub_=nh_.subscribe<planning_inputs::DetectedObjectArray>(objects_topic,1,[this](const planning_inputs::DetectedObjectArray::ConstPtr& m){objects_=*m;objects_seen_=ros::SteadyTime::now();have_objects_=true;++sequence_;});
    meta_sub_=nh_.subscribe<asmc_msgs::HdMapMeta>("/hd_map/meta",1,[this](const asmc_msgs::HdMapMeta::ConstPtr& m){meta_=*m;});
    lanes_sub_=nh_.subscribe<asmc_msgs::LaneArray>("/hd_map/global/lanes",1,[this](const asmc_msgs::LaneArray::ConstPtr& m){lanes_=*m;});
    markings_sub_=nh_.subscribe<asmc_msgs::LaneMarkingArray>("/hd_map/global/lane_markings",1,[this](const asmc_msgs::LaneMarkingArray::ConstPtr& m){markings_=*m;});
    areas_sub_=nh_.subscribe<asmc_msgs::DrivableAreaArray>("/hd_map/global/drivable_areas",1,[this](const asmc_msgs::DrivableAreaArray::ConstPtr& m){areas_=*m;});
    trajectory_pub_=nh_.advertise<asmc_msgs::Trajectory>(control_?"/planning/trajectory":"/planning/reference_handover/trajectory_preview",1);
    command_pub_=nh_.advertise<Command>("/planning/reference_handover/command",1);
    feedback_pub_=nh_.advertise<Feedback>("/planning/reference_handover/feedback",1);
    diagnostics_pub_=nh_.advertise<std_msgs::String>("/planning/reference_handover/diagnostics",1,true);
    arm_service_=pnh_.advertiseService("arm",&Trial::arm,this);
    manager_.SetImplementedManeuver(Command::KEEP,true);
    manager_.SetImplementedManeuver(Command::FOLLOW,true);
    manager_.SetImplementedManeuver(Command::STOP,true);
    manager_.SetImplementedManeuver(Command::LANE_CHANGE,true);
    timer_=nh_.createSteadyTimer(ros::WallDuration(1.0/planning_hz),&Trial::tick,this);
    ROS_INFO("Reference handover trial: control=%s, explicit ~arm required, no highway gap policy",control_?"true":"false");
  }
 private:
  bool input(ReferenceExecutionInput& i) const {
    const auto now=ros::Time::now();const auto steady=ros::SteadyTime::now();
    if(!have_vehicle_||!have_objects_||(steady-vehicle_seen_).toSec()>max_age_||
       (steady-objects_seen_).toSec()>max_age_||objects_.header.frame_id!="map"||
       vehicle_.header.stamp.isZero()||objects_.header.stamp.isZero()||
       (now-vehicle_.header.stamp).toSec()<0||(now-objects_.header.stamp).toSec()<0||
       (now-vehicle_.header.stamp).toSec()>max_age_||(now-objects_.header.stamp).toSec()>max_age_)return false;
    if(ConvertVehicleState(vehicle_,i.observation.ego)!=ConversionStatus::OK||
       ConvertDetectedObjects(objects_,i.objects)!=ConversionStatus::OK)return false;
    i.observation.now_s=now.toSec();i.observation.ego_stamp_s=vehicle_.header.stamp.toSec();
    i.observation.objects_stamp_s=objects_.header.stamp.toSec();i.observation.ego_valid=i.observation.objects_valid=true;
    i.sequence=sequence_;i.map={meta_.header.frame_id,meta_.build_id,meta_.map_id,meta_.geometry_sha256};
    const auto matches=[&](const auto& layer){return layer.complete&&layer.header.frame_id==meta_.header.frame_id&&
      layer.build_id==meta_.build_id&&layer.map_id==meta_.map_id&&layer.geometry_sha256==meta_.geometry_sha256;};
    return prepared_.map&&meta_.operational&&meta_.complete&&meta_.schema_major==1&&meta_.scope==asmc_msgs::HdMapMeta::SCOPE_GLOBAL&&i.map==prepared_.map->identity()&&
      matches(lanes_)&&matches(markings_)&&matches(areas_)&&meta_.lane_count==lanes_.lanes.size()&&
      meta_.lane_marking_count==markings_.markings.size()&&meta_.drivable_area_count==areas_.areas.size();
  }
  bool arm(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
    ReferenceExecutionInput i;
    response.success=false;
    if(armed_){response.message="trial_already_armed: restart the trial for another maneuver";return true;}
    if(!prepared_.map){response.message=preparation_error_.empty()?"map_preparing: wait for HD-map target reference prepared":"map_preparation_failed: "+preparation_error_;return true;}
    if(!input(i)){response.message="inputs_not_ready: vehicle/objects must be fresh and map layers must match";return true;}
    if(!TrialObjectsAllowStart(*prepared_.trial_corridor,i.objects,response.message))return true;
    if(i.observation.ego.v>speed_){response.message="ego_speed_exceeds_trial_speed_mps: "+std::to_string(speed_);return true;}
    if(!prepared_.source->coversFootprint(i.observation.ego.x,i.observation.ego.y,i.observation.ego.yaw,options_.vehicle,options_.footprint_margin_m)){
      response.message="ego_footprint_outside_source_lane: "+settings_["source_lane"].as<std::string>();return true;}
    const auto p=ProjectOnLaneReference(*prepared_.target,i.observation.ego.x,i.observation.ego.y,i.observation.ego.yaw,i.observation.ego.v);
    const double arrival=p.s+arrival_ahead_;
    if(!p.valid||arrival<prepared_.target->usableBegin()||arrival>=prepared_.target->usableEnd()-speed_*settings_["end_guard_s"].as<double>()){
      response.success=false;response.message="insufficient target-lane arrival and stopping horizon";return true;
    }
    const auto target_pose=Interpolate(prepared_.target->line(),arrival);
    double source_s,source_d;
    if(!prepared_.source_core->ProjectEgoToS(i.observation.ego,source_s)||
       !prepared_.source_core->ProjectPointToFrenet(target_pose.x,target_pose.y,source_s,
         target_projection_window_,max_target_lateral_,source_s,source_d)||
       std::abs(source_d)<options_.vehicle.half_width_m||std::abs(angle(target_pose.theta-i.observation.ego.yaw))>alignment_.heading_tolerance_rad){
      response.success=false;response.message="target pose does not agree with source projection/heading";return true;
    }
    session_.reset(new LaneReferenceExecution(std::move(prepared_.source_core),std::move(prepared_.target_core),
      prepared_.target,alignment_,continuity_,settings_["end_guard_s"].as<double>(),arrival));
    session_->planner().BindExecutionState(manager_.maneuvers());
    if(!session_->begin(1))throw std::runtime_error("trial handover binding rejected");
    core_command_.mode=source_d>0?LANE_CHANGE_LEFT:LANE_CHANGE_RIGHT;
    core_command_.has_target_pose_projection=true;core_command_.target_pose_projection_s=source_s;
    core_command_.target_pose_projection_d=source_d;core_command_.has_target_speed=true;core_command_.target_speed=speed_;
    command_.header.frame_id="map";command_.command_id=1;command_.maneuver_type=Command::LANE_CHANGE;
    command_.direction=source_d>0?Command::DIRECTION_LEFT:Command::DIRECTION_RIGHT;
    command_.merge_type=Command::MERGE_NONE;command_.execution_request=Command::PREPARE;
    command_.primary_object_id=command_.preferred_front_object_id=command_.preferred_rear_object_id=-1;
    command_.desired_speed_mps=speed_;command_.desired_time_gap_s=command_.minimum_gap_m=-1;
    command_.source_link_id=prepared_.map->lane(settings_["source_lane"].as<std::string>())->link_id;
    command_.target_link_id=prepared_.map->lane(settings_["target_lanes"][0].as<std::string>())->link_id;
    command_.has_target_pose=true;command_.target_pose_map.position.x=target_pose.x;command_.target_pose_map.position.y=target_pose.y;
    command_.target_pose_map.orientation.z=std::sin(target_pose.theta*.5);command_.target_pose_map.orientation.w=std::cos(target_pose.theta*.5);
    armed_=true;response.success=true;response.message="one empty-traffic LC armed; controller output follows enable_control";return true;
  }
  static double angle(double x){return std::atan2(std::sin(x),std::cos(x));}
  bool mapClear(const CartesianPath& path) const {
    if(path.x.size()<2||path.y.size()!=path.x.size()||path.yaw.size()!=path.x.size())return false;
    for(std::size_t n=0;n<path.x.size();++n)
      if(!prepared_.map->road()->coversFootprint(path.x[n],path.y[n],path.yaw[n],options_.vehicle,options_.footprint_margin_m)||
         prepared_.map->touchesSolid(path.x[n],path.y[n],path.yaw[n],options_.vehicle,options_.footprint_margin_m))return false;
    return true;
  }
  void startPreparation() {
    if(preparation_started_||!meta_.complete||!lanes_.complete||!markings_.complete||!areas_.complete)return;
    preparation_started_=true;
    const auto meta=meta_;const auto lanes=lanes_;const auto markings=markings_;const auto areas=areas_;
    const auto source_id=settings_["source_lane"].as<std::string>();
    const auto targets=settings_["target_lanes"].as<std::vector<std::string>>();
    const auto expected_build=settings_["expected_build_id"].as<std::string>();
    const auto expected_hash=settings_["expected_geometry_sha256"].as<std::string>();
    const auto options=options_;const auto config=planner_config_;
    future_=std::async(std::launch::async,[meta,lanes,markings,areas,source_id,targets,expected_build,expected_hash,options,config]{
      Prepared p;std::string error;
      p.map=ConvertHdMapLaneSnapshot(meta,lanes,markings,areas,error);
      if(!p.map)throw std::runtime_error(error);
      if(p.map->identity().build_id!=expected_build||p.map->identity().geometry_sha256!=expected_hash)throw std::runtime_error("unexpected trial map version");
      const auto result=BuildLaneReference(p.map,p.map->identity(),targets,options);
      if(!result)throw std::runtime_error(result.detail);p.target=result.candidate;
      p.source=p.map->corridor({source_id},error);
      if(!p.source)throw std::runtime_error(error);
      auto trial_lanes=targets;trial_lanes.insert(trial_lanes.begin(),source_id);
      p.trial_corridor=p.map->corridor(trial_lanes,error);
      if(!p.trial_corridor)throw std::runtime_error(error);
      const auto* source=p.map->lane(source_id);
      const auto target_id=targets.front();
      if(!source||(source->left_adjacent!=target_id&&source->right_adjacent!=target_id))
        throw std::runtime_error("trial target must be an immediate adjacent lane");
      p.source_core.reset(new FrenetPlanner);p.target_core.reset(new FrenetPlanner);
      if(!p.source_core->Init(config)||!p.target_core->Init(config))throw std::runtime_error("Frenet trial initialization failed");
      return p;
    });
  }
  void publish(const CartesianPath* path,bool valid,const std::string& detail) {
    asmc_msgs::Trajectory trajectory;
    const double dt=session_?session_->planner().sample_dt():.1;
    if(!path||!valid||ConvertCartesianPath(*path,dt,vehicle_,trajectory)!=ConversionStatus::OK){
      trajectory=MakeStopTrajectory(vehicle_,dt,asmc_msgs::Trajectory::FAILURE_GENERATION);valid=false;
    }
    trajectory_pub_.publish(trajectory);
    Feedback feedback;feedback.header.stamp=ros::Time::now();feedback.header.frame_id="map";
    feedback.command_id=command_.command_id;
    feedback.active_maneuver=manager_.maneuver_type();feedback.execution_state=manager_.execution_state();
    feedback.command_accepted=armed_;feedback.plan_valid=valid;feedback.result_detail=detail;
    feedback.active_leader_id=-1;feedback.selected_front_object_id=feedback.selected_rear_object_id=-1;
    feedback.blocking_object_id=-1;feedback.selected_cost=-1;feedback.maneuver_progress=-1;
    feedback.estimated_entry_time_s=-1;feedback.applied_speed_mps=valid&&!trajectory.points.empty()?trajectory.points.back().velocity:-1;
    feedback.trajectory_source=valid?Feedback::TRAJECTORY_NEW:Feedback::TRAJECTORY_SAFE_STOP;
    if(session_&&session_->adopted()){feedback.frenet_state_valid=valid;feedback.current_s=session_->frame()->ego.s;feedback.current_d=session_->frame()->ego.d;feedback.active_leader_id=session_->frame()->mode==FOLLOWING?session_->frame()->leader_id:-1;feedback.maneuver_progress=manager_.maneuver_type()==Command::LANE_CHANGE?1:-1;}
    feedback_pub_.publish(feedback);
    std::ostringstream out;out<<"armed="<<armed_<<" control="<<control_<<" plan_valid="<<valid<<" state="<<int(feedback.execution_state)<<" detail="<<detail;
    if(session_){out<<" ref_epoch="<<session_->epoch();if(session_->adopted()){const auto& f=*session_->frame();out<<" route_s="<<f.route_s<<" lane_s="<<f.ego.s<<" lane_d="<<f.ego.d<<" leader="<<f.leader_id<<" objects="<<f.objects.size()<<" input_sequence="<<f.input_sequence<<" ego_stamp="<<f.ego_stamp_s<<" objects_stamp="<<f.objects_stamp_s;}}
    std_msgs::String message;message.data=out.str();diagnostics_pub_.publish(message);
  }
  void tick(const ros::SteadyTimerEvent&) {
    startPreparation();
    if(future_.valid()&&future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
      try{prepared_=future_.get();ROS_INFO("HD-map target reference prepared; call ~arm in the selected source lane");}
      catch(const std::exception& e){preparation_error_=e.what();ROS_ERROR_STREAM(preparation_error_);}
    }
    ReferenceExecutionInput i;
    if(!armed_||!input(i)){
      // A receive-age/frame/map failure must also break the settling history,
      // even when new pose stamps keep arriving during an object outage.
      if(session_&&!session_->adopted()){
        i.observation.objects_valid=false;std::string ignored;
        session_->prepare(i,CartesianPath{},speed_,ignored);
      }
      publish(nullptr,false,!preparation_error_.empty()?preparation_error_:"waiting_for_arm_or_fresh_matching_inputs");return;
    }
    std::string detail;CartesianPath path;bool ok=false;
    command_.header.stamp=ros::Time::now();command_.valid_until=command_.header.stamp+ros::Duration(command_validity_s_);
    if(session_->adopted()){
      ok=session_->continueLane(i,speed_,detail);
      if(ok){
        path=session_->frame()->path;const auto& frame=*session_->frame();
        const uint8_t mode=frame.mode==FOLLOWING?Command::FOLLOW:(frame.mode==STOP?Command::STOP:Command::KEEP);
        const int leader=mode==Command::FOLLOW?frame.leader_id:-1;
        if(command_.maneuver_type!=mode||command_.primary_object_id!=leader){
          ++command_.command_id;command_.maneuver_type=mode;command_.primary_object_id=leader;
          command_.source_link_id=prepared_.map->lane(settings_["target_lanes"][0].as<std::string>())->link_id;
          command_.target_link_id.clear();
          command_.direction=Command::DIRECTION_NONE;command_.execution_request=Command::EXECUTE;
          command_.has_target_pose=mode==Command::STOP;
          command_.target_pose_map=geometry_msgs::Pose();
          if(command_.has_target_pose){const auto p=Interpolate(prepared_.target->line(),prepared_.target->usableEnd());
            command_.target_pose_map.position.x=p.x;command_.target_pose_map.position.y=p.y;
            command_.target_pose_map.orientation.z=std::sin(p.theta/2);command_.target_pose_map.orientation.w=std::cos(p.theta/2);}
        }
        const auto acceptance=manager_.Accept(command_);
        if(!acceptance.should_plan()){ok=false;detail="continuation_command_rejected";}else manager_.NotifyPlanSucceeded();
      }
    }else {
      const bool was_ready=manager_.execution_state()==Feedback::READY;
      command_.execution_request=was_ready||manager_.execution_state()==Feedback::COMMITTED||manager_.execution_state()==Feedback::EXECUTING?Command::EXECUTE:Command::PREPARE;
      const auto acceptance=manager_.Accept(command_);
      session_->planner().SetObjectObservationTime(i.observation.objects_stamp_s);
      FrenetPlanner::LaneChangePrepareResult result;
      if(manager_.execution_state()==Feedback::COMMITTED)manager_.NotifyExecutionStarted();
      if(manager_.execution_state()==Feedback::EXECUTING){
        ok=session_->planner().PlanCommittedLaneChange(i.observation.ego,i.objects,core_command_,path,result);
        ok=ok&&mapClear(path);
        detail="executing_original_reference_lc";
        asmc_msgs::Trajectory converted;
        if(ok&&ConvertCartesianPath(path,session_->planner().sample_dt(),vehicle_,converted)==ConversionStatus::OK&&
           session_->prepare(i,path,speed_,detail)&&
           ConvertCartesianPath(session_->preparedFrame()->path,session_->planner().sample_dt(),vehicle_,converted)==ConversionStatus::OK&&
           session_->commit(i,[this]{return manager_.NotifyManeuverCompleted();},detail)){
          path=session_->frame()->path;
          session_->planner().BindExecutionState(manager_.maneuvers());
          session_->planner().ResetExternalLaneChange();
        }
      }else if(acceptance.should_plan()){
        ok=session_->planner().PrepareLaneChange(i.observation.ego,i.objects,core_command_,path,result);
        // Apply the same admission rule at arm, READY and commit. Off-corridor
        // settled structures stay in i.objects for every original collision check.
        std::string objects_detail;
        const bool objects_clear=TrialObjectsAllowStart(*prepared_.trial_corridor,i.objects,objects_detail);
        const bool candidate_clear=result.candidate_valid&&mapClear(result.candidate_path);
        if(ok&&candidate_clear&&objects_clear)manager_.NotifyPlanSucceeded();else manager_.NotifyPlanFailed(false);
        asmc_msgs::Trajectory candidate;
        if(was_ready&&ok&&candidate_clear&&objects_clear&&
           ConvertCartesianPath(result.candidate_path,session_->planner().sample_dt(),vehicle_,candidate)==ConversionStatus::OK&&
           CommitLaneChangeAtomically(manager_,session_->planner(),result))path=result.candidate_path;
        detail=objects_clear?"preparing_empty_traffic_lc":objects_detail;
        ok=ok&&mapClear(path);
      }
    }
    command_pub_.publish(command_);publish(&path,ok,detail);
  }
  ros::NodeHandle nh_,pnh_;ros::Subscriber state_sub_,objects_sub_,meta_sub_,lanes_sub_,markings_sub_,areas_sub_;
  ros::Publisher trajectory_pub_,command_pub_,feedback_pub_,diagnostics_pub_;ros::ServiceServer arm_service_;ros::SteadyTimer timer_;
  YAML::Node settings_;std::string planner_config_,preparation_error_;LaneReferenceOptions options_;HandoverOptions alignment_;HandoverContinuity continuity_;
  double max_age_=0,speed_=0,arrival_ahead_=0,max_target_lateral_=0,command_validity_s_=0,target_projection_window_=0;
  bool control_=false,armed_=false,preparation_started_=false;
  std::uint64_t sequence_=0;bool have_vehicle_=false,have_objects_=false;ros::SteadyTime vehicle_seen_,objects_seen_;
  asmc_msgs::VehicleState vehicle_;planning_inputs::DetectedObjectArray objects_;
  asmc_msgs::HdMapMeta meta_;asmc_msgs::LaneArray lanes_;asmc_msgs::LaneMarkingArray markings_;asmc_msgs::DrivableAreaArray areas_;
  Prepared prepared_;std::future<Prepared> future_;std::unique_ptr<LaneReferenceExecution> session_;
  CommandExecutionManager manager_;Command command_;PlannerCommand core_command_;
  std::unique_ptr<asmc_planner::GpsModeSource> mode_source_;
};
}
int main(int argc,char** argv){
  ros::init(argc,argv,"reference_handover_trial");
  try{Trial trial;ros::spin();}catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 1;}return 0;
}
