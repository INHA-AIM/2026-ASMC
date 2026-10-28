#include "Execution/HighwayReferenceExecution.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace asmc_planner::lane_reference {
HighwayReferenceExecution::HighwayReferenceExecution(FrenetPlanner& mission,
    const std::string& config,execution::ManeuverState& state)
    :mission_(mission),execution_state_(state),active_(&mission),route_(mission.highway_route()) {
  if(!route_)throw std::runtime_error("reference handover requires prepared highway route");
  const auto p=YAML::LoadFile(config)["planner"]["highway_merge"]["reference_handover"];
  if(!p)throw std::runtime_error("missing highway reference_handover policy");
  LaneReferenceOptions options;options.reference=mission.reference_config();
  options.vehicle={route_->vehicle.length-route_->vehicle.rear_overhang,
      route_->vehicle.rear_overhang,route_->vehicle.width*.5};
  options.max_curvature=mission.kinematic_limits().max_curvature;
  options.max_lateral_accel=mission.kinematic_limits().max_lateral_accel;
  options.max_join_gap_m=p["max_join_gap_m"].as<double>();
  options.sample_distance_m=p["sample_distance_m"].as<double>();
  options.sample_yaw_rad=p["sample_yaw_rad"].as<double>();
  options.footprint_margin_m=p["footprint_margin_m"].as<double>();
  options.max_samples=p["max_samples"].as<std::size_t>();
  if(options.footprint_margin_m!=0)
    throw std::runtime_error("runtime road query requires the configured uninflated vehicle footprint");
  options.runtime_road_footprint=[&mission](double x,double y,double yaw) {
    return mission.RuntimeRoadFootprintClear(x,y,yaw);
  };
  HandoverOptions alignment{route_->config.completion_lateral_m,route_->config.completion_heading_rad,
      p["lateral_speed_tolerance_mps"].as<double>(),route_->config.completion_settle_s,
      route_->config.source_max_age_s,route_->config.source_max_age_s,p["max_pose_gap_s"].as<double>()};
  const auto c=p["continuity"];
  HandoverContinuity continuity{c["overlap_s"].as<double>(),c["position_m"].as<double>(),
      c["heading_rad"].as<double>(),c["curvature_inv_m"].as<double>(),
      c["speed_mps"].as<double>(),c["acceleration_mps2"].as<double>()};
  end_guard_s_=p["end_guard_s"].as<double>();
  if(!std::isfinite(end_guard_s_)||end_guard_s_<2)
    throw std::runtime_error("invalid finite-reference end guard");
  for(const auto& zone:route_->zones) {
    const auto lane=BuildLaneReference(route_->map,route_->map->identity(),zone.config.target_lanes,options,
        LaneReferenceSampling::UNIFORM_ARC_LENGTH);
    if(!lane)throw std::runtime_error(zone.config.name+": "+lane.detail+
        (lane.rejected_point?" at ("+std::to_string(lane.rejected_point->x)+","+
            std::to_string(lane.rejected_point->y)+") s="+std::to_string(lane.rejected_point->s):""));
    const auto common=BuildLaneReference(route_->map,route_->map->identity(),
        {zone.config.target_lanes.back()},options,LaneReferenceSampling::UNIFORM_ARC_LENGTH);
    if(!common)throw std::runtime_error(zone.config.name+" return: "+common.detail);
    auto core=std::make_unique<FrenetPlanner>();
    if(!core->Init(config))throw std::runtime_error("target core initialization failed: "+zone.config.name);
    core->ConfigurePreparedHighwayLane(route_->config,route_->map);
    entries_.push_back(std::make_unique<LaneReferenceExecution>(mission,*core,lane.candidate,
        alignment,continuity,end_guard_s_,lane.candidate->usableBegin()));
    // This target is the original, immutable mission core. It is never given
    // the local lane line; the common-lane reference only gates its adoption.
    returns_.push_back(std::make_unique<LaneReferenceExecution>(*core,mission,common.candidate,
        alignment,continuity,2.0,common.candidate->usableBegin(),false));
    lanes_.push_back(lane.candidate);cores_.push_back(std::move(core));
  }
  options_=options;alignment_=alignment;continuity_=continuity;
  const auto lc=YAML::LoadFile(config)["planner"]["highway_merge"]["lane_change"];
  const bool lane_keeping=YAML::LoadFile(config)["planner"]["highway_merge"]["traffic_lane_keeping"].as<bool>(false);
  if(lc&&(lc["enabled"].as<bool>(false)||lane_keeping)) {
    network_=std::make_shared<HighwayLaneChangeNetwork>();
    network_->forward_accel=route_->config.forward_accel_mps2;
    network_->comfortable_decel=route_->approach_decel_mps2;
    network_->source_max_age=route_->config.source_max_age_s;
    auto& policy=network_->policy;policy.enabled=lc["enabled"].as<bool>(false);
    policy.deficit_mps=lc["speed_deficit_kph"].as<double>()/3.6;
    policy.confirmation_s=lc["confirmation_s"].as<double>();
    policy.cooldown_s=lc["cooldown_s"].as<double>();
    policy.min_gain_s=lc["minimum_gain_s"].as<double>();
    policy.settle_reserve_s=lc["settle_reserve_s"].as<double>();
    policy.observation_front_m=lc["observation_front_m"].as<double>();
    policy.observation_rear_m=lc["observation_rear_m"].as<double>();
    policy.unseen_speed_mps=lc["unseen_speed_mps"].as<double>();
    for(double value:{policy.deficit_mps,policy.confirmation_s,policy.cooldown_s,
        policy.min_gain_s,policy.settle_reserve_s,policy.observation_front_m,
        policy.observation_rear_m,policy.unseen_speed_mps})
      if(!std::isfinite(value)||value<=0)throw std::runtime_error("invalid highway lane-change policy");
    if(policy.observation_front_m>50||policy.observation_rear_m>30||
       policy.unseen_speed_mps<route_->config.speed_cap_mps)
      throw std::runtime_error("lane-change observation policy exceeds configured LiDAR ROI or traffic bound");
    const auto common=BuildLaneReference(route_->map,route_->map->identity(),
        {route_->zones.back().config.target_lanes.back()},options,LaneReferenceSampling::UNIFORM_ARC_LENGTH);
    if(!common)throw std::runtime_error("lane-change common lane: "+common.detail);
    network_->common=common.candidate;
    if(!lc["lanes"].IsSequence()||lc["lanes"].size()<2||lc["lanes"].size()>8)
      throw std::runtime_error("lane-change requires a bounded explicit lane catalog");
    for(const auto& sequence:lc["lanes"]) {
      const auto ids=sequence.as<std::vector<std::string>>();
      if(ids.empty()||ids.back()!=network_->common->lanes().front())
        throw std::runtime_error("lane-change route must reach the explicit common mission lane");
      const auto built=BuildLaneReference(route_->map,route_->map->identity(),ids,
          options,LaneReferenceSampling::UNIFORM_ARC_LENGTH);
      if(!built)throw std::runtime_error("lane-change reference: "+built.detail);
      auto core=std::make_unique<FrenetPlanner>();
      if(!core->Init(config)||!core->SetPreparedReference(built.candidate->line()))
        throw std::runtime_error("lane-change dormant core preparation failed");
      core->ConfigurePreparedHighwayLane(route_->config,route_->map,ids);
      network_->lanes.push_back(built.candidate);lane_cores_.push_back(std::move(core));
    }
    auto common_core=std::make_unique<FrenetPlanner>();
    if(!common_core->Init(config)||!common_core->SetPreparedReference(network_->common->line()))
      throw std::runtime_error("common lane dormant core preparation failed");
    common_core->ConfigurePreparedHighwayLane(route_->config,route_->map,network_->common->lanes());
    lane_cores_.push_back(std::move(common_core));
    status_.network=network_;
    if(lane_keeping) {
      mission_.ConfigureHighwayTrafficLanes(network_->lanes);
      for(auto& core:cores_)core->ConfigureHighwayTrafficLanes(network_->lanes);
      for(auto& core:lane_cores_)core->ConfigureHighwayTrafficLanes(network_->lanes);
    }
  }
  status_.completed.resize(lanes_.size(),false);
}
HighwayReferenceStatus HighwayReferenceExecution::status() const {
  std::lock_guard<std::mutex> lock(status_mutex_);return status_;
}
bool HighwayReferenceExecution::begin(std::uint32_t command,std::size_t zone) {
  if(zone>=entries_.size()||status_.completed[zone])return false;
  if(command_==command&&zone_==zone)return true;
  if(!entries_[zone]->begin(command,status_.epoch))return false;
  mission_.ResetProjectionHistory();
  dynamic_=false;
  command_=command;zone_=zone;pending_return_=false;return true;
}
bool HighwayReferenceExecution::beginLaneTransition(std::uint32_t id,const PlannerCommand& c,
    const CartesianState& ego,bool optional,std::string& detail) {
  if(recoveringTransition()){detail="interrupted_transition_requires_recovery";return false;}
  if(hasLaneTransition(id))return true;
  if(!id||!network_||!status_.lane){detail="lane_change_requires_adopted_highway_reference";return false;}
  const auto source=network_->indexForLink(c.source_link_id);
  const auto target=optional?network_->indexForLink(c.target_link_id):source;
  if(source>=network_->lanes.size()||target>=network_->lanes.size()||
     (optional&&source==target)){detail="request_not_in_prepared_highway_lane_catalog";return false;}
  const auto* road=route_->map->lane("lane:"+c.source_link_id);
  if(!road){detail="missing_source_lane";return false;}
  if(optional) {
    const auto adjacent=c.mode==LANE_CHANGE_LEFT?road->left_adjacent:road->right_adjacent;
    if(adjacent!="lane:"+c.target_link_id){detail="target_is_not_requested_adjacent_lane";return false;}
  } else if(c.target_link_id!=network_->common->lanes().front().substr(5)) {
    detail="mandatory_target_is_not_common_mission_lane";return false;
  }
  const auto projected=ProjectOnLaneReference(*status_.lane,ego.x,ego.y,ego.yaw,ego.v);
  const auto source_p=ProjectOnLaneReference(*network_->lanes[source],ego.x,ego.y,ego.yaw,ego.v);
  if(!projected.valid||!source_p.valid||std::fabs(projected.d)>.5||
     std::fabs(source_p.d)>.5||std::fabs(source_p.heading_error_rad)>.12) {
    detail="ego_not_aligned_with_requested_source_lane";return false;
  }
  auto& target_core=*lane_cores_[optional?target:network_->lanes.size()];
  if(&target_core==active_){detail="target_core_is_active";return false;}
  target_core.DetachExecutionState();target_core.ResetProjectionHistory();target_core.ResetHighwayMerge();
  transition_target_=network_->lanes[target];
  // A mandatory taper follows the already adopted source chain into the
  // common lane. Its target for gap/handover is only that shared successor.
  const auto arrival=optional?transition_target_:network_->common;transition_arrival_=arrival;
  auto& zone=transition_zone_;zone={};zone.config.name=optional?"highway_lane_change":"active_lane_downstream_merge";
  zone.config.source_link=c.source_link_id;
  zone.config.target_lanes=optional?std::vector<std::string>{"lane:"+c.target_link_id}:arrival->lanes();
  zone.source_reference=active_->reference();
  if(optional) {
    zone.admission_rear_observation_m=network_->policy.observation_rear_m;
    zone.admission_unseen_speed_mps=network_->policy.unseen_speed_mps;
  }
  std::string error;
  auto source_ids=network_->lanes[source]->lanes();source_ids.pop_back();
  zone.source_region=route_->map->corridor(source_ids,error);
  zone.target_region=route_->map->corridor(arrival->lanes(),error);
  if(!zone.source_region||!zone.target_region){detail="request_corridor_invalid: "+error;return false;}
  zone.target_reference=arrival->line();
  const auto common_start=arrival->map()->lane(network_->common->lanes().front())->centerline.front();
  const double common_s=FindClosestS(active_->reference(),common_start.x,common_start.y,nullptr,100);
  zone.config.start_s=projected.s;
  const double reserve=HighwayLaneChangeReserve(*network_,ego.v,route_->vehicle.length,
      optional&&c.mode==LANE_CHANGE_LEFT&&HighwayLeftChangeSupportsMerge(*network_,c.source_link_id,c.target_link_id));
  zone.config.completion_s=optional?common_s-reserve:common_s+route_->vehicle.length;
  if(zone.config.completion_s<=projected.s+route_->vehicle.length){detail="insufficient_lane_change_or_taper_distance";return false;}
  zone.config.checkpoints={{zone.config.start_s,zone.config.completion_s}};
  zone.entry_s=zone.config.start_s;
  zone.source_stop_local_s=std::max(projected.s,common_s-route_->vehicle.length-route_->config.stop_buffer);
  zone.source_stop_s=zone.source_stop_local_s;
  transition_source_=active_;
  transition_request_=c;
  dynamic_entry_=std::make_unique<LaneReferenceExecution>(*active_,target_core,arrival,
      alignment_,continuity_,end_guard_s_,arrival->usableBegin());
  // Only the dormant target core receives the arrival reference. Active
  // driving retains its source chain until the completion certificate commits.
  staged_return_=std::make_unique<LaneReferenceExecution>(target_core,mission_,network_->common,
      alignment_,continuity_,2.0,network_->common->usableBegin(),false);
  if(!dynamic_entry_->begin(id,status_.epoch)){detail="reference_request_bind_failed";return false;}
  active_->ResetProjectionHistory();active_->ResetHighwayMerge();command_=id;dynamic_=true;optional_=optional;pending_return_=false;
  detail="prepared_highway_transition_request";return true;
}
bool HighwayReferenceExecution::preserveInterruptedTransition(std::uint32_t next) {
  if(recoveringTransition())return true;
  if(!next||next==command_||!dynamic_||!optional_||!dynamic_entry_||dynamic_entry_->adopted()||
      !transition_source_||!transition_source_->highway_merge_committed())return false;
  transition_source_->RevokeHighwayMergeReadiness();
  std::lock_guard<std::mutex> lock(status_mutex_);status_.transition_interrupted=true;return true;
}
void HighwayReferenceExecution::finishInterruptedOnSource() {
  if(!recoveringTransition())return;
  transition_source_->ResetHighwayMerge();
  dynamic_entry_.reset();staged_return_.reset();dynamic_=false;command_=0;
  std::lock_guard<std::mutex> lock(status_mutex_);status_.transition_interrupted=false;
}
bool HighwayReferenceExecution::prepare(const ReferenceExecutionInput& i,const CartesianPath& path,
    double speed,std::string& detail) {
  return command_&&(dynamic_?dynamic_entry_.get():entries_[zone_].get())->prepare(i,path,speed,detail);
}
const ActiveReferenceFrame* HighwayReferenceExecution::preparedFrame() const {
  return pending_return_?active_return_->preparedFrame():
      (command_?(dynamic_?dynamic_entry_.get():entries_[zone_].get())->preparedFrame():nullptr);
}
bool HighwayReferenceExecution::commit(const ReferenceExecutionInput& i,
    const std::function<bool()>& complete,std::string& detail) {
  if(!command_)return false;
  if(dynamic_) {
    if(!dynamic_entry_->commit(i,complete,detail))return false;
    active_->DetachExecutionState();active_=&dynamic_entry_->planner();
    execution_state_=active_->executionSnapshot();active_->BindExecutionState(execution_state_);
    {std::lock_guard<std::mutex> lock(status_mutex_);
     status_.lane=transition_arrival_;status_.transition_interrupted=false;
     if(!optional_)status_.completed.back()=true;++status_.epoch;}
    dynamic_return_=std::move(staged_return_);
    active_return_=dynamic_return_.get();active_return_->begin(command_,status_.epoch);
    return true;
  }
  if(!entries_[zone_]->commit(i,complete,detail))return false;
  active_->DetachExecutionState();
  active_=&entries_[zone_]->planner();active_zone_=zone_;
  execution_state_=active_->executionSnapshot();active_->BindExecutionState(execution_state_);
  {std::lock_guard<std::mutex> lock(status_mutex_);
   status_.lane=lanes_[zone_];status_.transition_interrupted=false;status_.completed[zone_]=true;++status_.epoch;}
  active_return_=returns_[active_zone_].get();active_return_->begin(command_,status_.epoch);
  return true;
}
bool HighwayReferenceExecution::prepareMissionReturn(const ReferenceExecutionInput& i,
    const CartesianPath& path,double speed,std::string& detail) {
  pending_return_=false;
  if(active_==&mission_)return false;
  // The dormant mission core must reproject from map x/y, never its last
  // station before a previous lane change.
  mission_.ResetProjectionHistory();
  pending_return_=active_return_&&active_return_->prepare(i,path,speed,detail);
  return pending_return_;
}
bool HighwayReferenceExecution::commitMissionReturn(const ReferenceExecutionInput& i,std::string& detail) {
  if(!pending_return_||!active_return_->commit(i,[]{return true;},detail))return false;
  active_->DetachExecutionState();
  active_=&mission_;pending_return_=false;
  execution_state_=active_->executionSnapshot();active_->BindExecutionState(execution_state_);
  std::lock_guard<std::mutex> lock(status_mutex_);status_.lane.reset();++status_.epoch;return true;
}
void HighwayReferenceExecution::guardFiniteEnd(const CartesianState& ego,PlannerCommand& command) const {
  if(active_==&mission_||command.mode==STOP||command.mode==EMERGENCY)return;
  const auto& lane=*status_.lane;
  const auto p=ProjectOnLaneReference(lane,ego.x,ego.y,ego.yaw,ego.v);
  const double braking=ego.v*ego.v/(2*route_->approach_decel_mps2);
  if(!p.valid||lane.usableEnd()-p.s<=braking+ego.v*end_guard_s_+lane.options().vehicle.front_m) {
    command.mode=STOP;command.target_object_id=-1;command.stop_position=lane.usableEnd();
  }
}
bool HighwayReferenceExecution::validActivePath(const CartesianPath& path) const {
  if(active_==&mission_)return true;
  const auto& lane=*status_.lane;
  if(path.x.size()<2||path.y.size()!=path.x.size()||path.yaw.size()!=path.x.size())return false;
  for(std::size_t n=0;n<path.x.size();++n) {
    const auto p=ProjectOnLaneReference(lane,path.x[n],path.y[n],path.yaw[n],0);
    if(!p.valid||p.s<lane.usableBegin()||p.s>lane.usableEnd()||
       !lane.coversRoadFootprint(path.x[n],path.y[n],path.yaw[n]))return false;
  }
  return true;
}
void HighwayReferenceExecution::observeMissionStation(double s) {
  if(!std::isfinite(s))return;
  if(active_==&mission_&&last_mission_s_>route_->reference.length*.75&&s<route_->reference.length*.25) {
    for(auto& entry:entries_)entry->resetDormant();
    for(auto& back:returns_)back->resetDormant();
    command_=0;pending_return_=false;dynamic_=false;dynamic_entry_.reset();dynamic_return_.reset();staged_return_.reset();active_return_=nullptr;
    std::lock_guard<std::mutex> lock(status_mutex_);
    std::fill(status_.completed.begin(),status_.completed.end(),false);status_.transition_interrupted=false;++status_.epoch;
  }
  last_mission_s_=s;
}
}  // namespace asmc_planner::lane_reference
