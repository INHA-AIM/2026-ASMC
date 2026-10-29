#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "Behavior/Context/behavior_types.hpp"
#include "Map/HighwayMergeRoute.hpp"
#include "Execution/HighwayLaneChange.hpp"
#include "Frenet/Geometry/GeometryTypes.hpp"
namespace behavior_planner {
struct EgoFeature { double x=0,y=0,yaw=0,speed=0,s=0,d=0,map_s=0; uint8_t source=0; bool state_valid=false,valid=false; };
struct RouteFeature { std::string current_link_id,target_link_id,route_required_link_id; double speed_limit=0,distance_to_turn=1e9,distance_to_intersection=1e9; bool lane_change_left_allowed=false,lane_change_right_allowed=false,current_link_valid=false,left_target_pose_valid=false,right_target_pose_valid=false; std::string left_target_link_id,right_target_link_id; double left_target_x=0,left_target_y=0,left_target_yaw=0,right_target_x=0,right_target_y=0,right_target_yaw=0; };
struct SignalFeature { TrafficLightState light_state=TrafficLightState::UNKNOWN; SignalMovement required_movement=SignalMovement::UNKNOWN; std::string traffic_light_id,controlled_link_id,related_signal,raw_label; bool has_stop_line=false,has_required_movement=false,recently_visible=false,has_passed_stop_line=false; double stop_line_s=1e9,stop_target_s=1e9,distance_to_stop_line=1e9,distance_since_stop_line=1e9,movement_link_s=1e9,distance_to_movement=1e9,stop_line_x=0,stop_line_y=0,stop_line_yaw=0; bool stop_line_valid=false;
  // Physical line never aliases the buffered command anchor. Frenet subtracts
  // its existing 0.5m buffer from this anchor, exactly once.
  double stop_target_x=0,stop_target_y=0,stop_target_yaw=0;
  bool stop_target_pose_valid=false;
};
struct ObjectFeature { ObjectType type=ObjectType::UNKNOWN; MotionState motion_state=MotionState::UNKNOWN; int id=-1; double x=0,y=0,yaw=0,vx=0,vy=0,speed=0,accel=0,length=0,width=0,s=0,d=0,link_s=0,gap=1e9,relative_speed=0,ttc=1e9; std::string link_id; bool valid=false; geometry::Corners8 corners{}; bool geometry_valid=false; };
enum class DynamicAction { KEEP, FOLLOW, AVOID, STOP, EMERGENCY };
// Current-frame diagnostic evidence only. It owns neither tracking nor action
// state; the native execution feedback remains authoritative for maneuvers.
struct DynamicAssessment {
  int id=-1,type=0,motion=0;
  double ttc=1e9,stop_time=0,object_speed=0,lateral=0,half_width=0;
  bool known_mobile=false,bypass_room=false;
  std::string gate;
};
struct DynamicDecision {
  bool in_scope = false;
  double section_speed_limit = 1e9;
  DynamicAction action = DynamicAction::KEEP;
  int object_id = -1;
  double ttc = 1e9, stop_s = 1e9, stop_x = 0, stop_y = 0, stop_yaw = 0;
  ObjectFeature lead;
  // Current-frame ownership, not track memory or proof of standstill.
  std::vector<int> evaluated_provisional_ids;
  std::vector<DynamicAssessment> assessments;
  std::size_t eligible = 0, outside = 0;
  std::string reason = "no_confirmed_dynamic_hazard";
};
struct SurroundingFeature { bool has_front_vehicle=false; ObjectFeature front_vehicle; bool left_lane_safe=false,right_lane_safe=false; double left_front_gap=1e9,left_rear_gap=1e9,right_front_gap=1e9,right_rear_gap=1e9,left_rear_ttc=1e9,right_rear_ttc=1e9; };
struct RiskFeature { bool static_obstacle_ahead=false,pedestrian_risk=false,emergency_risk=false,avoid_target_reclassified_moving=false,avoid_target_matches_front_lead=false,avoid_target_matches_recent_lead=false; int avoidance_object_id=-1; double min_ttc=1e9,nearest_obstacle_s=1e9,nearest_obstacle_length=0.0,nearest_pedestrian_s=1e9; };
struct RoundaboutFeature { bool enabled=false,approach=false; std::string source_link_id,target_link_id; double distance_to_stop=1e9; };
struct HighwayLaneChangeOffer {
  bool available=false;
  std::string source_link,target_link;
  double x=0,y=0,yaw=0,lateral_distance=0;
  double free_speed=INFINITY;
};
struct HighwayLaneChangeFeature {
  std::shared_ptr<const asmc_planner::lane_reference::HighwayLaneChangeNetwork> network;
  bool fresh=false;
  double observation_stamp=0,evaluation_stamp=0,free_speed=0,remaining_m=0;
  std::uint64_t reference_epoch=0;
  HighwayLaneChangeOffer left,right;
};
struct UrbanFeature { DynamicDecision dynamic; HighwayLaneChangeFeature lane_change; HighwayMergeApproach highway; RoundaboutFeature roundabout; double timestamp=0; int frame_count=0; EgoFeature ego; RouteFeature route; SignalFeature signal; SurroundingFeature surrounding; RiskFeature risk; std::unordered_map<std::string,double> extra_values; std::unordered_map<std::string,bool> extra_flags; };
using EgoState = asmc_planner::EgoState;
// Preserve original class/group for policy review; semantic identity alone does
// not establish motion. Existing type and motion_state decisions remain separate.
struct TrackedObject { int id=-1; ObjectType type=ObjectType::UNKNOWN; MotionState motion_state=MotionState::UNKNOWN; double x=0,y=0,yaw=0,vx=0,vy=0,speed=0,accel=0,length=4.5,width=1.8; bool velocity_unavailable=false; std::string classification, camera_class; geometry::Corners8 corners{}; bool geometry_valid=false; };
struct PerceptionObjects { std::vector<TrackedObject> vehicles,pedestrians,obstacles; double source_stamp=0; };
}
