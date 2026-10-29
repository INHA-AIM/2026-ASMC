#include "Parser/BehaviorInput.hpp"

#include "Planning/BehaviorContext.hpp"
#include <algorithm>
#include <cmath>
#include <XmlRpcValue.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iomanip>
#include "Parser/PlanningInput.hpp"

#include <asmc_msgs/BehaviorContext.h>
#include <asmc_msgs/BehaviorCommand.h>
#include <visualization_msgs/MarkerArray.h>

#include "Output/BehaviorMessages.hpp"
#include "Output/Behavior/utils.hpp"
#include "Parser/DetectedObjects.hpp"
#include "Parser/MoraiObjects.hpp"

namespace behavior_planner {
namespace {

std::string JsonString(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else out << c;
  }
  out << '"';
  return out.str();
}

DynamicPolicyConfig LoadDynamicPolicyConfig(ros::NodeHandle& node) {
  DynamicPolicyConfig config;
  node.getParam("dynamic_behavior/route_links",config.route_links);
  double speed_kph=25.0;
  node.param("dynamic_behavior/section_speed_limit_kph",speed_kph,speed_kph);
  if(!std::isfinite(speed_kph) || speed_kph<=0)
    throw std::runtime_error("Invalid dynamic section speed limit");
  config.section_speed_limit=speed_kph/3.6;
  return config;
}

PlannerFeedbackData NormalizeFeedback(
    const asmc_msgs::FrenetFeedback* feedback) {
  PlannerFeedbackData data;
  if (!feedback) return data;
  data.valid = true;
  data.command_id = feedback->command_id;
  data.result_code = feedback->result_code;
  switch (feedback->execution_state) {
    case asmc_msgs::FrenetFeedback::PREPARING:
      data.execution_state = PlannerExecutionState::PREPARING;
      break;
    case asmc_msgs::FrenetFeedback::WAITING:
      data.execution_state = PlannerExecutionState::WAITING;
      break;
    case asmc_msgs::FrenetFeedback::READY:
      data.execution_state = PlannerExecutionState::READY;
      break;
    case asmc_msgs::FrenetFeedback::COMMITTED:
      data.execution_state = PlannerExecutionState::COMMITTED;
      break;
    case asmc_msgs::FrenetFeedback::EXECUTING:
      data.execution_state = PlannerExecutionState::EXECUTING;
      break;
    case asmc_msgs::FrenetFeedback::COMPLETED:
      data.execution_state = PlannerExecutionState::COMPLETED;
      break;
    case asmc_msgs::FrenetFeedback::FAILED:
      data.execution_state = PlannerExecutionState::FAILED;
      break;
    default:
      data.execution_state = PlannerExecutionState::IDLE;
      break;
  }
  return data;
}

}  // namespace

BehaviorInput::BehaviorInput(ros::NodeHandle nh, ros::NodeHandle pnh)
    : BehaviorInput(std::move(nh), std::move(pnh), IntentSink{}) {}

BehaviorInput::BehaviorInput(ros::NodeHandle nh, ros::NodeHandle pnh,
                             IntentSink intent_sink)
    : BehaviorInput(std::move(nh),std::move(pnh),std::move(intent_sink),DynamicEnvironment{}) {}

BehaviorInput::BehaviorInput(ros::NodeHandle nh,ros::NodeHandle pnh,
                             IntentSink intent_sink,DynamicEnvironment environment)
    : nh_(std::move(nh)),pnh_(std::move(pnh)),intent_sink_(std::move(intent_sink)),
      dynamic_policy_(environment,LoadDynamicPolicyConfig(pnh_)) {
  nh_.param("/control_binding/enabled",control_binding_enabled_,false);
  if(control_binding_enabled_) {
    control_request_pub_=nh_.advertise<std_msgs::UInt8MultiArray>("/behavior/control_request",10);
    controller_epoch_sub_=nh_.subscribe<std_msgs::String>("/control/binding_epoch",1,
        [this](const std_msgs::String::ConstPtr& m) {
          if(m && m->data.size()==32 && m->data!=controller_epoch_) {
            controller_epoch_=m->data;control_status_sequence_=0;control_status_={};
          }
        });
    control_status_sub_=nh_.subscribe<std_msgs::UInt8MultiArray>("/control/binding_status",64,
        [this](const std_msgs::UInt8MultiArray::ConstPtr& m) {
          asmc_msgs::binding::Status status;
          if(m && asmc_msgs::binding::Decode(*m,status) && status.token==last_control_request_.token &&
             status.output_sequence>control_status_sequence_) {
            control_status_=status;control_status_sequence_=status.output_sequence;
            execution_recovery_.Observe(status.execution_failed,status.failure_mask,status.linked,
                last_control_request_.execution_recovery_ack);
            control_status_received_at_=ros::SteadyTime::now();
          }
        });
  }
  pnh_.param("dynamic_behavior/enabled",dynamic_enabled_,true);
  signal_shadow_=std::make_unique<SignalShadow>(nh_,pnh_);
  signal_active_=std::make_unique<SignalShadow>(nh_,pnh_,true);
  dynamic_diagnostics_pub_=nh_.advertise<std_msgs::String>("/planning/dynamic_behavior",10);
  static_mission_diagnostics_pub_=nh_.advertise<std_msgs::String>("/planning/static_mission_policy",2);
  std::string ego_topic = "/vehicle/state";
  std::string objects_topic = "/Object_topic";
  std::string objects_input_type = "morai";
  std::string feedback_topic = "/planning/feedback";
  std::string feedback_v2_topic = "/planning/frenet_feedback_v2";
  std::string context_topic = "/behavior/context";
  std::string command_v2_topic = "/behavior/command_v2";
  std::string interface_version = "v1";
  std::string debug_topic = "/behavior/feature_debug";
  std::string marker_topic = "/behavior/markers";
  std::string link_path;
  std::string node_path;
  std::string route_path;
  std::string traffic_light_path;
  // P4-C3: typed is authoritative; the old String topic remains as a
  // compatibility fallback for deployments that have not started the new
  // asmc_camera producer yet.
  std::string traffic_light_typed_topic =
      "/perception/traffic_light/detection";
  std::string traffic_light_legacy_topic = "/traffic_light/state";
  bool use_traffic_light_mock = false;
  double traffic_light_stale_seconds = 1.0;
  double traffic_light_detection_grace_seconds = 1.0;
  double intersection_approach_speed_limit_kph = 25.0;
  double stop_target_offset_m = 5.0;
  TrafficStopConfig traffic_stop;
  SafetyConfig safety;
  FollowTargetConfig follow_target;
  AvoidTargetConfig avoid_target;
  double emergency_ttc_exit = 3.0;
  double emergency_release_hold_s = 0.5;
  double follow_release_delay_s = 0.4;
  int avoid_release_grace_cycles = 3;

  pnh_.param("behavior/update_rate", rate_, rate_);
  pnh_.param("topics/ego", ego_topic, ego_topic);
  pnh_.param("topics/objects", objects_topic, objects_topic);
  pnh_.param("objects/input_type", objects_input_type, objects_input_type);
  pnh_.param("topics/plan_feedback", feedback_topic, feedback_topic);
  pnh_.param("topics/frenet_feedback_v2", feedback_v2_topic,
             feedback_v2_topic);
  pnh_.param("publish_topics/behavior_context", context_topic, context_topic);
  pnh_.param("publish_topics/behavior_command_v2", command_v2_topic,
             command_v2_topic);
  pnh_.param("interface_version", interface_version, interface_version);
  pnh_.param("v2/command_validity_seconds", command_validity_seconds_,
             command_validity_seconds_);
  pnh_.param("v2/feedback_stale_seconds", feedback_v2_stale_seconds_,
             feedback_v2_stale_seconds_);
  pnh_.param("objects/stale_seconds", objects_stale_seconds_,
             objects_stale_seconds_);
  pnh_.param("follow/max_target_distance", follow_target.max_distance,
             follow_target.max_distance);
  pnh_.param("follow/max_lateral_offset", follow_target.max_lateral_offset,
             follow_target.max_lateral_offset);
  pnh_.param("follow/release_delay_s", follow_release_delay_s,
             follow_release_delay_s);
  pnh_.param("avoid/max_target_distance", avoid_target.max_distance,
             avoid_target.max_distance);
  pnh_.param("avoid/max_lateral_offset", avoid_target.max_lateral_offset,
             avoid_target.max_lateral_offset);
  pnh_.param("avoid/path_margin_m", avoid_target.path_margin_m,
             avoid_target.path_margin_m);
  pnh_.param("avoid/min_physical_overlap_m",
             avoid_target.min_physical_overlap_m,
             avoid_target.min_physical_overlap_m);
  pnh_.param("avoid/min_margin_overlap_m",
             avoid_target.min_margin_overlap_m,
             avoid_target.min_margin_overlap_m);
  pnh_.param("avoid/min_overlap_ratio", avoid_target.min_overlap_ratio,
             avoid_target.min_overlap_ratio);
  pnh_.param("avoid/immediate_blocking_ratio",
             avoid_target.immediate_blocking_ratio,
             avoid_target.immediate_blocking_ratio);
  pnh_.param("avoid/blocking_enter_ratio",
             avoid_target.blocking_enter_ratio,
             avoid_target.blocking_enter_ratio);
  pnh_.param("avoid/blocking_exit_ratio",
             avoid_target.blocking_exit_ratio,
             avoid_target.blocking_exit_ratio);
  pnh_.param("avoid/overlap_confirm_cycles",
             avoid_target.overlap_confirm_cycles,
             avoid_target.overlap_confirm_cycles);
  pnh_.param("avoid/overlap_release_cycles",
             avoid_target.overlap_release_cycles,
             avoid_target.overlap_release_cycles);
  pnh_.param("avoid/reconnect_grace_cycles",
             avoid_target.reconnect_grace_cycles,
             avoid_target.reconnect_grace_cycles);
  pnh_.param("avoid/reconnect_max_longitudinal_delta_m",
             avoid_target.reconnect_max_longitudinal_delta_m,
             avoid_target.reconnect_max_longitudinal_delta_m);
  pnh_.param("avoid/reconnect_max_lateral_delta_m",
             avoid_target.reconnect_max_lateral_delta_m,
             avoid_target.reconnect_max_lateral_delta_m);
  pnh_.param("avoid/reconnect_max_size_delta_m",
             avoid_target.reconnect_max_size_delta_m,
             avoid_target.reconnect_max_size_delta_m);
  pnh_.param("avoid/release_grace_cycles", avoid_release_grace_cycles,
             avoid_release_grace_cycles);
  pnh_.param("vehicle/width", avoid_target.ego_width,
             avoid_target.ego_width);
  pnh_.param("vehicle/length", avoid_target.ego_length, avoid_target.ego_length);
  pnh_.param("vehicle/rear_overhang", avoid_target.rear_overhang, avoid_target.rear_overhang);
  pnh_.param("publish_topics/feature_debug", debug_topic, debug_topic);
  pnh_.param("publish_topics/marker", marker_topic, marker_topic);
  pnh_.param("map/link_set_path", link_path, std::string());
  pnh_.param("map/node_set_path", node_path, std::string());
  pnh_.param("route/route_links_path", route_path, std::string());
  pnh_.param("traffic_light/mock_path", traffic_light_path, std::string());
  pnh_.param("traffic_light/use_mock", use_traffic_light_mock,
             use_traffic_light_mock);
  pnh_.param("traffic_light/typed_topic", traffic_light_typed_topic,
             traffic_light_typed_topic);
  pnh_.param("traffic_light/topic", traffic_light_legacy_topic,
             traffic_light_legacy_topic);
  pnh_.param("traffic_light/stale_seconds", traffic_light_stale_seconds,
             traffic_light_stale_seconds);
  pnh_.param("traffic_light/detection_grace_seconds",
             traffic_light_detection_grace_seconds,
             traffic_light_detection_grace_seconds);
  pnh_.param("traffic_light/approach_speed_limit_kph",
             intersection_approach_speed_limit_kph,
             intersection_approach_speed_limit_kph);
  pnh_.param("traffic_light/approach_comfortable_decel",
             traffic_stop.approach_comfortable_decel,
             traffic_stop.approach_comfortable_decel);
  pnh_.param("traffic_light/approach_margin_m",
             traffic_stop.approach_margin,
             traffic_stop.approach_margin);
  pnh_.param("traffic_light/post_stopline_distance_m",
             traffic_stop.post_stopline_distance,
             traffic_stop.post_stopline_distance);
  pnh_.param("stop/target_offset_m", stop_target_offset_m,
             stop_target_offset_m);
  pnh_.param("stop/red_light_stop_distance",
             traffic_stop.minimum_detection_distance,
             traffic_stop.minimum_detection_distance);
  pnh_.param("stop/comfortable_decel", traffic_stop.comfortable_decel,
             traffic_stop.comfortable_decel);
  pnh_.param("stop/braking_margin", traffic_stop.braking_margin,
             traffic_stop.braking_margin);
  pnh_.param("stop/planner_stop_buffer", traffic_stop.planner_stop_buffer,
             traffic_stop.planner_stop_buffer);
  pnh_.param("safety/front_min_gap", safety.front_min_gap,
             safety.front_min_gap);
  pnh_.param("safety/emergency_ttc_enter", safety.emergency_ttc_enter,
             safety.emergency_ttc_enter);
  pnh_.param("safety/emergency_predicted_ttc",
             safety.emergency_predicted_ttc,
             safety.emergency_predicted_ttc);
  pnh_.param("safety/emergency_required_decel",
             safety.emergency_required_decel,
             safety.emergency_required_decel);
  pnh_.param("safety/pedestrian_stop_distance",
             safety.pedestrian_stop_distance,
             safety.pedestrian_stop_distance);
  pnh_.param("safety/emergency_ttc_exit", emergency_ttc_exit,
             emergency_ttc_exit);
  pnh_.param("safety/emergency_release_hold_s", emergency_release_hold_s,
             emergency_release_hold_s);

  if (!std::isfinite(intersection_approach_speed_limit_kph) ||
      intersection_approach_speed_limit_kph <= 0.0) {
    ROS_WARN("Invalid intersection approach speed limit; using 25.0 km/h");
    intersection_approach_speed_limit_kph = 25.0;
  }
  if (!std::isfinite(traffic_stop.approach_comfortable_decel) ||
      traffic_stop.approach_comfortable_decel <= 0.0) {
    ROS_WARN("Invalid intersection approach deceleration; using 2.0 m/s^2");
    traffic_stop.approach_comfortable_decel = 2.0;
  }
  if (!std::isfinite(traffic_stop.approach_margin) ||
      traffic_stop.approach_margin < 0.0) {
    ROS_WARN("Invalid intersection approach target offset; using 1.5 m");
    traffic_stop.approach_margin = 1.5;
  }
  if (!std::isfinite(traffic_stop.post_stopline_distance) ||
      traffic_stop.post_stopline_distance < 0.0) {
    ROS_WARN("Invalid post-stopline speed-limit distance; using 5.0 m");
    traffic_stop.post_stopline_distance = 5.0;
  }
  traffic_stop.approach_speed_limit =
      intersection_approach_speed_limit_kph / 3.6;

  if (!std::isfinite(traffic_light_stale_seconds) ||
      traffic_light_stale_seconds <= 0.0) {
    ROS_WARN("Invalid traffic-light transport stale timeout; using 1.0 s");
    traffic_light_stale_seconds = 1.0;
  }
  if (!std::isfinite(traffic_light_detection_grace_seconds) ||
      traffic_light_detection_grace_seconds <= 0.0) {
    ROS_WARN("Invalid traffic-light detection grace; using 1.0 s");
    traffic_light_detection_grace_seconds = 1.0;
  }

  if (follow_target.max_distance <= 0.0 ||
      follow_target.max_lateral_offset <= 0.0) {
    ROS_WARN("Invalid FOLLOW target limits; using 60.0 m / 1.75 m");
    follow_target.max_distance = 60.0;
    follow_target.max_lateral_offset = 1.75;
  }
  if (objects_stale_seconds_ <= 0.0) {
    ROS_WARN("Invalid object timeout; using 0.30 s");
    objects_stale_seconds_ = 0.30;
  }
  if (avoid_target.max_distance <= 0.0 ||
      avoid_target.max_lateral_offset <= 0.0 ||
      !std::isfinite(avoid_target.ego_width) || avoid_target.ego_width <= 0.0 ||
      !std::isfinite(avoid_target.path_margin_m) || avoid_target.path_margin_m < 0.0 ||
      !std::isfinite(avoid_target.ego_length) || avoid_target.ego_length <= 0.0 ||
      !std::isfinite(avoid_target.rear_overhang) || avoid_target.rear_overhang < 0.0 ||
      avoid_target.rear_overhang >= avoid_target.ego_length ||
      avoid_target.min_physical_overlap_m <= 0.0 ||
      avoid_target.min_margin_overlap_m <= 0.0 ||
      avoid_target.min_overlap_ratio < 0.0 ||
      avoid_target.min_overlap_ratio > 1.0 ||
      avoid_target.immediate_blocking_ratio < 0.0 ||
      avoid_target.immediate_blocking_ratio > 1.0 ||
      avoid_target.blocking_enter_ratio < 0.0 ||
      avoid_target.blocking_enter_ratio >
          avoid_target.immediate_blocking_ratio ||
      avoid_target.blocking_exit_ratio < 0.0 ||
      avoid_target.blocking_exit_ratio >=
          avoid_target.blocking_enter_ratio ||
      avoid_target.overlap_confirm_cycles < 1 ||
      avoid_target.overlap_release_cycles < 1 ||
      avoid_target.reconnect_grace_cycles < 0 ||
      avoid_target.reconnect_max_longitudinal_delta_m <= 0.0 ||
      avoid_target.reconnect_max_lateral_delta_m <= 0.0 ||
      avoid_target.reconnect_max_size_delta_m < 0.0) {
    ROS_WARN("Invalid AVOID corridor/overlap policy; using 120.0 m / 1.75 m "
             "legacy ceiling, 1.9 m ego width, 0.45 m margin, "
             "0.10/0.20 m overlap, margin ratio 0.20, "
             "blocking ratio 0.70/0.35/0.15, confirm/release 2/3 cycles");
    avoid_target.max_distance = 120.0;
    avoid_target.max_lateral_offset = 1.75;
    avoid_target.ego_width = 1.9;
    avoid_target.path_margin_m = 0.45;
    avoid_target.ego_length = 4.635;
    avoid_target.rear_overhang = 0.79;
    avoid_target.min_physical_overlap_m = 0.10;
    avoid_target.min_margin_overlap_m = 0.20;
    avoid_target.min_overlap_ratio = 0.20;
    avoid_target.immediate_blocking_ratio = 0.70;
    avoid_target.blocking_enter_ratio = 0.35;
    avoid_target.blocking_exit_ratio = 0.15;
    avoid_target.overlap_confirm_cycles = 2;
    avoid_target.overlap_release_cycles = 3;
    avoid_target.reconnect_grace_cycles = 5;
    avoid_target.reconnect_max_longitudinal_delta_m = 3.0;
    avoid_target.reconnect_max_lateral_delta_m = 0.8;
    avoid_target.reconnect_max_size_delta_m = 1.5;
  }

  bool roundabout_enabled=false;
  std::string roundabout_stop_node, roundabout_source, roundabout_target;
  pnh_.param("roundabout_merge/enabled",roundabout_enabled,false);
  pnh_.param<std::string>("roundabout_merge/stop_node",roundabout_stop_node,"A1256W000709");
  pnh_.param<std::string>("roundabout_merge/source_link",roundabout_source,"A2256W000148");
  pnh_.param<std::string>("roundabout_merge/target_link",roundabout_target,"A2256W000146");
  builder_.setRoundaboutMerge(roundabout_enabled,roundabout_stop_node,roundabout_source,roundabout_target);
  map_loaded_=map_.load(link_path, node_path);
  if (!map_loaded_) {
    ROS_ERROR("MGeo loading failed; check map paths");
  }
  route_loaded_=route_.initialize(map_, loadRouteLinks(route_path));
  if (!route_loaded_) {
    ROS_ERROR("Route initialization failed; check route_links.yaml");
  }
  builder_.setStopTargetOffset(stop_target_offset_m);
  traffic_stop.target_offset = stop_target_offset_m;
  decision_.setTrafficStopConfig(traffic_stop);
  double maximum_speed_kph=57.5;
  pnh_.param("maximum_speed_kph",maximum_speed_kph,maximum_speed_kph);
  if(!decision_.setAbsoluteSpeedLimit(maximum_speed_kph/3.6))
    throw std::runtime_error("Invalid maximum_speed_kph: finite positive limit required");
  NormalCruiseConfig normal_cruise;
  double cruise_target_kph = 0.0, cruise_max_kph = 0.0, cruise_map_min_kph = 0.0;
  pnh_.param("normal_cruise/enabled", normal_cruise.enabled, false);
  pnh_.param("normal_cruise/target_speed_kph", cruise_target_kph, 0.0);
  pnh_.param("normal_cruise/max_speed_kph", cruise_max_kph, 0.0);
  pnh_.param("normal_cruise/minimum_map_speed_kph", cruise_map_min_kph, 0.0);
  normal_cruise.target_speed = cruise_target_kph / 3.6;
  normal_cruise.max_speed = cruise_max_kph / 3.6;
  normal_cruise.minimum_map_speed = cruise_map_min_kph / 3.6;
  // ROS/YAML types and km/h conversion stay at the input boundary. The
  // decision layer receives only validated link IDs and m/s values.
  bool link_caps_valid = true;
  XmlRpc::XmlRpcValue link_caps;
  if (pnh_.hasParam("normal_cruise/link_speed_caps_kph")) {
    link_caps_valid = pnh_.getParam("normal_cruise/link_speed_caps_kph", link_caps) &&
        link_caps.getType() == XmlRpc::XmlRpcValue::TypeStruct;
    if (link_caps_valid) {
      for (auto it = link_caps.begin(); it != link_caps.end(); ++it) {
        const auto type = it->second.getType();
        if (type != XmlRpc::XmlRpcValue::TypeInt &&
            type != XmlRpc::XmlRpcValue::TypeDouble) {
          link_caps_valid = false;
          break;
        }
        const double kph = type == XmlRpc::XmlRpcValue::TypeInt
            ? static_cast<double>(static_cast<int>(it->second))
            : static_cast<double>(it->second);
        normal_cruise.link_speed_caps[it->first] = kph / 3.6;
      }
    }
  }
  if (!link_caps_valid) normal_cruise.enabled = false;
  const bool cruise_valid = decision_.setNormalCruiseConfig(normal_cruise);
  if (!link_caps_valid || !cruise_valid) {
    ROS_ERROR("Invalid normal_cruise settings; profile disabled, map limits retained");
  } else if (normal_cruise.enabled) {
    ROS_WARN("MORAI ordinary KEEP cruise: target=%.1f cap=%.1f km/h; "
             "eligible map links >= %.1f km/h or %zu explicit link caps",
             cruise_target_kph, cruise_max_kph, cruise_map_min_kph,
             normal_cruise.link_speed_caps.size());
  }

  builder_.setSafetyConfig(safety);
  builder_.setFollowTargetConfig(follow_target);
  builder_.setAvoidTargetConfig(avoid_target);
  decision_.setEmergencyRelease(emergency_ttc_exit,
                                emergency_release_hold_s);
  decision_.setFollowReleaseDelay(follow_release_delay_s);
  decision_.setAvoidReleaseGraceCycles(avoid_release_grace_cycles);
  ROS_INFO("Behavior stop target offset: %.2f m behind physical stop line",
           stop_target_offset_m);
  ROS_INFO("Behavior traffic STOP detection: min=%.2f m, dynamic=v^2/(2*%.2f)"
           "+%.2f margin+%.2f target_offset+%.2f planner_buffer",
           traffic_stop.minimum_detection_distance,
           traffic_stop.comfortable_decel, traffic_stop.braking_margin,
           traffic_stop.target_offset, traffic_stop.planner_stop_buffer);
  ROS_INFO("Behavior intersection approach: speed=%.1f km/h, "
           "comfortable_decel=%.2f m/s^2, pre-line=%.1f m, post-line=%.1f m, "
           "activation=current-speed based with stop-line latch",
           intersection_approach_speed_limit_kph,
           traffic_stop.approach_comfortable_decel,
           traffic_stop.approach_margin,
           traffic_stop.post_stopline_distance);
  ROS_INFO("Behavior FOLLOW target limits: distance=%.2f m lateral=%.2f m, "
           "release delay=%.2f s, objects stale after %.2f s",
           follow_target.max_distance, follow_target.max_lateral_offset,
           follow_release_delay_s, objects_stale_seconds_);
  ROS_INFO("Behavior AVOID corridor: distance=%.2f m half_width=%.2f m "
           "(ego_width=%.3f + margin=%.2f, legacy_ceiling=%.2f), "
           "overlap physical=%.2f m margin=%.2f m margin_ratio=%.2f, "
           "blocking_ratio immediate/enter/exit=%.2f/%.2f/%.2f "
           "confirm/release=%d/%d, "
           "reconnect grace=%d ds/dd/size=%.2f/%.2f/%.2f m, "
           "release grace=%d cycles",
           avoid_target.max_distance,
           0.5 * avoid_target.ego_width + avoid_target.path_margin_m,
           avoid_target.ego_width, avoid_target.path_margin_m,
           avoid_target.max_lateral_offset,
           avoid_target.min_physical_overlap_m,
           avoid_target.min_margin_overlap_m,
           avoid_target.min_overlap_ratio,
           avoid_target.immediate_blocking_ratio,
           avoid_target.blocking_enter_ratio,
           avoid_target.blocking_exit_ratio,
           avoid_target.overlap_confirm_cycles,
           avoid_target.overlap_release_cycles,
           avoid_target.reconnect_grace_cycles,
           avoid_target.reconnect_max_longitudinal_delta_m,
           avoid_target.reconnect_max_lateral_delta_m,
           avoid_target.reconnect_max_size_delta_m,
           avoid_release_grace_cycles);

  if (use_traffic_light_mock) {
    if(signal_active_->enabled())throw std::runtime_error("active signal policy forbids mock traffic lights");
    auto mock = std::make_shared<MockTrafficLightProvider>();
    if (!mock->load(traffic_light_path)) {
      ROS_WARN("Traffic-light mock loading failed; default GREEN is used");
    }
    lights_ = mock;
    ROS_WARN("Behavior traffic-light source: YAML mock %s",
             traffic_light_path.c_str());
  } else {
    lights_ = std::make_shared<RosTrafficLightProvider>(
        nh_, traffic_light_typed_topic, traffic_light_legacy_topic,
        traffic_light_stale_seconds, traffic_light_detection_grace_seconds,
        signal_shadow_ && signal_shadow_->enabled());
    ROS_INFO("Behavior traffic-light sources: typed=%s legacy_fallback=%s "
             "(stale after %.2f s)",
             traffic_light_typed_topic.c_str(),
             traffic_light_legacy_topic.c_str(),
             traffic_light_stale_seconds);
  }

  if(signal_active_->enabled()) {
    signal_active_source_=std::make_unique<RosTrafficLightProvider>(nh_,traffic_light_typed_topic,
        traffic_light_legacy_topic,traffic_light_stale_seconds,traffic_light_detection_grace_seconds,true);
    double offset=0,uncertainty=0;std::string camera_topic="/cam_extra/image_jpeg/compressed";
    if(!pnh_.getParam("signal_policy/source_to_ros_offset_s",offset) ||
       !pnh_.getParam("signal_policy/source_clock_uncertainty_s",uncertainty))
      throw std::runtime_error("active signal policy requires measured source-to-ROS clock bounds");
    pnh_.param("signal_policy/camera_topic",camera_topic,camera_topic);
    signal_active_source_->requireOriginalFrames(nh_,camera_topic,offset,uncertainty);
    signal_active_source_->setStopObservationWakeup([this] {
      // ProductionRuntime has one Behavior callback thread. Evaluate the same
      // policy with current pose/risks, including normal-entry exemption and
      // cause arbitration; never publish a camera-only brake/release command.
      if(signal_active_ && signal_active_->inScope())runOnce();
    });
  }
  if(signal_shadow_->enabled()) {
    double offset=0,uncertainty=0;std::string camera_topic="/cam_extra/image_jpeg/compressed";
    if(pnh_.getParam("signal_policy/source_to_ros_offset_s",offset) &&
       pnh_.getParam("signal_policy/source_clock_uncertainty_s",uncertainty)) {
      pnh_.param("signal_policy/camera_topic",camera_topic,camera_topic);
      if(auto* source=dynamic_cast<RosTrafficLightProvider*>(lights_.get()))
        source->requireOriginalFrames(nh_,camera_topic,offset,uncertainty);
    }
  }

  use_v2_interface_ = interface_version == "v2";
  if (interface_version != "v1" && !use_v2_interface_) {
    ROS_WARN("Unknown behavior interface_version '%s'; using v1",
             interface_version.c_str());
  }
  if (use_v2_interface_) {
    command_v2_pub_ =
        nh_.advertise<asmc_msgs::BehaviorCommand>(command_v2_topic, 10);
  } else {
    context_pub_ =
        nh_.advertise<asmc_msgs::BehaviorContext>(context_topic, 10);
  }
  debug_pub_ = nh_.advertise<UrbanFeatureDebug>(debug_topic, 10);
  marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 2);
  visualization_ = BehaviorVisualization(marker_pub_);

  ego_sub_ = nh_.subscribe(ego_topic, 5, &BehaviorInput::egoCallback, this);
  if (objects_input_type == "detected") {
    objects_sub_ = nh_.subscribe(
        objects_topic, 5, &BehaviorInput::detectedObjectsCallback, this);
  } else {
    if (objects_input_type != "morai") {
      ROS_WARN("Unknown objects/input_type '%s'; using morai",
               objects_input_type.c_str());
      objects_input_type = "morai";
    }
    objects_sub_ = nh_.subscribe(
        objects_topic, 5, &BehaviorInput::moraiObjectsCallback, this);
  }
  if (use_v2_interface_ && !intent_sink_) {
    feedback_sub_ = nh_.subscribe(feedback_v2_topic, 5,
                                  &BehaviorInput::feedbackV2Callback, this);
  } else if (!use_v2_interface_) {
    feedback_sub_ =
        nh_.subscribe(feedback_topic, 5, &BehaviorInput::feedbackCallback, this);
  }

  if (rate_ <= 0.0) {
    ROS_WARN("Invalid behavior update rate %.3f Hz; using 10 Hz", rate_);
    rate_ = 10.0;
  }
  update_timer_ = nh_.createSteadyTimer(
      ros::WallDuration(1.0 / rate_), &BehaviorInput::timerCallback, this);

  ROS_INFO_STREAM("Behavior planner input: ego=" << ego_topic
                  << ", objects=" << objects_topic
                  << " (" << objects_input_type << ")");
  ROS_INFO_STREAM("Behavior interface " << (use_v2_interface_ ? "v2" : "v1")
                  << ": "
                  << (use_v2_interface_ ? command_v2_topic : context_topic)
                  << " -> "
                  << (use_v2_interface_ ? feedback_v2_topic : feedback_topic));
}

void BehaviorInput::spin() {
  ros::spin();
}

void BehaviorInput::timerCallback(const ros::SteadyTimerEvent&) { runOnce(); }

void BehaviorInput::runOnce() {
  if (!ego_received_) {
    if(signal_shadow_) signal_shadow_->Update(ego_,false,false,nullptr,route_,
        dynamic_cast<RosTrafficLightProvider*>(lights_.get()));
    if(signal_active_ && signal_active_->enabled())
      signal_active_->Update(ego_,false,false,nullptr,route_,signal_active_source_.get());
    ROS_WARN_THROTTLE(5.0, "Waiting for valid asmc_msgs/VehicleState");
    return;
  }
  // Freeze feedback before feature evaluation as the legacy single-thread
  // subscriber/timer ordering did. Direct feedback can arrive on the planning
  // thread; its receipt clock is still sampled in receiveFeedback.
  asmc_msgs::FrenetFeedback feedback_snapshot;
  ros::SteadyTime feedback_received_at;
  bool feedback_received;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_snapshot = feedback_v2_;
    feedback_received_at = feedback_v2_received_at_;
    feedback_received = feedback_v2_received_;
  }
  const ros::SteadyTime steady_now = ros::SteadyTime::now();
  const bool objects_fresh =
      objects_received_ && objects_stale_seconds_ > 0.0 &&
      (steady_now - objects_received_at_).toSec() <= objects_stale_seconds_;
  const PerceptionObjects empty_objects;
  if (objects_received_ && !objects_fresh) {
    ROS_WARN_THROTTLE(1.0, "Behavior object input is stale; clearing targets");
  }
  // Recheck ego scope every cycle, but only rebuild the immutable observation
  // and converters when the input or scope changes. A zone exit still restores
  // person/vehicle semantics without waiting for a new object callback.
  std::size_t static_mission_remapped = 0;
  bool static_mission_ego_in_scope = false;
  if (static_mission_policy_ && raw_detected_objects_ && objects_fresh) {
    if (static_mission_observation_.update(*static_mission_policy_, raw_detected_objects_,
                                         ego_.x, ego_.y, ego_.yaw, ego_.valid)) {
      const auto& effective = static_mission_observation_.objects();
      canonical_objects_.clear();
      frenet_planner_ros::ConvertDetectedObjects(effective, canonical_objects_);
      objects_ = DetectedMsgConverter::toPerceptionObjects(effective);
      objects_.source_stamp = effective.header.stamp.toSec();
    }
    static_mission_ego_in_scope = static_mission_observation_.egoInScope();
    static_mission_remapped = static_mission_observation_.remapped();
    if (static_mission_remapped > 0) {
      ROS_INFO_THROTTLE(2.0, "Static mission: %zu objects use stationary mission prior (raw velocity/validity unchanged)",
                        static_mission_remapped);
    }
  }
  // Read-only evidence of the exact copy consumed above. Array order and full
  // observation identity let the viewer join private IDs to LiDAR track IDs.
  if (static_mission_diagnostics_pub_.getNumSubscribers() > 0) {
    const auto* raw = raw_detected_objects_.get();
    const bool fresh = raw && objects_fresh;
    if (static_mission_policy_ && !fresh)
      static_mission_ego_in_scope = static_mission_policy_->containsEgo(ego_.x, ego_.y, ego_.yaw, ego_.valid);
    std::ostringstream out;
    out << std::boolalpha << "{\"schema\":\"static_mission_policy/v1\",\"enabled\":"
        << bool(static_mission_policy_) << ",\"input_fresh\":" << fresh
        << ",\"ego_in_scope\":" << static_mission_ego_in_scope
        << ",\"remapped\":" << static_mission_remapped;
    if (raw) {
      out << ",\"stamp_ns\":\"" << raw->header.stamp.toNSec()
          << "\",\"producer_session_id\":" << JsonString(raw->producer_session_id)
          << ",\"generation\":\"" << raw->generation
          << "\",\"scan_sequence\":\"" << raw->scan_sequence
          << "\",\"config_id\":" << JsonString(raw->config_id)
          << ",\"calibration_id\":" << JsonString(raw->calibration_id);
    }
    out << ",\"objects\":[";
    if (fresh) for (std::size_t i = 0; i < raw->objects.size(); ++i) {
      if (i) out << ',';
      out << "{\"planner_id\":" << raw->objects[i].id
          << ",\"original_type\":" << int(raw->objects[i].type)
          << ",\"effective_type\":" << int(static_mission_policy_ ? static_mission_observation_.objects().objects[i].type : raw->objects[i].type)
          << ",\"original_motion\":" << int(raw->objects[i].motion_state)
          << ",\"effective_motion\":" << int(static_mission_policy_ ? static_mission_observation_.objects().objects[i].motion_state : raw->objects[i].motion_state)
          << ",\"mission_prior\":" << (static_mission_ego_in_scope && static_mission_policy_ && static_mission_policy_->contains(raw->objects[i].center.x, raw->objects[i].center.y))
          << '}';
    }
    out << "]}";
    std_msgs::String report; report.data = out.str();
    static_mission_diagnostics_pub_.publish(report);
  }
  double scope_s=0,scope_d=0;std::string scope_link;
  const bool projected=route_.projectEgo(ego_.x,ego_.y,&scope_s,&scope_d,&scope_link);
  const bool scoped_dynamic=dynamic_enabled_ && projected && dynamic_policy_.inScope(scope_link);
  // Cache the complete semantic observation. Decide ownership every ego cycle,
  // including a section crossing with no new object callback. Canonical Frenet
  // input remains unchanged; outside the section all legacy targets return.
  BehaviorType active=BehaviorType::KEEP;
  if(feedback_snapshot.active_maneuver==asmc_msgs::BehaviorCommand::AVOID)active=BehaviorType::AVOID;
  else if(feedback_snapshot.active_maneuver==asmc_msgs::BehaviorCommand::MERGE)active=BehaviorType::MERGE;
  else if(feedback_snapshot.active_maneuver==asmc_msgs::BehaviorCommand::LANE_CHANGE)active=BehaviorType::LEFT_CHANGE;
  const bool complex=feedback_received && DynamicObjectPolicy::usesNativePathSafety(
      active,NormalizeFeedback(&feedback_snapshot).execution_state);
  const double feedback_age=(steady_now-feedback_received_at).toSec();
  const bool previous_validated_stop=feedback_received &&
      feedback_snapshot.command_id==v2_command_manager_.activeCommandId() &&
      feedback_snapshot.command_accepted && feedback_snapshot.plan_valid &&
      feedback_age>=0 && feedback_age<=feedback_v2_stale_seconds_ &&
      (feedback_snapshot.active_maneuver==asmc_msgs::BehaviorCommand::STOP ||
       feedback_snapshot.active_maneuver==asmc_msgs::BehaviorCommand::EMERGENCY_STOP);
  const bool validated_native_avoid=feedback_received && active==BehaviorType::AVOID &&
      feedback_snapshot.command_id==v2_command_manager_.activeCommandId() &&
      feedback_snapshot.command_accepted && feedback_snapshot.plan_valid &&
      feedback_age>=0 && feedback_age<=feedback_v2_stale_seconds_ &&
      NormalizeFeedback(&feedback_snapshot).execution_state!=PlannerExecutionState::IDLE &&
      feedback_snapshot.execution_state!=asmc_msgs::FrenetFeedback::COMPLETED &&
      feedback_snapshot.execution_state!=asmc_msgs::FrenetFeedback::FAILED;
  // A freshly validated native AVOID already checks every final object.
  // Do not preempt it solely because a person slowed to LOW_THREAT. Unknown
  // pedestrian rules remain for unevaluated inputs; failed/stale planning
  // restores the legacy rule. Fresh provisional people use the same-frame
  // occupancy brake decision. The canonical collision input is never filtered.
  const asmc_planner::BehaviorContext input{
      ego_, objects_fresh?objects_:empty_objects, route_, *lights_};
  if(highway_reference_source_)builder_.setHighwayReference(highway_reference_source_());
  bool reserved_static_avoid = false;
  UrbanFeature feature = builder_.build(
      input.ego, input.perception, input.route, input.traffic,ros::Time::now().toSec(),
      [&](const ObjectFeature& object) {
        if (!dynamic_enabled_ || !dynamic_policy_.enabled() || complex) return true;
        const bool allowed=dynamic_policy_.allowsStaticAvoid(object,route_);
        reserved_static_avoid |= !allowed;
        return allowed;
      },
      [&](UrbanFeature& current,const PerceptionObjects& source) {
        if(dynamic_enabled_ && dynamic_policy_.enabled())
          current.dynamic=dynamic_policy_.evaluate(current,route_,
              objects_fresh?canonical_objects_:std::vector<::TrackedObject>{},
              ros::Time::now().toSec()-objects_source_time_,!complex,previous_validated_stop);
        return DynamicObjectPolicy::legacyObjects(source,scoped_dynamic,
            validated_native_avoid,current.dynamic.evaluated_provisional_ids,
            current.dynamic.action==DynamicAction::AVOID?current.dynamic.object_id:-1);
      });
  if(dynamic_enabled_ && dynamic_policy_.enabled()) {
    if(feature.dynamic.action==DynamicAction::FOLLOW &&
       (!feature.surrounding.has_front_vehicle ||
        feature.dynamic.lead.gap<feature.surrounding.front_vehicle.gap)) {
      feature.surrounding.has_front_vehicle=true;
      feature.surrounding.front_vehicle=feature.dynamic.lead;
    }
    feature.risk.min_ttc=std::min(feature.risk.min_ttc,feature.dynamic.ttc);
    std::ostringstream stream;
    stream<<"{\"action\":\""<<DynamicActionName(feature.dynamic.action)
        <<"\",\"object_id\":"<<feature.dynamic.object_id
        <<",\"eligible\":"<<feature.dynamic.eligible<<",\"outside\":"<<feature.dynamic.outside
        <<",\"in_scope\":"<<(feature.dynamic.in_scope?"true":"false")
        <<",\"scope_link_id\":\""<<feature.route.route_required_link_id<<"\""
        <<",\"ordinary\":"<<(!complex?"true":"false")
        <<",\"legacy_static_avoid_reserved\":"<<(reserved_static_avoid?"true":"false")
        <<",\"provisional_owned\":"<<feature.dynamic.evaluated_provisional_ids.size()
        <<",\"reason\":\""<<feature.dynamic.reason<<"\""
        <<",\"ego_speed_mps\":"<<(std::isfinite(feature.ego.speed)?feature.ego.speed:0.)
        <<",\"object_source_time\":"<<std::fixed<<std::setprecision(9)<<objects_source_time_
        <<",\"assessments\":[";
    bool first_assessment=true;
    for(const auto& a:feature.dynamic.assessments) {
      if(!first_assessment)stream<<",";
      first_assessment=false;
      stream<<"{\"id\":"<<a.id<<",\"type\":"<<a.type<<",\"motion\":"<<a.motion
          <<",\"gate\":\""<<a.gate<<"\",\"ttc_s\":"<<a.ttc<<",\"stop_time_s\":"<<a.stop_time
          <<",\"object_speed_mps\":"<<a.object_speed<<",\"lateral_m\":"<<a.lateral
          <<",\"half_width_m\":"<<a.half_width<<",\"known_mobile\":"<<(a.known_mobile?"true":"false")
          <<",\"bypass_room\":"<<(a.bypass_room?"true":"false")<<"}";
    }
    stream<<"]}";
    std_msgs::String message;message.data=stream.str();dynamic_diagnostics_pub_.publish(message);
  }
  const asmc_msgs::FrenetFeedback* feedback = nullptr;
  if (use_v2_interface_) {
    const bool feedback_fresh =
        feedback_received &&
        (steady_now - feedback_received_at).toSec() <=
            feedback_v2_stale_seconds_;
    feedback = feedback_fresh ? &feedback_snapshot : nullptr;
  }
  asmc_msgs::binding::SignalStop signal_constraint;
  bool release_authorized=true,recovery_release=false,signal_stop_owned=false;
  if(signal_active_ && signal_active_->enabled()) {
    asmc_planner::signal::Evidence evidence;
    const double status_age=(steady_now-control_status_received_at_).toSec();
    const bool status_fresh=control_status_sequence_ && status_age>=0 && status_age<=.25 &&
        control_status_.token==last_control_request_.token;
    evidence.linked=evidence.command_accepted=evidence.trajectory_belongs_to_command=
        status_fresh && control_status_.linked;
    evidence.command_id=last_control_request_.command.command_id;
    const bool recovery_ack=status_fresh && control_status_.failure_mask==2 &&
        control_status_.linked && control_status_.execution_recovery_safe && ego_.speed<=.15;
    evidence.planning_checked=status_fresh && control_status_.planning_checked &&
        (last_control_request_.signal.enabled || recovery_ack);
    evidence.stop_path_valid=control_status_.stop_path_valid;
    evidence.path_goal_satisfied=control_status_.goal_satisfied;
    evidence.exit_path_valid=status_fresh && control_status_.exit_path_valid;
    evidence.execution_failed=execution_recovery_.failed() && !execution_recovery_.pending();
    evidence.execution_failure_mask=control_status_.failure_mask;
    evidence.execution_recovery_acknowledged=recovery_ack;
    evidence.response_delay_known=status_fresh && brake_level_ &&
        control_status_.brake_generation==brake_generation_ && control_status_.brake_level==brake_level_ &&
        std::isfinite(control_status_.response_remaining) && control_status_.response_remaining>=0;
    evidence.response_delay_remaining=control_status_.response_remaining;
    evidence.response_observed=control_status_.response_observed;
    // Actuator response monitoring is connected separately to the original
    // decision baseline; a receipt alone must not restart its delay budget.
    signal_active_->ExecutionEvidence(evidence);
    signal_active_->Update(ego_,(steady_now-ego_received_at_).toSec()>=0 &&
        (steady_now-ego_received_at_).toSec()<=.3,objects_fresh,&feature,route_,signal_active_source_.get());
    execution_recovery_.Authorize(signal_active_->output().execution_recovery_acknowledged);
  }
  UrbanFeature decision_feature=feature;
  const bool signal_scoped=signal_active_ && signal_active_->enabled() && signal_active_->inScope();
  if(signal_scoped)decision_feature.signal.light_state=TrafficLightState::GREEN;
  auto decision_feedback = NormalizeFeedback(feedback);
  if (use_v2_interface_) {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    // Keep the actual latest feedback for arbitration and command sequencing.
    // Only the maneuver lifecycle consumes the previously observed READY.
    if (roundabout_ready_notice_.consume(feedback, ros::SteadyTime::now().toSec(),
                                        feedback_v2_stale_seconds_))
      decision_feedback.execution_state = PlannerExecutionState::READY;
  }
  BehaviorContextData context = use_v2_interface_
      ? decision_.decide(decision_feature, decision_feedback,
                         v2_command_manager_.activeCommandId())
      : decision_.decide(decision_feature, feedback_);
  if(signal_scoped) {
    namespace sig=asmc_planner::signal;
    const auto& result=signal_active_->output();const auto& g=signal_active_->geometry();
    const auto& cfg=signal_active_->config();
    // A momentary loss/reclassification of the bollard after entering this
    // signal's conflict area can make Behavior nominate another ordinary
    // maneuver while Execution still owns a committed AVOID. Execution
    // correctly refuses that new ID;
    // sending it anyway disconnects Controller from the safe AVOID path.
    // Continue the accepted maneuver until terminal feedback. STOP and any
    // other risk retain their own precedence below.
    const bool committed_signal_avoid=
        result.severity==sig::Severity::NONE &&
        context.selected_behavior!=BehaviorType::AVOID &&
        context.selected_behavior!=BehaviorType::STOP &&
        context.selected_behavior!=BehaviorType::EMERGENCY_STOP &&
        !context.hard.planned_stop && !context.hard.force_stop &&
        feedback && feedback->command_id==last_control_request_.command.command_id &&
        feedback->active_maneuver==asmc_msgs::BehaviorCommand::AVOID &&
        (feedback->execution_state==asmc_msgs::FrenetFeedback::COMMITTED ||
         feedback->execution_state==asmc_msgs::FrenetFeedback::EXECUTING) &&
        last_control_request_.command.maneuver_type==asmc_msgs::BehaviorCommand::AVOID &&
        last_control_request_.signal.route_generation==g.key.route_generation &&
        last_control_request_.signal.stop_line==g.key.stop_line &&
        last_control_request_.signal.movement==g.key.movement;
    if(committed_signal_avoid) {
      context.selected_behavior=BehaviorType::AVOID;
      context.state=BehaviorState::AVOID_EXECUTE;
      context.maneuver_phase=ManeuverPhase::COMMITTED;
      context.target.avoidance_object_id=last_control_request_.command.primary_object_id;
      context.debug.selected_reason="signal_committed_avoid_continuation";
    }
    signal_constraint.route_generation=g.key.route_generation;signal_constraint.stop_line=g.key.stop_line;
    signal_constraint.movement=g.key.movement;signal_constraint.target_revision=result.target_revision;
    signal_constraint.line_x=g.line.x;signal_constraint.line_y=g.line.y;
    signal_constraint.normal_x=g.approach.x;signal_constraint.normal_y=g.approach.y;
    signal_constraint.clearance=cfg.clearance;signal_constraint.position_error=cfg.position_error;
    signal_constraint.recovery_geometry=g.conflict_reviewed;
    signal_constraint.exit_x=g.exit.x;signal_constraint.exit_y=g.exit.y;
    signal_constraint.exit_normal_x=g.exit_direction.x;signal_constraint.exit_normal_y=g.exit_direction.y;
    const bool other_stop=context.selected_behavior==BehaviorType::STOP ||
        context.selected_behavior==BehaviorType::EMERGENCY_STOP || context.hard.force_stop;
    const bool continuing_recovery_avoid=feedback &&
        feedback->active_maneuver==asmc_msgs::BehaviorCommand::AVOID &&
        last_control_request_.recovery_release &&
        last_control_request_.command.maneuver_type==asmc_msgs::BehaviorCommand::AVOID &&
        last_control_request_.signal.route_generation==g.key.route_generation &&
        last_control_request_.signal.stop_line==g.key.stop_line &&
        last_control_request_.signal.movement==g.key.movement;
    const bool committed_lateral=feedback && !continuing_recovery_avoid &&
        (feedback->active_maneuver==asmc_msgs::BehaviorCommand::AVOID ||
         feedback->active_maneuver==asmc_msgs::BehaviorCommand::LANE_CHANGE ||
         feedback->active_maneuver==asmc_msgs::BehaviorCommand::MERGE) &&
        (feedback->execution_state==asmc_msgs::FrenetFeedback::COMMITTED ||
         feedback->execution_state==asmc_msgs::FrenetFeedback::EXECUTING);
    // Hand a committed maneuver to actual signal braking before asking KEEP
    // to preview a new exit. An ordinary KEEP cannot bypass Execution's lock.
    const bool preflight=!other_stop && !committed_lateral && (result.motion==sig::Motion::RELEASE_PENDING ||
        result.motion==sig::Motion::RECOVERY_CLEAR_PENDING || signal_active_->recoveryCandidate() ||
        signal_active_->pathRecoveryCandidate());
    recovery_release=preflight && (signal_active_->recoveryCandidate() ||
        signal_active_->pathRecoveryRequiresExit());
    if(preflight) {
      // A collision-checked AVOID is required when the exit contains an
      // obstacle. Keep its target and transaction while Controller holds the
      // brake until this command's complete exit path is proven.
      const bool recovery_avoid=recovery_release &&
          context.selected_behavior==BehaviorType::AVOID;
      if(!recovery_avoid) {
        context.selected_behavior=BehaviorType::KEEP;context.state=BehaviorState::KEEP_LANE;
        context.target.has_target_pose=context.target.has_stop_target=false;
      }
      context.target.desired_speed=std::min(25./3.6,feature.route.speed_limit>0?feature.route.speed_limit:25./3.6);
      if(recovery_release) {
        context.target.desired_speed=std::min(context.target.desired_speed,cfg.recovery_speed);
        context.extra_flags["signal_recovery_speed_limit"]=true;
      }
      context.debug.selected_reason=recovery_avoid?"signal_recovery_avoid_preflight":
          recovery_release?"signal_recovery_preflight":"signal_release_preflight";
      const auto& previous=last_control_request_;
      const uint8_t expected_maneuver=recovery_avoid?
          asmc_msgs::BehaviorCommand::AVOID:asmc_msgs::BehaviorCommand::KEEP;
      const bool previous_candidate=previous.command.maneuver_type==expected_maneuver &&
          previous.signal.route_generation==g.key.route_generation && previous.signal.stop_line==g.key.stop_line &&
          previous.signal.movement==g.key.movement && previous.recovery_release==recovery_release;
      release_authorized=release_commit_.Authorize(g.key,recovery_release,
          !execution_recovery_.failed() || execution_recovery_.pending(),result.severity==sig::Severity::NONE,
          previous_candidate && previous.release_authorized,steady_now.toSec());
      signal_stop_owned=!release_authorized;
    } else if(result.severity==sig::Severity::EMERGENCY) {
      context.selected_behavior=BehaviorType::EMERGENCY_STOP;context.state=BehaviorState::EMERGENCY_STOP;
      context.hard.force_stop=context.hard.emergency_stop=true;context.target.desired_speed=0;
      context.debug.selected_reason="signal_emergency";
      signal_stop_owned=true;
    } else if(result.severity==sig::Severity::STOP && context.selected_behavior!=BehaviorType::EMERGENCY_STOP) {
      const double anchor_s=g.route_s-signal_active_->targetOffset()+cfg.buffer;
      if(!other_stop || !context.target.has_stop_target || context.target.stop_before_s>anchor_s) {
        Point2D goal;double yaw=0;
        if(route_.getRoutePose(anchor_s,&goal,&yaw)) {
          context.selected_behavior=BehaviorType::STOP;context.state=BehaviorState::STOP;
          context.hard.planned_stop=true;context.hard.force_stop=false;
          context.target.has_stop_target=context.target.has_target_pose=true;
          context.target.stop_x=context.target.target_x=goal.x;
          context.target.stop_y=context.target.target_y=goal.y;context.target.target_yaw=yaw;
          context.target.stop_before_s=anchor_s;
          context.target.desired_speed=result.motion==sig::Motion::APPROACH ? 25./3.6 : 0.;
          context.debug.selected_reason="signal_stop";signal_stop_owned=true;
          signal_constraint.enabled=true;signal_constraint.braking_only=result.motion!=sig::Motion::APPROACH;
        } else {
          context.selected_behavior=BehaviorType::EMERGENCY_STOP;context.hard.force_stop=true;
          context.target.desired_speed=0;context.debug.selected_reason="signal_target_invalid";
        }
      } else {
        // A nearer pedestrian/collision STOP still carries the outstanding
        // signal's physical constraint and braking latch. Replacing its target
        // must not reopen legacy reapproach while SIGNAL(K) remains asserted.
        signal_stop_owned=true;signal_constraint.enabled=true;
        signal_constraint.braking_only=result.motion!=sig::Motion::APPROACH;
      }
    }
    if((signal_stop_owned || preflight) && !(preflight && recovery_release &&
        context.selected_behavior==BehaviorType::AVOID))decision_.preemptLateralForSignal();
    if(!preflight)release_commit_.Reset();
  }
  if(!signal_scoped)release_commit_.Reset();
  if (use_v2_interface_) {
    const auto command = v2_command_manager_.update(
        context, feedback, ros::Time::now(), command_validity_seconds_);
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      roundabout_ready_notice_.issuedCommand(command);
    }
    command_v2_pub_.publish(command); // external compatibility mirror
    asmc_msgs::binding::Request request;
    if(control_binding_enabled_ && !controller_epoch_.empty()) {
      request.command=command;
      request.signal=signal_constraint;request.release_authorized=release_authorized;
      request.recovery_release=recovery_release;
      request.execution_recovery_ack=signal_scoped && execution_recovery_.pending();
      const unsigned level=signal_stop_owned &&
          (command.maneuver_type==asmc_msgs::BehaviorCommand::EMERGENCY_STOP || signal_constraint.braking_only || !release_authorized) ?
          (signal_active_->output().severity==asmc_planner::signal::Severity::EMERGENCY?2:1) : 0;
      if(level && (!brake_level_ || level>brake_level_)) {
        ++brake_generation_;brake_level_=level;
        brake_baseline_.brake_decision_ros=ros::Time::now().toSec();
        brake_baseline_.brake_state_stamp=ego_.stamp;brake_baseline_.brake_speed=ego_.speed;
        brake_baseline_.brake_acceleration=ego_.acceleration;
      }
      if(!level && release_authorized)brake_level_=0;
      if(level) {
        request.brake_generation=brake_generation_;request.brake_level=level;
        request.brake_decision_ros=brake_baseline_.brake_decision_ros;
        request.brake_state_stamp=brake_baseline_.brake_state_stamp;
        request.brake_speed=brake_baseline_.brake_speed;request.brake_acceleration=brake_baseline_.brake_acceleration;
        request.brake_budget=asmc_planner::signal::ResponseBudget(signal_active_->config().measured_max_delay);
      }
      if(last_control_request_.token.controller_epoch!=controller_epoch_ ||
         asmc_msgs::binding::SemanticBytes(request)!=asmc_msgs::binding::SemanticBytes(last_control_request_))
        ++control_request_revision_;
      request.token={controller_epoch_,behavior_epoch_,control_request_revision_};
      last_control_request_=request;
      control_request_pub_.publish(asmc_msgs::binding::Encode(request));
    }
    if (intent_sink_) intent_sink_({context, command,control_binding_enabled_?&request:nullptr});
  } else {
    asmc_msgs::BehaviorContext command = MsgConverter::toMsg(context, 0);
    command.header.stamp = ros::Time::now();
    if (!has_last_command_ || !SameCommandIdentity(command, last_command_)) {
      ++current_command_id_;
      if (current_command_id_ == 0) ++current_command_id_;
    }
    command.command_id = current_command_id_;
    last_command_ = command;
    has_last_command_ = true;
    context_pub_.publish(command);
  }
  debug_pub_.publish(MsgConverter::toDebugMsg(feature, context));
  visualization_.publish(feature, context, route_, map_, *lights_);
  logger_.write(feature, context);
  if(signal_shadow_) signal_shadow_->Update(ego_,
      (steady_now-ego_received_at_).toSec()>=0 &&
      (steady_now-ego_received_at_).toSec()<=.3,objects_fresh,&feature,route_,
      dynamic_cast<RosTrafficLightProvider*>(lights_.get()));
}

void BehaviorInput::egoCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
  if (!message) return;
  if (!message->valid || !std::isfinite(message->x) ||
      !std::isfinite(message->y) || !std::isfinite(message->yaw) ||
      !std::isfinite(message->speed) || message->header.stamp.isZero()) {
    if(signal_active_ && signal_active_->enabled() && ego_received_) {
      // Keep the episode but invalidate its position so the active arbiter
      // emits INPUT_INVALID rather than silently disappearing from control.
      ego_.valid=false;ego_received_at_=ros::SteadyTime::now();
    } else ego_received_ = false;
    ROS_WARN_THROTTLE(2.0, "Behavior received invalid /vehicle/state");
    return;
  }
  ego_.x = message->x;
  ego_.y = message->y;
  ego_.yaw = message->yaw;
  ego_.speed = std::abs(message->speed);
  ego_.stamp = message->header.stamp.toSec();
  ego_.source = message->source;
  ego_.acceleration = message->acceleration;
  ego_.acceleration_valid = std::isfinite(message->acceleration);
  ego_.valid = true;
  ego_received_ = true;
  ego_received_at_=ros::SteadyTime::now();
  if(signal_shadow_) signal_shadow_->PushPose(ego_,route_);
  if(signal_active_) signal_active_->PushPose(ego_,route_);
}

void BehaviorInput::moraiObjectsCallback(
    const morai_msgs::ObjectStatusList::ConstPtr& message) {
  if (!message) return;
  raw_detected_objects_.reset();
  static_mission_observation_ = {};
  objects_ = MoraiMsgConverter::toPerceptionObjects(*message);
  objects_.source_stamp=message->header.stamp.toSec();
  objects_received_at_ = ros::SteadyTime::now();
  objects_received_ = true;
}

void BehaviorInput::detectedObjectsCallback(
    const planning_inputs::DetectedObjectArray::ConstPtr& message) {
  if (!message) return;
  if (message->header.stamp.isZero() || message->header.frame_id != "map") {
    ROS_WARN_THROTTLE(
        1.0, "Behavior rejected DetectedObjectArray with invalid header");
    return;
  }
  raw_detected_objects_ = message;
  objects_source_time_=message->header.stamp.toSec();
  if (!static_mission_policy_) {
    canonical_objects_.clear();
    frenet_planner_ros::ConvertDetectedObjects(*message,canonical_objects_);
    objects_ = DetectedMsgConverter::toPerceptionObjects(*message);
    objects_.source_stamp=message->header.stamp.toSec();
  }
  objects_received_at_ = ros::SteadyTime::now();
  objects_received_ = true;
}

void BehaviorInput::feedbackCallback(
    const asmc_msgs::PlanFeedback::ConstPtr& message) {
  if (message) feedback_ = *message;
}

void BehaviorInput::feedbackV2Callback(
    const asmc_msgs::FrenetFeedback::ConstPtr& message) {
  if (!message) return;
  receiveFeedback(*message);
}

void BehaviorInput::receiveFeedback(const asmc_msgs::FrenetFeedback& feedback) {
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  feedback_v2_ = feedback;
  feedback_v2_received_at_ = ros::SteadyTime::now();
  feedback_v2_received_ = true;
  roundabout_ready_notice_.observe(feedback, feedback_v2_received_at_.toSec());
}

}  // namespace behavior_planner


