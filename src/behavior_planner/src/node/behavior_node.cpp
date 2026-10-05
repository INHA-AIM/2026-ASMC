#include "behavior_planner/node/behavior_node.hpp"

#include <algorithm>
#include <cmath>

#include <asmc_msgs/BehaviorContext.h>
#include <asmc_msgs/BehaviorCommand.h>
#include <visualization_msgs/MarkerArray.h>

#include "behavior_planner/context/msg_converter.hpp"
#include "behavior_planner/global/utils.hpp"
#include "behavior_planner/node/detected_msg_converter.hpp"
#include "behavior_planner/node/morai_msg_converter.hpp"

namespace behavior_planner {
namespace {

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

BehaviorNode::BehaviorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
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
  std::string traffic_light_topic = "/traffic_light/state";
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
  pnh_.param("publish_topics/feature_debug", debug_topic, debug_topic);
  pnh_.param("publish_topics/marker", marker_topic, marker_topic);
  pnh_.param("map/link_set_path", link_path, std::string());
  pnh_.param("map/node_set_path", node_path, std::string());
  pnh_.param("route/route_links_path", route_path, std::string());
  pnh_.param("traffic_light/mock_path", traffic_light_path, std::string());
  pnh_.param("traffic_light/use_mock", use_traffic_light_mock,
             use_traffic_light_mock);
  pnh_.param("traffic_light/topic", traffic_light_topic, traffic_light_topic);
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
      avoid_target.ego_width <= 0.0 || avoid_target.path_margin_m < 0.0 ||
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
             "ceiling, 1.892 m ego width, 0.40 m margin, "
             "0.10/0.20 m overlap, margin ratio 0.20, "
             "blocking ratio 0.70/0.35/0.15, confirm/release 2/3 cycles");
    avoid_target.max_distance = 120.0;
    avoid_target.max_lateral_offset = 1.75;
    avoid_target.ego_width = 1.892;
    avoid_target.path_margin_m = 0.4;
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
  if (!map_.load(link_path, node_path)) {
    ROS_ERROR("MGeo loading failed; check map paths");
  }
  if (!route_.initialize(map_, loadRouteLinks(route_path))) {
    ROS_ERROR("Route initialization failed; check route_links.yaml");
  }
  builder_.setStopTargetOffset(stop_target_offset_m);
  traffic_stop.target_offset = stop_target_offset_m;
  decision_.setTrafficStopConfig(traffic_stop);
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
           "(ego_width=%.3f + margin=%.2f, ceiling=%.2f), "
           "overlap physical=%.2f m margin=%.2f m margin_ratio=%.2f, "
           "blocking_ratio immediate/enter/exit=%.2f/%.2f/%.2f "
           "confirm/release=%d/%d, "
           "reconnect grace=%d ds/dd/size=%.2f/%.2f/%.2f m, "
           "release grace=%d cycles",
           avoid_target.max_distance,
           std::min(avoid_target.max_lateral_offset,
                    0.5 * avoid_target.ego_width +
                        avoid_target.path_margin_m),
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
    auto mock = std::make_shared<MockTrafficLightProvider>();
    if (!mock->load(traffic_light_path)) {
      ROS_WARN("Traffic-light mock loading failed; default GREEN is used");
    }
    lights_ = mock;
    ROS_WARN("Behavior traffic-light source: YAML mock %s",
             traffic_light_path.c_str());
  } else {
    lights_ = std::make_shared<RosTrafficLightProvider>(
        nh_, traffic_light_topic, traffic_light_stale_seconds,
        traffic_light_detection_grace_seconds);
    ROS_INFO("Behavior traffic-light source: topic=%s, transport stale=%.2f s, "
             "detection grace=%.2f s",
             traffic_light_topic.c_str(), traffic_light_stale_seconds,
             traffic_light_detection_grace_seconds);
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

  ego_sub_ = nh_.subscribe(ego_topic, 5, &BehaviorNode::egoCallback, this);
  if (objects_input_type == "detected") {
    objects_sub_ = nh_.subscribe(
        objects_topic, 5, &BehaviorNode::detectedObjectsCallback, this);
  } else {
    if (objects_input_type != "morai") {
      ROS_WARN("Unknown objects/input_type '%s'; using morai",
               objects_input_type.c_str());
      objects_input_type = "morai";
    }
    objects_sub_ = nh_.subscribe(
        objects_topic, 5, &BehaviorNode::moraiObjectsCallback, this);
  }
  if (use_v2_interface_) {
    feedback_sub_ = nh_.subscribe(feedback_v2_topic, 5,
                                  &BehaviorNode::feedbackV2Callback, this);
  } else {
    feedback_sub_ =
        nh_.subscribe(feedback_topic, 5, &BehaviorNode::feedbackCallback, this);
  }

  if (rate_ <= 0.0) {
    ROS_WARN("Invalid behavior update rate %.3f Hz; using 10 Hz", rate_);
    rate_ = 10.0;
  }
  update_timer_ = nh_.createSteadyTimer(
      ros::WallDuration(1.0 / rate_), &BehaviorNode::timerCallback, this);

  ROS_INFO_STREAM("Behavior planner input: ego=" << ego_topic
                  << ", objects=" << objects_topic
                  << " (" << objects_input_type << ")");
  ROS_INFO_STREAM("Behavior interface " << (use_v2_interface_ ? "v2" : "v1")
                  << ": "
                  << (use_v2_interface_ ? command_v2_topic : context_topic)
                  << " -> "
                  << (use_v2_interface_ ? feedback_v2_topic : feedback_topic));
}

void BehaviorNode::spin() {
  ros::spin();
}

void BehaviorNode::timerCallback(const ros::SteadyTimerEvent&) { runOnce(); }

void BehaviorNode::runOnce() {
  if (!ego_received_) {
    ROS_WARN_THROTTLE(5.0, "Waiting for valid asmc_msgs/VehicleState");
    return;
  }
  const ros::SteadyTime steady_now = ros::SteadyTime::now();
  const bool objects_fresh =
      objects_received_ && objects_stale_seconds_ > 0.0 &&
      (steady_now - objects_received_at_).toSec() <= objects_stale_seconds_;
  const PerceptionObjects empty_objects;
  if (objects_received_ && !objects_fresh) {
    ROS_WARN_THROTTLE(1.0, "Behavior object input is stale; clearing targets");
  }
  const UrbanFeature feature = builder_.build(
      ego_, objects_fresh ? objects_ : empty_objects, route_, *lights_);
  const asmc_msgs::FrenetFeedback* feedback = nullptr;
  if (use_v2_interface_) {
    const bool feedback_fresh =
        feedback_v2_received_ &&
        (steady_now - feedback_v2_received_at_).toSec() <=
            feedback_v2_stale_seconds_;
    feedback = feedback_fresh ? &feedback_v2_ : nullptr;
  }
  const BehaviorContextData context = use_v2_interface_
      ? decision_.decide(feature, NormalizeFeedback(feedback),
                         v2_command_manager_.activeCommandId())
      : decision_.decide(feature, feedback_);
  if (use_v2_interface_) {
    command_v2_pub_.publish(v2_command_manager_.update(
        context, feedback, ros::Time::now(), command_validity_seconds_));
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
}

void BehaviorNode::egoCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
  if (!message) return;
  if (!message->valid || !std::isfinite(message->x) ||
      !std::isfinite(message->y) || !std::isfinite(message->yaw) ||
      !std::isfinite(message->speed) || message->header.stamp.isZero()) {
    ego_received_ = false;
    ROS_WARN_THROTTLE(2.0, "Behavior received invalid /vehicle/state");
    return;
  }
  ego_.x = message->x;
  ego_.y = message->y;
  ego_.yaw = message->yaw;
  ego_.speed = std::abs(message->speed);
  ego_.stamp = message->header.stamp.toSec();
  ego_.source = message->source;
  ego_.valid = true;
  ego_received_ = true;
}

void BehaviorNode::moraiObjectsCallback(
    const morai_msgs::ObjectStatusList::ConstPtr& message) {
  if (!message) return;
  objects_ = MoraiMsgConverter::toPerceptionObjects(*message);
  objects_received_at_ = ros::SteadyTime::now();
  objects_received_ = true;
}

void BehaviorNode::detectedObjectsCallback(
    const asmc_msgs::DetectedObjectArray::ConstPtr& message) {
  if (!message) return;
  if (message->header.stamp.isZero() || message->header.frame_id != "map") {
    ROS_WARN_THROTTLE(
        1.0, "Behavior rejected DetectedObjectArray with invalid header");
    return;
  }
  objects_ = DetectedMsgConverter::toPerceptionObjects(*message);
  objects_received_at_ = ros::SteadyTime::now();
  objects_received_ = true;
}

void BehaviorNode::feedbackCallback(
    const asmc_msgs::PlanFeedback::ConstPtr& message) {
  if (message) feedback_ = *message;
}

void BehaviorNode::feedbackV2Callback(
    const asmc_msgs::FrenetFeedback::ConstPtr& message) {
  if (!message) return;
  feedback_v2_ = *message;
  feedback_v2_received_at_ = ros::SteadyTime::now();
  feedback_v2_received_ = true;
}

}  // namespace behavior_planner

