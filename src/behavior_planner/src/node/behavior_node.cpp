#include "behavior_planner/node/behavior_node.hpp"

#include <cmath>

#include <asmc_msgs/BehaviorContext.h>
#include <asmc_msgs/BehaviorCommand.h>
#include <visualization_msgs/MarkerArray.h>

#include "behavior_planner/context/msg_converter.hpp"
#include "behavior_planner/global/utils.hpp"
#include "behavior_planner/node/morai_msg_converter.hpp"

namespace behavior_planner {

BehaviorNode::BehaviorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
  std::string ego_topic = "/vehicle/state";
  std::string objects_topic = "/Object_topic";
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
  double stop_target_offset_m = 5.0;
  TrafficStopConfig traffic_stop;
  SafetyConfig safety;
  FollowTargetConfig follow_target;
  AvoidTargetConfig avoid_target;
  double emergency_ttc_exit = 3.0;
  double emergency_release_hold_s = 0.5;
  double follow_release_delay_s = 0.4;

  pnh_.param("behavior/update_rate", rate_, rate_);
  pnh_.param("topics/ego", ego_topic, ego_topic);
  pnh_.param("topics/objects", objects_topic, objects_topic);
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
      avoid_target.max_lateral_offset <= 0.0) {
    ROS_WARN("Invalid AVOID target limits; using 120.0 m / 1.75 m");
    avoid_target.max_distance = 120.0;
    avoid_target.max_lateral_offset = 1.75;
  }

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
  ROS_INFO("Behavior stop target offset: %.2f m behind physical stop line",
           stop_target_offset_m);
  ROS_INFO("Behavior traffic STOP detection: min=%.2f m, dynamic=v^2/(2*%.2f)"
           "+%.2f margin+%.2f target_offset+%.2f planner_buffer",
           traffic_stop.minimum_detection_distance,
           traffic_stop.comfortable_decel, traffic_stop.braking_margin,
           traffic_stop.target_offset, traffic_stop.planner_stop_buffer);
  ROS_INFO("Behavior FOLLOW target limits: distance=%.2f m lateral=%.2f m, "
           "release delay=%.2f s, objects stale after %.2f s",
           follow_target.max_distance, follow_target.max_lateral_offset,
           follow_release_delay_s, objects_stale_seconds_);
  ROS_INFO("Behavior AVOID target limits: distance=%.2f m lateral=%.2f m",
           avoid_target.max_distance, avoid_target.max_lateral_offset);

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
        nh_, traffic_light_topic, traffic_light_stale_seconds);
    ROS_INFO("Behavior traffic-light source: %s (stale after %.2f s)",
             traffic_light_topic.c_str(), traffic_light_stale_seconds);
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
  objects_sub_ =
      nh_.subscribe(objects_topic, 5, &BehaviorNode::objectsCallback, this);
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
                  << ", objects=" << objects_topic);
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
  const BehaviorContextData context = decision_.decide(feature, feedback_);
  if (use_v2_interface_) {
    const bool feedback_fresh =
        feedback_v2_received_ &&
        (steady_now - feedback_v2_received_at_).toSec() <=
            feedback_v2_stale_seconds_;
    const asmc_msgs::FrenetFeedback* feedback =
        feedback_fresh ? &feedback_v2_ : nullptr;
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

void BehaviorNode::objectsCallback(
    const morai_msgs::ObjectStatusList::ConstPtr& message) {
  if (!message) return;
  objects_ = MoraiMsgConverter::toPerceptionObjects(*message);
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
