#include "behavior_planner/node/behavior_node.hpp"

#include <cmath>

#include <asmc_msgs/BehaviorContext.h>
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
  std::string context_topic = "/behavior/context";
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

  pnh_.param("behavior/update_rate", rate_, rate_);
  pnh_.param("topics/ego", ego_topic, ego_topic);
  pnh_.param("topics/objects", objects_topic, objects_topic);
  pnh_.param("topics/plan_feedback", feedback_topic, feedback_topic);
  pnh_.param("publish_topics/behavior_context", context_topic, context_topic);
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

  if (!map_.load(link_path, node_path)) {
    ROS_ERROR("MGeo loading failed; check map paths");
  }
  if (!route_.initialize(map_, loadRouteLinks(route_path))) {
    ROS_ERROR("Route initialization failed; check route_links.yaml");
  }
  builder_.setStopTargetOffset(stop_target_offset_m);
  ROS_INFO("Behavior stop target offset: %.2f m behind physical stop line",
           stop_target_offset_m);

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

  context_pub_ =
      nh_.advertise<asmc_msgs::BehaviorContext>(context_topic, 10);
  debug_pub_ = nh_.advertise<UrbanFeatureDebug>(debug_topic, 10);
  marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 2);
  visualization_ = BehaviorVisualization(marker_pub_);

  ego_sub_ = nh_.subscribe(ego_topic, 5, &BehaviorNode::egoCallback, this);
  objects_sub_ =
      nh_.subscribe(objects_topic, 5, &BehaviorNode::objectsCallback, this);
  feedback_sub_ =
      nh_.subscribe(feedback_topic, 5, &BehaviorNode::feedbackCallback, this);

  if (rate_ <= 0.0) {
    ROS_WARN("Invalid behavior update rate %.3f Hz; using 10 Hz", rate_);
    rate_ = 10.0;
  }
  update_timer_ = nh_.createSteadyTimer(
      ros::WallDuration(1.0 / rate_), &BehaviorNode::timerCallback, this);

  ROS_INFO_STREAM("Behavior planner input: ego=" << ego_topic
                  << ", objects=" << objects_topic);
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
  const UrbanFeature feature =
      builder_.build(ego_, objects_, route_, *lights_);
  const BehaviorContextData context = decision_.decide(feature, feedback_);
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
}

void BehaviorNode::feedbackCallback(
    const asmc_msgs::PlanFeedback::ConstPtr& message) {
  if (message) feedback_ = *message;
}

}  // namespace behavior_planner
