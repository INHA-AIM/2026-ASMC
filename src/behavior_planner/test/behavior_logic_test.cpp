#include <gtest/gtest.h>

#include "asmc_msgs/PlanFeedback.h"
#include "behavior_planner/context/msg_converter.hpp"
#include "behavior_planner/context/v2_command_manager.hpp"
#include "behavior_planner/decision/behavior_decision.hpp"
#include "behavior_planner/rule/hard_rule_filter.hpp"
#include "behavior_planner/signal/ros_traffic_light_provider.hpp"

namespace behavior_planner {

TEST(TrafficLightLabelTest, SelectsAspectForRouteMovement) {
  EXPECT_EQ(TrafficLightState::RED,
            InterpretTrafficLightLabel("red_left", SignalMovement::STRAIGHT));
  EXPECT_EQ(TrafficLightState::GREEN,
            InterpretTrafficLightLabel("red_left", SignalMovement::LEFT));
  EXPECT_EQ(TrafficLightState::GREEN,
            InterpretTrafficLightLabel("green_left", SignalMovement::STRAIGHT));
  EXPECT_EQ(TrafficLightState::GREEN,
            InterpretTrafficLightLabel("green_left", SignalMovement::LEFT));
  EXPECT_EQ(TrafficLightState::RED,
            InterpretTrafficLightLabel("green", SignalMovement::LEFT));
  EXPECT_EQ(TrafficLightState::UNKNOWN,
            InterpretTrafficLightLabel("not_detected", SignalMovement::LEFT));
}

TEST(V2CommandManagerTest, KeepFreezesDerivedSpeedUntilIntentChanges) {
  behavior_planner::V2CommandManager manager;
  behavior_planner::BehaviorContextData context;
  context.timestamp = 1.0;
  context.selected_behavior = behavior_planner::BehaviorType::KEEP;
  context.target.current_link_id = "link_1";
  context.target.desired_speed = 8.0;

  const auto first = manager.update(context, nullptr, ros::Time(10.0), 0.5);
  const auto refresh = manager.update(context, nullptr, ros::Time(10.1), 0.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::KEEP, first.maneuver_type);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::EXECUTE, first.execution_request);
  EXPECT_NE(0u, first.command_id);
  EXPECT_EQ(first.command_id, refresh.command_id);
  EXPECT_GT(refresh.valid_until, first.valid_until);

  context.target.desired_speed = 7.0;
  const auto speed_refresh =
      manager.update(context, nullptr, ros::Time(10.2), 0.5);
  EXPECT_EQ(first.command_id, speed_refresh.command_id);
  EXPECT_DOUBLE_EQ(first.desired_speed_mps, speed_refresh.desired_speed_mps);

  context.selected_behavior = behavior_planner::BehaviorType::STOP;
  context.target.has_target_pose = true;
  context.target.target_x = 20.0;
  context.target.target_y = 0.0;
  const auto changed = manager.update(context, nullptr, ros::Time(10.3), 0.5);
  EXPECT_NE(first.command_id, changed.command_id);
}

TEST(V2CommandManagerTest, LaneChangeUsesReadyExecuteHandshake) {
  behavior_planner::V2CommandManager manager;
  behavior_planner::BehaviorContextData context;
  context.timestamp = 1.0;
  context.selected_behavior = behavior_planner::BehaviorType::LEFT_CHANGE;
  context.target.current_link_id = "source";
  context.target.target_link_id = "left";
  context.target.desired_speed = 6.0;
  context.target.has_target_pose = true;
  context.target.target_x = 12.0;
  context.target.target_y = 3.5;
  context.target.target_yaw = 0.2;

  const auto prepare = manager.update(context, nullptr, ros::Time(20.0), 0.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::LANE_CHANGE, prepare.maneuver_type);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::DIRECTION_LEFT, prepare.direction);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::PREPARE, prepare.execution_request);
  EXPECT_TRUE(prepare.has_target_pose);

  asmc_msgs::FrenetFeedback feedback;
  feedback.command_id = prepare.command_id;
  feedback.execution_state = asmc_msgs::FrenetFeedback::READY;
  const auto execute =
      manager.update(context, &feedback, ros::Time(20.1), 0.5);
  EXPECT_EQ(prepare.command_id, execute.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::EXECUTE, execute.execution_request);
}

TEST(V2CommandManagerTest, StopCarriesPoseAndForceStopMapsToEmergency) {
  behavior_planner::V2CommandManager manager;
  behavior_planner::BehaviorContextData context;
  context.timestamp = 1.0;
  context.selected_behavior = behavior_planner::BehaviorType::STOP;
  context.target.current_link_id = "link";
  context.target.desired_speed = 5.0;
  context.target.has_target_pose = true;
  context.target.target_x = 30.0;
  context.target.target_y = 4.0;
  context.target.target_yaw = 0.5;

  const auto stop = manager.update(context, nullptr, ros::Time(30.0), 0.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::STOP, stop.maneuver_type);
  EXPECT_TRUE(stop.has_target_pose);
  EXPECT_DOUBLE_EQ(30.0, stop.target_pose_map.position.x);
  EXPECT_NEAR(1.0, std::hypot(stop.target_pose_map.orientation.z,
                             stop.target_pose_map.orientation.w), 1e-9);

  context.hard.force_stop = true;
  const auto emergency =
      manager.update(context, nullptr, ros::Time(30.1), 0.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::EMERGENCY_STOP,
            emergency.maneuver_type);
  EXPECT_FALSE(emergency.has_target_pose);
  EXPECT_DOUBLE_EQ(0.0, emergency.desired_speed_mps);
}

TEST(V2CommandManagerTest, AvoidLetsFrenetChooseDirectionAndUsesHandshake) {
  V2CommandManager manager;
  BehaviorContextData context;
  context.timestamp = 1.0;
  context.selected_behavior = BehaviorType::AVOID;
  context.target.current_link_id = "source";
  context.target.desired_speed = 6.0;
  context.target.avoidance_object_id = 23;

  const auto prepare = manager.update(context, nullptr, ros::Time(40.0), 0.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::AVOID, prepare.maneuver_type);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::PREPARE, prepare.execution_request);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::DIRECTION_NONE, prepare.direction);
  EXPECT_EQ(23, prepare.primary_object_id);
  EXPECT_FALSE(prepare.has_target_pose);

  asmc_msgs::FrenetFeedback feedback;
  feedback.command_id = prepare.command_id;
  feedback.execution_state = asmc_msgs::FrenetFeedback::READY;
  const auto execute = manager.update(context, &feedback, ros::Time(40.1), 0.5);
  EXPECT_EQ(prepare.command_id, execute.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::EXECUTE, execute.execution_request);

  feedback.execution_state = asmc_msgs::FrenetFeedback::EXECUTING;
  context.selected_behavior = BehaviorType::KEEP;
  const auto locked = manager.update(context, &feedback, ros::Time(40.2), 0.5);
  EXPECT_EQ(prepare.command_id, locked.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::AVOID, locked.maneuver_type);

  feedback.execution_state = asmc_msgs::FrenetFeedback::COMPLETED;
  const auto keep = manager.update(context, &feedback, ros::Time(40.3), 0.5);
  EXPECT_NE(prepare.command_id, keep.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::KEEP, keep.maneuver_type);
}

TEST(V2CommandManagerTest, EarlierPlannedStopPreemptsCommittedAvoid) {
  V2CommandManager manager;
  BehaviorContextData context;
  context.selected_behavior = BehaviorType::AVOID;
  context.target.current_link_id = "source";
  context.target.avoidance_object_id = 23;
  context.target.desired_speed = 6.0;

  const auto prepare = manager.update(context, nullptr, ros::Time(45.0), 0.5);
  asmc_msgs::FrenetFeedback feedback;
  feedback.command_id = prepare.command_id;
  feedback.execution_state = asmc_msgs::FrenetFeedback::EXECUTING;

  context.selected_behavior = BehaviorType::STOP;
  context.target.has_target_pose = true;
  context.target.target_x = 10.0;
  context.target.target_y = 0.0;
  context.extra_flags["stop_preempts_avoid"] = true;
  const auto stop = manager.update(context, &feedback, ros::Time(45.1), 0.5);
  EXPECT_NE(prepare.command_id, stop.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::STOP, stop.maneuver_type);
}

TEST(HardRuleFilterTest, GreenLightDoesNotForceStop) {
  UrbanFeature feature;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 10.0;
  feature.signal.stop_line_s = 50.0;
  feature.signal.light_state = TrafficLightState::GREEN;
  std::string reason;
  const HardConstraint hard = HardRuleFilter().apply(feature, &reason);
  EXPECT_TRUE(hard.stop_line_constraint);
  EXPECT_FALSE(hard.force_stop);
  EXPECT_EQ("no_hard_constraint", reason);
}

TEST(HardRuleFilterTest, RedLightRequestsPlannedStop) {
  UrbanFeature feature;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 10.0;
  feature.signal.stop_line_s = 50.0;
  feature.signal.light_state = TrafficLightState::RED;
  std::string reason;
  const HardConstraint hard = HardRuleFilter().apply(feature, &reason);
  EXPECT_TRUE(hard.planned_stop);
  EXPECT_FALSE(hard.force_stop);
  EXPECT_DOUBLE_EQ(50.0, hard.stop_before_s);
  EXPECT_NE(std::string::npos, reason.find("traffic_light"));
}

TEST(HardRuleFilterTest, RedLightDetectionDistanceGrowsWithSpeed) {
  TrafficStopConfig config;
  config.minimum_detection_distance = 30.0;
  config.comfortable_decel = 2.0;
  config.braking_margin = 3.0;
  config.target_offset = 5.0;
  config.planner_stop_buffer = 0.5;
  HardRuleFilter filter;
  filter.setTrafficStopConfig(config);

  UrbanFeature feature;
  feature.ego.speed = 13.9;
  feature.signal.has_stop_line = true;
  feature.signal.stop_line_s = 100.0;
  feature.signal.light_state = TrafficLightState::RED;

  // 50 km/h에서는 동적 검출거리가 약 56.8 m이므로 55 m에서도 STOP한다.
  feature.signal.distance_to_stop_line = 55.0;
  std::string reason;
  EXPECT_TRUE(filter.apply(feature, &reason).planned_stop);

  // 같은 속도에서도 실제 필요 거리 밖의 먼 신호는 아직 구속하지 않는다.
  feature.signal.distance_to_stop_line = 60.0;
  EXPECT_FALSE(filter.apply(feature, &reason).planned_stop);
}

TEST(HardRuleFilterTest, UnprotectedRightIgnoresVehicleRedLight) {
  UrbanFeature feature;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 10.0;
  feature.signal.stop_line_s = 50.0;
  feature.signal.has_required_movement = true;
  feature.signal.required_movement = SignalMovement::RIGHT_UNPROTECTED;
  feature.signal.light_state = TrafficLightState::RED;

  std::string reason;
  const HardConstraint hard = HardRuleFilter().apply(feature, &reason);
  EXPECT_TRUE(hard.stop_line_constraint);
  EXPECT_FALSE(hard.planned_stop);
  EXPECT_FALSE(hard.force_stop);
  EXPECT_EQ("no_hard_constraint", reason);
}

TEST(HardRuleFilterTest, UnprotectedRightStillStopsForPedestrianRisk) {
  UrbanFeature feature;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 10.0;
  feature.signal.stop_line_s = 50.0;
  feature.signal.has_required_movement = true;
  feature.signal.required_movement = SignalMovement::RIGHT_UNPROTECTED;
  feature.signal.light_state = TrafficLightState::RED;
  feature.risk.pedestrian_risk = true;

  std::string reason;
  const HardConstraint hard = HardRuleFilter().apply(feature, &reason);
  EXPECT_FALSE(hard.planned_stop);
  EXPECT_TRUE(hard.force_stop);
  EXPECT_NE(std::string::npos, reason.find("pedestrian_risk"));
}

TEST(BehaviorDecisionTest, RedLightPublishesPlannedStopTarget) {
  UrbanFeature feature;
  feature.timestamp = 1.0;
  feature.route.speed_limit = 10.0;
  feature.signal.has_stop_line = true;
  feature.signal.stop_line_valid = true;
  feature.signal.distance_to_stop_line = 10.0;
  feature.signal.stop_line_s = 50.0;
  feature.signal.stop_line_x = 123.4;
  feature.signal.stop_line_y = 567.8;
  feature.signal.light_state = TrafficLightState::RED;

  const BehaviorContextData context =
      BehaviorDecision().decide(feature, asmc_msgs::PlanFeedback{});
  ASSERT_EQ(BehaviorType::STOP, context.selected_behavior);
  EXPECT_FALSE(context.hard.force_stop);
  EXPECT_TRUE(context.target.has_stop_target);
  EXPECT_DOUBLE_EQ(10.0, context.target.desired_speed);

  const asmc_msgs::BehaviorContext message = MsgConverter::toMsg(context, 1);
  EXPECT_EQ(asmc_msgs::BehaviorContext::STOP, message.selected_behavior);
  EXPECT_FALSE(message.force_stop);
  EXPECT_TRUE(message.has_stop_target);
  EXPECT_DOUBLE_EQ(123.4, message.stop_x);
  EXPECT_DOUBLE_EQ(567.8, message.stop_y);

  const UrbanFeatureDebug debug = MsgConverter::toDebugMsg(feature, context);
  EXPECT_EQ(static_cast<uint8_t>(BehaviorType::STOP), debug.selected_behavior);
  EXPECT_TRUE(debug.planned_stop);
  EXPECT_FALSE(debug.force_stop);
  EXPECT_TRUE(debug.has_stop_target);
  EXPECT_DOUBLE_EQ(123.4, debug.stop_x);
  EXPECT_DOUBLE_EQ(567.8, debug.stop_y);
  EXPECT_NE(std::string::npos, debug.hard_rule_reason.find("traffic_light"));
}

TEST(BehaviorDecisionTest, EmptyRoadSelectsKeep) {
  UrbanFeature feature;
  feature.route.speed_limit = 13.9;
  asmc_msgs::PlanFeedback feedback;
  const BehaviorContextData context = BehaviorDecision().decide(feature, feedback);
  EXPECT_EQ(BehaviorType::KEEP, context.selected_behavior);
  EXPECT_FALSE(context.enable.follow);
}

TEST(BehaviorDecisionTest, StaticObstacleSelectsAvoidTarget) {
  UrbanFeature feature;
  feature.timestamp = 1.0;
  feature.route.speed_limit = 10.0;
  feature.risk.static_obstacle_ahead = true;
  feature.risk.avoidance_object_id = 31;

  const BehaviorContextData context =
      BehaviorDecision().decide(feature, asmc_msgs::PlanFeedback{});
  EXPECT_EQ(BehaviorType::AVOID, context.selected_behavior);
  EXPECT_TRUE(context.enable.avoid);
  EXPECT_EQ(31, context.target.avoidance_object_id);
}

TEST(BehaviorDecisionTest, ChoosesFirstLongitudinalConstraint) {
  UrbanFeature feature;
  feature.route.speed_limit = 10.0;
  feature.signal.has_stop_line = true;
  feature.signal.stop_line_valid = true;
  feature.signal.distance_to_stop_line = 20.0;
  feature.signal.stop_line_s = 120.0;
  feature.signal.stop_target_s = 115.0;
  feature.signal.stop_line_x = 115.0;
  feature.signal.light_state = TrafficLightState::RED;
  feature.risk.static_obstacle_ahead = true;
  feature.risk.avoidance_object_id = 31;
  feature.risk.nearest_obstacle_length = 4.0;

  feature.risk.nearest_obstacle_s = 130.0;
  BehaviorContextData stop_first =
      BehaviorDecision().decide(feature, asmc_msgs::PlanFeedback{});
  EXPECT_EQ(BehaviorType::STOP, stop_first.selected_behavior);
  EXPECT_TRUE(stop_first.extra_flags["stop_preempts_avoid"]);
  EXPECT_EQ("stop_before_avoid", stop_first.debug.selected_reason);

  feature.risk.nearest_obstacle_s = 110.0;
  BehaviorContextData avoid_first =
      BehaviorDecision().decide(feature, asmc_msgs::PlanFeedback{});
  EXPECT_EQ(BehaviorType::AVOID, avoid_first.selected_behavior);
  EXPECT_TRUE(avoid_first.extra_flags["avoid_precedes_stop"]);
  EXPECT_EQ("avoid_before_stop", avoid_first.debug.selected_reason);
}

TEST(FsmTest, EmergencyReleaseRequiresSafeTtcHoldTime) {
  behavior_planner::Fsm fsm;
  fsm.setEmergencyRelease(3.0, 0.5);

  EXPECT_TRUE(fsm.updateEmergency(true, 0.8, 10.0));
  EXPECT_TRUE(fsm.emergencyLatched());
  EXPECT_TRUE(fsm.updateEmergency(false, 10.0, 10.1));
  EXPECT_TRUE(fsm.updateEmergency(false, 10.0, 10.59));
  EXPECT_FALSE(fsm.updateEmergency(false, 10.0, 10.60));
  EXPECT_FALSE(fsm.emergencyLatched());
}

TEST(FsmTest, FollowReleaseKeepsLastTargetForConfiguredDelay) {
  behavior_planner::Fsm fsm;
  fsm.setFollowReleaseDelay(0.4);

  EXPECT_EQ(7, fsm.updateFollowTarget(true, 7, 10.0));
  EXPECT_FALSE(fsm.followReleaseGraceActive());
  EXPECT_EQ(7, fsm.updateFollowTarget(false, -1, 10.2));
  EXPECT_TRUE(fsm.followReleaseGraceActive());
  EXPECT_EQ(7, fsm.updateFollowTarget(false, -1, 10.39));
  EXPECT_EQ(-1, fsm.updateFollowTarget(false, -1, 10.41));
  EXPECT_FALSE(fsm.followReleaseGraceActive());
}

TEST(BehaviorDecisionTest, FollowDropoutGraceRetainsTargetThenReturnsKeep) {
  BehaviorDecision decision;
  decision.setFollowReleaseDelay(0.4);
  UrbanFeature feature;
  feature.timestamp = 10.0;
  feature.route.speed_limit = 13.9;
  feature.surrounding.has_front_vehicle = true;
  feature.surrounding.front_vehicle.id = 7;
  feature.surrounding.front_vehicle.gap = 10.0;
  feature.surrounding.front_vehicle.ttc = 2.0;

  EXPECT_EQ(BehaviorType::FOLLOW,
            decision.decide(feature, asmc_msgs::PlanFeedback{})
                .selected_behavior);

  feature.timestamp = 10.2;
  feature.surrounding.has_front_vehicle = false;
  const BehaviorContextData grace =
      decision.decide(feature, asmc_msgs::PlanFeedback{});
  EXPECT_EQ(BehaviorType::FOLLOW, grace.selected_behavior);
  EXPECT_EQ(7, grace.target.lead_vehicle_id);
  EXPECT_TRUE(grace.extra_flags.at("follow_release_grace"));

  feature.timestamp = 10.41;
  EXPECT_EQ(BehaviorType::KEEP,
            decision.decide(feature, asmc_msgs::PlanFeedback{})
                .selected_behavior);
}

TEST(BehaviorDecisionTest, FollowKeepsCruiseSpeedCeilingForFrenetGapControl) {
  UrbanFeature feature;
  feature.ego.speed = 12.0;
  feature.route.speed_limit = 13.9;
  feature.surrounding.has_front_vehicle = true;
  feature.surrounding.front_vehicle.id = 7;
  feature.surrounding.front_vehicle.speed = 4.0;
  feature.surrounding.front_vehicle.gap = 10.0;
  feature.surrounding.front_vehicle.ttc = 2.0;
  feature.surrounding.front_vehicle.relative_speed = 8.0;
  asmc_msgs::PlanFeedback feedback;
  const BehaviorContextData context = BehaviorDecision().decide(feature, feedback);
  EXPECT_EQ(BehaviorType::FOLLOW, context.selected_behavior);
  EXPECT_DOUBLE_EQ(feature.route.speed_limit, context.target.desired_speed);
  EXPECT_EQ(7, context.target.lead_vehicle_id);
}

TEST(MsgConverterTest, MapsKeepContractAndCommandId) {
  BehaviorContextData context;
  context.timestamp = 12.5;
  context.selected_behavior = BehaviorType::KEEP;
  context.target.desired_speed = 8.0;
  context.hard.max_speed = 1e9;
  context.target.target_link_id = "link-7";

  const asmc_msgs::BehaviorContext message =
      MsgConverter::toMsg(context, 42);

  EXPECT_EQ(42u, message.command_id);
  EXPECT_TRUE(message.valid);
  EXPECT_EQ(asmc_msgs::BehaviorContext::KEEP, message.selected_behavior);
  EXPECT_TRUE(message.has_desired_speed);
  EXPECT_DOUBLE_EQ(8.0, message.desired_speed);
  EXPECT_FALSE(message.has_max_speed);
  EXPECT_FALSE(message.has_target_lane);
  EXPECT_FALSE(message.has_target_d);
  EXPECT_FALSE(message.has_stop_target);
  EXPECT_EQ("link-7", message.target_link_id);
}

TEST(MsgConverterTest, PreservesFollowWithoutForcingKeep) {
  BehaviorContextData context;
  context.timestamp = 1.0;
  context.selected_behavior = BehaviorType::FOLLOW;
  context.hard.max_speed = 6.0;
  context.target.lead_vehicle_id = 17;

  const asmc_msgs::BehaviorContext message =
      MsgConverter::toMsg(context, 9);

  EXPECT_TRUE(message.valid);
  EXPECT_EQ(asmc_msgs::BehaviorContext::FOLLOW, message.selected_behavior);
  EXPECT_TRUE(message.has_max_speed);
  EXPECT_DOUBLE_EQ(6.0, message.max_speed);
  EXPECT_TRUE(message.has_lead_vehicle);
  EXPECT_EQ(17, message.lead_vehicle_id);
}

TEST(MsgConverterTest, CommandIdentityIgnoresPublishMetadataAndTinyNoise) {
  BehaviorContextData context;
  context.timestamp = 1.0;
  context.target.desired_speed = 8.0;
  const asmc_msgs::BehaviorContext first = MsgConverter::toMsg(context, 1);

  context.timestamp = 2.0;
  context.target.desired_speed += 0.0005;
  const asmc_msgs::BehaviorContext repeated = MsgConverter::toMsg(context, 2);

  EXPECT_TRUE(SameCommandIdentity(first, repeated));
}

TEST(MsgConverterTest, CommandIdentityChangesWithExecutionMeaning) {
  BehaviorContextData context;
  context.timestamp = 1.0;
  context.target.desired_speed = 8.0;
  const asmc_msgs::BehaviorContext keep = MsgConverter::toMsg(context, 1);

  context.target.desired_speed = 8.1;
  const asmc_msgs::BehaviorContext faster = MsgConverter::toMsg(context, 1);
  EXPECT_FALSE(SameCommandIdentity(keep, faster));

  context.target.desired_speed = 8.0;
  context.selected_behavior = BehaviorType::FOLLOW;
  const asmc_msgs::BehaviorContext follow = MsgConverter::toMsg(context, 1);
  EXPECT_FALSE(SameCommandIdentity(keep, follow));
}

}  // namespace behavior_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
