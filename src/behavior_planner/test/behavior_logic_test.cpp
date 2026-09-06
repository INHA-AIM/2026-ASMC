#include <gtest/gtest.h>

#include "asmc_msgs/PlanFeedback.h"
#include "behavior_planner/context/msg_converter.hpp"
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

TEST(BehaviorDecisionTest, FollowReducesTargetSpeed) {
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
  EXPECT_LT(context.target.desired_speed, feature.route.speed_limit);
  EXPECT_GE(context.target.desired_speed, 0.0);
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
