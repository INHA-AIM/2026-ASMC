#include <gtest/gtest.h>

#include "asmc_msgs/PlanFeedback.h"
#include "behavior_planner/context/msg_converter.hpp"
#include "behavior_planner/context/v2_command_manager.hpp"
#include "behavior_planner/decision/behavior_decision.hpp"
#include "behavior_planner/decision/fsm.hpp"
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

TEST(V2CommandManagerTest, SignalSpeedPolicyRefreshesKeepOnlyOnTransitions) {
  V2CommandManager manager;
  BehaviorContextData context;
  context.timestamp = 1.0;
  context.selected_behavior = BehaviorType::KEEP;
  context.target.current_link_id = "link_1";
  context.target.desired_speed = 13.9;

  const auto cruise =
      manager.update(context, nullptr, ros::Time(11.0), 0.5);

  context.target.desired_speed = 20.0 / 3.6;
  context.extra_flags["signal_approach_speed_limit"] = true;
  const auto limited =
      manager.update(context, nullptr, ros::Time(11.1), 0.5);
  EXPECT_NE(cruise.command_id, limited.command_id);
  EXPECT_NEAR(20.0 / 3.6, limited.desired_speed_mps, 1e-9);

  context.target.desired_speed = 5.4;
  const auto limited_refresh =
      manager.update(context, nullptr, ros::Time(11.2), 0.5);
  EXPECT_EQ(limited.command_id, limited_refresh.command_id);
  EXPECT_DOUBLE_EQ(limited.desired_speed_mps,
                   limited_refresh.desired_speed_mps);

  context.target.desired_speed = 13.9;
  context.extra_flags.erase("signal_approach_speed_limit");
  const auto released =
      manager.update(context, nullptr, ros::Time(11.3), 0.5);
  EXPECT_NE(limited.command_id, released.command_id);
  EXPECT_DOUBLE_EQ(13.9, released.desired_speed_mps);
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
  context.maneuver_phase = ManeuverPhase::PREPARE;

  const auto prepare = manager.update(context, nullptr, ros::Time(20.0), 0.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::LANE_CHANGE, prepare.maneuver_type);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::DIRECTION_LEFT, prepare.direction);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::PREPARE, prepare.execution_request);
  EXPECT_TRUE(prepare.has_target_pose);

  asmc_msgs::FrenetFeedback feedback;
  feedback.command_id = prepare.command_id;
  feedback.execution_state = asmc_msgs::FrenetFeedback::READY;
  context.maneuver_phase = ManeuverPhase::EXECUTE;
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
  context.maneuver_phase = ManeuverPhase::PREPARE;

  const auto prepare = manager.update(context, nullptr, ros::Time(40.0), 0.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::AVOID, prepare.maneuver_type);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::PREPARE, prepare.execution_request);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::DIRECTION_NONE, prepare.direction);
  EXPECT_EQ(23, prepare.primary_object_id);
  EXPECT_FALSE(prepare.has_target_pose);

  asmc_msgs::FrenetFeedback feedback;
  feedback.command_id = prepare.command_id;
  feedback.execution_state = asmc_msgs::FrenetFeedback::READY;
  context.maneuver_phase = ManeuverPhase::EXECUTE;
  const auto execute = manager.update(context, &feedback, ros::Time(40.1), 0.5);
  EXPECT_EQ(prepare.command_id, execute.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::EXECUTE, execute.execution_request);

  feedback.execution_state = asmc_msgs::FrenetFeedback::EXECUTING;
  // The command manager consumes the FSM output. A committed FSM keeps AVOID,
  // so the serializer must preserve the same transaction.
  context.selected_behavior = BehaviorType::AVOID;
  const auto locked = manager.update(context, &feedback, ros::Time(40.2), 0.5);
  EXPECT_EQ(prepare.command_id, locked.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::AVOID, locked.maneuver_type);

  feedback.execution_state = asmc_msgs::FrenetFeedback::COMPLETED;
  context.selected_behavior = BehaviorType::KEEP;
  const auto keep = manager.update(context, &feedback, ros::Time(40.3), 0.5);
  EXPECT_NE(prepare.command_id, keep.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::KEEP, keep.maneuver_type);
}

TEST(V2CommandManagerTest, AvoidKeepsTransactionAcrossPerceptionIdChange) {
  V2CommandManager manager;
  BehaviorContextData context;
  context.selected_behavior = BehaviorType::AVOID;
  context.target.current_link_id = "source";
  context.target.avoidance_object_id = 150;
  context.maneuver_phase = ManeuverPhase::PREPARE;

  const auto first = manager.update(context, nullptr, ros::Time(41.0), 0.5);
  context.target.current_link_id = "next_source";
  context.target.avoidance_object_id = 191;
  const auto changed_detection =
      manager.update(context, nullptr, ros::Time(41.1), 0.5);

  EXPECT_EQ(first.command_id, changed_detection.command_id);
  EXPECT_EQ(150, changed_detection.primary_object_id);
  EXPECT_EQ("source", changed_detection.source_link_id);

  asmc_msgs::FrenetFeedback failed;
  failed.command_id = first.command_id;
  failed.execution_state = asmc_msgs::FrenetFeedback::FAILED;
  context.target.current_link_id = "next_source";
  context.target.avoidance_object_id = 191;
  const auto restarted =
      manager.update(context, &failed, ros::Time(41.2), 0.5);
  EXPECT_NE(first.command_id, restarted.command_id);
  EXPECT_EQ(191, restarted.primary_object_id);
  EXPECT_EQ("next_source", restarted.source_link_id);
}

TEST(V2CommandManagerTest, FsmModeTransitionPreemptsLockedAvoid) {
  V2CommandManager manager;
  BehaviorContextData context;
  context.selected_behavior = BehaviorType::AVOID;
  context.target.current_link_id = "source";
  context.target.avoidance_object_id = 150;
  context.maneuver_phase = ManeuverPhase::PREPARE;

  const auto avoid = manager.update(context, nullptr, ros::Time(42.0), 0.5);
  asmc_msgs::FrenetFeedback executing;
  executing.command_id = avoid.command_id;
  executing.execution_state = asmc_msgs::FrenetFeedback::EXECUTING;
  manager.update(context, &executing, ros::Time(42.1), 0.5);

  context.selected_behavior = BehaviorType::FOLLOW;
  context.target.lead_vehicle_id = 191;
  context.maneuver_phase = ManeuverPhase::NONE;
  const auto follow =
      manager.update(context, &executing, ros::Time(42.2), 0.5);

  EXPECT_NE(avoid.command_id, follow.command_id);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::FOLLOW, follow.maneuver_type);
  EXPECT_EQ(191, follow.primary_object_id);
}

TEST(BehaviorFsmTest, KeepFollowKeepUsesCandidateWithoutLateralLifecycle) {
  Fsm fsm;
  const PlannerFeedbackData no_feedback;

  auto output = fsm.updateManeuver(BehaviorType::KEEP, no_feedback, 0, false);
  EXPECT_EQ(BehaviorType::KEEP, output.behavior);
  EXPECT_EQ(ManeuverPhase::NONE, output.phase);

  output = fsm.updateManeuver(BehaviorType::FOLLOW, no_feedback, 0, false);
  EXPECT_EQ(BehaviorType::FOLLOW, output.behavior);
  EXPECT_EQ(BehaviorState::FOLLOW, output.state);

  output = fsm.updateManeuver(BehaviorType::KEEP, no_feedback, 0, false);
  EXPECT_EQ(BehaviorType::KEEP, output.behavior);
  EXPECT_EQ(BehaviorState::KEEP_LANE, output.state);
}

TEST(BehaviorFsmTest, AvoidLifecycleUsesOnlyMatchingFreshFeedback) {
  Fsm fsm;
  PlannerFeedbackData feedback;

  auto output = fsm.updateManeuver(BehaviorType::AVOID, feedback, 0, false);
  EXPECT_EQ(BehaviorType::AVOID, output.behavior);
  EXPECT_EQ(ManeuverPhase::PREPARE, output.phase);

  output = fsm.updateManeuver(BehaviorType::AVOID, feedback, 42, false);
  EXPECT_EQ(ManeuverPhase::PREPARE, output.phase);

  feedback.valid = true;
  feedback.command_id = 99;
  feedback.execution_state = PlannerExecutionState::READY;
  output = fsm.updateManeuver(BehaviorType::AVOID, feedback, 42, false);
  EXPECT_EQ(ManeuverPhase::PREPARE, output.phase);

  feedback.command_id = 42;
  output = fsm.updateManeuver(BehaviorType::AVOID, feedback, 42, false);
  EXPECT_EQ(ManeuverPhase::EXECUTE, output.phase);
  EXPECT_EQ(BehaviorState::AVOID_EXECUTE, output.state);

  feedback.execution_state = PlannerExecutionState::EXECUTING;
  output = fsm.updateManeuver(BehaviorType::KEEP, feedback, 42, false);
  EXPECT_EQ(BehaviorType::AVOID, output.behavior);
  EXPECT_EQ(ManeuverPhase::COMMITTED, output.phase);

  feedback.valid = false;
  output = fsm.updateManeuver(BehaviorType::KEEP, feedback, 42, false);
  EXPECT_EQ(BehaviorType::AVOID, output.behavior);
  EXPECT_EQ(ManeuverPhase::COMMITTED, output.phase);

  feedback.valid = true;
  feedback.command_id = 99;
  feedback.execution_state = PlannerExecutionState::COMPLETED;
  output = fsm.updateManeuver(BehaviorType::KEEP, feedback, 42, false);
  EXPECT_EQ(BehaviorType::AVOID, output.behavior);

  feedback.command_id = 42;
  output = fsm.updateManeuver(BehaviorType::KEEP, feedback, 42, false);
  EXPECT_EQ(BehaviorType::KEEP, output.behavior);
  EXPECT_EQ(ManeuverPhase::NONE, output.phase);
}

TEST(BehaviorFsmTest, AvoidPrepareSurvivesShortKeepDropout) {
  Fsm fsm;
  fsm.setAvoidReleaseGraceCycles(2);
  PlannerFeedbackData feedback;

  auto output = fsm.updateManeuver(BehaviorType::AVOID, feedback, 0, false);
  output = fsm.updateManeuver(BehaviorType::AVOID, feedback, 51, false);
  output = fsm.updateManeuver(BehaviorType::KEEP, feedback, 51, false);
  EXPECT_EQ(BehaviorType::AVOID, output.behavior);
  output = fsm.updateManeuver(BehaviorType::KEEP, feedback, 51, false);
  EXPECT_EQ(BehaviorType::AVOID, output.behavior);
  output = fsm.updateManeuver(BehaviorType::KEEP, feedback, 51, false);
  EXPECT_EQ(BehaviorType::KEEP, output.behavior);
}

TEST(BehaviorFsmTest, MovingReclassificationCancelsCommittedAvoid) {
  Fsm fsm;
  PlannerFeedbackData feedback;
  fsm.updateManeuver(BehaviorType::AVOID, feedback, 0, false);
  fsm.updateManeuver(BehaviorType::AVOID, feedback, 61, false);
  feedback.valid = true;
  feedback.command_id = 61;
  feedback.execution_state = PlannerExecutionState::EXECUTING;
  ASSERT_EQ(ManeuverPhase::COMMITTED,
            fsm.updateManeuver(BehaviorType::AVOID, feedback, 61, false)
                .phase);

  const auto output = fsm.updateManeuver(
      BehaviorType::FOLLOW, feedback, 61, false, true);
  EXPECT_EQ(BehaviorType::FOLLOW, output.behavior);
  EXPECT_EQ(ManeuverPhase::NONE, output.phase);
}

TEST(BehaviorFsmTest, RetainedFollowTargetOwnsTransientStaticAvoid) {
  Fsm fsm;
  PlannerFeedbackData feedback;
  fsm.updateManeuver(BehaviorType::AVOID, feedback, 0, false);
  fsm.updateManeuver(BehaviorType::AVOID, feedback, 61, false);
  feedback.valid = true;
  feedback.command_id = 61;
  feedback.execution_state = PlannerExecutionState::EXECUTING;
  ASSERT_EQ(ManeuverPhase::COMMITTED,
            fsm.updateManeuver(BehaviorType::AVOID, feedback, 61, false)
                .phase);

  const auto output = fsm.updateManeuver(
      BehaviorType::AVOID, feedback, 61, false,
      /*cancel_reclassified_avoid=*/false,
      /*follow_owns_avoid_target=*/true);
  EXPECT_EQ(BehaviorType::FOLLOW, output.behavior);
  EXPECT_EQ(ManeuverPhase::NONE, output.phase);
}

TEST(BehaviorFsmTest, LaneChangeUsesSameLifecycleWithoutEnablingSelection) {
  Fsm fsm;
  PlannerFeedbackData feedback;

  auto output =
      fsm.updateManeuver(BehaviorType::LEFT_CHANGE, feedback, 0, false);
  EXPECT_EQ(ManeuverPhase::PREPARE, output.phase);
  output = fsm.updateManeuver(BehaviorType::LEFT_CHANGE, feedback, 17, false);

  feedback.valid = true;
  feedback.command_id = 17;
  feedback.execution_state = PlannerExecutionState::READY;
  output = fsm.updateManeuver(BehaviorType::LEFT_CHANGE, feedback, 17, false);
  EXPECT_EQ(ManeuverPhase::EXECUTE, output.phase);

  feedback.execution_state = PlannerExecutionState::COMPLETED;
  output = fsm.updateManeuver(BehaviorType::KEEP, feedback, 17, false);
  EXPECT_EQ(BehaviorType::KEEP, output.behavior);
}

TEST(BehaviorFsmTest, EmergencyAndEarlierStopPreemptCommittedAvoid) {
  Fsm fsm;
  PlannerFeedbackData feedback;
  fsm.updateManeuver(BehaviorType::AVOID, feedback, 0, false);
  fsm.updateManeuver(BehaviorType::AVOID, feedback, 8, false);
  feedback.valid = true;
  feedback.command_id = 8;
  feedback.execution_state = PlannerExecutionState::COMMITTED;
  auto output = fsm.updateManeuver(BehaviorType::AVOID, feedback, 8, false);
  ASSERT_EQ(ManeuverPhase::COMMITTED, output.phase);

  output = fsm.updateManeuver(BehaviorType::STOP, feedback, 8, true);
  EXPECT_EQ(BehaviorType::STOP, output.behavior);
  EXPECT_EQ(ManeuverPhase::NONE, output.phase);

  fsm.updateManeuver(BehaviorType::AVOID, PlannerFeedbackData{}, 0, false);
  output =
      fsm.updateManeuver(BehaviorType::EMERGENCY_STOP, feedback, 9, false);
  EXPECT_EQ(BehaviorType::EMERGENCY_STOP, output.behavior);
  EXPECT_EQ(BehaviorState::EMERGENCY_STOP, output.state);
  EXPECT_EQ(ManeuverPhase::NONE, output.phase);
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

TEST(HardRuleFilterTest, GreenLightUsesTwentyKphLimitWithoutStop) {
  UrbanFeature feature;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 10.0;
  feature.signal.stop_line_s = 50.0;
  feature.signal.light_state = TrafficLightState::GREEN;
  std::string reason;
  const HardConstraint hard = HardRuleFilter().apply(feature, &reason);
  EXPECT_TRUE(hard.stop_line_constraint);
  EXPECT_FALSE(hard.force_stop);
  EXPECT_FALSE(hard.planned_stop);
  EXPECT_TRUE(hard.signal_approach_speed_limit);
  EXPECT_NEAR(20.0 / 3.6, hard.max_speed, 1e-9);
  EXPECT_NE(std::string::npos, reason.find("visible_signal_speed_limit"));
}

TEST(HardRuleFilterTest, UnknownSignalDoesNotApplyApproachSpeedLimit) {
  UrbanFeature feature;
  feature.route.speed_limit = 13.9;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 50.0;
  feature.signal.light_state = TrafficLightState::UNKNOWN;

  const HardConstraint hard = HardRuleFilter().apply(feature, nullptr);
  EXPECT_FALSE(hard.signal_approach_speed_limit);
  EXPECT_DOUBLE_EQ(13.9, hard.max_speed);
}

TEST(HardRuleFilterTest, VisibleSignalPreservesLowerRouteSpeedLimit) {
  UrbanFeature feature;
  feature.route.speed_limit = 4.0;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 50.0;
  feature.signal.light_state = TrafficLightState::GREEN;

  const HardConstraint hard = HardRuleFilter().apply(feature, nullptr);
  EXPECT_TRUE(hard.signal_approach_speed_limit);
  EXPECT_DOUBLE_EQ(4.0, hard.max_speed);
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
  EXPECT_TRUE(hard.signal_approach_speed_limit);
  EXPECT_NE(std::string::npos, reason.find("visible_signal_speed_limit"));
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
  EXPECT_NEAR(20.0 / 3.6, context.target.desired_speed, 1e-9);
  EXPECT_TRUE(context.extra_flags.at("signal_approach_speed_limit"));

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

TEST(BehaviorDecisionTest, MovingReclassificationSwitchesCommittedAvoidToFollow) {
  BehaviorDecision decision;
  UrbanFeature feature;
  feature.timestamp = 1.0;
  feature.route.speed_limit = 10.0;
  feature.risk.static_obstacle_ahead = true;
  feature.risk.avoidance_object_id = 31;

  EXPECT_EQ(BehaviorType::AVOID,
            decision.decide(feature, PlannerFeedbackData{}, 0)
                .selected_behavior);
  EXPECT_EQ(BehaviorType::AVOID,
            decision.decide(feature, PlannerFeedbackData{}, 77)
                .selected_behavior);

  PlannerFeedbackData executing;
  executing.valid = true;
  executing.command_id = 77;
  executing.execution_state = PlannerExecutionState::EXECUTING;
  ASSERT_EQ(ManeuverPhase::COMMITTED,
            decision.decide(feature, executing, 77).maneuver_phase);

  feature.timestamp = 1.1;
  feature.risk.static_obstacle_ahead = false;
  feature.risk.avoidance_object_id = -1;
  feature.risk.avoid_target_reclassified_moving = true;
  feature.surrounding.has_front_vehicle = true;
  feature.surrounding.front_vehicle.id = 31;
  feature.surrounding.front_vehicle.gap = 12.0;
  feature.surrounding.front_vehicle.ttc = 6.0;
  const BehaviorContextData follow = decision.decide(feature, executing, 77);

  EXPECT_EQ(BehaviorType::FOLLOW, follow.selected_behavior);
  EXPECT_EQ(ManeuverPhase::NONE, follow.maneuver_phase);
  EXPECT_EQ(31, follow.target.lead_vehicle_id);
  EXPECT_TRUE(follow.extra_flags.at("cancel_reclassified_avoid"));
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

TEST(BehaviorDecisionTest, SameTargetStaticBlipStaysFollowThenAllowsAvoid) {
  BehaviorDecision decision;
  decision.setFollowReleaseDelay(0.4);
  UrbanFeature feature;
  feature.timestamp = 20.0;
  feature.route.speed_limit = 13.9;
  feature.surrounding.has_front_vehicle = true;
  feature.surrounding.front_vehicle.id = 7;
  feature.surrounding.front_vehicle.gap = 10.0;
  feature.surrounding.front_vehicle.ttc = 3.0;
  ASSERT_EQ(BehaviorType::FOLLOW,
            decision.decide(feature, PlannerFeedbackData{}).selected_behavior);

  feature.timestamp = 20.2;
  feature.surrounding.has_front_vehicle = false;
  feature.risk.static_obstacle_ahead = true;
  feature.risk.avoidance_object_id = 7;
  feature.risk.nearest_obstacle_s = 50.0;
  feature.risk.nearest_obstacle_length = 2.0;
  const BehaviorContextData retained =
      decision.decide(feature, PlannerFeedbackData{});
  EXPECT_EQ(BehaviorType::FOLLOW, retained.selected_behavior);
  EXPECT_EQ(7, retained.target.lead_vehicle_id);
  EXPECT_TRUE(retained.extra_flags.at("fsm_follow_owns_avoid_target"));
  EXPECT_EQ("fsm_follow_owns_same_target", retained.debug.selected_reason);

  feature.timestamp = 20.41;
  const BehaviorContextData stable_static =
      decision.decide(feature, PlannerFeedbackData{});
  EXPECT_EQ(BehaviorType::AVOID, stable_static.selected_behavior);
}

TEST(BehaviorDecisionTest, DistinctStaticObstacleStillPreemptsFollow) {
  BehaviorDecision decision;
  UrbanFeature feature;
  feature.timestamp = 30.0;
  feature.route.speed_limit = 13.9;
  feature.surrounding.has_front_vehicle = true;
  feature.surrounding.front_vehicle.id = 7;
  feature.surrounding.front_vehicle.gap = 20.0;
  feature.surrounding.front_vehicle.ttc = 10.0;
  feature.risk.static_obstacle_ahead = true;
  feature.risk.avoidance_object_id = 31;
  feature.risk.nearest_obstacle_s = 10.0;
  feature.risk.nearest_obstacle_length = 2.0;

  const BehaviorContextData context =
      decision.decide(feature, PlannerFeedbackData{});
  EXPECT_EQ(BehaviorType::AVOID, context.selected_behavior);
  EXPECT_EQ(0U, context.extra_flags.count("fsm_follow_owns_avoid_target"));
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
