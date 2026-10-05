#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "behavior_planner/feature/feature_builder.hpp"
#include "behavior_planner/global/utils.hpp"
#include "behavior_planner/map/mgeo_loader.hpp"
#include "behavior_planner/map/route_manager.hpp"
#include "behavior_planner/signal/traffic_light_provider.hpp"

namespace behavior_planner {
namespace {

class GreenTrafficLight final : public TrafficLightProvider {
 public:
  TrafficLightState getState(const std::string&,
                             SignalMovement) const override {
    return TrafficLightState::GREEN;
  }
};

class FeatureBuilderAvoidTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string root = BEHAVIOR_PLANNER_SOURCE_DIR;
    ASSERT_TRUE(map_.load(root + "/src/config/link_set.json",
                          root + "/src/config/node_set.json"));
    const std::vector<std::string> route_ids =
        loadRouteLinks(root + "/src/config/route_links_1.yaml");
    ASSERT_FALSE(route_ids.empty());
    ASSERT_TRUE(route_manager_.initialize(map_, route_ids));

    const auto& reference = route_manager_.referenceLine();
    ASSERT_GT(reference.size(), 30U);
    ASSERT_TRUE(route_manager_.projectEgo(
        reference[20].x, reference[20].y, &ego_s_, &ego_d_, &ego_link_));
    ASSERT_TRUE(route_manager_.getRoutePose(
        ego_s_, &ego_pose_, &ego_yaw_));
  }

  TrackedObject ObjectAt(double forward_m, double d, double width) const {
    Point2D pose;
    double yaw = 0.0;
    EXPECT_TRUE(route_manager_.getRoutePose(ego_s_ + forward_m,
                                            &pose, &yaw));
    TrackedObject object;
    object.id = 71;
    object.type = ObjectType::UNKNOWN;
    object.x = pose.x - std::sin(yaw) * d;
    object.y = pose.y + std::cos(yaw) * d;
    object.yaw = yaw;
    object.length = 2.0;
    object.width = width;
    return object;
  }

  UrbanFeature Build(const TrackedObject& object) {
    EgoState ego;
    ego.x = ego_pose_.x;
    ego.y = ego_pose_.y;
    ego.yaw = ego_yaw_;
    ego.speed = 5.0;
    ego.stamp = 1.0;
    ego.valid = true;
    PerceptionObjects objects;
    objects.obstacles.push_back(object);
    return builder_.build(ego, objects, route_manager_, traffic_light_);
  }

  UrbanFeature BuildObjects(const PerceptionObjects& objects,
                            double stamp = 1.0) {
    EgoState ego;
    ego.x = ego_pose_.x;
    ego.y = ego_pose_.y;
    ego.yaw = ego_yaw_;
    ego.speed = 5.0;
    ego.stamp = stamp;
    ego.valid = true;
    return builder_.build(ego, objects, route_manager_, traffic_light_);
  }

  UrbanFeature BuildEmpty() {
    EgoState ego;
    ego.x = ego_pose_.x;
    ego.y = ego_pose_.y;
    ego.yaw = ego_yaw_;
    ego.speed = 5.0;
    ego.stamp = 2.0;
    ego.valid = true;
    const PerceptionObjects empty;
    return builder_.build(ego, empty, route_manager_, traffic_light_);
  }

  MGeoLoader map_;
  RouteManager route_manager_;
  FeatureBuilder builder_;
  GreenTrafficLight traffic_light_;
  Point2D ego_pose_;
  std::string ego_link_;
  double ego_s_ = 0.0;
  double ego_d_ = 0.0;
  double ego_yaw_ = 0.0;
};

TEST_F(FeatureBuilderAvoidTest, UnknownOutsideRouteCorridorDoesNotSelectAvoid) {
  AvoidTargetConfig config;
  config.ego_width = 1.892;
  config.path_margin_m = 0.40;
  config.max_lateral_offset = 1.75;
  builder_.setAvoidTargetConfig(config);

  const UrbanFeature feature = Build(ObjectAt(20.0, 1.8, 0.5));
  EXPECT_FALSE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(feature.risk.avoidance_object_id, -1);
}

TEST_F(FeatureBuilderAvoidTest, TinyMarginTouchDoesNotSelectAvoid) {
  AvoidTargetConfig config;
  config.ego_width = 1.892;
  config.path_margin_m = 0.40;
  config.max_lateral_offset = 1.75;
  builder_.setAvoidTargetConfig(config);

  const UrbanFeature feature = Build(ObjectAt(20.0, 1.8, 1.2));
  EXPECT_FALSE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(feature.risk.avoidance_object_id, -1);
}

TEST_F(FeatureBuilderAvoidTest, CenterInsidePhysicalCorridorSelectsImmediately) {
  const UrbanFeature feature = Build(ObjectAt(20.0, 0.0, 0.15));
  EXPECT_TRUE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(feature.risk.avoidance_object_id, 71);
}

TEST_F(FeatureBuilderAvoidTest, SmallPhysicalBoundaryTouchDoesNotSelectAvoid) {
  AvoidTargetConfig config;
  config.ego_width = 1.892;
  config.min_physical_overlap_m = 0.10;
  builder_.setAvoidTargetConfig(config);

  // OBB interval [0.80, 2.00] overlaps the physical half-width 0.946 m
  // by 0.146 m even though its center lies outside the body corridor.
  const UrbanFeature first = Build(ObjectAt(20.0, 1.4, 1.2));
  const UrbanFeature second = Build(ObjectAt(20.0, 1.4, 1.2));
  EXPECT_FALSE(first.risk.static_obstacle_ahead);
  EXPECT_FALSE(second.risk.static_obstacle_ahead);
}

TEST_F(FeatureBuilderAvoidTest, MeaningfulPhysicalOverlapNeedsConfirmation) {
  AvoidTargetConfig config;
  config.overlap_confirm_cycles = 2;
  builder_.setAvoidTargetConfig(config);

  // OBB interval [0.35, 1.55] blocks 0.596 m, about 50% of its projected
  // width: below immediate threshold but above the confirmed entry threshold.
  const TrackedObject object = ObjectAt(20.0, 0.95, 1.2);
  EXPECT_FALSE(Build(object).risk.static_obstacle_ahead);
  const UrbanFeature second = Build(object);
  EXPECT_TRUE(second.risk.static_obstacle_ahead);
  EXPECT_EQ(second.risk.avoidance_object_id, 71);
}

TEST_F(FeatureBuilderAvoidTest, LatchedOverlapUsesSeparateReleaseThreshold) {
  AvoidTargetConfig config;
  config.overlap_confirm_cycles = 2;
  config.overlap_release_cycles = 3;
  builder_.setAvoidTargetConfig(config);

  const TrackedObject entry = ObjectAt(20.0, 0.95, 1.2);
  EXPECT_FALSE(Build(entry).risk.static_obstacle_ahead);
  EXPECT_TRUE(Build(entry).risk.static_obstacle_ahead);

  // About 12% physical blocking: below the 15% exit threshold. The latched
  // target survives two noisy frames and releases on the third.
  const TrackedObject weak = ObjectAt(20.0, 1.4, 1.2);
  EXPECT_TRUE(Build(weak).risk.static_obstacle_ahead);
  EXPECT_TRUE(Build(weak).risk.static_obstacle_ahead);
  EXPECT_FALSE(Build(weak).risk.static_obstacle_ahead);
}

TEST_F(FeatureBuilderAvoidTest, MarginOnlyOverlapNeedsConsecutiveConfirmation) {
  AvoidTargetConfig config;
  config.ego_width = 1.892;
  config.path_margin_m = 0.40;
  config.min_physical_overlap_m = 0.10;
  config.min_margin_overlap_m = 0.20;
  config.min_overlap_ratio = 0.20;
  config.overlap_confirm_cycles = 2;
  builder_.setAvoidTargetConfig(config);

  // OBB interval [0.95, 2.15] does not physically overlap the 0.946 m
  // half-width, but overlaps 0.396 m of the margin corridor (33%).
  const TrackedObject object = ObjectAt(20.0, 1.55, 1.2);
  const UrbanFeature first = Build(object);
  EXPECT_FALSE(first.risk.static_obstacle_ahead);
  const UrbanFeature second = Build(object);
  EXPECT_TRUE(second.risk.static_obstacle_ahead);
  EXPECT_EQ(second.risk.avoidance_object_id, 71);
}

TEST_F(FeatureBuilderAvoidTest, MarginConfirmationSurvivesShortMissingFrame) {
  AvoidTargetConfig config;
  config.overlap_confirm_cycles = 2;
  builder_.setAvoidTargetConfig(config);

  const TrackedObject object = ObjectAt(20.0, 1.55, 1.2);
  EXPECT_FALSE(Build(object).risk.static_obstacle_ahead);
  EXPECT_FALSE(BuildEmpty().risk.static_obstacle_ahead);
  EXPECT_TRUE(Build(object).risk.static_obstacle_ahead);
}

TEST_F(FeatureBuilderAvoidTest, ChangedIdInheritsUniqueSpatialConfirmation) {
  AvoidTargetConfig config;
  config.overlap_confirm_cycles = 2;
  builder_.setAvoidTargetConfig(config);

  TrackedObject object = ObjectAt(20.0, 1.55, 1.2);
  EXPECT_FALSE(Build(object).risk.static_obstacle_ahead);

  object.id = 72;
  const UrbanFeature reidentified = Build(object);
  EXPECT_TRUE(reidentified.risk.static_obstacle_ahead);
  EXPECT_EQ(reidentified.risk.avoidance_object_id, 72);
}

TEST_F(FeatureBuilderAvoidTest, LatchedTargetSurvivesShortMissingFrames) {
  AvoidTargetConfig config;
  config.reconnect_grace_cycles = 3;
  builder_.setAvoidTargetConfig(config);

  EXPECT_TRUE(Build(ObjectAt(20.0, 0.0, 1.2))
                  .risk.static_obstacle_ahead);
  for (int i = 0; i < config.reconnect_grace_cycles; ++i) {
    const UrbanFeature missing = BuildEmpty();
    EXPECT_TRUE(missing.risk.static_obstacle_ahead);
    EXPECT_EQ(missing.risk.avoidance_object_id, 71);
  }
  EXPECT_FALSE(BuildEmpty().risk.static_obstacle_ahead);
}

TEST_F(FeatureBuilderAvoidTest, AmbiguousReidentificationKeepsOldIdentity) {
  AvoidTargetConfig config;
  config.overlap_confirm_cycles = 2;
  builder_.setAvoidTargetConfig(config);

  const TrackedObject original = ObjectAt(20.0, 1.55, 1.2);
  EXPECT_FALSE(Build(original).risk.static_obstacle_ahead);
  EXPECT_TRUE(Build(original).risk.static_obstacle_ahead);

  TrackedObject first_fragment = original;
  first_fragment.id = 72;
  TrackedObject second_fragment = original;
  second_fragment.id = 73;
  second_fragment.x += 0.15 * std::cos(ego_yaw_);
  second_fragment.y += 0.15 * std::sin(ego_yaw_);

  EgoState ego;
  ego.x = ego_pose_.x;
  ego.y = ego_pose_.y;
  ego.yaw = ego_yaw_;
  ego.speed = 5.0;
  ego.stamp = 3.0;
  ego.valid = true;
  PerceptionObjects objects;
  objects.obstacles = {first_fragment, second_fragment};
  const UrbanFeature ambiguous =
      builder_.build(ego, objects, route_manager_, traffic_light_);
  EXPECT_TRUE(ambiguous.risk.static_obstacle_ahead);
  EXPECT_EQ(ambiguous.risk.avoidance_object_id, 71);
}

TEST_F(FeatureBuilderAvoidTest, DistantChangedIdDoesNotInheritConfirmation) {
  AvoidTargetConfig config;
  config.overlap_confirm_cycles = 2;
  config.reconnect_max_longitudinal_delta_m = 3.0;
  builder_.setAvoidTargetConfig(config);

  EXPECT_FALSE(Build(ObjectAt(20.0, 1.55, 1.2))
                   .risk.static_obstacle_ahead);
  TrackedObject distant = ObjectAt(24.0, 1.55, 1.2);
  distant.id = 72;
  const UrbanFeature changed = Build(distant);
  EXPECT_FALSE(changed.risk.static_obstacle_ahead);
  EXPECT_EQ(changed.risk.avoidance_object_id, -1);
}

TEST_F(FeatureBuilderAvoidTest, PendingUnknownBecomesLongitudinalLead) {
  TrackedObject object = ObjectAt(20.0, 0.0, 1.2);
  object.motion_state = MotionState::DYNAMIC_PENDING;
  object.vx = 3.0 * std::cos(object.yaw);
  object.vy = 3.0 * std::sin(object.yaw);
  object.speed = 3.0;

  const UrbanFeature feature = Build(object);
  EXPECT_FALSE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(feature.risk.avoidance_object_id, -1);
  EXPECT_TRUE(feature.surrounding.has_front_vehicle);
  EXPECT_EQ(feature.surrounding.front_vehicle.id, 71);
}

TEST_F(FeatureBuilderAvoidTest, RearUnknownDoesNotSelectAvoid) {
  const UrbanFeature feature = Build(ObjectAt(-1.0, 0.0, 1.2));
  EXPECT_FALSE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(feature.risk.avoidance_object_id, -1);
}

TEST_F(FeatureBuilderAvoidTest, StaticLatchClearsWhenSameIdBecomesMoving) {
  TrackedObject object = ObjectAt(20.0, 0.0, 1.2);
  ASSERT_TRUE(Build(object).risk.static_obstacle_ahead);

  object.motion_state = MotionState::DYNAMIC_PENDING;
  object.vx = 3.0 * std::cos(object.yaw);
  object.vy = 3.0 * std::sin(object.yaw);
  object.speed = 3.0;
  const UrbanFeature moving = Build(object);
  EXPECT_FALSE(moving.risk.static_obstacle_ahead);
  EXPECT_TRUE(moving.risk.avoid_target_reclassified_moving);
  EXPECT_TRUE(moving.surrounding.has_front_vehicle);
}

TEST_F(FeatureBuilderAvoidTest, StaticLatchClearsAcrossMovingReidentification) {
  TrackedObject object = ObjectAt(20.0, 0.0, 1.2);
  ASSERT_TRUE(Build(object).risk.static_obstacle_ahead);

  object.id = 72;
  object.motion_state = MotionState::DYNAMIC;
  object.vx = 3.0 * std::cos(object.yaw);
  object.vy = 3.0 * std::sin(object.yaw);
  object.speed = 3.0;
  const UrbanFeature moving = Build(object);
  EXPECT_FALSE(moving.risk.static_obstacle_ahead);
  EXPECT_TRUE(moving.risk.avoid_target_reclassified_moving);
  EXPECT_TRUE(moving.surrounding.has_front_vehicle);
  EXPECT_EQ(moving.surrounding.front_vehicle.id, 72);
}

TEST_F(FeatureBuilderAvoidTest, ConfirmedDynamicUnknownIsNotStaticAvoidTarget) {
  TrackedObject object = ObjectAt(20.0, 0.0, 1.2);
  object.motion_state = MotionState::DYNAMIC;
  object.vx = 3.0 * std::cos(object.yaw);
  object.vy = 3.0 * std::sin(object.yaw);
  object.speed = 3.0;

  const UrbanFeature feature = Build(object);
  EXPECT_FALSE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(feature.risk.avoidance_object_id, -1);
  EXPECT_TRUE(feature.surrounding.has_front_vehicle);
  EXPECT_EQ(feature.surrounding.front_vehicle.id, 71);
}

TEST_F(FeatureBuilderAvoidTest, StaticBlipMatchesRecentLeadWithSameId) {
  TrackedObject object = ObjectAt(20.0, 0.0, 1.2);
  object.motion_state = MotionState::DYNAMIC;
  object.vx = 3.0 * std::cos(object.yaw);
  object.vy = 3.0 * std::sin(object.yaw);
  object.speed = 3.0;
  ASSERT_TRUE(Build(object).surrounding.has_front_vehicle);

  object.motion_state = MotionState::LOW_THREAT;
  object.vx = 0.0;
  object.vy = 0.0;
  object.speed = 0.0;
  const UrbanFeature static_blip = Build(object);
  ASSERT_TRUE(static_blip.risk.static_obstacle_ahead);
  EXPECT_TRUE(static_blip.risk.avoid_target_matches_recent_lead);
}

TEST_F(FeatureBuilderAvoidTest, StaticBlipMatchesRecentLeadAcrossChangedId) {
  TrackedObject object = ObjectAt(20.0, 0.0, 1.2);
  object.motion_state = MotionState::DYNAMIC_PENDING;
  object.vx = 3.0 * std::cos(object.yaw);
  object.vy = 3.0 * std::sin(object.yaw);
  object.speed = 3.0;
  ASSERT_TRUE(Build(object).surrounding.has_front_vehicle);

  object.id = 72;
  object.motion_state = MotionState::LOW_THREAT;
  object.vx = 0.0;
  object.vy = 0.0;
  object.speed = 0.0;
  const UrbanFeature reidentified_static = Build(object);
  ASSERT_TRUE(reidentified_static.risk.static_obstacle_ahead);
  EXPECT_EQ(72, reidentified_static.risk.avoidance_object_id);
  EXPECT_TRUE(reidentified_static.risk.avoid_target_matches_recent_lead);
}

TEST_F(FeatureBuilderAvoidTest,
       SmallOverlappingUnknownIsOwnedByStableCurrentLead) {
  TrackedObject lead = ObjectAt(20.0, 0.0, 2.0);
  lead.id = 80;
  lead.type = ObjectType::VEHICLE;
  lead.motion_state = MotionState::DYNAMIC;
  lead.length = 4.5;
  lead.vx = 3.0 * std::cos(lead.yaw);
  lead.vy = 3.0 * std::sin(lead.yaw);
  lead.speed = 3.0;

  PerceptionObjects first;
  first.vehicles.push_back(lead);
  ASSERT_TRUE(BuildObjects(first).surrounding.has_front_vehicle);

  TrackedObject fragment = ObjectAt(20.8, 0.15, 0.6);
  fragment.id = 81;
  fragment.length = 1.0;
  fragment.motion_state = MotionState::LOW_THREAT;
  PerceptionObjects split;
  split.vehicles.push_back(lead);
  split.obstacles.push_back(fragment);

  const UrbanFeature feature = BuildObjects(split, 2.0);
  ASSERT_TRUE(feature.surrounding.has_front_vehicle);
  EXPECT_EQ(80, feature.surrounding.front_vehicle.id);
  EXPECT_FALSE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(-1, feature.risk.avoidance_object_id);
  EXPECT_DOUBLE_EQ(1.0, feature.extra_values.at("follow_fragment_count"));
}

TEST_F(FeatureBuilderAvoidTest,
       SeparateSmallObstacleNearLeadRemainsAvoidTarget) {
  TrackedObject lead = ObjectAt(20.0, 0.0, 2.0);
  lead.id = 80;
  lead.type = ObjectType::VEHICLE;
  lead.motion_state = MotionState::DYNAMIC;
  lead.length = 4.5;
  lead.vx = 3.0 * std::cos(lead.yaw);
  lead.vy = 3.0 * std::sin(lead.yaw);
  lead.speed = 3.0;

  PerceptionObjects first;
  first.vehicles.push_back(lead);
  ASSERT_TRUE(BuildObjects(first).surrounding.has_front_vehicle);

  TrackedObject obstacle = ObjectAt(24.0, 0.0, 0.6);
  obstacle.id = 82;
  obstacle.length = 1.0;
  obstacle.motion_state = MotionState::LOW_THREAT;
  PerceptionObjects separated;
  separated.vehicles.push_back(lead);
  separated.obstacles.push_back(obstacle);

  const UrbanFeature feature = BuildObjects(separated, 2.0);
  EXPECT_TRUE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(82, feature.risk.avoidance_object_id);
  EXPECT_DOUBLE_EQ(0.0, feature.extra_values.at("follow_fragment_count"));
}

}  // namespace
}  // namespace behavior_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
