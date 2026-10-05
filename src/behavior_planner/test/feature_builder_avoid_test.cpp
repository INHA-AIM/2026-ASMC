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
  config.ego_width = 1.85;
  config.path_margin_m = 0.40;
  config.max_lateral_offset = 1.75;
  builder_.setAvoidTargetConfig(config);

  const UrbanFeature feature = Build(ObjectAt(20.0, 1.8, 0.5));
  EXPECT_FALSE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(feature.risk.avoidance_object_id, -1);
}

TEST_F(FeatureBuilderAvoidTest, UnknownObbOverlappingRouteCorridorSelectsAvoid) {
  AvoidTargetConfig config;
  config.ego_width = 1.85;
  config.path_margin_m = 0.40;
  config.max_lateral_offset = 1.75;
  builder_.setAvoidTargetConfig(config);

  const UrbanFeature feature = Build(ObjectAt(20.0, 1.8, 1.2));
  EXPECT_TRUE(feature.risk.static_obstacle_ahead);
  EXPECT_EQ(feature.risk.avoidance_object_id, 71);
}

}  // namespace
}  // namespace behavior_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
