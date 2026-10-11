#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "Output/Behavior/utils.hpp"
#include "Behavior/Map/mgeo_loader.hpp"
#include "Behavior/Map/route_manager.hpp"

namespace behavior_planner {
namespace {

class RouteSignalMovementTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string root = BEHAVIOR_PLANNER_SOURCE_DIR;
    ASSERT_TRUE(map_.load(root + "/config/behavior/link_set.json",
                          root + "/config/behavior/node_set.json"));
    const std::vector<std::string> route =
        loadRouteLinks(root + "/config/behavior/route_links_1.yaml");
    ASSERT_FALSE(route.empty());
    ASSERT_TRUE(route_.initialize(map_, route));
  }

  double RouteSAtLinkStart(const std::string& link_id) {
    LinkInfo link;
    EXPECT_TRUE(map_.getLink(link_id, &link));
    EXPECT_FALSE(link.centerline.empty());
    double s = 0.0;
    double d = 0.0;
    std::string projected_link;
    EXPECT_TRUE(route_.projectEgo(link.centerline.front().x,
                                  link.centerline.front().y,
                                  &s, &d, &projected_link));
    return s;
  }

  MGeoLoader map_;
  RouteManager route_;
};

TEST_F(RouteSignalMovementTest, FindsRightThenProtectedLeftOnGlobalRoute) {
  RouteSignalMovement movement;
  const double before_right = RouteSAtLinkStart("A2256W000315") + 1.0;
  ASSERT_TRUE(route_.findNextSignalMovement(before_right, 200.0, &movement));
  EXPECT_EQ(SignalMovement::RIGHT_UNPROTECTED, movement.movement);
  EXPECT_EQ("right_unprotected", movement.related_signal);
  EXPECT_EQ("A2256W000318", movement.link_id);

  const double after_right = RouteSAtLinkStart("A2256W000308") + 1.0;
  ASSERT_TRUE(route_.findNextSignalMovement(after_right, 200.0, &movement));
  EXPECT_EQ(SignalMovement::LEFT, movement.movement);
  EXPECT_EQ("left", movement.related_signal);
  EXPECT_EQ("A2256W000083", movement.link_id);
}

TEST_F(RouteSignalMovementTest, FindsStraightMovement) {
  RouteSignalMovement movement;
  const double before_straight = RouteSAtLinkStart("A2256W000728") + 1.0;
  ASSERT_TRUE(route_.findNextSignalMovement(before_straight, 200.0,
                                            &movement));
  EXPECT_EQ(SignalMovement::STRAIGHT, movement.movement);
  EXPECT_EQ("straight", movement.related_signal);
  EXPECT_EQ("A2256W000599", movement.link_id);
}

TEST_F(RouteSignalMovementTest,
       FindsOnlyForwardRouteAssociatedSignalizedStopLine) {
  const double ego_s = RouteSAtLinkStart("A2256W000315") + 1.0;
  StopLineInfo stop;
  double distance = 0.0;
  ASSERT_TRUE(route_.findNextSignalizedStopLine(
      ego_s, 200.0, &stop, &distance));
  EXPECT_TRUE(stop.valid);
  EXPECT_FALSE(stop.traffic_light_id.empty());
  EXPECT_GE(distance, 0.0);
  EXPECT_LE(distance, 200.0);
  EXPECT_NEAR(stop.route_s - ego_s, distance, 1e-9);

  // Once this line is behind the ego, it cannot remain latched as the active
  // approach owner. With a zero forward window no later line is eligible.
  StopLineInfo passed_stop;
  double passed_distance = 0.0;
  EXPECT_FALSE(route_.findNextSignalizedStopLine(
      stop.route_s + 0.01, 0.0, &passed_stop, &passed_distance));

  ASSERT_TRUE(route_.findPreviousSignalizedStopLine(
      stop.route_s + 0.01, &passed_stop, &passed_distance));
  EXPECT_EQ(stop.traffic_light_id, passed_stop.traffic_light_id);
  EXPECT_NEAR(0.01, passed_distance, 1e-9);
}

}  // namespace
}  // namespace behavior_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
