#include "Parser/DetectedObjects.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace behavior_planner {
namespace {

planning_inputs::DetectedObject MakeObject(int id, std::uint8_t type) {
  planning_inputs::DetectedObject object;
  object.id = id;
  object.type = type;
  object.motion_state = planning_inputs::DetectedObject::MOTION_DYNAMIC_PENDING;
  object.center.x = 10.0 + id;
  object.center.y = -2.0;
  object.yaw = 0.25;
  object.vx = 3.0;
  object.vy = 4.0;
  object.length = 0.8;
  object.width = 0.6;
  return object;
}

TEST(DetectedMsgConverter, PreservesUnknownForGeometryBasedAvoidance) {
  planning_inputs::DetectedObjectArray message;
  message.objects.push_back(MakeObject(1, 1));
  message.objects.push_back(MakeObject(2, 2));
  message.objects.push_back(MakeObject(3, 3));
  message.objects.push_back(MakeObject(4, 0));

  const PerceptionObjects converted =
      DetectedMsgConverter::toPerceptionObjects(message);
  ASSERT_EQ(converted.vehicles.size(), 1u);
  ASSERT_EQ(converted.pedestrians.size(), 1u);
  ASSERT_EQ(converted.obstacles.size(), 2u);
  EXPECT_EQ(converted.vehicles.front().id, 1);
  EXPECT_EQ(converted.pedestrians.front().id, 2);
  EXPECT_EQ(converted.obstacles.front().id, 3);
  EXPECT_EQ(converted.obstacles.back().id, 4);
  EXPECT_EQ(converted.obstacles.back().type, ObjectType::UNKNOWN);
  EXPECT_EQ(converted.obstacles.back().motion_state,
            MotionState::DYNAMIC_PENDING);
  EXPECT_NEAR(converted.pedestrians.front().speed, 5.0, 1e-9);
}

TEST(DetectedMsgConverter, LowThreatPreservesVectorButUsesStaticSpeed) {
  planning_inputs::DetectedObjectArray message;
  auto object = MakeObject(5, 0);
  object.motion_state = planning_inputs::DetectedObject::MOTION_LOW_THREAT;
  message.objects.push_back(object);

  const PerceptionObjects converted =
      DetectedMsgConverter::toPerceptionObjects(message);
  ASSERT_EQ(converted.obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(converted.obstacles.front().vx, 3.0);
  EXPECT_DOUBLE_EQ(converted.obstacles.front().vy, 4.0);
  EXPECT_DOUBLE_EQ(converted.obstacles.front().speed, 0.0);
}

TEST(DetectedMsgConverter, UnavailableVelocityRemainsExplicitUnknownOccupancy) {
  planning_inputs::DetectedObjectArray message;
  auto object = MakeObject(8, 1);
  object.motion_state = object.MOTION_UNKNOWN;
  object.velocity_unavailable = true;
  object.vx = object.vy = 0.;
  message.objects.push_back(object);
  const auto converted = DetectedMsgConverter::toPerceptionObjects(message);
  ASSERT_EQ(converted.vehicles.size(), 1u);
  EXPECT_TRUE(converted.vehicles[0].velocity_unavailable);
  EXPECT_EQ(converted.vehicles[0].motion_state, MotionState::UNKNOWN);
  message.objects[0].motion_state = object.MOTION_LOW_THREAT;
  EXPECT_TRUE(DetectedMsgConverter::toPerceptionObjects(message).vehicles.empty());
}

TEST(DetectedMsgConverter, RejectsInvalidNumericObjects) {
  planning_inputs::DetectedObjectArray message;
  auto invalid = MakeObject(2, 2);
  invalid.center.x = std::numeric_limits<double>::quiet_NaN();
  message.objects.push_back(invalid);

  const PerceptionObjects converted =
      DetectedMsgConverter::toPerceptionObjects(message);
  EXPECT_TRUE(converted.pedestrians.empty());
}

}  // namespace
}  // namespace behavior_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
