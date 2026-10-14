#include <gtest/gtest.h>
#include "planning_inputs/converter.hpp"

namespace {
asmc_msgs::TrackedObjectArray Frame(const std::string& group,
                                   const std::string& raw) {
  asmc_msgs::TrackedObjectArray frame;
  frame.header.frame_id = "map";
  frame.header.stamp = ros::Time(1, 0);
  asmc_msgs::TrackedObject object;
  object.id = 731;
  object.age = 1;
  object.classification = group;
  object.yolo_class = raw;
  object.length = 4.0;
  object.width = 2.0;
  object.height = 1.0;
  object.obb_center.x = object.cluster_center.x = 100.0;
  object.obb_center.y = object.cluster_center.y = 100.0;
  object.obb_center.z = object.cluster_center.z = 0.5;
  object.velocity_mps.x = 2.5;
  object.speed_kph = 9.0;
  for (unsigned i = 0; i < 8; ++i) {
    object.obb_corners[i].x = i & 1 ? 102.0 : 98.0;
    object.obb_corners[i].y = i & 2 ? 101.0 : 99.0;
    object.obb_corners[i].z = i & 4 ? 1.0 : 0.0;
  }
  frame.objects.push_back(object);
  return frame;
}

TEST(MissionGroupConversion, PreservesVehicleAndPersonNumericConsumers) {
  for (const std::string raw : {"car", "bus", "truck", "person"}) {
    const bool person = raw == "person";
    auto input = Frame(person ? "dynamic_obstacle" : "vehicle", raw);
    planning_inputs::DetectedObjectArray output;
    planning_inputs::ConversionStats stats;
    ASSERT_EQ(planning_inputs::ConversionStatus::OK,
              planning_inputs::ConvertTrackedObjects(input, &output, &stats));
    ASSERT_EQ(1u, output.objects.size());
    EXPECT_EQ(person ? planning_inputs::kObjectPedestrian : planning_inputs::kObjectVehicle,
              output.objects[0].type);
    EXPECT_EQ(0u, stats.unsupported_classification);
    EXPECT_DOUBLE_EQ(2.5, output.objects[0].vx);
    EXPECT_EQ(input.header, output.header);
  }
}

TEST(MissionGroupConversion, ObstacleGroupDoesNotInventMotionOrDropGeometry) {
  for (const std::string group : {"static_obstacle", "dynamic_obstacle"}) {
    auto input = Frame(group, group == "static_obstacle" ? "motorcycle" : "bicycle");
    planning_inputs::DetectedObjectArray output;
    planning_inputs::ConversionStats stats;
    ASSERT_EQ(planning_inputs::ConversionStatus::OK,
              planning_inputs::ConvertTrackedObjects(input, &output, &stats));
    ASSERT_EQ(1u, output.objects.size());
    EXPECT_EQ(planning_inputs::kObjectUnknown, output.objects[0].type);
    EXPECT_EQ(planning_inputs::kMotionUnknown, output.objects[0].motion_state);
    EXPECT_DOUBLE_EQ(2.5, output.objects[0].vx);
    EXPECT_EQ(input.objects[0].obb_corners, output.objects[0].obb_corners);
    EXPECT_EQ(0u, stats.unsupported_classification);
  }
}
}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
