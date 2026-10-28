#include <gtest/gtest.h>
#include "planning_inputs/converter.hpp"

namespace {
asmc_msgs::LidarTrackingObservation Frame(const std::string& group,
                                   const std::string& raw) {
  asmc_msgs::LidarTrackingObservation frame;
  frame.observation.objects.header.frame_id = "map";
  frame.observation.objects.header.stamp = ros::Time(1, 0);
  asmc_msgs::LidarObject object;
  object.track_id = 731;
  object.age = 1;
  object.classification = group;
  object.camera_class = raw;
  object.length = 4.0;
  object.width = 2.0;
  object.height = 1.0;
  object.center.x = 100.0;
  object.center.y = 100.0;
  object.center.z = 0.5;
  object.absolute_velocity_mps.x = 2.5;
  object.absolute_velocity_valid = true;
  object.observation_state = object.OBSERVED;
  for (unsigned i = 0; i < 8; ++i) {
    object.obb_corners[i].x = i & 1 ? 102.0 : 98.0;
    object.obb_corners[i].y = i & 2 ? 101.0 : 99.0;
    object.obb_corners[i].z = i & 4 ? 1.0 : 0.0;
  }
  auto& observation = frame.observation;
  observation.producer_session_id = "test";
  observation.generation = observation.scan_sequence = 1;
  observation.config_id = "config"; observation.calibration_id = "calibration";
  observation.objects.status = observation.objects.OBSERVED;
  observation.objects.objects.push_back(object);
  asmc_msgs::LidarTrackingState tracking;
  tracking.track_id = object.track_id; tracking.cluster_center = object.center;
  frame.tracking.push_back(tracking);
  return frame;
}

TEST(MissionGroupConversion, PreservesVehicleAndPersonNumericConsumers) {
  for (const std::string raw : {"car", "bus", "truck", "person"}) {
    const bool person = raw == "person";
    auto input = Frame(person ? "dynamic_obstacle" : "vehicle", raw);
    planning_inputs::DetectedObjectArray output;
    planning_inputs::ConversionStats stats;
    ASSERT_EQ(planning_inputs::ConversionStatus::OK,
              planning_inputs::ConvertLidarObservation(input, &output, &stats));
    ASSERT_EQ(1u, output.objects.size());
    EXPECT_EQ(person ? planning_inputs::kObjectPedestrian : planning_inputs::kObjectVehicle,
              output.objects[0].type);
    EXPECT_EQ(0u, stats.unsupported_classification);
    EXPECT_DOUBLE_EQ(2.5, output.objects[0].vx);
    EXPECT_EQ(input.observation.objects.header, output.header);
  }
}

TEST(MissionGroupConversion, ObstacleGroupDoesNotInventMotionOrDropGeometry) {
  for (const std::string group : {"static_obstacle", "dynamic_obstacle"}) {
    auto input = Frame(group, group == "static_obstacle" ? "motorcycle" : "bicycle");
    planning_inputs::DetectedObjectArray output;
    planning_inputs::ConversionStats stats;
    ASSERT_EQ(planning_inputs::ConversionStatus::OK,
              planning_inputs::ConvertLidarObservation(input, &output, &stats));
    ASSERT_EQ(1u, output.objects.size());
    EXPECT_EQ(planning_inputs::kObjectUnknown, output.objects[0].type);
    EXPECT_EQ(planning_inputs::kMotionUnknown, output.objects[0].motion_state);
    EXPECT_DOUBLE_EQ(2.5, output.objects[0].vx);
    EXPECT_EQ(input.observation.objects.objects[0].obb_corners, output.objects[0].obb_corners);
    EXPECT_EQ(0u, stats.unsupported_classification);
  }
}
TEST(MissionGroupConversion, AllMissionLabelsSurviveBothBoundariesWithoutMotionPolicyChange) {
  const std::vector<std::pair<std::string, std::string>> labels = {
    {"car","vehicle"},{"bus","vehicle"},{"truck","vehicle"},
    {"person","dynamic_obstacle"},{"dog","dynamic_obstacle"},{"deer","dynamic_obstacle"},
    {"bicycle","dynamic_obstacle"},{"stroller","dynamic_obstacle"},
    {"motorcycle","static_obstacle"},{"bollard","static_obstacle"},{"box","static_obstacle"},
    {"concrete_barrier","static_obstacle"},{"steel_barricade","static_obstacle"},
    {"traffic_drum","static_obstacle"},{"plastic_barrier","static_obstacle"},{"traffic_cone","static_obstacle"}};
  for (const auto& label : labels) {
    auto input = Frame(label.second, label.first);
    planning_inputs::DetectedObjectArray out;
    ASSERT_EQ(planning_inputs::ConversionStatus::OK,
      planning_inputs::ConvertLidarObservation(input, &out));
    EXPECT_EQ(out.producer_session_id, input.observation.producer_session_id);
    EXPECT_EQ(out.generation, input.observation.generation);
    EXPECT_EQ(out.scan_sequence, input.observation.scan_sequence);
    EXPECT_EQ(out.config_id, input.observation.config_id);
    EXPECT_EQ(out.calibration_id, input.observation.calibration_id);
    EXPECT_EQ(out.objects[0].classification, label.second);
    EXPECT_EQ(out.objects[0].camera_class, label.first);
    EXPECT_DOUBLE_EQ(out.objects[0].vx, 2.5);
    EXPECT_EQ(out.objects[0].motion_state, planning_inputs::kMotionUnknown);
    EXPECT_EQ(out.objects[0].type, label.second == "vehicle" ? 1 : label.first == "person" ? 2 : 0);
    ASSERT_EQ(planning_inputs::ConversionStatus::OK,
      planning_inputs::ConvertPlanningGeometry(input.observation.objects, &out));
    EXPECT_EQ(out.objects[0].classification, label.second);
    EXPECT_EQ(out.objects[0].camera_class, label.first);
  }
}
}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
