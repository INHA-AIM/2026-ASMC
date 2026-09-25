#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include <asmc_msgs/LaneBoundaryObservation.h>
#include <asmc_msgs/LaneBoundaryObservationArray.h>
#include <geometry_msgs/Point32.h>

#include "lane_boundary_adapter/lane_boundary_adapter.hpp"

namespace lane_boundary_adapter {
namespace {

asmc_msgs::LaneBoundaryObservation MakeBoundary(
    std::uint32_t id, std::uint16_t order, std::uint8_t color,
    double reference_y, double slope = 0.0) {
  asmc_msgs::LaneBoundaryObservation boundary;
  boundary.tracking_id = id;
  boundary.lateral_order = order;
  boundary.order_valid = true;
  boundary.color = color;
  boundary.source_mask =
      asmc_msgs::LaneBoundaryObservation::SOURCE_FRONT |
      asmc_msgs::LaneBoundaryObservation::SOURCE_LEFT |
      asmc_msgs::LaneBoundaryObservation::SOURCE_RIGHT;
  boundary.semantic_score = 0.90f;
  boundary.geometric_quality = 0.85f;
  boundary.reference_x_m = 5.0f;
  boundary.reference_y_m = static_cast<float>(reference_y);
  boundary.valid_x_min_m = 0.0f;
  boundary.valid_x_max_m = 15.0f;
  boundary.observed_length_m = 15.0f;
  boundary.bridged_length_ratio = 0.05f;
  boundary.age_frames = 10;
  for (int index = 0; index <= 15; ++index) {
    geometry_msgs::Point32 point;
    point.x = static_cast<float>(index);
    point.y = static_cast<float>(reference_y + slope * (index - 5.0));
    point.z = -0.35f;
    boundary.polyline.push_back(point);
  }
  return boundary;
}

asmc_msgs::LaneBoundaryObservationArray MakeCompleteObservation() {
  asmc_msgs::LaneBoundaryObservationArray message;
  message.header.frame_id = "base_link";
  message.valid = true;
  message.completeness =
      asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_COMPLETE;
  message.expected_boundary_count = 3;
  message.operating_domain = "shadow_two_way_v1";
  message.algorithm_id = "twinlite_v6_shadow_boundary_contract_v1";
  message.config_sha256 = "test-config";
  message.boundaries = {
      MakeBoundary(11, 0, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE,
                   5.325),
      MakeBoundary(12, 1, asmc_msgs::LaneBoundaryObservation::COLOR_YELLOW,
                   1.775),
      MakeBoundary(13, 2, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE,
                   -1.775),
  };
  return message;
}

TEST(LaneBoundaryAdapterTest, BuildsTwoOrderedStripsWithoutSemanticRoles) {
  const auto context =
      BuildLaneStripContext(MakeCompleteObservation(), AdapterConfig{});
  ASSERT_TRUE(context.usable()) << context.reason;
  ASSERT_EQ(context.strips.size(), 2u);
  EXPECT_EQ(context.occupied_strip_index, 1);
  EXPECT_EQ(context.strips[0].left_boundary_id, 11u);
  EXPECT_EQ(context.strips[0].right_boundary_id, 12u);
  EXPECT_EQ(context.strips[1].left_boundary_id, 12u);
  EXPECT_EQ(context.strips[1].right_boundary_id, 13u);
  EXPECT_NEAR(context.strips[0].width_median_m, 3.55, 1e-4);
  EXPECT_NEAR(context.strips[1].width_median_m, 3.55, 1e-4);
  EXPECT_FALSE(context.strips[0].contains_vehicle_projection);
  EXPECT_TRUE(context.strips[1].contains_vehicle_projection);
}

TEST(LaneBoundaryAdapterTest, VehicleProjectionMayLieInLeftStrip) {
  auto message = MakeCompleteObservation();
  message.boundaries[0] = MakeBoundary(
      11, 0, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE, 1.775);
  message.boundaries[1] = MakeBoundary(
      12, 1, asmc_msgs::LaneBoundaryObservation::COLOR_YELLOW, -1.775);
  message.boundaries[2] = MakeBoundary(
      13, 2, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE, -5.325);
  const auto context = BuildLaneStripContext(message, AdapterConfig{});
  ASSERT_TRUE(context.usable()) << context.reason;
  EXPECT_EQ(context.occupied_strip_index, 0);
}

TEST(LaneBoundaryAdapterTest, AcceptsAConsistentPartialObservation) {
  auto message = MakeCompleteObservation();
  message.completeness =
      asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_PARTIAL;
  message.boundaries.erase(message.boundaries.begin());
  message.boundaries[0].lateral_order = 0;
  message.boundaries[1].lateral_order = 1;
  const auto context = BuildLaneStripContext(message, AdapterConfig{});
  ASSERT_TRUE(context.usable()) << context.reason;
  ASSERT_EQ(context.strips.size(), 1u);
  EXPECT_EQ(context.strips[0].left_boundary_id, 12u);
  EXPECT_EQ(context.strips[0].right_boundary_id, 13u);
}

TEST(LaneBoundaryAdapterTest, AcceptsDynamicSharedOrderReference) {
  auto message = MakeCompleteObservation();
  for (auto& boundary : message.boundaries) {
    boundary.reference_x_m = 2.0f;
  }
  const auto context = BuildLaneStripContext(message, AdapterConfig{});
  ASSERT_TRUE(context.usable()) << context.reason;
  EXPECT_EQ(context.strips.size(), 2u);
}

TEST(LaneBoundaryAdapterTest, RejectsDifferentOrderReferencesInOneFrame) {
  auto message = MakeCompleteObservation();
  message.boundaries[2].reference_x_m = 2.0f;
  const auto context = BuildLaneStripContext(message, AdapterConfig{});
  EXPECT_EQ(context.status, ContextStatus::kBoundaryInvalid);
}

TEST(LaneBoundaryAdapterTest, RejectsPairsWithoutYellowBoundaryInThisOdd) {
  auto message = MakeCompleteObservation();
  message.boundaries[1].color =
      asmc_msgs::LaneBoundaryObservation::COLOR_WHITE;
  AdapterConfig config;
  config.allow_outer_white_pair_fallback = false;
  const auto context = BuildLaneStripContext(message, config);
  EXPECT_EQ(context.status, ContextStatus::kNoUsableStrip);
  EXPECT_TRUE(context.strips.empty());
  EXPECT_EQ(context.rejected_pair_count, 2u);
}

TEST(LaneBoundaryAdapterTest, InfersDegradedCenterFromTwoWhiteOuterBoundaries) {
  auto message = MakeCompleteObservation();
  message.completeness =
      asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_PARTIAL;
  message.boundaries.erase(message.boundaries.begin() + 1);
  message.boundaries[1].lateral_order = 1;
  const auto context = BuildLaneStripContext(message, AdapterConfig{});
  ASSERT_TRUE(context.usable()) << context.reason;
  EXPECT_EQ(context.status, ContextStatus::kDegradedInferredCenter);
  ASSERT_EQ(context.strips.size(), 2u);
  EXPECT_TRUE(context.strips[0].inferred_center_boundary);
  EXPECT_TRUE(context.strips[1].inferred_center_boundary);
  EXPECT_EQ(context.strips[0].right_boundary_id, 0u);
  EXPECT_EQ(context.strips[1].left_boundary_id, 0u);
  EXPECT_NEAR(context.strips[0].width_median_m, 3.55, 1e-4);
  EXPECT_NEAR(context.strips[1].width_median_m, 3.55, 1e-4);
}

TEST(LaneBoundaryAdapterTest, RejectsWhitePairThatCannotDescribeTwoLanes) {
  auto message = MakeCompleteObservation();
  message.completeness =
      asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_PARTIAL;
  message.boundaries = {
      MakeBoundary(11, 0, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE,
                   1.75),
      MakeBoundary(13, 1, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE,
                   -1.75),
  };
  const auto context = BuildLaneStripContext(message, AdapterConfig{});
  EXPECT_EQ(context.status, ContextStatus::kNoUsableStrip);
  EXPECT_TRUE(context.strips.empty());
}

TEST(LaneBoundaryAdapterTest, RejectsInvalidOrdering) {
  auto message = MakeCompleteObservation();
  message.boundaries[1].order_valid = false;
  auto context = BuildLaneStripContext(message, AdapterConfig{});
  EXPECT_EQ(context.status, ContextStatus::kOrderInvalid);

  message = MakeCompleteObservation();
  message.boundaries[2].lateral_order = 1;
  context = BuildLaneStripContext(message, AdapterConfig{});
  EXPECT_EQ(context.status, ContextStatus::kBoundaryInvalid);
}

TEST(LaneBoundaryAdapterTest, RejectsWrongContractIdentifiers) {
  auto message = MakeCompleteObservation();
  message.header.frame_id = "map";
  EXPECT_EQ(BuildLaneStripContext(message, AdapterConfig{}).status,
            ContextStatus::kWrongFrame);

  message = MakeCompleteObservation();
  message.operating_domain = "general_road";
  EXPECT_EQ(BuildLaneStripContext(message, AdapterConfig{}).status,
            ContextStatus::kWrongOperatingDomain);

  message = MakeCompleteObservation();
  message.algorithm_id = "different-model";
  EXPECT_EQ(BuildLaneStripContext(message, AdapterConfig{}).status,
            ContextStatus::kWrongAlgorithm);
}

TEST(LaneBoundaryAdapterTest, DistinguishesValidEmptyFromInvalidMetadata) {
  auto message = MakeCompleteObservation();
  message.boundaries.clear();
  message.completeness =
      asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_EMPTY;
  EXPECT_EQ(BuildLaneStripContext(message, AdapterConfig{}).status,
            ContextStatus::kEmpty);

  message.completeness =
      asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_COMPLETE;
  EXPECT_EQ(BuildLaneStripContext(message, AdapterConfig{}).status,
            ContextStatus::kArrayInvalid);

  message = MakeCompleteObservation();
  message.completeness =
      asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_PARTIAL;
  EXPECT_EQ(BuildLaneStripContext(message, AdapterConfig{}).status,
            ContextStatus::kArrayInvalid);
}

TEST(LaneBoundaryAdapterTest, RejectsMalformedBoundaryAndDuplicateIds) {
  auto message = MakeCompleteObservation();
  message.boundaries[0].semantic_score =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(BuildLaneStripContext(message, AdapterConfig{}).status,
            ContextStatus::kBoundaryInvalid);

  message = MakeCompleteObservation();
  message.boundaries[2].tracking_id = message.boundaries[1].tracking_id;
  EXPECT_EQ(BuildLaneStripContext(message, AdapterConfig{}).status,
            ContextStatus::kBoundaryInvalid);
}

TEST(LaneBoundaryAdapterTest, RejectsImplausibleOrCrossingStrips) {
  auto message = MakeCompleteObservation();
  message.boundaries[0] = MakeBoundary(
      11, 0, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE, 7.0);
  auto context = BuildLaneStripContext(message, AdapterConfig{});
  ASSERT_TRUE(context.usable()) << context.reason;
  EXPECT_EQ(context.strips.size(), 1u);
  EXPECT_EQ(context.rejected_pair_count, 1u);

  message = MakeCompleteObservation();
  message.boundaries[1].reference_y_m = 6.0f;
  context = BuildLaneStripContext(message, AdapterConfig{});
  EXPECT_EQ(context.status, ContextStatus::kBoundaryInvalid);
}

TEST(LaneBoundaryAdapterTest, PreservesParallelCurvatureInCenterline) {
  auto message = MakeCompleteObservation();
  message.boundaries[0] = MakeBoundary(
      11, 0, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE, 5.325, 0.02);
  message.boundaries[1] = MakeBoundary(
      12, 1, asmc_msgs::LaneBoundaryObservation::COLOR_YELLOW, 1.775, 0.02);
  message.boundaries[2] = MakeBoundary(
      13, 2, asmc_msgs::LaneBoundaryObservation::COLOR_WHITE, -1.775, 0.02);
  const auto context = BuildLaneStripContext(message, AdapterConfig{});
  ASSERT_TRUE(context.usable()) << context.reason;
  ASSERT_EQ(context.strips.size(), 2u);
  EXPECT_NEAR(context.strips[1].width_min_m, 3.55, 1e-4);
  EXPECT_NEAR(context.strips[1].width_max_m, 3.55, 1e-4);
  EXPECT_LT(context.strips[1].centerline.front().y,
            context.strips[1].centerline.back().y);
}

TEST(LaneBoundaryAdapterTest, ClipsLegacyDisplayTailsToDeclaredValidRange) {
  auto message = MakeCompleteObservation();
  for (auto& boundary : message.boundaries) {
    auto first = boundary.polyline.front();
    first.x = -1.0f;
    auto last = boundary.polyline.back();
    last.x = 16.0f;
    boundary.polyline.insert(boundary.polyline.begin(), first);
    boundary.polyline.push_back(last);
  }
  const auto context = BuildLaneStripContext(message, AdapterConfig{});
  ASSERT_TRUE(context.usable()) << context.reason;
  ASSERT_FALSE(context.strips.empty());
  EXPECT_NEAR(context.strips.front().centerline.front().x, 0.0, 1e-9);
  EXPECT_NEAR(context.strips.front().centerline.back().x, 15.0, 1e-9);
}

TEST(LaneBoundaryAdapterTest, ValidatesAdapterConfiguration) {
  AdapterConfig config;
  std::string reason;
  EXPECT_TRUE(ValidateConfig(config, &reason));
  config.lane_width_min_m = config.lane_width_max_m;
  EXPECT_FALSE(ValidateConfig(config, &reason));
  config = AdapterConfig{};
  config.reference_x_m = 16.0;
  EXPECT_FALSE(ValidateConfig(config, &reason));
  config = AdapterConfig{};
  config.inferred_center_confidence_scale = 0.0;
  EXPECT_FALSE(ValidateConfig(config, &reason));
}

}  // namespace
}  // namespace lane_boundary_adapter

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
