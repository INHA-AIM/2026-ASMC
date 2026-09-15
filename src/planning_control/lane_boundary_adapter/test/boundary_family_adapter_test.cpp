#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <asmc_msgs/LaneBoundaryFamily.h>
#include <asmc_msgs/LaneBoundaryFamilyMember.h>
#include <geometry_msgs/Point32.h>
#include <gtest/gtest.h>

#include "lane_boundary_adapter/boundary_family_adapter.hpp"

namespace lane_boundary_adapter {
namespace {

using Member = asmc_msgs::LaneBoundaryFamilyMember;

Member MakeMember(std::uint8_t role, std::uint8_t color, double y,
                  std::size_t sample_count = 6) {
  Member member;
  member.role = role;
  member.color = color;
  member.source_mask = Member::SOURCE_FRONT;
  member.overall_state = Member::SUPPORT_OBSERVED;
  member.confidence = 0.9F;
  member.residual_valid = true;
  member.residual_p50_m = 0.05F;
  member.residual_p95_m = 0.12F;
  member.observed_length_m = static_cast<float>(sample_count - 1);
  member.inferred_length_m = 0.0F;
  member.invalid_length_m = 0.0F;
  member.max_inferred_gap_m = 0.0F;
  member.bridged_length_ratio = 0.0F;
  for (std::size_t index = 0; index < sample_count; ++index) {
    geometry_msgs::Point32 point;
    point.x = static_cast<float>(index);
    point.y = static_cast<float>(y + 0.01 * index);
    point.z = -0.35F;
    member.polyline.push_back(point);
    member.support_state.push_back(Member::SUPPORT_OBSERVED);
  }
  return member;
}

asmc_msgs::LaneBoundaryFamily MakeFamily() {
  asmc_msgs::LaneBoundaryFamily family;
  family.header.frame_id = "base_link";
  family.valid = true;
  family.deployment_allowed = false;
  family.expected_boundary_count = 3;
  family.operating_domain = "shadow_two_way_v1";
  family.algorithm_id = "twinlite_v6_shadow_boundary_family_candidate_v3";
  family.representation = "shared_parametric_bspline_normal_offsets_v2";
  family.config_sha256 = "test-family-config";
  family.status_reason = "accepted";
  family.valid_x_min_m = 0.0F;
  family.valid_x_max_m = 5.0F;
  family.left_width_m = 3.5F;
  family.right_width_m = 3.5F;
  family.data_residual_p50_m = 0.08F;
  family.data_residual_p95_m = 0.24F;
  family.inlier_residual_p50_m = 0.05F;
  family.inlier_residual_p95_m = 0.14F;
  family.outlier_rate = 0.1F;
  family.weighted_rms_m = 0.11F;
  family.boundaries = {
      MakeMember(Member::ROLE_LEFT_OUTER_BOUNDARY, Member::COLOR_WHITE, 3.5),
      MakeMember(Member::ROLE_ROAD_CENTRE_BOUNDARY, Member::COLOR_YELLOW,
                 0.0),
      MakeMember(Member::ROLE_RIGHT_OUTER_BOUNDARY, Member::COLOR_WHITE,
                 -3.5),
  };
  return family;
}

TEST(BoundaryFamilyAdapterConfigTest, AcceptsDefaultConfiguration) {
  BoundaryFamilyAdapterConfig config;
  std::string reason;
  EXPECT_TRUE(ValidateBoundaryFamilyAdapterConfig(config, &reason));
  EXPECT_TRUE(reason.empty());
}

TEST(BoundaryFamilyAdapterConfigTest, RejectsInvalidRanges) {
  BoundaryFamilyAdapterConfig config;
  config.consumer_x_max_m = config.consumer_x_min_m;
  std::string reason;
  EXPECT_FALSE(ValidateBoundaryFamilyAdapterConfig(config, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(BoundaryFamilyAdapterTest, BuildsTwoObservedStrips) {
  const BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(MakeFamily(), {});
  ASSERT_TRUE(context.usable()) << context.reason;
  EXPECT_EQ(context.status, FamilyContextStatus::kObservedOnly);
  ASSERT_EQ(context.segments.size(), 2U);
  EXPECT_EQ(context.observed_segment_count, 2U);
  EXPECT_EQ(context.inferred_segment_count, 0U);
  EXPECT_EQ(context.invalid_interval_count, 0U);
  EXPECT_EQ(context.segments[0].support, SegmentSupport::kObserved);
  EXPECT_EQ(context.segments[1].support, SegmentSupport::kObserved);
  EXPECT_NEAR(context.segments[0].width_median_m, 3.5, 1e-5);
  EXPECT_NEAR(context.segments[1].width_median_m, 3.5, 1e-5);
  // A vehicle projection exactly on the yellow boundary may belong to both
  // adjacent geometric strips. The read-only adapter deliberately does not
  // choose home/opposing semantics here.
  EXPECT_TRUE(std::any_of(
      context.segments.begin(), context.segments.end(),
      [](const FamilyLaneStripSegment& segment) {
        return segment.contains_vehicle_projection;
      }));
}

TEST(BoundaryFamilyAdapterTest, LocallyInferredIntervalsAreSeparateAndScaled) {
  asmc_msgs::LaneBoundaryFamily family = MakeFamily();
  family.boundaries[1].support_state[2] = Member::SUPPORT_STRUCTURE_INFERRED;
  family.boundaries[1].inferred_length_m = 2.0F;
  family.boundaries[1].observed_length_m = 3.0F;
  family.boundaries[1].bridged_length_ratio = 0.4F;
  family.boundaries[1].max_inferred_gap_m = 2.0F;

  BoundaryFamilyAdapterConfig config;
  config.inferred_confidence_scale = 0.6;
  const BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(family, config);
  ASSERT_TRUE(context.usable()) << context.reason;
  EXPECT_EQ(context.status, FamilyContextStatus::kDegradedUsable);
  EXPECT_GT(context.observed_segment_count, 0U);
  EXPECT_EQ(context.inferred_segment_count, 2U);
  for (const FamilyLaneStripSegment& segment : context.segments) {
    if (segment.support != SegmentSupport::kStructureInferred) continue;
    EXPECT_TRUE(segment.adapter_applied_inferred_scale);
    EXPECT_NEAR(segment.effective_confidence,
                segment.min_boundary_confidence * 0.6, 1e-6);
  }
}

TEST(BoundaryFamilyAdapterTest, InvalidSamplesCreateHardGaps) {
  asmc_msgs::LaneBoundaryFamily family = MakeFamily();
  for (Member& member : family.boundaries) {
    member.support_state[2] = Member::SUPPORT_INVALID;
    member.support_state[3] = Member::SUPPORT_INVALID;
    member.observed_length_m = 2.0F;
    member.invalid_length_m = 3.0F;
  }
  const BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(family, {});
  ASSERT_TRUE(context.usable()) << context.reason;
  ASSERT_EQ(context.segments.size(), 4U);
  EXPECT_EQ(context.invalid_interval_count, 6U);
  for (const FamilyLaneStripSegment& segment : context.segments) {
    const bool before_gap = segment.valid_x_max_m <= 1.0 + 1e-6;
    const bool after_gap = segment.valid_x_min_m >= 4.0 - 1e-6;
    EXPECT_TRUE(before_gap || after_gap)
        << "segment crossed an invalid support gap: ["
        << segment.valid_x_min_m << "," << segment.valid_x_max_m << "]";
  }
}

TEST(BoundaryFamilyAdapterTest, EntirelyInferredMemberIsNotScaledTwice) {
  asmc_msgs::LaneBoundaryFamily family = MakeFamily();
  Member& centre = family.boundaries[1];
  centre.overall_state = Member::SUPPORT_STRUCTURE_INFERRED;
  centre.source_mask = 0;
  centre.confidence = 0.54F;
  centre.residual_valid = false;
  centre.residual_p50_m = 0.0F;
  centre.residual_p95_m = 0.0F;
  centre.observed_length_m = 0.0F;
  centre.inferred_length_m = 5.0F;
  centre.max_inferred_gap_m = 5.0F;
  centre.bridged_length_ratio = 1.0F;
  std::fill(centre.support_state.begin(), centre.support_state.end(),
            Member::SUPPORT_STRUCTURE_INFERRED);

  const BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(family, {});
  ASSERT_TRUE(context.usable()) << context.reason;
  ASSERT_EQ(context.segments.size(), 2U);
  EXPECT_EQ(context.status, FamilyContextStatus::kDegradedUsable);
  for (const FamilyLaneStripSegment& segment : context.segments) {
    EXPECT_EQ(segment.support, SegmentSupport::kStructureInferred);
    EXPECT_FALSE(segment.adapter_applied_inferred_scale);
    EXPECT_NEAR(segment.effective_confidence, 0.54, 1e-5);
  }
}

TEST(BoundaryFamilyAdapterTest, FreshProducerRejectionProducesNoGeometry) {
  asmc_msgs::LaneBoundaryFamily family = MakeFamily();
  family.valid = false;
  family.status_reason = "shared family rejected: insufficient support";
  family.boundaries.clear();
  const BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(family, {});
  EXPECT_FALSE(context.usable());
  EXPECT_EQ(context.status, FamilyContextStatus::kFreshInvalid);
  EXPECT_TRUE(context.segments.empty());
  EXPECT_EQ(context.reason, family.status_reason);
}

TEST(BoundaryFamilyAdapterTest, RejectsMalformedSampleStateArray) {
  asmc_msgs::LaneBoundaryFamily family = MakeFamily();
  family.boundaries[0].support_state.pop_back();
  const BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(family, {});
  EXPECT_FALSE(context.usable());
  EXPECT_EQ(context.status, FamilyContextStatus::kMalformedFamily);
  EXPECT_TRUE(context.segments.empty());
}

TEST(BoundaryFamilyAdapterTest, RejectsUnexpectedContractIdentity) {
  asmc_msgs::LaneBoundaryFamily family = MakeFamily();
  family.representation = "unknown_representation";
  BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(family, {});
  EXPECT_EQ(context.status, FamilyContextStatus::kWrongRepresentation);

  family = MakeFamily();
  family.operating_domain = "general_road";
  context = BuildBoundaryFamilyStripContext(family, {});
  EXPECT_EQ(context.status, FamilyContextStatus::kWrongOperatingDomain);

  family = MakeFamily();
  family.deployment_allowed = true;
  context = BuildBoundaryFamilyStripContext(family, {});
  EXPECT_EQ(context.status, FamilyContextStatus::kWrongDeploymentFlag);
}

TEST(BoundaryFamilyAdapterTest, RejectsFamilyOutsideConsumerRoi) {
  BoundaryFamilyAdapterConfig config;
  config.consumer_x_min_m = 10.0;
  config.consumer_x_max_m = 15.0;
  const BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(MakeFamily(), config);
  EXPECT_FALSE(context.usable());
  EXPECT_EQ(context.status, FamilyContextStatus::kNoUsableSegment);
}

TEST(BoundaryFamilyAdapterTest, NeverUsesSamplesOutsideProducerValidRange) {
  asmc_msgs::LaneBoundaryFamily family = MakeFamily();
  family.valid_x_min_m = 1.0F;
  family.valid_x_max_m = 4.0F;
  const BoundaryFamilyStripContext context =
      BuildBoundaryFamilyStripContext(family, {});
  ASSERT_TRUE(context.usable()) << context.reason;
  ASSERT_EQ(context.segments.size(), 2U);
  for (const FamilyLaneStripSegment& segment : context.segments) {
    EXPECT_GE(segment.valid_x_min_m, 1.0 - 1e-6);
    EXPECT_LE(segment.valid_x_max_m, 4.0 + 1e-6);
  }
}

}  // namespace
}  // namespace lane_boundary_adapter

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
