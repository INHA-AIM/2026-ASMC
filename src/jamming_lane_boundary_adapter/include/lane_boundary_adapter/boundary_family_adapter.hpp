#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <asmc_msgs/LaneBoundaryFamily.h>

namespace lane_boundary_adapter {

// Pure, read-only conversion from perception geometry to bounded strip
// candidates. This library deliberately does not decide ODD activation,
// home/opposing semantics, obstacle clearance, route intent, or control. A
// planning integration may reuse it, but must own those policies and must keep
// INVALID intervals split.

struct FamilyPoint2d {
  double x = 0.0;
  double y = 0.0;
};

struct BoundaryFamilyAdapterConfig {
  std::string expected_frame_id = "base_link";
  std::string expected_operating_domain = "shadow_two_way_v1";
  std::string expected_algorithm_id =
      "twinlite_v6_shadow_boundary_family_candidate_v3";
  std::string expected_representation =
      "shared_parametric_bspline_normal_offsets_v2";
  std::uint8_t expected_boundary_count = 3;

  double consumer_x_min_m = 0.0;
  double consumer_x_max_m = 15.0;
  double min_segment_length_m = 0.25;
  double lane_width_min_m = 2.4;
  double lane_width_max_m = 4.6;
  double max_width_variation_m = 0.8;
  double inferred_confidence_scale = 0.60;

  // The current read-only adapter intentionally consumes only candidate
  // messages. A future promotion must update this contract explicitly rather
  // than silently changing the source used by planning.
  bool require_deployment_disallowed = true;
};

enum class FamilyContextStatus : std::uint8_t {
  kObservedOnly = 0,
  kDegradedUsable = 1,
  kFreshInvalid = 2,
  kSourceTimestampInvalid = 3,
  kWrongFrame = 4,
  kWrongOperatingDomain = 5,
  kWrongAlgorithm = 6,
  kWrongRepresentation = 7,
  kWrongDeploymentFlag = 8,
  kMalformedFamily = 9,
  kNoUsableSegment = 10,
  kCount = 11,
};

enum class SegmentSupport : std::uint8_t {
  kObserved = 0,
  kStructureInferred = 1,
};

struct FamilyLaneStripSegment {
  std::size_t lateral_index = 0;
  std::size_t segment_index = 0;
  std::uint8_t left_role = 0;
  std::uint8_t right_role = 0;
  std::uint8_t left_color = 0;
  std::uint8_t right_color = 0;
  SegmentSupport support = SegmentSupport::kObserved;

  double valid_x_min_m = 0.0;
  double valid_x_max_m = 0.0;
  double length_m = 0.0;
  double width_min_m = 0.0;
  double width_median_m = 0.0;
  double width_max_m = 0.0;
  double min_boundary_confidence = 0.0;
  double effective_confidence = 0.0;
  bool adapter_applied_inferred_scale = false;
  bool contains_vehicle_projection = false;

  std::vector<FamilyPoint2d> left_boundary;
  std::vector<FamilyPoint2d> right_boundary;
  std::vector<FamilyPoint2d> centerline;
};

struct BoundaryFamilyStripContext {
  FamilyContextStatus status = FamilyContextStatus::kMalformedFamily;
  std::string reason;
  std::string source_config_sha256;
  std::size_t input_boundary_count = 0;
  std::size_t observed_segment_count = 0;
  std::size_t inferred_segment_count = 0;
  std::size_t invalid_interval_count = 0;
  double observed_length_m = 0.0;
  double inferred_length_m = 0.0;
  std::vector<FamilyLaneStripSegment> segments;

  bool usable() const {
    return (status == FamilyContextStatus::kObservedOnly ||
            status == FamilyContextStatus::kDegradedUsable) &&
           !segments.empty();
  }
};

bool ValidateBoundaryFamilyAdapterConfig(
    const BoundaryFamilyAdapterConfig& config, std::string* reason);

BoundaryFamilyStripContext BuildBoundaryFamilyStripContext(
    const asmc_msgs::LaneBoundaryFamily& message,
    const BoundaryFamilyAdapterConfig& config);

const char* ToString(FamilyContextStatus status);
const char* ToString(SegmentSupport support);

}  // namespace lane_boundary_adapter
