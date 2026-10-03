#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <asmc_msgs/LaneBoundaryObservationArray.h>

namespace lane_boundary_adapter {

struct Point2d {
  double x = 0.0;
  double y = 0.0;
};

struct AdapterConfig {
  std::string expected_frame_id = "base_link";
  std::string expected_operating_domain = "shadow_two_way_v1";
  std::string expected_algorithm_id =
      "twinlite_v6_shadow_boundary_contract_v1";
  std::uint8_t expected_boundary_count = 3;

  double reference_x_m = 5.0;
  double consumer_x_min_m = 0.0;
  double consumer_x_max_m = 15.0;
  double sample_spacing_m = 0.25;
  double min_common_length_m = 6.0;
  double lane_width_min_m = 2.4;
  double lane_width_max_m = 4.6;
  double max_width_variation_m = 0.8;

  // The first adapter is intentionally restricted to a two-way-road ODD,
  // where one yellow and one white boundary define a lane strip. This is not
  // a property of the general perception message.
  bool require_white_yellow_pair = true;
  bool allow_outer_white_pair_fallback = true;
  double inferred_center_confidence_scale = 0.65;
};

enum class ContextStatus : std::uint8_t {
  kValid = 0,
  kEmpty = 1,
  kArrayInvalid = 2,
  kWrongFrame = 3,
  kWrongOperatingDomain = 4,
  kWrongAlgorithm = 5,
  kOrderInvalid = 6,
  kBoundaryInvalid = 7,
  kNoUsableStrip = 8,
  kDegradedInferredCenter = 9,
  kCount = 10,
};

struct LaneStripCandidate {
  std::size_t lateral_index = 0;
  std::uint32_t left_boundary_id = 0;
  std::uint32_t right_boundary_id = 0;
  std::uint8_t left_color = 0;
  std::uint8_t right_color = 0;

  double valid_x_min_m = 0.0;
  double valid_x_max_m = 0.0;
  double width_min_m = 0.0;
  double width_median_m = 0.0;
  double width_max_m = 0.0;
  double min_semantic_score = 0.0;
  double min_geometric_quality = 0.0;
  bool contains_vehicle_projection = false;
  bool inferred_center_boundary = false;

  std::vector<Point2d> left_boundary;
  std::vector<Point2d> right_boundary;
  std::vector<Point2d> centerline;
};

struct LaneStripContext {
  ContextStatus status = ContextStatus::kArrayInvalid;
  std::string reason;
  std::uint8_t input_completeness = 0;
  std::uint8_t expected_boundary_count = 0;
  std::size_t input_boundary_count = 0;
  std::size_t rejected_pair_count = 0;
  int occupied_strip_index = -1;
  std::vector<LaneStripCandidate> strips;

  bool usable() const {
    return (status == ContextStatus::kValid ||
            status == ContextStatus::kDegradedInferredCenter) &&
           !strips.empty();
  }
};

bool ValidateConfig(const AdapterConfig& config, std::string* reason);

LaneStripContext BuildLaneStripContext(
    const asmc_msgs::LaneBoundaryObservationArray& message,
    const AdapterConfig& config);

const char* ToString(ContextStatus status);

}  // namespace lane_boundary_adapter
