#include "lane_boundary_adapter/boundary_family_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

#include <asmc_msgs/LaneBoundaryFamilyMember.h>

namespace lane_boundary_adapter {
namespace {

constexpr double kEpsilon = 1e-6;
constexpr std::uint8_t kKnownSourceMask =
    asmc_msgs::LaneBoundaryFamilyMember::SOURCE_FRONT |
    asmc_msgs::LaneBoundaryFamilyMember::SOURCE_LEFT |
    asmc_msgs::LaneBoundaryFamilyMember::SOURCE_RIGHT;

bool IsFinite(double value) { return std::isfinite(value); }

bool IsKnownSupport(std::uint8_t value) {
  return value == asmc_msgs::LaneBoundaryFamilyMember::SUPPORT_INVALID ||
         value == asmc_msgs::LaneBoundaryFamilyMember::SUPPORT_OBSERVED ||
         value ==
                    asmc_msgs::LaneBoundaryFamilyMember::
                        SUPPORT_STRUCTURE_INFERRED;
}

double Distance(const FamilyPoint2d& first, const FamilyPoint2d& second) {
  return std::hypot(second.x - first.x, second.y - first.y);
}

double Median(std::vector<double> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  if (values.size() % 2 == 1) return values[middle];
  const double upper = values[middle];
  std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
  return 0.5 * (values[middle - 1] + upper);
}

bool ValidateMember(const asmc_msgs::LaneBoundaryFamilyMember& member,
                    std::string* reason) {
  const bool role_known =
      member.role ==
          asmc_msgs::LaneBoundaryFamilyMember::ROLE_LEFT_OUTER_BOUNDARY ||
      member.role ==
          asmc_msgs::LaneBoundaryFamilyMember::ROLE_ROAD_CENTRE_BOUNDARY ||
      member.role ==
          asmc_msgs::LaneBoundaryFamilyMember::ROLE_RIGHT_OUTER_BOUNDARY;
  if (!role_known) {
    *reason = "boundary role is outside the candidate enum";
    return false;
  }
  if (member.color != asmc_msgs::LaneBoundaryFamilyMember::COLOR_WHITE &&
      member.color != asmc_msgs::LaneBoundaryFamilyMember::COLOR_YELLOW) {
    *reason = "boundary color is not white or yellow";
    return false;
  }
  if (member.overall_state !=
          asmc_msgs::LaneBoundaryFamilyMember::SUPPORT_OBSERVED &&
      member.overall_state !=
          asmc_msgs::LaneBoundaryFamilyMember::SUPPORT_STRUCTURE_INFERRED) {
    *reason = "boundary overall state is not usable";
    return false;
  }
  if ((member.source_mask & static_cast<std::uint8_t>(~kKnownSourceMask)) !=
      0) {
    *reason = "boundary source mask contains unknown bits";
    return false;
  }
  if (member.overall_state ==
          asmc_msgs::LaneBoundaryFamilyMember::SUPPORT_OBSERVED &&
      member.source_mask == 0) {
    *reason = "an observed boundary has no camera source";
    return false;
  }
  const std::array<double, 8> scalar_values = {
      member.confidence,
      member.residual_p50_m,
      member.residual_p95_m,
      member.observed_length_m,
      member.inferred_length_m,
      member.invalid_length_m,
      member.max_inferred_gap_m,
      member.bridged_length_ratio,
  };
  if (!std::all_of(scalar_values.begin(), scalar_values.end(), IsFinite)) {
    *reason = "boundary contains a non-finite scalar";
    return false;
  }
  if (member.confidence < 0.0 || member.confidence > 1.0 ||
      member.observed_length_m < 0.0 || member.inferred_length_m < 0.0 ||
      member.invalid_length_m < 0.0 || member.max_inferred_gap_m < 0.0 ||
      member.bridged_length_ratio < 0.0 ||
      member.bridged_length_ratio > 1.0 || member.residual_p50_m < 0.0 ||
      member.residual_p95_m < 0.0) {
    *reason = "boundary quality or length is outside its valid range";
    return false;
  }
  if (member.residual_valid && member.residual_p95_m < member.residual_p50_m) {
    *reason = "boundary residual percentiles are reversed";
    return false;
  }
  if (member.overall_state ==
          asmc_msgs::LaneBoundaryFamilyMember::SUPPORT_OBSERVED &&
      !member.residual_valid) {
    *reason = "an observed boundary has no residual measurement";
    return false;
  }
  if (member.polyline.size() < 2 ||
      member.polyline.size() != member.support_state.size()) {
    *reason = "boundary point and support-state arrays do not align";
    return false;
  }
  bool has_usable_sample = false;
  for (std::size_t index = 0; index < member.polyline.size(); ++index) {
    const auto& point = member.polyline[index];
    if (!IsFinite(point.x) || !IsFinite(point.y) || !IsFinite(point.z)) {
      *reason = "boundary contains a non-finite point";
      return false;
    }
    const std::uint8_t support = member.support_state[index];
    if (!IsKnownSupport(support)) {
      *reason = "boundary contains an unknown support state";
      return false;
    }
    has_usable_sample =
        has_usable_sample ||
        support != asmc_msgs::LaneBoundaryFamilyMember::SUPPORT_INVALID;
  }
  if (!has_usable_sample) {
    *reason = "boundary contains no usable support";
    return false;
  }
  return true;
}

bool ValidateFamilyScalars(const asmc_msgs::LaneBoundaryFamily& message,
                           const BoundaryFamilyAdapterConfig& config,
                           std::string* reason) {
  const std::array<double, 10> values = {
      message.valid_x_min_m,
      message.valid_x_max_m,
      message.left_width_m,
      message.right_width_m,
      message.data_residual_p50_m,
      message.data_residual_p95_m,
      message.inlier_residual_p50_m,
      message.inlier_residual_p95_m,
      message.outlier_rate,
      message.weighted_rms_m,
  };
  if (!std::all_of(values.begin(), values.end(), IsFinite)) {
    *reason = "family contains a non-finite scalar";
    return false;
  }
  if (message.valid_x_max_m < message.valid_x_min_m ||
      message.left_width_m < config.lane_width_min_m ||
      message.left_width_m > config.lane_width_max_m ||
      message.right_width_m < config.lane_width_min_m ||
      message.right_width_m > config.lane_width_max_m ||
      message.data_residual_p50_m < 0.0 ||
      message.data_residual_p95_m < message.data_residual_p50_m ||
      message.inlier_residual_p50_m < 0.0 ||
      message.inlier_residual_p95_m < message.inlier_residual_p50_m ||
      message.outlier_rate < 0.0 || message.outlier_rate > 1.0 ||
      message.weighted_rms_m < 0.0) {
    *reason = "family scalar values violate the candidate contract";
    return false;
  }
  return true;
}

SegmentSupport IntervalSupport(
    const asmc_msgs::LaneBoundaryFamilyMember& left,
    const asmc_msgs::LaneBoundaryFamilyMember& right, std::size_t index,
    bool* valid) {
  const std::array<std::uint8_t, 4> states = {
      left.support_state[index], left.support_state[index + 1],
      right.support_state[index], right.support_state[index + 1]};
  if (std::any_of(states.begin(), states.end(), [](std::uint8_t value) {
        return value ==
               asmc_msgs::LaneBoundaryFamilyMember::SUPPORT_INVALID;
      })) {
    *valid = false;
    return SegmentSupport::kObserved;
  }
  *valid = true;
  return std::any_of(states.begin(), states.end(), [](std::uint8_t value) {
           return value == asmc_msgs::LaneBoundaryFamilyMember::
                               SUPPORT_STRUCTURE_INFERRED;
         })
             ? SegmentSupport::kStructureInferred
             : SegmentSupport::kObserved;
}

FamilyPoint2d PointAt(const asmc_msgs::LaneBoundaryFamilyMember& member,
                      std::size_t index) {
  return {member.polyline[index].x, member.polyline[index].y};
}

FamilyPoint2d Midpoint(const FamilyPoint2d& left,
                       const FamilyPoint2d& right) {
  return {0.5 * (left.x + right.x), 0.5 * (left.y + right.y)};
}

bool PointInsideRoi(const FamilyPoint2d& point,
                    const BoundaryFamilyAdapterConfig& config) {
  return point.x >= config.consumer_x_min_m - kEpsilon &&
         point.x <= config.consumer_x_max_m + kEpsilon;
}

struct SegmentBuilder {
  bool active = false;
  SegmentSupport support = SegmentSupport::kObserved;
  std::vector<FamilyPoint2d> left;
  std::vector<FamilyPoint2d> right;
};

bool FinalizeSegment(
    SegmentBuilder* builder, std::size_t lateral_index,
    const asmc_msgs::LaneBoundaryFamilyMember& left_member,
    const asmc_msgs::LaneBoundaryFamilyMember& right_member,
    const BoundaryFamilyAdapterConfig& config,
    BoundaryFamilyStripContext* context, std::string* reason) {
  if (!builder->active) return true;
  builder->active = false;
  if (builder->left.size() < 2 ||
      builder->left.size() != builder->right.size()) {
    builder->left.clear();
    builder->right.clear();
    return true;
  }

  FamilyLaneStripSegment segment;
  segment.lateral_index = lateral_index;
  segment.segment_index = context->segments.size();
  segment.left_role = left_member.role;
  segment.right_role = right_member.role;
  segment.left_color = left_member.color;
  segment.right_color = right_member.color;
  segment.support = builder->support;
  segment.left_boundary = std::move(builder->left);
  segment.right_boundary = std::move(builder->right);
  builder->left.clear();
  builder->right.clear();

  std::vector<double> widths;
  widths.reserve(segment.left_boundary.size());
  segment.centerline.reserve(segment.left_boundary.size());
  for (std::size_t index = 0; index < segment.left_boundary.size(); ++index) {
    const FamilyPoint2d& left = segment.left_boundary[index];
    const FamilyPoint2d& right = segment.right_boundary[index];
    if (left.y <= right.y + kEpsilon) {
      *reason = "family boundaries cross or reverse lateral order";
      return false;
    }
    const double width = Distance(left, right);
    if (!IsFinite(width) || width < config.lane_width_min_m ||
        width > config.lane_width_max_m) {
      *reason = "family segment width lies outside the shadow ODD bounds";
      return false;
    }
    widths.push_back(width);
    segment.centerline.push_back(Midpoint(left, right));
    segment.contains_vehicle_projection =
        segment.contains_vehicle_projection ||
        (left.y >= -kEpsilon && right.y <= kEpsilon);
    if (index > 0) {
      segment.length_m +=
          Distance(segment.centerline[index - 1], segment.centerline[index]);
    }
  }
  if (segment.length_m + kEpsilon < config.min_segment_length_m) return true;

  segment.width_min_m = *std::min_element(widths.begin(), widths.end());
  segment.width_max_m = *std::max_element(widths.begin(), widths.end());
  segment.width_median_m = Median(widths);
  if (segment.width_max_m - segment.width_min_m >
      config.max_width_variation_m) {
    *reason = "family segment width varies too much";
    return false;
  }
  segment.valid_x_min_m = std::numeric_limits<double>::infinity();
  segment.valid_x_max_m = -std::numeric_limits<double>::infinity();
  for (const auto& point : segment.centerline) {
    segment.valid_x_min_m = std::min(segment.valid_x_min_m, point.x);
    segment.valid_x_max_m = std::max(segment.valid_x_max_m, point.x);
  }
  segment.min_boundary_confidence =
      std::min(static_cast<double>(left_member.confidence),
               static_cast<double>(right_member.confidence));
  double scale = 1.0;
  if (segment.support == SegmentSupport::kStructureInferred) {
    // An entirely inferred member is already down-weighted by the producer.
    // Apply the local degradation factor only when both aggregate members are
    // otherwise observed, which avoids silently squaring the 0.60 scale.
    const bool producer_already_scaled =
        left_member.overall_state == asmc_msgs::LaneBoundaryFamilyMember::
                                         SUPPORT_STRUCTURE_INFERRED ||
        right_member.overall_state == asmc_msgs::LaneBoundaryFamilyMember::
                                          SUPPORT_STRUCTURE_INFERRED;
    scale = producer_already_scaled ? 1.0 : config.inferred_confidence_scale;
    segment.adapter_applied_inferred_scale = !producer_already_scaled;
  }
  segment.effective_confidence = segment.min_boundary_confidence * scale;
  if (segment.support == SegmentSupport::kStructureInferred) {
    ++context->inferred_segment_count;
    context->inferred_length_m += segment.length_m;
  } else {
    ++context->observed_segment_count;
    context->observed_length_m += segment.length_m;
  }
  context->segments.push_back(std::move(segment));
  return true;
}

bool BuildPairSegments(
    const asmc_msgs::LaneBoundaryFamilyMember& left,
    const asmc_msgs::LaneBoundaryFamilyMember& right,
    std::size_t lateral_index, const BoundaryFamilyAdapterConfig& config,
    BoundaryFamilyStripContext* context, std::string* reason) {
  if (left.polyline.size() != right.polyline.size()) {
    *reason = "shared family members have different sample counts";
    return false;
  }
  SegmentBuilder builder;
  for (std::size_t index = 0; index + 1 < left.polyline.size(); ++index) {
    const FamilyPoint2d left_start = PointAt(left, index);
    const FamilyPoint2d right_start = PointAt(right, index);
    const FamilyPoint2d left_end = PointAt(left, index + 1);
    const FamilyPoint2d right_end = PointAt(right, index + 1);
    const FamilyPoint2d centre_start = Midpoint(left_start, right_start);
    const FamilyPoint2d centre_end = Midpoint(left_end, right_end);
    bool support_valid = false;
    const SegmentSupport support =
        IntervalSupport(left, right, index, &support_valid);
    const bool interval_usable =
        support_valid && PointInsideRoi(centre_start, config) &&
        PointInsideRoi(centre_end, config);
    if (!interval_usable) {
      if (!support_valid) ++context->invalid_interval_count;
      if (!FinalizeSegment(&builder, lateral_index, left, right, config,
                           context, reason)) {
        return false;
      }
      continue;
    }
    if (builder.active && builder.support != support) {
      if (!FinalizeSegment(&builder, lateral_index, left, right, config,
                           context, reason)) {
        return false;
      }
    }
    if (!builder.active) {
      builder.active = true;
      builder.support = support;
      builder.left.push_back(left_start);
      builder.right.push_back(right_start);
    }
    builder.left.push_back(left_end);
    builder.right.push_back(right_end);
  }
  return FinalizeSegment(&builder, lateral_index, left, right, config, context,
                         reason);
}

}  // namespace

bool ValidateBoundaryFamilyAdapterConfig(
    const BoundaryFamilyAdapterConfig& config, std::string* reason) {
  if (reason == nullptr) return false;
  if (config.expected_frame_id.empty() ||
      config.expected_operating_domain.empty() ||
      config.expected_algorithm_id.empty() ||
      config.expected_representation.empty()) {
    *reason = "expected boundary-family identifiers must be non-empty";
    return false;
  }
  if (config.expected_boundary_count != 3) {
    *reason = "the shadow candidate currently requires exactly three boundaries";
    return false;
  }
  const std::array<double, 7> finite_values = {
      config.consumer_x_min_m,
      config.consumer_x_max_m,
      config.min_segment_length_m,
      config.lane_width_min_m,
      config.lane_width_max_m,
      config.max_width_variation_m,
      config.inferred_confidence_scale,
  };
  if (!std::all_of(finite_values.begin(), finite_values.end(), IsFinite)) {
    *reason = "boundary-family adapter config contains a non-finite value";
    return false;
  }
  if (config.consumer_x_max_m <= config.consumer_x_min_m ||
      config.min_segment_length_m <= 0.0 || config.lane_width_min_m <= 0.0 ||
      config.lane_width_max_m <= config.lane_width_min_m ||
      config.max_width_variation_m <= 0.0 ||
      config.inferred_confidence_scale <= 0.0 ||
      config.inferred_confidence_scale > 1.0) {
    *reason = "boundary-family adapter config ranges are invalid";
    return false;
  }
  reason->clear();
  return true;
}

BoundaryFamilyStripContext BuildBoundaryFamilyStripContext(
    const asmc_msgs::LaneBoundaryFamily& message,
    const BoundaryFamilyAdapterConfig& config) {
  BoundaryFamilyStripContext context;
  context.input_boundary_count = message.boundaries.size();
  context.source_config_sha256 = message.config_sha256;
  std::string reason;
  if (!ValidateBoundaryFamilyAdapterConfig(config, &reason)) {
    context.reason = "adapter config invalid: " + reason;
    return context;
  }
  if (message.header.frame_id != config.expected_frame_id) {
    context.status = FamilyContextStatus::kWrongFrame;
    context.reason = "boundary family is not in the expected frame";
    return context;
  }
  if (message.operating_domain != config.expected_operating_domain) {
    context.status = FamilyContextStatus::kWrongOperatingDomain;
    context.reason = "boundary family is outside the configured ODD";
    return context;
  }
  if (message.algorithm_id != config.expected_algorithm_id) {
    context.status = FamilyContextStatus::kWrongAlgorithm;
    context.reason = "boundary family algorithm_id is not approved";
    return context;
  }
  if (message.representation != config.expected_representation) {
    context.status = FamilyContextStatus::kWrongRepresentation;
    context.reason = "boundary family representation is not approved";
    return context;
  }
  if (config.require_deployment_disallowed && message.deployment_allowed) {
    context.status = FamilyContextStatus::kWrongDeploymentFlag;
    context.reason = "read-only candidate unexpectedly claims deployment approval";
    return context;
  }
  if (!message.valid) {
    context.status = FamilyContextStatus::kFreshInvalid;
    context.reason = message.status_reason.empty() ? "producer rejected family"
                                                   : message.status_reason;
    return context;
  }
  if (message.expected_boundary_count != config.expected_boundary_count ||
      message.boundaries.size() != config.expected_boundary_count ||
      message.config_sha256.empty()) {
    context.reason = "boundary family count or config hash is invalid";
    return context;
  }
  if (!ValidateFamilyScalars(message, config, &reason)) {
    context.reason = reason;
    return context;
  }

  BoundaryFamilyAdapterConfig effective_config = config;
  effective_config.consumer_x_min_m =
      std::max(config.consumer_x_min_m,
               static_cast<double>(message.valid_x_min_m));
  effective_config.consumer_x_max_m =
      std::min(config.consumer_x_max_m,
               static_cast<double>(message.valid_x_max_m));
  if (effective_config.consumer_x_max_m <=
      effective_config.consumer_x_min_m + kEpsilon) {
    context.status = FamilyContextStatus::kNoUsableSegment;
    context.reason =
        "producer valid range does not overlap the consumer ROI";
    return context;
  }

  std::map<std::uint8_t, const asmc_msgs::LaneBoundaryFamilyMember*> by_role;
  for (const auto& member : message.boundaries) {
    if (!ValidateMember(member, &reason)) {
      context.reason = reason;
      return context;
    }
    if (!by_role.emplace(member.role, &member).second) {
      context.reason = "boundary family contains a duplicate role";
      return context;
    }
  }
  const std::array<std::uint8_t, 3> roles = {
      asmc_msgs::LaneBoundaryFamilyMember::ROLE_LEFT_OUTER_BOUNDARY,
      asmc_msgs::LaneBoundaryFamilyMember::ROLE_ROAD_CENTRE_BOUNDARY,
      asmc_msgs::LaneBoundaryFamilyMember::ROLE_RIGHT_OUTER_BOUNDARY,
  };
  if (!std::all_of(roles.begin(), roles.end(), [&](std::uint8_t role) {
        return by_role.count(role) == 1;
      })) {
    context.reason = "boundary family is missing a required role";
    return context;
  }
  const auto& left = *by_role.at(roles[0]);
  const auto& centre = *by_role.at(roles[1]);
  const auto& right = *by_role.at(roles[2]);
  if (left.color != asmc_msgs::LaneBoundaryFamilyMember::COLOR_WHITE ||
      centre.color != asmc_msgs::LaneBoundaryFamilyMember::COLOR_YELLOW ||
      right.color != asmc_msgs::LaneBoundaryFamilyMember::COLOR_WHITE) {
    context.reason = "boundary family role colors violate the shadow ODD";
    return context;
  }
  if (!BuildPairSegments(left, centre, 0, effective_config, &context,
                         &reason) ||
      !BuildPairSegments(centre, right, 1, effective_config, &context,
                         &reason)) {
    context.segments.clear();
    context.reason = reason;
    return context;
  }
  if (context.segments.empty()) {
    context.status = FamilyContextStatus::kNoUsableSegment;
    context.reason = "all family intervals are invalid or outside the consumer ROI";
    return context;
  }
  if (context.inferred_segment_count > 0) {
    context.status = FamilyContextStatus::kDegradedUsable;
    context.reason = "usable family contains structure-inferred segments";
  } else {
    context.status = FamilyContextStatus::kObservedOnly;
    context.reason = "all usable family segments are directly observed";
  }
  return context;
}

const char* ToString(FamilyContextStatus status) {
  switch (status) {
    case FamilyContextStatus::kObservedOnly:
      return "observed_only";
    case FamilyContextStatus::kDegradedUsable:
      return "degraded_usable";
    case FamilyContextStatus::kFreshInvalid:
      return "fresh_invalid";
    case FamilyContextStatus::kSourceTimestampInvalid:
      return "source_timestamp_invalid";
    case FamilyContextStatus::kWrongFrame:
      return "wrong_frame";
    case FamilyContextStatus::kWrongOperatingDomain:
      return "wrong_operating_domain";
    case FamilyContextStatus::kWrongAlgorithm:
      return "wrong_algorithm";
    case FamilyContextStatus::kWrongRepresentation:
      return "wrong_representation";
    case FamilyContextStatus::kWrongDeploymentFlag:
      return "wrong_deployment_flag";
    case FamilyContextStatus::kMalformedFamily:
      return "malformed_family";
    case FamilyContextStatus::kNoUsableSegment:
      return "no_usable_segment";
    case FamilyContextStatus::kCount:
      return "count";
  }
  return "unknown";
}

const char* ToString(SegmentSupport support) {
  switch (support) {
    case SegmentSupport::kObserved:
      return "observed";
    case SegmentSupport::kStructureInferred:
      return "structure_inferred";
  }
  return "unknown";
}

}  // namespace lane_boundary_adapter
