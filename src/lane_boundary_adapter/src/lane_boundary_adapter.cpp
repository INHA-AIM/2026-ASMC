#include "lane_boundary_adapter/lane_boundary_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

#include <asmc_msgs/LaneBoundaryObservation.h>

namespace lane_boundary_adapter {
namespace {

constexpr double kEpsilon = 1e-6;
constexpr std::uint8_t kKnownSourceMask =
    asmc_msgs::LaneBoundaryObservation::SOURCE_FRONT |
    asmc_msgs::LaneBoundaryObservation::SOURCE_LEFT |
    asmc_msgs::LaneBoundaryObservation::SOURCE_RIGHT;

bool IsFinite(double value) { return std::isfinite(value); }

bool IsKnownColor(std::uint8_t color) {
  return color == asmc_msgs::LaneBoundaryObservation::COLOR_UNKNOWN ||
         color == asmc_msgs::LaneBoundaryObservation::COLOR_WHITE ||
         color == asmc_msgs::LaneBoundaryObservation::COLOR_YELLOW;
}

bool IsWhiteYellowPair(std::uint8_t left, std::uint8_t right) {
  return (left == asmc_msgs::LaneBoundaryObservation::COLOR_WHITE &&
          right == asmc_msgs::LaneBoundaryObservation::COLOR_YELLOW) ||
         (left == asmc_msgs::LaneBoundaryObservation::COLOR_YELLOW &&
          right == asmc_msgs::LaneBoundaryObservation::COLOR_WHITE);
}

bool ValidateBoundary(const asmc_msgs::LaneBoundaryObservation& boundary,
                      std::string* reason) {
  if (!IsKnownColor(boundary.color)) {
    *reason = "boundary color is outside the public enum";
    return false;
  }
  if (boundary.source_mask == 0 ||
      (boundary.source_mask & static_cast<std::uint8_t>(~kKnownSourceMask)) !=
          0) {
    *reason = "boundary source_mask is invalid";
    return false;
  }
  const double scalar_values[] = {
      boundary.semantic_score,
      boundary.geometric_quality,
      boundary.reference_x_m,
      boundary.reference_y_m,
      boundary.valid_x_min_m,
      boundary.valid_x_max_m,
      boundary.observed_length_m,
      boundary.bridged_length_ratio,
  };
  for (const double value : scalar_values) {
    if (!IsFinite(value)) {
      *reason = "boundary contains a non-finite scalar";
      return false;
    }
  }
  if (boundary.semantic_score < 0.0 || boundary.semantic_score > 1.0 ||
      boundary.geometric_quality < 0.0 ||
      boundary.geometric_quality > 1.0 ||
      boundary.bridged_length_ratio < 0.0 ||
      boundary.bridged_length_ratio > 1.0 ||
      boundary.observed_length_m < 0.0) {
    *reason = "boundary quality or length is outside its valid range";
    return false;
  }
  if (boundary.valid_x_max_m < boundary.valid_x_min_m) {
    *reason = "boundary valid x range is reversed";
    return false;
  }
  if (boundary.polyline.size() < 2) {
    *reason = "boundary polyline has fewer than two points";
    return false;
  }
  double previous_x = -std::numeric_limits<double>::infinity();
  for (const auto& point : boundary.polyline) {
    if (!IsFinite(point.x) || !IsFinite(point.y) || !IsFinite(point.z)) {
      *reason = "boundary polyline contains a non-finite point";
      return false;
    }
    if (point.x + kEpsilon < previous_x) {
      *reason = "boundary polyline is not ordered by increasing x";
      return false;
    }
    previous_x = point.x;
  }
  // Older typed result bags can contain fitted display samples outside the
  // declared evidence-valid interval.  Never consume those tails, but accept
  // the message when the declared interval overlaps the ordered polyline.
  // The current producer clips new messages to the valid interval.
  if (boundary.polyline.back().x < boundary.valid_x_min_m - kEpsilon ||
      boundary.polyline.front().x > boundary.valid_x_max_m + kEpsilon) {
    *reason = "boundary polyline does not overlap its declared valid x range";
    return false;
  }
  return true;
}

bool InterpolateY(const asmc_msgs::LaneBoundaryObservation& boundary,
                  double x, double* y) {
  if (boundary.polyline.empty() || x < boundary.polyline.front().x - kEpsilon ||
      x > boundary.polyline.back().x + kEpsilon) {
    return false;
  }
  const auto upper = std::lower_bound(
      boundary.polyline.begin(), boundary.polyline.end(), x,
      [](const geometry_msgs::Point32& point, double target_x) {
        return point.x < target_x;
      });
  if (upper == boundary.polyline.begin()) {
    *y = upper->y;
    return true;
  }
  if (upper == boundary.polyline.end()) {
    *y = boundary.polyline.back().y;
    return true;
  }
  const auto lower = std::prev(upper);
  const double dx = static_cast<double>(upper->x) - lower->x;
  if (std::abs(dx) <= kEpsilon) {
    *y = 0.5 * (static_cast<double>(lower->y) + upper->y);
    return true;
  }
  const double ratio = (x - lower->x) / dx;
  *y = lower->y + ratio * (static_cast<double>(upper->y) - lower->y);
  return IsFinite(*y);
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

std::vector<double> SamplePositions(double start, double end,
                                    double spacing) {
  std::vector<double> result;
  const std::size_t count = static_cast<std::size_t>(
      std::floor((end - start) / spacing + kEpsilon));
  result.reserve(count + 2);
  for (std::size_t index = 0; index <= count; ++index) {
    result.push_back(start + spacing * static_cast<double>(index));
  }
  if (result.empty() || end - result.back() > kEpsilon) result.push_back(end);
  return result;
}

bool BuildStrip(const asmc_msgs::LaneBoundaryObservation& left,
                const asmc_msgs::LaneBoundaryObservation& right,
                std::size_t lateral_index, const AdapterConfig& config,
                LaneStripCandidate* result, std::string* reason) {
  if (config.require_white_yellow_pair &&
      !IsWhiteYellowPair(left.color, right.color)) {
    *reason = "pair is not one white and one yellow boundary";
    return false;
  }
  const double start = std::max(
      {static_cast<double>(left.valid_x_min_m),
       static_cast<double>(right.valid_x_min_m), config.consumer_x_min_m,
       static_cast<double>(left.polyline.front().x),
       static_cast<double>(right.polyline.front().x)});
  const double end = std::min(
      {static_cast<double>(left.valid_x_max_m),
       static_cast<double>(right.valid_x_max_m), config.consumer_x_max_m,
       static_cast<double>(left.polyline.back().x),
       static_cast<double>(right.polyline.back().x)});
  if (!IsFinite(start) || !IsFinite(end) ||
      end - start + kEpsilon < config.min_common_length_m) {
    *reason = "pair has insufficient common longitudinal support";
    return false;
  }

  LaneStripCandidate candidate;
  candidate.lateral_index = lateral_index;
  candidate.left_boundary_id = left.tracking_id;
  candidate.right_boundary_id = right.tracking_id;
  candidate.left_color = left.color;
  candidate.right_color = right.color;
  candidate.valid_x_min_m = start;
  candidate.valid_x_max_m = end;
  candidate.min_semantic_score =
      std::min(static_cast<double>(left.semantic_score),
               static_cast<double>(right.semantic_score));
  candidate.min_geometric_quality =
      std::min(static_cast<double>(left.geometric_quality),
               static_cast<double>(right.geometric_quality));

  std::vector<double> widths;
  for (const double x : SamplePositions(start, end, config.sample_spacing_m)) {
    double left_y = 0.0;
    double right_y = 0.0;
    if (!InterpolateY(left, x, &left_y) ||
        !InterpolateY(right, x, &right_y)) {
      *reason = "pair interpolation failed inside common support";
      return false;
    }
    const double width = left_y - right_y;
    if (!IsFinite(width) || width <= 0.0) {
      *reason = "pair boundaries cross or have reversed lateral order";
      return false;
    }
    candidate.left_boundary.push_back({x, left_y});
    candidate.right_boundary.push_back({x, right_y});
    candidate.centerline.push_back({x, 0.5 * (left_y + right_y)});
    widths.push_back(width);
  }
  candidate.width_min_m = *std::min_element(widths.begin(), widths.end());
  candidate.width_max_m = *std::max_element(widths.begin(), widths.end());
  candidate.width_median_m = Median(widths);
  if (candidate.width_min_m < config.lane_width_min_m ||
      candidate.width_max_m > config.lane_width_max_m) {
    *reason = "pair width lies outside the shadow ODD bounds";
    return false;
  }
  if (candidate.width_max_m - candidate.width_min_m >
      config.max_width_variation_m) {
    *reason = "pair width varies too much over the common support";
    return false;
  }

  if (config.reference_x_m >= start - kEpsilon &&
      config.reference_x_m <= end + kEpsilon) {
    double left_y = 0.0;
    double right_y = 0.0;
    if (InterpolateY(left, config.reference_x_m, &left_y) &&
        InterpolateY(right, config.reference_x_m, &right_y)) {
      candidate.contains_vehicle_projection =
          left_y >= -kEpsilon && right_y <= kEpsilon;
    }
  }
  *result = std::move(candidate);
  return true;
}

bool BuildOuterWhiteFallback(
    const asmc_msgs::LaneBoundaryObservation& left,
    const asmc_msgs::LaneBoundaryObservation& right,
    const AdapterConfig& config, std::vector<LaneStripCandidate>* result,
    std::string* reason) {
  if (left.color != asmc_msgs::LaneBoundaryObservation::COLOR_WHITE ||
      right.color != asmc_msgs::LaneBoundaryObservation::COLOR_WHITE) {
    *reason = "outer-boundary fallback requires exactly two white boundaries";
    return false;
  }
  const double start = std::max(
      {static_cast<double>(left.valid_x_min_m),
       static_cast<double>(right.valid_x_min_m), config.consumer_x_min_m,
       static_cast<double>(left.polyline.front().x),
       static_cast<double>(right.polyline.front().x)});
  const double end = std::min(
      {static_cast<double>(left.valid_x_max_m),
       static_cast<double>(right.valid_x_max_m), config.consumer_x_max_m,
       static_cast<double>(left.polyline.back().x),
       static_cast<double>(right.polyline.back().x)});
  if (!IsFinite(start) || !IsFinite(end) ||
      end - start + kEpsilon < config.min_common_length_m) {
    *reason = "white outer pair has insufficient common longitudinal support";
    return false;
  }

  const auto positions = SamplePositions(start, end, config.sample_spacing_m);
  std::vector<Point2d> left_outer;
  std::vector<Point2d> inferred_center;
  std::vector<Point2d> right_outer;
  std::vector<double> half_widths;
  left_outer.reserve(positions.size());
  inferred_center.reserve(positions.size());
  right_outer.reserve(positions.size());
  half_widths.reserve(positions.size());
  for (const double x : positions) {
    double left_y = 0.0;
    double right_y = 0.0;
    if (!InterpolateY(left, x, &left_y) ||
        !InterpolateY(right, x, &right_y)) {
      *reason = "white outer pair interpolation failed inside common support";
      return false;
    }
    const double road_width = left_y - right_y;
    const double half_width = 0.5 * road_width;
    if (!IsFinite(half_width) || half_width < config.lane_width_min_m ||
        half_width > config.lane_width_max_m) {
      *reason = "white outer pair does not describe two plausible lane widths";
      return false;
    }
    left_outer.push_back({x, left_y});
    inferred_center.push_back({x, 0.5 * (left_y + right_y)});
    right_outer.push_back({x, right_y});
    half_widths.push_back(half_width);
  }
  const double width_min =
      *std::min_element(half_widths.begin(), half_widths.end());
  const double width_max =
      *std::max_element(half_widths.begin(), half_widths.end());
  if (width_max - width_min > config.max_width_variation_m) {
    *reason = "inferred lane width varies too much over common support";
    return false;
  }

  auto make_strip = [&](std::size_t index, const std::vector<Point2d>& strip_left,
                        const std::vector<Point2d>& strip_right,
                        std::uint32_t left_id, std::uint32_t right_id,
                        std::uint8_t left_color,
                        std::uint8_t right_color) {
    LaneStripCandidate candidate;
    candidate.lateral_index = index;
    candidate.left_boundary_id = left_id;
    candidate.right_boundary_id = right_id;
    candidate.left_color = left_color;
    candidate.right_color = right_color;
    candidate.valid_x_min_m = start;
    candidate.valid_x_max_m = end;
    candidate.width_min_m = width_min;
    candidate.width_median_m = Median(half_widths);
    candidate.width_max_m = width_max;
    candidate.min_semantic_score =
        std::min(static_cast<double>(left.semantic_score),
                 static_cast<double>(right.semantic_score)) *
        config.inferred_center_confidence_scale;
    candidate.min_geometric_quality =
        std::min(static_cast<double>(left.geometric_quality),
                 static_cast<double>(right.geometric_quality)) *
        config.inferred_center_confidence_scale;
    candidate.inferred_center_boundary = true;
    candidate.left_boundary = strip_left;
    candidate.right_boundary = strip_right;
    candidate.centerline.reserve(strip_left.size());
    for (std::size_t point = 0; point < strip_left.size(); ++point) {
      candidate.centerline.push_back(
          {strip_left[point].x,
           0.5 * (strip_left[point].y + strip_right[point].y)});
    }
    if (config.reference_x_m >= start - kEpsilon &&
        config.reference_x_m <= end + kEpsilon) {
      double left_y = 0.0;
      double right_y = 0.0;
      if (InterpolateY(left, config.reference_x_m, &left_y) &&
          InterpolateY(right, config.reference_x_m, &right_y)) {
        const double center_y = 0.5 * (left_y + right_y);
        const double strip_left_y = index == 0 ? left_y : center_y;
        const double strip_right_y = index == 0 ? center_y : right_y;
        candidate.contains_vehicle_projection =
            strip_left_y >= -kEpsilon && strip_right_y <= kEpsilon;
      }
    }
    return candidate;
  };

  result->clear();
  result->push_back(make_strip(
      0, left_outer, inferred_center, left.tracking_id, 0,
      asmc_msgs::LaneBoundaryObservation::COLOR_WHITE,
      asmc_msgs::LaneBoundaryObservation::COLOR_YELLOW));
  result->push_back(make_strip(
      1, inferred_center, right_outer, 0, right.tracking_id,
      asmc_msgs::LaneBoundaryObservation::COLOR_YELLOW,
      asmc_msgs::LaneBoundaryObservation::COLOR_WHITE));
  return true;
}

}  // namespace

bool ValidateConfig(const AdapterConfig& config, std::string* reason) {
  if (reason == nullptr) return false;
  if (config.expected_frame_id.empty() ||
      config.expected_operating_domain.empty()) {
    *reason = "expected frame and operating domain are required";
    return false;
  }
  const double values[] = {
      config.reference_x_m,       config.consumer_x_min_m,
      config.consumer_x_max_m,   config.sample_spacing_m,
      config.min_common_length_m, config.lane_width_min_m,
      config.lane_width_max_m,   config.max_width_variation_m,
      config.inferred_center_confidence_scale,
  };
  for (const double value : values) {
    if (!IsFinite(value)) {
      *reason = "adapter configuration contains a non-finite value";
      return false;
    }
  }
  if (config.consumer_x_max_m <= config.consumer_x_min_m ||
      config.sample_spacing_m <= 0.0 || config.min_common_length_m <= 0.0 ||
      config.lane_width_min_m <= 0.0 ||
      config.lane_width_max_m <= config.lane_width_min_m ||
      config.max_width_variation_m < 0.0 ||
      config.inferred_center_confidence_scale <= 0.0 ||
      config.inferred_center_confidence_scale > 1.0) {
    *reason = "adapter configuration ranges are invalid";
    return false;
  }
  if (config.reference_x_m < config.consumer_x_min_m ||
      config.reference_x_m > config.consumer_x_max_m) {
    *reason = "reference x must lie inside the consumer x range";
    return false;
  }
  *reason = "ok";
  return true;
}

LaneStripContext BuildLaneStripContext(
    const asmc_msgs::LaneBoundaryObservationArray& message,
    const AdapterConfig& config) {
  LaneStripContext context;
  context.input_completeness = message.completeness;
  context.expected_boundary_count = message.expected_boundary_count;
  context.input_boundary_count = message.boundaries.size();

  std::string config_reason;
  if (!ValidateConfig(config, &config_reason)) {
    context.status = ContextStatus::kBoundaryInvalid;
    context.reason = "invalid adapter config: " + config_reason;
    return context;
  }
  if (!message.valid) {
    context.status = ContextStatus::kArrayInvalid;
    context.reason = "perception marked the synchronized cycle invalid";
    return context;
  }
  if (message.expected_boundary_count != config.expected_boundary_count) {
    context.status = ContextStatus::kArrayInvalid;
    context.reason = "unexpected expected_boundary_count for the shadow ODD";
    return context;
  }
  if (message.completeness >
      asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_COMPLETE) {
    context.status = ContextStatus::kArrayInvalid;
    context.reason = "completeness is outside the public enum";
    return context;
  }
  if (message.boundaries.size() > message.expected_boundary_count) {
    context.status = ContextStatus::kArrayInvalid;
    context.reason = "boundary count exceeds expected_boundary_count";
    return context;
  }
  if (message.boundaries.empty() &&
      message.completeness !=
          asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_EMPTY) {
    context.status = ContextStatus::kArrayInvalid;
    context.reason = "empty boundary array has inconsistent completeness";
    return context;
  }
  if (!message.boundaries.empty() &&
      message.completeness ==
          asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_EMPTY) {
    context.status = ContextStatus::kArrayInvalid;
    context.reason = "non-empty boundary array is marked empty";
    return context;
  }
  if (message.completeness ==
          asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_COMPLETE &&
      message.boundaries.size() != message.expected_boundary_count) {
    context.status = ContextStatus::kArrayInvalid;
    context.reason = "complete observation does not contain the expected count";
    return context;
  }
  if (message.completeness ==
          asmc_msgs::LaneBoundaryObservationArray::COMPLETENESS_PARTIAL &&
      message.boundaries.size() >= message.expected_boundary_count) {
    context.status = ContextStatus::kArrayInvalid;
    context.reason = "partial observation contains the complete boundary count";
    return context;
  }
  if (message.header.frame_id != config.expected_frame_id) {
    context.status = ContextStatus::kWrongFrame;
    context.reason = "unexpected frame_id";
    return context;
  }
  if (message.operating_domain != config.expected_operating_domain) {
    context.status = ContextStatus::kWrongOperatingDomain;
    context.reason = "unexpected operating_domain";
    return context;
  }
  if (!config.expected_algorithm_id.empty() &&
      message.algorithm_id != config.expected_algorithm_id) {
    context.status = ContextStatus::kWrongAlgorithm;
    context.reason = "unexpected boundary algorithm_id";
    return context;
  }
  if (message.boundaries.empty()) {
    context.status = ContextStatus::kEmpty;
    context.reason = "fresh valid observation contains no boundaries";
    return context;
  }

  std::set<std::uint32_t> tracking_ids;
  std::vector<const asmc_msgs::LaneBoundaryObservation*> ordered;
  ordered.reserve(message.boundaries.size());
  for (const auto& boundary : message.boundaries) {
    std::string boundary_reason;
    if (!ValidateBoundary(boundary, &boundary_reason)) {
      context.status = ContextStatus::kBoundaryInvalid;
      context.reason = boundary_reason;
      return context;
    }
    if (!tracking_ids.insert(boundary.tracking_id).second) {
      context.status = ContextStatus::kBoundaryInvalid;
      context.reason = "tracking_id is duplicated in one observation";
      return context;
    }
    if (!boundary.order_valid) {
      context.status = ContextStatus::kOrderInvalid;
      context.reason = "perception did not establish a common lateral order";
      return context;
    }
    ordered.push_back(&boundary);
  }
  std::sort(ordered.begin(), ordered.end(), [](const auto* left,
                                               const auto* right) {
    return left->lateral_order < right->lateral_order;
  });
  const double shared_reference_x = ordered.front()->reference_x_m;
  for (std::size_t index = 0; index < ordered.size(); ++index) {
    if (ordered[index]->lateral_order != index) {
      context.status = ContextStatus::kBoundaryInvalid;
      context.reason = "lateral_order is not contiguous and zero-based";
      return context;
    }
    if (std::abs(ordered[index]->reference_x_m - shared_reference_x) > 1e-3) {
      context.status = ContextStatus::kBoundaryInvalid;
      context.reason = "boundaries do not share one lateral-order reference x";
      return context;
    }
    if (shared_reference_x < ordered[index]->valid_x_min_m - kEpsilon ||
        shared_reference_x > ordered[index]->valid_x_max_m + kEpsilon ||
        shared_reference_x < ordered[index]->polyline.front().x - kEpsilon ||
        shared_reference_x > ordered[index]->polyline.back().x + kEpsilon) {
      context.status = ContextStatus::kBoundaryInvalid;
      context.reason = "lateral-order reference lies outside boundary support";
      return context;
    }
    if (index > 0 &&
        ordered[index - 1]->reference_y_m <=
            ordered[index]->reference_y_m) {
      context.status = ContextStatus::kBoundaryInvalid;
      context.reason = "reference y does not follow left-to-right order";
      return context;
    }
  }

  for (std::size_t index = 0; index + 1 < ordered.size(); ++index) {
    LaneStripCandidate candidate;
    std::string pair_reason;
    if (BuildStrip(*ordered[index], *ordered[index + 1], index, config,
                   &candidate, &pair_reason)) {
      if (candidate.contains_vehicle_projection) {
        if (context.occupied_strip_index >= 0) {
          context.status = ContextStatus::kBoundaryInvalid;
          context.reason =
              "more than one strip contains the vehicle projection";
          context.strips.clear();
          context.occupied_strip_index = -1;
          return context;
        }
        context.occupied_strip_index =
            static_cast<int>(context.strips.size());
      }
      context.strips.push_back(std::move(candidate));
    } else {
      ++context.rejected_pair_count;
    }
  }

  if (context.strips.empty()) {
    if (config.require_white_yellow_pair &&
        config.allow_outer_white_pair_fallback && ordered.size() == 2) {
      std::vector<LaneStripCandidate> fallback;
      std::string fallback_reason;
      if (BuildOuterWhiteFallback(*ordered[0], *ordered[1], config,
                                  &fallback, &fallback_reason)) {
        context.strips = std::move(fallback);
        int occupied = -1;
        for (std::size_t index = 0; index < context.strips.size(); ++index) {
          if (!context.strips[index].contains_vehicle_projection) continue;
          occupied = occupied < 0 ? static_cast<int>(index) : -2;
        }
        context.occupied_strip_index = occupied >= 0 ? occupied : -1;
        context.status = ContextStatus::kDegradedInferredCenter;
        context.reason =
            "yellow boundary missing; two lane strips use an ODD-constrained "
            "center inferred from observed white outer boundaries";
        return context;
      }
    }
    context.status = ContextStatus::kNoUsableStrip;
    std::ostringstream stream;
    stream << "no adjacent boundary pair passed geometry/semantic gates; "
           << context.rejected_pair_count << " pair(s) rejected";
    context.reason = stream.str();
    return context;
  }
  context.status = ContextStatus::kValid;
  context.reason = "one or more lane strip candidates are usable";
  return context;
}

const char* ToString(ContextStatus status) {
  switch (status) {
    case ContextStatus::kValid:
      return "valid";
    case ContextStatus::kEmpty:
      return "empty";
    case ContextStatus::kArrayInvalid:
      return "array_invalid";
    case ContextStatus::kWrongFrame:
      return "wrong_frame";
    case ContextStatus::kWrongOperatingDomain:
      return "wrong_operating_domain";
    case ContextStatus::kWrongAlgorithm:
      return "wrong_algorithm";
    case ContextStatus::kOrderInvalid:
      return "order_invalid";
    case ContextStatus::kBoundaryInvalid:
      return "boundary_invalid";
    case ContextStatus::kNoUsableStrip:
      return "no_usable_strip";
    case ContextStatus::kDegradedInferredCenter:
      return "degraded_inferred_center";
    case ContextStatus::kCount:
      break;
  }
  return "unknown";
}

}  // namespace lane_boundary_adapter
