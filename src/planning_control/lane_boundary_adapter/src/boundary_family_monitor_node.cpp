#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <asmc_msgs/LaneBoundaryFamily.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "lane_boundary_adapter/boundary_family_adapter.hpp"

namespace lane_boundary_adapter {
namespace {

diagnostic_msgs::KeyValue Value(const std::string& key,
                                const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

template <typename T>
diagnostic_msgs::KeyValue Value(const std::string& key, const T& value) {
  std::ostringstream stream;
  stream << value;
  return Value(key, stream.str());
}

geometry_msgs::Point RosPoint(const FamilyPoint2d& point, double z) {
  geometry_msgs::Point result;
  result.x = point.x;
  result.y = point.y;
  result.z = z;
  return result;
}

std_msgs::ColorRGBA Color(float red, float green, float blue, float alpha) {
  std_msgs::ColorRGBA result;
  result.r = red;
  result.g = green;
  result.b = blue;
  result.a = alpha;
  return result;
}

}  // namespace

class BoundaryFamilyMonitorNode {
 public:
  BoundaryFamilyMonitorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
      : nh_(std::move(nh)), pnh_(std::move(pnh)) {
    std::string input_topic = "/perception/lane/boundary_family";
    std::string marker_topic =
        "/planning/debug/lane_boundary_family/strips";
    std::string diagnostics_topic =
        "/planning/diagnostics/lane_boundary_family_adapter";
    pnh_.param("topics/input", input_topic, input_topic);
    pnh_.param("topics/markers", marker_topic, marker_topic);
    pnh_.param("topics/diagnostics", diagnostics_topic, diagnostics_topic);

    pnh_.param("adapter/expected_frame_id", config_.expected_frame_id,
               config_.expected_frame_id);
    pnh_.param("adapter/expected_operating_domain",
               config_.expected_operating_domain,
               config_.expected_operating_domain);
    pnh_.param("adapter/expected_algorithm_id", config_.expected_algorithm_id,
               config_.expected_algorithm_id);
    pnh_.param("adapter/expected_representation",
               config_.expected_representation,
               config_.expected_representation);
    int expected_boundary_count = config_.expected_boundary_count;
    pnh_.param("adapter/expected_boundary_count", expected_boundary_count,
               expected_boundary_count);
    if (expected_boundary_count < 1 || expected_boundary_count > 255) {
      throw std::runtime_error("expected_boundary_count must be in [1,255]");
    }
    config_.expected_boundary_count =
        static_cast<std::uint8_t>(expected_boundary_count);
    pnh_.param("adapter/consumer_x_min_m", config_.consumer_x_min_m,
               config_.consumer_x_min_m);
    pnh_.param("adapter/consumer_x_max_m", config_.consumer_x_max_m,
               config_.consumer_x_max_m);
    pnh_.param("adapter/min_segment_length_m", config_.min_segment_length_m,
               config_.min_segment_length_m);
    pnh_.param("adapter/lane_width_min_m", config_.lane_width_min_m,
               config_.lane_width_min_m);
    pnh_.param("adapter/lane_width_max_m", config_.lane_width_max_m,
               config_.lane_width_max_m);
    pnh_.param("adapter/max_width_variation_m",
               config_.max_width_variation_m,
               config_.max_width_variation_m);
    pnh_.param("adapter/inferred_confidence_scale",
               config_.inferred_confidence_scale,
               config_.inferred_confidence_scale);
    pnh_.param("adapter/require_deployment_disallowed",
               config_.require_deployment_disallowed,
               config_.require_deployment_disallowed);
    pnh_.param("freshness/receive_timeout_sec", receive_timeout_sec_,
               receive_timeout_sec_);
    pnh_.param("freshness/source_timeout_sec", source_timeout_sec_,
               source_timeout_sec_);
    pnh_.param("freshness/future_tolerance_sec", future_tolerance_sec_,
               future_tolerance_sec_);
    pnh_.param("monitor_rate_hz", monitor_rate_hz_, monitor_rate_hz_);
    pnh_.param("visualization/marker_z_m", marker_z_m_, marker_z_m_);
    pnh_.param("visualization/marker_lifetime_sec", marker_lifetime_sec_,
               marker_lifetime_sec_);
    pnh_.param("visualization/vehicle_length_m", vehicle_length_m_,
               vehicle_length_m_);
    pnh_.param("visualization/vehicle_width_m", vehicle_width_m_,
               vehicle_width_m_);
    pnh_.param("visualization/vehicle_rear_overhang_m",
               vehicle_rear_overhang_m_, vehicle_rear_overhang_m_);
    pnh_.param("visualization/vehicle_wheelbase_m", vehicle_wheelbase_m_,
               vehicle_wheelbase_m_);
    pnh_.param("visualization/vehicle_marker_height_m",
               vehicle_marker_height_m_, vehicle_marker_height_m_);

    std::string reason;
    if (!ValidateBoundaryFamilyAdapterConfig(config_, &reason)) {
      throw std::runtime_error("invalid family adapter config: " + reason);
    }
    if (receive_timeout_sec_ <= 0.0 || source_timeout_sec_ <= 0.0 ||
        future_tolerance_sec_ < 0.0 || monitor_rate_hz_ <= 0.0 ||
        marker_lifetime_sec_ < 0.0 || vehicle_length_m_ <= 0.0 ||
        vehicle_width_m_ <= 0.0 || vehicle_rear_overhang_m_ < 0.0 ||
        vehicle_rear_overhang_m_ >= vehicle_length_m_ ||
        vehicle_wheelbase_m_ <= 0.0 ||
        vehicle_wheelbase_m_ >
            vehicle_length_m_ - vehicle_rear_overhang_m_ ||
        vehicle_marker_height_m_ <= 0.0) {
      throw std::runtime_error("invalid freshness/monitor parameters");
    }

    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
        marker_topic, 1);
    diagnostics_publisher_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        diagnostics_topic, 1);
    subscriber_ = nh_.subscribe(
        input_topic, 1, &BoundaryFamilyMonitorNode::Callback, this);
    timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / monitor_rate_hz_),
        &BoundaryFamilyMonitorNode::Timer, this);
    ROS_INFO_STREAM("Read-only boundary-family candidate adapter: "
                    << input_topic << " -> " << marker_topic << ", "
                    << diagnostics_topic
                    << "; inferred is degraded, invalid is never bridged, "
                       "and no planning/control topic is published");
  }

 private:
  static constexpr std::size_t kStatusCount =
      static_cast<std::size_t>(FamilyContextStatus::kCount);
  using StatusCounts = std::array<std::uint64_t, kStatusCount>;

  struct State {
    bool received = false;
    bool source_time_usable = false;
    bool receive_stale = false;
    double source_age_sec = std::numeric_limits<double>::infinity();
    ros::SteadyTime received_at;
    ros::Time source_stamp;
    std::string algorithm_id;
    std::string representation;
    bool deployment_allowed = false;
    BoundaryFamilyStripContext context;
  };

  void Callback(const asmc_msgs::LaneBoundaryFamily::ConstPtr& message) {
    if (!message) return;
    const ros::Time now = ros::Time::now();
    const double source_age =
        message->header.stamp.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - message->header.stamp).toSec();
    const bool source_usable =
        std::isfinite(source_age) && source_age >= -future_tolerance_sec_ &&
        source_age <= source_timeout_sec_;

    State next;
    next.received = true;
    next.received_at = ros::SteadyTime::now();
    next.source_stamp = message->header.stamp;
    next.source_age_sec = source_age;
    next.source_time_usable = source_usable;
    next.algorithm_id = message->algorithm_id;
    next.representation = message->representation;
    next.deployment_allowed = message->deployment_allowed;
    if (source_usable) {
      next.context = BuildBoundaryFamilyStripContext(*message, config_);
    } else {
      next.context.status = FamilyContextStatus::kSourceTimestampInvalid;
      next.context.reason = "source timestamp is zero, stale, or in the future";
      next.context.input_boundary_count = message->boundaries.size();
      next.context.source_config_sha256 = message->config_sha256;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!last_source_stamp_.isZero() &&
          message->header.stamp + ros::Duration(0.1) < last_source_stamp_) {
        ++source_epoch_resets_;
      }
      last_source_stamp_ = message->header.stamp;
      state_ = next;
      ++input_messages_;
      if (next.context.usable()) {
        ++usable_messages_;
        if (next.context.status == FamilyContextStatus::kDegradedUsable) {
          ++degraded_messages_;
        }
      } else {
        ++unusable_messages_;
      }
      ++status_counts_.at(static_cast<std::size_t>(next.context.status));
    }
    PublishMarkers(message->header, next.context);
  }

  void Timer(const ros::WallTimerEvent&) {
    State snapshot;
    std::uint64_t input_messages = 0;
    std::uint64_t usable_messages = 0;
    std::uint64_t degraded_messages = 0;
    std::uint64_t unusable_messages = 0;
    std::uint64_t stale_events = 0;
    std::uint64_t source_epoch_resets = 0;
    StatusCounts status_counts{};
    bool clear_stale_markers = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot = state_;
      if (snapshot.received) {
        const double receive_age =
            (ros::SteadyTime::now() - snapshot.received_at).toSec();
        if (receive_age > receive_timeout_sec_ && !state_.receive_stale) {
          state_.receive_stale = true;
          snapshot.receive_stale = true;
          ++stale_events_;
          clear_stale_markers = true;
        } else {
          snapshot.receive_stale = state_.receive_stale;
        }
      }
      input_messages = input_messages_;
      usable_messages = usable_messages_;
      degraded_messages = degraded_messages_;
      unusable_messages = unusable_messages_;
      stale_events = stale_events_;
      source_epoch_resets = source_epoch_resets_;
      status_counts = status_counts_;
    }
    if (clear_stale_markers) PublishDeleteAll(ros::Time::now());
    PublishDiagnostics(snapshot, input_messages, usable_messages,
                       degraded_messages, unusable_messages, stale_events,
                       source_epoch_resets, status_counts);
  }

  void AppendMissingMarkerDeletes(const std_msgs::Header& header,
                                  std::size_t current_count,
                                  visualization_msgs::MarkerArray* result) {
    static const std::array<const char*, 3> namespaces = {
        "family_strip_fill", "family_strip_centerline", "family_strip_text"};
    for (std::size_t index = current_count; index < published_segment_count_;
         ++index) {
      for (const char* marker_namespace : namespaces) {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = marker_namespace;
        marker.id = static_cast<int>(index);
        marker.action = visualization_msgs::Marker::DELETE;
        result->markers.push_back(marker);
        ++marker_delete_count_;
      }
    }
  }

  void PublishDeleteAll(const ros::Time& stamp) {
    visualization_msgs::MarkerArray result;
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = config_.expected_frame_id;
    AppendVehicleMarkers(header, &result);
    AppendMissingMarkerDeletes(header, 0, &result);
    if (!result.markers.empty()) marker_publisher_.publish(result);
    if (published_segment_count_ > 0) ++marker_clear_events_;
    published_segment_count_ = 0;
  }

  void AppendVehicleMarkers(const std_msgs::Header& header,
                            visualization_msgs::MarkerArray* result) const {
    const double rear_x = -vehicle_rear_overhang_m_;
    const double front_x = vehicle_length_m_ - vehicle_rear_overhang_m_;
    const double half_width = 0.5 * vehicle_width_m_;
    const double body_center_x = 0.5 * (rear_x + front_x);
    const double body_z = marker_z_m_ + 0.5 * vehicle_marker_height_m_;

    visualization_msgs::Marker body;
    body.header = header;
    body.ns = "ego_vehicle_body";
    body.id = 0;
    body.type = visualization_msgs::Marker::CUBE;
    body.action = visualization_msgs::Marker::ADD;
    body.pose.position.x = body_center_x;
    body.pose.position.y = 0.0;
    body.pose.position.z = body_z;
    body.pose.orientation.w = 1.0;
    body.scale.x = vehicle_length_m_;
    body.scale.y = vehicle_width_m_;
    body.scale.z = vehicle_marker_height_m_;
    body.color = Color(0.15f, 0.55f, 0.95f, 0.28f);
    result->markers.push_back(body);

    visualization_msgs::Marker outline;
    outline.header = header;
    outline.ns = "ego_vehicle_outline";
    outline.id = 0;
    outline.type = visualization_msgs::Marker::LINE_STRIP;
    outline.action = visualization_msgs::Marker::ADD;
    outline.pose.orientation.w = 1.0;
    outline.scale.x = 0.08;
    outline.color = Color(0.25f, 0.85f, 1.0f, 0.95f);
    outline.points = {
        RosPoint({rear_x, half_width}, marker_z_m_ + 0.02),
        RosPoint({front_x, half_width}, marker_z_m_ + 0.02),
        RosPoint({front_x, -half_width}, marker_z_m_ + 0.02),
        RosPoint({rear_x, -half_width}, marker_z_m_ + 0.02),
        RosPoint({rear_x, half_width}, marker_z_m_ + 0.02),
    };
    result->markers.push_back(outline);

    visualization_msgs::Marker axles;
    axles.header = header;
    axles.ns = "ego_vehicle_axles";
    axles.id = 0;
    axles.type = visualization_msgs::Marker::LINE_LIST;
    axles.action = visualization_msgs::Marker::ADD;
    axles.pose.orientation.w = 1.0;
    axles.scale.x = 0.10;
    axles.points = {
        RosPoint({0.0, half_width}, marker_z_m_ + 0.04),
        RosPoint({0.0, -half_width}, marker_z_m_ + 0.04),
        RosPoint({vehicle_wheelbase_m_, half_width}, marker_z_m_ + 0.04),
        RosPoint({vehicle_wheelbase_m_, -half_width}, marker_z_m_ + 0.04),
    };
    axles.colors = {
        Color(1.0f, 0.35f, 0.10f, 1.0f),
        Color(1.0f, 0.35f, 0.10f, 1.0f),
        Color(0.25f, 1.0f, 0.35f, 1.0f),
        Color(0.25f, 1.0f, 0.35f, 1.0f),
    };
    result->markers.push_back(axles);

    visualization_msgs::Marker origin;
    origin.header = header;
    origin.ns = "ego_vehicle_base_link";
    origin.id = 0;
    origin.type = visualization_msgs::Marker::SPHERE;
    origin.action = visualization_msgs::Marker::ADD;
    origin.pose.position.x = 0.0;
    origin.pose.position.y = 0.0;
    origin.pose.position.z = marker_z_m_ + 0.10;
    origin.pose.orientation.w = 1.0;
    origin.scale.x = 0.22;
    origin.scale.y = 0.22;
    origin.scale.z = 0.22;
    origin.color = Color(1.0f, 0.18f, 0.10f, 1.0f);
    result->markers.push_back(origin);
  }

  void PublishMarkers(const std_msgs::Header& header,
                      const BoundaryFamilyStripContext& context) {
    visualization_msgs::MarkerArray result;
    AppendVehicleMarkers(header, &result);
    if (!context.usable()) {
      AppendMissingMarkerDeletes(header, 0, &result);
      if (!result.markers.empty()) marker_publisher_.publish(result);
      if (published_segment_count_ > 0) ++marker_clear_events_;
      published_segment_count_ = 0;
      return;
    }

    for (std::size_t index = 0; index < context.segments.size(); ++index) {
      const FamilyLaneStripSegment& segment = context.segments[index];
      const bool degraded =
          segment.support == SegmentSupport::kStructureInferred;
      const std_msgs::ColorRGBA fill_color =
          degraded ? Color(1.0f, 0.45f, 0.02f, 0.16f)
                   : Color(0.12f, 0.72f, 1.0f, 0.14f);
      const std_msgs::ColorRGBA line_color =
          degraded ? Color(1.0f, 0.45f, 0.02f, 0.95f)
                   : Color(0.15f, 0.85f, 1.0f, 0.95f);

      visualization_msgs::Marker fill;
      fill.header = header;
      fill.ns = "family_strip_fill";
      fill.id = static_cast<int>(index);
      fill.type = visualization_msgs::Marker::TRIANGLE_LIST;
      fill.action = visualization_msgs::Marker::ADD;
      fill.pose.orientation.w = 1.0;
      fill.scale.x = fill.scale.y = fill.scale.z = 1.0;
      fill.color = fill_color;
      fill.lifetime = ros::Duration(marker_lifetime_sec_);
      for (std::size_t point = 0;
           point + 1 < segment.centerline.size(); ++point) {
        fill.points.push_back(
            RosPoint(segment.left_boundary[point], marker_z_m_));
        fill.points.push_back(
            RosPoint(segment.right_boundary[point], marker_z_m_));
        fill.points.push_back(
            RosPoint(segment.left_boundary[point + 1], marker_z_m_));
        fill.points.push_back(
            RosPoint(segment.left_boundary[point + 1], marker_z_m_));
        fill.points.push_back(
            RosPoint(segment.right_boundary[point], marker_z_m_));
        fill.points.push_back(
            RosPoint(segment.right_boundary[point + 1], marker_z_m_));
      }
      result.markers.push_back(fill);

      visualization_msgs::Marker centerline;
      centerline.header = header;
      centerline.ns = "family_strip_centerline";
      centerline.id = static_cast<int>(index);
      centerline.type = visualization_msgs::Marker::LINE_STRIP;
      centerline.action = visualization_msgs::Marker::ADD;
      centerline.pose.orientation.w = 1.0;
      centerline.scale.x = degraded ? 0.10 : 0.12;
      centerline.color = line_color;
      centerline.lifetime = ros::Duration(marker_lifetime_sec_);
      for (const FamilyPoint2d& point : segment.centerline) {
        centerline.points.push_back(RosPoint(point, marker_z_m_ + 0.02));
      }
      result.markers.push_back(centerline);

      visualization_msgs::Marker text;
      text.header = header;
      text.ns = "family_strip_text";
      text.id = static_cast<int>(index);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.orientation.w = 1.0;
      text.pose.position =
          RosPoint(segment.centerline.front(), marker_z_m_ + 0.45);
      text.scale.z = 0.30;
      text.color = Color(1.0f, 1.0f, 1.0f, 0.95f);
      text.lifetime = ros::Duration(marker_lifetime_sec_);
      std::ostringstream label;
      label << "strip " << segment.lateral_index << "/segment "
            << segment.segment_index << " " << ToString(segment.support)
            << " confidence " << std::fixed << std::setprecision(2)
            << segment.effective_confidence << " width "
            << segment.width_median_m << "m";
      text.text = label.str();
      result.markers.push_back(text);
    }
    AppendMissingMarkerDeletes(header, context.segments.size(), &result);
    published_segment_count_ = context.segments.size();
    marker_publisher_.publish(result);
  }

  void PublishDiagnostics(const State& state, std::uint64_t input_messages,
                          std::uint64_t usable_messages,
                          std::uint64_t degraded_messages,
                          std::uint64_t unusable_messages,
                          std::uint64_t stale_events,
                          std::uint64_t source_epoch_resets,
                          const StatusCounts& status_counts) {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lane_boundary_adapter/boundary_family_candidate";
    status.hardware_id = "camera_lane_boundary_family";
    const double receive_age =
        state.received
            ? (ros::SteadyTime::now() - state.received_at).toSec()
            : std::numeric_limits<double>::infinity();
    if (!state.received || state.receive_stale) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = state.received ? "input receive timeout"
                                      : "waiting for candidate input";
    } else if (state.context.status == FamilyContextStatus::kObservedOnly) {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = state.context.reason;
    } else if (state.context.status ==
                   FamilyContextStatus::kDegradedUsable ||
               state.context.status == FamilyContextStatus::kFreshInvalid ||
               state.context.status ==
                   FamilyContextStatus::kSourceTimestampInvalid ||
               state.context.status == FamilyContextStatus::kNoUsableSegment) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = state.context.reason;
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = state.context.reason;
    }

    status.values = {
        Value("context_status", ToString(state.context.status)),
        Value("read_only", true),
        Value("publishes_control", false),
        Value("deployment_allowed", state.deployment_allowed),
        Value("operating_domain", config_.expected_operating_domain),
        Value("algorithm_id", state.algorithm_id),
        Value("representation", state.representation),
        Value("producer_config_sha256", state.context.source_config_sha256),
        Value("received", state.received),
        Value("receive_stale", state.receive_stale),
        Value("receive_age_sec", receive_age),
        Value("source_time_usable", state.source_time_usable),
        Value("source_age_sec", state.source_age_sec),
        Value("input_messages", input_messages),
        Value("usable_messages", usable_messages),
        Value("degraded_messages", degraded_messages),
        Value("unusable_messages", unusable_messages),
        Value("input_boundary_count", state.context.input_boundary_count),
        Value("strip_segment_count", state.context.segments.size()),
        Value("observed_segment_count", state.context.observed_segment_count),
        Value("inferred_segment_count", state.context.inferred_segment_count),
        Value("invalid_pair_interval_count",
              state.context.invalid_interval_count),
        Value("observed_length_m", state.context.observed_length_m),
        Value("inferred_length_m", state.context.inferred_length_m),
        Value("stale_events", stale_events),
        Value("source_epoch_resets", source_epoch_resets),
        Value("marker_policy", "split_at_invalid_explicit_delete"),
        Value("marker_delete_count", marker_delete_count_),
        Value("marker_clear_events", marker_clear_events_),
        Value("status_observed_only", status_counts[0]),
        Value("status_degraded_usable", status_counts[1]),
        Value("status_fresh_invalid", status_counts[2]),
        Value("status_source_timestamp_invalid", status_counts[3]),
        Value("status_wrong_frame", status_counts[4]),
        Value("status_wrong_operating_domain", status_counts[5]),
        Value("status_wrong_algorithm", status_counts[6]),
        Value("status_wrong_representation", status_counts[7]),
        Value("status_wrong_deployment_flag", status_counts[8]),
        Value("status_malformed_family", status_counts[9]),
        Value("status_no_usable_segment", status_counts[10]),
        Value("consumer_x_range_m",
              std::to_string(config_.consumer_x_min_m) + "," +
                  std::to_string(config_.consumer_x_max_m)),
        Value("inferred_policy", "usable_degraded_never_bridges_invalid"),
        Value("inferred_confidence_scale",
              config_.inferred_confidence_scale),
    };
    diagnostic_msgs::DiagnosticArray message;
    message.header.stamp = ros::Time::now();
    message.status.push_back(status);
    diagnostics_publisher_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber subscriber_;
  ros::Publisher marker_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::WallTimer timer_;

  BoundaryFamilyAdapterConfig config_;
  double receive_timeout_sec_ = 0.35;
  double source_timeout_sec_ = 0.35;
  double future_tolerance_sec_ = 0.05;
  double monitor_rate_hz_ = 10.0;
  double marker_z_m_ = -0.31;
  double marker_lifetime_sec_ = 0.0;
  double vehicle_length_m_ = 4.635;
  double vehicle_width_m_ = 1.892;
  double vehicle_rear_overhang_m_ = 0.790;
  double vehicle_wheelbase_m_ = 3.000;
  double vehicle_marker_height_m_ = 0.12;

  std::mutex mutex_;
  State state_;
  ros::Time last_source_stamp_;
  std::uint64_t input_messages_ = 0;
  std::uint64_t usable_messages_ = 0;
  std::uint64_t degraded_messages_ = 0;
  std::uint64_t unusable_messages_ = 0;
  std::uint64_t stale_events_ = 0;
  std::uint64_t source_epoch_resets_ = 0;
  std::size_t published_segment_count_ = 0;
  std::uint64_t marker_delete_count_ = 0;
  std::uint64_t marker_clear_events_ = 0;
  StatusCounts status_counts_{};
};

}  // namespace lane_boundary_adapter

int main(int argc, char** argv) {
  ros::init(argc, argv, "boundary_family_monitor");
  try {
    lane_boundary_adapter::BoundaryFamilyMonitorNode node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Boundary family monitor failed to start: %s", error.what());
    return 2;
  }
  return 0;
}
