#include <algorithm>
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

#include <asmc_msgs/LaneBoundaryObservationArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "lane_boundary_adapter/lane_boundary_adapter.hpp"

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

geometry_msgs::Point RosPoint(const Point2d& point, double z) {
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

class LaneBoundaryMonitorNode {
 public:
  LaneBoundaryMonitorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
      : nh_(std::move(nh)), pnh_(std::move(pnh)) {
    std::string input_topic = "/perception/lane/boundary_observations";
    std::string marker_topic = "/planning/debug/camera_lane_strips";
    std::string diagnostics_topic =
        "/planning/lane_observation/diagnostics";
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
    int expected_boundary_count = config_.expected_boundary_count;
    pnh_.param("adapter/expected_boundary_count", expected_boundary_count,
               expected_boundary_count);
    if (expected_boundary_count < 1 || expected_boundary_count > 255) {
      throw std::runtime_error("expected_boundary_count must be in [1,255]");
    }
    config_.expected_boundary_count =
        static_cast<std::uint8_t>(expected_boundary_count);
    pnh_.param("adapter/reference_x_m", config_.reference_x_m,
               config_.reference_x_m);
    pnh_.param("adapter/consumer_x_min_m", config_.consumer_x_min_m,
               config_.consumer_x_min_m);
    pnh_.param("adapter/consumer_x_max_m", config_.consumer_x_max_m,
               config_.consumer_x_max_m);
    pnh_.param("adapter/sample_spacing_m", config_.sample_spacing_m,
               config_.sample_spacing_m);
    pnh_.param("adapter/min_common_length_m", config_.min_common_length_m,
               config_.min_common_length_m);
    pnh_.param("adapter/lane_width_min_m", config_.lane_width_min_m,
               config_.lane_width_min_m);
    pnh_.param("adapter/lane_width_max_m", config_.lane_width_max_m,
               config_.lane_width_max_m);
    pnh_.param("adapter/max_width_variation_m",
               config_.max_width_variation_m,
               config_.max_width_variation_m);
    pnh_.param("adapter/require_white_yellow_pair",
               config_.require_white_yellow_pair,
               config_.require_white_yellow_pair);
    pnh_.param("adapter/allow_outer_white_pair_fallback",
               config_.allow_outer_white_pair_fallback,
               config_.allow_outer_white_pair_fallback);
    pnh_.param("adapter/inferred_center_confidence_scale",
               config_.inferred_center_confidence_scale,
               config_.inferred_center_confidence_scale);
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

    std::string reason;
    if (!ValidateConfig(config_, &reason)) {
      throw std::runtime_error("invalid lane adapter config: " + reason);
    }
    if (receive_timeout_sec_ <= 0.0 || source_timeout_sec_ <= 0.0 ||
        future_tolerance_sec_ < 0.0 || monitor_rate_hz_ <= 0.0 ||
        marker_lifetime_sec_ < 0.0) {
      throw std::runtime_error("invalid freshness/monitor parameters");
    }

    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
        marker_topic, 1);
    diagnostics_publisher_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        diagnostics_topic, 1);
    subscriber_ = nh_.subscribe(
        input_topic, 1, &LaneBoundaryMonitorNode::Callback, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / monitor_rate_hz_),
                                 &LaneBoundaryMonitorNode::Timer, this);
    ROS_INFO_STREAM("Lane boundary consumer pilot: " << input_topic << " -> "
                    << marker_topic << ", " << diagnostics_topic
                    << "; no BehaviorCommand or Trajectory output");
  }

 private:
  static constexpr std::size_t kContextStatusCount =
      static_cast<std::size_t>(ContextStatus::kCount);
  using StatusCounts = std::array<std::uint64_t, kContextStatusCount>;

  struct State {
    bool received = false;
    bool source_time_usable = false;
    bool receive_stale = false;
    double source_age_sec = 0.0;
    ros::SteadyTime received_at;
    ros::Time source_stamp;
    std::string config_sha256;
    std::string algorithm_id;
    LaneStripContext context;
  };

  void Callback(
      const asmc_msgs::LaneBoundaryObservationArray::ConstPtr& message) {
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
    next.config_sha256 = message->config_sha256;
    next.algorithm_id = message->algorithm_id;
    if (source_usable) {
      next.context = BuildLaneStripContext(*message, config_);
    } else {
      next.context.status = ContextStatus::kArrayInvalid;
      next.context.reason = "source timestamp is zero, stale, or in the future";
      next.context.input_boundary_count = message->boundaries.size();
      next.context.input_completeness = message->completeness;
      next.context.expected_boundary_count = message->expected_boundary_count;
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
      unusable_messages = unusable_messages_;
      stale_events = stale_events_;
      source_epoch_resets = source_epoch_resets_;
      status_counts = status_counts_;
    }
    if (clear_stale_markers) PublishDeleteAll(ros::Time::now());
    PublishDiagnostics(snapshot, input_messages, usable_messages,
                       unusable_messages, stale_events, source_epoch_resets,
                       status_counts);
  }

  void PublishDeleteAll(const ros::Time& stamp) {
    visualization_msgs::MarkerArray array;
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = config_.expected_frame_id;
    AppendMissingMarkerDeletes(header, 0, &array);
    if (!array.markers.empty()) marker_publisher_.publish(array);
    if (published_strip_count_ > 0) ++marker_clear_events_;
    published_strip_count_ = 0;
  }

  void AppendMissingMarkerDeletes(
      const std_msgs::Header& header, std::size_t current_strip_count,
      visualization_msgs::MarkerArray* result) {
    static const std::array<const char*, 3> namespaces = {
        "camera_lane_strip_fill", "camera_lane_strip_centerline",
        "camera_lane_strip_text"};
    for (std::size_t index = current_strip_count;
         index < published_strip_count_; ++index) {
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

  void PublishMarkers(const std_msgs::Header& header,
                      const LaneStripContext& context) {
    visualization_msgs::MarkerArray result;
    if (!context.usable()) {
      AppendMissingMarkerDeletes(header, 0, &result);
      if (!result.markers.empty()) marker_publisher_.publish(result);
      if (published_strip_count_ > 0) ++marker_clear_events_;
      published_strip_count_ = 0;
      return;
    }

    for (std::size_t index = 0; index < context.strips.size(); ++index) {
      const auto& strip = context.strips[index];
      const bool occupied =
          static_cast<int>(index) == context.occupied_strip_index;
      const std_msgs::ColorRGBA strip_color =
          strip.inferred_center_boundary
              ? Color(1.0f, 0.55f, 0.05f, 0.12f)
              : (occupied ? Color(0.15f, 0.95f, 0.25f, 0.18f)
                          : Color(0.1f, 0.55f, 1.0f, 0.14f));

      visualization_msgs::Marker fill;
      fill.header = header;
      fill.ns = "camera_lane_strip_fill";
      fill.id = static_cast<int>(index);
      fill.type = visualization_msgs::Marker::TRIANGLE_LIST;
      fill.action = visualization_msgs::Marker::ADD;
      fill.pose.orientation.w = 1.0;
      fill.scale.x = fill.scale.y = fill.scale.z = 1.0;
      fill.color = strip_color;
      fill.lifetime = ros::Duration(marker_lifetime_sec_);
      for (std::size_t point = 0;
           point + 1 < strip.centerline.size(); ++point) {
        fill.points.push_back(RosPoint(strip.left_boundary[point], marker_z_m_));
        fill.points.push_back(RosPoint(strip.right_boundary[point], marker_z_m_));
        fill.points.push_back(
            RosPoint(strip.left_boundary[point + 1], marker_z_m_));
        fill.points.push_back(
            RosPoint(strip.left_boundary[point + 1], marker_z_m_));
        fill.points.push_back(RosPoint(strip.right_boundary[point], marker_z_m_));
        fill.points.push_back(
            RosPoint(strip.right_boundary[point + 1], marker_z_m_));
      }
      result.markers.push_back(fill);

      visualization_msgs::Marker centerline;
      centerline.header = header;
      centerline.ns = "camera_lane_strip_centerline";
      centerline.id = static_cast<int>(index);
      centerline.type = visualization_msgs::Marker::LINE_STRIP;
      centerline.action = visualization_msgs::Marker::ADD;
      centerline.pose.orientation.w = 1.0;
      centerline.scale.x = occupied ? 0.14 : 0.09;
      centerline.color = strip.inferred_center_boundary
                             ? Color(1.0f, 0.55f, 0.05f, 0.85f)
                             : (occupied
                                    ? Color(0.15f, 1.0f, 0.25f, 0.95f)
                                    : Color(0.15f, 0.7f, 1.0f, 0.85f));
      centerline.lifetime = ros::Duration(marker_lifetime_sec_);
      for (const auto& point : strip.centerline) {
        centerline.points.push_back(RosPoint(point, marker_z_m_ + 0.02));
      }
      result.markers.push_back(centerline);

      visualization_msgs::Marker text;
      text.header = header;
      text.ns = "camera_lane_strip_text";
      text.id = static_cast<int>(index);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.orientation.w = 1.0;
      text.pose.position = RosPoint(strip.centerline.front(), marker_z_m_ + 0.5);
      text.scale.z = 0.35;
      text.color = Color(1.0f, 1.0f, 1.0f, 0.95f);
      text.lifetime = ros::Duration(marker_lifetime_sec_);
      std::ostringstream label;
      label << "strip " << strip.lateral_index << " IDs "
            << strip.left_boundary_id << "/" << strip.right_boundary_id
            << " width " << std::fixed << std::setprecision(2)
            << strip.width_median_m << "m"
            << (strip.inferred_center_boundary ? " [DEGRADED inferred center]"
                                               : "")
            << (occupied ? " [vehicle projection]" : "");
      text.text = label.str();
      result.markers.push_back(text);
    }
    AppendMissingMarkerDeletes(header, context.strips.size(), &result);
    published_strip_count_ = context.strips.size();
    marker_publisher_.publish(result);
  }

  void PublishDiagnostics(const State& state, std::uint64_t input_messages,
                          std::uint64_t usable_messages,
                          std::uint64_t unusable_messages,
                          std::uint64_t stale_events,
                          std::uint64_t source_epoch_resets,
                          const StatusCounts& status_counts) {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lane_boundary_adapter/shadow_consumer";
    status.hardware_id = "camera_lane_boundary";
    const double receive_age =
        state.received
            ? (ros::SteadyTime::now() - state.received_at).toSec()
            : std::numeric_limits<double>::infinity();
    if (!state.received || state.receive_stale) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = state.received ? "input receive timeout" : "waiting for input";
    } else if (state.context.status == ContextStatus::kValid) {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = state.context.reason;
    } else if (state.context.status ==
               ContextStatus::kDegradedInferredCenter) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = state.context.reason;
    } else if (state.context.status == ContextStatus::kEmpty ||
               state.context.status == ContextStatus::kOrderInvalid ||
               state.context.status == ContextStatus::kNoUsableStrip) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = state.context.reason;
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = state.context.reason;
    }
    status.values = {
        Value("context_status", ToString(state.context.status)),
        Value("operating_domain", config_.expected_operating_domain),
        Value("algorithm_id", state.algorithm_id),
        Value("producer_config_sha256", state.config_sha256),
        Value("received", state.received),
        Value("receive_stale", state.receive_stale),
        Value("receive_age_sec", receive_age),
        Value("source_time_usable", state.source_time_usable),
        Value("source_age_sec", state.source_age_sec),
        Value("input_messages", input_messages),
        Value("usable_messages", usable_messages),
        Value("unusable_messages", unusable_messages),
        Value("input_boundary_count", state.context.input_boundary_count),
        Value("input_completeness",
              static_cast<int>(state.context.input_completeness)),
        Value("strip_count", state.context.strips.size()),
        Value("rejected_pair_count", state.context.rejected_pair_count),
        Value("occupied_strip_index", state.context.occupied_strip_index),
        Value("stale_events", stale_events),
        Value("source_epoch_resets", source_epoch_resets),
        Value("marker_policy", "explicit_delete_missing_ids"),
        Value("marker_delete_count", marker_delete_count_),
        Value("marker_clear_events", marker_clear_events_),
        Value("status_valid", status_counts[0]),
        Value("status_empty", status_counts[1]),
        Value("status_array_invalid", status_counts[2]),
        Value("status_wrong_frame", status_counts[3]),
        Value("status_wrong_operating_domain", status_counts[4]),
        Value("status_wrong_algorithm", status_counts[5]),
        Value("status_order_invalid", status_counts[6]),
        Value("status_boundary_invalid", status_counts[7]),
        Value("status_no_usable_strip", status_counts[8]),
        Value("status_degraded_inferred_center", status_counts[9]),
        Value("consumer_x_range_m",
              std::to_string(config_.consumer_x_min_m) + "," +
                  std::to_string(config_.consumer_x_max_m)),
        Value("lane_width_range_m",
              std::to_string(config_.lane_width_min_m) + "," +
                  std::to_string(config_.lane_width_max_m)),
        Value("publishes_control", false),
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

  AdapterConfig config_;
  double receive_timeout_sec_ = 0.30;
  double source_timeout_sec_ = 0.30;
  double future_tolerance_sec_ = 0.05;
  double monitor_rate_hz_ = 5.0;
  double marker_z_m_ = -0.32;
  double marker_lifetime_sec_ = 0.40;

  std::mutex mutex_;
  State state_;
  ros::Time last_source_stamp_;
  std::uint64_t input_messages_ = 0;
  std::uint64_t usable_messages_ = 0;
  std::uint64_t unusable_messages_ = 0;
  std::uint64_t stale_events_ = 0;
  std::uint64_t source_epoch_resets_ = 0;
  std::size_t published_strip_count_ = 0;
  std::uint64_t marker_delete_count_ = 0;
  std::uint64_t marker_clear_events_ = 0;
  StatusCounts status_counts_{};
};

}  // namespace lane_boundary_adapter

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_boundary_monitor");
  try {
    lane_boundary_adapter::LaneBoundaryMonitorNode node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Lane boundary monitor failed to start: %s", error.what());
    return 2;
  }
  return 0;
}
