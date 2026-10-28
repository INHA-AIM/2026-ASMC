#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <asmc_msgs/LidarPlanningObservation.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <planning_inputs/DetectedObjectArray.h>
#include <ros/ros.h>
#include "planning_inputs/converter.hpp"
#include "planning_inputs/lidar_observation_gate.hpp"

namespace planning_inputs {
// Perception owns geometry, filtering and motion estimation. This boundary
// checks source identity/time/validity and converts the accepted observation.
// TTC/follow/stop/avoid/sensor-failure actions remain planner-owned.
class ObjectsNode {
 public:
  ObjectsNode() : private_("~") {
    std::string input, output;
    private_.param<std::string>("input_topic", input, "/perception/lidar/planning_observations_map");
    private_.param<std::string>("output_topic", output, "/planning/objects");
    if (input == output) throw std::runtime_error("planning object topics must differ");
    private_.param("source_max_age_s", source_max_age_s_, .30);
    private_.param("source_future_tolerance_s", source_future_s_, .05);
    if (!std::isfinite(source_max_age_s_) || source_max_age_s_ <= 0 ||
        !std::isfinite(source_future_s_) || source_future_s_ < 0)
      throw std::runtime_error("invalid LiDAR source time limits");
    // Preserve the previous intersection of the configurable outer limit and
    // the .30/.05 contract gate, but perform one integer-ns check at one now.
    gate_ = LidarObservationGate(std::min(source_max_age_s_, .30),
                                std::min(source_future_s_, .05));
    publisher_ = node_.advertise<DetectedObjectArray>(output, 2);
    input_ = node_.subscribe<asmc_msgs::LidarPlanningObservation>(
        input, 1, &ObjectsNode::LidarObjects, this, ros::TransportHints().tcpNoDelay());
    diagnostics_ = private_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    diagnostics_timer_ = private_.createWallTimer(ros::WallDuration(1.), &ObjectsNode::Diagnostics, this);
    ROS_INFO("Planning LiDAR input: %s -> %s", input.c_str(), output.c_str());
  }
 private:
  void LidarObjects(const asmc_msgs::LidarPlanningObservation::ConstPtr& source) {
    if (!source) return;
    ++received_;
    std::string reason;
    DetectedObjectArray out;
    if (!gate_.Convert(*source, ros::Time::now(), &out, &reason)) {
      ++rejected_; source_reason_ = reason;
      ROS_WARN_THROTTLE(1., "Withheld common LiDAR input: %s", reason.c_str());
      return;  // invalid/pose-unavailable != an observed-empty free-space frame
    }
    publisher_.publish(out);
    source_reason_ = out.objects.empty() ? "observed_empty" : "observed";
    last_common_ = ros::SteadyTime::now();
    ++published_;
  }

  void Diagnostics(const ros::WallTimerEvent&) {
    diagnostic_msgs::DiagnosticArray message;
    message.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "planning_inputs/common_lidar";
    status.hardware_id = "asmc_lidar";
    const bool stale = last_common_.isZero() ||
        (ros::SteadyTime::now() - last_common_).toSec() > source_max_age_s_;
    status.level = stale || (source_reason_!="observed" && source_reason_!="observed_empty") ? status.WARN : status.OK;
    status.message = stale ? "no_fresh_representable_observation: " + source_reason_ : source_reason_;
    for (const auto& item : std::vector<std::pair<std::string, std::uint64_t>>{
        {"received",received_},{"published",published_},{"rejected",rejected_}}) {
      diagnostic_msgs::KeyValue value; value.key=item.first; value.value=std::to_string(item.second);
      status.values.push_back(std::move(value));
    }
    message.status.push_back(std::move(status));
    diagnostics_.publish(message);
  }
  ros::NodeHandle node_, private_;
  ros::Subscriber input_;
  ros::Publisher publisher_, diagnostics_;
  ros::WallTimer diagnostics_timer_;
  LidarObservationGate gate_;
  ros::SteadyTime last_common_;
  double source_max_age_s_ = .30, source_future_s_ = .05;
  std::string source_reason_ = "waiting_for_input";
  std::uint64_t received_ = 0, published_ = 0, rejected_ = 0;
};
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "planning_objects_adapter");
  try { planning_inputs::ObjectsNode node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
