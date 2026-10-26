#include <planning_inputs/lidar_observation_gate.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

class LidarObservationNode {
 public:
  LidarObservationNode() : private_("~") {
    std::string input, output;
    private_.param<std::string>("input_topic", input, "/migration/lidar/planning_observations_map");
    private_.param<std::string>("output_topic", output, "/planning/objects");
    if (node_.resolveName(input) == node_.resolveName(output)) throw std::runtime_error("input equals output");
    publisher_ = node_.advertise<planning_inputs::DetectedObjectArray>(output, 2);
    diagnostics_ = private_.advertise<std_msgs::String>("diagnostics", 1, true);
    subscriber_ = node_.subscribe(input, 2, &LidarObservationNode::OnObservation, this);
    timer_ = node_.createWallTimer(ros::WallDuration(.2), &LidarObservationNode::Status, this);
  }
 private:
  void OnObservation(const asmc_msgs::LidarPlanningObservation::ConstPtr& packet) {
    planning_inputs::DetectedObjectArray output;
    received_ = ros::SteadyTime::now();
    if (gate_.Convert(*packet, ros::Time::now().toSec(), &output, &reason_)) {
      ++accepted_; publisher_.publish(output);
    } else { ++rejected_; ROS_WARN_STREAM_THROTTLE(1., "asmc_lidar planning input withheld: " << reason_); }
  }
  void Status(const ros::WallTimerEvent&) {
    const bool stale = received_.isZero() || (ros::SteadyTime::now() - received_).toSec() > .3;
    std_msgs::String message;
    message.data = "source=asmc_lidar status=" + (stale ? std::string("input_stale") : reason_) +
        " accepted=" + std::to_string(accepted_) + " rejected=" + std::to_string(rejected_);
    diagnostics_.publish(message);
  }
  ros::NodeHandle node_, private_;
  ros::Publisher publisher_, diagnostics_; ros::Subscriber subscriber_; ros::WallTimer timer_;
  ros::SteadyTime received_;
  planning_inputs::LidarObservationGate gate_;
  std::string reason_ = "waiting_for_input";
  uint64_t accepted_ = 0, rejected_ = 0;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "asmc_lidar_planning_adapter");
  try { LidarObservationNode node; ros::spin(); }
  catch (const std::exception& error) { ROS_FATAL("%s", error.what()); return 1; }
}
