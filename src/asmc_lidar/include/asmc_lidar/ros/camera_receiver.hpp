#pragma once
#include <asmc_lidar/fusion/camera_observations.hpp>
#include <asmc_msgs/CameraObjectObservation.h>
#include <asmc_msgs/InferenceStatus.h>
#include <ros/ros.h>

namespace asmc::lidar::camera {
Ns steady_now_ns();
Observation from_ros(const asmc_msgs::CameraObjectObservation&);
Status from_ros(const asmc_msgs::InferenceStatus&);
// Embed in the LiDAR node. Caller owns ROS spinning and fusion scheduling.
// Callbacks capture only the shared buffer, never a destructing Receiver.
class Receiver {
 public:
  Receiver(ros::NodeHandle&, Calibrations, Limits limits={},
           const std::string& observations="/perception/camera/objects/observations",
           const std::string& status="/perception/camera/objects/status");
  ~Receiver();
  Receiver(const Receiver&)=delete;
  Receiver& operator=(const Receiver&)=delete;
  std::shared_ptr<Buffer> buffer() const {return buffer_;}
  void close();
 private:
  std::shared_ptr<Buffer> buffer_;
  ros::Subscriber observations_, status_;
};
} // namespace asmc::lidar::camera
