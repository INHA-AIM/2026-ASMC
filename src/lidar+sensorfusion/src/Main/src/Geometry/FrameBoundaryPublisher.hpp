#pragma once
#include <Geometry/FrameBoundary.hpp>
#include <asmc_msgs/LidarObjectArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <mutex>
#include <tf2_ros/transform_broadcaster.h>

namespace frame_boundary {
class Publisher {
 public:
  explicit Publisher(ros::NodeHandle& node);
  void Publish(const LiDAR& lidar);
  void SetLegacyReason(const std::string& reason);
  void Invalid(const ros::Time& stamp,const std::string& reason);
 private:
  bool Accept(const ros::Time& stamp);
  void Diagnostics(const ros::WallTimerEvent&);
  asmc::lidar::Context last_context_;
  std::recursive_mutex mutex_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> leveling_tf_;
  ros::Publisher local_,map_,diagnostics_;
  ros::WallTimer timer_;
  ros::Time last_stamp_;
  ros::WallTime last_received_;
  std::string input_reason_="waiting_for_input",map_reason_="gps_missing",legacy_reason_="waiting_for_input";
  std::uint64_t processed_=0,invalid_=0,stamp_errors_=0;
  double last_serialization_ms_=0;
};
}  // namespace frame_boundary
