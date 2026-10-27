// ROS message -> shadow pipeline input converters (see ros_inputs.hpp).
#include <asmc_lidar/ros/ros_inputs.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace asmc {
namespace lidar {
namespace runtime {

std::int64_t StampNs(const ros::Time& stamp) {
  return static_cast<std::int64_t>(stamp.toNSec());
}

// Ported from Main src/Parser/Parser.cpp:566-607 (8f93c22e): message fields only.
pipeline::ImuInput ImuInputFromMsg(const sensor_msgs::Imu& msg) {
  pipeline::ImuInput in;
  in.stamp_ns = msg.header.stamp.toNSec();
  in.qx = msg.orientation.x; in.qy = msg.orientation.y;
  in.qz = msg.orientation.z; in.qw = msg.orientation.w;
  in.ax = msg.linear_acceleration.x; in.ay = msg.linear_acceleration.y; in.az = msg.linear_acceleration.z;
  in.wx = msg.angular_velocity.x; in.wy = msg.angular_velocity.y; in.wz = msg.angular_velocity.z;
  return in;
}

// Ported from Main src/Parser/Parser.cpp:519-538 (8f93c22e): message fields only.
pipeline::CanInput CanInputFromMsg(const morai_msgs::EgoVehicleStatus& msg) {
  pipeline::CanInput in;
  in.stamp_ns = msg.header.stamp.toNSec();
  in.vx = msg.velocity.x; in.vy = msg.velocity.y; in.vz = msg.velocity.z;
  in.wheel_angle_deg = msg.wheel_angle;  // float32 in the message, exact in double
  return in;
}

// Ported from Main src/Parser/Parser.cpp:398-421 (8f93c22e): message fields only.
pipeline::GpsInput GpsInputFromMsg(const morai_msgs::GPSMessage& msg) {
  pipeline::GpsInput in;
  in.stamp_ns = msg.header.stamp.toNSec();
  in.latitude = msg.latitude; in.longitude = msg.longitude; in.altitude = msg.altitude;
  in.east_offset = msg.eastOffset; in.north_offset = msg.northOffset;
  in.status = msg.status;
  return in;
}

// Ported from Main src/Parser/Parser.cpp:278-279,288-331 (fromROSMsg + raw
// buffer copy) and src/Lidar/LidarProcess/LidarProcess.cpp:22-50
// (MakePointCloudFromRaw) (8f93c22e). The raw buffer memcpy of whole PointXYZI
// records round-trips the points bit-exactly. Decode into the final owned buffer.
pipeline::CloudPtr CloudFromMsg(const sensor_msgs::PointCloud2& msg) {
  pipeline::CloudPtr out(new pipeline::Cloud());
  pcl::fromROSMsg(msg, *out);  // Decode directly into the owned buffer; no second vector copy.
  out->header = pcl::PCLHeader{};  // Public stamp/frame live in ScanInput, not PCL internal metadata.
  if (out->points.empty()) {
    out->width = out->height = 0;
    out->is_dense = true;  // Preserve the original empty Cloud metadata.
    return out;
  }
  out->width = static_cast<std::uint32_t>(out->points.size());
  out->height = 1;
  out->is_dense = false;
  return out;
}

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
