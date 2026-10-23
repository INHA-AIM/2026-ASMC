#pragma once
// ROS message -> shadow pipeline input converters. Stateless field extraction
// from Main's Parser.cpp callbacks (8f93c22e); the conversion math lives in
// pipeline::ImuFromInput / CanFromInput / GpsIngest and the gates in
// LidarRuntime (AcceptScan, OnImu, OnCan, OnGps).
//
// Node/replay usage, in Main's callback order:
//   LiDAR: stamp = StampNs(header.stamp); cloud = CloudFromMsg(msg);
//          AcceptScan(stamp, cloud->size(), &reason) -> ScanInput{time.stamp_ns = stamp, cloud}
//   IMU/CAN/GPS: OnImu(ImuInputFromMsg(msg), receipt_ns, ros_now_ns) etc.

#include <asmc_lidar/geometry/pose_history.hpp>
#include <asmc_lidar/core/types.hpp>

#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <cstdint>

namespace asmc {
namespace lidar {
namespace runtime {

// Header stamp in ns (0 for a zero stamp; ROS stamps are never negative).
std::int64_t StampNs(const ros::Time& stamp);

// sensor_msgs/Imu: header stamp, orientation, linear_acceleration, angular_velocity.
pipeline::ImuInput ImuInputFromMsg(const sensor_msgs::Imu& msg);
// morai_msgs/EgoVehicleStatus: header stamp, velocity.x/y/z, wheel_angle (deg).
// Position/heading/accel are not used by Main.
pipeline::CanInput CanInputFromMsg(const morai_msgs::EgoVehicleStatus& msg);
// morai_msgs/GPSMessage: header stamp, latitude, longitude, altitude,
// eastOffset, northOffset, status.
pipeline::GpsInput GpsInputFromMsg(const morai_msgs::GPSMessage& msg);

// pcl::fromROSMsg into PointCloud<PointXYZI> exactly as Main's LiDAR callback
// (x, y, z, intensity; ring/time and other fields dropped; a missing field keeps
// the PCL default), then the cloud Main's LidarProcess rebuilds from the raw
// buffer (MakePointCloudFromRaw): same points in order, width = N, height = 1,
// is_dense = false, empty PCL header. A message without points gives an empty
// default cloud. Never null.
pipeline::CloudPtr CloudFromMsg(const sensor_msgs::PointCloud2& msg);

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
