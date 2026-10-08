#pragma once
#include <Global/Global.hpp>
#include <asmc_lidar/projection.hpp>

namespace frame_boundary {
bool Enabled();
bool UseRoll();
double MapAltitudeOrigin();
bool MapOffsetsValid(double east,double north);
const std::string& CalibrationId();
void Configure(ros::NodeHandle& node, const VehicleSpec& vehicle, const CAMERA& camera);
asmc::lidar::Pose SensorMount(const VehicleSpec& vehicle);
asmc::lidar::Rot MapRotation(const GPS_IMU& pose);
void SetCurrent(const asmc::lidar::Context& context);
const asmc::lidar::Context& Current();
void PrepareScan(const ros::Time& stamp, const GPS_IMU& local,
                 const VehicleSpec& vehicle, const asmc::lidar::PoseSelection& map_pose, std::uint64_t input_generation = 0);
void PrepareScan(const asmc::lidar::Observation& time, const GPS_IMU& local,
                 const VehicleSpec& vehicle, const asmc::lidar::PoseSelection& map_pose,
                 std::uint64_t input_generation);
asmc::lidar::Observation ScanTime(const RAW_LIDAR_DATA& raw);
ros::Time SourceStamp(const RAW_LIDAR_DATA& raw);
bool LegacyMotionReady(const LiDAR& lidar);
void RecordTrackingFrame(LiDARObjectInfo& object);
void AlignTrackingFrame(LiDARObjectInfo& object, double yaw_now_rad);
void StoreLeveling(const asmc::lidar::Rot& rotation);
}  // namespace frame_boundary
