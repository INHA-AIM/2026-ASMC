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
// Previous tracking frame -> current one. AlignTrackingState moves only the
// filter state copied from the previous scan (KF state/covariance, height,
// frame record); use it when the OBB already is this scan's detection.
// AlignTrackingFrame moves a whole previous-scan track: that state plus its
// OBB center, corners and yaw, all with the same single frame change (the box
// is re-expressed, not motion-predicted).
void AlignTrackingState(LiDARObjectInfo& object, double yaw_now_rad);
void AlignTrackingFrame(LiDARObjectInfo& object, double yaw_now_rad);
void StoreLeveling(const asmc::lidar::Rot& rotation);
}  // namespace frame_boundary
