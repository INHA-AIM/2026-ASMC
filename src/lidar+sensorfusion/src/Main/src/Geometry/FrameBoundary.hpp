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
// Re-expresses the previous scan's KF state in the current scan frame exactly once.
// rotate_obb_yaw=true: object's f32_OBB_YawRad is a previous-scan value (aligned copy, missed track).
// rotate_obb_yaw=false: current detection (kalman_filter_update); its OBB yaw is already current.
void AlignTrackingFrame(LiDARObjectInfo& object, double yaw_now_rad, bool rotate_obb_yaw = true);
// Single-leveling contract: stored exactly once per PrepareScan. A second store or an invalid
// rotation clears leveling_source (base_from_input fails closed) and returns false.
bool StoreLeveling(const asmc::lidar::Rot& rotation);
// StoreLeveling calls since the last PrepareScan (diagnostics/tests).
unsigned LevelingStoreCount();
// Unit-test seam only: selects common geometry on/off without ROS params or package files
// and resets the per-scan state. The runtime path (main.cpp) never calls it.
void ConfigureForTest(bool enabled, bool use_roll, const std::string& calibration_id);
}  // namespace frame_boundary
