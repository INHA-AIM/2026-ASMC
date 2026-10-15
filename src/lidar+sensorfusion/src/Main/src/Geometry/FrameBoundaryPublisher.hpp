#pragma once
#include <Geometry/FrameBoundary.hpp>
#include <asmc_msgs/LidarObjectArray.h>
#include <asmc_msgs/LidarObjectArrayV2.h>
#include <Geometry/OutputGate.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <mutex>
#include <tf2_ros/transform_broadcaster.h>

namespace frame_boundary {
// B0 gate inputs for one scan, all taken from ONE context: the scan's own copy
// lidar.frame_context when LidarProcess accepted the scan (LidarProcess.cpp copies Current()),
// otherwise Current(), which PrepareScan refreshed for this scan only when pose_ready.
// Nothing is read when !pose_ready (Current() would still hold the previous scan).
output_gate::Inputs GateInputs(bool pose_ready, const std::string& pose_reason, bool lidar_processed,
                               const LiDAR& lidar, std::uint64_t generation);
// Pure V2 builder (no publication). Objects use lidar.frame_context exactly like V1 Publish():
// same leveling, same transform, same publishable-object predicate. Validity never depends on
// subscribers. Any invalid object rejects the whole frame (valid=false); no dummy objects.
struct V2Frame {
  bool valid = false;
  std::string reason;
  output_gate::Mode mode = output_gate::Mode::kNone;
  asmc_msgs::LidarObjectArrayV2 array;
};
V2Frame BuildV2(const LiDAR& lidar, output_gate::Mode mode);
// Lossless MotionState -> (motion_valid, motion_state): Unknown->(false,0), LowThreat->(true,1),
// DynamicPending->(true,2), Dynamic->(true,3). motion_valid=false or an out-of-range state is
// (false, MOTION_UNKNOWN). Compatibility-derived: dynamic_valid=motion_valid,
// is_dynamic=(motion_state==MOTION_DYNAMIC); never derived from b_IsDynamic or the class.
void FillMotionV2(const LiDARObjectInfo& object, asmc_msgs::LidarObjectV2* out);

class Publisher {
 public:
  explicit Publisher(ros::NodeHandle& node);
  void Publish(const LiDAR& lidar);
  void SetLegacyReason(const std::string& reason);
  void Invalid(const ros::Time& stamp,const std::string& reason);
  // Final LiDAR output of one scan: exactly one message on /lidar/map_object (kMap) or
  // /lidar/local_object (kLocal), or nothing at all (kNone, rejected stamp or rejected frame).
  // Returns the effective decision (downgraded to kNone on rejection); the reason goes to diagnostics.
  output_gate::Decision PublishV2(const LiDAR& lidar, const ros::Time& scan_stamp,
                                  const output_gate::Decision& decision);
  void ReportTotals();
 private:
  bool Accept(const ros::Time& stamp);
  void Diagnostics(const ros::WallTimerEvent&);
  asmc::lidar::Context last_context_;
  std::recursive_mutex mutex_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> leveling_tf_;
  ros::Publisher local_,map_,diagnostics_,local_v2_,map_v2_;
  ros::WallTimer timer_;
  ros::Time last_stamp_;
  ros::WallTime last_received_;
  std::string input_reason_="waiting_for_input",map_reason_="gps_missing",legacy_reason_="waiting_for_input";
  std::uint64_t processed_=0,invalid_=0,stamp_errors_=0;
  double last_serialization_ms_=0;
  std::string v2_mode_="none",v2_reason_="waiting_for_input",gps_state_="not_evaluated";
  ros::Time last_v2_stamp_;
  std::uint64_t v2_local_=0,v2_map_=0,v2_none_=0,v2_stamp_errors_=0;
  unsigned last_leveling_store_count_=0;
};
}  // namespace frame_boundary
