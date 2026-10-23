#include <asmc_lidar/tracking/track.hpp>
// Per-scan coordinate context and track re-expression between scan frames.
// Ported from Main src/Geometry/FrameBoundary.cpp at 8f93c22e (tracker fix),
// common-geometry behaviour only: Main's namespace globals `current`,
// `scan_attitude`, `scan_yaw` and `calibration_id` are members here, and every
// `if(!Enabled()) return;` guard is dropped (always enabled). ROS parameter and
// calibration-file validation (Configure) belongs to the runtime config loader.
#include <asmc_lidar/geometry/scan_frame.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

// Ported from Main Global.cpp:10-13 (8f93c22e): float overload used by SensorMount.
float32 Deg2Rad(const float32& f32_Deg) { return f32_Deg * M_PI / 180.0f; }

}  // namespace

// Ported from Main FrameBoundary.cpp:44-50 (8f93c22e)
asmc::lidar::Pose ScanFrame::SensorMount(const VehicleSpec& vehicle) {
  asmc::lidar::Pose mount = asmc::lidar::Pose::Identity();
  mount.translation() =
      asmc::lidar::Vec(vehicle.st_LiDARPos.f32_X, vehicle.st_LiDARPos.f32_Y, vehicle.st_LiDARPos.f32_Z);
  mount.linear() = asmc::lidar::rpy(Deg2Rad(vehicle.st_LiDARRotationDeg.f32_X),
                                    Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Y),
                                    Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Z));
  return mount;
}

// Ported from Main FrameBoundary.cpp:51-53 (8f93c22e)
asmc::lidar::Rot ScanFrame::MapRotation(const LocalState& local) {
  return asmc::lidar::rpy(local.st_IMU.f32_Roll_rad, local.st_IMU.f32_Pitch_rad, local.st_IMU.f32_Yaw_rad);
}

// Main Configure() keeps only the parts the tracker needs: the mount (Main
// recomputes SensorMount(vehicle) in every PrepareScan) and the calibration id.
void ScanFrame::Configure(const VehicleSpec& vehicle, const std::string& calibration_id) {
  if (calibration_id.empty()) throw std::invalid_argument("ScanFrame: calibration_id must be non-empty");
  mount_ = SensorMount(vehicle);
  calibration_id_ = calibration_id;
}

// Ported from Main FrameBoundary.cpp:149-163 (8f93c22e)
void ScanFrame::Prepare(const asmc::lidar::Observation& time, const LocalState& local,
                        const asmc::lidar::PoseSelection& map_pose, std::uint64_t scan_sequence) {
  current_ = asmc::lidar::Context{};
  scan_attitude_ = MapRotation(local);
  scan_yaw_ = local.st_IMU.f32_Yaw_rad;
  current_.time = time;
  current_.scan_sequence = scan_sequence;
  current_.frame = asmc::lidar::Frame::LeveledSensor;
  current_.base_from_sensor = mount_;
  current_.calibration_source = calibration_id_;
  // Kept verbatim for Context parity with Main (LevelPoints is its port).
  current_.leveling_source = "Main::CorrectingPoints";
  current_.map_pose = map_pose;
  // KF derivatives are not promoted to an absolute 3D velocity by this adapter.
  current_.ego_linear_base = asmc::lidar::Vec(local.st_CAN.f32_CurrentVel_MPS, 0, 0);
}

// Ported from Main FrameBoundary.cpp:243 (8f93c22e)
void ScanFrame::StoreLeveling(const asmc::lidar::Rot& rotation) { current_.applied_leveling = rotation; }

// Ported from Main FrameBoundary.cpp:165-168 (8f93c22e)
asmc::lidar::Rot ScanFrame::YawFromInput() const {
  return asmc::lidar::rpy(0, 0, scan_yaw_).transpose() * scan_attitude_ * current_.base_from_sensor.linear() *
         current_.applied_leveling.transpose();
}

// Ported from Main FrameBoundary.cpp:169-172 (8f93c22e)
asmc::lidar::Vec ScanFrame::OriginInYaw() const {
  return asmc::lidar::rpy(0, 0, scan_yaw_).transpose() * scan_attitude_ * current_.base_from_sensor.translation();
}

// Ported from Main FrameBoundary.cpp:181-186 (8f93c22e)
void ScanFrame::RecordTrackingFrame(TrackedObject& object) const {
  object.tracking.tracking_frame_valid = true;
  object.tracking.tracking_height_input = object.detection.f32_CoreZ;
  object.tracking.tracking_yaw_from_input = YawFromInput();
  object.tracking.tracking_origin_in_yaw = OriginInYaw();
}

// Ported from Main FrameBoundary.cpp:190-195 (8f93c22e)
ScanFrame::FrameChange ScanFrame::ChangeSincePrevious(const TrackedObject& object, double yaw_now_rad) const {
  const auto inverse = YawFromInput().transpose().eval();
  const auto yaw_change = asmc::lidar::rpy(0, 0, object.tracking.f32_PrevYaw - yaw_now_rad);
  return {(inverse * yaw_change * object.tracking.tracking_yaw_from_input).eval(),
          (inverse * (yaw_change * object.tracking.tracking_origin_in_yaw - OriginInYaw())).eval()};
}

// Ported from Main FrameBoundary.cpp:196-209 (8f93c22e)
void ScanFrame::AlignState(TrackedObject& object, const FrameChange& change, double yaw_now_rad) const {
  // Re-express the previous state before the unchanged relative CA prediction.
  // Do not subtract CAN displacement: relative CA already models ego motion.
  const asmc::lidar::Vec position(object.tracking.Eigen_KfState_CA(0), object.tracking.Eigen_KfState_CA(1),
                                  object.tracking.tracking_height_input);
  const auto next = (change.rotation * position + change.translation).eval();
  Eigen::Matrix<float, 6, 6> jacobian = Eigen::Matrix<float, 6, 6>::Zero();
  for (unsigned block = 0; block < 3; ++block)
    jacobian.block<2, 2>(2 * block, 2 * block) = change.rotation.topLeftCorner<2, 2>().cast<float>();
  object.tracking.Eigen_KfState_CA = jacobian * object.tracking.Eigen_KfState_CA;
  object.tracking.Eigen_KfState_CA(0) = next.x();
  object.tracking.Eigen_KfState_CA(1) = next.y();
  object.tracking.Eigen_KfP_CA = jacobian * object.tracking.Eigen_KfP_CA * jacobian.transpose();
  object.tracking.f32_PrevYaw = yaw_now_rad;
  RecordTrackingFrame(object);
  object.tracking.tracking_height_input = next.z();
}

// Ported from Main FrameBoundary.cpp:211-214 (8f93c22e)
void ScanFrame::AlignTrackingState(TrackedObject& object, double yaw_now_rad) const {
  if (!object.tracking.tracking_frame_valid) return;
  AlignState(object, ChangeSincePrevious(object, yaw_now_rad), yaw_now_rad);
}

// Ported from Main FrameBoundary.cpp:215-242 (8f93c22e); EdgeFollowStats added.
void ScanFrame::AlignTrackingFrame(TrackedObject& object, double yaw_now_rad) const {
  ++edge_stats_.calls;
  if (!object.tracking.tracking_frame_valid) {
    ++edge_stats_.not_recorded;
    return;
  }
  const auto change = ChangeSincePrevious(object, yaw_now_rad);
  // The OBB was measured in the previous scan: centre, corners and long axis
  // take the same frame change as the filter state, exactly once. The box is
  // only re-expressed; the relative CA prediction moves the KF state alone.
  const auto move = [&](float& x, float& y, float& z) {
    const auto p = (change.rotation * asmc::lidar::Vec(x, y, z) + change.translation).eval();
    x = p.x();
    y = p.y();
    z = p.z();
  };
  const float previous_center_z = object.detection.f32_OBB_CenterZ;
  move(object.detection.f32_OBB_CenterX, object.detection.f32_OBB_CenterY, object.detection.f32_OBB_CenterZ);
  object.detection.f32_OBB_ZMin += object.detection.f32_OBB_CenterZ - previous_center_z;
  object.detection.f32_OBB_ZMax += object.detection.f32_OBB_CenterZ - previous_center_z;
  for (auto& corner : object.detection.obb_corners_level) move(corner.x, corner.y, corner.z);
  // Preserved: std::cos/std::sin of the float yaw (float overloads), as in Main.
  const auto axis = (change.rotation * asmc::lidar::Vec(std::cos(object.detection.f32_OBB_YawRad),
                                                        std::sin(object.detection.f32_OBB_YawRad), 0))
                        .eval();
  double yaw = std::atan2(axis.y(), axis.x());
  // Corner 0->1 is the yaw edge (computeOBBAtYaw order). A tilted frame change
  // gives that edge a z component the 2D yaw cannot carry, so follow the edge.
  // That correction is tiny; a large one means another corner order, so skip it.
  const auto& first = object.detection.obb_corners_level[0];
  const auto& second = object.detection.obb_corners_level[1];
  if (std::hypot(second.x - first.x, second.y - first.y) > 1e-6) {
    const double correction = std::remainder(std::atan2(second.y - first.y, second.x - first.x) - yaw, M_PI);
    if (std::abs(correction) < .1) {
      yaw += correction;
      ++edge_stats_.applied;
      edge_stats_.max_applied_abs_rad = std::max(edge_stats_.max_applied_abs_rad, std::abs(correction));
    } else {
      ++edge_stats_.rejected;
      edge_stats_.max_rejected_abs_rad = std::max(edge_stats_.max_rejected_abs_rad, std::abs(correction));
    }
  } else {
    ++edge_stats_.degenerate_edge;
  }
  object.detection.f32_OBB_YawRad = std::atan2(std::sin(yaw), std::cos(yaw));
  AlignState(object, change, yaw_now_rad);
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
