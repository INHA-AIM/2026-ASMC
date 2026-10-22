#pragma once
// Relative constant-acceleration Kalman filter of the tracker.
// Ported from Main src/Lidar/Kalman/Kalman.cpp + kalman_utils.hpp at 8f93c22e.
// State [x, y, vx, vy, ax, ay] in the scan's leveled LiDAR frame; Q = 0.003 I,
// R = 0.01 I, P0 = I, update P = (I - KH) P_pred. Every `ego_yaw_now_deg` is the
// IMU yaw in DEGREES (Main documents it as rad; all callers pass f32_Yaw_deg).
//
// Frame change (common geometry): a track with a recorded tracking frame is
// re-expressed by ScanFrame (AlignTrackingState / AlignTrackingFrame). A track
// without one keeps Main's 2D yaw-rotation fallback.

#include <asmc_lidar/pipeline/scan_frame.hpp>
#include <asmc_lidar/pipeline/types.hpp>

#include <utility>

namespace asmc {
namespace lidar {
namespace pipeline {

// Association prediction only (no state change): position of `obj` at dt in
// the current scan frame.
std::pair<float, float> predictKalmanPosition(const TrackedObject& obj, float64 dt, float ego_yaw_now_deg,
                                              const ScanFrame& frame);

// Missed track: re-express the whole previous-scan track (KF state plus OBB
// centre, Z range, corners, yaw; AlignTrackingFrame), then CA predict;
// f32_CoreX/Y = KF position.
void kalman_filter_predict(TrackedObject& o, float64 dt, float ego_yaw_now_deg, const ScanFrame& frame);

// Matched track (or birth init when !b_KfInitialized): the OBB is this scan's
// detection and is not touched; only the copied filter state is re-expressed
// (AlignTrackingState). Second observation initialises velocity from
// (z - pred) / dt. f32_CoreX/Y = KF position.
void kalman_filter_update(TrackedObject& obj, float32 zx, float32 zy, float32 pred_x, float32 pred_y,
                          float64 dt, float ego_yaw_now_deg, const ScanFrame& frame);

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
