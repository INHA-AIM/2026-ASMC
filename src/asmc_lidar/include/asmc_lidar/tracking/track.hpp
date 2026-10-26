#pragma once
#include <asmc_lidar/detection/observation.hpp>
#include <asmc_lidar/fusion/classification.hpp>
#include <asmc_lidar/tracking/motion_display.hpp>

namespace asmc::lidar::pipeline {
// Filter/frame state, separate from current scan geometry and camera evidence.
struct TrackState {
  // Main-compatible applied motion history (formerly PR #105 diagnostics only).
  motion_display::MotionObject display_motion;
  // Common tracking frame record (Global.hpp:522-529).
  bool tracking_frame_valid = false;
  float tracking_height_input = 0.f;
  asmc::lidar::Rot tracking_yaw_from_input = asmc::lidar::Rot::Identity();
  asmc::lidar::Vec tracking_origin_in_yaw = asmc::lidar::Vec::Zero();

  float32 f32_PredCoreX = 0.f, f32_PredCoreY = 0.f;
  float32 f32_PrevCoreX = 0.f, f32_PrevCoreY = 0.f;
  float32 f32_PrevYaw = 0.f;
  // Internal velocity bookkeeping kept for behaviour parity. f32_AbsVx/Vy is
  // Main's "KF relative velocity + CAN speed on the leveled x axis"; its frame is
  // not a certified absolute vector; public output uses the classifier-applied estimate.
  float32 f32_Velocity = 0.f;
  float32 f32_AbsVx = 0.f, f32_AbsVy = 0.f;
  float32 f32_DirX = 0.f, f32_DirY = 0.f;

  int32 s32_Id = -1;
  int32 s32_Age = 0;
  int32 s32_MissCount = 0;
  int32 s32_KfInitCount = 0;
  int32 s32_DynamicCount = 0;
  int32 s32_StaticCount = 0;

  // Relative constant-acceleration state [x, y, vx, vy, ax, ay] in the leveled frame.
  Eigen::Matrix<float, 6, 1> Eigen_KfState_CA = Eigen::Matrix<float, 6, 1>::Zero();
  Eigen::Matrix<float, 6, 6> Eigen_KfP_CA = Eigen::Matrix<float, 6, 6>::Identity();
  bool b_KfInitialized = false;
};

struct TrackedObject {
  DetectionObservation detection;
  TrackState tracking;
  CameraClassEvidence camera;
};

}  // namespace asmc::lidar::pipeline
