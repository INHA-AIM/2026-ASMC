#include <asmc_lidar/tracking/track.hpp>
// CA Kalman / Hungarian tracker.
// Ported from Main src/Lidar/FrameFilter/FrameFilter.cpp (FrameFilter) and
// src/Lidar/Tracking/Tracking.cpp (CompareCentroids) at 8f93c22e, common
// geometry. Globals become members: st_Tracking -> config_, st_Tracking.
// s32_NewTrackId -> next_track_id_, Prev_Objects_Queue -> history_. Not ported
// (no effect on any tracker decision on the common path): Body2ENU/MinMax2ENU
// and the f32_PrevE/N bookkeeping (map coordinates), fusion_trace /
// fusion_state_trace events, and dead locals (Ton/Toff/N_on/N_off, vx/vy/
// raw_speed, f32_Distance, f32_Pred_x/y in the matched loop).
#include <asmc_lidar/tracking/hungarian.hpp>
#include <asmc_lidar/tracking/kalman.hpp>
#include <asmc_lidar/detection/obb.hpp>
#include <asmc_lidar/tracking/tracker.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

bool IsDummy(const TrackedObject& object) {
  return object.detection.f32_CoreX == 0.0f && object.detection.f32_CoreY == 0.0f && object.detection.f32_CoreZ == 0.0f;
}

// Ported from Main Tracking.cpp:576-965 (8f93c22e) (CompareCentroids, common path).
void CompareCentroids(const std::vector<TrackedObject>& vec_Compare, std::vector<TrackedObject>& vec_NowObject,
                      float64 f64_Time, const LocalState& st_GPSIMU, const ScanFrame& frame,
                      const TrackingConfig& st_Tracking, int32& s32_NewTrackId) {
  int32 N = static_cast<int32>(vec_Compare.size());
  int32 M = static_cast<int32>(vec_NowObject.size());

  // Predicted positions for the cost matrix.
  std::vector<std::pair<float, float>> vec_PredictedPositions(N);
  for (int32 i = 0; i < N; ++i) {
    std::pair<float, float> pp =
        predictKalmanPosition(vec_Compare[i], f64_Time, st_GPSIMU.st_IMU.f32_Yaw_deg, frame);
    vec_PredictedPositions[i].first = pp.first;
    vec_PredictedPositions[i].second = pp.second;
  }

  // Cost: body-frame distance to the prediction + (1 - ShapeAgreement) - age
  // continuity bonus, gated by COST_THRESHOLD.
  std::vector<std::vector<double>> vec_CostMatrix(N, std::vector<double>(M, st_Tracking.f32_COST_INF));
  for (int32 i = 0; i < N; ++i) {
    float32 f32_Pred_x = vec_PredictedPositions[i].first;
    float32 f32_Pred_y = vec_PredictedPositions[i].second;

    for (int32 j = 0; j < M; ++j) {
      const int measured_id=vec_NowObject[j].detection.measured_vehicle_track_id;
      const int previous_id=vec_Compare[i].tracking.s32_Id;
      const bool reserved=std::any_of(vec_NowObject.begin(),vec_NowObject.end(),[&](const auto& o){
        return o.detection.measured_vehicle_track_id==previous_id;});
      if(reserved) {
        // The preceding stage verified current points against this exact
        // track with measured ego motion. A small adjacent residual must not
        // steal its ID through a lower KF-centroid distance.
        if(measured_id==previous_id)vec_CostMatrix[i][j]=0;
        continue;
      }
      if(vec_NowObject[j].detection.measured_vehicle_track_id>=0&&
          vec_NowObject[j].detection.measured_vehicle_track_id!=vec_Compare[i].tracking.s32_Id)continue;
      float32 f32_Det_x = vec_NowObject[j].detection.f32_CoreX;
      float32 f32_Det_y = vec_NowObject[j].detection.f32_CoreY;

      // 1) Euclidean distance. Legacy-only map-centre distance
      // (!Enabled() && UseMapAssociation, Tracking.cpp:621-626) not ported.
      float32 f32_Dx = f32_Det_x - f32_Pred_x;
      float32 f32_Dy = f32_Det_y - f32_Pred_y;
      float32 f32_Cost_dist = std::sqrt(f32_Dx * f32_Dx + f32_Dy * f32_Dy);

      // 2) Shape term in [0, 1] (no IoU despite the weight name).
      const float32 f32_Cost_shape = 1.0F - tracking_association::ShapeAgreement(vec_Compare[i], vec_NowObject[j]);

      // (commented heading term :635-654 not ported)
      float32 f32_Cost = st_Tracking.f32_COST_W_DIST * f32_Cost_dist + st_Tracking.f32_COST_W_IOU * f32_Cost_shape;
      // Prefer long-lived tracks among equal candidates; bounded by the bonus.
      const float32 age_ratio = std::clamp(static_cast<float32>(vec_Compare[i].tracking.s32_Age) / 30.0F, 0.0F, 1.0F);
      f32_Cost = std::max(0.0F, f32_Cost - st_Tracking.f32_ContinuityBonus * age_ratio);

      if (f32_Cost < st_Tracking.f32_COST_THRESHOLD) {
        vec_CostMatrix[i][j] = f32_Cost;
      } else {
        vec_CostMatrix[i][j] = st_Tracking.f32_COST_INF;
      }
    }
  }

  AssignmentProblemSolver solver;
  std::vector<int32> assignment;
  solver.Solve(vec_CostMatrix, assignment, AssignmentProblemSolver::optimal);
  // (fusion_trace "tracking_assignment" event :681-693 not ported)

  std::vector<bool> matched_now(M, false);
  std::vector<bool> matched_prev(N, false);

  for (int i = 0; i < N; ++i) {
    int j = (i < static_cast<int>(assignment.size())) ? assignment[i] : -1;
    // Gated (COST_INF) Hungarian pairs are rejected here.
    if (j >= 0 && j < M && vec_CostMatrix[i][j] < st_Tracking.f32_COST_THRESHOLD) {
      vec_NowObject[j].tracking.f32_PrevCoreX = vec_Compare[i].detection.f32_CoreX;
      vec_NowObject[j].tracking.f32_PrevCoreY = vec_Compare[i].detection.f32_CoreY;
      vec_NowObject[j].tracking.f32_PredCoreX = vec_PredictedPositions[i].first;
      vec_NowObject[j].tracking.f32_PredCoreY = vec_PredictedPositions[i].second;
      float32 f32_Det_x = vec_NowObject[j].detection.f32_CoreX;
      float32 f32_Det_y = vec_NowObject[j].detection.f32_CoreY;

      vec_NowObject[j].tracking.Eigen_KfState_CA = vec_Compare[i].tracking.Eigen_KfState_CA;
      vec_NowObject[j].tracking.Eigen_KfP_CA = vec_Compare[i].tracking.Eigen_KfP_CA;
      vec_NowObject[j].tracking.b_KfInitialized = vec_Compare[i].tracking.b_KfInitialized;
      vec_NowObject[j].tracking.s32_KfInitCount = vec_Compare[i].tracking.s32_KfInitCount;
      vec_NowObject[j].tracking.s32_Id = vec_Compare[i].tracking.s32_Id;
      vec_NowObject[j].tracking.s32_Age = vec_Compare[i].tracking.s32_Age + 1;
      vec_NowObject[j].tracking.s32_MissCount = 0;
      vec_NowObject[j].tracking.s32_StaticCount = vec_Compare[i].tracking.s32_StaticCount;
      vec_NowObject[j].tracking.s32_DynamicCount = vec_Compare[i].tracking.s32_DynamicCount;
      vec_NowObject[j].tracking.f32_PrevYaw = vec_Compare[i].tracking.f32_PrevYaw;
      vec_NowObject[j].tracking.tracking_frame_valid = vec_Compare[i].tracking.tracking_frame_valid;
      vec_NowObject[j].tracking.tracking_height_input = vec_Compare[i].tracking.tracking_height_input;
      vec_NowObject[j].tracking.tracking_yaw_from_input = vec_Compare[i].tracking.tracking_yaw_from_input;
      vec_NowObject[j].tracking.tracking_origin_in_yaw = vec_Compare[i].tracking.tracking_origin_in_yaw;

      if (tracking_association::ShouldCarryPreviousClass(vec_Compare[i], vec_NowObject[j])) {
        tracking_association::CopyClassState(vec_Compare[i], vec_NowObject[j]);
      }  // else: Main only emits a "class_tracking_reset" trace.

      if (vec_Compare[i].tracking.tracking_frame_valid) {
        // Heading alignment needs no point-cloud copy of the old track. Preserved:
        // value-initialized copy (OBB centre/corners zero, so the edge follow is
        // skipped and only the yaw axis is re-expressed, once).
        TrackedObject aligned{};
        aligned.tracking.s32_MissCount = vec_Compare[i].tracking.s32_MissCount;
        aligned.tracking.Eigen_KfState_CA = vec_Compare[i].tracking.Eigen_KfState_CA;
        aligned.tracking.Eigen_KfP_CA = vec_Compare[i].tracking.Eigen_KfP_CA;
        aligned.tracking.tracking_frame_valid = true;
        aligned.tracking.tracking_height_input = vec_Compare[i].tracking.tracking_height_input;
        aligned.tracking.tracking_yaw_from_input = vec_Compare[i].tracking.tracking_yaw_from_input;
        aligned.tracking.tracking_origin_in_yaw = vec_Compare[i].tracking.tracking_origin_in_yaw;
        aligned.tracking.f32_PrevYaw = vec_Compare[i].tracking.f32_PrevYaw;
        aligned.detection.f32_OBB_YawRad = vec_Compare[i].detection.f32_OBB_YawRad;
        frame.AlignTrackingFrame(aligned, st_GPSIMU.st_IMU.f32_Yaw_rad);
        obb_util::stabilizeMatchedOBB(aligned, vec_NowObject[j], st_GPSIMU.st_IMU.f32_Yaw_deg);
      } else {
        obb_util::stabilizeMatchedOBB(vec_Compare[i], vec_NowObject[j], st_GPSIMU.st_IMU.f32_Yaw_deg);
      }
      // Kalman update (the OBB stays this scan's detection).
      kalman_filter_update(vec_NowObject[j], f32_Det_x, f32_Det_y, vec_NowObject[j].tracking.f32_PredCoreX,
                           vec_NowObject[j].tracking.f32_PredCoreY, f64_Time, st_GPSIMU.st_IMU.f32_Yaw_deg, frame);

      // Preserved: KF relative velocity + CAN speed on the leveled x axis only.
      const float32 f32_Rel_vx = vec_NowObject[j].tracking.Eigen_KfState_CA(2);
      const float32 f32_Rel_vy = vec_NowObject[j].tracking.Eigen_KfState_CA(3);
      const float32 v_mps = st_GPSIMU.st_CAN.f32_CurrentVel_KPH / 3.6f;
      const float32 f32_Abs_vx = f32_Rel_vx + v_mps;
      const float32 f32_Abs_vy = f32_Rel_vy;

      vec_NowObject[j].tracking.f32_AbsVx = f32_Abs_vx;
      vec_NowObject[j].tracking.f32_AbsVy = f32_Abs_vy;
      vec_NowObject[j].tracking.f32_Velocity = std::hypot(f32_Abs_vx, f32_Abs_vy);
      if (vec_NowObject[j].tracking.f32_Velocity > 1e-3f) {
        vec_NowObject[j].tracking.f32_DirX = f32_Abs_vx / vec_NowObject[j].tracking.f32_Velocity;
        vec_NowObject[j].tracking.f32_DirY = f32_Abs_vy / vec_NowObject[j].tracking.f32_Velocity;
      } else {
        vec_NowObject[j].tracking.f32_DirX = 0.0f;
        vec_NowObject[j].tracking.f32_DirY = 0.0f;
      }

      matched_now[j] = true;
      matched_prev[i] = true;
    }
  }

  // A track split in two in the previous scan and merged again now leaves one
  // Hungarian loser; retire it when a matched current object sits at its
  // prediction with the same size. Common: distance from the KF-filtered
  // matched centre to the prediction (legacy map distance not ported).
  const float32 retire_distance = std::max(0.0F, st_Tracking.f32_MergedTrackRetireDistance);
  const float32 retire_shape_agreement = std::clamp(st_Tracking.f32_DuplicateIouThreshold * 0.8F, 0.0F, 1.0F);
  for (int i = 0; i < N; ++i) {
    if (matched_prev[i]) continue;
    for (int j = 0; j < M; ++j) {
      if (!matched_now[j]) continue;
      const float retire_dist = std::hypot(vec_NowObject[j].detection.f32_CoreX - vec_PredictedPositions[i].first,
                                           vec_NowObject[j].detection.f32_CoreY - vec_PredictedPositions[i].second);
      if (retire_dist > retire_distance) {
        continue;
      }
      if (tracking_association::ShapeAgreement(vec_Compare[i], vec_NowObject[j]) < retire_shape_agreement) {
        continue;
      }
      matched_prev[i] = true;
      break;
    }
  }

  // Unmatched current detections: births (KF init only, b_KfInitialized=false).
  for (int j = 0; j < M; ++j) {
    if (!matched_now[j] && vec_NowObject[j].tracking.s32_Id < 0) {
      vec_NowObject[j].tracking.b_KfInitialized = false;
      vec_NowObject[j].tracking.s32_Id = s32_NewTrackId++;
      vec_NowObject[j].tracking.s32_Age = 1;
      vec_NowObject[j].tracking.s32_MissCount = 0;
      kalman_filter_update(vec_NowObject[j], vec_NowObject[j].detection.f32_CoreX, vec_NowObject[j].detection.f32_CoreY, 0, 0, f64_Time,
                           st_GPSIMU.st_IMU.f32_Yaw_deg, frame);
    }
  }

  // Unmatched, not retired previous tracks: carried with a KF prediction until
  // the 25th consecutive miss. Points/AABB/f32_Dist/CoreZ/velocity/class stay
  // stale copies (preserved).
  for (int i = 0; i < N; ++i) {
    if (!matched_prev[i] && vec_Compare[i].tracking.b_KfInitialized) {
      TrackedObject missed = vec_Compare[i];
      missed.tracking.s32_Age = vec_Compare[i].tracking.s32_Age + 1;
      missed.tracking.s32_MissCount = vec_Compare[i].tracking.s32_MissCount + 1;

      if (missed.tracking.s32_MissCount < Tracker::kMaxMissCount) {
        kalman_filter_predict(missed, f64_Time, st_GPSIMU.st_IMU.f32_Yaw_deg, frame);
        missed.detection.f32_CoreX = missed.tracking.Eigen_KfState_CA(0);
        missed.detection.f32_CoreY = missed.tracking.Eigen_KfState_CA(1);
        vec_NowObject.push_back(missed);
      }
    }
  }
}

}  // namespace

Tracker::Tracker(const TrackingConfig& config) : config_(config) {}

// Ported from Main FrameFilter.cpp:10-151 (8f93c22e)
void Tracker::Update(std::vector<TrackedObject>& vec_Object, double dt_s, const LocalState& st_GPSIMU,
                     const ScanFrame& frame) {
  // Merge vertically split clusters of one XY occupancy before IDs are given.
  ConsolidateCurrentDetections(vec_Object, config_);
  // (Body2ENU/MinMax2ENU :27-30 not ported: map coordinates only.)

  if (history_.empty()) {
    for (int32 s32_Idx = 0; s32_Idx < static_cast<int>(vec_Object.size()); ++s32_Idx) {
      // Preserved: stops at the first detection whose centroid is exactly (0,0,0).
      if (IsDummy(vec_Object[s32_Idx])) {
        break;
      }
      vec_Object[s32_Idx].tracking.s32_Id = next_track_id_++;
      vec_Object[s32_Idx].tracking.s32_Age = 1;
      vec_Object[s32_Idx].tracking.s32_MissCount = 0;
      vec_Object[s32_Idx].tracking.b_KfInitialized = false;

      // Initialise the filter on the first observation (Main's hard-coded dt).
      kalman_filter_update(vec_Object[s32_Idx], vec_Object[s32_Idx].detection.f32_CoreX, vec_Object[s32_Idx].detection.f32_CoreY,
                           vec_Object[s32_Idx].detection.f32_CoreX, vec_Object[s32_Idx].detection.f32_CoreY, kBirthInitDt,
                           st_GPSIMU.st_IMU.f32_Yaw_deg, frame);
      // (Body2ENU/MinMax2ENU, PrevE/N = CoreE/N :57-60 not ported)
    }
    return;
  }

  const std::vector<TrackedObject>& prev = history_.back();

  // Is any previous object a real measurement?
  bool b_prevAllDummy = true;
  for (int32 s32_Idx = 0; s32_Idx < static_cast<int>(prev.size()); ++s32_Idx) {
    if (!IsDummy(prev[s32_Idx])) {
      b_prevAllDummy = false;
      break;
    }
  }

  if (!b_prevAllDummy) {
    CompareCentroids(prev, vec_Object, dt_s, st_GPSIMU, frame, config_, next_track_id_);
    // (per-object Body2ENU/MinMax2ENU and PrevE/N update :85-118 not ported;
    // its vx/vy/raw_speed are computed but unused in Main)
  } else {
    // Only dummies before: no association, births only.
    for (int32 s32_Idx = 0; s32_Idx < static_cast<int32>(vec_Object.size()); ++s32_Idx) {
      if (IsDummy(vec_Object[s32_Idx])) {
        break;
      }
      if (vec_Object[s32_Idx].tracking.s32_Id < 0) {
        vec_Object[s32_Idx].tracking.s32_Id = next_track_id_++;
        vec_Object[s32_Idx].tracking.s32_Age = 1;
        vec_Object[s32_Idx].tracking.s32_MissCount = 0;
        vec_Object[s32_Idx].tracking.b_KfInitialized = false;

        kalman_filter_update(vec_Object[s32_Idx], vec_Object[s32_Idx].detection.f32_CoreX, vec_Object[s32_Idx].detection.f32_CoreY,
                             vec_Object[s32_Idx].detection.f32_CoreX, vec_Object[s32_Idx].detection.f32_CoreY, kBirthInitDt,
                             st_GPSIMU.st_IMU.f32_Yaw_deg, frame);
        // (Body2ENU/MinMax2ENU, PrevE/N = CoreE/N :144-147 not ported)
      }
    }
  }
}

// Ported from Main LidarProcess.cpp:307-313 (8f93c22e)
void Tracker::Commit(const std::vector<TrackedObject>& frame) {
  history_.push_back(frame);
  if (history_.size() > kMaxHistory) {
    history_.pop_front();
  }
}

std::vector<TrackedObject>* Tracker::LastFrame() { return history_.empty() ? nullptr : &history_.back(); }

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
