// Relative constant-acceleration Kalman filter of the tracker.
// Ported from Main src/Lidar/Kalman/Kalman.cpp at 8f93c22e (common geometry).
// `frame_boundary::Enabled()` is always true here; the `!tracking_frame_valid`
// 2D fallback stays because it is reachable on Main's common path. Not ported:
// unused MAX_FRAMES/speed_counter/rot2 (:6-7, :42-46) and the commented IMM/CV
// filters (:208-424).
#include <asmc_lidar/pipeline/kalman.hpp>

#include <cmath>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

// Ported from Main Kalman.cpp:12-18 (8f93c22e)
constexpr float kYawSign = +1.f;  // CCW+ unchanged
inline float to_ccw_rad(float imu_yaw_now_deg) {
  float y = imu_yaw_now_deg * 0.017453292519943295f;  // deg -> rad (float multiply)
  return kYawSign * y;
}

// Ported from Main Kalman.cpp:21-26 (8f93c22e); float, distinct from obb_util::wrapToPi.
inline float wrapToPi(float a) {
  const float PI = 3.14159265358979323846f;
  while (a > PI) a -= 2.f * PI;
  while (a < -PI) a += 2.f * PI;
  return a;
}

// Ported from Main Kalman.cpp:29-39 (8f93c22e): T(dpsi) = diag(R, R, R).
inline Eigen::Matrix<float, 6, 6> makeYawTransform(float dpsi) {
  float c = std::cos(dpsi), s = std::sin(dpsi);
  Eigen::Matrix<float, 6, 6> T = Eigen::Matrix<float, 6, 6>::Identity();
  T(0, 0) = c; T(0, 1) = -s; T(1, 0) = s; T(1, 1) = c;
  T(2, 2) = c; T(2, 3) = -s; T(3, 2) = s; T(3, 3) = c;
  T(4, 4) = c; T(4, 5) = -s; T(5, 4) = s; T(5, 5) = c;
  return T;
}

// Ported from Main Kalman.cpp:50-77 (8f93c22e). previous_obb: the OBB still is
// the previous scan's (predict-only track) and is re-expressed with the state.
inline void compensate_ego_yaw(TrackedObject& o, float ego_yaw_now_deg, bool previous_obb, const ScanFrame& frame) {
  // Preserved: deg * M_PI (double) / 180.0f, stored as float.
  float yaw_now = ego_yaw_now_deg * M_PI / 180.0f;

  if (o.tracking_frame_valid) {
    if (previous_obb) frame.AlignTrackingFrame(o, yaw_now);
    else frame.AlignTrackingState(o, yaw_now);
    return;
  }
  float dpsi = wrapToPi(yaw_now - o.f32_PrevYaw);

  if (std::fabs(dpsi) < 1e-8f) {
    o.f32_PrevYaw = yaw_now;
    return;
  }

  Eigen::Matrix<float, 6, 6> T = makeYawTransform(-dpsi);
  o.Eigen_KfState_CA = T * o.Eigen_KfState_CA;
  o.Eigen_KfP_CA = T * o.Eigen_KfP_CA * T.transpose();

  o.f32_PrevYaw = yaw_now;
}

}  // namespace

// Ported from Main Kalman.cpp:80-111 (8f93c22e)
std::pair<float, float> predictKalmanPosition(const TrackedObject& obj, float64 dt, float ego_yaw_now_deg,
                                              const ScanFrame& frame) {
  if (obj.tracking_frame_valid) {
    TrackedObject aligned{};
    aligned.Eigen_KfState_CA = obj.Eigen_KfState_CA;
    aligned.Eigen_KfP_CA = obj.Eigen_KfP_CA;
    aligned.tracking_frame_valid = true;
    aligned.tracking_height_input = obj.tracking_height_input;
    aligned.tracking_yaw_from_input = obj.tracking_yaw_from_input;
    aligned.tracking_origin_in_yaw = obj.tracking_origin_in_yaw;
    aligned.f32_PrevYaw = obj.f32_PrevYaw;
    frame.AlignTrackingState(aligned, to_ccw_rad(ego_yaw_now_deg));
    const auto& x = aligned.Eigen_KfState_CA;
    // Preserved: double arithmetic (dt is double), narrowed to float on return.
    return {static_cast<float>(x(0) + dt * x(2) + .5 * dt * dt * x(4)),
            static_cast<float>(x(1) + dt * x(3) + .5 * dt * dt * x(5))};
  }
  // 1) dpsi = yaw now - yaw of the previous frame
  const float yaw_now = to_ccw_rad(ego_yaw_now_deg);
  float dpsi = wrapToPi(yaw_now - obj.f32_PrevYaw);

  // 2) align by -dpsi
  Eigen::Matrix<float, 6, 6> T = makeYawTransform(-dpsi);
  Eigen::Matrix<float, 6, 1> x0 = T * obj.Eigen_KfState_CA;

  // 3) CA prediction
  Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
  A(0, 2) = dt; A(1, 3) = dt;
  A(0, 4) = 0.5f * dt * dt; A(1, 5) = 0.5f * dt * dt;
  A(2, 4) = dt; A(3, 5) = dt;

  Eigen::Matrix<float, 6, 1> x_pred = A * x0;
  return {x_pred(0), x_pred(1)};
}

// Ported from Main Kalman.cpp:112-140 (8f93c22e)
void kalman_filter_predict(TrackedObject& o, float64 dt, float ego_yaw_now_deg, const ScanFrame& frame) {
  // 1) Re-express into the current scan (updates PrevYaw).
  compensate_ego_yaw(o, ego_yaw_now_deg, true, frame);

  // 2) Standard CA prediction.
  Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
  A(0, 2) = dt; A(1, 3) = dt;
  A(0, 4) = 0.5f * dt * dt; A(1, 5) = 0.5f * dt * dt;
  A(2, 4) = dt; A(3, 5) = dt;

  Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity() * 0.003f;

  o.Eigen_KfState_CA = A * o.Eigen_KfState_CA;
  o.Eigen_KfP_CA = A * o.Eigen_KfP_CA * A.transpose() + Q;

  o.f32_CoreX = o.Eigen_KfState_CA(0);
  o.f32_CoreY = o.Eigen_KfState_CA(1);
}

// Ported from Main Kalman.cpp:143-206 (8f93c22e)
void kalman_filter_update(TrackedObject& obj, float32 zx, float32 zy, float32 pred_x, float32 pred_y,
                          float64 dt, float ego_yaw_now_deg, const ScanFrame& frame) {
  // --- initialisation ---
  if (!obj.b_KfInitialized) {
    obj.Eigen_KfState_CA << zx, zy, 0.f, 0.f, 0.f, 0.f;
    obj.Eigen_KfP_CA.setIdentity();
    obj.b_KfInitialized = true;
    obj.s32_KfInitCount = 1;
    obj.f32_CoreX = zx;
    obj.f32_CoreY = zy;
    obj.f32_PrevYaw = to_ccw_rad(ego_yaw_now_deg);
    return;
  }

  // --- re-express the copied previous-scan state (updates PrevYaw) ---
  compensate_ego_yaw(obj, ego_yaw_now_deg, false, frame);

  // --- second observation: velocity initialisation (pred in the current frame) ---
  if (obj.s32_KfInitCount == 1) {
    // Preserved: no dt > 0 guard (Main divides unconditionally).
    float32 vx_init = (zx - pred_x) / dt;
    float32 vy_init = (zy - pred_y) / dt;

    obj.Eigen_KfState_CA << pred_x, pred_y, vx_init, vy_init, 0.f, 0.f;
    obj.s32_KfInitCount = 2;
  }

  // --- CA model / noise ---
  Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
  A(0, 2) = dt; A(1, 3) = dt;
  A(0, 4) = 0.5f * dt * dt; A(1, 5) = 0.5f * dt * dt;
  A(2, 4) = dt; A(3, 5) = dt;

  Eigen::Matrix<float, 2, 6> H = Eigen::Matrix<float, 2, 6>::Zero();
  H(0, 0) = 1.f; H(1, 1) = 1.f;

  Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity() * 0.003f;
  Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.01f;
  Eigen::Vector2f z(zx, zy);

  // --- predict ---
  Eigen::Matrix<float, 6, 1> x_pred = A * obj.Eigen_KfState_CA;
  Eigen::Matrix<float, 6, 6> P_pred = A * obj.Eigen_KfP_CA * A.transpose() + Q;

  // --- update ---
  Eigen::Vector2f y = z - H * x_pred;
  Eigen::Matrix2f S = H * P_pred * H.transpose() + R;
  Eigen::Matrix<float, 6, 2> K = P_pred * H.transpose() * S.inverse();

  obj.Eigen_KfState_CA = x_pred + K * y;
  obj.Eigen_KfP_CA = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * P_pred;

  obj.f32_CoreX = obj.Eigen_KfState_CA(0);
  obj.f32_CoreY = obj.Eigen_KfState_CA(1);
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
