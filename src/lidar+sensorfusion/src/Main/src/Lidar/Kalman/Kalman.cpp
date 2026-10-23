#include "kalman_utils.hpp"
#include <Geometry/FrameBoundary.hpp>
#include <Lidar/Tracking/TrackTrace.hpp>
#include <Lidar/Tracking/ObservationModel.hpp>



const int MAX_FRAMES = 5;
int speed_counter=0; //속도값 갯수
//전프레임의 시간

//[-pi, pi] 정규화
// IMU 요각을 "CCW+, rad"로 표준화
constexpr float kYawSign  = +1.f;   // CCW+ 그대로 유지


inline float to_ccw_rad(float imu_yaw_now_deg){
    float y = imu_yaw_now_deg * 0.017453292519943295f; // deg → rad
    return kYawSign * y; // 여기선 사실상 그냥 y
}

// [-π, π] 범위로 정규화
inline float wrapToPi(float a){
    const float PI = 3.14159265358979323846f;
    while (a >  PI) a -= 2.f*PI;
    while (a < -PI) a += 2.f*PI;
    return a;
}

// 6x6 블록 대각 회전행렬 T(Δψ) = diag(R, R, R)
inline Eigen::Matrix<float,6,6> makeYawTransform(float dpsi){
    float c = std::cos(dpsi), s = std::sin(dpsi);
    Eigen::Matrix<float,6,6> T = Eigen::Matrix<float,6,6>::Identity();
    // pos (x,y)
    T(0,0)=c; T(0,1)=-s; T(1,0)=s; T(1,1)=c;
    // vel (vx,vy)
    T(2,2)=c; T(2,3)=-s; T(3,2)=s; T(3,3)=c;
    // acc (ax,ay)
    T(4,4)=c; T(4,5)=-s; T(5,4)=s; T(5,5)=c;
    return T;
}

// 2D 회전행렬
inline Eigen::Matrix2f rot2(float dpsi){
    float c = std::cos(dpsi), s = std::sin(dpsi);
    Eigen::Matrix2f R; R << c, -s, s, c;
    return R;
}

// ★ 자차 요 변화량만큼 상태/공분산을 현재 바디 프레임으로 회전 정렬
// rotate_obb_yaw: 공통 기하에서 o의 OBB yaw가 이전 scan 값(miss predict)이면 true,
// 현재 검출(update)이면 false. legacy 분기는 원래 OBB yaw를 돌리지 않으므로 무관하다.
inline void compensate_ego_yaw(LiDARObjectInfo &o, float ego_yaw_now_deg, bool rotate_obb_yaw)
{
    // 1) 현재 yaw [deg] → rad 변환
    float yaw_now = ego_yaw_now_deg * M_PI / 180.0f;

    if(frame_boundary::Enabled() && o.tracking_frame_valid) {
        frame_boundary::AlignTrackingFrame(o,yaw_now,rotate_obb_yaw); return;
    }
    // 2) Δyaw = 현재 - 과거 ([-π, π]로 정규화)
    float dpsi = wrapToPi(yaw_now - o.f32_PrevYaw);

    if (fabs(dpsi) < 1e-8f) {
        o.f32_PrevYaw = yaw_now; // 그냥 업데이트만 하고 리턴
        return;
    }

    // 3) Δyaw만큼 상태/공분산 회전
    Eigen::Matrix<float,6,6> T = makeYawTransform(-dpsi);
    o.Eigen_KfState_CA = T * o.Eigen_KfState_CA;
    o.Eigen_KfP_CA     = T * o.Eigen_KfP_CA * T.transpose();

    // 4) 현재 yaw 저장 (rad)
    o.f32_PrevYaw = yaw_now;
}


std::pair<float, float> predictKalmanPosition(
        const LiDARObjectInfo& obj, float64 dt, float ego_yaw_now)
{
    if(frame_boundary::Enabled() && obj.tracking_frame_valid) {
        LiDARObjectInfo aligned{};
        aligned.Eigen_KfState_CA=obj.Eigen_KfState_CA;aligned.Eigen_KfP_CA=obj.Eigen_KfP_CA;
        aligned.tracking_frame_valid=true;aligned.tracking_height_input=obj.tracking_height_input;
        aligned.tracking_yaw_from_input=obj.tracking_yaw_from_input;
        aligned.tracking_origin_in_yaw=obj.tracking_origin_in_yaw;
        aligned.tracking_stamp_ns=obj.tracking_stamp_ns;aligned.tracking_ego_speed_mps=obj.tracking_ego_speed_mps;
        aligned.f32_PrevYaw=obj.f32_PrevYaw;aligned.f32_OBB_YawRad=obj.f32_OBB_YawRad;
        frame_boundary::AlignTrackingFrame(aligned,to_ccw_rad(ego_yaw_now));
        const auto& x=aligned.Eigen_KfState_CA;
        return {x(0)+dt*x(2)+.5*dt*dt*x(4),x(1)+dt*x(3)+.5*dt*dt*x(5)};
    }
    //1) Δψ = 현재요 - 이전프레임요
    const float yaw_now = to_ccw_rad(ego_yaw_now);                 // ★
    float dpsi = wrapToPi(yaw_now - obj.f32_PrevYaw);  

    //2) 좌표계 변환(수동 회전): -Δψ 로 정렬
    Eigen::Matrix<float,6,6> T = makeYawTransform(-dpsi);
    Eigen::Matrix<float,6,1> x0 = T * obj.Eigen_KfState_CA;
    
    // Eigen::Matrix<float,6,1> x0 = obj.Eigen_KfState_CA;
    // 3) CA 예측
    Eigen::Matrix<float,6,6> A = Eigen::Matrix<float,6,6>::Identity();
    A(0,2)=dt; A(1,3)=dt;
    A(0,4)=0.5f*dt*dt; A(1,5)=0.5f*dt*dt;
    A(2,4)=dt; A(3,5)=dt;

    Eigen::Matrix<float,6,1> x_pred = A * x0;
    return { x_pred(0), x_pred(1) };   // 현재 프레임 바디 기준 예측 위치
}
void kalman_filter_predict(LiDARObjectInfo &o, float64 dt, float ego_yaw_now)
{
    // // 0) 초기화 여부에 따라 PrevYaw 정리
    // if (!o.b_KfInitialized) {
    //     // 아직 초기화 전이면 기준만 맞춰두고 종료(보통 update에서 init)
    //     o.f32_PrevYaw = to_ccw_rad(ego_yaw_now);
    //     return;
    // }

    // 1) "현재 프레임 바디 좌표"로 회전 정렬 (수동 회전 내부에서 처리)
    //    이 호출 안에서 PrevYaw가 ego_yaw_now로 갱신됨
    //    miss track은 이전 scan의 OBB를 그대로 들고 있으므로 OBB yaw도 1회 재표현한다.
    compensate_ego_yaw(o, ego_yaw_now, true);

    // 2) 표준 CA 예측
    Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
    A(0,2)=dt; A(1,3)=dt;
    A(0,4)=0.5f*dt*dt; A(1,5)=0.5f*dt*dt;
    A(2,4)=dt; A(3,5)=dt;

    // 간단한 등방성 Q (튜닝 지점)
    Eigen::Matrix<float, 6, 6> Q = observation_model::ProcessCovariance(dt);

    o.Eigen_KfState_CA = A * o.Eigen_KfState_CA;
    o.Eigen_KfP_CA     = A * o.Eigen_KfP_CA * A.transpose() + Q;

    // 필요하다면 편의용 캐시(f32_CoreX/Y) 업데이트
    o.f32_CoreX = o.Eigen_KfState_CA(0);
    o.f32_CoreY = o.Eigen_KfState_CA(1);
    o.kf_elapsed_since_meas_s += dt;   // bookkeeping only (used by reacquire_elapsed_init)
    if (track_trace::Sink().WantsTrack(o.s32_Id)) {
        track_trace::Event("kf_predict_miss", [&](std::ostream& out) {
            out << ",\"id\":" << o.s32_Id << ",\"miss\":" << o.s32_MissCount << ",\"kfc\":" << o.s32_KfInitCount
                << ",\"dt\":" << dt << ",\"elapsed\":" << o.kf_elapsed_since_meas_s << ",\"x\":";
            track_trace::Arr(out, o.Eigen_KfState_CA, 6);
            out << ",\"Pdiag\":";
            track_trace::Arr(out, o.Eigen_KfP_CA.diagonal(), 6);
        });
    }
}

// 매칭된 트랙용 Update (회전 보정 → 예측 → 업데이트)
void kalman_filter_update(LiDARObjectInfo &obj,
                          float32 zx, float32 zy,     // 측정 (현재 프레임, 바디 기준)
                          float32 pred_x, float32 pred_y, // 헝가리안에서 쓴 예측 위치(바디 기준)
                          float64 dt,
                          float ego_yaw_now)          // 현재 자차 요각 [rad]
{
    // --- 초기화 ---
    if (!obj.b_KfInitialized) {
        obj.Eigen_KfState_CA << zx, zy, 0.f, 0.f, 0.f, 0.f;
        obj.Eigen_KfP_CA.setIdentity();
        obj.b_KfInitialized = true;
        obj.s32_KfInitCount = 1;
        obj.f32_CoreX = zx; obj.f32_CoreY = zy;
        obj.f32_PrevYaw = to_ccw_rad(ego_yaw_now);
        obj.kf_elapsed_since_meas_s = 0.;
        if (track_trace::Sink().WantsTrack(obj.s32_Id)) {
            track_trace::Event("kf_init", [&](std::ostream& out) {
                out << ",\"id\":" << obj.s32_Id << ",\"z\":[" << zx << ',' << zy << "],\"yaw_rad\":" << obj.f32_PrevYaw;
            });
        }
        return;
    }
    const bool trace = track_trace::Sink().WantsTrack(obj.s32_Id);
    const Eigen::Matrix<float, 6, 1> x_in = obj.Eigen_KfState_CA;
    const int kfc_in = obj.s32_KfInitCount;

    // --- 프레임 정렬(수동 회전) : 지난 프레임 기준 상태/공분산을 "현재 바디 프레임"으로 재표현 ---
    // 내부에서 PrevYaw가 ego_yaw_now로 갱신됨
    // ROS_INFO("[BEFORE] &obj=%p prev=%.6f", (void*)&obj, obj.f32_PrevYaw);
    // obj는 현재 검출이다. OBB yaw는 Objectinfo_for_km/stabilizeMatchedOBB가 이미 현재
    // 프레임으로 기록했으므로 KF 상태만 정렬한다(OBB yaw 이중 회전 금지).
    compensate_ego_yaw(obj, ego_yaw_now, false);
    const Eigen::Matrix<float, 6, 1> x_aligned = obj.Eigen_KfState_CA;

    // --- 두 번째 프레임: 속도 초기화 (pred_x/pred_y는 반드시 '현재 프레임 기준'이어야 함) ---
    // P0: z - pred spans every scan since the first observation (missed scans included) but is divided
    // by the last scan dt only. Option reacquire_elapsed_init divides by the real elapsed time.
    double init_elapsed = dt;
    if (obj.s32_KfInitCount == 1) {
        // Only a track that missed scans since its first observation takes the new path; without misses
        // the elapsed time is dt and the P0 expression is used unchanged.
        const bool elapsed_init = tracking_options::Get().reacquire_elapsed_init && obj.kf_elapsed_since_meas_s > 0.;
        if (elapsed_init) init_elapsed = obj.kf_elapsed_since_meas_s + dt;
        float32 vx_init = (zx - pred_x) / init_elapsed ;
        float32 vy_init = (zy - pred_y) / init_elapsed ;

        // P0 expression kept bit-exact when the option is off. With the option the position is set one
        // scan before z along v_init, so the CA step below lands on z (zero innovation) as in P0.
        if (elapsed_init) obj.Eigen_KfState_CA << zx - vx_init * dt, zy - vy_init * dt, vx_init, vy_init, 0.f, 0.f;
        else obj.Eigen_KfState_CA << pred_x, pred_y, vx_init, vy_init, 0.f, 0.f;
        obj.s32_KfInitCount = 2;
        // 필요하면 여기서 바로 return해도 되지만, 일반적으로 아래 예측+업데이트 계속 진행
    }

    // --- 모델/잡음 설정 (CA) ---
    Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
    A(0,2)=dt; A(1,3)=dt;
    A(0,4)=0.5f*dt*dt; A(1,5)=0.5f*dt*dt;
    A(2,4)=dt; A(3,5)=dt;

    Eigen::Matrix<float, 2, 6> H = Eigen::Matrix<float, 2, 6>::Zero();
    H(0,0)=1.f; H(1,1)=1.f;

    Eigen::Matrix<float, 6, 6> Q = observation_model::ProcessCovariance(dt);
    Eigen::Matrix2f R = obj.observation_R_valid ? obj.observation_R : Eigen::Matrix2f::Identity() * 0.01f;
    Eigen::Vector2f z(zx, zy);

    // --- 예측 ---
    Eigen::Matrix<float, 6, 1> x_pred = A * obj.Eigen_KfState_CA;
    Eigen::Matrix<float, 6, 6> P_pred = A * obj.Eigen_KfP_CA * A.transpose() + Q;

    // --- 업데이트 ---
    Eigen::Vector2f y = z - H * x_pred;
    Eigen::Matrix2f  S = H * P_pred * H.transpose() + R;
    Eigen::Matrix<float, 6, 2> K = P_pred * H.transpose() * S.inverse();

    obj.Eigen_KfState_CA = x_pred + K * y;
    // 수치안정성을 더 원하면 Joseph form 사용 가능:
    // auto I = Eigen::Matrix<float,6,6>::Identity();
    // obj.Eigen_KfP_CA = (I - K*H) * P_pred * (I - K*H).transpose() + K*R*K.transpose();
    obj.Eigen_KfP_CA = (Eigen::Matrix<float,6,6>::Identity() - K * H) * P_pred;
    if(obj.observation_R_valid) {
        const auto ikh=(Eigen::Matrix<float,6,6>::Identity()-K*H).eval();
        obj.Eigen_KfP_CA=ikh*P_pred*ikh.transpose()+K*R*K.transpose();
    }

    // --- 캐시 ---
    obj.f32_CoreX = obj.Eigen_KfState_CA(0);
    obj.f32_CoreY = obj.Eigen_KfState_CA(1);
    if (trace) {
        track_trace::Event("kf_update", [&](std::ostream& out) {
            out << ",\"id\":" << obj.s32_Id << ",\"kfc_in\":" << kfc_in << ",\"dt\":" << dt
                << ",\"elapsed_before\":" << obj.kf_elapsed_since_meas_s << ",\"init_elapsed\":" << init_elapsed
                << ",\"z\":[" << zx << ',' << zy << "],\"pred\":[" << pred_x << ',' << pred_y << "],\"x_in\":";
            track_trace::Arr(out, x_in, 6);
            out << ",\"measurement_R\":";track_trace::Mat2(out,R,2,2);
            out << ",\"x_aligned\":"; track_trace::Arr(out, x_aligned, 6);
            out << ",\"x_prior\":"; track_trace::Arr(out, x_pred, 6);
            out << ",\"P_prior_diag\":"; track_trace::Arr(out, P_pred.diagonal(), 6);
            out << ",\"innov\":"; track_trace::Arr(out, y, 2);
            out << ",\"S\":"; track_trace::Mat2(out, S, 2, 2);
            out << ",\"K\":"; track_trace::Mat2(out, K, 6, 2);
            out << ",\"x_post\":"; track_trace::Arr(out, obj.Eigen_KfState_CA, 6);
            out << ",\"P_post_diag\":"; track_trace::Arr(out, obj.Eigen_KfP_CA.diagonal(), 6);
        });
    }
    obj.kf_elapsed_since_meas_s = 0.;
}

//이거는 imm

// std::pair<float, float> predictKalmanPosition(
//     const LiDARObjectInfo& obj, float64 dt, float ego_yaw_now) {
//     // 1) Δψ = 현재요 - 이전프레임요
//     const float yaw_now = to_ccw_rad(ego_yaw_now);  // 현재 자차 Yaw
//     float dpsi = wrapToPi(yaw_now - obj.f32_PrevYaw);  // ΔYaw 계산 (현재와 이전)

//     // 2) 회전 보정 (두 모델(CV, CA) 회전 적용)
//     Eigen::Matrix<float, 6, 6> T = makeYawTransform(-dpsi);  // 회전 행렬 (ΔYaw로)
    
//     // 두 모델(CV, CA) 회전 보정
//     Eigen::Matrix<float, 6, 1> x0_CV = T * obj.Eigen_KfState_CV;  // 회전된 CV 상태
//     Eigen::Matrix<float, 6, 1> x0_CA = T * obj.Eigen_KfState_CA;  // 회전된 CA 상태

//     // 3) 두 모델 예측
//     Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
//     A(0, 2) = dt; A(1, 3) = dt;
//     A(0, 4) = 0.5f * dt * dt; A(1, 5) = 0.5f * dt * dt;
//     A(2, 4) = dt; A(3, 5) = dt;

//     // 두 모델 예측 (CV, CA)
//     Eigen::Matrix<float, 6, 1> x_pred_CV = A * x0_CV;
//     Eigen::Matrix<float, 6, 1> x_pred_CA = A * x0_CA;

//     // 4) 모델 가중치 (확률 기반으로)
//     float w_cv = obj.vec_Mu(0);  // CV 모델 가중치
//     float w_ca = obj.vec_Mu(1);  // CA 모델 가중치

//     // 5) 두 모델 예측값을 가중치 기반으로 혼합
//     Eigen::Matrix<float, 6, 1> x_pred_mixed = w_cv * x_pred_CV + w_ca * x_pred_CA;

//     // 예측된 혼합 위치 반환 (혼합된 예측값 사용)
//     return { x_pred_mixed(0), x_pred_mixed(1) };  // 혼합된 위치 반환
// }

// void kalman_filter_predict(LiDARObjectInfo &o, float64 dt, float ego_yaw_now) {
//     // 0) 초기화 여부에 따라 PrevYaw 정리

//     // 1) 회전 보정 (두 모델(CV, CA) 모두 회전 적용)
//     compensate_ego_yaw(o, ego_yaw_now);  // 회전 보정 함수에서 두 모델(CV/CA) 모두 회전

//     // 2) 두 모델 예측 (CV, CA)
//     Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
//     A(0, 2) = dt; A(1, 3) = dt;
//     A(0, 4) = 0.5f * dt * dt; A(1, 5) = 0.5f * dt * dt;
//     A(2, 4) = dt; A(3, 5) = dt;

//     Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity() * 0.003f;
//     Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.01f;
//     // 두 모델 예측 (CV, CA)
//     Eigen::Matrix<float, 6, 1> x_pred_CV = A * o.Eigen_KfState_CV;
//     Eigen::Matrix<float, 6, 6> P_pred_CV = A * o.Eigen_KfP_CV * A.transpose() + Q;  // CV 공분산 예측

//     Eigen::Matrix<float, 6, 1> x_pred_CA = A * o.Eigen_KfState_CA;
//     Eigen::Matrix<float, 6, 6> P_pred_CA = A * o.Eigen_KfP_CA * A.transpose() + Q;  // CA 공분산 예측

//     // 예측된 위치 혼합
//     float w_cv = o.vec_Mu(0);
//     float w_ca = o.vec_Mu(1);

//     Eigen::Matrix<float, 6, 1> x_pred_mixed = w_cv * x_pred_CV + w_ca * x_pred_CA;

//     // 예측된 위치 갱신
//     o.Eigen_KfState_CV = x_pred_CV;  // CV 모델 상태 업데이트
//     o.Eigen_KfP_CV = P_pred_CV;

//     o.Eigen_KfState_CA = x_pred_CA;  // CA 모델 상태 업데이트
//     o.Eigen_KfP_CA = P_pred_CA;

//     // 혼합된 위치 저장
//     o.f32_CoreX = x_pred_mixed(0);  // 혼합된 위치 사용
//     o.f32_CoreY = x_pred_mixed(1);  // 혼합된 위치 사용
// }

// void kalman_filter_update_CV(LiDARObjectInfo &obj,
//                               float32 zx, float32 zy,
//                               float32 f32_Pred_x, float32 f32_Pred_y,
//                               float64 f64_Dt,
//                               float ego_yaw_now) {
//     // --- 초기화 ---
//     if (!obj.b_KfInitialized) {
//         obj.Eigen_KfState_CV << zx, zy, 0.f, 0.f, 0.f, 0.f;
//         obj.Eigen_KfP_CV.setIdentity();
//         obj.b_KfInitialized = true;
//         obj.s32_KfInitCount = 1;
//         obj.f32_CoreX = zx; obj.f32_CoreY = zy;
//         obj.f32_PrevYaw = to_ccw_rad(ego_yaw_now);
//         return;
//     }

//     // --- 회전 보정 ---
//     compensate_ego_yaw(obj, ego_yaw_now);  // 회전 보정

//     // --- 두 번째 프레임: 속도 초기화 ---
//     if (obj.s32_KfInitCount == 1) {
//         float32 f32_Vxinit = (zx - f32_Pred_x) / f64_Dt;
//         float32 f32_Vyinit = (zy - f32_Pred_y) / f64_Dt;
//         obj.Eigen_KfState_CV << f32_Pred_x, f32_Pred_y, f32_Vxinit, f32_Vyinit, 0.f, 0.f;
//         obj.s32_KfInitCount = 2;
//     }

//     // --- IMM 초기화 (두 번째 프레임부터) ---
//     if (obj.s32_KfInitCount == 2 && !obj.b_IMMInit) {
//         float vx_init = (zx - f32_Pred_x) / f64_Dt;
//         float vy_init = (zy - f32_Pred_y) / f64_Dt;
//         IMM_Init_Lite(obj, zx, zy, vx_init, vy_init);  // IMM 초기화
//     }

//     Eigen::Matrix<float, 2, 6> H = Eigen::Matrix<float, 2, 6>::Zero();
//     H(0,0)=1.f; H(1,1)=1.f;
//     Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity() * 0.003f;
//     Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.01f;

//     // --- IMM 업데이트 ---
//     if (obj.b_IMMInit) {
//         IMMParams prm;  // IMM 파라미터 설정
//         if (obj.s32_MissCount == 0) {
//             IMM_Step_Lite(obj, Eigen::Vector2f(zx, zy), f64_Dt, H, R);  // 관측 있음
//         } else {
//             IMM_PredictOnly_Lite(obj, f64_Dt);   // 관측 X (예측만)
//         }
//     }

//     // --- 기존 예측 및 보정 로직 ---
//     Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
//     A(0, 2) = f64_Dt;
//     A(1, 3) = f64_Dt;


//     Eigen::Vector2f z(zx, zy);
//     Eigen::Matrix<float, 6, 6> P_pred = A * obj.Eigen_KfP_CV * A.transpose() + Q;
//     Eigen::Matrix<float, 6, 1> x_pred = A * obj.Eigen_KfState_CV;

//     // --- 업데이트 ---
//     Eigen::Vector2f y_residual = z - H * x_pred;
//     Eigen::Matrix2f S = H * P_pred * H.transpose() + R;
//     Eigen::Matrix<float, 6, 2> K = P_pred * H.transpose() * S.inverse();

//     obj.Eigen_KfState_CV = x_pred + K * y_residual;
//     obj.Eigen_KfP_CV = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * P_pred;

//     // --- 결과 저장 ---
//     obj.f32_CoreX = obj.Eigen_KfState_CV(0);
//     obj.f32_CoreY = obj.Eigen_KfState_CV(1);
// }

// void kalman_filter_update_CA(LiDARObjectInfo &obj,
//                               float32 zx, float32 zy,
//                               float32 f32_Pred_x, float32 f32_Pred_y,
//                               float64 f64_Dt,
//                               float ego_yaw_now) {
//     // --- 초기화 ---
//     if (!obj.b_KfInitialized) {
//         obj.Eigen_KfState_CA << zx, zy, 0.f, 0.f, 0.f, 0.f;
//         obj.Eigen_KfP_CA.setIdentity();
//         obj.b_KfInitialized = true;
//         obj.s32_KfInitCount = 1;
//         obj.f32_CoreX = zx; obj.f32_CoreY = zy;
//         obj.f32_PrevYaw = to_ccw_rad(ego_yaw_now);
//         return;
//     }

//     // --- 회전 보정 ---
//     compensate_ego_yaw(obj, ego_yaw_now);  // 회전 보정

//     // --- 두 번째 프레임: 속도 초기화 ---
//     if (obj.s32_KfInitCount == 1) {
//         float32 f32_Vxinit = (zx - f32_Pred_x) / f64_Dt;
//         float32 f32_Vyinit = (zy - f32_Pred_y) / f64_Dt;
//         obj.Eigen_KfState_CA << f32_Pred_x, f32_Pred_y, f32_Vxinit, f32_Vyinit, 0.f, 0.f;
//         obj.s32_KfInitCount = 2;
//     }

//     // --- IMM 초기화 (두 번째 프레임부터) ---
//     if (obj.s32_KfInitCount == 2 && !obj.b_IMMInit) {
//         float vx_init = (zx - f32_Pred_x) / f64_Dt;
//         float vy_init = (zy - f32_Pred_y) / f64_Dt;
//         IMM_Init_Lite(obj, zx, zy, vx_init, vy_init);  // IMM 초기화
//     }

//     Eigen::Matrix<float, 2, 6> H = Eigen::Matrix<float, 2, 6>::Zero();
//     H(0,0)=1.f; H(1,1)=1.f;
//     Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity() * 0.003f;
//     Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.01f;
    
//     // --- IMM 업데이트 ---
//     if (obj.b_IMMInit) {
//         IMMParams prm;  // IMM 파라미터 설정
//         if (obj.s32_MissCount == 0) {
//             IMM_Step_Lite(obj, Eigen::Vector2f(zx, zy), f64_Dt, H, R);  // 관측 있음
//         } else {
//             IMM_PredictOnly_Lite(obj, f64_Dt);   // 관측 X (예측만)
//         }
//     }

//     // --- 예측 및 보정 (기존의 CA 예측/업데이트 로직) ---
//     Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Identity();
//     A(0, 2) = f64_Dt;
//     A(1, 3) = f64_Dt;

//     Eigen::Vector2f z(zx, zy);
//     Eigen::Matrix<float, 6, 6> P_pred = A * obj.Eigen_KfP_CA * A.transpose() + Q;
//     Eigen::Matrix<float, 6, 1> x_pred = A * obj.Eigen_KfState_CA;

//     // --- 업데이트 ---
//     Eigen::Vector2f y_residual = z - H * x_pred;
//     Eigen::Matrix2f S = H * P_pred * H.transpose() + R;
//     Eigen::Matrix<float, 6, 2> K = P_pred * H.transpose() * S.inverse();

//     obj.Eigen_KfState_CA = x_pred + K * y_residual;
//     obj.Eigen_KfP_CA = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * P_pred;

//     // --- 결과 저장 ---
//     obj.f32_CoreX = obj.Eigen_KfState_CA(0);
//     obj.f32_CoreY = obj.Eigen_KfState_CA(1);
// }
