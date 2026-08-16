#include "IMMFilter.hpp" 


// void build_FQ_CV(float dt, float gamma, float sigma_a,
//                  Eigen::Matrix<float,6,6>& F,
//                  Eigen::Matrix<float,6,6>& Q) {
//     F = Eigen::Matrix<float,6,6>::Identity();
//     F(0,2) = dt; F(1,3) = dt;
//     F(0,4) = 0.5f*dt*dt; F(1,5) = 0.5f*dt*dt;
//     F(2,4) = dt; F(3,5) = dt;
//     Q = Eigen::Matrix<float,6,6>::Identity() * sigma_a;
// }

// void build_FQ_CA(float dt, float q_jerk,
//                  Eigen::Matrix<float,6,6>& F,
//                  Eigen::Matrix<float,6,6>& Q) {
//     F = Eigen::Matrix<float,6,6>::Identity();
//     F(0,2) = dt; F(1,3) = dt;
//     F(0,4) = 0.5f*dt*dt; F(1,5) = 0.5f*dt*dt;
//     F(2,4) = dt; F(3,5) = dt;
//     Q = Eigen::Matrix<float,6,6>::Identity() * q_jerk;
// }

// float LogLik2D(const Eigen::Vector2f& v, const Eigen::Matrix2f& S) {
//     float detS = S.determinant();
//     if(detS < 1e-6f) return -1e6f;
//     float exponent = -0.5f * v.transpose() * S.inverse() * v;
//     return exponent - 0.5f*std::log(detS);
// }

// // ---------------- IMM 초기화 함수 ----------------
// void IMM_Init_Lite(LiDARObjectInfo& o, float zx, float zy, float vx0, float vy0) {
//     if (!o.b_KfInitialized) {
//         o.Eigen_KfState_CV << zx, zy, 0.f, 0.f, 0.f, 0.f;
//         o.Eigen_KfP_CV.setIdentity();
//         o.b_KfInitialized = true;
//         o.s32_KfInitCount = 1;
//         o.f32_CoreX = zx; 
//         o.f32_CoreY = zy;
//         o.f32_PrevYaw = to_ccw_rad(ego_yaw_now);
//         return;
//     }
//     // CV, CA 모델 초기화
//     o.Eigen_KfState_CV << zx, zy, vx0, vy0, 0.f, 0.f;
//     o.Eigen_KfP_CV.setIdentity();
//     o.Eigen_KfState_CA  << zx, zy, vx0, vy0, 0.f, 0.f;
//     o.Eigen_KfP_CA.setIdentity();

//     o.vec_Mu << 0.5f, 0.5f;  // 초기 모델 가중치
//     o.mat_Pi << 0.95f, 0.05f,
//                 0.05f, 0.95f;  // 전이확률 초기화
//     o.b_IMMInit = true;
// }

// // ---------------- IMM 예측 + 업데이트 함수 ----------------
// void IMM_Step_Lite(LiDARObjectInfo& o, const Eigen::Vector2f& z, float dt, const Eigen::Matrix<float,2,6>& H, 
// const Eigen::Matrix2f& R, float chi2_gate,float ego_yaw_now) {
    
//     // Interaction (혼합 사전확률 c, 혼합 초기 x0, P0)
//     float c0 = o.mat_Pi(0,0)*o.vec_Mu(0) + o.mat_Pi(0,1)*o.vec_Mu(1);
//     float c1 = o.mat_Pi(1,0)*o.vec_Mu(0) + o.mat_Pi(1,1)*o.vec_Mu(1);

//     Eigen::Matrix<float,6,1> x0_cv = o.mat_Pi(0,0) * o.Eigen_KfState_CV + o.mat_Pi(0,1) * o.Eigen_KfState_CA;
//     Eigen::Matrix<float,6,1> x0_ca = o.mat_Pi(1,0) * o.Eigen_KfState_CV + o.mat_Pi(1,1) * o.Eigen_KfState_CA;
    
//     Eigen::Matrix<float,6,6> P0_cv = o.mat_Pi(0,0) * o.Eigen_KfP_CV + o.mat_Pi(0,1) * o.Eigen_KfP_CA;
//     Eigen::Matrix<float,6,6> P0_ca = o.mat_Pi(1,0) * o.Eigen_KfP_CV + o.mat_Pi(1,1) * o.Eigen_KfP_CA;

//     // 예측
//     Eigen::Matrix<float,6,6> Fcv, Qcv, Fca, Qca;
//     build_FQ_CV(dt, 0.8f, 1.0f, Fcv, Qcv); // CV 예측
//     build_FQ_CA(dt, 1e-2f, Fca, Qca);     // CA 예측
    
//     Eigen::Matrix<float,6,1> xcv_pred = Fcv * x0_cv;
//     Eigen::Matrix<float,6,6> Pcv_pred = Fcv * P0_cv * Fcv.transpose() + Qcv;

//     Eigen::Matrix<float,6,1> xca_pred = Fca * x0_ca;
//     Eigen::Matrix<float,6,6> Pca_pred = Fca * P0_ca * Fca.transpose() + Qca;

//     // 우도 계산
//     Eigen::Vector2f v_cv = z - H * xcv_pred;
//     Eigen::Matrix2f  S_cv = H * Pcv_pred * H.transpose() + R;

//     Eigen::Vector2f v_ca = z - H * xca_pred;
//     Eigen::Matrix2f  S_ca = H * Pca_pred * H.transpose() + R;

//     // 우도 기반 가중치 갱신
//     float loglik_cv = LogLik2D(v_cv, S_cv);
//     float loglik_ca = LogLik2D(v_ca, S_ca);

//     if (v_cv.dot(v_cv) > chi2_gate) loglik_cv -= 20.f;
//     if (v_ca.dot(v_ca) > chi2_gate) loglik_ca -= 20.f;

//     float logw_cv = std::log(c0) + loglik_cv;
//     float logw_ca = std::log(c1) + loglik_ca;

//     float m = std::max(logw_cv, logw_ca);
//     float e_cv = std::exp(logw_cv - m), e_ca = std::exp(logw_ca - m);

//     o.vec_Mu(0) = e_cv / (e_cv + e_ca);
//     o.vec_Mu(1) = e_ca / (e_cv + e_ca);

//     // 모델 업데이트
//     o.Eigen_KfState_CV = xcv_pred; o.Eigen_KfP_CV = Pcv_pred;
//     o.Eigen_KfState_CA = xca_pred; o.Eigen_KfP_CA = Pca_pred;
    
//     kalman_filter_update_CV(o, z(0), z(1),
//                         xcv_pred(0), xcv_pred(1),
//                         dt, ego_yaw_now); // CV 업데이트
//     kalman_filter_update_CA(o, z(0), z(1),
//                         xca_pred(0), xca_pred(1),
//                         dt, ego_yaw_now);  // CA 업데이트

//     // 혼합 출력(소프트)
//     Eigen::Matrix<float,6,1> x_hat = o.vec_Mu(0)*o.Eigen_KfState_CV + o.vec_Mu(1)*o.Eigen_KfState_CA;
//     o.f32_CoreX = x_hat(0); o.f32_CoreY = x_hat(1);
// }

// // ---------------- IMM 예측만 하는 함수 ----------------
// void IMM_PredictOnly_Lite(LiDARObjectInfo& o, float dt,float ego_yaw_now) {
//     // 예측만 하는 부분
//     kalman_filter_predict(o, dt, ego_yaw_now); 
    
// }