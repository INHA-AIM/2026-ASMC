#ifndef IMMFilter_HPP
#define IMMFilter_HPP


#include <Global/Global.hpp>

void IMM_Init_Lite(LiDARObjectInfo& o, float zx, float zy, float vx0, float vy0);

// IMM 예측+업데이트 함수
void IMM_Step_Lite(LiDARObjectInfo& o, const Eigen::Vector2f& z, float dt, const Eigen::Matrix<float,2,6>& H, const Eigen::Matrix2f& R, float chi2_gate = 9.21f);

// IMM 예측만 하는 함수 (관측이 없을 때)
void IMM_PredictOnly_Lite(LiDARObjectInfo& o, float dt);

// IMM 파라미터 구조체 (시작 시 기본 값)
struct IMMParams {
    float gamma_cv   = 0.8f;   // CV에서 a 감쇠(0.6~0.9)
    float sigma_a_cv = 1.0f;   // CV 가속도 표준편차 [m/s^2]
    float q_jerk_ca  = 1e-2f;  // CA 저크 파워
    float r_xy       = 0.01f;  // 위치 측정 잡음 분산
    float chi2_gate  = 9.21f;  // m=2, 99% 카이제곱 게이트
};


#endif // IMMFilter_HPP
