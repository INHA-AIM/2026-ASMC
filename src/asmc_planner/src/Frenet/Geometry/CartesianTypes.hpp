// Cartesian(전역 좌표계) 상태/경로 표현. Tracking Controller(MPC)로 넘어가는
// 경계 타입이다.

#ifndef PLANNER_COMMON_CARTESIAN_TYPES_HPP
#define PLANNER_COMMON_CARTESIAN_TYPES_HPP

#include <vector>

// =========================================================
// 한 시점(instant)의 Cartesian 상태.
// 필드명은 CartesianPath와 맞춰 x, y, yaw, kappa, v, a로 통일.
// =========================================================

struct CartesianState {
    double x, y;
    double yaw;     // theta_x
    double kappa;   // kappa_x
    double v;       // v_x
    double a;       // a_x
};

// =========================================================
// Frenet -> Cartesian 변환 결과 (Tracking Controller 입력)
// 이 구조체가 MPC Controller로 넘어감.
// =========================================================

struct CartesianPath {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> kappa;
    std::vector<double> v;
    std::vector<double> a;
};

#endif  // PLANNER_COMMON_CARTESIAN_TYPES_HPP
