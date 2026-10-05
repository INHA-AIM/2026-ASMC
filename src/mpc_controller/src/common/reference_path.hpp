#pragma once

#include <cstddef>
#include <vector>

// 참조 경로 (현재 ego 위치 기준 window)
struct ReferencePath {
    std::vector<double> x_ref;     // global X [m]
    std::vector<double> y_ref;     // global Y [m]
    std::vector<double> yaw_ref;   // global yaw [rad]
    std::vector<double> v_ref;     // 목표 속도 [m/s]
    // MPC time grid로 최종 리샘플된 v_ref의 시간 미분으로부터 유도한
    // nominal longitudinal acceleration reference [m/s^2]. MPCControl.accel과
    // 동일하게 양수는 가속, 음수는 제동이다. Frenet CartesianPath.a가 아니다.
    std::vector<double> a_ref;
    std::vector<double> k_ref;     // 곡률 [1/m]

    std::size_t size() const { return x_ref.size(); }
    bool   empty() const { return x_ref.empty(); }
    void   clear() {
        x_ref.clear(); y_ref.clear();
        yaw_ref.clear(); v_ref.clear(); a_ref.clear(); k_ref.clear();
    }
};
