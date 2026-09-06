#pragma once

#include <cstddef>
#include <vector>

// 참조 경로 (현재 ego 위치 기준 window)
struct ReferencePath {
    std::vector<double> x_ref;     // global X [m]
    std::vector<double> y_ref;     // global Y [m]
    std::vector<double> yaw_ref;   // global yaw [rad]
    std::vector<double> v_ref;     // 목표 속도 [m/s]
    std::vector<double> k_ref;     // 곡률 [1/m]

    std::size_t size() const { return x_ref.size(); }
    bool   empty() const { return x_ref.empty(); }
    void   clear() {
        x_ref.clear(); y_ref.clear();
        yaw_ref.clear(); v_ref.clear(); k_ref.clear();
    }
};
