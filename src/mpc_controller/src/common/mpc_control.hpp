#pragma once

// 제어 입력
struct MPCControl {
    double delta = 0.0;    // 조향각 [rad]
    double accel = 0.0;    // 가속도 [m/s^2]
};
