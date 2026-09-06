#pragma once

// 차량 상태 (MPC State)
struct MPCState {
    double x = 0.0;        // X 좌표 [m] (global ENU)
    double y = 0.0;        // Y 좌표 [m] (global ENU)
    double yaw = 0.0;      // Yaw 각도 [rad]
    double vx = 0.0;       // 종방향 속도 [m/s]
};
