#pragma once

// MPC 파라미터
struct MPCParams {
    // 예측 파라미터
    int    horizon         = 15;     // 예측 horizon (steps)
    double dt              = 0.1;    // 샘플링 간격 [s]
    int    max_iterations  = 25;     // 솔버 최대 반복
    double wheelbase       = 3.0;    // 축거 [m]
    double control_frequency = 20.0; // 제어 주기 [Hz]

    // 제약 조건
    double steering_max     = 0.6109;  // 35deg [rad]
    double steering_rate_max= 1.0;     // [rad/s]
    double accel_min        = -5.0;    // [m/s^2]
    double accel_max        = 3.0;     // [m/s^2]
    // MORAI longlCmdType=1의 brake=1.0에 대응하는 차량 최대 감속도.
    // MPC 제약(accel_min)과 페달 변환 캘리브레이션을 분리한다.
    double brake_full_decel = 8.9;     // [m/s^2]
    double vel_min          = 0.0;     // [m/s]
    double vel_max          = 30.0;    // [m/s]

    // 모드 독립 정지 데드락 감지/복구. 목표 경로는 전진하는데 실제 차량만
    // 멈춘 상태가 지속될 때 solver의 제동 warm-start를 비우고 제한된 재출발을 한다.
    double stuck_ego_speed = 0.1;       // [m/s]
    double stuck_target_speed = 0.3;    // [m/s]
    double stuck_min_path_progress = 0.5; // [m]
    double stuck_timeout = 1.0;         // [s]
    double stuck_recovery_duration = 0.75; // [s]
    double stuck_launch_accel = 0.5;    // [m/s^2]

    // 비용 가중치
    double weight_path_error    = 5.0;
    double weight_heading_error = 2.0;
    double weight_speed_error   = 0.3;
    double weight_control       = 0.05;
    double weight_control_rate  = 0.5;
    // steer와 독립된 acceleration 변화율 가중치
    double weight_control_rate_accel = 4.5;
    double weight_terminal      = 5.0;   // 마지막 상태 가중치

    // 솔버 hyper-params
    double lr_init      = 0.05;  // 초기 학습률
    double lr_min       = 1e-4;  // 최소 학습률
    double convergence_eps = 1e-3;
    int    line_search_steps = 6;
};
