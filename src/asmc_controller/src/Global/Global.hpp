#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

struct MPCControl {
    double delta = 0.0;    // 조향각 [rad]
    double accel = 0.0;    // 가속도 [m/s^2]
};

struct MPCParams {
    // 예측 파라미터
    int    horizon         = 15;     // 예측 horizon (steps)
    double dt              = 0.1;    // 샘플링 간격 [s]
    int    max_iterations  = 25;     // 솔버 최대 반복
    double wheelbase       = 3.0;    // 축거 [m]
    // MORAI 실측 kappa = gain * tan(delta) / wheelbase 보정계수.
    double steering_curvature_gain = 1.0;
    double control_frequency = 20.0; // 제어 주기 [Hz]

    // 제약 조건
    double steering_max     = 0.6981317008;  // 40deg [rad]
    double steering_rate_max= 1.0;     // [rad/s]
    double accel_min        = -5.0;    // [m/s^2]
    double accel_max        = 3.0;     // [m/s^2]
    // MORAI longlCmdType=1의 brake=1.0에 대응하는 차량 최대 감속도.
    // MPC 제약(accel_min)과 페달 변환 캘리브레이션을 분리한다.
    double brake_full_decel = 8.9;     // [m/s^2]
    // MORAI IONIQ 5 측정 기반 pedal -> 실제 가감속 LUT. 출력 변환에서는
    // 요구 가감속을 response 축에서 역보간해 pedal 값을 구한다.
    bool use_calibrated_pedal_mapping = false;
    std::vector<double> accel_pedal_lut;
    std::vector<double> accel_response_lut;
    std::vector<double> brake_pedal_lut;
    std::vector<double> brake_response_lut;
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
    double weight_contour_error = 6.0;
    double weight_lag_error     = 0.5;
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

struct MPCState {
    double x = 0.0;        // X 좌표 [m] (global ENU)
    double y = 0.0;        // Y 좌표 [m] (global ENU)
    double yaw = 0.0;      // Yaw 각도 [rad]
    double vx = 0.0;       // 종방향 속도 [m/s]
};

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

struct MPCResult {
    MPCControl control;
    std::vector<MPCState>   predicted_states;
    std::vector<MPCControl> controls;
    double cost = 0.0;
    bool   success = false;
    std::string solver_msg;
};

struct PlannedPath {
    std::vector<double> x, y, yaw, kappa, v, a;
    // a는 Frenet CartesianPath의 kinematic/diagnostic acceleration을 그대로
    // 보존한다. MPC actuator reference는 이 배열이 아니라 ToReferencePath가
    // 최종 destination grid의 v_ref를 미분해 만드는 ReferencePath::a_ref다.
    double dt = 0.0;      // 샘플 간격 [s] (frenet_planner.sample_dt())
    double d = 0.0;       // 진단용: 차선 중앙 기준 횡방향 오프셋
    double d_dot = 0.0;   // 진단용: 횡방향 속도

    std::size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }
};

struct TrackingError {
    double contour = 0.0;  // left-normal error: [-sin(psi), cos(psi)] dot dp
    double lag = 0.0;      // tangent error: [cos(psi), sin(psi)] dot dp
};

struct CostBreakdown {
    double contour = 0.0;
    double lag = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    // control/control_rate 총합 안에 포함된 종방향 부분. total()에는 다시
    // 더하지 않으며 runtime 진단에서 acceleration reference 추종과 rate
    // smoothing의 상대 크기를 분리해 보기 위한 값이다.
    double acceleration_control = 0.0;
    double acceleration_rate = 0.0;
    double control = 0.0;
    double control_rate = 0.0;
    double terminal = 0.0;
    double total() const {
        return contour + lag + heading + speed + control + control_rate +
               terminal;
    }
};

enum class ReferenceValidationStatus {
    OK = 0,
    EMPTY,
    SIZE_MISMATCH,
    NON_FINITE,
    VELOCITY_OUT_OF_RANGE,
    CURVATURE_OUT_OF_RANGE,
    POSITION_DISCONTINUITY,
};

struct ReferenceValidationResult {
    ReferenceValidationStatus status = ReferenceValidationStatus::EMPTY;
    // 실패를 유발한 첫 인덱스. 진단/테스트용이며 OK일 때는 의미가 없다.
    std::size_t failed_index = 0;

    bool ok() const { return status == ReferenceValidationStatus::OK; }
};

struct LinearizedModel {
    // A: 4×4  (∂f/∂x, 상태에 대한 편미분)
    std::array<std::array<double, 4>, 4> A{};
    // B: 4×2  (∂f/∂u, 입력에 대한 편미분)
    std::array<std::array<double, 2>, 4> B{};
    // c: 4×1  affine 보정항  c = f(x̄,ū) - A*x̄ - B*ū
    std::array<double, 4> c{};
};

enum class StuckEvent {
        NONE,
        RECOVERY_STARTED,
        RECOVERY_COMPLETE,
    };

struct MPCOutput {
        bool success = false;
        MPCControl control;
        const char* failure_reason = nullptr;

        StuckEvent stuck_event = StuckEvent::NONE;
        double stuck_event_ego_v = 0.0;
        double stuck_event_max_ref_v = 0.0;
        double stuck_event_path_progress = 0.0;

        // Final nonlinear predicted trajectory/control sequence diagnostics.
        // These describe the accepted solution, never the warm-start guess.
        CostBreakdown cost;
        double first_reference_curvature = 0.0;
        double first_feedforward_steering = 0.0;
        double first_reference_speed = 0.0;
        // Raw derivative of final MPC-grid v_ref and the actuator-bounded
        // value actually used by solver/cost, respectively.
        double first_reference_acceleration = 0.0;
        double first_used_reference_acceleration = 0.0;
        double current_speed = 0.0;
        double first_speed_error = 0.0;
        double first_acceleration_residual = 0.0;
    };
