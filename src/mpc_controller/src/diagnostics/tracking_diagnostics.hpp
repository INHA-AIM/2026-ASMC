#pragma once

#include <cstdio>
#include <string>

#include "common/mpc_control.hpp"
#include "common/mpc_params.hpp"
#include "common/mpc_state.hpp"
#include "common/reference_path.hpp"
#include "control/mpc_controller.hpp"

// MPC-D1: 20Hz 상세 추종 진단(디버그 전용). solver/warm-start/reference/
// constraints/발행되는 CtrlCmd 중 어느 것도 바꾸지 않는다 - 이미 계산된
// 값(ReferencePath, MpcController::Output, VehicleState)을 읽어서 정리만
// 한다. ROS 타입에 의존하지 않는 순수 계산이라 mpc_core 경계를 넘지 않고,
// node 쪽에서 타이밍(ros::Time 등)만 double로 변환해 채운다.
struct TrackingDiagnosticSample {
    // timing / provenance
    double ros_time = 0.0;
    double vehicle_state_stamp = 0.0;
    double trajectory_stamp = 0.0;
    double vehicle_state_age_s = 0.0;
    double trajectory_age_s = 0.0;
    double trajectory_vehicle_time_delta_s = 0.0;

    // vehicle state
    double ego_x = 0.0;
    double ego_y = 0.0;
    double ego_yaw = 0.0;
    double ego_speed = 0.0;
    double yaw_rate_meas = 0.0;
    double ego_accel = 0.0;

    // reference preview (curvature)
    double kappa_ref_0s = 0.0;
    double kappa_ref_02s = 0.0;
    double kappa_ref_05s = 0.0;
    double kappa_ref_10s = 0.0;
    double kappa_ref_peak = 0.0;

    // steering feed-forward (same SteeringFeedforward() formula, no
    // duplicated calculation - values passed in already computed)
    double ff_0s = 0.0;
    double ff_02s = 0.0;
    double ff_05s = 0.0;
    double ff_10s = 0.0;
    double ff_peak = 0.0;

    // warm-start (curvature-aware residual rebase, values read from
    // MpcController::Output as-is)
    bool warm_used_rebase = false;
    double warm_raw_delta0 = 0.0;
    double warm_prev_ff_shift0 = 0.0;
    double warm_residual0 = 0.0;
    double warm_current_ff0 = 0.0;
    double warm_rebased_delta0 = 0.0;
    double warm_projected_delta0 = 0.0;

    // optimized / final command
    //
    // 현재 solveMPC 구현(mpc_solver.cpp)에서는 line-search 루프의 매 후보가
    // projectControls()를 거친 뒤에야 cost 비교/채택되므로, 최종
    // result.control은 "최적화 결과"이자 이미 "투영된 값"이다 - 반복 종료
    // 후 이 둘을 나누는 별도 pre-projection 최종값이 존재하지 않는다(warm
    // start 자체의 pre-projection 값은 [MPC-WARM]의 warm_projected_delta0가
    // 이미 담당한다 - 그건 반복 "이전" 초기 guess다). 그래서 여기서는
    // optimized_delta0/projected_delta0를 별도로 재구성하지 않고 실제
    // 코드에서 같은 값임을 그대로 반영한다(둘 다 optimized_projected_delta0
    // 하나로 기록, 의미 없는 중복 필드를 만들지 않는다).
    double optimized_projected_delta0 = 0.0;
    // ToRosControlCommand가 publish 직전 적용하는 최종 clip(±steering_max)
    // 결과 - production 코드의 그 clip을 그대로 재현한다(새 공식 없음).
    // optimized_projected_delta0가 이미 그 범위 안에 있으면 항상 같은 값이다.
    double published_steer_cmd = 0.0;

    // steering constraints
    double delta0_minus_prev_delta = 0.0;
    double max_allowed_delta_change = 0.0;
    bool steer_rate_limit_active = false;
    bool steer_near_angle_limit = false;

    // model vs measured yaw response
    // yaw_rate_cmd_model = ego_speed/wheelbase * tan(published steering) -
    // vehicle_model.cpp의 kinematic bicycle 항(yaw_dot = vx/L*tan(delta))과
    // 동일한 공식, 새 모델 아님.
    double yaw_rate_cmd_model = 0.0;
    double yaw_rate_meas_echo = 0.0;  // = yaw_rate_meas, CSV 열 이름 명확화용
};

// reference[round(time/params.dt)]의 유효 인덱스(clamp)를 계산한다. horizon
// 밖 시간은 마지막 유효 인덱스로 클램핑한다 - reference.size()는 항상
// horizon과 같다(ToReferencePath가 steps=horizon으로 채운다). 기존
// mpc_controller_node.cpp의 [MPC-TRACK] 진단이 쓰던 것과 같은 함수를
// 여기로 옮겨 [MPC-TRACK]/MPC-D1 양쪽이 재사용한다(복붙 금지).
std::size_t ReferenceIndexAtTime(double time, const MPCParams& params,
                                 std::size_t ref_size);

// existing computed values -> 위 struct로 정리만 한다(재계산 없음). 호출자가
// reference/output/ego 등을 이미 만든 뒤 넘긴다.
struct TrackingDiagnosticInputs {
    double ros_time_now_s = 0.0;
    double vehicle_state_stamp_s = 0.0;
    double trajectory_stamp_s = 0.0;
    double trajectory_vehicle_time_delta_s = 0.0;

    MPCState ego{};
    double measured_yaw_rate = 0.0;
    double measured_accel = 0.0;

    ReferencePath reference{};
    MPCControl published_control{};  // output.control (post-projection)
    MpcController::Output mpc_output{};

    MPCParams params{};
};

TrackingDiagnosticSample BuildTrackingDiagnosticSample(
    const TrackingDiagnosticInputs& in);

// CSV 헤더(컬럼 순서 고정) - csv writer와 오프라인 분석 스크립트가 같은
// 순서를 기대한다.
std::string TrackingDiagnosticCsvHeader();
std::string TrackingDiagnosticCsvRow(const TrackingDiagnosticSample& s);

// 파일에 안전하게 append하는 최소 CSV writer. 실패해도 예외를 던지지
// 않는다 - 호출자가 반환값으로 성공 여부만 확인하고, 실패해도 제어 루프를
// 막지 않는다(파일이 없거나 권한이 없어도 컨트롤러는 계속 동작해야 한다).
class TrackingDiagnosticCsvWriter {
 public:
    // 파일이 없으면 만들고 헤더를 쓴다. 이미 있으면(재시작 등) 그대로
    // append만 한다 - 헤더를 다시 쓰지 않는다.
    bool Open(const std::string& path);
    // Open이 실패했거나 호출되지 않았으면 아무 것도 하지 않고 false를
    // 반환한다(호출자는 이 반환값을 무시해도 안전하다 - 컨트롤 루프를
    // 막지 않기 위해 이 클래스 자체가 실패를 삼킨다).
    bool WriteRow(const TrackingDiagnosticSample& s);
    bool is_open() const { return file_ != nullptr; }

    ~TrackingDiagnosticCsvWriter();

 private:
    std::FILE* file_ = nullptr;
};
