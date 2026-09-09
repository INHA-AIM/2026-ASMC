#include "diagnostics/tracking_diagnostics.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "constraints/constraints.hpp"
#include "math/utils.hpp"
#include "model/vehicle_model.hpp"

namespace {

// path의 부모 디렉터리를 best-effort로 만든다(이미 있으면 그대로 성공
// 취급) - 실패해도 예외를 던지지 않는다. 이후 fopen 자체의 성공/실패가
// 최종 판단이므로, 여기서는 흔한 기본 경로(예: "mpc_debug_logs/x.csv")가
// 그냥 동작하게만 해준다.
void BestEffortCreateParentDir(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    if (slash == std::string::npos) return;
    const std::string dir = path.substr(0, slash);
    if (dir.empty()) return;
    ::mkdir(dir.c_str(), 0755);
}

}  // namespace

std::size_t ReferenceIndexAtTime(double time, const MPCParams& params,
                                 std::size_t ref_size) {
    if (ref_size == 0) return 0;
    const double raw_index = (params.dt > 0.0) ? std::round(time / params.dt) : 0.0;
    const long clamped = std::max<long>(
        0, std::min<long>(static_cast<long>(ref_size) - 1,
                          static_cast<long>(raw_index)));
    return static_cast<std::size_t>(clamped);
}

TrackingDiagnosticSample BuildTrackingDiagnosticSample(
    const TrackingDiagnosticInputs& in) {
    TrackingDiagnosticSample s;

    // timing / provenance
    s.ros_time = in.ros_time_now_s;
    s.vehicle_state_stamp = in.vehicle_state_stamp_s;
    s.trajectory_stamp = in.trajectory_stamp_s;
    s.vehicle_state_age_s = in.ros_time_now_s - in.vehicle_state_stamp_s;
    s.trajectory_age_s = in.ros_time_now_s - in.trajectory_stamp_s;
    s.trajectory_vehicle_time_delta_s = in.trajectory_vehicle_time_delta_s;

    // vehicle state
    s.ego_x = in.ego.x;
    s.ego_y = in.ego.y;
    s.ego_yaw = in.ego.yaw;
    s.ego_speed = in.ego.vx;
    s.yaw_rate_meas = in.measured_yaw_rate;
    s.ego_accel = in.measured_accel;

    // reference preview
    if (!in.reference.empty()) {
        const std::size_t idx0 =
            ReferenceIndexAtTime(0.0, in.params, in.reference.size());
        const std::size_t idx02 =
            ReferenceIndexAtTime(0.2, in.params, in.reference.size());
        const std::size_t idx05 =
            ReferenceIndexAtTime(0.5, in.params, in.reference.size());
        const std::size_t idx10 =
            ReferenceIndexAtTime(1.0, in.params, in.reference.size());
        s.kappa_ref_0s = in.reference.k_ref[idx0];
        s.kappa_ref_02s = in.reference.k_ref[idx02];
        s.kappa_ref_05s = in.reference.k_ref[idx05];
        s.kappa_ref_10s = in.reference.k_ref[idx10];

        const auto peak_it = std::max_element(
            in.reference.k_ref.begin(), in.reference.k_ref.end(),
            [](double a, double b) { return std::fabs(a) < std::fabs(b); });
        s.kappa_ref_peak =
            (peak_it != in.reference.k_ref.end()) ? *peak_it : 0.0;

        // feed-forward: 기존 SteeringFeedforward(model/vehicle_model.hpp)를
        // 그대로 재사용한다 - 두 번째 구현을 만들지 않는다.
        s.ff_0s = SteeringFeedforward(s.kappa_ref_0s, in.params.wheelbase,
                                      in.params.steering_max);
        s.ff_02s = SteeringFeedforward(s.kappa_ref_02s, in.params.wheelbase,
                                       in.params.steering_max);
        s.ff_05s = SteeringFeedforward(s.kappa_ref_05s, in.params.wheelbase,
                                       in.params.steering_max);
        s.ff_10s = SteeringFeedforward(s.kappa_ref_10s, in.params.wheelbase,
                                       in.params.steering_max);
        s.ff_peak = SteeringFeedforward(s.kappa_ref_peak, in.params.wheelbase,
                                        in.params.steering_max);
    }

    // warm-start (MpcController::Output이 이미 계산한 값을 그대로 옮긴다)
    s.warm_used_rebase = in.mpc_output.used_curvature_rebase;
    s.warm_raw_delta0 = in.mpc_output.warm_raw0;
    s.warm_prev_ff_shift0 = in.mpc_output.prev_ff_shift0;
    s.warm_residual0 = in.mpc_output.warm_residual0;
    s.warm_current_ff0 = in.mpc_output.current_ff0;
    s.warm_rebased_delta0 = in.mpc_output.warm_rebased0;
    s.warm_projected_delta0 = in.mpc_output.warm_projected0;

    // optimized / final command
    //
    // mpc_solver.cpp의 line-search 루프는 매 후보를 projectControls()에
    // 통과시킨 뒤에야 채택 여부를 판정하므로, 최종 채택된 U[0](=
    // in.published_control.delta, stuck-recovery는 accel만 건드리므로
    // steering은 그대로다)는 이미 "최적화됨"이자 "투영됨" 둘 다다 - 이
    // 구현에서는 그 둘을 구분할 별도 pre-projection 최종값이 존재하지
    // 않는다(문서화, 새 필드로 억지 분리하지 않음).
    s.optimized_projected_delta0 = in.published_control.delta;
    // ToRosControlCommand(message_converter.cpp)가 publish 직전에 적용하는
    // 것과 정확히 같은 clip을 재현한다 - production 코드에 있는 것과 별도
    // 공식이 아니다.
    s.published_steer_cmd = clip(in.published_control.delta,
                                 -in.params.steering_max, in.params.steering_max);

    // steering constraints: clipSteeringRate가 실제로 썼던 것과 같은 계산
    // (constraints.cpp)을 재현해, 이번 cycle 최종 delta가 rate 한계에
    // 얼마나 붙어 있었는지를 본다 - 새 클램프를 도입하지 않는다(진단만).
    s.delta0_minus_prev_delta =
        in.published_control.delta - in.mpc_output.prev_applied_delta;
    s.max_allowed_delta_change =
        in.params.steering_rate_max * in.params.dt;
    constexpr double kRateLimitCloseFraction = 0.95;
    s.steer_rate_limit_active =
        std::fabs(s.delta0_minus_prev_delta) >=
        kRateLimitCloseFraction * s.max_allowed_delta_change;

    constexpr double kNearAngleLimitFraction = 0.95;
    s.steer_near_angle_limit =
        std::fabs(in.published_control.delta) >=
        kNearAngleLimitFraction * in.params.steering_max;

    // model vs measured yaw response: vehicle_model.cpp의 kinematic bicycle
    // 항(yaw_dot = vx/L * tan(delta))과 동일한 공식, 새 모델 아님. 발행되는
    // steering(published_steer_cmd) 기준으로 계산한다 - MORAI로 실제 나가는
    // 값과 대응시키기 위함.
    s.yaw_rate_cmd_model =
        (in.ego.vx / in.params.wheelbase) * std::tan(s.published_steer_cmd);
    s.yaw_rate_meas_echo = in.measured_yaw_rate;

    return s;
}

std::string TrackingDiagnosticCsvHeader() {
    return
        "ros_time,vehicle_state_stamp,trajectory_stamp,"
        "vehicle_state_age_s,trajectory_age_s,trajectory_vehicle_time_delta_s,"
        "ego_x,ego_y,ego_yaw,ego_speed,yaw_rate_meas,ego_accel,"
        "kappa_ref_0s,kappa_ref_02s,kappa_ref_05s,kappa_ref_10s,kappa_ref_peak,"
        "ff_0s,ff_02s,ff_05s,ff_10s,ff_peak,"
        "warm_used_rebase,warm_raw_delta0,warm_prev_ff_shift0,warm_residual0,"
        "warm_current_ff0,warm_rebased_delta0,warm_projected_delta0,"
        "optimized_projected_delta0,published_steer_cmd,"
        "delta0_minus_prev_delta,max_allowed_delta_change,"
        "steer_rate_limit_active,steer_near_angle_limit,"
        "yaw_rate_cmd_model,yaw_rate_meas_echo\n";
}

std::string TrackingDiagnosticCsvRow(const TrackingDiagnosticSample& s) {
    char buf[1024];
    std::snprintf(
        buf, sizeof(buf),
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.4f,%.4f,%.5f,%.4f,%.5f,%.4f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.5f,%.5f,%.5f,%.5f,%.5f,"
        "%d,%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,"
        "%.5f,%.5f,"
        "%d,%d,"
        "%.5f,%.5f\n",
        s.ros_time, s.vehicle_state_stamp, s.trajectory_stamp,
        s.vehicle_state_age_s, s.trajectory_age_s,
        s.trajectory_vehicle_time_delta_s,
        s.ego_x, s.ego_y, s.ego_yaw, s.ego_speed, s.yaw_rate_meas, s.ego_accel,
        s.kappa_ref_0s, s.kappa_ref_02s, s.kappa_ref_05s, s.kappa_ref_10s,
        s.kappa_ref_peak,
        s.ff_0s, s.ff_02s, s.ff_05s, s.ff_10s, s.ff_peak,
        s.warm_used_rebase ? 1 : 0, s.warm_raw_delta0, s.warm_prev_ff_shift0,
        s.warm_residual0,
        s.warm_current_ff0, s.warm_rebased_delta0, s.warm_projected_delta0,
        s.optimized_projected_delta0, s.published_steer_cmd,
        s.delta0_minus_prev_delta, s.max_allowed_delta_change,
        s.steer_rate_limit_active ? 1 : 0, s.steer_near_angle_limit ? 1 : 0,
        s.yaw_rate_cmd_model, s.yaw_rate_meas_echo);
    return std::string(buf);
}

bool TrackingDiagnosticCsvWriter::Open(const std::string& path) {
    if (path.empty()) return false;
    BestEffortCreateParentDir(path);
    // 파일이 이미 있으면(재시작 등) 헤더를 다시 쓰지 않고 그대로 append만
    // 한다 - 있는지 먼저 읽기 모드로 확인한다.
    const bool exists = [&path]() {
        std::FILE* probe = std::fopen(path.c_str(), "r");
        if (!probe) return false;
        std::fclose(probe);
        return true;
    }();

    file_ = std::fopen(path.c_str(), "a");
    if (!file_) return false;
    if (!exists) {
        const std::string header = TrackingDiagnosticCsvHeader();
        std::fwrite(header.data(), 1, header.size(), file_);
        std::fflush(file_);
    }
    return true;
}

bool TrackingDiagnosticCsvWriter::WriteRow(const TrackingDiagnosticSample& s) {
    if (!file_) return false;
    const std::string row = TrackingDiagnosticCsvRow(s);
    const std::size_t written = std::fwrite(row.data(), 1, row.size(), file_);
    // 매 cycle fflush는 20Hz에서 비싸지 않다(작은 텍스트 라인) - 컨트롤러가
    // 비정상 종료해도 지금까지의 진단 데이터를 잃지 않기 위해 그대로 둔다.
    std::fflush(file_);
    return written == row.size();
}

TrackingDiagnosticCsvWriter::~TrackingDiagnosticCsvWriter() {
    if (file_) std::fclose(file_);
}
