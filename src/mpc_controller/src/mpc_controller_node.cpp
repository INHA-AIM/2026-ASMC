#include "mpc_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

#include "config/parameter_loader.hpp"
#include "math/utils.hpp"
#include "message_converter.hpp"
#include "model/vehicle_model.hpp"
#include "reference/path_resampler.hpp"
#include "reference/reference_validator.hpp"
#include "reference/trajectory_timing.hpp"

namespace mpc_controller_ros {
namespace {

MPCParams LoadParamsForNode(ros::NodeHandle& private_nh) {
    std::string params_path;
    if (!private_nh.getParam("mpc/config_path", params_path) ||
        params_path.empty()) {
        throw std::runtime_error(
            "required private parameter '~mpc/config_path' is missing");
    }
    MPCParams params;
    loadMPCParameters(params_path, params);
    return params;
}

// reference[round(time/params.dt)]의 곡률(clamp된 유효 인덱스)을 읽는다.
// horizon 밖 시간은 마지막 유효 인덱스로 클램핑한다 - reference.size()는
// 항상 horizon과 같다(ToReferencePath가 steps=horizon으로 채운다).
std::size_t ReferenceIndexAtTime(double time, const MPCParams& params,
                                 std::size_t ref_size) {
    if (ref_size == 0) return 0;
    const double raw_index = (params.dt > 0.0) ? std::round(time / params.dt) : 0.0;
    const long clamped = std::max<long>(
        0, std::min<long>(static_cast<long>(ref_size) - 1,
                          static_cast<long>(raw_index)));
    return static_cast<std::size_t>(clamped);
}

// curve-tracking 진단 전용 - solver 입력/warm-start/reference/constraints/
// 발행되는 CtrlCmd 중 어느 것도 바꾸지 않는다. output.control은 이미 결정된
// 값을 읽기만 한다.
//
// traj_e_lat/e_yaw는 reference[0](MPC가 이번 사이클에 실제로 추종하는
// time-aligned 지점)과 ego의 차이다 - Frenet 차선 중심 d와는 다른 값이다.
// Frenet은 매 사이클 "현재 ego 위치"에서 새로 replanning하므로, 이미 차선
// 중심에서 1m 이상 벗어난 ego라도 그 지점에서 시작하는 새 trajectory의
// reference[0]은 ego와 가깝다 - trajectory-local 추종 오차(traj_e_lat)가
// 작아도 Frenet 차선 중심 오차(d)는 클 수 있다(MORAI 실측으로 확인). 차선
// 중심 기준 authoritative 값은 이 로그가 아니라 frenet_planner_node의
// [FrenetPlanner-STATE] ... d=... 로그다 - 여기서 그 값을 재현하지 않는다
// (MPC를 Frenet 내부에 결합하지 않기 위해).
// dsteer_cmd(이전 사이클 대비 steer_cmd 변화량)는 이 로그에 넣지 않는다 -
// MpcController::last_control_이 private이라 node가 "이전에 실제로 발행한
// steer_cmd"에 깨끗하게 접근할 방법이 없다. 그것만을 위해 새 getter나 node
// 레벨의 중복 상태를 추가하지 않는다 - solver API를 건드리지 않는 선에서
// 얻을 수 있는 정보만 보고한다.
void LogCurveTrackingDiagnostics(const MPCState& ego, const ReferencePath& reference,
                                 const MPCControl& control,
                                 double measured_yaw_rate,
                                 const MPCParams& params) {
    if (reference.empty()) return;

    const double ref_x = reference.x_ref[0];
    const double ref_y = reference.y_ref[0];
    const double ref_yaw = reference.yaw_ref[0];
    const double ref_v = reference.v_ref[0];

    // 참조선 진행방향 기준 좌표계로 투영한 lateral error. dx/dy는 global
    // frame 차이이고, -sin(ref_yaw)*dx + cos(ref_yaw)*dy가 그 지점의 진행
    // 방향에 수직인(왼쪽 양수) 성분이다 - frenet_planner의 d와 같은 부호
    // 규약(FrenetConverter.hpp 참고)이지만 위 주석대로 같은 값은 아니다.
    const double dx = ego.x - ref_x;
    const double dy = ego.y - ref_y;
    const double traj_e_lat = -std::sin(ref_yaw) * dx + std::cos(ref_yaw) * dy;
    const double e_yaw = angleDiff(ego.yaw, ref_yaw);

    // curve-entry lag 진단: reference[0]뿐 아니라 MPC가 실제로 최적화하는
    // horizon(0~horizon*dt) 안의 미래 곡률/feed-forward도 함께 본다 -
    // reference[0]은 아직 거의 직선인데 0.5~1.0s 앞은 이미 급커브인 경우를
    // 놓치지 않기 위함이다.
    const std::size_t idx0 = ReferenceIndexAtTime(0.0, params, reference.size());
    const std::size_t idx05 = ReferenceIndexAtTime(0.5, params, reference.size());
    const std::size_t idx10 = ReferenceIndexAtTime(1.0, params, reference.size());
    const double k0 = reference.k_ref[idx0];
    const double k05 = reference.k_ref[idx05];
    const double k10 = reference.k_ref[idx10];

    // |kappa|가 최대인 지점의 부호 있는 값. std::max_element를 절대값
    // 비교자로 써서 부호를 그대로 보존한다(단순 std::max는 부호를 잃는다).
    const auto peak_it = std::max_element(
        reference.k_ref.begin(), reference.k_ref.end(),
        [](double a, double b) { return std::fabs(a) < std::fabs(b); });
    const double kpeak = (peak_it != reference.k_ref.end()) ? *peak_it : 0.0;

    // 이미 존재하는 공식을 그대로 재사용한다 - cost/solver가 warm-start에
    // 쓰는 것과 같은 SteeringFeedforward(model/vehicle_model.hpp)이며, 여기서
    // atan(wheelbase*curvature)를 다시 구현하지 않는다.
    const double ff0 = SteeringFeedforward(k0, params.wheelbase, params.steering_max);
    const double ff05 = SteeringFeedforward(k05, params.wheelbase, params.steering_max);
    const double ff10 = SteeringFeedforward(k10, params.wheelbase, params.steering_max);
    const double ffpeak = SteeringFeedforward(kpeak, params.wheelbase, params.steering_max);

    // 실측 yaw_rate(VehicleState, MORAI/GPS-IMU 소스)와 "실제로 발행한
    // steer_cmd가 현재 속도에서 정상상태로 만들어낼 모델상 yaw_rate"를
    // 나란히 둔다 - vehicle_model.cpp의 kinematic bicycle 항
    // (yaw_dot = vx/L * tan(delta))과 동일한 공식이며 새 헬퍼를 추가하지
    // 않는다. steer_cmd가 빠르게 반응하는데 실측 yaw_rate가 못 따라오면
    // MORAI 액추에이션/모델 불일치 쪽을, steer_cmd 자체가 늦으면 MPC
    // 튜닝/최적화 쪽을 본다.
    const double yaw_rate_cmd_model =
        (ego.vx / params.wheelbase) * std::tan(control.delta);

    constexpr double kNearLimitFraction = 0.95;
    const bool steer_near_limit =
        std::fabs(control.delta) >= kNearLimitFraction * params.steering_max;

    // curve 진입 전환은 ~0.2s 안에 일어날 수 있어 5Hz로는 중간 사이클을
    // 놓칠 수 있다 - 이번 튜닝 실험 동안만 10Hz로 올린다(제어 루프 자체는
    // 20Hz 그대로).
    ROS_INFO_THROTTLE(
        0.1,
        "[MPC-TRACK] traj_e_lat=%.3f e_yaw=%.3f ego_v=%.2f ref_v=%.2f "
        "k0=%.4f k05=%.4f k10=%.4f kpeak=%.4f "
        "ff0=%.3f ff05=%.3f ff10=%.3f ffpeak=%.3f "
        "steer_cmd=%.3f yaw_rate_meas=%.3f yaw_rate_cmd_model=%.3f "
        "steer_near_limit=%d",
        traj_e_lat, e_yaw, ego.vx, ref_v, k0, k05, k10, kpeak, ff0, ff05,
        ff10, ffpeak, control.delta, measured_yaw_rate, yaw_rate_cmd_model,
        steer_near_limit ? 1 : 0);
}

}  // namespace

MpcControllerNode::MpcControllerNode(ros::NodeHandle nh,
                                     ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)),
      controller_(LoadParamsForNode(private_nh_)) {
    std::string trajectory_topic = "/planning/trajectory";
    std::string control_topic = "/ctrl_cmd";
    std::string behavior_command_topic = "/behavior/command_v2";
    std::string vehicle_state_topic = "/vehicle/state";
    private_nh_.param("topics/trajectory", trajectory_topic, trajectory_topic);
    private_nh_.param("publish_topics/control", control_topic, control_topic);
    private_nh_.param("topics/behavior_command_v2", behavior_command_topic,
                      behavior_command_topic);
    private_nh_.param("topics/vehicle_state", vehicle_state_topic,
                      vehicle_state_topic);
    private_nh_.param("timeouts/trajectory", trajectory_timeout_,
                      trajectory_timeout_);
    private_nh_.param("timeouts/vehicle_state", vehicle_state_timeout_,
                      vehicle_state_timeout_);

    double bridge_max_steer_degrees = 36.25;
    private_nh_.param("vehicle/max_steer_deg", bridge_max_steer_degrees,
                      bridge_max_steer_degrees);
    constexpr double kRadiansToDegrees =
        180.0 / 3.14159265358979323846;
    const double mpc_max_steer_degrees =
        controller_.params().steering_max * kRadiansToDegrees;
    if (std::fabs(mpc_max_steer_degrees - bridge_max_steer_degrees) > 0.1) {
        ROS_WARN(
            "MPC max steering %.2f deg differs from MORAI bridge %.2f deg; "
            "align both to the physical vehicle limit before driving",
            mpc_max_steer_degrees, bridge_max_steer_degrees);
    }

    trajectory_sub_ = nh_.subscribe(
        trajectory_topic, 1, &MpcControllerNode::trajectoryCallback, this);
    vehicle_state_sub_ = nh_.subscribe(
        vehicle_state_topic, 1, &MpcControllerNode::vehicleStateCallback, this);
    behavior_command_sub_ = nh_.subscribe(
        behavior_command_topic, 1,
        &MpcControllerNode::behaviorCommandCallback, this);
    control_pub_ = nh_.advertise<morai_msgs::CtrlCmd>(control_topic, 1);
    const double frequency = controller_.params().control_frequency > 0.0
        ? controller_.params().control_frequency : 20.0;
    control_timer_ = nh_.createSteadyTimer(
        ros::WallDuration(1.0 / frequency),
        &MpcControllerNode::controlTimerCallback, this);

    ROS_INFO("MPC ROS runtime @ %.1f Hz: %s -> %s, stale timeout %.3f s",
             frequency, trajectory_topic.c_str(), control_topic.c_str(),
             trajectory_timeout_);
}

void MpcControllerNode::behaviorCommandCallback(
    const asmc_msgs::BehaviorCommand::ConstPtr& message) {
    if (!message) return;
    const bool emergency =
        message->maneuver_type == asmc_msgs::BehaviorCommand::EMERGENCY_STOP;
    if (emergency != emergency_stop_active_) {
        ROS_WARN("MPC emergency brake override %s (command_id=%u)",
                 emergency ? "ACTIVE" : "RELEASED", message->command_id);
    }
    emergency_stop_active_ = emergency;
}

void MpcControllerNode::trajectoryCallback(
    const asmc_msgs::Trajectory::ConstPtr& message) {
    if (!message) return;
    latest_trajectory_ = *message;
    trajectory_received_at_ = ros::SteadyTime::now();
    has_trajectory_ = true;
}

void MpcControllerNode::vehicleStateCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
    if (!message) return;
    latest_vehicle_state_ = *message;
    vehicle_state_received_at_ = ros::SteadyTime::now();
    has_vehicle_state_ = true;
}

void MpcControllerNode::publishStop(const char* reason) {
    controller_.ResetOnFailSafe();
    control_pub_.publish(MakeRosStopCommand());
    ROS_WARN_THROTTLE(1.0, "MPC fail-safe STOP: %s", reason);
}

void MpcControllerNode::controlTimerCallback(const ros::SteadyTimerEvent&) {
    if (emergency_stop_active_) {
        controller_.ResetOnFailSafe();
        control_pub_.publish(MakeRosEmergencyStopCommand());
        return;
    }
    if (!has_trajectory_) {
        publishStop("trajectory missing");
        return;
    }
    if (!IsMessageFresh(trajectory_received_at_, ros::SteadyTime::now(),
                       trajectory_timeout_)) {
        publishStop("trajectory stale");
        return;
    }
    if (!has_vehicle_state_) {
        publishStop("vehicle state missing");
        return;
    }
    if (!IsMessageFresh(vehicle_state_received_at_, ros::SteadyTime::now(),
                       vehicle_state_timeout_)) {
        publishStop("vehicle state stale");
        return;
    }

    // Trajectory -> PlannedPath만 변환한다. ego_at_plan의 provenance는
    // ConvertTrajectory가 내부적으로 검증하지만 MPCState로는 변환하지 않는다
    // - x0는 아래에서 최신 /vehicle/state로부터 독립적으로 만든다.
    PlannedPath path;
    const TrajectoryStatus status = ConvertTrajectory(latest_trajectory_, path);
    if (status == TrajectoryStatus::STOP_TRAJECTORY) {
        publishStop("valid planner STOP trajectory");
        return;
    }
    if (status != TrajectoryStatus::OK) {
        publishStop(TrajectoryStatusName(status));
        return;
    }

    MPCState current_state;
    const VehicleStateStatus vehicle_status =
        ConvertVehicleState(latest_vehicle_state_, current_state);
    if (vehicle_status != VehicleStateStatus::OK) {
        publishStop(VehicleStateStatusName(vehicle_status));
        return;
    }

    const double start_time_offset = ComputeReferenceStartOffset(
        latest_trajectory_.header.stamp.toSec(),
        latest_vehicle_state_.header.stamp.toSec());
    const ReferencePath reference = ToReferencePath(
        path, controller_.params().dt, controller_.params().horizon,
        start_time_offset);
    if (reference.empty()) {
        publishStop("reference interpolation failed");
        return;
    }

    // 물리적으로 말이 되는 참조인지 검증한다(방어적 defense-in-depth) -
    // 예를 들어 Planner/interface 쪽 미래 버그가 다시 불연속 참조를 보내도
    // solver가 그대로 따라가지 않는다. 여기서 거부하면 이 사이클은 ControlProcess를
    // 호출하지 않고 기존 fail-safe STOP 경로를 그대로 재사용한다.
    const ReferenceValidationResult validation =
        ValidateReference(reference, controller_.params());
    if (!validation.ok()) {
        publishStop(ReferenceValidationStatusName(validation.status));
        return;
    }

    const MpcController::Output output =
        controller_.ControlProcess(current_state, reference);
    if (output.stuck_event ==
        MpcController::StuckEvent::RECOVERY_STARTED) {
        ROS_WARN("MPC stuck recovery start: ego_v=%.2f max_ref_v=%.2f "
                 "path_progress=%.2f duration=%.2fs launch_accel=%.2f",
                 output.stuck_event_ego_v, output.stuck_event_max_ref_v,
                 output.stuck_event_path_progress,
                 controller_.params().stuck_recovery_duration,
                 controller_.params().stuck_launch_accel);
    } else if (output.stuck_event ==
               MpcController::StuckEvent::RECOVERY_COMPLETE) {
        ROS_INFO("MPC stuck recovery pulse complete; normal MPC resumed");
    }
    if (!output.success) {
        publishStop(output.failure_reason ? output.failure_reason
                                          : "solver failure");
        return;
    }

    LogCurveTrackingDiagnostics(current_state, reference, output.control,
                               latest_vehicle_state_.yaw_rate,
                               controller_.params());

    control_pub_.publish(ToRosControlCommand(output.control,
                                             controller_.params()));
}

}  // namespace mpc_controller_ros
