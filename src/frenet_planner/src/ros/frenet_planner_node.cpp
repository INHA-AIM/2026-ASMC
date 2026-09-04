#include "ros/frenet_planner_node.hpp"

#include <cmath>
#include <vector>
#include <utility>

#include "ros/message_converter.hpp"

namespace frenet_planner_ros {

namespace {

// quaternion -> yaw. D3-2 validation이 단위 quaternion을 이미 보장하므로
// 정규화는 하지 않는다.
double YawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// 각도를 (-pi, pi]로 정규화. 단순 뺄셈으로 비교하면 179도와 -179도의 차이가
// 358도로 계산되어 실제 2도 차이를 큰 오차로 오판한다.
double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

FrenetPlannerNode::FrenetPlannerNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
    std::string vehicle_topic = "/vehicle/state";
    std::string objects_topic = "/perception/objects";
    std::string behavior_topic = "/behavior/context";
    std::string trajectory_topic = "/planning/trajectory";
    std::string feedback_topic = "/planning/feedback";
    // v2 인터페이스. v1과 병행 운영하며 D10 cutover까지 두 계통을 모두
    // 유지한다(계약 1.2절, 10.6절).
    std::string behavior_command_v2_topic = "/behavior/command_v2";
    std::string feedback_v2_topic = "/planning/frenet_feedback_v2";
    std::string planner_config_path;

    pnh_.param("planning/frequency", planning_frequency_hz_, planning_frequency_hz_);
    pnh_.param("timeouts/vehicle_state", vehicle_timeout_seconds_,
               vehicle_timeout_seconds_);
    pnh_.param("timeouts/objects", objects_timeout_seconds_,
               objects_timeout_seconds_);
    pnh_.param("timeouts/behavior", behavior_timeout_seconds_,
               behavior_timeout_seconds_);
    pnh_.param<std::string>("planner/config_path", planner_config_path, "");
    pnh_.param("topics/vehicle_state", vehicle_topic, vehicle_topic);
    pnh_.param("topics/objects", objects_topic, objects_topic);
    pnh_.param("topics/behavior", behavior_topic, behavior_topic);
    pnh_.param("publish_topics/trajectory", trajectory_topic, trajectory_topic);
    pnh_.param("publish_topics/feedback", feedback_topic, feedback_topic);
    pnh_.param("topics/behavior_command_v2", behavior_command_v2_topic,
               behavior_command_v2_topic);
    pnh_.param("publish_topics/feedback_v2", feedback_v2_topic,
               feedback_v2_topic);
    pnh_.param("v2/stop/max_target_heading_error_rad",
               stop_max_target_heading_error_rad_,
               stop_max_target_heading_error_rad_);

    if (!std::isfinite(stop_max_target_heading_error_rad_) ||
        stop_max_target_heading_error_rad_ <= 0.0 ||
        stop_max_target_heading_error_rad_ > M_PI) {
        ROS_WARN("Invalid v2/stop/max_target_heading_error_rad %.3f; using 45 deg",
                 stop_max_target_heading_error_rad_);
        stop_max_target_heading_error_rad_ = 0.785398;
    }

    if (planning_frequency_hz_ <= 0.0) {
        ROS_WARN("Invalid planning frequency %.3f Hz; using 20 Hz",
                 planning_frequency_hz_);
        planning_frequency_hz_ = 20.0;
    }
    if (vehicle_timeout_seconds_ <= 0.0 || objects_timeout_seconds_ <= 0.0 ||
        behavior_timeout_seconds_ <= 0.0) {
        ROS_WARN("Invalid input timeout configuration; using 0.20/0.30/0.50 s");
        vehicle_timeout_seconds_ = 0.20;
        objects_timeout_seconds_ = 0.30;
        behavior_timeout_seconds_ = 0.50;
    }

    if (planner_config_path.empty()) {
        ROS_ERROR("Missing required private parameter: planner/config_path");
    } else {
        planner_initialized_ = planner_.Init(planner_config_path);
        if (!planner_initialized_) {
            ROS_ERROR_STREAM("Frenet planner initialization failed: "
                             << planner_config_path);
        }
    }

    trajectory_pub_ = nh_.advertise<asmc_msgs::Trajectory>(trajectory_topic, 1);
    feedback_pub_ = nh_.advertise<asmc_msgs::PlanFeedback>(feedback_topic, 1);
    feedback_v2_pub_ =
        nh_.advertise<asmc_msgs::FrenetFeedback>(feedback_v2_topic, 1);

    // v2 구현 상태. 열린 maneuver만 planner를 호출한다(계약 11.4절).
    // 닫혀 있는 maneuver는 FAILED로 회신한다 - 조용히 velocity keeping
    // 궤적을 만드는 것보다 안전하다.
    //
    // D4-1: KEEP. D4-2: STOP(target pose 투영 + StopContext).
    // D4-3: EMERGENCY_STOP(직접 감속, 목표점 불필요).
    // D5-1: FOLLOW(primary_object_id로 지정된 대상만 추종).
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::KEEP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::STOP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::EMERGENCY_STOP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::FOLLOW, true);

    vehicle_sub_ = nh_.subscribe(
        vehicle_topic, 1, &FrenetPlannerNode::vehicleCallback, this);
    objects_sub_ = nh_.subscribe(
        objects_topic, 1, &FrenetPlannerNode::objectsCallback, this);
    behavior_sub_ = nh_.subscribe(
        behavior_topic, 1, &FrenetPlannerNode::behaviorCallback, this);
    behavior_command_v2_sub_ = nh_.subscribe(
        behavior_command_v2_topic, 1,
        &FrenetPlannerNode::behaviorCommandV2Callback, this);

    planning_timer_ = nh_.createTimer(
        ros::Duration(1.0 / planning_frequency_hz_),
        &FrenetPlannerNode::planningTimerCallback, this);

    ROS_INFO_STREAM("Frenet ROS runtime initialized at " << planning_frequency_hz_
                    << " Hz; inputs: " << vehicle_topic << ", "
                    << objects_topic << ", " << behavior_topic
                    << "; outputs: " << trajectory_topic << ", "
                    << feedback_topic << "; v2: " << behavior_command_v2_topic
                    << " -> " << feedback_v2_topic
                    << " (command handling only, no trajectory yet)"
                    << "; planner_initialized="
                    << (planner_initialized_ ? "true" : "false"));
}

void FrenetPlannerNode::vehicleCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_vehicle_ = *message;
    has_vehicle_ = true;
}

void FrenetPlannerNode::objectsCallback(
    const asmc_msgs::DetectedObjectArray::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_objects_ = *message;
    has_objects_ = true;
}

void FrenetPlannerNode::behaviorCallback(
    const asmc_msgs::BehaviorContext::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_behavior_ = *message;
    has_behavior_ = true;
}

FrenetPlannerNode::InputSnapshot FrenetPlannerNode::takeSnapshot() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    InputSnapshot snapshot;
    snapshot.vehicle = latest_vehicle_;
    snapshot.objects = latest_objects_;
    snapshot.behavior = latest_behavior_;
    snapshot.has_vehicle = has_vehicle_;
    snapshot.has_objects = has_objects_;
    snapshot.has_behavior = has_behavior_;
    return snapshot;
}

bool FrenetPlannerNode::isFresh(const ros::Time& stamp, double timeout_seconds,
                                const ros::Time& now,
                                double& age_seconds) const {
    // 정상적인 전송/스케줄링 지터로 stamp가 아주 조금 미래인 경우.
    constexpr double kFutureToleranceSeconds = 0.01;
    if (stamp.isZero()) {
        age_seconds = 0.0;
        return false;
    }
    age_seconds = (now - stamp).toSec();

    // 시계가 뒤로 점프하면 age가 크게 음수가 된다. 이건 데이터가 오래된 것이
    // 아니라 시각 기준 자체가 흔들린 것이므로 stale로 처리하면 안 된다.
    // stale로 보면 멀쩡한 입력에 대해 즉시 정지를 반복하게 된다(실측: WSL2/Docker
    // 환경에서 약 28초 주기로 wall clock이 -1.2초 점프, 그때마다 급정지).
    // 이 사이클은 판정을 보류하고 다음 사이클에서 정상 비교한다.
    if (age_seconds < -kFutureToleranceSeconds) {
        if (age_seconds >= -kMaxClockJumpSeconds) {
            ROS_WARN_THROTTLE(
                1.0,
                "Clock moved backwards (age=%.3f s); treating input as fresh "
                "for this cycle instead of forcing a stop",
                age_seconds);
            return true;
        }
        // 관용 범위를 넘는 미래 stamp는 시계 점프로 설명되지 않는다.
        // 잘못된 타임스탬프를 가진 입력이므로 그대로 거부한다.
        return false;
    }
    return age_seconds <= timeout_seconds;
}

double FrenetPlannerNode::trajectoryDt() const {
    const double dt = planner_.sample_dt();
    return planner_initialized_ && std::isfinite(dt) && dt > 0.0 ? dt : 0.1;
}

asmc_msgs::VehicleState FrenetPlannerNode::stopReferenceVehicle(
    const InputSnapshot& snapshot) const {
    if (snapshot.has_vehicle && std::isfinite(snapshot.vehicle.x) &&
        std::isfinite(snapshot.vehicle.y) && std::isfinite(snapshot.vehicle.yaw)) {
        return snapshot.vehicle;
    }
    if (has_stop_reference_vehicle_) return last_stop_reference_vehicle_;

    asmc_msgs::VehicleState unknown;
    unknown.header.frame_id = "map";
    unknown.valid = false;
    return unknown;
}

void FrenetPlannerNode::publishResult(
    const asmc_msgs::Trajectory& trajectory, const InputSnapshot& snapshot,
    bool previous_plan_still_valid) {
    trajectory_pub_.publish(trajectory);

    asmc_msgs::PlanFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.header.frame_id = "map";
    feedback.command_id = snapshot.has_behavior ? snapshot.behavior.command_id : 0;
    feedback.plan_valid = trajectory.valid;
    feedback.failure_reason = trajectory.failure_reason;
    feedback.selected_behavior = snapshot.has_behavior
        ? snapshot.behavior.selected_behavior
        : asmc_msgs::BehaviorContext::KEEP;
    feedback.selected_target_link_id = snapshot.has_behavior
        ? snapshot.behavior.target_link_id : "";
    feedback.previous_plan_still_valid = previous_plan_still_valid;
    feedback_pub_.publish(feedback);
}

void FrenetPlannerNode::publishStop(const InputSnapshot& snapshot,
                                    uint8_t failure_reason,
                                    const ros::Time& now) {
    has_last_valid_trajectory_ = false;
    plan_fail_streak_ = 0;
    const asmc_msgs::Trajectory stop = MakeStopTrajectory(
        stopReferenceVehicle(snapshot), trajectoryDt(), failure_reason, now);
    publishResult(stop, snapshot, false);
}

void FrenetPlannerNode::planningTimerCallback(const ros::TimerEvent&) {
    // v2 명령이 하나라도 도착했으면 v2가 이 사이클을 소유한다. v1 경로는
    // v2 명령이 없을 때만 동작하므로, FSM이 v2로 전환하기 전까지 기존
    // KEEP/STOP 주행이 그대로 유지된다(계약 10.6절).
    //
    // 두 계통이 같은 사이클에 궤적을 발행하면 MPC가 어느 것을 따를지 알 수
    // 없으므로 반드시 하나만 처리한다.
    if (runV2PlanningCycle(ros::Time::now())) return;

    if (!planner_initialized_) {
        ROS_ERROR_THROTTLE(
            5.0, "Frenet planning disabled: planner is not initialized");
        publishStop(takeSnapshot(), asmc_msgs::Trajectory::FAILURE_NOT_INITIALIZED,
                    ros::Time::now());
        return;
    }

    const InputSnapshot snapshot = takeSnapshot();
    if (!snapshot.has_vehicle || !snapshot.has_objects || !snapshot.has_behavior) {
        ROS_WARN_THROTTLE(
            5.0,
            "Frenet input snapshot incomplete: vehicle=%d objects=%d behavior=%d",
            snapshot.has_vehicle, snapshot.has_objects, snapshot.has_behavior);
        const uint8_t reason = !snapshot.has_vehicle
            ? asmc_msgs::Trajectory::FAILURE_STATE_STALE
            : (!snapshot.has_objects
                ? asmc_msgs::Trajectory::FAILURE_OBJECTS_STALE
                : asmc_msgs::Trajectory::FAILURE_BEHAVIOR_STALE);
        publishStop(snapshot, reason, ros::Time::now());
        return;
    }

    const ros::Time now = ros::Time::now();
    double vehicle_age = 0.0;
    double objects_age = 0.0;
    double behavior_age = 0.0;
    if (!isFresh(snapshot.vehicle.header.stamp, vehicle_timeout_seconds_,
                 now, vehicle_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale vehicle state age=%.3f s",
            vehicle_age);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_STATE_STALE, now);
        return;
    }
    if (!isFresh(snapshot.objects.header.stamp, objects_timeout_seconds_,
                 now, objects_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale objects age=%.3f s",
            objects_age);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_OBJECTS_STALE, now);
        return;
    }
    if (!isFresh(snapshot.behavior.header.stamp, behavior_timeout_seconds_,
                 now, behavior_age)) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: invalid/stale behavior age=%.3f s",
            behavior_age);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_BEHAVIOR_STALE, now);
        return;
    }

    if (snapshot.behavior.force_stop) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: behavior force_stop command_id=%u",
            snapshot.behavior.command_id);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_NONE, now);
        return;
    }

    CartesianState ego{};
    PlannerCommand command{};
    std::vector<ObjectInfo> objects;
    std::vector<TrackedObstacle> collision_objects;

    const ConversionStatus vehicle_status =
        ConvertVehicleState(snapshot.vehicle, ego);
    if (vehicle_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: vehicle conversion failed (%s)",
            ConversionStatusName(vehicle_status));
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        return;
    }
    last_stop_reference_vehicle_ = snapshot.vehicle;
    has_stop_reference_vehicle_ = true;

    const ConversionStatus behavior_status =
        ConvertBehaviorContext(snapshot.behavior, command);
    if (behavior_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: behavior conversion failed (%s)",
            ConversionStatusName(behavior_status));
        publishStop(
            snapshot,
            behavior_status == ConversionStatus::UNSUPPORTED_BEHAVIOR
                ? asmc_msgs::Trajectory::FAILURE_UNSUPPORTED_BEHAVIOR
                : asmc_msgs::Trajectory::FAILURE_INVALID_INPUT,
            now);
        return;
    }

    // STOP 목표는 BehaviorContext가 map 좌표(stop_x/y)로 준다. message_converter는
    // RefLine을 모르므로 유효성만 검사하고 stop_position을 NaN으로 남긴다.
    // RefLine을 가진 planner가 여기서 Frenet s로 투영해 명령을 완성한다.
    if (command.mode == STOP) {
        double ego_s = 0.0;
        if (!planner_.ProjectEgoToS(ego, ego_s)) {
            ROS_WARN_THROTTLE(
                1.0, "Frenet STOP required: cannot project ego onto reference line");
            publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            return;
        }
        double stop_s = 0.0;
        // ego_s를 hint로 넘겨 트랙이 자기 자신과 가까워지는 구간에서 최근접점이
        // 다른 구간으로 튀지 않게 한다. 목표가 경로에서 너무 멀면 false.
        if (!planner_.ProjectPointToS(snapshot.behavior.stop_x,
                                      snapshot.behavior.stop_y,
                                      ego_s, stop_s)) {
            ROS_WARN_THROTTLE(
                1.0,
                "Frenet STOP required: stop target (%.2f, %.2f) is not on the "
                "reference line (command_id=%u)",
                snapshot.behavior.stop_x, snapshot.behavior.stop_y,
                snapshot.behavior.command_id);
            publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            return;
        }
        command.stop_position = stop_s;
    }

    const ConversionStatus objects_status =
        ConvertDetectedObjects(snapshot.objects, objects, collision_objects);
    if (objects_status != ConversionStatus::OK) {
        ROS_WARN_THROTTLE(
            1.0, "Frenet STOP required: objects conversion failed (%s)",
            ConversionStatusName(objects_status));
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        return;
    }

    CartesianPath path;
    if (!planner_.Plan(ego, objects, collision_objects, command, path)) {
        ++plan_fail_streak_;
        // 이전 궤적 재사용은 "이번 사이클만 후보를 못 만든" 일시적 실패에만
        // 허용된다. 명령 자체가 잘못됐거나(INVALID_COMMAND) 지원하지 않는
        // 모드(UNSUPPORTED_BEHAVIOR)인데 이전 궤적을 계속 내보내면, 예를 들어
        // STOP 명령을 받고도 직전 KEEP 궤적을 따라 계속 주행하게 된다.
        // 이런 실패는 재사용 없이 즉시 정지한다.
        const FrenetPlanner::PlanFailureReason reason =
            planner_.last_failure_reason();
        bool immediate_stop = true;
        uint8_t failure_code = asmc_msgs::Trajectory::FAILURE_GENERATION;
        const char* reason_name = "UNKNOWN";
        switch (reason) {
            case FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED:
                failure_code = asmc_msgs::Trajectory::FAILURE_COLLISION_BLOCKED;
                reason_name = "COLLISION_BLOCKED";
                break;
            case FrenetPlanner::PlanFailureReason::INVALID_COMMAND:
                failure_code = asmc_msgs::Trajectory::FAILURE_INVALID_INPUT;
                reason_name = "INVALID_COMMAND";
                break;
            case FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR:
                failure_code =
                    asmc_msgs::Trajectory::FAILURE_UNSUPPORTED_BEHAVIOR;
                reason_name = "UNSUPPORTED_BEHAVIOR";
                break;
            case FrenetPlanner::PlanFailureReason::NOT_INITIALIZED:
                failure_code = asmc_msgs::Trajectory::FAILURE_NOT_INITIALIZED;
                reason_name = "NOT_INITIALIZED";
                break;
            case FrenetPlanner::PlanFailureReason::TRANSIENT_GENERATION_FAILURE:
                immediate_stop = false;
                reason_name = "TRANSIENT_GENERATION_FAILURE";
                break;
            case FrenetPlanner::PlanFailureReason::NONE:
                // Plan()이 false를 반환했는데 NONE이면 planner 내부 계약 위반이다.
                // 재사용 대상으로 삼지 않고 안전하게 정지한다.
                reason_name = "NONE";
                break;
        }

        if (immediate_stop) {
            ROS_WARN_THROTTLE(
                1.0,
                "Frenet immediate STOP: reason=%s command_id=%u",
                reason_name, snapshot.behavior.command_id);
            publishStop(snapshot, failure_code, now);
            return;
        }

        if (has_last_valid_trajectory_ &&
            plan_fail_streak_ <= kMaxTransientFailCycles) {
            asmc_msgs::Trajectory reused = last_valid_trajectory_;
            reused.failure_reason = asmc_msgs::Trajectory::FAILURE_GENERATION;
            publishResult(reused, snapshot, true);
            ROS_WARN("Frenet transient failure %d/%d: reusing previous trajectory",
                     plan_fail_streak_, kMaxTransientFailCycles);
            return;
        }
        ROS_WARN_THROTTLE(
            1.0,
            "Frenet Plan(command) failed: reason=%s command_id=%u; "
            "publishing STOP after transient reuse window",
            reason_name, snapshot.behavior.command_id);
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_GENERATION, now);
        return;
    }

    asmc_msgs::Trajectory trajectory;
    const ConversionStatus trajectory_status = ConvertCartesianPath(
        path, planner_.sample_dt(), snapshot.vehicle, now, trajectory);
    if (trajectory_status != ConversionStatus::OK) {
        ++plan_fail_streak_;
        if (has_last_valid_trajectory_ &&
            plan_fail_streak_ <= kMaxTransientFailCycles) {
            asmc_msgs::Trajectory reused = last_valid_trajectory_;
            reused.failure_reason = asmc_msgs::Trajectory::FAILURE_GENERATION;
            publishResult(reused, snapshot, true);
            return;
        }
        ROS_WARN_THROTTLE(
            1.0, "Frenet trajectory conversion failed (%s)",
            ConversionStatusName(trajectory_status));
        publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_GENERATION, now);
        return;
    }

    plan_fail_streak_ = 0;
    last_valid_trajectory_ = trajectory;
    has_last_valid_trajectory_ = true;
    publishResult(trajectory, snapshot, false);

    ROS_INFO_THROTTLE(
        1.0,
        "Frenet KEEP plan ready: command_id=%u points=%zu dt=%.3f "
        "vehicle_age=%.3f objects_age=%.3f behavior_age=%.3f; "
        "trajectory published",
        snapshot.behavior.command_id, path.x.size(), planner_.sample_dt(),
        vehicle_age, objects_age, behavior_age);
}

void FrenetPlannerNode::behaviorCommandV2Callback(
    const asmc_msgs::BehaviorCommand::ConstPtr& message) {
    if (!message) return;
    // 저장만 한다. validation/transaction/planning은 20 Hz timer가 수행한다
    // (runV2PlanningCycle 주석 참고).
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_v2_command_ = *message;
    has_v2_command_ = true;
}

bool FrenetPlannerNode::runV2PlanningCycle(const ros::Time& now) {
    asmc_msgs::BehaviorCommand command;
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (!has_v2_command_) return false;
        command = latest_v2_command_;
    }

    // ---- 1. 명령 자체의 형식 검증 -------------------------------------
    // Ego/Object 입력의 freshness는 여기서 보지 않는다. 명령의 내용과 무관한
    // runtime 상태이며 아래 3단계에서 별도로 검사한다(계약 11.2절).
    const CommandValidationResult validation =
        ValidateBehaviorCommand(command, now);
    if (!validation.ok()) {
        CommandAcceptance acceptance;
        acceptance.command_accepted = false;
        acceptance.should_plan = false;
        acceptance.execution_state = command_manager_.has_active_command()
            ? command_manager_.execution_state()
            : asmc_msgs::FrenetFeedback::IDLE;
        acceptance.result_code =
            validation.code == CommandValidationCode::STALE_COMMAND
                ? asmc_msgs::FrenetFeedback::STALE_COMMAND
                : asmc_msgs::FrenetFeedback::INVALID_COMMAND;
        acceptance.detail = validation.detail;
        ROS_WARN_THROTTLE(1.0, "v2 command rejected (%s): %s",
                          CommandValidationCodeName(validation.code),
                          validation.detail.c_str());
        // 명령이 유효하지 않으면 궤적을 내보내지 않는다. 캐시도 무효화해
        // 이전 명령의 궤적이 새 사이클에 새는 것을 막는다.
        has_last_v2_trajectory_ = false;
        v2_plan_fail_streak_ = 0;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(takeSnapshot()), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, acceptance, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }

    // ---- 2. transaction 규칙 ------------------------------------------
    const CommandAcceptance acceptance = command_manager_.Accept(command);
    if (!acceptance.should_plan) {
        // planner를 호출하지 않는 사이클이다. 다만 **궤적 출력을 멈추면 안
        // 된다**(계약 6.2절) - 기동은 끝났어도 차량은 계속 주행하거나 정지를
        // 유지해야 한다. 그래서 latch 성격에 따라 출력을 나눈다.
        //
        //   정상 완료 latch (COMPLETED) -> 완료 상태를 유지하는 궤적
        //   그 외 (미구현/실패/CANCEL)   -> 안전정지
        //
        // STOP이 HOLD에 도달해 COMPLETED로 latch된 경우, 마지막으로 만든
        // 0속도 HOLD 궤적을 계속 보낸다. 여기서 일괄 안전정지로 빠지면 정지
        // 유지 궤적이 매 사이클 교체되어 MPC 입장에서 불필요한 흔들림이 된다.
        const InputSnapshot latch_snapshot = takeSnapshot();
        if (command_manager_.completed_normally() && has_last_v2_trajectory_ &&
            last_v2_trajectory_command_id_ == command.command_id) {
            asmc_msgs::Trajectory held = last_v2_trajectory_;
            held.header.stamp = now;
            trajectory_pub_.publish(held);
            publishFeedbackV2(command.command_id, acceptance, &held,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_LOCKED);
            return true;
        }
        has_last_v2_trajectory_ = false;
        v2_plan_fail_streak_ = 0;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(latch_snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_NONE, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, acceptance, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }

    // ---- 3. 입력 freshness ---------------------------------------------
    const InputSnapshot snapshot = takeSnapshot();
    double vehicle_age = 0.0;
    double objects_age = 0.0;
    // planner 초기화 실패는 입력 stale과 다른 문제다. reference line 로드나
    // Planner 초기화가 실패한 것이며, 입력이 회복되어도 해결되지 않는다.
    // FrenetFeedback.msg의 NOT_INITIALIZED가 정확히 이 경우를 위한 코드다.
    //
    // freshness 검사보다 먼저 분리해야 한다. 묶어두면 초기화 실패가
    // "ego/object input is not fresh"로 보고되어 FSM이 센서 복구를 기다리며
    // 원인을 찾지 못한다.
    if (!planner_initialized_) {
        // 입력이 회복되어도 복구되지 않으므로 terminal 실패다.
        command_manager_.NotifyPlanFailed(/*terminal=*/true);
        CommandAcceptance failed = acceptance;
        failed.should_plan = false;
        failed.execution_state = command_manager_.execution_state();
        failed.result_code = asmc_msgs::FrenetFeedback::NOT_INITIALIZED;
        failed.detail = "planner is not initialized";
        ROS_ERROR_THROTTLE(5.0,
                           "v2 planning disabled: planner is not initialized "
                           "(command_id=%u)",
                           command.command_id);
        has_last_v2_trajectory_ = false;
        v2_plan_fail_streak_ = 0;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_NOT_INITIALIZED, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }

    const bool inputs_ready =
        snapshot.has_vehicle && snapshot.has_objects &&
        isFresh(snapshot.vehicle.header.stamp, vehicle_timeout_seconds_, now,
                vehicle_age) &&
        isFresh(snapshot.objects.header.stamp, objects_timeout_seconds_, now,
                objects_age);
    if (!inputs_ready) {
        // 명령은 정상이므로 transaction을 버리지 않는다. 이 사이클의 planning
        // 만 막고, 입력이 회복되면 같은 transaction으로 계속한다(계약 11.3절).
        command_manager_.NotifyInputDataStale();
        CommandAcceptance stale = acceptance;
        stale.should_plan = false;
        // result_code는 RESULT_NONE으로 남는다 - FrenetFeedback.msg에
        // INPUT_DATA_STALE이 없고 기존 코드로 거짓 매핑하지 않는다.
        stale.detail = "ego/object input is not fresh";
        ROS_WARN_THROTTLE(1.0,
                          "v2 planning skipped: input not fresh "
                          "(vehicle_age=%.3f objects_age=%.3f)",
                          vehicle_age, objects_age);
        has_last_v2_trajectory_ = false;
        v2_plan_fail_streak_ = 0;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_STATE_STALE, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, stale, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }

    // ---- 4. 입력 변환 ---------------------------------------------------
    CartesianState ego{};
    if (ConvertVehicleState(snapshot.vehicle, ego) != ConversionStatus::OK) {
        CommandAcceptance failed = acceptance;
        failed.should_plan = false;
        failed.detail = "vehicle state conversion failed";
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }
    last_stop_reference_vehicle_ = snapshot.vehicle;
    has_stop_reference_vehicle_ = true;

    std::vector<ObjectInfo> objects;
    std::vector<TrackedObstacle> collision_objects;
    if (ConvertDetectedObjects(snapshot.objects, objects, collision_objects) !=
        ConversionStatus::OK) {
        CommandAcceptance failed = acceptance;
        failed.should_plan = false;
        failed.detail = "object conversion failed";
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }

    // ego의 Frenet s. 두 곳에서 쓴다:
    //   1. STOP target pose 투영의 s_hint
    //   2. FrenetFeedback.current_s
    // 새 core API를 만들지 않고 기존 public ProjectEgoToS 결과를 재사용한다.
    double ego_s = 0.0;
    const bool ego_s_valid = planner_.ProjectEgoToS(ego, ego_s);

    PlannerCommand planner_command{};
    if (!ConvertBehaviorCommand(command, planner_command)) {
        CommandAcceptance failed = acceptance;
        failed.should_plan = false;
        failed.result_code = asmc_msgs::FrenetFeedback::INVALID_COMMAND;
        failed.detail = "command could not be mapped to a planner mode";
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }

    // ---- 4b. STOP 목표 투영 ---------------------------------------------
    // ConvertBehaviorCommand는 RefLine을 모르므로 stop_position을 NaN으로
    // 남긴다. RefLine을 가진 이 계층에서 map 좌표 -> Frenet s로 투영한다.
    // (v1과 동일한 방식. ProjectPointToS가 이 용도로 만들어진 함수다.)
    if (planner_command.mode == STOP) {
        if (!ego_s_valid) {
            // 투영 기준점을 얻지 못하면 정지 목표도 만들 수 없다.
            command_manager_.NotifyPlanFailed(
                /*terminal=*/true,
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
                "cannot project ego onto the reference line");
            CommandAcceptance failed = acceptance;
            failed.should_plan = false;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail = "cannot project ego onto the reference line";
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(command.command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
            return true;
        }
        double stop_s = 0.0;
        // ego_s를 hint로 넘겨 트랙이 자기 자신과 가까워지는 구간에서 최근접점이
        // 다른 구간으로 튀지 않게 한다. 목표가 경로에서 너무 멀면 false다.
        if (!planner_.ProjectPointToS(command.target_pose_map.position.x,
                                      command.target_pose_map.position.y,
                                      ego_s, stop_s)) {
            command_manager_.NotifyPlanFailed(
                /*terminal=*/true,
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
                "stop target is not on the reference line");
            CommandAcceptance failed = acceptance;
            failed.should_plan = false;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail = "stop target is not on the reference line";
            ROS_WARN_THROTTLE(1.0,
                              "v2 STOP target (%.2f, %.2f) is not on the "
                              "reference line (command_id=%u)",
                              command.target_pose_map.position.x,
                              command.target_pose_map.position.y,
                              command.command_id);
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(command.command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
            return true;
        }

        // 투영이 성공해도 방향이 맞는지는 별개다. 왕복 구간이
        // max_lateral_offset(5m) 이내로 나란히 지나가면 Cartesian 거리는
        // 가깝지만 진행 방향이 반대인 지점에 목표가 붙을 수 있다.
        // 계약 3.4절 4항의 heading 검증이다.
        double reference_heading = 0.0;
        if (!planner_.ReferenceHeadingAt(stop_s, reference_heading)) {
            command_manager_.NotifyPlanFailed(
                /*terminal=*/true,
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
                "cannot read reference heading at the stop target");
            CommandAcceptance failed = acceptance;
            failed.should_plan = false;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail = "cannot read reference heading at the stop target";
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(command.command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
            return true;
        }

        const double target_yaw =
            YawFromQuaternion(command.target_pose_map.orientation);
        // 단순 뺄셈이 아니라 정규화해서 비교한다. 179도와 -179도는 실제로
        // 2도 차이인데 뺄셈만 하면 358도가 된다.
        const double heading_error =
            std::fabs(NormalizeAngle(target_yaw - reference_heading));
        if (heading_error > stop_max_target_heading_error_rad_) {
            command_manager_.NotifyPlanFailed(
                /*terminal=*/true,
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
                "stop target heading disagrees with the reference line");
            CommandAcceptance failed = acceptance;
            failed.should_plan = false;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail =
                "stop target heading disagrees with the reference line";
            ROS_WARN_THROTTLE(
                1.0,
                "v2 STOP target heading rejected: target=%.3f ref=%.3f "
                "error=%.3f rad > %.3f (command_id=%u)",
                target_yaw, reference_heading, heading_error,
                stop_max_target_heading_error_rad_, command.command_id);
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(command.command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
            return true;
        }

        planner_command.stop_position = stop_s;
    }

    // ---- 5. planning ----------------------------------------------------
    CartesianPath path;
    const bool plan_ok = planner_.Plan(ego, objects, collision_objects,
                                       planner_command, path);
    const CommandFingerprint fingerprint = MakeCommandFingerprint(command);

    if (!plan_ok) {
        ++v2_plan_fail_streak_;
        const FrenetPlanner::PlanFailureReason reason =
            planner_.last_failure_reason();

        // 재사용은 "이번 사이클만 후보를 못 만든" 일시적 실패에만 허용한다.
        // 그리고 반드시 **같은 command_id + 같은 fingerprint**여야 한다 -
        // 다른 명령의 궤적을 재사용하면 STOP 중 KEEP 궤적을 따라가는 사고가
        // 난다(계약 8.4절).
        const bool transient =
            reason == FrenetPlanner::PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
        const bool cache_matches =
            has_last_v2_trajectory_ &&
            last_v2_trajectory_command_id_ == command.command_id &&
            SameCommandFingerprint(last_v2_trajectory_fingerprint_, fingerprint);

        if (transient && cache_matches &&
            v2_plan_fail_streak_ <= kMaxTransientFailCycles) {
            asmc_msgs::Trajectory reused = last_v2_trajectory_;
            reused.failure_reason = asmc_msgs::Trajectory::FAILURE_GENERATION;
            trajectory_pub_.publish(reused);
            CommandAcceptance transient_acceptance = acceptance;
            transient_acceptance.detail = "reusing previous trajectory";
            publishFeedbackV2(command.command_id, transient_acceptance, &reused,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_REUSED);
            ROS_WARN_THROTTLE(1.0,
                              "v2 transient failure %d/%d: reusing previous "
                              "trajectory (command_id=%u)",
                              v2_plan_fail_streak_, kMaxTransientFailCycles,
                              command.command_id);
            return true;
        }

        // 재사용 불가. 안전 정지로 회신하고 실패를 manager에 알린다.
        const bool terminal =
            reason == FrenetPlanner::PlanFailureReason::INVALID_COMMAND ||
            reason == FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR ||
            reason == FrenetPlanner::PlanFailureReason::NOT_INITIALIZED ||
            reason == FrenetPlanner::PlanFailureReason::TARGET_OBJECT_LOST;
        // 사유를 함께 latch한다. 그러지 않으면 다음 사이클에 같은
        // command_id가 들어올 때 result_code가 RESULT_NONE으로 떨어진다.
        const uint8_t latched_code =
            reason == FrenetPlanner::PlanFailureReason::TARGET_OBJECT_LOST
                ? asmc_msgs::FrenetFeedback::TARGET_OBJECT_LOST
                : (reason == FrenetPlanner::PlanFailureReason::NOT_INITIALIZED
                       ? asmc_msgs::FrenetFeedback::NOT_INITIALIZED
                       : asmc_msgs::FrenetFeedback::INVALID_COMMAND);
        command_manager_.NotifyPlanFailed(
            terminal, terminal ? latched_code
                               : asmc_msgs::FrenetFeedback::RESULT_NONE,
            terminal ? "planner rejected the maneuver" : std::string());
        has_last_v2_trajectory_ = false;

        CommandAcceptance failed = acceptance;
        failed.execution_state = command_manager_.execution_state();
        switch (reason) {
            case FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED:
                failed.result_code =
                    asmc_msgs::FrenetFeedback::COLLISION_BLOCKED;
                failed.detail = "all candidates blocked by collision";
                break;
            case FrenetPlanner::PlanFailureReason::INVALID_COMMAND:
                failed.result_code = asmc_msgs::FrenetFeedback::INVALID_COMMAND;
                failed.detail = "planner rejected the command fields";
                break;
            case FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR:
                // planner core가 아직 열지 않은 모드. manager의 구현 게이트를
                // 통과했는데 여기서 걸리면 두 목록이 어긋난 것이다.
                failed.detail = "planner core does not accept this mode yet";
                break;
            case FrenetPlanner::PlanFailureReason::NOT_INITIALIZED:
                failed.result_code =
                    asmc_msgs::FrenetFeedback::NOT_INITIALIZED;
                failed.detail = "planner is not initialized";
                break;
            case FrenetPlanner::PlanFailureReason::TRANSIENT_GENERATION_FAILURE:
                failed.result_code =
                    asmc_msgs::FrenetFeedback::TRANSIENT_GENERATION_FAILURE;
                failed.detail = "no valid candidate this cycle";
                break;
            case FrenetPlanner::PlanFailureReason::TARGET_OBJECT_LOST:
                // 대상 소실은 terminal이다. KEEP으로 자동 전환하지 않고
                // FSM이 새 command_id로 다음 행동을 정한다(계약 8.2절).
                failed.result_code =
                    asmc_msgs::FrenetFeedback::TARGET_OBJECT_LOST;
                failed.detail = "the designated target object is not usable";
                break;
            case FrenetPlanner::PlanFailureReason::NONE:
                failed.detail = "planner reported failure without a reason";
                break;
        }
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_GENERATION, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }

    // ---- 6. 궤적 변환 및 발행 -------------------------------------------
    asmc_msgs::Trajectory trajectory;
    if (ConvertCartesianPath(path, planner_.sample_dt(), snapshot.vehicle, now,
                             trajectory) != ConversionStatus::OK) {
        ++v2_plan_fail_streak_;
        command_manager_.NotifyPlanFailed(/*terminal=*/false);
        has_last_v2_trajectory_ = false;
        CommandAcceptance failed = acceptance;
        failed.execution_state = command_manager_.execution_state();
        failed.result_code =
            asmc_msgs::FrenetFeedback::TRANSIENT_GENERATION_FAILURE;
        failed.detail = "trajectory conversion failed";
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_GENERATION, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(command.command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP);
        return true;
    }

    v2_plan_fail_streak_ = 0;
    last_v2_trajectory_ = trajectory;
    has_last_v2_trajectory_ = true;
    last_v2_trajectory_command_id_ = command.command_id;
    last_v2_trajectory_fingerprint_ = fingerprint;
    command_manager_.NotifyPlanSucceeded();

    // STOP이 HOLD에 도달하면 기하학적으로 완료다. HOLD 판정은 stop_cfg_의
    // position/speed tolerance로 core가 이미 계산했으므로 그 결과를 읽는다 -
    // node가 조건을 다시 구현하면 두 판정이 어긋난다.
    //
    // COMPLETED로 latch되면 다음 사이클부터 위 latch 분기가 이 궤적을 계속
    // 보낸다(TRAJECTORY_LOCKED).
    if (planner_command.mode == STOP && planner_.stop_holding()) {
        command_manager_.NotifyManeuverCompleted();
    }

    trajectory_pub_.publish(trajectory);
    CommandAcceptance succeeded = acceptance;
    succeeded.execution_state = command_manager_.execution_state();
    publishFeedbackV2(command.command_id, succeeded, &trajectory,
                      asmc_msgs::FrenetFeedback::TRAJECTORY_NEW,
                      ego_s_valid, ego_s);

    ROS_INFO_THROTTLE(1.0,
                      "v2 plan ready: command_id=%u maneuver=%u points=%zu "
                      "dt=%.3f vehicle_age=%.3f objects_age=%.3f",
                      command.command_id, command.maneuver_type, path.x.size(),
                      planner_.sample_dt(), vehicle_age, objects_age);
    return true;
}

void FrenetPlannerNode::publishFeedbackV2(
    uint32_t command_id, const CommandAcceptance& acceptance,
    const asmc_msgs::Trajectory* trajectory, uint8_t trajectory_source,
    bool frenet_state_valid, double current_s) {
    asmc_msgs::FrenetFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.header.frame_id = "map";

    // command_id echo가 틀리면 FSM이 피드백 전체를 무시한다(계약 1.5절).
    feedback.command_id = command_id;
    feedback.command_accepted = acceptance.command_accepted;
    feedback.execution_state = acceptance.execution_state;
    feedback.result_code = acceptance.result_code;
    feedback.result_detail = acceptance.detail;

    // active_maneuver는 **활성 transaction**의 maneuver다. 방금 들어온 명령이
    // 아니다 - 거부된 명령의 필드를 쓰면 실제 수행 중인 기동을 잘못 보고한다.
    // manager의 maneuver_type_은 이미 BehaviorCommand 체계이므로 그대로 echo.
    feedback.active_maneuver = command_manager_.has_active_command()
        ? command_manager_.maneuver_type()
        : asmc_msgs::FrenetFeedback::MANEUVER_NONE;

    feedback.trajectory_source = trajectory_source;
    // plan_valid는 "이번 사이클에 요청 maneuver용 새 유효 후보를 만들었는가"다.
    // 실제로 MPC에 무엇을 보냈는지(trajectory_source)와는 별개 축이다
    // (계약 8.3절).
    feedback.plan_valid =
        trajectory_source == asmc_msgs::FrenetFeedback::TRAJECTORY_NEW;

    // Frenet 상태. current_s는 호출자가 ProjectEgoToS로 얻은 값이고,
    // current_d는 planner가 매 Plan()마다 갱신한 값이다.
    //
    // current_d는 음수가 유효하므로 -1.0을 무효값으로 쓸 수 없다. 유효성은
    // frenet_state_valid로만 판단한다(계약 1.3절).
    if (trajectory_source == asmc_msgs::FrenetFeedback::TRAJECTORY_NEW) {
        feedback.applied_speed_mps = planner_.last_target_speed();
        feedback.frenet_state_valid = frenet_state_valid;
        feedback.current_s = current_s;
        feedback.current_d = planner_.last_d();
    } else {
        feedback.applied_speed_mps = -1.0;
        feedback.frenet_state_valid = false;
        feedback.current_s = 0.0;
        feedback.current_d = 0.0;
    }

    // D4-1은 KEEP만 다룬다. 아직 채우지 않는 필드는 메시지가 정의한 sentinel을
    // 정확히 사용한다 - 임의의 0이나 추측값을 넣지 않는다(계약 9.4절).
    feedback.maneuver_progress = -1.0;
    // 실제로 추종 중인 leader. command.primary_object_id를 그대로 되돌려
    // 보내면 "지정했다"와 "확보했다"를 구분할 수 없어, 확보에 실패한
    // 사이클에도 ID가 나간다(계약 9.2절).
    feedback.active_leader_id = planner_.active_leader_id();
    feedback.selected_front_object_id = -1;
    feedback.selected_rear_object_id = -1;
    feedback.gap_evaluated = false;
    feedback.gap_safe = false;
    feedback.estimated_entry_time_s = -1.0;
    feedback.target_d_valid = false;
    feedback.target_d = 0.0;
    feedback.generated_candidate_count = 0;
    feedback.valid_candidate_count = 0;
    feedback.selected_cost = -1.0;
    feedback.blocking_object_id = -1;

    (void)trajectory;
    feedback_v2_pub_.publish(feedback);
}

}  // namespace frenet_planner_ros
