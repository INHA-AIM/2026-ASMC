#include "ros/frenet_planner_node.hpp"

#include <cmath>
#include <vector>
#include <utility>

#include "ros/message_converter.hpp"

namespace frenet_planner_ros {

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

    // D3-3 시점의 구현 상태. 아직 어떤 maneuver도 v2 궤적 생성에 연결되지
    // 않았으므로 전부 닫아둔다. D4에서 KEEP/STOP/EMERGENCY_STOP을 열고,
    // 이후 단계마다 하나씩 추가한다(계약 11.4절).
    //
    // 닫혀 있는 maneuver는 planner를 호출하지 않고 FAILED로 회신한다.
    // 조용히 velocity keeping 궤적을 만드는 것보다 안전하다.

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

    // 1. 명령 자체의 형식 검증. Ego/Object 입력의 freshness는 여기서 보지
    //    않는다 - 그것은 명령의 내용과 무관한 runtime 상태다(계약 11.2절).
    const CommandValidationResult validation =
        ValidateBehaviorCommand(*message, ros::Time::now());
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
        publishFeedbackV2(*message, acceptance);
        return;
    }

    // 2. transaction 규칙 적용 (새 ID / payload 변경 / CANCEL / preemption /
    //    terminal latch / 구현 상태).
    const CommandAcceptance acceptance = command_manager_.Accept(*message);

    // 3. D3-3에서는 여기서 planner를 호출하지 않는다. 명령 관리 인프라까지가
    //    이 단계의 범위이며, 궤적 생성 연결은 D4부터다(계약 12.4절).
    //    acceptance.should_plan은 D4가 소비할 값으로 이미 계산되어 있다.
    publishFeedbackV2(*message, acceptance);
}

void FrenetPlannerNode::publishFeedbackV2(
    const asmc_msgs::BehaviorCommand& message,
    const CommandAcceptance& acceptance) {
    asmc_msgs::FrenetFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.header.frame_id = "map";

    // command_id echo가 틀리면 FSM이 피드백 전체를 무시한다(계약 1.5절).
    // 거부된 명령에도 그 명령의 ID를 그대로 되돌려 보내야 FSM이 어느 명령에
    // 대한 응답인지 알 수 있다.
    feedback.command_id = message.command_id;

    feedback.command_accepted = acceptance.command_accepted;
    feedback.execution_state = acceptance.execution_state;
    feedback.result_code = acceptance.result_code;
    feedback.result_detail = acceptance.detail;

    // active_maneuver는 **활성 transaction**의 maneuver다. 방금 들어온 명령이
    // 아니다 - 거부된 명령의 필드를 쓰면 실제로 수행 중인 기동을 잘못
    // 보고한다.
    //
    //   활성: id=30 LANE_CHANGE LEFT (COMMITTED)
    //   수신: id=31 KEEP direction=NONE  -> 거부되어 30이 계속 수행됨
    //   이때 incoming direction(NONE)으로 LANE_CHANGE를 변환하면 실패해
    //   MANEUVER_NONE이 나간다. 실제로는 LANE_CHANGE 중이다.
    //
    // manager의 maneuver_type_은 이미 BehaviorCommand.maneuver_type 체계이므로
    // 내부 enum을 경유할 필요가 없다. 그대로 echo한다(계약 1.5절).
    feedback.active_maneuver = command_manager_.has_active_command()
        ? command_manager_.maneuver_type()
        : asmc_msgs::FrenetFeedback::MANEUVER_NONE;

    // D3-3은 궤적을 만들지 않는다. 아직 채우지 않는 필드는 메시지가 정의한
    // sentinel을 정확히 사용한다 - 임의의 0이나 추측값을 넣지 않는다
    // (계약 9.4절).
    feedback.plan_valid = false;
    feedback.trajectory_source = asmc_msgs::FrenetFeedback::TRAJECTORY_NONE;
    feedback.maneuver_progress = -1.0;
    feedback.active_leader_id = -1;
    feedback.selected_front_object_id = -1;
    feedback.selected_rear_object_id = -1;
    feedback.gap_evaluated = false;
    feedback.gap_safe = false;
    feedback.estimated_entry_time_s = -1.0;
    feedback.frenet_state_valid = false;
    feedback.current_s = 0.0;
    feedback.current_d = 0.0;
    feedback.target_d_valid = false;
    feedback.target_d = 0.0;
    feedback.applied_speed_mps = -1.0;
    feedback.generated_candidate_count = 0;
    feedback.valid_candidate_count = 0;
    feedback.selected_cost = -1.0;
    feedback.blocking_object_id = -1;

    feedback_v2_pub_.publish(feedback);
}

}  // namespace frenet_planner_ros
