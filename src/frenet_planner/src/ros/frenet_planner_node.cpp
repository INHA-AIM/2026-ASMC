#include "ros/frenet_planner_node.hpp"

#include <cmath>
#include <vector>
#include <utility>

#include "ros/avoid_commit_coordinator.hpp"
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

    // v2 AVOID target pose 투영 (D6-1a). STOP과 값을 공유하지 않는다 -
    // 두 질문의 허용 횡거리가 다르다(헤더 주석 참고).
    pnh_.param("v2/avoid/projection_search_window",
               avoid_projection_search_window_,
               avoid_projection_search_window_);
    pnh_.param("v2/avoid/max_target_lateral_offset",
               avoid_max_target_lateral_offset_,
               avoid_max_target_lateral_offset_);
    pnh_.param("v2/avoid/max_target_heading_error_rad",
               avoid_max_target_heading_error_rad_,
               avoid_max_target_heading_error_rad_);

    if (!std::isfinite(avoid_projection_search_window_) ||
        avoid_projection_search_window_ <= 0.0) {
        ROS_WARN("Invalid v2/avoid/projection_search_window %.3f; using 100 m",
                 avoid_projection_search_window_);
        avoid_projection_search_window_ = 100.0;
    }
    if (!std::isfinite(avoid_max_target_lateral_offset_) ||
        avoid_max_target_lateral_offset_ <= 0.0) {
        ROS_WARN("Invalid v2/avoid/max_target_lateral_offset %.3f; using 4 m",
                 avoid_max_target_lateral_offset_);
        avoid_max_target_lateral_offset_ = 4.0;
    }
    if (!std::isfinite(avoid_max_target_heading_error_rad_) ||
        avoid_max_target_heading_error_rad_ <= 0.0 ||
        avoid_max_target_heading_error_rad_ > M_PI) {
        ROS_WARN("Invalid v2/avoid/max_target_heading_error_rad %.3f; "
                 "using 45 deg", avoid_max_target_heading_error_rad_);
        avoid_max_target_heading_error_rad_ = 0.785398;
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
    // D6-4a: AVOID(PREPARE/EXECUTE transaction + post-COMMIT 실행).
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::KEEP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::STOP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::EMERGENCY_STOP, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::FOLLOW, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::AVOID, true);

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
                    << " (KEEP/STOP/E-STOP/FOLLOW/AVOID enabled)"
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
    // 방금 수신한 명령. 아래 2단계에서 transaction 소유권에 따라 실제 계획
    // 대상으로 다시 묶는다(PlanningTarget).
    asmc_msgs::BehaviorCommand command;
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (!has_v2_command_) return false;
        command = latest_v2_command_;
    }
    // 피드백의 command_id는 **방금 들어온 명령**의 것이다. 계약 1.5절에 따라
    // FSM은 자기가 보낸 ID와 일치하는 피드백만 반영하므로, 거부 회신도 그
    // ID로 나가야 한다. 계획 대상이 활성 transaction으로 바뀌어도 이 값은
    // 유지한다.
    const uint32_t feedback_command_id = command.command_id;

    // 거부된 명령의 회신을 보관한다 (D6-4b). validation이 실패했지만 잠긴
    // 기동이 있어 계획을 계속하는 경우에만 채워진다 - 아래 1단계 주석 참고.
    // 이 사이클의 모든 피드백은 planning 결과가 아니라 이 값을 써야 한다.
    CommandAcceptance rejected_acceptance;
    bool has_rejected_acceptance = false;

    // ---- 1. 명령 자체의 형식 검증 -------------------------------------
    // Ego/Object 입력의 freshness는 여기서 보지 않는다. 명령의 내용과 무관한
    // runtime 상태이며 아래 3단계에서 별도로 검사한다(계약 11.2절).
    const CommandValidationResult validation =
        ValidateBehaviorCommand(command, now);
    if (!validation.ok()) {
        CommandAcceptance acceptance;
        acceptance.command_accepted = false;
        acceptance.planning_target = PlanningTarget::NONE;
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

        // ===== 잘못된 명령이 잠긴 기동을 끊지 못하게 한다 (D6-4b) =====
        //
        // validation은 Accept()보다 먼저 일어나므로, 여기서 곧바로 안전정지로
        // 빠지면 **FSM의 잘못된 메시지 하나가 COMMIT된 회피를 중단시킨다.**
        // 실측(D6-4b): AVOID id=300이 EXECUTING(locked d=+2.55)인 상태에서
        // frame_id가 틀린 KEEP id=301 하나를 보내자
        //   feedback : active_maneuver=AVOID, EXECUTING  (transaction은 살아 있다)
        //   MPC      : SAFE_STOP, 종단 d = 0.000          (횡이동 중단)
        // 이 되었다. 계약 7.2절이 막으려는 것과 같은 사고이며, 장애물 옆에서
        // 차선 사이에 서는 것이므로 8.2.1절이 지적한 위험과도 같다.
        //
        // 거부 자체는 그대로 유지한다 - 명령은 정말로 잘못되었고, FSM은 자기
        // ID로 INVALID_COMMAND/STALE_COMMAND를 받아야 한다(계약 1.5절).
        // 바꾸는 것은 **그 사이클에 무엇을 계획하는가**뿐이다. 이미 있는
        // ACTIVE_TRANSACTION 경로를 그대로 쓰므로 새 정책을 만들지 않는다
        // (계약 7.2.1절).
        //
        // 잠긴 기동이 없을 때(IDLE/KEEP/FOLLOW/PREPARING 등)는 기존대로
        // 안전정지다. 그때는 중단할 횡이동이 없고, 잘못된 명령으로 주행을
        // 계속하는 것이 오히려 위험하다.
        if (command_manager_.is_non_preemptible() &&
            command_manager_.has_active_command()) {
            acceptance.planning_target = PlanningTarget::ACTIVE_TRANSACTION;
            ROS_WARN_THROTTLE(1.0,
                              "v2 keeping the committed maneuver: rejected "
                              "command_id=%u (%s) does not interrupt active "
                              "command_id=%u",
                              feedback_command_id,
                              CommandValidationCodeName(validation.code),
                              command_manager_.command_id());
        } else {
            // 명령이 유효하지 않으면 궤적을 내보내지 않는다. 캐시도 무효화해
            // 이전 명령의 궤적이 새 사이클에 새는 것을 막는다.
            has_last_v2_trajectory_ = false;
            v2_plan_fail_streak_ = 0;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(takeSnapshot()), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, acceptance, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }
        // 잠긴 기동을 계속 계획한다. 아래 3단계부터는 활성 명령을 쓰므로
        // 여기서 command를 바꾸고, 피드백 ID는 방금 들어온 명령의 것을
        // 유지한다(계약 7.2.1절 마지막 문단).
        command = command_manager_.active_command();
        rejected_acceptance = acceptance;
        has_rejected_acceptance = true;
    }

    // ---- 2. transaction 규칙 ------------------------------------------
    //
    // 거부된 명령(has_rejected_acceptance)은 Accept()에 넣지 않는다. 형식이
    // 잘못된 명령을 transaction 규칙에 태우면 그 자체가 새 transaction을
    // 열거나 fingerprint를 흔들 수 있다. 여기서 필요한 것은 "잠긴 기동을 그대로
    // 계속한다"뿐이므로, 이미 만들어 둔 ACTIVE_TRANSACTION 회신을 쓴다.
    const CommandAcceptance acceptance = has_rejected_acceptance
        ? rejected_acceptance
        : command_manager_.Accept(command);

    // manager와 core의 transaction boundary를 맞춘다. 새 AVOID뿐 아니라 새
    // KEEP/FOLLOW/STOP/E-STOP도 freshness/conversion 조기 반환 전에 이전 AVOID
    // context를 끊는다. same-ID refresh와 거부된 교체에는 플래그가 서지 않아
    // 실행 중인 lock을 보존한다.
    if (acceptance.transaction_started) {
        planner_.ResetExternalAvoidance();
    }

    // ===== 계획 대상 재결속 (D6-0a) =====
    //
    // COMMITTED/EXECUTING인 handshake maneuver 중에 다른 명령이 오면 manager는
    // 그것을 거부하고 기존 transaction을 유지한다(계약 7.2절). 이때 아래로
    // 내려가는 command가 **거부된 새 명령**이면, 피드백은 회피 중이라고
    // 말하면서 MPC로는 차선 복귀 궤적이 나간다.
    //
    // 그래서 planner에 넘길 명령을 여기서 활성 transaction의 것으로 바꾼다.
    // 이후 코드는 STOP 목표 투영·fingerprint·궤적 캐시까지 모두 이 command를
    // 쓰므로, 캐시가 다른 명령의 궤적으로 오염되지도 않는다(계약 8.4절).
    if (acceptance.planning_target == PlanningTarget::ACTIVE_TRANSACTION) {
        if (!command_manager_.has_active_command()) {
            // manager가 ACTIVE_TRANSACTION을 돌려주면서 활성 명령이 없는
            // 것은 manager 내부 모순이다. 조용히 새 명령을 계획하지 않고
            // 안전정지로 빠진다.
            ROS_ERROR_THROTTLE(
                1.0,
                "v2 planning target is ACTIVE_TRANSACTION but no active "
                "command exists (command_id=%u)", feedback_command_id);
            CommandAcceptance failed = acceptance;
            failed.planning_target = PlanningTarget::NONE;
            failed.detail = "no active transaction to plan";
            has_last_v2_trajectory_ = false;
            v2_plan_fail_streak_ = 0;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(takeSnapshot()), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_NONE, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }
        command = command_manager_.active_command();
        ROS_WARN_THROTTLE(1.0,
                          "v2 keeping the committed maneuver: planning "
                          "active command_id=%u (incoming id=%u rejected)",
                          command.command_id, feedback_command_id);
    }

    if (!acceptance.should_plan()) {
        // ---- 취소된 transaction의 core context 정리 (D6-3a) -------------
        //
        // CANCEL이 성공하면 planner를 호출하지 않으므로, PlanInternal 안의
        // context reset이 일어나지 않는다. 그러면 다음 AVOID가 사라진 대상에
        // 대해 stale context로 유예를 받는다
        // (CommandAcceptance::transaction_cancelled 주석 참고).
        //
        // 거부된 CANCEL(TOO_LATE_TO_CANCEL)에서는 이 분기로 오지 않는다 -
        // 그때 planning_target은 ACTIVE_TRANSACTION이므로 위에서 계획을
        // 계속한다. 잠긴 목표가 사라지지 않는다.
        if (ShouldResetAvoidContext(acceptance)) {
            planner_.ResetExternalAvoidance();
        }

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
            const AvoidFeedbackFields* avoid_fields =
                command_manager_.maneuver_type() ==
                            asmc_msgs::BehaviorCommand::AVOID &&
                        has_last_v2_avoid_feedback_
                    ? &last_v2_avoid_feedback_
                    : nullptr;
            publishFeedbackV2(feedback_command_id, acceptance, &held,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_LOCKED,
                              /*plan_valid=*/false, false, 0.0, avoid_fields);
            return true;
        }
        has_last_v2_trajectory_ = false;
        v2_plan_fail_streak_ = 0;
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(latch_snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_NONE, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_command_id, acceptance, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
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
        failed.planning_target = PlanningTarget::NONE;
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
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
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
        stale.planning_target = PlanningTarget::NONE;
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
        publishFeedbackV2(feedback_command_id, stale, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
        return true;
    }

    // ---- 4. 입력 변환 ---------------------------------------------------
    CartesianState ego{};
    if (ConvertVehicleState(snapshot.vehicle, ego) != ConversionStatus::OK) {
        CommandAcceptance failed = acceptance;
        failed.planning_target = PlanningTarget::NONE;
        failed.detail = "vehicle state conversion failed";
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
        return true;
    }
    last_stop_reference_vehicle_ = snapshot.vehicle;
    has_stop_reference_vehicle_ = true;

    std::vector<ObjectInfo> objects;
    std::vector<TrackedObstacle> collision_objects;
    if (ConvertDetectedObjects(snapshot.objects, objects, collision_objects) !=
        ConversionStatus::OK) {
        CommandAcceptance failed = acceptance;
        failed.planning_target = PlanningTarget::NONE;
        failed.detail = "object conversion failed";
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
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
        failed.planning_target = PlanningTarget::NONE;
        failed.result_code = asmc_msgs::FrenetFeedback::INVALID_COMMAND;
        failed.detail = "command could not be mapped to a planner mode";
        const asmc_msgs::Trajectory stop = MakeStopTrajectory(
            stopReferenceVehicle(snapshot), trajectoryDt(),
            asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
        trajectory_pub_.publish(stop);
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
        return true;
    }

    // AVOID는 execution_request가 아니라 transaction state로 경로를 고른다.
    // READY의 첫 EXECUTE는 아래 pre-COMMIT 재검증/원자 COMMIT을 그대로 타고,
    // 다음 tick의 COMMITTED에서만 한 번 EXECUTING으로 전이한다.
    AvoidPlanningRoute avoid_route = AvoidPlanningRoute::INVALID;
    if (planner_command.mode == AVOID) {
        avoid_route = RouteAvoidPlanning(command_manager_);
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
            failed.planning_target = PlanningTarget::NONE;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail = "cannot project ego onto the reference line";
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
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
            failed.planning_target = PlanningTarget::NONE;
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
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
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
            failed.planning_target = PlanningTarget::NONE;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail = "cannot read reference heading at the stop target";
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
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
            failed.planning_target = PlanningTarget::NONE;
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
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }

        planner_command.stop_position = stop_s;
    }

    // ---- 4c. AVOID 목표점 투영 (D6-1a) ----------------------------------
    //
    // AVOID의 target pose는 **선택 사항**이다(원본 메시지: "AVOID: 필요하면
    // 회피 목표점"). 그래서 STOP과 달리:
    //   has_target_pose=false -> 정상. 장애물 geometry + 방향으로 목표를
    //                            계산한다(D6-2). 여기서 아무것도 하지 않는다.
    //   has_target_pose=true  -> 투영해서 [A] 필드에 보존한다. 합법적인
    //                            명령의 pose를 조용히 버리지 않는다.
    //
    // 투영 결과를 avoidance_d_offset([B])에 바로 넣지 않는 이유는
    // PlannerCommand의 has_avoid_target_d 주석에 있다 - command/result 경계.
    if (planner_command.mode == AVOID && command.has_target_pose &&
        avoid_route == AvoidPlanningRoute::PRE_COMMIT) {
        if (!ego_s_valid) {
            // 투영 기준점이 없으면 목표점도 만들 수 없다. STOP과 달리
            // pose는 선택 사항이지만, **주어진 pose를 해석하지 못하는 것**은
            // 조용히 넘길 일이 아니다 - FSM은 그 목표로 회피할 것을
            // 기대하고 있다.
            command_manager_.NotifyPlanFailed(
                /*terminal=*/true,
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
                "cannot project ego onto the reference line");
            CommandAcceptance failed = acceptance;
            failed.planning_target = PlanningTarget::NONE;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail = "cannot project ego onto the reference line";
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }

        double avoid_s = 0.0;
        double avoid_d = 0.0;
        if (!planner_.ProjectPointToFrenet(
                command.target_pose_map.position.x,
                command.target_pose_map.position.y, ego_s,
                avoid_projection_search_window_,
                avoid_max_target_lateral_offset_, avoid_s, avoid_d)) {
            command_manager_.NotifyPlanFailed(
                /*terminal=*/true,
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
                "avoid target is not near the reference line");
            CommandAcceptance failed = acceptance;
            failed.planning_target = PlanningTarget::NONE;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail = "avoid target is not near the reference line";
            ROS_WARN_THROTTLE(1.0,
                              "v2 AVOID target (%.2f, %.2f) is not near the "
                              "reference line (command_id=%u)",
                              command.target_pose_map.position.x,
                              command.target_pose_map.position.y,
                              command.command_id);
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }

        // heading 검증. STOP과 같은 이유다 - 왕복 구간이 허용 횡거리 이내로
        // 나란히 지나가면 Cartesian 거리는 가깝지만 진행 방향이 반대인
        // 지점에 목표가 붙을 수 있다(계약 3.4절 4항).
        double reference_heading = 0.0;
        if (!planner_.ReferenceHeadingAt(avoid_s, reference_heading)) {
            command_manager_.NotifyPlanFailed(
                /*terminal=*/true,
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
                "cannot read reference heading at the avoid target");
            CommandAcceptance failed = acceptance;
            failed.planning_target = PlanningTarget::NONE;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail = "cannot read reference heading at the avoid target";
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }

        const double target_yaw =
            YawFromQuaternion(command.target_pose_map.orientation);
        // 정규화해서 비교한다. 179도와 -179도는 실제로 2도 차이인데 단순
        // 뺄셈만 하면 358도가 된다.
        const double heading_error =
            std::fabs(NormalizeAngle(target_yaw - reference_heading));
        if (heading_error > avoid_max_target_heading_error_rad_) {
            command_manager_.NotifyPlanFailed(
                /*terminal=*/true,
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED,
                "avoid target heading disagrees with the reference line");
            CommandAcceptance failed = acceptance;
            failed.planning_target = PlanningTarget::NONE;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TARGET_PROJECTION_FAILED;
            failed.detail =
                "avoid target heading disagrees with the reference line";
            ROS_WARN_THROTTLE(
                1.0,
                "v2 AVOID target heading rejected: target=%.3f ref=%.3f "
                "error=%.3f rad > %.3f (command_id=%u)",
                target_yaw, reference_heading, heading_error,
                avoid_max_target_heading_error_rad_, command.command_id);
            has_last_v2_trajectory_ = false;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_INVALID_INPUT, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }

        planner_command.has_avoid_target_d = true;
        planner_command.avoid_target_d = avoid_d;
        planner_command.avoid_target_s = avoid_s;
    }

    const CommandFingerprint fingerprint = MakeCommandFingerprint(command);

    // ---- 4d. AVOID post-COMMIT (D6-3b) --------------------------------
    // target observation/pose는 더 이상 실행 조건이 아니다. core의 잠긴
    // target_d/context만 사용하되 collision geometry는 매 cycle 새로 쓴다.
    if (planner_command.mode == AVOID &&
        avoid_route == AvoidPlanningRoute::POST_COMMIT) {
        CartesianPath committed_path;
        FrenetPlanner::AvoidPrepareResult committed;
        const bool plan_ok = planner_.PlanCommittedAvoid(
            ego, objects, collision_objects, planner_command, committed_path,
            committed);

        if (!plan_ok) {
            ++v2_plan_fail_streak_;
            const FrenetPlanner::PlanFailureReason reason =
                planner_.last_failure_reason();
            // COMMITTED/EXECUTING에서는 이 통지가 상태를 WAITING으로 낮추지
            // 않는다. lock/context도 그대로 둔다.
            command_manager_.NotifyPlanFailed(/*terminal=*/false);
            has_last_v2_trajectory_ = false;
            CommandAcceptance failed = acceptance;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                reason == FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED
                    ? asmc_msgs::FrenetFeedback::COLLISION_BLOCKED
                    : asmc_msgs::FrenetFeedback::TRANSIENT_GENERATION_FAILURE;
            failed.detail =
                reason == FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED
                    ? "committed avoid path blocked by collision"
                    : "could not generate the committed avoid path";
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                reason == FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED
                    ? asmc_msgs::Trajectory::FAILURE_COLLISION_BLOCKED
                    : asmc_msgs::Trajectory::FAILURE_GENERATION,
                now);
            trajectory_pub_.publish(stop);

            // ===== 잠긴 진단은 SAFE_STOP에서도 유지한다 (D6-4b) =====
            //
            // 여기서 바뀐 것은 **이번 cycle에 MPC로 보낸 궤적**뿐이다. AVOID
            // transaction과 COMMIT lock은 그대로 살아 있다(계약 8.2.1절):
            // manager는 EXECUTING이고 core의 v2_avoidance_도 committed다.
            //
            // 그런데 avoid_fields 없이 발행하면 publishFeedbackV2가 전부
            // sentinel로 채워
            //   target_d_valid=false / target_d=0 / progress=-1 / blocking=-1
            // 이 나간다. FSM 입장에서는 "회피 중인데 목표도 진행률도 없고
            // 무엇이 막았는지도 모른다"가 되어, 잠긴 기동이 사라진 것처럼
            // 보인다. 실측(D6-4b)에서 정확히 이 값들이 관측되었다.
            //
            // plan_valid=false와 SAFE_STOP은 그대로 둔다 - 새 유효 후보를
            // 만들지 못했고 실제로 안전정지를 보냈다는 것은 사실이다
            // (계약 8.3절의 두 축). 잃으면 안 되는 것은 **진단**이다.
            AvoidFeedbackFields blocked_fields;
            // 실제로 발행한 SAFE_STOP 궤적의 종단 속도. maintenance와 같은
            // 이유로 last_target_speed_를 쓰지 않는다.
            blocked_fields.applied_speed_mps =
                stop.points.empty() ? -1.0 : stop.points.back().velocity;
            blocked_fields.target_d_valid = planner_.avoid_committed();
            blocked_fields.target_d = planner_.avoid_locked_target_d();
            blocked_fields.generated_candidate_count =
                static_cast<uint32_t>(committed.generated_candidate_count);
            blocked_fields.valid_candidate_count =
                static_cast<uint32_t>(committed.valid_candidate_count);
            // 새 후보를 선택하지 못했으므로 cost는 sentinel이다.
            blocked_fields.selected_cost = -1.0;
            blocked_fields.blocking_object_id = committed.blocking_object_id;
            // core가 마지막 committed cycle의 진행률을 보관한다. 실패 cycle에
            // UpdateCommittedAvoidLifecycle가 호출되지 않아 result 쪽은
            // sentinel이므로 core 값을 읽는다.
            blocked_fields.maneuver_progress =
                planner_.avoid_committed() ? planner_.avoid_maneuver_progress()
                                           : -1.0;

            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false, ego_s_valid, ego_s,
                              &blocked_fields);
            return true;
        }

        asmc_msgs::Trajectory trajectory;
        if (ConvertCartesianPath(committed_path, planner_.sample_dt(),
                                 snapshot.vehicle, now, trajectory) !=
                ConversionStatus::OK || trajectory.points.empty()) {
            ++v2_plan_fail_streak_;
            command_manager_.NotifyPlanFailed(/*terminal=*/false);
            has_last_v2_trajectory_ = false;
            CommandAcceptance failed = acceptance;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TRANSIENT_GENERATION_FAILURE;
            failed.detail = "committed avoid trajectory conversion failed";
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_GENERATION, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false, ego_s_valid, ego_s);
            return true;
        }

        AvoidFeedbackFields avoid_fields;
        avoid_fields.applied_speed_mps = trajectory.points.back().velocity;
        avoid_fields.target_d_valid = true;
        avoid_fields.target_d = planner_.avoid_locked_target_d();
        avoid_fields.generated_candidate_count =
            static_cast<uint32_t>(committed.generated_candidate_count);
        avoid_fields.valid_candidate_count =
            static_cast<uint32_t>(committed.valid_candidate_count);
        avoid_fields.selected_cost = committed.selected_cost;
        avoid_fields.blocking_object_id = committed.blocking_object_id;
        avoid_fields.maneuver_progress = committed.maneuver_progress;

        // trajectory와 feedback payload가 모두 준비된 뒤에만 manager를
        // COMPLETED로 올린다. 변환 실패 뒤 COMPLETED만 남는 상태를 막는다.
        if (committed.maneuver_completed &&
            !CompleteAvoidAtomically(command_manager_, committed,
                                     /*trajectory_ready=*/true)) {
            command_manager_.NotifyPlanFailed(/*terminal=*/false);
            has_last_v2_trajectory_ = false;
            CommandAcceptance failed = acceptance;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TRANSIENT_GENERATION_FAILURE;
            failed.detail = "avoid completion state transition rejected";
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_GENERATION, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false, ego_s_valid, ego_s,
                              &avoid_fields);
            return true;
        }

        v2_plan_fail_streak_ = 0;
        last_v2_trajectory_ = trajectory;
        has_last_v2_trajectory_ = true;
        last_v2_trajectory_command_id_ = command.command_id;
        last_v2_trajectory_fingerprint_ = fingerprint;
        last_v2_avoid_feedback_ = avoid_fields;
        has_last_v2_avoid_feedback_ = true;

        CommandAcceptance executing = acceptance;
        executing.execution_state = command_manager_.execution_state();
        // ===== 거부 회신을 planning 결과로 덮지 않는다 (D6-4b) =====
        //
        // acceptance.result_code는 **방금 들어온 명령에 대한 manager의 답**이다.
        // COMMITTED/EXECUTING 중에 다른 ID나 CANCEL이 오면 TOO_LATE_TO_CANCEL이
        // 들어 있고(계약 7.1/7.2절), 그 cycle의 planning 대상은
        // ACTIVE_TRANSACTION이므로 여기까지 내려온다.
        //
        // 무조건 RESULT_NONE으로 덮으면 FSM은 "취소도 교체도 정상 처리됐다"로
        // 읽는다. 실측(D6-4b)에서 AVOID EXECUTING 중 KEEP id=101과 same-id
        // CANCEL 모두 result_code=0으로 나갔다. active_maneuver=AVOID와
        // 잠긴 궤적은 맞았지만, 거부 사유만 사라진 것이다.
        //
        // 계획이 성공했다는 사실은 plan_valid=true와 LOCKED source가 이미
        // 전달한다. result_code는 명령 수용 축이므로 acceptance 값을 남긴다.
        if (acceptance.result_code == asmc_msgs::FrenetFeedback::RESULT_NONE) {
            executing.detail = committed.maneuver_completed
                ? "avoid maneuver completed"
                : "executing committed avoid trajectory";
        }
        trajectory_pub_.publish(trajectory);
        publishFeedbackV2(feedback_command_id, executing, &trajectory,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_LOCKED,
                          /*plan_valid=*/true, ego_s_valid, ego_s,
                          &avoid_fields);
        return true;
    }

    // ---- 4e. AVOID PREPARE (D6-2c) --------------------------------------
    //
    // 계약 5.2절. PREPARE는 일반 planning과 다른 경로다:
    //   후보 평가 결과  -> plan_valid / 진단 필드
    //   maintenance     -> MPC로 발행
    //
    // **AVOID candidate는 절대 발행하지 않는다.** FSM이 EXECUTE를 주기 전에
    // 차가 횡이동을 시작하면 안 된다.
    //
    // D6-4a에서 production gate가 열렸다. PREPARE에는 maintenance를,
    // EXECUTE가 현재 snapshot 재검증에 성공한 최초 COMMIT cycle에는
    // locked candidate를 발행한다.
    if (planner_command.mode == AVOID &&
        avoid_route == AvoidPlanningRoute::PRE_COMMIT &&
        (command.execution_request == asmc_msgs::BehaviorCommand::PREPARE ||
         command.execution_request == asmc_msgs::BehaviorCommand::EXECUTE)) {
        // EXECUTE는 COMMIT 허가다. 준비된 후보를 그대로 실행하지 않고
        // **현재 snapshot으로 다시 평가**한다(계약 5.3절). PREPARE와 같은
        // PrepareAvoid()를 쓰는 이유가 그것이다 - 재검증이 준비와 같은
        // 계산이어야 두 판정이 어긋나지 않는다.
        const bool execute_requested =
            command.execution_request == asmc_msgs::BehaviorCommand::EXECUTE;

        // ===== EXECUTE는 이미 관측된 READY에만 허가된다 (D6-4b) =====
        //
        // 계약 5.5절의 정상 순서는
        //   PREPARING -> READY -> EXECUTE -> 재검증 -> COMMITTED
        // 이고, 5.3절의 EXECUTE는 "**이미 준비된** 후보를 현재 snapshot으로
        // 다시 검증하는" 단계다. 그런데 아래 NotifyPlanSucceeded()가
        // PREPARING/WAITING을 READY로 올린 직후 같은 cycle에서 COMMIT까지
        // 진행하면 다음이 가능하다.
        //
        //   새 AVOID transaction의 첫 command가 EXECUTE
        //     -> BeginTransaction: PREPARING
        //     -> PrepareAvoid(): candidate_valid
        //     -> NotifyPlanSucceeded(): PREPARING -> READY
        //     -> CommitAvoidAtomically(): READY -> COMMITTED
        //
        // 한 cycle 안에서 PREPARING -> READY -> COMMITTED가 끝나므로 FSM은
        // READY를 **한 번도 관측하지 못한다.** 실측(D6-4b)에서도 첫 feedback이
        // 곧바로 COMMITTED/LOCKED였다. 그러면 EXECUTE가 "재검증"이 아니라
        // 최초 준비 결과를 그대로 잠그는 것이 되어, 5.3절이 막으려던
        // "준비와 실행 사이의 상황 변화"를 검사할 기회 자체가 없어진다.
        //
        // 그래서 COMMIT 조건에 "planning 이전부터 READY였는가"를 넣는다.
        // 보수적인 쪽으로 판단한다 - 한 cycle 늦게 COMMIT하는 비용은 작고,
        // 미관측 READY로 잠그는 위험은 크다.
        //
        //   첫 command부터 EXECUTE
        //     cycle N   : PREPARING -> candidate valid -> READY, maintenance NEW
        //     cycle N+1 : 이미 READY -> 현재 snapshot 재검증 -> COMMITTED
        //
        //   WAITING에서 EXECUTE 재발행
        //     후보 회복 cycle : WAITING -> READY (COMMIT 안 함)
        //     다음 EXECUTE    : 재검증 -> COMMITTED
        //
        // PREPARE로 READY에 도달한 뒤의 정상 EXECUTE에는 추가 지연이 없다 -
        // 그때는 planning 이전 상태가 이미 READY다.
        const bool was_ready_before_planning =
            command_manager_.execution_state() ==
            asmc_msgs::FrenetFeedback::READY;

        CartesianPath maintenance;
        FrenetPlanner::AvoidPrepareResult prepare;
        const bool prepare_ok = planner_.PrepareAvoid(
            ego, objects, collision_objects, planner_command, maintenance,
            prepare);

        if (!prepare_ok) {
            // maintenance조차 만들지 못했다. 실패 종류를 구분해야 한다 -
            // 전부 WAITING으로 만들면 대상 소실이 조용히 묻힌다.
            ++v2_plan_fail_streak_;
            const FrenetPlanner::PlanFailureReason reason =
                planner_.last_failure_reason();
            const bool terminal =
                reason == FrenetPlanner::PlanFailureReason::TARGET_OBJECT_LOST ||
                reason == FrenetPlanner::PlanFailureReason::INVALID_COMMAND ||
                reason == FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR ||
                reason == FrenetPlanner::PlanFailureReason::NOT_INITIALIZED;
            uint8_t code = asmc_msgs::FrenetFeedback::RESULT_NONE;
            std::string detail;
            switch (reason) {
                case FrenetPlanner::PlanFailureReason::TARGET_OBJECT_LOST:
                    code = asmc_msgs::FrenetFeedback::TARGET_OBJECT_LOST;
                    detail = "the designated avoid target is not usable";
                    break;
                case FrenetPlanner::PlanFailureReason::INVALID_COMMAND:
                    code = asmc_msgs::FrenetFeedback::INVALID_COMMAND;
                    detail = "planner rejected the avoid command fields";
                    break;
                case FrenetPlanner::PlanFailureReason::NOT_INITIALIZED:
                    code = asmc_msgs::FrenetFeedback::NOT_INITIALIZED;
                    detail = "planner is not initialized";
                    break;
                case FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR:
                    detail = "planner core does not accept AVOID PREPARE yet";
                    break;
                default:
                    // maintenance 생성의 일시 실패. 대상은 정상이므로
                    // transaction을 버리지 않는다.
                    code = asmc_msgs::FrenetFeedback::
                        TRANSIENT_GENERATION_FAILURE;
                    detail = "could not build an avoid maintenance path";
                    break;
            }
            command_manager_.NotifyPlanFailed(terminal, terminal ? code
                                                                 : asmc_msgs::
                                                       FrenetFeedback::
                                                           RESULT_NONE,
                                              terminal ? detail
                                                       : std::string());
            has_last_v2_trajectory_ = false;
            CommandAcceptance failed = acceptance;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code = code;
            failed.detail = detail;
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_GENERATION, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }

        // maintenance를 ROS 궤적으로 변환한다.
        asmc_msgs::Trajectory trajectory;
        if (ConvertCartesianPath(maintenance, planner_.sample_dt(),
                                 snapshot.vehicle, now, trajectory) !=
            ConversionStatus::OK) {
            ++v2_plan_fail_streak_;
            command_manager_.NotifyPlanFailed(/*terminal=*/false);
            has_last_v2_trajectory_ = false;
            CommandAcceptance failed = acceptance;
            failed.execution_state = command_manager_.execution_state();
            failed.result_code =
                asmc_msgs::FrenetFeedback::TRANSIENT_GENERATION_FAILURE;
            failed.detail = "avoid maintenance conversion failed";
            const asmc_msgs::Trajectory stop = MakeStopTrajectory(
                stopReferenceVehicle(snapshot), trajectoryDt(),
                asmc_msgs::Trajectory::FAILURE_GENERATION, now);
            trajectory_pub_.publish(stop);
            publishFeedbackV2(feedback_command_id, failed, &stop,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                              /*plan_valid=*/false);
            return true;
        }

        // ---- 상태 전이 --------------------------------------------------
        //
        // **candidate_valid만으로 정한다.** low_speed_context는 상황 정보이며
        // 상태 판단에 쓰지 않는다(AvoidPrepareResult 주석 참고).
        //
        //   candidate 유효  -> READY   (EXECUTE를 받을 준비가 됐다)
        //   candidate 무효  -> WAITING (조건이 바뀌면 가능하다)
        //
        // WAITING은 terminal 실패가 아니다. 후보가 막힌 것은 "지금은 안 되지만
        // 조건이 바뀌면 된다"이므로 같은 command_id로 계속 재평가된다.
        if (prepare.candidate_valid) {
            command_manager_.NotifyPlanSucceeded();
        } else {
            command_manager_.NotifyPlanFailed(/*terminal=*/false);
        }

        // ---- COMMIT (D6-3a) ---------------------------------------------
        //
        // ===== COMMIT은 원자적이어야 한다 =====
        //
        // 순서가 계약이다:
        //   1. 재검증 통과 (candidate_valid)
        //   2. candidate -> ROS Trajectory 변환 성공
        //   3. core COMMIT preflight 성공
        //   4. manager + core COMMIT 동시 적용
        //   5. 캐시 저장 + publish
        //
        // 2번을 3번보다 먼저 확인해야 한다. COMMIT을 먼저 올리고 변환이
        // 실패하면
        //   Feedback = COMMITTED AVOID
        //   MPC      = 직전 PREPARE의 중앙 maintenance
        // 라는 불일치가 생긴다 - D6-0a에서 막은 것과 같은 종류다.
        bool committed_now = false;
        asmc_msgs::Trajectory avoid_trajectory;
        // was_ready_before_planning: 위 direct EXECUTE 주석 참고. 이번 cycle에
        // 처음 READY가 된 후보는 잠그지 않고 다음 EXECUTE cycle에서 재검증한다.
        if (execute_requested && was_ready_before_planning &&
            prepare.candidate_valid) {
            // 2번: 변환을 **먼저** 시도한다.
            const ConversionStatus convert_status = ConvertCartesianPath(
                prepare.candidate_path, planner_.sample_dt(), snapshot.vehicle,
                now, avoid_trajectory);
            if (convert_status == ConversionStatus::OK &&
                !avoid_trajectory.points.empty()) {
                // 3~4번: pure core preflight 뒤 manager와 core를 연속 적용한다.
                // preflight가 실패하면 manager는 READY 그대로이므로
                // COMMITTED/core-unlocked 분리 상태가 남지 않는다.
                committed_now = CommitAvoidAtomically(
                    command_manager_, planner_, prepare);
            } else {
                ROS_WARN_THROTTLE(1.0,
                                  "v2 AVOID candidate conversion failed; "
                                  "staying in READY (command_id=%u)",
                                  command.command_id);
            }
        }

        // 5번: 발행 대상 결정.
        //
        //   COMMIT 성공  -> AVOID candidate를 **처음** MPC로 보낸다
        //   그 외        -> maintenance만 보낸다 (계약 5.2절)
        const asmc_msgs::Trajectory& published =
            committed_now ? avoid_trajectory : trajectory;

        v2_plan_fail_streak_ = 0;
        last_v2_trajectory_ = published;
        has_last_v2_trajectory_ = true;
        last_v2_trajectory_command_id_ = command.command_id;
        last_v2_trajectory_fingerprint_ = fingerprint;

        AvoidFeedbackFields avoid_fields;
        // 실제 발행한 궤적의 목표속도. last_target_speed_는 maintenance 결정
        // 전의 순항 목표속도이므로 감속/정지 maintenance에서 거짓 보고가 된다.
        avoid_fields.applied_speed_mps =
            published.points.empty() ? -1.0
                                     : published.points.back().velocity;
        avoid_fields.target_d_valid = prepare.candidate_valid;
        avoid_fields.target_d =
            prepare.candidate_valid ? prepare.selected_target_d : 0.0;
        avoid_fields.generated_candidate_count =
            static_cast<uint32_t>(prepare.generated_candidate_count);
        avoid_fields.valid_candidate_count =
            static_cast<uint32_t>(prepare.valid_candidate_count);
        avoid_fields.selected_cost =
            prepare.candidate_valid ? prepare.selected_cost : -1.0;
        avoid_fields.blocking_object_id =
            static_cast<int32_t>(prepare.blocking_object_id);

        CommandAcceptance prepared = acceptance;
        prepared.execution_state = command_manager_.execution_state();
        // ===== 후보가 충돌로 막혀도 COLLISION_BLOCKED를 쓰지 않는다 =====
        //
        // PREPARE에서 안전 후보가 없는 것은 실패가 아니라 WAITING이다.
        // COLLISION_BLOCKED(8)를 내보내면 FSM이 terminal 성격으로 오해할
        // 여지가 있다. 차단 정보는 blocking_object_id로 전달한다.
        prepared.result_code = asmc_msgs::FrenetFeedback::RESULT_NONE;
        prepared.detail = committed_now
            ? std::string("avoid maneuver committed")
            : (prepare.candidate_valid
                   ? std::string("avoid candidate is ready")
                   : std::string("no safe avoid candidate this cycle"));

        trajectory_pub_.publish(published);
        const uint8_t trajectory_source =
            committed_now
                ? asmc_msgs::FrenetFeedback::TRAJECTORY_LOCKED
                : asmc_msgs::FrenetFeedback::TRAJECTORY_NEW;
        publishFeedbackV2(feedback_command_id, prepared, &published,
                          trajectory_source,
                          prepare.candidate_valid, ego_s_valid, ego_s,
                          &avoid_fields);

        ROS_INFO_THROTTLE(1.0,
                          "v2 AVOID %s: command_id=%u candidate=%d "
                          "committed=%d target_d=%.3f locked=%.3f phase=%s "
                          "low_speed=%d conflict=%d behind=%d "
                          "candidates=%zu/%zu points=%zu applied=%.2f",
                          execute_requested ? "EXECUTE" : "PREPARE",
                          command.command_id,
                          prepare.candidate_valid ? 1 : 0,
                          committed_now ? 1 : 0,
                          prepare.selected_target_d,
                          planner_.avoid_locked_target_d(),
                          planner_.avoid_phase_name(),
                          prepare.low_speed_context ? 1 : 0,
                          prepare.direction_conflict ? 1 : 0,
                          prepare.target_behind_ego ? 1 : 0,
                          prepare.valid_candidate_count,
                          prepare.generated_candidate_count,
                          published.points.size(),
                          avoid_fields.applied_speed_mps);
        return true;
    }

    // ---- 5. planning ----------------------------------------------------
    CartesianPath path;
    const bool plan_ok = planner_.Plan(ego, objects, collision_objects,
                                       planner_command, path);

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
            publishFeedbackV2(feedback_command_id, transient_acceptance, &reused,
                              asmc_msgs::FrenetFeedback::TRAJECTORY_REUSED,
                              /*plan_valid=*/false);
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
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
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
        publishFeedbackV2(feedback_command_id, failed, &stop,
                          asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP,
                          /*plan_valid=*/false);
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
    publishFeedbackV2(feedback_command_id, succeeded, &trajectory,
                      asmc_msgs::FrenetFeedback::TRAJECTORY_NEW,
                      /*plan_valid=*/true, ego_s_valid, ego_s);

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
    bool plan_valid, bool frenet_state_valid, double current_s,
    const AvoidFeedbackFields* avoid_fields) {
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
    // 실제로 MPC에 무엇을 보냈는지(trajectory_source)와는 별개 축이므로
    // 유도하지 않고 호출자가 정한다(계약 8.3절, 헤더 D6-0 주석 참고).
    feedback.plan_valid = plan_valid;

    // Frenet 상태와 trajectory source는 독립 축이다. post-COMMIT AVOID는
    // LOCKED source여도 이번 cycle에 실제 planning을 했으므로 유효한 속도와
    // s/d를 보낸다. terminal latch는 caller가 state_valid=false로 구분한다.
    const FeedbackKinematics kinematics = ResolveFeedbackKinematics(
        trajectory_source, avoid_fields != nullptr,
        avoid_fields ? avoid_fields->applied_speed_mps : -1.0,
        planner_.last_target_speed(), frenet_state_valid, current_s,
        planner_.last_d());
    feedback.applied_speed_mps = kinematics.applied_speed_mps;
    feedback.frenet_state_valid = kinematics.frenet_state_valid;
    feedback.current_s = kinematics.current_s;
    feedback.current_d = kinematics.current_d;

    // D4-1은 KEEP만 다룬다. 아직 채우지 않는 필드는 메시지가 정의한 sentinel을
    // 정확히 사용한다 - 임의의 0이나 추측값을 넣지 않는다(계약 9.4절).
    feedback.maneuver_progress = avoid_fields
        ? avoid_fields->maneuver_progress : -1.0;
    // 실제로 추종 중인 leader. command.primary_object_id를 그대로 되돌려
    // 보내면 "지정했다"와 "확보했다"를 구분할 수 없어, 확보에 실패한
    // 사이클에도 ID가 나간다(계약 9.2절).
    feedback.active_leader_id = planner_.active_leader_id();
    feedback.selected_front_object_id = -1;
    feedback.selected_rear_object_id = -1;
    feedback.gap_evaluated = false;
    feedback.gap_safe = false;
    feedback.estimated_entry_time_s = -1.0;
    // AVOID PREPARE는 후보 진단을 채운다. 그 외 모드는 메시지가 정의한
    // sentinel을 정확히 사용한다 - 임의의 0이나 추측값을 넣지 않는다
    // (계약 9.4절).
    if (avoid_fields) {
        // target_d는 음수가 유효하므로 -1.0 sentinel을 쓸 수 없다. 유효성은
        // 플래그로만 판단한다(계약 1.3절).
        feedback.target_d_valid = avoid_fields->target_d_valid;
        feedback.target_d = avoid_fields->target_d;
        feedback.generated_candidate_count =
            avoid_fields->generated_candidate_count;
        feedback.valid_candidate_count = avoid_fields->valid_candidate_count;
        feedback.selected_cost = avoid_fields->selected_cost;
        feedback.blocking_object_id = avoid_fields->blocking_object_id;
    } else {
        feedback.target_d_valid = false;
        feedback.target_d = 0.0;
        feedback.generated_candidate_count = 0;
        feedback.valid_candidate_count = 0;
        feedback.selected_cost = -1.0;
        feedback.blocking_object_id = -1;
    }

    (void)trajectory;
    feedback_v2_pub_.publish(feedback);
}

}  // namespace frenet_planner_ros
