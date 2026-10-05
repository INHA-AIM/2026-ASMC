// D4-4: v2 ROS 인터페이스 통합 회귀.
//
// 지금까지 D4-1 ~ D4-3을 각 단계마다 손으로 확인했다. 그 검증을 실행 가능한
// 자산으로 고정한다. D5 이후 모드를 추가할 때마다 케이스를 늘려 재사용한다.
//
// ===== 실행 방법 =====
//
//   터미널 1:  roscore
//   터미널 2:  rosrun frenet_planner frenet_planner_node \
//                _planner/config_path:=<params.yaml 경로>
//   터미널 3:  rosrun frenet_planner v2_ros_regression_test
//
// add_test()에 등록하지 않았다. 실행 중인 roscore와 노드가 필요하므로
// catkin_make run_tests가 자동으로 돌릴 수 없다. D10에서 환경 경로 문제
// (params.yaml의 배포 절대경로)가 정리되면 rostest로 편입한다.
//
// ===== 검증 범위 =====
//
//   KEEP        궤적 생성, desired_speed 반영
//   STOP        target pose 투영, APPROACH->BRAKING->HOLD, COMPLETED latch,
//               HOLD 궤적 유지(LOCKED), heading 검증
//   E-STOP      직접 감속, 단조 감소, v>=0, 마지막 v=0, preemption
//   실패 경로    invalid command, stale command, 미구현 maneuver
//   transaction command_id echo, terminal 사유 latch, 모드 전환
//   AVOID       PREPARE->READY->EXECUTE->COMMIT->EXECUTING->COMPLETED (D6-4a)
//
// D6-4b에서 AVOID edge를 추가했다:
//   handshake   첫 명령부터 EXECUTE여도 READY를 건너뛰지 않는다,
//               WAITING 회복 cycle은 COMMIT하지 않는다
//   target pose heading 44/45/46도 경계, ±pi wrap(NormalizeAngle)
//   대상 소유권  authoritative primary_object_id, LEFT/RIGHT/NONE,
//               optional target pose가 목표 d를 바꾼다
//   WAITING     direction conflict와 실제 collision blocking을 구분한다
//   dropout     COMMIT 이전(즉시/유예/LOST)과 이후(중단하지 않음)
//   소유권      CANCEL 전후, 거부된 새 KEEP, 잘못된/만료된 명령,
//               E-STOP preemption, COMPLETED 뒤 새 transaction
//   Feedback    post-COMMIT collision SAFE_STOP에서 잠긴 진단 유지
//
// 계약: docs/fsm-frenet-interface-v2.md

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/DetectedObjectArray.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;

int g_failures = 0;
int g_checks = 0;

void Expect(bool condition, const std::string& name) {
    ++g_checks;
    if (condition) {
        std::printf("[PASS] %s\n", name.c_str());
    } else {
        std::printf("[FAIL] %s\n", name.c_str());
        ++g_failures;
    }
}

const char* StateName(uint8_t state) {
    switch (state) {
        case Feedback::IDLE: return "IDLE";
        case Feedback::PREPARING: return "PREPARING";
        case Feedback::WAITING: return "WAITING";
        case Feedback::READY: return "READY";
        case Feedback::COMMITTED: return "COMMITTED";
        case Feedback::EXECUTING: return "EXECUTING";
        case Feedback::COMPLETED: return "COMPLETED";
        case Feedback::FAILED: return "FAILED";
        default: return "?";
    }
}

const char* SourceName(uint8_t source) {
    switch (source) {
        case Feedback::TRAJECTORY_NONE: return "NONE";
        case Feedback::TRAJECTORY_NEW: return "NEW";
        case Feedback::TRAJECTORY_REUSED: return "REUSED";
        case Feedback::TRAJECTORY_LOCKED: return "LOCKED";
        case Feedback::TRAJECTORY_SAFE_STOP: return "SAFE_STOP";
        default: return "?";
    }
}

// reference line waypoint. ego/목표 pose를 실제 경로 위에 놓기 위해 읽는다.
struct Waypoints {
    std::vector<double> x;
    std::vector<double> y;
};

Waypoints LoadWaypoints(const std::string& path) {
    std::ifstream input(path);
    Waypoints waypoints;
    double x = 0.0, y = 0.0, ignored = 0.0;
    while (input >> x >> y >> ignored) {
        waypoints.x.push_back(x);
        waypoints.y.push_back(y);
    }
    return waypoints;
}

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// 누적 호길이가 target_s에 도달하는 waypoint의 pose.
Pose2D PoseAt(const Waypoints& waypoints, double target_s) {
    double s = 0.0;
    std::size_t index = 1;
    for (; index < waypoints.x.size(); ++index) {
        s += std::hypot(waypoints.x[index] - waypoints.x[index - 1],
                        waypoints.y[index] - waypoints.y[index - 1]);
        if (s >= target_s) break;
    }
    if (index >= waypoints.x.size()) index = waypoints.x.size() - 1;
    Pose2D pose;
    pose.x = waypoints.x[index];
    pose.y = waypoints.y[index];
    pose.yaw = std::atan2(waypoints.y[index] - waypoints.y[index - 1],
                          waypoints.x[index] - waypoints.x[index - 1]);
    return pose;
}

Pose2D PoseAtOffset(const Waypoints& waypoints, double target_s,
                    double lateral_d) {
    Pose2D pose = PoseAt(waypoints, target_s);
    pose.x -= lateral_d * std::sin(pose.yaw);
    pose.y += lateral_d * std::cos(pose.yaw);
    return pose;
}

double LateralOffsetAt(const Waypoints& waypoints, double x, double y) {
    if (waypoints.x.size() < 2) return std::nan("");
    std::size_t best = 1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < waypoints.x.size(); ++i) {
        const double dx = x - waypoints.x[i];
        const double dy = y - waypoints.y[i];
        const double distance = dx * dx + dy * dy;
        if (distance < best_distance) {
            best_distance = distance;
            best = i;
        }
    }
    const double yaw = std::atan2(
        waypoints.y[best] - waypoints.y[best - 1],
        waypoints.x[best] - waypoints.x[best - 1]);
    const double dx = x - waypoints.x[best];
    const double dy = y - waypoints.y[best];
    return -dx * std::sin(yaw) + dy * std::cos(yaw);
}

double TrajectoryEndpointD(const Waypoints& waypoints,
                           const asmc_msgs::Trajectory* trajectory) {
    if (!trajectory || trajectory->points.empty()) return std::nan("");
    const auto& point = trajectory->points.back();
    return LateralOffsetAt(waypoints, point.x, point.y);
}

bool SameTrajectoryGeometry(const asmc_msgs::Trajectory& lhs,
                            const asmc_msgs::Trajectory& rhs) {
    if (lhs.points.size() != rhs.points.size()) return false;
    for (std::size_t i = 0; i < lhs.points.size(); ++i) {
        if (std::fabs(lhs.points[i].x - rhs.points[i].x) > 1e-9 ||
            std::fabs(lhs.points[i].y - rhs.points[i].y) > 1e-9 ||
            std::fabs(lhs.points[i].yaw - rhs.points[i].yaw) > 1e-9 ||
            std::fabs(lhs.points[i].velocity - rhs.points[i].velocity) > 1e-9) {
            return false;
        }
    }
    return true;
}

// 노드와 주고받는 한 사이클을 담당한다. 명령/입력을 20 Hz로 발행하고
// 피드백과 궤적을 모은다.
class Harness {
 public:
    Harness(ros::NodeHandle& nh, const Waypoints& waypoints)
        : waypoints_(waypoints) {
        feedback_sub_ = nh.subscribe<Feedback>(
            "/planning/frenet_feedback_v2", 32,
            [this](const Feedback::ConstPtr& message) {
                if (message) feedback_.push_back(*message);
            });
        trajectory_sub_ = nh.subscribe<asmc_msgs::Trajectory>(
            "/planning/trajectory", 32,
            [this](const asmc_msgs::Trajectory::ConstPtr& message) {
                if (message) trajectory_.push_back(*message);
            });
        command_pub_ = nh.advertise<Command>("/behavior/command_v2", 1);
        vehicle_pub_ = nh.advertise<asmc_msgs::VehicleState>("/vehicle/state", 1);
        objects_pub_ =
            nh.advertise<asmc_msgs::DetectedObjectArray>("/perception/objects", 1);
    }

    // 연결이 성립할 때까지 기다린다. latch 없는 topic이라 이 대기가 없으면
    // 첫 명령이 유실된다.
    bool WaitForConnections(double timeout_seconds) {
        const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_seconds);
        while (ros::ok() && ros::Time::now() < deadline) {
            // 노드가 두 개 이상 떠 있으면 두 인스턴스가 같은 topic에 피드백을
            // 발행해 검증 결과가 뒤섞인다. 실측에서 이 상황으로 34/34가
            // 33/34로 보인 적이 있어 환경 오류로 먼저 잡는다.
            if (command_pub_.getNumSubscribers() > 1 ||
                feedback_sub_.getNumPublishers() > 1) {
                std::printf("[FAIL] more than one frenet_planner_node is "
                            "running (command subscribers=%u, feedback "
                            "publishers=%u); results would be mixed\n",
                            command_pub_.getNumSubscribers(),
                            feedback_sub_.getNumPublishers());
                return false;
            }
            if (command_pub_.getNumSubscribers() > 0 &&
                feedback_sub_.getNumPublishers() > 0) {
                // 노드가 구독을 등록한 직후에는 아직 큐가 준비되지 않을 수
                // 있어 한 박자 더 기다린다.
                ros::Duration(0.5).sleep();
                return true;
            }
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        }
        return false;
    }

    Command MakeCommand(uint32_t command_id, uint8_t maneuver) const {
        Command command;
        command.header.frame_id = "map";
        command.command_id = command_id;
        command.maneuver_type = maneuver;
        command.execution_request = Command::EXECUTE;
        command.direction = Command::DIRECTION_NONE;
        command.merge_type = Command::MERGE_NONE;
        command.desired_speed_mps = -1.0;
        command.primary_object_id = -1;
        command.preferred_front_object_id = -1;
        command.preferred_rear_object_id = -1;
        command.use_custom_follow_gap = false;
        command.desired_time_gap_s = -1.0;
        command.minimum_gap_m = -1.0;
        return command;
    }

    // 목표 pose를 reference line 위에 놓고 heading도 맞춘다. heading을 맞추지
    // 않으면 D4-2a 검증이 정상 목표를 거부한다.
    void SetTargetOnPath(Command& command, double target_s) const {
        const Pose2D pose = PoseAt(waypoints_, target_s);
        command.has_target_pose = true;
        command.target_pose_map.position.x = pose.x;
        command.target_pose_map.position.y = pose.y;
        command.target_pose_map.orientation.z = std::sin(pose.yaw * 0.5);
        command.target_pose_map.orientation.w = std::cos(pose.yaw * 0.5);
    }

    // 명령을 cycles회 발행한다. advance=true이면 직전 궤적을 따라 ego를
    // 전진시켜 실제 주행에 가깝게 만든다(STOP 단계 전이 관찰용).
    void Run(const Command& command, const Pose2D& start, double speed,
             int cycles, bool advance) {
        first_new_index_ = -1;
        begin_index_ = static_cast<int>(feedback_.size());
        Pose2D pose = start;
        double current_speed = speed;
        for (int cycle = 0; cycle < cycles && ros::ok(); ++cycle) {
            PublishInputs(pose, current_speed);
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() + ros::Duration(2.0);
            command_pub_.publish(stamped);
            SpinFor(0.05);
            if (advance && !trajectory_.empty() &&
                trajectory_.back().points.size() > 2) {
                const auto& point = trajectory_.back().points[2];
                pose.x = point.x;
                pose.y = point.y;
                pose.yaw = point.yaw;
                current_speed = point.velocity;
            }
        }
        // 명령 발행을 멈춘 뒤에도 입력은 유지한다. 마지막 사이클의 피드백을
        // 받기 위한 것이며, 입력을 끊으면 freshness 검사가 먼저 걸린다.
        for (int cycle = 0; cycle < 4 && ros::ok(); ++cycle) {
            PublishInputs(pose, current_speed);
            SpinFor(0.05);
        }
    }

    // 명령은 계속 보내면서 vehicle/object 발행만 멈춘다.
    //
    // BehaviorCommand.valid_until 만료(STALE_COMMAND)와 완전히 다른 축이다.
    // 명령은 정상이고 Ego/Object 입력만 오래된 상태이며, transaction을
    // 버리지 않고 이 사이클의 planning만 막아야 한다(계약 11.3절).
    void RunWithoutInputs(const Command& command, int cycles) {
        begin_index_ = static_cast<int>(feedback_.size());
        for (int cycle = 0; cycle < cycles && ros::ok(); ++cycle) {
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() + ros::Duration(2.0);
            command_pub_.publish(stamped);
            SpinFor(0.05);
        }
    }

    // 사이클 단위로 명령을 보낸다. Run()과 달리 창을 새로 열지 않아서
    // 여러 구간(정상 -> dropout -> 재등장)을 한 transaction으로 이어붙일 수
    // 있다. dropout grace를 사이클 수로 정확히 세려면 이 제어가 필요하다.
    void BeginWindow() {
        begin_index_ = static_cast<int>(feedback_.size());
        trajectory_begin_index_ = static_cast<int>(trajectory_.size());
    }

    void RunCycles(const Command& command, const Pose2D& pose, double speed,
                   int cycles) {
        for (int cycle = 0; cycle < cycles && ros::ok(); ++cycle) {
            PublishInputs(pose, speed);
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() + ros::Duration(2.0);
            command_pub_.publish(stamped);
            SpinFor(0.05);
        }
    }

    // 특정 command_id가 요구한 progress 이상에 도달할 때까지 돌린다 (D6-4b).
    //
    // RunUntilState(EXECUTING)은 **첫** EXECUTING 피드백에서 멈추므로, ego를
    // 잠긴 offset으로 옮긴 직후에는 아직 SHIFT 단계(progress<0.5)인 프레임이
    // 잡힐 수 있다. 실측에서 6회 중 1회 그렇게 어긋났다. 어느 프레임이
    // 잡히느냐는 20 Hz timer와의 위상 문제일 뿐이므로, 판정 자체를 약하게
    // 만들지 않고 "PASS에 도달한 프레임"을 기다린다.
    bool RunUntilProgress(const Command& command, const Pose2D& pose,
                          double speed, uint8_t expected_state,
                          double minimum_progress, int max_cycles,
                          Feedback& out) {
        BeginWindow();
        for (int cycle = 0; cycle < max_cycles && ros::ok(); ++cycle) {
            PublishInputs(pose, speed);
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() + ros::Duration(2.0);
            command_pub_.publish(stamped);
            SpinFor(0.06);
            for (std::size_t i = static_cast<std::size_t>(begin_index_);
                 i < feedback_.size(); ++i) {
                if (feedback_[i].command_id == command.command_id &&
                    feedback_[i].execution_state == expected_state &&
                    feedback_[i].maneuver_progress >= minimum_progress) {
                    out = feedback_[i];
                    return true;
                }
            }
        }
        return false;
    }

    bool RunUntilState(const Command& command, const Pose2D& pose,
                       double speed, uint8_t expected_state, int max_cycles,
                       Feedback& out) {
        BeginWindow();
        for (int cycle = 0; cycle < max_cycles && ros::ok(); ++cycle) {
            PublishInputs(pose, speed);
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() + ros::Duration(2.0);
            command_pub_.publish(stamped);
            SpinFor(0.06);
            for (std::size_t i = static_cast<std::size_t>(begin_index_);
                 i < feedback_.size(); ++i) {
                if (feedback_[i].command_id == command.command_id &&
                    feedback_[i].execution_state == expected_state) {
                    out = feedback_[i];
                    return true;
                }
            }
        }
        return false;
    }

    // 명령 발행을 멈추고 입력만 유지한다. 마지막 사이클의 피드백을 받기
    // 위한 것이며, 입력을 끊으면 freshness 검사가 먼저 걸린다.
    void Settle(const Pose2D& pose, double speed, int cycles = 4) {
        for (int cycle = 0; cycle < cycles && ros::ok(); ++cycle) {
            PublishInputs(pose, speed);
            SpinFor(0.05);
        }
    }

    // 만료된 명령을 한 번 보낸다(stale 검증용).
    void RunExpired(const Command& command, const Pose2D& start, double speed) {
        begin_index_ = static_cast<int>(feedback_.size());
        for (int cycle = 0; cycle < 10 && ros::ok(); ++cycle) {
            PublishInputs(start, speed);
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() - ros::Duration(1.0);
            command_pub_.publish(stamped);
            SpinFor(0.05);
        }
        for (int cycle = 0; cycle < 4 && ros::ok(); ++cycle) {
            PublishInputs(start, speed);
            SpinFor(0.05);
        }
    }

    // 이번 Run에서 받은 피드백.
    std::vector<Feedback> Collected() const {
        if (begin_index_ < 0 ||
            begin_index_ >= static_cast<int>(feedback_.size())) {
            return {};
        }
        return std::vector<Feedback>(feedback_.begin() + begin_index_,
                                     feedback_.end());
    }

    bool HasFeedback() const { return !Collected().empty(); }
    Feedback Last() const { return Collected().back(); }

    // 이번 Run에서 특정 command_id에 대한 마지막 피드백.
    bool LastFor(uint32_t command_id, Feedback& out) const {
        const auto collected = Collected();
        for (auto it = collected.rbegin(); it != collected.rend(); ++it) {
            if (it->command_id == command_id) { out = *it; return true; }
        }
        return false;
    }

    // 이번 Run에서 TRAJECTORY_NEW를 처음 받은 피드백.
    bool FirstNew(Feedback& out) const {
        for (const auto& feedback : Collected()) {
            if (feedback.trajectory_source == Feedback::TRAJECTORY_NEW) {
                out = feedback;
                return true;
            }
        }
        return false;
    }

    bool SawState(uint8_t state) const {
        for (const auto& feedback : Collected()) {
            if (feedback.execution_state == state) return true;
        }
        return false;
    }

    bool SawSource(uint8_t source) const {
        for (const auto& feedback : Collected()) {
            if (feedback.trajectory_source == source) return true;
        }
        return false;
    }

    const asmc_msgs::Trajectory* LastTrajectory() const {
        return trajectory_.empty() ? nullptr : &trajectory_.back();
    }

    // **이번 window에서 새로 받은** 궤적만 반환한다.
    //
    // LastTrajectory()는 실행 전체의 마지막 궤적을 주므로, 이번 명령이 궤적을
    // 만들지 못했을 때 직전 명령의 것을 그대로 돌려준다. 그러면 두 명령을
    // 비교하는 검증이 "값이 같다"는 이유로 통과해버린다(false PASS).
    //
    // 예: default FOLLOW가 궤적 A를 만들고, custom FOLLOW가 planning에
    // 실패하면 두 조회 모두 A를 반환해 "custom <= default"가 성립한다.
    const asmc_msgs::Trajectory* LastTrajectoryInWindow() const {
        if (trajectory_begin_index_ < 0 ||
            trajectory_begin_index_ >= static_cast<int>(trajectory_.size())) {
            return nullptr;
        }
        return &trajectory_.back();
    }

    // 이번 window에서 받은 궤적 개수. 0이면 이 명령은 궤적을 만들지 못했다.
    std::size_t TrajectoryCountInWindow() const {
        if (trajectory_begin_index_ < 0) return 0;
        const int total = static_cast<int>(trajectory_.size());
        return total > trajectory_begin_index_
                   ? static_cast<std::size_t>(total - trajectory_begin_index_)
                   : 0;
    }

    void PrintSummary(const std::string& label) const {
        const auto collected = Collected();
        if (collected.empty()) {
            std::printf("  %-30s (no feedback)\n", label.c_str());
            return;
        }
        // 상태 전이를 압축해서 출력한다.
        std::printf("  %-30s", label.c_str());
        std::string previous;
        int repeat = 0;
        for (const auto& feedback : collected) {
            std::string key = std::string(StateName(feedback.execution_state)) +
                              "/" + SourceName(feedback.trajectory_source);
            if (key == previous) { ++repeat; continue; }
            if (!previous.empty()) std::printf(" %s x%d", previous.c_str(), repeat);
            previous = key;
            repeat = 1;
        }
        if (!previous.empty()) std::printf(" %s x%d", previous.c_str(), repeat);
        std::printf("\n");
    }

    // FOLLOW 검증용 객체 목록을 설정한다. 다음 PublishInputs부터 반영된다.
    void SetObjects(const std::vector<asmc_msgs::DetectedObject>& objects) {
        objects_ = objects;
    }

    // 목표 pose를 경로에서 lateral_d 만큼 비켜난 곳에 놓는다 (D6-4b).
    //
    // SetTargetOnPath는 항상 중앙선 위에 놓으므로 AVOID의 optional target
    // pose가 목표 d를 실제로 바꾸는지 검증할 수 없다. heading은 투영점의
    // reference yaw를 그대로 써서 heading 검증(계약 3.4절 4항)과 분리한다.
    void SetTargetOffPath(Command& command, double target_s,
                          double lateral_d) const {
        const Pose2D reference = PoseAt(waypoints_, target_s);
        const Pose2D pose = PoseAtOffset(waypoints_, target_s, lateral_d);
        command.has_target_pose = true;
        command.target_pose_map.position.x = pose.x;
        command.target_pose_map.position.y = pose.y;
        command.target_pose_map.orientation.z = std::sin(reference.yaw * 0.5);
        command.target_pose_map.orientation.w = std::cos(reference.yaw * 0.5);
    }

    // 목표 pose를 경로 위에 놓되 heading만 지정한 오차만큼 틀어 놓는다.
    // heading 임계값(기본 45도) 경계 검증용이다 (D6-4b).
    void SetTargetWithHeadingError(Command& command, double target_s,
                                   double heading_error_rad) const {
        const Pose2D pose = PoseAt(waypoints_, target_s);
        const double yaw = pose.yaw + heading_error_rad;
        command.has_target_pose = true;
        command.target_pose_map.position.x = pose.x;
        command.target_pose_map.position.y = pose.y;
        command.target_pose_map.orientation.z = std::sin(yaw * 0.5);
        command.target_pose_map.orientation.w = std::cos(yaw * 0.5);
    }

    // 목표 pose를 map 좌표와 절대 yaw로 직접 지정한다. ±pi wrap fixture는
    // reference yaw를 그대로 쓰지 않고 부호가 반대인 값을 넣어야 한다.
    void SetTargetPoseRaw(Command& command, const Pose2D& position,
                          double yaw) const {
        command.has_target_pose = true;
        command.target_pose_map.position.x = position.x;
        command.target_pose_map.position.y = position.y;
        command.target_pose_map.orientation.z = std::sin(yaw * 0.5);
        command.target_pose_map.orientation.w = std::cos(yaw * 0.5);
    }

    // reference yaw가 ±pi에 가장 먼저 근접하는 지점의 호길이 [m] (D6-4b).
    //
    // NormalizeAngle 경로를 실제 트랙으로 통과시키기 위한 fixture다. 임계값을
    // 억지로 바꾸지 않고 waypoint yaw 분포에서 실제 wrap 구간을 찾는다.
    // 현재 트랙에는 s≈2020 부근에 -179.7deg -> +179.5deg 전이가 있다.
    double FindPoseWithHeadingNearPi(double tolerance_rad) const {
        double s = 0.0;
        for (std::size_t i = 1; i < waypoints_.x.size(); ++i) {
            s += std::hypot(waypoints_.x[i] - waypoints_.x[i - 1],
                            waypoints_.y[i] - waypoints_.y[i - 1]);
            const double yaw = std::atan2(
                waypoints_.y[i] - waypoints_.y[i - 1],
                waypoints_.x[i] - waypoints_.x[i - 1]);
            if (std::fabs(std::fabs(yaw) - M_PI) < tolerance_rad) return s;
        }
        return -1.0;
    }

    // 경로에서 lateral_d 만큼 비켜난 정적 장애물 (D6-4b).
    //
    // MakeVehicle은 항상 중앙선/고정 폭이므로 두 객체를 만들면 목표 d가
    // 같아져 authoritative target 검증이 약해진다. 실측 기준(D6-4b):
    //   d=-0.8 w=2.0 -> target_d 약 +1.73
    //   d=+0.8 w=2.0 -> target_d 약 +3.82
    // 이 차이가 nearest-selector 돌연변이를 잡는다.
    asmc_msgs::DetectedObject MakeStaticObjectAtOffset(
        int id, double target_s, double lateral_d, double width,
        double length) const {
        const Pose2D pose = PoseAtOffset(waypoints_, target_s, lateral_d);
        const Pose2D reference = PoseAt(waypoints_, target_s);
        asmc_msgs::DetectedObject object;
        object.id = id;
        object.center.x = pose.x;
        object.center.y = pose.y;
        object.yaw = reference.yaw;
        object.vx = 0.0;
        object.vy = 0.0;
        object.width = width;
        object.length = length;
        object.height = 1.5;
        const double half_length = length * 0.5;
        const double half_width = width * 0.5;
        const double cos_yaw = std::cos(reference.yaw);
        const double sin_yaw = std::sin(reference.yaw);
        const double offsets[4][2] = {
            {-half_length, -half_width}, {half_length, -half_width},
            {half_length, half_width}, {-half_length, half_width},
        };
        for (std::size_t i = 0; i < 4; ++i) {
            const double x = pose.x + offsets[i][0] * cos_yaw -
                             offsets[i][1] * sin_yaw;
            const double y = pose.y + offsets[i][0] * sin_yaw +
                             offsets[i][1] * cos_yaw;
            object.obb_corners[i].x = x;
            object.obb_corners[i].y = y;
            object.obb_corners[i].z = 0.0;
            object.obb_corners[i + 4].x = x;
            object.obb_corners[i + 4].y = y;
            object.obb_corners[i + 4].z = object.height;
        }
        return object;
    }

    // 명령은 계속 보내면서 vehicle/object 발행만 멈춘 채 한 window를 돈다.
    // RunWithoutInputs와 달리 명령 payload를 그대로 두고 창만 새로 연다.
    void RunCyclesWithoutInputs(const Command& command, int cycles) {
        for (int cycle = 0; cycle < cycles && ros::ok(); ++cycle) {
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() + ros::Duration(2.0);
            command_pub_.publish(stamped);
            SpinFor(0.06);
        }
    }

    // valid_until이 이미 지난 명령을 cycles회 보낸다. 입력은 정상 유지한다.
    // 잠긴 기동이 stale 명령에 끊기지 않는지 확인하는 데 쓴다 (D6-4b).
    void RunExpiredCycles(const Command& command, const Pose2D& pose,
                          double speed, int cycles) {
        for (int cycle = 0; cycle < cycles && ros::ok(); ++cycle) {
            PublishInputs(pose, speed);
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() - ros::Duration(1.0);
            command_pub_.publish(stamped);
            SpinFor(0.06);
        }
    }

    // 이번 window에서 해당 command_id의 **첫** 피드백.
    bool FirstFor(uint32_t command_id, Feedback& out) const {
        for (const auto& feedback : Collected()) {
            if (feedback.command_id == command_id) { out = feedback; return true; }
        }
        return false;
    }

    // 해당 command_id의 첫 피드백이 도착할 때까지만 명령을 보낸다 (D6-4b).
    //
    // "한 cycle 돌리고 첫 피드백을 본다"로는 20 Hz timer와 어긋나 window가
    // 비는 경우가 있다. 반대로 여러 cycle을 돌린 뒤 첫 피드백을 고르면 그
    // window에 이미 다음 상태들이 섞여 들어와, "같은 cycle에 COMMIT하지
    // 않았다"를 window 전체로 판정할 수 없다.
    //
    // 그래서 첫 피드백이 나오면 즉시 멈추고, **그 시점까지 이 id로 관측한
    // 상태 수**도 함께 돌려준다. 호출자는 그 개수로 "첫 결과만 보았다"를
    // 확인한다.
    bool RunUntilFirstFeedback(const Command& command, const Pose2D& pose,
                               double speed, int max_cycles, Feedback& out,
                               std::size_t& observed_count) {
        BeginWindow();
        for (int cycle = 0; cycle < max_cycles && ros::ok(); ++cycle) {
            PublishInputs(pose, speed);
            Command stamped = command;
            stamped.header.stamp = ros::Time::now();
            stamped.valid_until = ros::Time::now() + ros::Duration(2.0);
            command_pub_.publish(stamped);
            SpinFor(0.06);
            if (FirstFor(command.command_id, out)) {
                observed_count = CountFor(command.command_id);
                return true;
            }
        }
        observed_count = 0;
        return false;
    }

    // 이번 window에서 해당 command_id의 피드백 개수.
    std::size_t CountFor(uint32_t command_id) const {
        std::size_t count = 0;
        for (const auto& feedback : Collected()) {
            if (feedback.command_id == command_id) ++count;
        }
        return count;
    }

    // 이번 window에서 해당 command_id가 특정 상태였던 마지막 피드백.
    bool LastForState(uint32_t command_id, uint8_t state, Feedback& out) const {
        const auto collected = Collected();
        for (auto it = collected.rbegin(); it != collected.rend(); ++it) {
            if (it->command_id == command_id && it->execution_state == state) {
                out = *it;
                return true;
            }
        }
        return false;
    }

    // 이번 window에서 first가 second보다 먼저 관측되었는지 (D6-4b).
    //
    // "READY를 거치지 않고 COMMIT하지 않는다"처럼 **순서**가 계약인 검증에
    // 쓴다. 단순히 "COMMITTED를 보지 못했다"로 판정하면, 한 spin에 여러
    // 피드백이 쌓여 READY 다음 cycle의 정상 COMMIT까지 같은 window에 들어올
    // 때 거짓 실패가 난다. 순서를 보면 그 위상 문제와 실제 계약 위반
    // (READY 없이 곧바로 COMMITTED)이 구분된다.
    bool SawStateBefore(uint32_t command_id, uint8_t first,
                        uint8_t second) const {
        for (const auto& feedback : Collected()) {
            if (feedback.command_id != command_id) continue;
            if (feedback.execution_state == first) return true;
            if (feedback.execution_state == second) return false;
        }
        return false;
    }

    // 이번 window에서 해당 command_id가 특정 상태를 관측했는지.
    bool SawStateFor(uint32_t command_id, uint8_t state) const {
        for (const auto& feedback : Collected()) {
            if (feedback.command_id == command_id &&
                feedback.execution_state == state) {
                return true;
            }
        }
        return false;
    }

    // 경로 위 target_s 지점의 차량. FOLLOW 대상 검증(진행 방향/같은 차선/
    // 전방/범위)을 통과하도록 heading과 속도를 맞춘다.
    asmc_msgs::DetectedObject MakeVehicle(int id, double target_s,
                                          double speed) const {
        const Pose2D pose = PoseAt(waypoints_, target_s);
        asmc_msgs::DetectedObject object;
        object.id = id;
        object.center.x = pose.x;
        object.center.y = pose.y;
        object.yaw = pose.yaw;
        object.vx = speed * std::cos(pose.yaw);
        object.vy = speed * std::sin(pose.yaw);
        object.width = 2.0;
        object.length = 4.5;
        object.height = 1.5;
        // perception이 주는 authoritative OBB. 8 corners가 유효하지 않으면
        // message_converter가 INVALID_OBJECT_GEOMETRY로 거부한다.
        const double half_length = object.length * 0.5;
        const double half_width = object.width * 0.5;
        const double cos_yaw = std::cos(pose.yaw);
        const double sin_yaw = std::sin(pose.yaw);
        const double offsets[4][2] = {
            {-half_length, -half_width}, {half_length, -half_width},
            {half_length, half_width}, {-half_length, half_width},
        };
        for (std::size_t i = 0; i < 4; ++i) {
            const double x = pose.x + offsets[i][0] * cos_yaw -
                             offsets[i][1] * sin_yaw;
            const double y = pose.y + offsets[i][0] * sin_yaw +
                             offsets[i][1] * cos_yaw;
            object.obb_corners[i].x = x;
            object.obb_corners[i].y = y;
            object.obb_corners[i].z = 0.0;
            object.obb_corners[i + 4].x = x;
            object.obb_corners[i + 4].y = y;
            object.obb_corners[i + 4].z = object.height;
        }
        return object;
    }

 private:
    void PublishInputs(const Pose2D& pose, double speed) {
        const ros::Time stamp = ros::Time::now();
        asmc_msgs::VehicleState vehicle;
        vehicle.header.stamp = stamp;
        vehicle.header.frame_id = "map";
        vehicle.valid = true;
        vehicle.x = pose.x;
        vehicle.y = pose.y;
        vehicle.yaw = pose.yaw;
        vehicle.speed = speed;
        vehicle_pub_.publish(vehicle);

        asmc_msgs::DetectedObjectArray objects;
        objects.header.stamp = stamp;
        objects.header.frame_id = "map";
        objects.objects = objects_;
        objects_pub_.publish(objects);
    }

    void SpinFor(double seconds) {
        const ros::Time deadline = ros::Time::now() + ros::Duration(seconds);
        while (ros::ok() && ros::Time::now() < deadline) {
            ros::spinOnce();
            ros::Duration(0.005).sleep();
        }
    }

    Waypoints waypoints_;
    ros::Subscriber feedback_sub_;
    ros::Subscriber trajectory_sub_;
    ros::Publisher command_pub_;
    ros::Publisher vehicle_pub_;
    ros::Publisher objects_pub_;
    std::vector<asmc_msgs::DetectedObject> objects_;
    std::vector<Feedback> feedback_;
    std::vector<asmc_msgs::Trajectory> trajectory_;
    int begin_index_ = -1;
    int trajectory_begin_index_ = -1;
    int first_new_index_ = -1;
};

// 궤적이 정지 계약(모든 v>=0, 단조 감소, 마지막 v=0)을 만족하는지.
bool IsDecelToZero(const asmc_msgs::Trajectory& trajectory) {
    if (trajectory.points.empty()) return false;
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
        const double v = trajectory.points[i].velocity;
        if (!std::isfinite(v) || v < 0.0) return false;
        if (i > 0 && v > trajectory.points[i - 1].velocity + 1e-9) return false;
    }
    return trajectory.points.back().velocity == 0.0;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "v2_ros_regression_test");
    ros::NodeHandle nh;

    std::string waypoint_file = FRENET_TEST_WAYPOINT_FILE;
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("waypoint_file", waypoint_file, waypoint_file);

    const Waypoints waypoints = LoadWaypoints(waypoint_file);
    if (waypoints.x.size() < 100) {
        std::printf("[FAIL] waypoint load: %s\n", waypoint_file.c_str());
        return 1;
    }

    Harness harness(nh, waypoints);
    if (!harness.WaitForConnections(20.0)) {
        std::printf("[FAIL] frenet_planner_node is not running "
                    "(no connection on /behavior/command_v2 or "
                    "/planning/frenet_feedback_v2)\n");
        return 1;
    }

    // ego는 회전교차로/저속 fallback과 무관한 평범한 구간에 둔다.
    const Pose2D ego = PoseAt(waypoints, 200.0);
    constexpr double kCruiseSpeed = 8.0;
    uint32_t command_id = 7000;

    std::printf("\n=== KEEP ===\n");
    {
        const uint32_t id = ++command_id;
        harness.Run(harness.MakeCommand(id, Command::KEEP), ego, kCruiseSpeed,
                    14, false);
        harness.PrintSummary("KEEP");
        Feedback feedback;
        Expect(harness.FirstNew(feedback), "KEEP produces a new trajectory");
        if (harness.HasFeedback()) {
            const Feedback last = harness.Last();
            Expect(last.command_id == id, "KEEP echoes the command_id");
            Expect(last.command_accepted, "KEEP is accepted");
            Expect(last.execution_state == Feedback::EXECUTING,
                   "KEEP reports EXECUTING");
            Expect(last.active_maneuver == Command::KEEP,
                   "KEEP reports active_maneuver=KEEP");
            Expect(last.plan_valid, "KEEP reports plan_valid");
            Expect(last.result_code == Feedback::RESULT_NONE,
                   "KEEP reports no error");
            Expect(last.frenet_state_valid && last.current_s > 0.0,
                   "KEEP reports a valid frenet state");
        }
        const auto* trajectory = harness.LastTrajectory();
        Expect(trajectory != nullptr && trajectory->points.size() > 1 &&
                   trajectory->dt > 0.0,
               "KEEP trajectory has samples and a positive dt");
    }

    std::printf("\n=== KEEP with desired_speed ===\n");
    {
        const uint32_t id = ++command_id;
        Command command = harness.MakeCommand(id, Command::KEEP);
        command.desired_speed_mps = 5.0;
        harness.Run(command, ego, kCruiseSpeed, 14, false);
        // 이 명령의 마지막 피드백으로 확인한다. FirstNew()는 창 안의 첫
        // TRAJECTORY_NEW를 돌려주는데, 노드가 막 뜬 직후에는 직전 케이스의
        // 잔여 피드백이 먼저 잡혀 값이 어긋난다(실측: 첫 실행만 실패).
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) &&
                   feedback.trajectory_source == Feedback::TRAJECTORY_NEW &&
                   std::fabs(feedback.applied_speed_mps - 5.0) < 1e-6,
               "desired_speed_mps is applied");
    }

    std::printf("\n=== STOP ===\n");
    {
        const uint32_t id = ++command_id;
        Command command = harness.MakeCommand(id, Command::STOP);
        harness.SetTargetOnPath(command, 230.0);
        // ego를 궤적에 따라 전진시켜 APPROACH -> BRAKING -> HOLD를 관찰한다.
        harness.Run(command, ego, kCruiseSpeed, 90, true);
        harness.PrintSummary("STOP");
        Expect(harness.SawState(Feedback::EXECUTING),
               "STOP passes through EXECUTING");
        Expect(harness.SawState(Feedback::COMPLETED),
               "STOP reaches COMPLETED at HOLD");
        // 계약 6.2절: terminal latch 중에도 궤적 출력이 멈추면 안 된다.
        // 일괄 안전정지로 빠지면 이 검사가 실패한다.
        Expect(harness.SawSource(Feedback::TRAJECTORY_LOCKED),
               "STOP holds its trajectory after completion (not SAFE_STOP)");
        if (harness.HasFeedback()) {
            const Feedback last = harness.Last();
            Expect(last.command_id == id, "STOP echoes the command_id");
            Expect(last.active_maneuver == Command::STOP,
                   "STOP reports active_maneuver=STOP");
        }
    }

    std::printf("\n=== STOP with a bad target heading ===\n");
    {
        const uint32_t id = ++command_id;
        Command command = harness.MakeCommand(id, Command::STOP);
        harness.SetTargetOnPath(command, 230.0);
        // 목표 위치는 경로 위에 있지만 방향을 반대로 돌린다. 왕복 구간이
        // 가까이 지나갈 때 반대 차선 정지선에 붙는 상황을 재현한다.
        const Pose2D target = PoseAt(waypoints, 230.0);
        const double opposite = target.yaw + M_PI;
        command.target_pose_map.orientation.z = std::sin(opposite * 0.5);
        command.target_pose_map.orientation.w = std::cos(opposite * 0.5);
        harness.Run(command, ego, kCruiseSpeed, 12, false);
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) &&
                   feedback.result_code == Feedback::TARGET_PROJECTION_FAILED,
               "opposite target heading is TARGET_PROJECTION_FAILED");
        // 사유가 latch되어 다음 사이클에도 유지되는지(계약 6절).
        Expect(harness.LastFor(id, feedback) &&
                   feedback.execution_state == Feedback::FAILED,
               "heading failure latches FAILED");
    }

    std::printf("\n=== STOP with an off-path target ===\n");
    {
        const uint32_t id = ++command_id;
        Command command = harness.MakeCommand(id, Command::STOP);
        command.has_target_pose = true;
        command.target_pose_map.position.x = ego.x + 500.0;
        command.target_pose_map.position.y = ego.y + 500.0;
        command.target_pose_map.orientation.w = 1.0;
        harness.Run(command, ego, kCruiseSpeed, 12, false);
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) &&
                   feedback.result_code == Feedback::TARGET_PROJECTION_FAILED,
               "off-path target is TARGET_PROJECTION_FAILED");
    }

    std::printf("\n=== EMERGENCY_STOP ===\n");
    {
        const uint32_t id = ++command_id;
        // has_target_pose=false로 보낸다. 정지 위치 계산은 Frenet 책임이다.
        harness.Run(harness.MakeCommand(id, Command::EMERGENCY_STOP), ego,
                    kCruiseSpeed, 14, false);
        harness.PrintSummary("E-STOP");
        Feedback feedback;
        Expect(harness.FirstNew(feedback),
               "EMERGENCY_STOP produces a new trajectory without a target pose");
        if (harness.HasFeedback()) {
            const Feedback last = harness.Last();
            Expect(last.active_maneuver == Command::EMERGENCY_STOP,
                   "EMERGENCY_STOP reports active_maneuver=EMERGENCY_STOP");
            // E-stop은 한 번 멈추고 끝나는 maneuver가 아니다. 새 정상 명령이
            // 올 때까지 긴급정지 상태를 계속 강제한다.
            Expect(last.execution_state == Feedback::EXECUTING,
                   "EMERGENCY_STOP stays EXECUTING (not COMPLETED)");
            Expect(std::fabs(last.applied_speed_mps) < 1e-9,
                   "EMERGENCY_STOP reports a zero applied speed");
        }
        const auto* trajectory = harness.LastTrajectory();
        Expect(trajectory != nullptr && IsDecelToZero(*trajectory),
               "EMERGENCY_STOP trajectory decelerates monotonically to zero");
    }

    std::printf("\n=== EMERGENCY_STOP at standstill ===\n");
    {
        const uint32_t id = ++command_id;
        harness.Run(harness.MakeCommand(id, Command::EMERGENCY_STOP), ego, 0.0,
                    12, false);
        const auto* trajectory = harness.LastTrajectory();
        bool any_motion = false;
        if (trajectory) {
            for (const auto& point : trajectory->points) {
                if (point.velocity > 1e-6) any_motion = true;
            }
        }
        // 저속 fallback으로 새면 크롤링 속도로 전진한다.
        Expect(trajectory != nullptr && !any_motion,
               "EMERGENCY_STOP at standstill never creeps forward");
    }

    std::printf("\n=== FOLLOW ===\n");
    {
        // FSM이 지정한 대상만 추종한다. 더 가까운 차량이 있어도 바꾸지
        // 않는다(계약 D1 0.1절 B 분류).
        const uint32_t id = ++command_id;
        Command follow = harness.MakeCommand(id, Command::FOLLOW);
        follow.primary_object_id = 15;
        harness.SetObjects({harness.MakeVehicle(15, 230.0, 6.0)});
        harness.Run(follow, ego, kCruiseSpeed, 16, false);
        harness.PrintSummary("FOLLOW");
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) &&
                   feedback.active_maneuver == Command::FOLLOW,
               "FOLLOW reports active_maneuver=FOLLOW");
        Expect(harness.LastFor(id, feedback) && feedback.plan_valid &&
                   feedback.execution_state == Feedback::EXECUTING,
               "FOLLOW plans a trajectory for the designated target");
        Expect(harness.LastFor(id, feedback) && feedback.active_leader_id == 15,
               "FOLLOW reports the designated leader in active_leader_id");
    }

    std::printf("\n=== FOLLOW keeps its target ===\n");
    {
        // 지정 대상(15)보다 가까운 차량(17)이 있어도 leader는 15다.
        const uint32_t id = ++command_id;
        Command follow = harness.MakeCommand(id, Command::FOLLOW);
        follow.primary_object_id = 15;
        harness.SetObjects({harness.MakeVehicle(15, 245.0, 6.0),
                            harness.MakeVehicle(17, 260.0, 6.0)});
        harness.Run(follow, ego, kCruiseSpeed, 16, false);
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) && feedback.active_leader_id == 15,
               "a nearer vehicle does not become the leader");
    }

    std::printf("\n=== FOLLOW target lost ===\n");
    {
        // 지정 대상이 관측되지 않으면 다른 차량으로 대체하지 않는다.
        // KEEP으로 자동 전환하지도 않는다 - FSM이 결정한다.
        const uint32_t id = ++command_id;
        Command follow = harness.MakeCommand(id, Command::FOLLOW);
        follow.primary_object_id = 99;  // 존재하지 않는 ID
        harness.SetObjects({harness.MakeVehicle(15, 230.0, 6.0)});
        harness.Run(follow, ego, kCruiseSpeed, 14, false);
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) &&
                   feedback.result_code == Feedback::TARGET_OBJECT_LOST,
               "a missing FOLLOW target is TARGET_OBJECT_LOST");
        Expect(harness.LastFor(id, feedback) &&
                   feedback.active_maneuver == Command::FOLLOW,
               "a lost target still reports FOLLOW (no automatic KEEP)");
        Expect(harness.LastFor(id, feedback) &&
                   feedback.active_leader_id == -1,
               "a lost target reports no active leader");
        Expect(harness.LastFor(id, feedback) &&
                   feedback.execution_state == Feedback::FAILED,
               "a lost target latches FAILED");
    }

    std::printf("\n=== FOLLOW recovery with a new command ===\n");
    {
        // 대상 소실 후 복귀는 새 command_id로만 이루어진다.
        const uint32_t id = ++command_id;
        Command follow = harness.MakeCommand(id, Command::FOLLOW);
        follow.primary_object_id = 15;
        harness.SetObjects({harness.MakeVehicle(15, 230.0, 6.0)});
        harness.Run(follow, ego, kCruiseSpeed, 16, false);
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) && feedback.plan_valid &&
                   feedback.active_leader_id == 15,
               "a new command_id recovers FOLLOW after a lost target");
        harness.SetObjects({});
    }

    std::printf("\n=== FOLLOW dropout grace ===\n");
    {
        // dropout_grace_cycles=5. 1~5 사이클은 유예, 6번째에 LOST.
        //
        // 노드가 20 Hz timer로 planning하므로 harness의 사이클 수와 노드의
        // 사이클 수가 정확히 1:1이 아니다. 그래서 "정확히 6번째"를 세는 대신
        // 짧은 dropout은 유지되고 긴 dropout은 LOST가 되는 것을 본다.
        const uint32_t id = ++command_id;
        Command follow = harness.MakeCommand(id, Command::FOLLOW);
        follow.primary_object_id = 15;

        // 1. 정상 확보.
        harness.SetObjects({harness.MakeVehicle(15, 240.0, 6.0)});
        harness.BeginWindow();
        harness.RunCycles(follow, ego, kCruiseSpeed, 14);
        harness.Settle(ego, kCruiseSpeed);
        Feedback acquired;
        Expect(harness.LastFor(id, acquired) && acquired.plan_valid &&
                   acquired.active_leader_id == 15 &&
                   acquired.execution_state == Feedback::EXECUTING,
               "grace: FOLLOW is acquired and EXECUTING");

        // 2. 짧은 dropout(유예 이내). 객체만 끊고 명령은 계속 보낸다.
        harness.SetObjects({});
        harness.BeginWindow();
        harness.RunCycles(follow, ego, kCruiseSpeed, 3);
        harness.Settle(ego, kCruiseSpeed, 1);
        Feedback in_grace;
        const bool got_grace = harness.LastFor(id, in_grace);
        Expect(got_grace && in_grace.execution_state == Feedback::EXECUTING,
               "grace: a short dropout stays EXECUTING");
        Expect(got_grace && in_grace.active_leader_id == 15,
               "grace: a short dropout keeps active_leader_id");
        Expect(got_grace && in_grace.result_code == Feedback::RESULT_NONE,
               "grace: a short dropout reports no error");

        // 3. 재관측하면 정상으로 돌아온다.
        harness.SetObjects({harness.MakeVehicle(15, 240.0, 6.0)});
        harness.BeginWindow();
        harness.RunCycles(follow, ego, kCruiseSpeed, 10);
        harness.Settle(ego, kCruiseSpeed);
        Feedback recovered;
        Expect(harness.LastFor(id, recovered) && recovered.plan_valid &&
                   recovered.active_leader_id == 15,
               "grace: re-observation resumes planning");
    }

    std::printf("\n=== FOLLOW target lost latch ===\n");
    {
        // 유예를 초과하면 FAILED로 latch된다. 같은 command_id로는 대상이
        // 다시 보여도 FAILED가 유지되고, 새 command_id로만 복귀한다.
        const uint32_t lost_id = ++command_id;
        Command follow = harness.MakeCommand(lost_id, Command::FOLLOW);
        follow.primary_object_id = 15;

        harness.SetObjects({harness.MakeVehicle(15, 240.0, 6.0)});
        harness.BeginWindow();
        harness.RunCycles(follow, ego, kCruiseSpeed, 14);
        harness.Settle(ego, kCruiseSpeed);
        Feedback acquired;
        Expect(harness.LastFor(lost_id, acquired) &&
                   acquired.active_leader_id == 15,
               "latch: leader acquired before the loss");

        // 유예를 충분히 넘기는 긴 dropout.
        harness.SetObjects({});
        harness.BeginWindow();
        harness.RunCycles(follow, ego, kCruiseSpeed, 20);
        harness.Settle(ego, kCruiseSpeed);
        Feedback lost;
        const bool got_lost = harness.LastFor(lost_id, lost);
        Expect(got_lost && lost.result_code == Feedback::TARGET_OBJECT_LOST,
               "latch: a long dropout is TARGET_OBJECT_LOST");
        Expect(got_lost && lost.execution_state == Feedback::FAILED,
               "latch: a lost target reaches FAILED");
        Expect(got_lost && lost.active_leader_id == -1,
               "latch: a lost target reports no active leader");
        // KEEP으로 자동 전환하지 않는다 - 다음 행동은 FSM이 정한다.
        Expect(got_lost && lost.active_maneuver == Command::FOLLOW,
               "latch: a lost target still reports FOLLOW (no automatic KEEP)");
        Expect(got_lost && lost.command_accepted,
               "latch: the command itself stays accepted");

        // **핵심**: 대상이 다시 보여도 같은 command_id면 FAILED가 유지된다.
        // FSM이 종료 피드백을 놓치지 않도록 새 command_id까지 latch한다
        // (계약 6절).
        harness.SetObjects({harness.MakeVehicle(15, 240.0, 6.0)});
        harness.BeginWindow();
        harness.RunCycles(follow, ego, kCruiseSpeed, 12);
        harness.Settle(ego, kCruiseSpeed);
        Feedback still_failed;
        const bool got_still = harness.LastFor(lost_id, still_failed);
        Expect(got_still && still_failed.execution_state == Feedback::FAILED,
               "latch: the same command_id stays FAILED after the target returns");
        Expect(got_still &&
                   still_failed.result_code == Feedback::TARGET_OBJECT_LOST,
               "latch: the loss reason is preserved across cycles");
        Expect(got_still && !still_failed.plan_valid,
               "latch: no planning happens while latched");

        // 새 command_id로만 복귀한다.
        const uint32_t new_id = ++command_id;
        Command retry = harness.MakeCommand(new_id, Command::FOLLOW);
        retry.primary_object_id = 15;
        harness.BeginWindow();
        harness.RunCycles(retry, ego, kCruiseSpeed, 14);
        harness.Settle(ego, kCruiseSpeed);
        Feedback retried;
        Expect(harness.LastFor(new_id, retried) &&
                   retried.execution_state == Feedback::EXECUTING &&
                   retried.plan_valid && retried.active_leader_id == 15,
               "latch: a new command_id recovers FOLLOW");
    }

    std::printf("\n=== FOLLOW with a stopped leader ===\n");
    {
        // min_leader_speed를 v2 validity에서 제거했으므로 정지한 앞차도
        // FOLLOW 대상이다(D5-2 정책). 실제 도로에서 앞차가 신호에 멈추면
        // 그 뒤에 같이 멈추는 것이 FOLLOW다.
        const uint32_t id = ++command_id;
        Command follow = harness.MakeCommand(id, Command::FOLLOW);
        follow.primary_object_id = 15;
        harness.SetObjects({harness.MakeVehicle(15, 250.0, 0.0)});
        harness.BeginWindow();
        harness.RunCycles(follow, ego, kCruiseSpeed, 16);
        harness.Settle(ego, kCruiseSpeed);
        Feedback feedback;
        const bool got = harness.LastFor(id, feedback);
        Expect(got && feedback.result_code != Feedback::TARGET_OBJECT_LOST,
               "stopped leader: not reported as a lost target");
        Expect(got && feedback.active_leader_id == 15,
               "stopped leader: stays the active leader");
        // 대상으로 인정하는 것과 실제로 궤적을 만드는 것은 다르다.
        // GenerateFollowingCandidates가 정지 leader에 대해 min_gap을 유지하는
        // 종방향 계획을 실제로 하는지 확인한다.
        Expect(got && feedback.execution_state == Feedback::EXECUTING &&
                   feedback.plan_valid,
               "stopped leader: a FOLLOW trajectory is actually planned");
        Expect(harness.TrajectoryCountInWindow() > 0,
               "stopped leader: a trajectory is published in this window");
    }

    std::printf("\n=== FOLLOW custom gap ===\n");
    {
        // custom gap이 실제 궤적에 반영되는지. 더 큰 gap을 요구하면 더
        // 보수적인 궤적이 나와야 한다(계약 11.8절).
        const uint32_t default_id = ++command_id;
        Command default_gap = harness.MakeCommand(default_id, Command::FOLLOW);
        default_gap.primary_object_id = 15;
        harness.SetObjects({harness.MakeVehicle(15, 235.0, 5.0)});
        harness.BeginWindow();
        harness.RunCycles(default_gap, ego, kCruiseSpeed, 16);
        harness.Settle(ego, kCruiseSpeed);
        // 이번 명령이 실제로 만든 궤적만 본다. LastTrajectory()를 쓰면
        // planning 실패 시 직전 명령의 궤적을 그대로 돌려받아 비교가
        // 무의미해진다.
        const asmc_msgs::Trajectory* default_trajectory =
            harness.LastTrajectoryInWindow();
        const std::size_t default_count = harness.TrajectoryCountInWindow();
        double default_end_speed = -1.0;
        if (default_trajectory && !default_trajectory->points.empty()) {
            default_end_speed = default_trajectory->points.back().velocity;
        }

        const uint32_t custom_id = ++command_id;
        Command custom_gap = harness.MakeCommand(custom_id, Command::FOLLOW);
        custom_gap.primary_object_id = 15;
        custom_gap.use_custom_follow_gap = true;
        custom_gap.desired_time_gap_s = 3.0;   // config 기본값 1.0보다 크게
        custom_gap.minimum_gap_m = 25.0;       // config 기본값 5.0보다 크게
        harness.BeginWindow();
        harness.RunCycles(custom_gap, ego, kCruiseSpeed, 16);
        harness.Settle(ego, kCruiseSpeed);
        const asmc_msgs::Trajectory* custom_trajectory =
            harness.LastTrajectoryInWindow();
        const std::size_t custom_count = harness.TrajectoryCountInWindow();
        double custom_end_speed = -1.0;
        if (custom_trajectory && !custom_trajectory->points.empty()) {
            custom_end_speed = custom_trajectory->points.back().velocity;
        }

        Feedback feedback;
        Expect(harness.LastFor(custom_id, feedback) &&
                   feedback.command_accepted,
               "custom gap: the command is accepted");
        // 두 명령이 각자 새 궤적을 만들었는지 먼저 확인한다. 이 검사가 없으면
        // custom이 planning에 실패해도 값이 같아서 아래 비교가 통과한다.
        Expect(default_count > 0 && custom_count > 0,
               "custom gap: each variant produces its own trajectory");
        Expect(default_end_speed >= 0.0 && custom_end_speed >= 0.0,
               "custom gap: both variants produce trajectories");
        // 25 m 최소 간격을 요구하면 현재 gap(약 35 m)에서 훨씬 조심스럽게
        // 접근한다. 기본값(5 m)보다 **확실히** 낮아야 한다 - 같은 값이면
        // custom gap이 소비되지 않았다는 뜻이다.
        Expect(custom_end_speed < default_end_speed - 1e-3,
               "custom gap: a larger gap yields a strictly slower trajectory");

        harness.SetObjects({});
    }

    std::printf("\n=== AVOID basic handshake lifecycle ===\n");
    {
        constexpr double kAvoidEgoS = 200.0;
        constexpr double kAvoidObstacleS = 225.0;
        const Pose2D avoid_ego = PoseAt(waypoints, kAvoidEgoS);
        harness.SetObjects(
            {harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});

        const uint32_t id = ++command_id;
        Command avoid = harness.MakeCommand(id, Command::AVOID);
        avoid.execution_request = Command::PREPARE;
        avoid.direction = Command::DIRECTION_LEFT;
        avoid.primary_object_id = 21;

        Feedback ready;
        const bool got_ready = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::READY, 30, ready);
        const asmc_msgs::Trajectory* maintenance_ptr =
            harness.LastTrajectoryInWindow();
        const bool has_maintenance = maintenance_ptr != nullptr;
        const double maintenance_d =
            TrajectoryEndpointD(waypoints, maintenance_ptr);
        const double locked_target_d = ready.target_d;
        Expect(got_ready && ready.command_accepted &&
                   ready.active_maneuver == Command::AVOID &&
                   ready.execution_state == Feedback::READY &&
                   ready.plan_valid &&
                   ready.trajectory_source == Feedback::TRAJECTORY_NEW &&
                   ready.result_code == Feedback::RESULT_NONE,
               "AVOID PREPARE reaches READY with maintenance NEW");
        Expect(got_ready && ready.target_d_valid &&
                   std::fabs(locked_target_d) > 1.0 &&
                   ready.generated_candidate_count > 0 &&
                   ready.valid_candidate_count > 0 &&
                   ready.selected_cost >= 0.0 &&
                   ready.blocking_object_id == -1,
               "AVOID PREPARE reports a valid lateral candidate");
        Expect(has_maintenance && std::isfinite(maintenance_d) &&
                   std::fabs(maintenance_d) < 0.75 &&
                   std::fabs(locked_target_d - maintenance_d) > 1.5,
               "AVOID PREPARE publishes center maintenance, not its candidate");

        avoid.execution_request = Command::EXECUTE;
        Feedback committed;
        const bool got_committed = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::COMMITTED, 30,
            committed);
        const asmc_msgs::Trajectory* committed_ptr =
            harness.LastTrajectoryInWindow();
        const bool has_committed = committed_ptr != nullptr;
        const double committed_d =
            TrajectoryEndpointD(waypoints, committed_ptr);
        Expect(got_committed && committed.command_accepted &&
                   committed.active_maneuver == Command::AVOID &&
                   committed.execution_state == Feedback::COMMITTED,
               "same-ID AVOID EXECUTE revalidates and reaches COMMITTED");
        Expect(got_committed &&
                   committed.trajectory_source == Feedback::TRAJECTORY_LOCKED &&
                   committed.plan_valid && committed.target_d_valid,
               "AVOID COMMITTED reports a valid LOCKED trajectory");
        Expect(got_committed &&
                   std::fabs(committed.target_d - locked_target_d) < 1e-9,
               "AVOID COMMITTED locks the PREPARE target_d");
        Expect(has_committed && std::isfinite(committed_d) &&
                   std::fabs(committed_d - locked_target_d) < 0.75 &&
                   std::fabs(committed_d - maintenance_d) > 1.5,
               "first COMMIT publishes the lateral locked trajectory");

        Feedback executing;
        const bool got_executing = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::EXECUTING, 30,
            executing);
        const asmc_msgs::Trajectory* executing_ptr =
            harness.LastTrajectoryInWindow();
        const bool has_executing = executing_ptr != nullptr &&
            !executing_ptr->points.empty();
        const double executing_d =
            TrajectoryEndpointD(waypoints, executing_ptr);
        const double executing_speed = has_executing
            ? executing_ptr->points.back().velocity : -1.0;
        Expect(got_executing && executing.execution_state == Feedback::EXECUTING &&
                   executing.trajectory_source == Feedback::TRAJECTORY_LOCKED &&
                   executing.plan_valid && executing.target_d_valid &&
                   std::fabs(executing.target_d - locked_target_d) < 1e-9 &&
                   executing.maneuver_progress >= 0.0 &&
                   executing.maneuver_progress < 1.0,
               "COMMITTED next cycle continues as EXECUTING LOCKED");
        Expect(has_executing && std::isfinite(executing_d) &&
                   std::fabs(executing_d - locked_target_d) < 0.75 &&
                   std::fabs(executing_d - maintenance_d) > 1.5,
               "EXECUTING does not fall back to PREPARE maintenance");
        Expect(executing.applied_speed_mps >= 0.0 &&
                   std::fabs(executing.applied_speed_mps - executing_speed) <
                       1e-9 &&
                   executing.frenet_state_valid &&
                   std::isfinite(executing.current_s) &&
                   std::isfinite(executing.current_d),
               "EXECUTING LOCKED reports actual speed and Frenet state");

        const Pose2D pass_ego =
            PoseAtOffset(waypoints, kAvoidEgoS, locked_target_d);
        Feedback pass;
        // PASS 진입 프레임을 기다린다. 첫 EXECUTING 프레임은 아직 SHIFT일 수
        // 있고, 그것은 계약 위반이 아니라 관측 위상 문제다(RunUntilProgress).
        const bool got_pass = harness.RunUntilProgress(
            avoid, pass_ego, kCruiseSpeed, Feedback::EXECUTING, 0.5, 30, pass);
        Expect(got_pass && pass.maneuver_progress >= 0.5 &&
                   pass.maneuver_progress < 1.0 && pass.target_d_valid &&
                   std::fabs(pass.target_d - locked_target_d) < 1e-9,
               "controlled lateral pose advances AVOID SHIFT to PASS only");

        // RETURN 단계는 측정된 ego_d가 실제로 avoid_cfg_.lateral_tolerance
        // 이내로 돌아와야 completed=true가 된다(AvoidLifecycle.cpp 주석:
        // "장애물을 완전히 지난 뒤에도 바로 COMPLETED하지 않는다. RETURN에서
        // d=0 복귀가 끝나야..."). ego를 locked_target_d에 그대로 두면
        // |ego_d|가 lateral_tolerance(0.4)를 영원히 넘어 RunUntilState가
        // 항상 timeout한다 - d=0 근처로 옮겨 실제 복귀를 시뮬레이션한다.
        const Pose2D clear_ego =
            PoseAtOffset(waypoints, kAvoidObstacleS + 20.0, 0.0);
        Feedback completed;
        const bool got_completed = harness.RunUntilState(
            avoid, clear_ego, kCruiseSpeed, Feedback::COMPLETED, 30,
            completed);
        const asmc_msgs::Trajectory* final_ptr =
            harness.LastTrajectoryInWindow();
        const bool has_final = final_ptr != nullptr;
        const asmc_msgs::Trajectory final_trajectory = has_final
            ? *final_ptr : asmc_msgs::Trajectory{};
        Expect(got_completed && completed.execution_state == Feedback::COMPLETED &&
                   completed.trajectory_source == Feedback::TRAJECTORY_LOCKED &&
                   completed.plan_valid && completed.maneuver_progress == 1.0 &&
                   completed.target_d_valid &&
                   std::fabs(completed.target_d - locked_target_d) < 1e-9,
               "PASS plus longitudinal clearance publishes final COMPLETED AVOID");
        Expect(has_final && !final_trajectory.points.empty(),
               "first COMPLETED cycle still publishes a final trajectory");

        Feedback latched;
        const bool got_latched = harness.RunUntilState(
            avoid, clear_ego, kCruiseSpeed, Feedback::COMPLETED, 20, latched);
        const asmc_msgs::Trajectory* latch_ptr =
            harness.LastTrajectoryInWindow();
        Expect(got_latched && latched.execution_state == Feedback::COMPLETED &&
                   latched.trajectory_source == Feedback::TRAJECTORY_LOCKED &&
                   !latched.plan_valid && latched.maneuver_progress == 1.0 &&
                   latched.target_d_valid &&
                   std::fabs(latched.target_d - locked_target_d) < 1e-9 &&
                   !latched.frenet_state_valid,
               "same-ID COMPLETED latch preserves diagnostics without fake state");
        Expect(latch_ptr && SameTrajectoryGeometry(final_trajectory, *latch_ptr),
               "same-ID COMPLETED latch republishes the cached final trajectory");
        // Commit E / Test 11.3: latch은 재발행일 뿐 content가 바뀌지 않았다.
        // header.stamp(source/reference time)도 원래 생성 시점 그대로여야
        // 한다 - 재발행 수신 시각으로 덮어쓰면 안 된다(interfaces/README.md
        // 8절, provenance 규칙).
        Expect(latch_ptr && latch_ptr->header.stamp == final_trajectory.header.stamp,
               "same-ID COMPLETED latch keeps the original source timestamp, "
               "not the republish receive time");

        harness.SetObjects({});
        const uint32_t keep_id = ++command_id;
        harness.Run(harness.MakeCommand(keep_id, Command::KEEP), ego,
                    kCruiseSpeed, 12, false);
        Feedback recovered;
        Expect(harness.LastFor(keep_id, recovered) && recovered.plan_valid &&
                   recovered.active_maneuver == Command::KEEP &&
                   recovered.execution_state == Feedback::EXECUTING,
               "new KEEP owns recovery after completed AVOID");
    }

    std::printf("\n=== mode transitions ===\n");
    {
        // KEEP -> STOP. 계약 7.2절이 막는 것은 COMMIT된 handshake maneuver의
        // 중단이며, 일반 모드 사이의 전환은 정상 주행의 일부다.
        const uint32_t keep_id = ++command_id;
        harness.Run(harness.MakeCommand(keep_id, Command::KEEP), ego,
                    kCruiseSpeed, 10, false);
        const uint32_t stop_id = ++command_id;
        Command stop = harness.MakeCommand(stop_id, Command::STOP);
        harness.SetTargetOnPath(stop, 240.0);
        harness.Run(stop, ego, kCruiseSpeed, 12, false);
        Feedback feedback;
        Expect(harness.LastFor(stop_id, feedback) &&
                   feedback.command_id == stop_id &&
                   feedback.active_maneuver == Command::STOP,
               "KEEP -> STOP transition is accepted");

        // STOP -> E-STOP preemption.
        const uint32_t estop_id = ++command_id;
        harness.Run(harness.MakeCommand(estop_id, Command::EMERGENCY_STOP), ego,
                    kCruiseSpeed, 12, false);
        Expect(harness.LastFor(estop_id, feedback) &&
                   feedback.active_maneuver == Command::EMERGENCY_STOP,
               "STOP -> EMERGENCY_STOP preemption is accepted");

        // E-STOP -> KEEP 복귀.
        const uint32_t back_id = ++command_id;
        harness.Run(harness.MakeCommand(back_id, Command::KEEP), ego,
                    kCruiseSpeed, 12, false);
        Expect(harness.LastFor(back_id, feedback) &&
                   feedback.active_maneuver == Command::KEEP &&
                   feedback.plan_valid,
               "EMERGENCY_STOP -> KEEP recovery is accepted");
    }

    std::printf("\n=== invalid commands ===\n");
    {
        // command_id=0은 "처리 중인 명령 없음"을 뜻하는 값이라 명령 ID로
        // 쓸 수 없다.
        Command zero_id = harness.MakeCommand(0, Command::KEEP);
        harness.Run(zero_id, ego, kCruiseSpeed, 10, false);
        Feedback feedback;
        Expect(harness.LastFor(0, feedback) &&
                   !feedback.command_accepted &&
                   feedback.result_code == Feedback::INVALID_COMMAND,
               "command_id=0 is INVALID_COMMAND");

        const uint32_t id = ++command_id;
        Command bad_frame = harness.MakeCommand(id, Command::KEEP);
        bad_frame.header.frame_id = "odom";
        harness.Run(bad_frame, ego, kCruiseSpeed, 10, false);
        Expect(harness.LastFor(id, feedback) &&
                   feedback.result_code == Feedback::INVALID_COMMAND,
               "non-map frame_id is INVALID_COMMAND");

        // STOP은 목표 pose가 필수다.
        const uint32_t no_target_id = ++command_id;
        Command no_target = harness.MakeCommand(no_target_id, Command::STOP);
        harness.Run(no_target, ego, kCruiseSpeed, 10, false);
        Expect(harness.LastFor(no_target_id, feedback) &&
                   feedback.result_code == Feedback::INVALID_COMMAND,
               "STOP without a target pose is INVALID_COMMAND");
    }

    std::printf("\n=== stale command ===\n");
    {
        const uint32_t id = ++command_id;
        harness.RunExpired(harness.MakeCommand(id, Command::KEEP), ego,
                           kCruiseSpeed);
        Feedback feedback;
        // 형식은 정상이고 전달이 늦은 것이므로 INVALID과 구분되어야 한다.
        Expect(harness.LastFor(id, feedback) &&
                   feedback.result_code == Feedback::STALE_COMMAND,
               "expired valid_until is STALE_COMMAND (not INVALID)");
    }

    std::printf("\n=== input stale -> same-command recovery ===\n");
    {
        // 이것은 STALE_COMMAND와 완전히 다른 축이다.
        //   STALE_COMMAND    명령 자체가 만료됨 (FSM이 재발행해야 함)
        //   입력 stale        명령은 정상, Ego/Object 입력만 오래됨
        //
        // 후자에서 transaction을 버리면 입력이 회복될 때 FSM이 새 command_id를
        // 보내야 한다. 계약 11.3절은 같은 transaction으로 계속하도록 한다.
        // 반드시 **같은 command_id**로 회복되어야 검증 의미가 있다.
        const uint32_t id = ++command_id;
        const Command keep = harness.MakeCommand(id, Command::KEEP);

        // 1. 정상 주행 확인.
        harness.Run(keep, ego, kCruiseSpeed, 14, false);
        Feedback before;
        Expect(harness.LastFor(id, before) && before.plan_valid &&
                   before.execution_state == Feedback::EXECUTING,
               "input stale: baseline KEEP is planning");

        // 2. 명령은 계속 보내면서 입력 발행만 멈춘다. vehicle timeout이
        //    0.20 s이므로 넉넉히 넘긴다.
        harness.RunWithoutInputs(keep, 20);
        Feedback stale;
        const bool got_stale = harness.LastFor(id, stale);
        Expect(got_stale && stale.command_id == id,
               "input stale: command_id is preserved");
        Expect(got_stale && stale.command_accepted,
               "input stale: the command stays accepted");
        Expect(got_stale && stale.active_maneuver == Command::KEEP,
               "input stale: active_maneuver stays KEEP");
        Expect(got_stale && !stale.plan_valid &&
                   stale.trajectory_source == Feedback::TRAJECTORY_SAFE_STOP,
               "input stale: planning is skipped with a safe stop");
        // 명령은 정상이므로 명령 결함으로 보고하면 안 된다. INPUT_DATA_STALE이
        // 원본 msg에 없어 result_code는 RESULT_NONE으로 남고 사유는 detail에만
        // 담긴다 - 기존 코드로 거짓 매핑하지 않는다(계약 11.1절).
        Expect(got_stale &&
                   stale.result_code != Feedback::INVALID_COMMAND &&
                   stale.result_code != Feedback::STALE_COMMAND,
               "input stale: not reported as a command defect");
        Expect(got_stale && !stale.result_detail.empty(),
               "input stale: the reason is carried in result_detail");

        // 3. 입력을 되살린다. 새 command_id를 쓰지 않는 것이 핵심이다.
        harness.Run(keep, ego, kCruiseSpeed, 14, false);
        Feedback recovered;
        Expect(harness.LastFor(id, recovered) && recovered.command_id == id &&
                   recovered.plan_valid &&
                   recovered.execution_state == Feedback::EXECUTING &&
                   recovered.trajectory_source == Feedback::TRAJECTORY_NEW,
               "input stale: the same command_id resumes planning after recovery");
        Expect(harness.LastFor(id, recovered) &&
                   recovered.result_code == Feedback::RESULT_NONE,
               "input stale: the error clears after recovery");
    }

    std::printf("\n=== unimplemented maneuver ===\n");
    {
        // R9-B: LANE_CHANGE는 D6-4a 이후 gate가 열려(SetImplementedManeuver
        // 참고) 더 이상 unimplemented 예시가 아니다 - 이 절이 실제로는 정상
        // LANE_CHANGE PREPARE 경로를 타면서 result_code/detail이 채워져
        // "does not fake a result_code" 단언만 실패하고 있었다(R9-A 감사에서
        // 확인). 여전히 닫혀 있는 MERGE로 교체한다 - MERGE는
        // SetImplementedManeuver로 열린 적이 없고 PrepareMerge/
        // PlanCommittedMerge 진입점 자체가 없다(R9-A Generator/Maneuver 감사
        // 참고). validation을 먼저 통과해야 node gate의
        // MANEUVER_NOT_IMPLEMENTED 경로를 실제로 검증할 수 있으므로
        // merge_type을 채운다(behavior_command_adapter의 "MERGE requires a
        // merge_type" validation).
        const uint32_t id = ++command_id;
        Command merge = harness.MakeCommand(id, Command::MERGE);
        merge.merge_type = Command::MERGE_ROUNDABOUT;
        harness.Run(merge, ego, kCruiseSpeed, 12, false);
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) &&
                   feedback.command_accepted &&
                   feedback.execution_state == Feedback::FAILED &&
                   !feedback.plan_valid &&
                   feedback.trajectory_source == Feedback::TRAJECTORY_SAFE_STOP,
               "unimplemented maneuver is accepted but not planned");
        Expect(harness.LastFor(id, feedback) &&
                   feedback.result_code == Feedback::RESULT_NONE &&
                   !feedback.result_detail.empty(),
               "unimplemented maneuver does not fake a result_code");

        // 새 command_id로 정상 복귀.
        const uint32_t keep_id = ++command_id;
        harness.Run(harness.MakeCommand(keep_id, Command::KEEP), ego,
                    kCruiseSpeed, 12, false);
        Expect(harness.LastFor(keep_id, feedback) && feedback.plan_valid,
               "new command_id recovers after an unimplemented maneuver");
    }

    // ================================================================
    // D6-4b: AVOID ROS edge regression
    //
    // D6-4a는 PREPARE -> READY -> EXECUTE -> COMMIT -> EXECUTING ->
    // COMPLETED의 정상 경로를 고정했다. 여기서는 그 경로를 벗어나는 edge를
    // 실제 ROS로 고정한다. 아래 fixture 상수는 전부 D6-4b 실측으로 고른
    // 값이며, 임의의 좌표를 하드코딩하지 않았다.
    // ================================================================

    // AVOID edge 공통 fixture. ego는 회전교차로/저속 fallback과 무관한
    // 구간이고, 장애물은 그 25 m 앞 중앙선이다(D6-4a와 같은 배치).
    constexpr double kAvoidEgoS = 200.0;
    constexpr double kAvoidObstacleS = 225.0;
    const Pose2D avoid_ego = PoseAt(waypoints, kAvoidEgoS);

    // AVOID edge 블록 사이의 상태 정리.
    //
    // KEEP으로 정리하면 안 된다 - COMMITTED/EXECUTING인 AVOID는 KEEP을
    // 거부하므로(계약 7.2절, 아래 "rejected KEEP" 검증이 증명한다) 이전
    // transaction이 그대로 남는다. E-STOP만이 preempt할 수 있다(7.3절).
    const auto reset_avoid_state = [&]() {
        harness.SetObjects({});
        harness.BeginWindow();
        harness.RunCycles(harness.MakeCommand(++command_id,
                                              Command::EMERGENCY_STOP),
                          avoid_ego, kCruiseSpeed, 4);
        harness.BeginWindow();
        harness.RunCycles(harness.MakeCommand(++command_id, Command::KEEP),
                          avoid_ego, kCruiseSpeed, 4);
        harness.Settle(avoid_ego, kCruiseSpeed, 2);
    };

    // LEFT AVOID 명령 하나. 대상은 kAvoidObstacleS의 정지 차량이다.
    const auto make_avoid = [&](uint32_t id, uint8_t request) {
        Command avoid = harness.MakeCommand(id, Command::AVOID);
        avoid.execution_request = request;
        avoid.direction = Command::DIRECTION_LEFT;
        avoid.primary_object_id = 21;
        return avoid;
    };

    // PREPARE -> READY -> EXECUTE -> COMMITTED -> EXECUTING까지 진행시킨다.
    // 실패하면 false. locked_target_d에 잠긴 목표를 돌려준다.
    const auto drive_to_executing = [&](uint32_t id, double& locked_target_d) {
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        Command avoid = make_avoid(id, Command::PREPARE);
        Feedback feedback;
        if (!harness.RunUntilState(avoid, avoid_ego, kCruiseSpeed,
                                   Feedback::READY, 40, feedback)) {
            return false;
        }
        locked_target_d = feedback.target_d;
        avoid.execution_request = Command::EXECUTE;
        if (!harness.RunUntilState(avoid, avoid_ego, kCruiseSpeed,
                                   Feedback::COMMITTED, 40, feedback)) {
            return false;
        }
        return harness.RunUntilState(avoid, avoid_ego, kCruiseSpeed,
                                     Feedback::EXECUTING, 40, feedback);
    };

    std::printf("\n=== AVOID direct EXECUTE handshake ===\n");
    {
        // ===== 첫 명령부터 EXECUTE여도 READY를 건너뛰지 않는다 =====
        //
        // 계약 5.5절의 정상 순서는 PREPARING -> READY -> EXECUTE -> 재검증 ->
        // COMMITTED이고, 5.3절의 EXECUTE는 "이미 준비된 후보를 현재 snapshot
        // 으로 다시 검증"하는 단계다. D6-4b 이전에는 새 transaction의 첫
        // 명령이 EXECUTE이면 한 cycle 안에서
        //   PREPARING -> (PrepareAvoid) -> READY -> (Commit) -> COMMITTED
        // 가 끝나 FSM이 READY를 한 번도 관측하지 못했다. 실측 첫 feedback이
        // 곧바로 COMMITTED/LOCKED였다.
        //
        // same-cycle COMMIT을 되돌리는 돌연변이는 A가 잡는다.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t id = ++command_id;
        Command direct = make_avoid(id, Command::EXECUTE);

        Feedback first;
        std::size_t first_count = 0;
        const bool got_first = harness.RunUntilFirstFeedback(
            direct, avoid_ego, kCruiseSpeed, 20, first, first_count);
        const asmc_msgs::Trajectory* first_ptr =
            harness.LastTrajectoryInWindow();
        const double first_d = TrajectoryEndpointD(waypoints, first_ptr);
        // A. 첫 planning 결과는 READY이며 maintenance NEW다.
        Expect(got_first && first.execution_state == Feedback::READY &&
                   first.trajectory_source == Feedback::TRAJECTORY_NEW &&
                   first.plan_valid && first.command_accepted,
               "brand-new direct EXECUTE reports READY with maintenance NEW");
        // 첫 피드백에서 멈췄으므로 이 window에는 그 하나만 있다. 같은 cycle에
        // COMMIT했다면 그 하나가 COMMITTED였을 것이다.
        Expect(got_first && first_count == 1 &&
                   first.execution_state != Feedback::COMMITTED,
               "brand-new direct EXECUTE does not COMMIT in the same cycle");
        Expect(std::isfinite(first_d) && std::fabs(first_d) < 0.75,
               "brand-new direct EXECUTE publishes center maintenance");

        // B. 같은 EXECUTE가 다음 cycle에 현재 snapshot 재검증으로 COMMIT된다.
        Feedback committed;
        const bool got_committed = harness.RunUntilState(
            direct, avoid_ego, kCruiseSpeed, Feedback::COMMITTED, 30,
            committed);
        const asmc_msgs::Trajectory* committed_ptr =
            harness.LastTrajectoryInWindow();
        const double committed_d =
            TrajectoryEndpointD(waypoints, committed_ptr);
        Expect(got_committed &&
                   committed.trajectory_source == Feedback::TRAJECTORY_LOCKED &&
                   committed.plan_valid && committed.target_d_valid,
               "same direct EXECUTE commits on the next cycle");
        Expect(got_committed && std::isfinite(committed_d) &&
                   std::fabs(committed_d - committed.target_d) < 0.75 &&
                   std::fabs(committed_d) > 1.5,
               "direct EXECUTE COMMIT publishes the locked lateral trajectory");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID WAITING then EXECUTE recovery ===\n");
    {
        // C. WAITING에서 EXECUTE를 계속 재발행하는 경우.
        //    후보가 회복된 cycle은 READY까지만 가고, 그 다음 EXECUTE cycle에
        //    재검증을 거쳐 COMMIT되어야 한다.
        //
        // WAITING fixture는 direction conflict를 쓴다 - 아래 별도 블록이
        // 증명하듯 안정적으로 WAITING을 만들고, 명령을 바꾸지 않고 objects만
        // 바꿔 회복시킬 수 있다.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0),
                            harness.MakeStaticObjectAtOffset(
                                37, kAvoidObstacleS + 7.0, 2.55, 2.4, 4.5)});
        const uint32_t id = ++command_id;
        Command execute = make_avoid(id, Command::EXECUTE);

        Feedback waiting;
        const bool got_waiting = harness.RunUntilState(
            execute, avoid_ego, kCruiseSpeed, Feedback::WAITING, 30, waiting);
        Expect(got_waiting && !waiting.plan_valid &&
                   waiting.trajectory_source == Feedback::TRAJECTORY_NEW &&
                   !harness.SawStateFor(id, Feedback::COMMITTED),
               "blocked EXECUTE waits instead of committing");

        // 차단 객체를 제거해 후보를 회복시킨다.
        //
        // 여기서 고정하려는 성질은 "후보가 처음 유효해진 cycle은 COMMIT하지
        // 않는다"이다. objects를 바꾼 직후 몇 프레임은 아직 WAITING일 수
        // 있으므로(발행/처리 위상), 첫 피드백이 아니라 **첫 READY**를 찾아
        // 그것이 COMMITTED가 아니었는지를 본다. 같은 cycle에 COMMIT한다면
        // READY를 관측하기 전에 COMMITTED가 먼저 나온다.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        Feedback recovered;
        const bool got_recovered = harness.RunUntilState(
            execute, avoid_ego, kCruiseSpeed, Feedback::READY, 30, recovered);
        Expect(got_recovered &&
                   recovered.trajectory_source == Feedback::TRAJECTORY_NEW &&
                   recovered.plan_valid,
               "candidate recovery cycle reaches READY only");
        // 회복 cycle이 곧바로 잠그면 READY를 관측하기 전에 COMMITTED가 먼저
        // 나온다. 순서로 판정해야 "READY 다음 cycle의 정상 COMMIT"과
        // 구분된다(SawStateBefore).
        Expect(got_recovered &&
                   harness.SawStateBefore(id, Feedback::READY,
                                          Feedback::COMMITTED),
               "candidate recovery cycle does not COMMIT");

        // 위 window가 READY 직후의 정상 COMMIT까지 이미 담았을 수 있다. 새
        // window를 열기 **전에** 꺼내 둔다 - RunUntilState는 BeginWindow()로
        // 창을 새로 열기 때문이다.
        Feedback committed;
        const bool already_committed =
            harness.LastForState(id, Feedback::COMMITTED, committed);
        const bool got_committed =
            already_committed ||
            harness.RunUntilState(execute, avoid_ego, kCruiseSpeed,
                                  Feedback::COMMITTED, 30, committed);
        Expect(got_committed &&
                   committed.trajectory_source == Feedback::TRAJECTORY_LOCKED &&
                   committed.target_d_valid,
               "the following EXECUTE cycle commits after revalidation");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID target heading threshold ===\n");
    {
        // 계약 3.4절 4항. 기본 임계값 45도이고 비교는 '>'이므로 정확히 45도는
        // 통과한다. 임계값을 바꾸지 않고 경계 동작만 고정한다.
        //
        // 같은 target position을 쓰고 quaternion yaw만 조절한다.
        constexpr double kDeg = M_PI / 180.0;
        const auto heading_trial = [&](double error_deg, const char* name,
                                       bool expect_rejected) {
            harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
            const uint32_t id = ++command_id;
            Command avoid = make_avoid(id, Command::PREPARE);
            harness.SetTargetWithHeadingError(avoid, kAvoidObstacleS + 10.0,
                                              error_deg * kDeg);
            harness.BeginWindow();
            harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 8);
            harness.Settle(avoid_ego, kCruiseSpeed, 2);
            Feedback feedback;
            const bool got = harness.LastFor(id, feedback);
            const bool rejected =
                got && feedback.execution_state == Feedback::FAILED &&
                feedback.result_code == Feedback::TARGET_PROJECTION_FAILED;
            Expect(got && rejected == expect_rejected, name);
            reset_avoid_state();
        };
        heading_trial(44.0, "44 deg heading error is accepted", false);
        heading_trial(45.0, "exactly 45 deg heading error is accepted", false);
        heading_trial(46.0, "46 deg heading error is TARGET_PROJECTION_FAILED",
                      true);
    }

    std::printf("\n=== AVOID target heading wrap at +-pi ===\n");
    {
        // NormalizeAngle 경로를 실제 트랙으로 통과시킨다. 단순 뺄셈으로
        // 바꾸면 358도 차이로 읽혀 TARGET_PROJECTION_FAILED가 된다.
        const double wrap_s = harness.FindPoseWithHeadingNearPi(1.0 * M_PI / 180.0);
        Expect(wrap_s > 0.0,
               "the track provides a reference heading near +-pi");
        if (wrap_s > 0.0) {
            const Pose2D wrap_pose = PoseAt(waypoints, wrap_s);
            // reference가 -179도대이면 목표는 +178도대로 준다. 표현상 358도
            // 차이지만 실제 각거리는 2도다.
            const double target_yaw =
                wrap_pose.yaw > 0.0
                    ? wrap_pose.yaw - 2.0 * M_PI + 2.0 * M_PI / 180.0
                    : wrap_pose.yaw + 2.0 * M_PI - 2.0 * M_PI / 180.0;
            const double raw_difference = std::fabs(target_yaw - wrap_pose.yaw);
            Expect(raw_difference > 6.0,
                   "the wrap fixture really differs by ~2pi before normalizing");

            const Pose2D wrap_ego = PoseAt(waypoints, wrap_s - 30.0);
            harness.SetObjects({harness.MakeVehicle(41, wrap_s - 5.0, 0.0)});
            const uint32_t id = ++command_id;
            Command avoid = make_avoid(id, Command::PREPARE);
            avoid.primary_object_id = 41;
            harness.SetTargetPoseRaw(avoid, wrap_pose, target_yaw);
            harness.BeginWindow();
            harness.RunCycles(avoid, wrap_ego, 6.0, 10);
            harness.Settle(wrap_ego, 6.0, 2);
            Feedback feedback;
            Expect(harness.LastFor(id, feedback) &&
                       feedback.result_code != Feedback::TARGET_PROJECTION_FAILED,
                   "a +-pi wrapped target heading is not rejected");
            reset_avoid_state();
        }
    }

    std::printf("\n=== AVOID authoritative target id ===\n");
    {
        // FSM이 지정한 primary_object_id만 쓴다. 두 객체의 lateral offset을
        // 다르게 두어 목표 d가 실제로 달라지게 만든다 - 같은 폭/중앙선으로
        // 두면 nearest-selector 돌연변이가 통과한다.
        //
        // 실측(D6-4b): d=-0.8 -> 약 +1.73, d=+0.8 -> 약 +3.82.
        const std::vector<asmc_msgs::DetectedObject> both = {
            harness.MakeStaticObjectAtOffset(17, 240.0, -0.8, 2.0, 4.5),
            harness.MakeStaticObjectAtOffset(21, 255.0, 0.8, 2.0, 4.5),
        };

        // 각각 단독으로 두었을 때의 기준 목표 d.
        harness.SetObjects({both[1]});
        const uint32_t only_21_id = ++command_id;
        Command only_21 = make_avoid(only_21_id, Command::PREPARE);
        Feedback reference_21;
        const bool got_21 = harness.RunUntilState(
            only_21, avoid_ego, kCruiseSpeed, Feedback::READY, 30,
            reference_21);
        reset_avoid_state();

        harness.SetObjects({both[0]});
        const uint32_t only_17_id = ++command_id;
        Command only_17 = make_avoid(only_17_id, Command::PREPARE);
        only_17.primary_object_id = 17;
        Feedback reference_17;
        const bool got_17 = harness.RunUntilState(
            only_17, avoid_ego, kCruiseSpeed, Feedback::READY, 30,
            reference_17);
        reset_avoid_state();

        Expect(got_21 && got_17 &&
                   std::fabs(reference_21.target_d - reference_17.target_d) >
                       1.0,
               "the two obstacles really produce different target_d");

        // 둘 다 있을 때 지정 대상 21의 geometry를 써야 한다. 17이 더 가깝다.
        harness.SetObjects(both);
        const uint32_t id = ++command_id;
        Command avoid = make_avoid(id, Command::PREPARE);
        Feedback selected;
        const bool got_selected = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::READY, 30, selected);
        Expect(got_selected && got_21 &&
                   std::fabs(selected.target_d - reference_21.target_d) < 0.2,
               "AVOID uses the FSM designated target, not the nearest one");
        Expect(got_selected && got_17 &&
                   std::fabs(selected.target_d - reference_17.target_d) > 1.0,
               "AVOID does not fall back to the nearer object geometry");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID LEFT / RIGHT / NONE ===\n");
    {
        // target pose 없는 geometry 경로. NONE에서 특정 방향을 무조건
        // 기대하지 않고, 허용된 후보 중 하나를 골랐는지만 본다.
        const auto direction_trial = [&](uint8_t direction, Feedback& out) {
            harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
            const uint32_t id = ++command_id;
            Command avoid = make_avoid(id, Command::PREPARE);
            avoid.direction = direction;
            const bool got = harness.RunUntilState(
                avoid, avoid_ego, kCruiseSpeed, Feedback::READY, 30, out);
            reset_avoid_state();
            return got;
        };

        Feedback left;
        Feedback right;
        Feedback none;
        const bool got_left = direction_trial(Command::DIRECTION_LEFT, left);
        const bool got_right = direction_trial(Command::DIRECTION_RIGHT, right);
        const bool got_none = direction_trial(Command::DIRECTION_NONE, none);

        Expect(got_left && left.target_d_valid && left.target_d > 1.0,
               "AVOID LEFT evaluates only the left target");
        Expect(got_right && right.target_d_valid && right.target_d < -1.0,
               "AVOID RIGHT evaluates only the right target");
        // NONE은 양쪽이 모두 후보이므로 어느 쪽이든 허용한다. 다만 방향이
        // 결정되어 있어야 하고, 후보 수는 한쪽만 볼 때보다 많아야 한다.
        Expect(got_none && none.target_d_valid &&
                   std::fabs(none.target_d) > 1.0,
               "AVOID NONE selects one of the allowed candidates");
        Expect(got_none && got_left && got_right &&
                   none.generated_candidate_count >
                       std::max(left.generated_candidate_count,
                                right.generated_candidate_count),
               "AVOID NONE evaluates both sides");

        // NONE이 고른 방향은 lock 이후 drift하지 않아야 한다.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t id = ++command_id;
        Command avoid = make_avoid(id, Command::PREPARE);
        avoid.direction = Command::DIRECTION_NONE;
        Feedback ready;
        const bool got_ready = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::READY, 30, ready);
        avoid.execution_request = Command::EXECUTE;
        Feedback committed;
        const bool got_committed = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::COMMITTED, 30, committed);
        Feedback executing;
        const bool got_executing = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::EXECUTING, 30, executing);
        Expect(got_ready && got_committed && got_executing &&
                   std::fabs(committed.target_d - executing.target_d) < 1e-9 &&
                   (committed.target_d > 0.0) == (ready.target_d > 0.0),
               "AVOID NONE does not drift direction after the lock");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID optional target pose ===\n");
    {
        // has_target_pose=true이면 투영된 목표 의도가 후보 선택에 반영되어야
        // 한다. target pose 없이 geometry가 정하는 d와 명확히 달라야 한다.
        //
        // 실측(D6-4b): geometry 기본값 +2.55, pose d=+3.8 -> 목표 +3.80.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t geometry_id = ++command_id;
        Command geometry = make_avoid(geometry_id, Command::PREPARE);
        Feedback geometry_feedback;
        const bool got_geometry = harness.RunUntilState(
            geometry, avoid_ego, kCruiseSpeed, Feedback::READY, 30,
            geometry_feedback);
        reset_avoid_state();

        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t pose_id = ++command_id;
        Command with_pose = make_avoid(pose_id, Command::PREPARE);
        harness.SetTargetOffPath(with_pose, kAvoidObstacleS + 10.0, 3.8);
        Feedback pose_feedback;
        const bool got_pose = harness.RunUntilState(
            with_pose, avoid_ego, kCruiseSpeed, Feedback::READY, 30,
            pose_feedback);

        Expect(got_geometry && geometry_feedback.target_d_valid &&
                   geometry_feedback.target_d > 1.0,
               "AVOID without a target pose uses obstacle geometry");
        Expect(got_pose && pose_feedback.target_d_valid &&
                   pose_feedback.target_d > 3.0,
               "AVOID consumes the projected target pose intent");
        // R9-B: selected_target_d는 요청 target_d가 아니라 실제 선택된
        // 후보의 종단 d다(Avoid.cpp 주석: "요청 +2.50 -> 실제 종단 +3.00").
        // lateral_target_tolerance(0.5m) 격자 안에서 자유롭게 벗어날 수
        // 있으므로, 요청 delta(3.8-2.75=1.05)에서 최악의 경우 양쪽으로
        // 각 0.5m씩 줄어들 수 있다(worst-case 최소 delta = 0.05). 0.8은
        // 그 worst-case보다 커서 항상 통과할 수 없는 threshold였다 -
        // 이 tolerance를 명시적으로 반영해 0.0으로 낮춘다(그래도 방향은
        // 확실히 검증한다: 위 두 개별 bound(>1.0, >3.0)가 이미 절대값을
        // 검증하므로 여기서는 순서만 확인하면 된다).
        Expect(got_geometry && got_pose &&
                   pose_feedback.target_d - geometry_feedback.target_d > 0.0,
               "the target pose actually changes the selected target_d");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID direction conflict ===\n");
    {
        // direction=LEFT인데 target pose가 장애물 오른쪽이면, 그 오른쪽 경로가
        // 물리적으로 비어 있어도 candidate를 만들지 않는다(계약 3.3.4절).
        // 명령 결함이 아니므로 INVALID_COMMAND/FAILED가 아니라 WAITING이다.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t id = ++command_id;
        Command avoid = make_avoid(id, Command::PREPARE);
        harness.SetTargetOffPath(avoid, kAvoidObstacleS + 10.0, -2.55);
        harness.BeginWindow();
        harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 10);
        harness.Settle(avoid_ego, kCruiseSpeed, 2);
        const asmc_msgs::Trajectory* maintenance_ptr =
            harness.LastTrajectoryInWindow();
        const double maintenance_d =
            TrajectoryEndpointD(waypoints, maintenance_ptr);
        Feedback feedback;
        const bool got = harness.LastFor(id, feedback);
        Expect(got && feedback.command_accepted &&
                   feedback.execution_state == Feedback::WAITING &&
                   !feedback.plan_valid &&
                   feedback.result_code == Feedback::RESULT_NONE,
               "direction conflict waits instead of failing");
        Expect(got && feedback.trajectory_source == Feedback::TRAJECTORY_NEW &&
                   std::isfinite(maintenance_d) &&
                   std::fabs(maintenance_d) < 0.75,
               "direction conflict keeps publishing center maintenance");
        Expect(got && feedback.result_code != Feedback::INVALID_COMMAND &&
                   feedback.execution_state != Feedback::FAILED,
               "direction conflict is not an INVALID_COMMAND");
        // 후보를 아예 만들지 않았으므로 차단 객체도 없다. 아래 실제
        // collision-blocked WAITING과 구분되는 지점이다.
        Expect(got && feedback.blocking_object_id == -1 &&
                   feedback.generated_candidate_count == 0,
               "direction conflict generates no candidate at all");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID collision-blocked WAITING ===\n");
    {
        // direction conflict와 달리 요청 방향의 후보를 실제로 만들었지만
        // 다른 장애물이 전부 막은 경우다. blocking_object_id가 실제 차단
        // 객체를 가리켜야 한다.
        //
        // blocker의 s/d는 D6-4b probe로 고른 값이다(228~236 m / +2.55 부근이
        // 모두 안정적으로 WAITING을 만들었다).
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0),
                            harness.MakeStaticObjectAtOffset(
                                37, kAvoidObstacleS + 7.0, 2.55, 2.4, 4.5)});
        const uint32_t id = ++command_id;
        Command avoid = make_avoid(id, Command::PREPARE);
        harness.BeginWindow();
        harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 10);
        harness.Settle(avoid_ego, kCruiseSpeed, 2);
        const asmc_msgs::Trajectory* maintenance_ptr =
            harness.LastTrajectoryInWindow();
        const double maintenance_d =
            TrajectoryEndpointD(waypoints, maintenance_ptr);
        Feedback feedback;
        const bool got = harness.LastFor(id, feedback);
        Expect(got && feedback.command_accepted &&
                   feedback.execution_state == Feedback::WAITING &&
                   !feedback.plan_valid &&
                   feedback.result_code == Feedback::RESULT_NONE,
               "collision-blocked AVOID PREPARE waits");
        Expect(got && feedback.trajectory_source == Feedback::TRAJECTORY_NEW &&
                   std::isfinite(maintenance_d) &&
                   std::fabs(maintenance_d) < 0.75,
               "collision-blocked AVOID publishes NEW maintenance");
        Expect(got && feedback.blocking_object_id >= 0 &&
                   feedback.generated_candidate_count > 0 &&
                   feedback.valid_candidate_count == 0,
               "collision-blocked AVOID reports the blocking object");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID pre-COMMIT target dropout ===\n");
    {
        // A. 첫 binding부터 대상이 없으면 유예를 주지 않는다.
        harness.SetObjects({harness.MakeVehicle(17, kAvoidObstacleS, 0.0)});
        const uint32_t missing_id = ++command_id;
        Command missing = make_avoid(missing_id, Command::PREPARE);
        harness.BeginWindow();
        harness.RunCycles(missing, avoid_ego, kCruiseSpeed, 3);
        Feedback first;
        Expect(harness.FirstFor(missing_id, first) &&
                   first.execution_state == Feedback::FAILED &&
                   first.result_code == Feedback::TARGET_OBJECT_LOST,
               "a never-observed AVOID target fails immediately");
        Expect(harness.FirstFor(missing_id, first) && first.command_accepted,
               "TARGET_OBJECT_LOST is not a command defect");
        reset_avoid_state();

        // B. 확보한 뒤의 dropout은 유예를 받고, 유예를 넘기면 LOST다.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t id = ++command_id;
        Command avoid = make_avoid(id, Command::PREPARE);
        Feedback ready;
        const bool got_ready = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::READY, 30, ready);
        Expect(got_ready, "AVOID acquires its target before the dropout");

        // 짧은 dropout: 유예 안이므로 상태가 유지된다.
        harness.SetObjects({});
        harness.BeginWindow();
        harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 2);
        Feedback during;
        Expect(harness.LastFor(id, during) &&
                   during.execution_state != Feedback::FAILED &&
                   during.result_code != Feedback::TARGET_OBJECT_LOST,
               "a short pre-COMMIT dropout keeps the transaction");

        // 대상 복귀: 같은 대상으로 계속한다.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        harness.BeginWindow();
        harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 4);
        Feedback recovered;
        Expect(harness.LastFor(id, recovered) &&
                   recovered.execution_state == Feedback::READY &&
                   recovered.target_d_valid &&
                   std::fabs(recovered.target_d - ready.target_d) < 0.2,
               "the reacquired AVOID target keeps the same geometry");

        // 긴 dropout: 유예를 넘기면 terminal LOST다.
        harness.SetObjects({});
        harness.BeginWindow();
        harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 15);
        harness.Settle(avoid_ego, kCruiseSpeed, 2);
        Feedback lost;
        Expect(harness.LastFor(id, lost) &&
                   lost.execution_state == Feedback::FAILED &&
                   lost.result_code == Feedback::TARGET_OBJECT_LOST,
               "a long pre-COMMIT dropout ends in TARGET_OBJECT_LOST");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID post-COMMIT SHIFT long dropout ===\n");
    {
        // 이미 SHIFT가 시작된 뒤에는 대상 소실만으로 기동을 끊지 않는다.
        // 잠긴 target_d와 context로 장애물 옆 급복귀 없이 완주한다.
        const uint32_t id = ++command_id;
        double locked_target_d = 0.0;
        const bool executing = drive_to_executing(id, locked_target_d);
        Expect(executing, "AVOID reaches EXECUTING before the dropout");

        // 현재 동적 시작거리 설정에서는 25 m target도 COMMIT 직후 잠깐
        // TRACKING일 수 있다. target을 관측한 채 5 m 접근해 SHIFT에 먼저
        // 들어간 뒤 dropout을 만든다.
        const Pose2D shift_ego = PoseAt(waypoints, kAvoidEgoS + 5.0);
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        Command avoid = make_avoid(id, Command::EXECUTE);
        harness.BeginWindow();
        harness.RunCycles(avoid, shift_ego, kCruiseSpeed, 4);

        // 대상을 제거하고 유예보다 훨씬 오래 유지한다. ego는 clearance
        // 이전에 고정해 completion이 먼저 일어나지 않게 한다.
        harness.SetObjects({});
        harness.BeginWindow();
        harness.RunCycles(avoid, shift_ego, kCruiseSpeed, 25);
        const asmc_msgs::Trajectory* locked_ptr =
            harness.LastTrajectoryInWindow();
        const double locked_d = TrajectoryEndpointD(waypoints, locked_ptr);
        Feedback feedback;
        const bool got = harness.LastFor(id, feedback);
        Expect(got && feedback.execution_state == Feedback::EXECUTING &&
                   feedback.result_code != Feedback::TARGET_OBJECT_LOST,
               "post-COMMIT SHIFT dropout does not report TARGET_OBJECT_LOST");
        Expect(got && feedback.execution_state != Feedback::FAILED &&
                   feedback.trajectory_source == Feedback::TRAJECTORY_LOCKED &&
                   feedback.plan_valid,
               "post-COMMIT dropout keeps executing the locked trajectory");
        Expect(got && feedback.target_d_valid &&
                   std::fabs(feedback.target_d - locked_target_d) < 1e-9,
               "post-COMMIT dropout does not unlock target_d");
        Expect(std::isfinite(locked_d) &&
                   std::fabs(locked_d - locked_target_d) < 0.75,
               "post-COMMIT dropout keeps the lateral trajectory");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID post-COMMIT TRACKING long dropout ===\n");
    {
        // 횡이동을 시작하기 전 먼 target이 사라지면, 마지막 obstacle_s를
        // 향해 허공으로 회피하지 않고 유예 뒤 terminal LOST를 올린다.
        constexpr double kFarObstacleS = kAvoidObstacleS + 55.0;
        harness.SetObjects({harness.MakeVehicle(21, kFarObstacleS, 0.0)});
        const uint32_t id = ++command_id;
        Command avoid = make_avoid(id, Command::PREPARE);
        Feedback feedback;
        const bool ready = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::READY, 40, feedback);
        avoid.execution_request = Command::EXECUTE;
        const bool committed = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::COMMITTED, 40, feedback);
        const bool executing = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::EXECUTING, 40, feedback);
        Expect(ready && committed && executing,
               "far AVOID reaches EXECUTING while still approaching");

        harness.SetObjects({});
        harness.BeginWindow();
        harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 15);
        harness.Settle(avoid_ego, kCruiseSpeed, 2);
        Feedback lost;
        Expect(harness.LastFor(id, lost) &&
                   lost.execution_state == Feedback::FAILED &&
                   lost.result_code == Feedback::TARGET_OBJECT_LOST,
               "post-COMMIT TRACKING dropout becomes TARGET_OBJECT_LOST");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID CANCEL before COMMIT ===\n");
    {
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t id = ++command_id;
        Command avoid = make_avoid(id, Command::PREPARE);
        Feedback ready;
        const bool got_ready = harness.RunUntilState(
            avoid, avoid_ego, kCruiseSpeed, Feedback::READY, 30, ready);
        Expect(got_ready, "AVOID reaches READY before the cancel");

        // 한 cycle만 돌면 20 Hz timer와 어긋나 이 window에 피드백이 없을 수
        // 있다. CANCEL은 성공 후에도 IDLE을 유지하므로(아래 검증) 몇 cycle을
        // 돌고 마지막을 보는 편이 안정적이고 더 강한 검증이다.
        Command cancel = make_avoid(id, Command::CANCEL);
        harness.BeginWindow();
        harness.RunCycles(cancel, avoid_ego, kCruiseSpeed, 4);
        Feedback cancelled;
        Expect(harness.LastFor(id, cancelled) &&
                   cancelled.command_accepted &&
                   cancelled.execution_state == Feedback::IDLE &&
                   cancelled.active_maneuver == Feedback::MANEUVER_NONE,
               "CANCEL before COMMIT returns to IDLE");
        Expect(!harness.SawStateFor(id, Feedback::COMMITTED) &&
                   !harness.SawStateFor(id, Feedback::EXECUTING),
               "CANCEL before COMMIT never starts the maneuver");

        // 같은 ID의 CANCEL을 다시 보내도 terminal 실패로 latch되지 않는다.
        // 취소할 transaction이 없는 CANCEL은 새 기동을 열지 않는다.
        harness.BeginWindow();
        harness.RunCycles(cancel, avoid_ego, kCruiseSpeed, 4);
        Feedback repeated;
        Expect(harness.LastFor(id, repeated) &&
                   repeated.execution_state == Feedback::IDLE &&
                   repeated.result_code != Feedback::INVALID_COMMAND,
               "a repeated CANCEL stays IDLE instead of failing");

        // 다음 새 AVOID는 이전 target/missing grace를 상속하지 않는다.
        harness.SetObjects({});
        const uint32_t next_id = ++command_id;
        Command next = make_avoid(next_id, Command::PREPARE);
        harness.BeginWindow();
        harness.RunCycles(next, avoid_ego, kCruiseSpeed, 3);
        Feedback inherited;
        Expect(harness.FirstFor(next_id, inherited) &&
                   inherited.execution_state == Feedback::FAILED &&
                   inherited.result_code == Feedback::TARGET_OBJECT_LOST,
               "a new AVOID after CANCEL does not inherit the dropout grace");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID CANCEL after COMMIT ===\n");
    {
        // 계약 7.1절. COMMITTED/EXECUTING의 CANCEL은 거부하고 잠긴 궤적을
        // 계속 실행한다. ResetExternalAvoidance가 불리면 안 된다.
        const uint32_t id = ++command_id;
        double locked_target_d = 0.0;
        const bool executing = drive_to_executing(id, locked_target_d);
        Expect(executing, "AVOID reaches EXECUTING before the late cancel");

        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        Command cancel = make_avoid(id, Command::CANCEL);
        harness.BeginWindow();
        harness.RunCycles(cancel, avoid_ego, kCruiseSpeed, 6);
        const asmc_msgs::Trajectory* locked_ptr =
            harness.LastTrajectoryInWindow();
        const double locked_d = TrajectoryEndpointD(waypoints, locked_ptr);
        Feedback feedback;
        const bool got = harness.LastFor(id, feedback);
        Expect(got && feedback.command_accepted &&
                   feedback.result_code == Feedback::TOO_LATE_TO_CANCEL,
               "CANCEL after COMMIT is rejected with TOO_LATE_TO_CANCEL");
        Expect(got && feedback.active_maneuver == Command::AVOID &&
                   feedback.execution_state == Feedback::EXECUTING &&
                   feedback.trajectory_source == Feedback::TRAJECTORY_LOCKED,
               "CANCEL after COMMIT keeps the AVOID execution state");
        Expect(got && feedback.target_d_valid &&
                   std::fabs(feedback.target_d - locked_target_d) < 1e-9,
               "CANCEL after COMMIT keeps the target lock");
        Expect(std::isfinite(locked_d) &&
                   std::fabs(locked_d - locked_target_d) < 0.75,
               "CANCEL after COMMIT keeps publishing the AVOID trajectory");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID rejected new command ownership ===\n");
    {
        // 계약 7.2.1절. D6-0a의 manager-only 회귀를 실제 ROS로 완성한다.
        // 가장 중요한 것은 MPC 궤적이 KEEP 중앙 경로로 바뀌지 않는 것이다.
        const uint32_t id = ++command_id;
        double locked_target_d = 0.0;
        const bool executing = drive_to_executing(id, locked_target_d);
        Expect(executing, "AVOID reaches EXECUTING before the new KEEP");

        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t keep_id = ++command_id;
        Command keep = harness.MakeCommand(keep_id, Command::KEEP);
        harness.BeginWindow();
        harness.RunCycles(keep, avoid_ego, kCruiseSpeed, 8);
        const asmc_msgs::Trajectory* locked_ptr =
            harness.LastTrajectoryInWindow();
        const double locked_d = TrajectoryEndpointD(waypoints, locked_ptr);
        Feedback feedback;
        const bool got = harness.LastFor(keep_id, feedback);
        Expect(got && feedback.command_id == keep_id &&
                   feedback.command_accepted &&
                   feedback.result_code == Feedback::TOO_LATE_TO_CANCEL,
               "a new KEEP during AVOID EXECUTING is answered on its own id");
        Expect(got && feedback.active_maneuver == Command::AVOID &&
                   feedback.execution_state == Feedback::EXECUTING &&
                   feedback.trajectory_source == Feedback::TRAJECTORY_LOCKED,
               "the rejected KEEP does not become the active maneuver");
        Expect(got && feedback.target_d_valid &&
                   std::fabs(feedback.target_d - locked_target_d) < 1e-9,
               "the rejected KEEP does not release the AVOID target lock");
        // 이 검증이 핵심이다. 궤적이 중앙으로 돌아오면 장애물 옆에서 조향이
        // 반대로 꺾인다.
        Expect(std::isfinite(locked_d) &&
                   std::fabs(locked_d - locked_target_d) < 0.75 &&
                   std::fabs(locked_d) > 1.5,
               "the MPC trajectory stays on the committed AVOID, not KEEP");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID survives an invalid or stale new command ===\n");
    {
        // validation은 Accept()보다 먼저 수행되므로, 형식이 잘못되었거나
        // 만료된 새 명령은 manager에 도달하지 못한다. 그렇다고 잠긴 회피를
        // 끊으면 계약 7.2절이 막으려던 사고가 그대로 일어난다 - 장애물 옆
        // 차선 사이 정지다(8.2.1절).
        //
        // 거부 회신 자체는 유지하고, 계획 대상만 활성 transaction으로 둔다.
        const uint32_t id = ++command_id;
        double locked_target_d = 0.0;
        const bool executing = drive_to_executing(id, locked_target_d);
        Expect(executing, "AVOID reaches EXECUTING before the bad command");
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});

        // (a) frame_id가 잘못된 새 KEEP.
        const uint32_t invalid_id = ++command_id;
        Command invalid = harness.MakeCommand(invalid_id, Command::KEEP);
        invalid.header.frame_id = "base_link";
        harness.BeginWindow();
        harness.RunCycles(invalid, avoid_ego, kCruiseSpeed, 6);
        const asmc_msgs::Trajectory* invalid_ptr =
            harness.LastTrajectoryInWindow();
        const double invalid_d = TrajectoryEndpointD(waypoints, invalid_ptr);
        Feedback invalid_feedback;
        const bool got_invalid = harness.LastFor(invalid_id, invalid_feedback);
        Expect(got_invalid && !invalid_feedback.command_accepted &&
                   invalid_feedback.result_code == Feedback::INVALID_COMMAND,
               "an invalid new command is still rejected on its own id");
        Expect(got_invalid &&
                   invalid_feedback.active_maneuver == Command::AVOID &&
                   invalid_feedback.execution_state == Feedback::EXECUTING &&
                   invalid_feedback.trajectory_source ==
                       Feedback::TRAJECTORY_LOCKED &&
                   invalid_feedback.target_d_valid &&
                   std::fabs(invalid_feedback.target_d - locked_target_d) < 1e-9,
               "an invalid new command does not interrupt the committed AVOID");
        Expect(std::isfinite(invalid_d) &&
                   std::fabs(invalid_d - locked_target_d) < 0.75,
               "an invalid new command does not force a SAFE_STOP trajectory");

        // (b) valid_until이 지난 새 KEEP.
        const uint32_t stale_id = ++command_id;
        Command stale = harness.MakeCommand(stale_id, Command::KEEP);
        harness.BeginWindow();
        harness.RunExpiredCycles(stale, avoid_ego, kCruiseSpeed, 6);
        const asmc_msgs::Trajectory* stale_ptr =
            harness.LastTrajectoryInWindow();
        const double stale_d = TrajectoryEndpointD(waypoints, stale_ptr);
        Feedback stale_feedback;
        const bool got_stale = harness.LastFor(stale_id, stale_feedback);
        Expect(got_stale && !stale_feedback.command_accepted &&
                   stale_feedback.result_code == Feedback::STALE_COMMAND,
               "an expired new command is still rejected on its own id");
        Expect(got_stale && stale_feedback.active_maneuver == Command::AVOID &&
                   stale_feedback.execution_state == Feedback::EXECUTING &&
                   stale_feedback.trajectory_source ==
                       Feedback::TRAJECTORY_LOCKED &&
                   stale_feedback.target_d_valid &&
                   std::fabs(stale_feedback.target_d - locked_target_d) < 1e-9,
               "an expired new command does not interrupt the committed AVOID");
        Expect(std::isfinite(stale_d) &&
                   std::fabs(stale_d - locked_target_d) < 0.75,
               "an expired new command does not force a SAFE_STOP trajectory");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID E-STOP preemption ===\n");
    {
        // 계약 7.3절. E-STOP만 COMMITTED/EXECUTING을 preempt한다.
        const uint32_t id = ++command_id;
        double locked_target_d = 0.0;
        const bool executing = drive_to_executing(id, locked_target_d);
        Expect(executing, "AVOID reaches EXECUTING before the emergency stop");

        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0)});
        const uint32_t estop_id = ++command_id;
        Command estop = harness.MakeCommand(estop_id, Command::EMERGENCY_STOP);
        harness.BeginWindow();
        harness.RunCycles(estop, avoid_ego, kCruiseSpeed, 8);
        const asmc_msgs::Trajectory* estop_ptr =
            harness.LastTrajectoryInWindow();
        Feedback feedback;
        const bool got = harness.LastFor(estop_id, feedback);
        Expect(got && feedback.command_accepted &&
                   feedback.active_maneuver == Command::EMERGENCY_STOP,
               "E-STOP preempts the committed AVOID");
        Expect(got && !feedback.target_d_valid,
               "E-STOP releases the AVOID target lock");
        Expect(estop_ptr && IsDecelToZero(*estop_ptr),
               "E-STOP publishes a decel-to-zero trajectory");

        // 이후 새 AVOID의 대상이 미관측이면 old grace를 상속하지 않는다.
        harness.SetObjects({});
        const uint32_t next_id = ++command_id;
        Command next = make_avoid(next_id, Command::PREPARE);
        harness.BeginWindow();
        harness.RunCycles(next, avoid_ego, kCruiseSpeed, 3);
        Feedback inherited;
        Expect(harness.FirstFor(next_id, inherited) &&
                   inherited.execution_state == Feedback::FAILED &&
                   inherited.result_code == Feedback::TARGET_OBJECT_LOST,
               "a new AVOID after E-STOP does not inherit the dropout grace");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID post-COMMIT collision SAFE_STOP ===\n");
    {
        // 계약 8.2.1절. committed path가 새 장애물로 막히면 **출력 궤적만**
        // SAFE_STOP이 되고 transaction과 lock은 유지된다. 그래서 잠긴 진단이
        // 사라지면 안 된다 - FSM 입장에서는 "회피 중인데 목표도 진행률도
        // 없고 무엇이 막았는지도 모른다"가 된다.
        //
        // D6-4b 이전에는 이 분기가 avoid_fields 없이 발행해
        // target_d_valid=false / target_d=0 / progress=-1 / blocking=-1이
        // 나갔다.
        const uint32_t id = ++command_id;
        double locked_target_d = 0.0;
        const bool executing = drive_to_executing(id, locked_target_d);
        Expect(executing, "AVOID reaches EXECUTING before the new blocker");

        Feedback before;
        Command avoid = make_avoid(id, Command::EXECUTE);
        harness.BeginWindow();
        harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 2);
        const bool got_before = harness.LastFor(id, before);

        // 잠긴 경로 위에 새 blocker를 놓는다. probe에서 212~222 m / 잠긴 d
        // 부근이 모두 안정적으로 COLLISION_BLOCKED를 만들었다.
        harness.SetObjects({harness.MakeVehicle(21, kAvoidObstacleS, 0.0),
                            harness.MakeStaticObjectAtOffset(
                                31, kAvoidEgoS + 15.0, locked_target_d,
                                2.4, 4.5)});
        harness.BeginWindow();
        harness.RunCycles(avoid, avoid_ego, kCruiseSpeed, 6);
        Feedback blocked;
        const bool got = harness.LastFor(id, blocked);
        Expect(got && blocked.execution_state == Feedback::EXECUTING &&
                   blocked.active_maneuver == Command::AVOID,
               "a blocked committed AVOID stays EXECUTING");
        Expect(got && blocked.trajectory_source == Feedback::TRAJECTORY_SAFE_STOP &&
                   !blocked.plan_valid &&
                   blocked.result_code == Feedback::COLLISION_BLOCKED,
               "a blocked committed AVOID reports COLLISION_BLOCKED SAFE_STOP");
        Expect(got && blocked.target_d_valid &&
                   std::fabs(blocked.target_d - locked_target_d) < 1e-9,
               "a blocked committed AVOID retains the locked target_d");
        Expect(got && got_before && blocked.maneuver_progress >= 0.0 &&
                   std::fabs(blocked.maneuver_progress -
                             before.maneuver_progress) < 1e-9,
               "a blocked committed AVOID retains the last committed progress");
        Expect(got && blocked.blocking_object_id >= 0,
               "a blocked committed AVOID reports the blocking object");
        Expect(got && blocked.applied_speed_mps >= 0.0,
               "a blocked committed AVOID reports the SAFE_STOP speed");
        reset_avoid_state();
    }

    std::printf("\n=== AVOID new transaction after COMPLETED ===\n");
    {
        // 완료 직후 중간 KEEP 없이 바로 새 AVOID를 보낸다. 이전 committed
        // 상태/목표/유예를 전혀 상속하지 않아야 한다(계약 12.7.6절).
        const uint32_t id = ++command_id;
        double locked_target_d = 0.0;
        const bool executing = drive_to_executing(id, locked_target_d);
        Expect(executing, "AVOID reaches EXECUTING before completing");

        Command avoid = make_avoid(id, Command::EXECUTE);
        const Pose2D pass_ego =
            PoseAtOffset(waypoints, kAvoidEgoS, locked_target_d);
        Feedback pass;
        harness.RunUntilProgress(avoid, pass_ego, kCruiseSpeed,
                                 Feedback::EXECUTING, 0.5, 30, pass);
        // RETURN 완료 조건은 측정된 ego_d가 실제로 lateral_tolerance
        // 이내로 돌아와야 한다(위 "AVOID basic handshake lifecycle" 절
        // 주석 참고). locked_target_d에 그대로 두면 항상 timeout한다.
        const Pose2D clear_ego =
            PoseAtOffset(waypoints, kAvoidObstacleS + 20.0, 0.0);
        Feedback completed;
        const bool got_completed = harness.RunUntilState(
            avoid, clear_ego, kCruiseSpeed, Feedback::COMPLETED, 40, completed);
        Expect(got_completed && completed.maneuver_progress == 1.0,
               "AVOID completes before the next transaction");

        // 중간 KEEP 없이 새 AVOID id. 새 대상은 ego 앞 중앙선이다.
        harness.SetObjects({harness.MakeStaticObjectAtOffset(
            41, kAvoidObstacleS + 50.0, 0.0, 2.0, 4.5)});
        const uint32_t next_id = ++command_id;
        Command next = make_avoid(next_id, Command::PREPARE);
        next.primary_object_id = 41;
        Feedback ready;
        const bool got_ready = harness.RunUntilState(
            next, clear_ego, kCruiseSpeed, Feedback::READY, 30, ready);
        Expect(got_ready && ready.command_accepted &&
                   ready.active_maneuver == Command::AVOID &&
                   ready.plan_valid &&
                   ready.trajectory_source == Feedback::TRAJECTORY_NEW,
               "a new AVOID id right after COMPLETED reaches READY");
        Expect(got_ready && ready.maneuver_progress < 0.0,
               "the new AVOID does not inherit the completed progress");
        Expect(got_ready && ready.target_d_valid &&
                   std::fabs(ready.target_d - completed.target_d) > 1e-9,
               "the new AVOID binds its own target instead of the old lock");
        reset_avoid_state();
    }

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    if (g_failures != 0) {
        std::printf("%d v2 ROS regression check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("all v2 ROS regression checks passed\n");
    return 0;
}
