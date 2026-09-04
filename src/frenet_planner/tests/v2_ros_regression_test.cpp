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
        // AVOID는 node gate가 아직 닫혀 있다(D6-4에서 연다). planner를
        // 호출하지 않고 거짓 result_code를 쓰지도 않는다.
        //
        // D5-1에서 FOLLOW가 열렸으므로 이 케이스의 예시를 AVOID로 바꿨다.
        // 모드가 열릴 때마다 여기를 갱신해야 한다 - D6-4에서는 LANE_CHANGE,
        // D7에서는 MERGE가 남는다.
        //
        // ===== D6-1에서 이 케이스가 갖는 추가 의미 =====
        //
        // D6-1은 planner **core**의 whitelist에 AVOID를 추가했다(resolver를
        // 검증하기 위해). 이 케이스가 계속 통과한다는 것은 core가 열려도
        // node gate가 독립적으로 AVOID를 막고 있다는 증거다. 두 게이트를
        // 분리해 둔 이유가 정확히 이것이다 - core는 단계적으로 열면서 ROS
        // 경로는 완성될 때까지 닫아둔다(계약 12.7.1절).
        const uint32_t id = ++command_id;
        Command avoid = harness.MakeCommand(id, Command::AVOID);
        avoid.execution_request = Command::PREPARE;
        avoid.primary_object_id = 7;
        harness.Run(avoid, ego, kCruiseSpeed, 12, false);
        Feedback feedback;
        Expect(harness.LastFor(id, feedback) &&
                   feedback.command_accepted &&
                   feedback.execution_state == Feedback::FAILED &&
                   !feedback.plan_valid,
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

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    if (g_failures != 0) {
        std::printf("%d v2 ROS regression check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("all v2 ROS regression checks passed\n");
    return 0;
}
