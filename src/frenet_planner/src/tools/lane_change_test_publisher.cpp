// LANE_CHANGE FSM wire contract를 재현하는 테스트 tool.
//
// Behavior Planner의 scorer/policy는 여전히 LANE_CHANGE를 자동으로 고르지
// 않으므로, 이 tool이 PREPARE/EXECUTE/CANCEL handshake를 손으로 재현해
// core/node 경로를 확인한다.
//
// 실제 MORAI 주행 검증은 사용자가 직접 수행한다 - 이 tool은 자동으로 반복
// 주행을 만들지 않고, 사용자가 지정한 한 번의 LANE_CHANGE 명령 lifecycle만
// 손으로 진행시킨다(PREPARE 발행 -> READY 관측 -> 사용자 입력으로 EXECUTE ->
// 진행 상황 관측).

#include <cmath>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <string>

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <ros/ros.h>

namespace {

std::mutex g_feedback_mutex;
asmc_msgs::FrenetFeedback g_latest_feedback;
bool g_has_feedback = false;

const char* ExecutionStateName(uint8_t state) {
    switch (state) {
        case asmc_msgs::FrenetFeedback::IDLE: return "IDLE";
        case asmc_msgs::FrenetFeedback::PREPARING: return "PREPARING";
        case asmc_msgs::FrenetFeedback::WAITING: return "WAITING";
        case asmc_msgs::FrenetFeedback::READY: return "READY";
        case asmc_msgs::FrenetFeedback::COMMITTED: return "COMMITTED";
        case asmc_msgs::FrenetFeedback::EXECUTING: return "EXECUTING";
        case asmc_msgs::FrenetFeedback::COMPLETED: return "COMPLETED";
        case asmc_msgs::FrenetFeedback::FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

void FeedbackCallback(const asmc_msgs::FrenetFeedback::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(g_feedback_mutex);
    g_latest_feedback = *message;
    g_has_feedback = true;
    std::printf(
        "[feedback] command_id=%u state=%s plan_valid=%d target_d_valid=%d "
        "target_d=%.3f progress=%.3f result_code=%u detail=\"%s\"\n",
        message->command_id, ExecutionStateName(message->execution_state),
        message->plan_valid ? 1 : 0, message->target_d_valid ? 1 : 0,
        message->target_d, message->maneuver_progress, message->result_code,
        message->result_detail.c_str());
}

asmc_msgs::BehaviorCommand MakeLaneChangeCommand(
    uint32_t command_id, uint8_t execution_request, uint8_t direction,
    const std::string& source_link_id, const std::string& target_link_id,
    double target_x, double target_y, double target_yaw,
    double desired_speed_mps) {
    asmc_msgs::BehaviorCommand command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "map";
    command.command_id = command_id;
    command.maneuver_type = asmc_msgs::BehaviorCommand::LANE_CHANGE;
    command.execution_request = execution_request;
    command.direction = direction;
    command.merge_type = asmc_msgs::BehaviorCommand::MERGE_NONE;
    command.desired_speed_mps = desired_speed_mps;
    command.primary_object_id = -1;
    command.preferred_front_object_id = -1;
    command.preferred_rear_object_id = -1;
    command.use_custom_follow_gap = false;
    command.desired_time_gap_s = -1.0;
    command.minimum_gap_m = -1.0;
    command.source_link_id = source_link_id;
    command.target_link_id = target_link_id;
    command.has_target_pose = true;
    command.target_pose_map.position.x = target_x;
    command.target_pose_map.position.y = target_y;
    command.target_pose_map.position.z = 0.0;
    command.target_pose_map.orientation.z = std::sin(target_yaw * 0.5);
    command.target_pose_map.orientation.w = std::cos(target_yaw * 0.5);
    command.valid_until = ros::Time::now() + ros::Duration(5.0);
    return command;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_change_test_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string command_topic = "/behavior/command_v2";
    std::string feedback_topic = "/planning/frenet_feedback_v2";
    std::string direction_param = "LEFT";
    std::string source_link_id = "test_source_link";
    std::string target_link_id = "test_target_link";
    double target_x = 0.0;
    double target_y = 3.5;
    double target_yaw = 0.0;
    double desired_speed_mps = -1.0;
    uint32_t command_id = 1;

    pnh.param("command_topic", command_topic, command_topic);
    pnh.param("feedback_topic", feedback_topic, feedback_topic);
    pnh.param("direction", direction_param, direction_param);
    pnh.param("source_link_id", source_link_id, source_link_id);
    pnh.param("target_link_id", target_link_id, target_link_id);
    pnh.param("target_x", target_x, target_x);
    pnh.param("target_y", target_y, target_y);
    pnh.param("target_yaw", target_yaw, target_yaw);
    pnh.param("desired_speed_mps", desired_speed_mps, desired_speed_mps);
    int command_id_param = static_cast<int>(command_id);
    pnh.param("command_id", command_id_param, command_id_param);
    command_id = static_cast<uint32_t>(command_id_param);

    uint8_t direction = asmc_msgs::BehaviorCommand::DIRECTION_LEFT;
    if (direction_param == "RIGHT" || direction_param == "right") {
        direction = asmc_msgs::BehaviorCommand::DIRECTION_RIGHT;
    } else if (direction_param != "LEFT" && direction_param != "left") {
        ROS_WARN("Unknown ~direction '%s'; defaulting to LEFT",
                 direction_param.c_str());
    }

    auto command_pub =
        nh.advertise<asmc_msgs::BehaviorCommand>(command_topic, 1);
    ros::Subscriber feedback_sub =
        nh.subscribe(feedback_topic, 10, FeedbackCallback);

    ros::Duration(0.5).sleep();  // 구독자 연결 대기.

    ROS_INFO(
        "lane_change_test_publisher ready: direction=%s target=(%.2f, "
        "%.2f, yaw=%.2f) command_id=%u source_link=%s target_link=%s",
        direction == asmc_msgs::BehaviorCommand::DIRECTION_LEFT ? "LEFT"
                                                                : "RIGHT",
        target_x, target_y, target_yaw, command_id, source_link_id.c_str(),
        target_link_id.c_str());
    std::cout << "Commands: [p]repare  [e]xecute  [c]ancel  [q]uit\n";

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string line;
    while (ros::ok() && std::getline(std::cin, line)) {
        if (line.empty()) continue;
        const char key = line[0];
        if (key == 'q') break;
        uint8_t execution_request = asmc_msgs::BehaviorCommand::PREPARE;
        bool valid = true;
        switch (key) {
            case 'p':
                execution_request = asmc_msgs::BehaviorCommand::PREPARE;
                break;
            case 'e':
                execution_request = asmc_msgs::BehaviorCommand::EXECUTE;
                break;
            case 'c':
                execution_request = asmc_msgs::BehaviorCommand::CANCEL;
                break;
            default:
                valid = false;
                std::cout << "unknown command '" << key << "'\n";
                break;
        }
        if (!valid) continue;
        const asmc_msgs::BehaviorCommand command = MakeLaneChangeCommand(
            command_id, execution_request, direction, source_link_id,
            target_link_id, target_x, target_y, target_yaw,
            desired_speed_mps);
        command_pub.publish(command);
        std::printf("[sent] command_id=%u execution_request=%u\n",
                    command.command_id, execution_request);
    }

    spinner.stop();
    return 0;
}
