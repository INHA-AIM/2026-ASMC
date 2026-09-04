#pragma once

#include <mutex>
#include <string>

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/BehaviorContext.h>
#include <asmc_msgs/DetectedObjectArray.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <asmc_msgs/PlanFeedback.h>
#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>
#include <ros/ros.h>

#include "frenet_planner/frenet_planner.hpp"
#include "ros/behavior_command_adapter.hpp"
#include "ros/command_execution_manager.hpp"

namespace frenet_planner_ros {

class FrenetPlannerNode {
 public:
    FrenetPlannerNode(ros::NodeHandle nh, ros::NodeHandle pnh);

 private:
    struct InputSnapshot {
        asmc_msgs::VehicleState vehicle;
        asmc_msgs::DetectedObjectArray objects;
        asmc_msgs::BehaviorContext behavior;
        bool has_vehicle = false;
        bool has_objects = false;
        bool has_behavior = false;
    };

    void vehicleCallback(const asmc_msgs::VehicleState::ConstPtr& message);
    void objectsCallback(const asmc_msgs::DetectedObjectArray::ConstPtr& message);
    void behaviorCallback(const asmc_msgs::BehaviorContext::ConstPtr& message);
    void planningTimerCallback(const ros::TimerEvent& event);

    // ===== v2 인터페이스 (D3-3) =====
    //
    // v1(/behavior/context -> /planning/feedback)과 병행 운영한다. v2 경로는
    // 추가될 뿐 v1의 코드 경로를 바꾸지 않는다 - 현재 MORAI에서 검증된
    // KEEP/STOP 주행을 깨지 않기 위함이다(계약 10.6절).
    //
    // D3-3 범위는 명령 수신 -> validation -> transaction -> 최소 피드백까지다.
    // 실제 궤적 생성 연결은 D4부터다(계약 12.4절).
    void behaviorCommandV2Callback(
        const asmc_msgs::BehaviorCommand::ConstPtr& message);
    void publishFeedbackV2(const asmc_msgs::BehaviorCommand& message,
                           const CommandAcceptance& acceptance);
    InputSnapshot takeSnapshot() const;
    bool isFresh(const ros::Time& stamp, double timeout_seconds,
                 const ros::Time& now, double& age_seconds) const;
    void publishStop(const InputSnapshot& snapshot, uint8_t failure_reason,
                     const ros::Time& now);
    void publishResult(const asmc_msgs::Trajectory& trajectory,
                       const InputSnapshot& snapshot,
                       bool previous_plan_still_valid);
    asmc_msgs::VehicleState stopReferenceVehicle(
        const InputSnapshot& snapshot) const;
    double trajectoryDt() const;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber vehicle_sub_;
    ros::Subscriber objects_sub_;
    ros::Subscriber behavior_sub_;
    ros::Subscriber behavior_command_v2_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher feedback_pub_;
    ros::Publisher feedback_v2_pub_;
    ros::Timer planning_timer_;

    // v2 명령 처리. adapter는 stateless이므로 멤버가 필요 없고, transaction
    // 상태만 manager가 갖는다(계약 12.8절).
    CommandExecutionManager command_manager_;

    FrenetPlanner planner_;
    bool planner_initialized_ = false;

    mutable std::mutex input_mutex_;
    asmc_msgs::VehicleState latest_vehicle_;
    asmc_msgs::DetectedObjectArray latest_objects_;
    asmc_msgs::BehaviorContext latest_behavior_;
    bool has_vehicle_ = false;
    bool has_objects_ = false;
    bool has_behavior_ = false;

    asmc_msgs::VehicleState last_stop_reference_vehicle_;
    bool has_stop_reference_vehicle_ = false;
    asmc_msgs::Trajectory last_valid_trajectory_;
    bool has_last_valid_trajectory_ = false;
    int plan_fail_streak_ = 0;

    double planning_frequency_hz_ = 20.0;
    double vehicle_timeout_seconds_ = 0.20;
    double objects_timeout_seconds_ = 0.30;
    double behavior_timeout_seconds_ = 0.50;
    static constexpr int kMaxTransientFailCycles = 5;

    // 시계가 뒤로 점프했다고 볼 수 있는 age 하한 [s]. 이보다 더 미래인 stamp는
    // 시계 점프로 설명되지 않는 잘못된 입력으로 보고 거부한다.
    // 실측 기준: WSL2/Docker 환경에서 약 28초 주기로 wall clock이 -1.2초 점프.
    static constexpr double kMaxClockJumpSeconds = 5.0;
};

}  // namespace frenet_planner_ros
