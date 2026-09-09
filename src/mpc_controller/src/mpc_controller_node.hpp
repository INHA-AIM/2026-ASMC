#pragma once

#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/ros.h>

#include "control/mpc_controller.hpp"
#include "diagnostics/tracking_diagnostics.hpp"

namespace mpc_controller_ros {

class MpcControllerNode {
 public:
    MpcControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
    void trajectoryCallback(const asmc_msgs::Trajectory::ConstPtr& message);
    void vehicleStateCallback(const asmc_msgs::VehicleState::ConstPtr& message);
    void behaviorCommandCallback(
        const asmc_msgs::BehaviorCommand::ConstPtr& message);
    void controlTimerCallback(const ros::SteadyTimerEvent&);
    void publishStop(const char* reason);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber behavior_command_sub_;
    ros::Publisher control_pub_;
    ros::SteadyTimer control_timer_;

    asmc_msgs::Trajectory latest_trajectory_;
    bool has_trajectory_ = false;
    ros::SteadyTime trajectory_received_at_;
    double trajectory_timeout_ = 0.25;

    // x0(ControlProcess의 현재 상태)의 canonical source. trajectory.ego_at_plan
    // 은 provenance 검증에만 쓰고 더 이상 x0로 쓰지 않는다 (Commit B).
    asmc_msgs::VehicleState latest_vehicle_state_;
    bool has_vehicle_state_ = false;
    ros::SteadyTime vehicle_state_received_at_;
    double vehicle_state_timeout_ = 0.20;

    MpcController controller_;
    bool emergency_stop_active_ = false;

    // MPC-D1: 개발용 상세 추종 진단(디버그 전용, 기본 false). 제어 로직에는
    // 영향을 주지 않는다 - detailed_tracking_이 꺼져 있으면 이번 cycle의
    // 진단 계산 자체를 건너뛴다(20Hz 루프에 불필요한 연산을 남기지 않기
    // 위함). csv_logging_이 켜져 있으면 detailed_tracking_ 계산 결과를 매
    // cycle csv_writer_에 append한다.
    bool detailed_tracking_ = false;
    bool csv_logging_ = false;
    TrackingDiagnosticCsvWriter csv_writer_;
};

}  // namespace mpc_controller_ros
