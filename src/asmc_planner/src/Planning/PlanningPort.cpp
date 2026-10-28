#include "Planning/PlanningPort.hpp"
#include "Planning/RuntimeCycle.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>
#include <utility>

#include "Parser/PlanningInput.hpp"
#include "Planning/Compatibility/steady_freshness.hpp"

namespace frenet_planner_ros {

PlanningPort::PlanningPort(ros::NodeHandle nh, ros::NodeHandle pnh)
    : PlanningPort(std::move(nh), std::move(pnh), FeedbackSink{}) {}

PlanningPort::PlanningPort(ros::NodeHandle nh, ros::NodeHandle pnh,
                           FeedbackSink feedback_sink)
    : feedback_sink_(std::move(feedback_sink)),
      nh_(std::move(nh)), pnh_(std::move(pnh)) {
    nh_.param("/planning_mode/local_prepare_enabled", local_prepare_enabled_, false);
    if (local_prepare_enabled_) {
      nh_.param("/local_planner/speed_cap_kph", local_entry_cap_kph_, 10.);
      local_prepare_sub_ = nh_.subscribe<std_msgs::Bool>("/planning/mode/local_prepare",1,
          [this](const std_msgs::Bool::ConstPtr& m) {
            local_preparing_ = m->data; local_prepare_received_ = ros::SteadyTime::now();
          });
    }
    nh_.param("/control_binding/enabled",control_binding_enabled_,false);
    if(control_binding_enabled_) {
      control_plan_pub_=nh_.advertise<std_msgs::UInt8MultiArray>("/planning/control_plan",10);
      control_request_sub_=nh_.subscribe("/behavior/control_request",10,&PlanningPort::controlRequestCallback,this);
    }
    planner_.BindExecutionState(command_manager_.maneuvers());
    std::string planner_config_path;
    pnh_.param<std::string>("planner/config_path", planner_config_path, "");

    if (planner_config_path.empty()) {
        ROS_ERROR("Missing required private parameter: planner/config_path");
    } else {
        std::string runtime_error;
        if (!LoadRuntimeConfig(planner_config_path, runtime_, runtime_error)) {
            ROS_ERROR_STREAM("Frenet runtime config load failed ('"
                             << planner_config_path << "'): " << runtime_error);
        } else {
            runtime_loaded_ = true;
        }

        planner_initialized_ = planner_.Init(planner_config_path);
        if (!planner_initialized_) {
            ROS_ERROR_STREAM("Frenet planner initialization failed: "
                             << planner_config_path);
        }
    }

    bool highway_enabled=false;
    pnh_.param("highway_merge_enabled",highway_enabled,false);
    if(highway_enabled) {
        std::string error;
        if(!planner_initialized_||!planner_.EnableHighwayMerge(error))
            throw std::runtime_error("highway preflight failed: "+error);
        highway_reference_=std::make_unique<asmc_planner::lane_reference::HighwayReferenceExecution>(
            planner_,planner_config_path,command_manager_.maneuvers());
    }

    trajectory_pub_ =
        nh_.advertise<asmc_msgs::Trajectory>(runtime_.trajectory_topic, 1);
    feedback_pub_ =
        nh_.advertise<asmc_msgs::PlanFeedback>(runtime_.feedback_topic, 1);
    feedback_v2_pub_ = nh_.advertise<asmc_msgs::FrenetFeedback>(
        runtime_.feedback_v2_topic, 1);

    // v2 구현 상태. 열린 maneuver만 planner를 호출한다(계약 11.4절).
    // 닫혀 있는 maneuver는 FAILED로 회신한다 - 조용히 velocity keeping
    // 궤적을 만드는 것보다 안전하다.
    //
    // LANE_CHANGE는 fixed-reference 검증을 위해 이 gate를 연다 - Behavior
    // Planner의 scorer/policy는 그대로 닫혀 있으므로 실제 FSM이 자동으로
    // LANE_CHANGE를 고르지는 않는다. 수동 검증은
    // tools/lane_change_test_publisher.cpp로 명령을 직접 발행해 수행한다.
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
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::LANE_CHANGE, true);
    command_manager_.SetImplementedManeuver(
        asmc_msgs::BehaviorCommand::MERGE, planner_.roundabout_merge_enabled()||planner_.highway_merge_enabled());
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        runtime_.markers_topic, 1);

    vehicle_sub_ = nh_.subscribe(
        runtime_.vehicle_state_topic, 1, &PlanningPort::vehicleCallback, this);
    objects_sub_ = nh_.subscribe(
        runtime_.objects_topic, 1, &PlanningPort::objectsCallback, this);
    behavior_sub_ = nh_.subscribe(
        runtime_.behavior_topic, 1, &PlanningPort::behaviorCallback, this);
    behavior_command_v2_sub_ = nh_.subscribe(
        runtime_.behavior_command_v2_topic, 1,
        &PlanningPort::behaviorCommandV2Callback, this);

    // runtime_loaded_==false면 planning_frequency_hz가 정의되지 않은 값일 수
    // 있다 - 이 경우 timer를 만들지 않는다(1Hz fallback으로 조용히 계속
    // 도는 대신 명백히 동작하지 않는 상태로 둔다).
    if (runtime_loaded_) {
        planning_timer_ = nh_.createSteadyTimer(
            ros::WallDuration(1.0 / runtime_.planning_frequency_hz),
            &PlanningPort::planningTimerCallback, this);
    }

    ROS_INFO_STREAM("Frenet ROS runtime initialized at "
                    << runtime_.planning_frequency_hz
                    << " Hz; inputs: " << runtime_.vehicle_state_topic << ", "
                    << runtime_.objects_topic << ", " << runtime_.behavior_topic
                    << "; outputs: " << runtime_.trajectory_topic << ", "
                    << runtime_.feedback_topic << "; v2: "
                    << runtime_.behavior_command_v2_topic
                    << " -> " << runtime_.feedback_v2_topic
                    << " (KEEP/STOP/E-STOP/FOLLOW/AVOID/LANE_CHANGE enabled)"
                    << "; planner_initialized="
                    << (planner_initialized_ ? "true" : "false")
                    << "; runtime_loaded="
                    << (runtime_loaded_ ? "true" : "false"));
}

void PlanningPort::planningTimerCallback(const ros::SteadyTimerEvent&) {
    coordinator_.RunCycle(*this);
}

// 기존 v1 planning path(레거시 /behavior/context -> /planning/feedback).
// v2 명령이 없을 때만 호출된다(planningTimerCallback 주석 참고). 이 함수는
// 예전 timer callback 본문을 그대로 옮긴 것으로, ros::SteadyTime::now()를
// 자체적으로 다시 얻는다 - v2와 달리 이 경로는 처음부터 외부에서 주입받는
// timestamp가 아니라 이 시점의 실제 시각을 썼다(이 semantics를 바꾸지
// 않는다).
void PlanningPort::runV1PlanningCycle() {
    coordinator_.RunV1(*this);
}

}  // namespace frenet_planner_ros
