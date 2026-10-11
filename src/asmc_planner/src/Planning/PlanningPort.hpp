#pragma once

#include "Planning/Planner.hpp"
#include <mutex>
#include <functional>
#include "Execution/RequestedIntent.hpp"
#include "Planning/PlanningResult.hpp"
#include <string>

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/BehaviorContext.h>
#include <planning_inputs/DetectedObjectArray.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <asmc_msgs/PlanFeedback.h>
#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "Frenet/frenet_planner.hpp"
#include "Parser/BehaviorCommandAdapter.hpp"
#include "Execution/CommandExecutionCompatibility.hpp"
#include "Parser/RuntimeConfig.hpp"

namespace frenet_planner_ros {

class PlanningPort {
 public:
    using FeedbackSink = std::function<void(const asmc_msgs::FrenetFeedback&)>;
    PlanningPort(ros::NodeHandle nh, ros::NodeHandle pnh);
    PlanningPort(ros::NodeHandle nh, ros::NodeHandle pnh, FeedbackSink);
    void receiveIntent(const asmc_planner::RequestedIntent&);


 private:
    friend class asmc_planner::Planner;
    asmc_planner::Planner coordinator_;
    FeedbackSink feedback_sink_;
    // ===== R8: Runtime 책임 구획 =====
    //
    // 이 클래스는 ROS 리소스와 runtime 상태를 계속 소유하지만, 실제 구현은
    // 책임별로 나뉜 여러 .cpp에 있다(정의: Runtime/<file>.cpp).
    //   Input           input_snapshot.cpp
    //   V1 Planning     frenet_planner_node.cpp (runV1PlanningCycle)
    //   V2 Planning     v2_planning.cpp
    //   Projection      target_projection.cpp (STOP만; AVOID/LANE_CHANGE는
    //                   v2_planning.cpp에 남아 있다 - R8-B2 참고)
    //   Feedback        feedback_builder.cpp
    //   Output          output_publisher.cpp
    //   Visualization   debug_marker_publisher.cpp
    //   Bootstrap       frenet_planner_node.cpp (constructor)
    // 상태 소유권은 전부 PlanningPort에 남아 있다 - 파일 이동은 정의
    // 위치만 바꾼다(R6-F에서 확립한 것과 동일한 원칙).

    // ---- Input snapshot (input_snapshot.cpp) ----
    struct InputSnapshot {
        asmc_msgs::VehicleState vehicle;
        planning_inputs::DetectedObjectArray objects;
        asmc_msgs::BehaviorContext behavior;
        ros::SteadyTime vehicle_received_at;
        ros::SteadyTime objects_received_at;
        ros::SteadyTime behavior_received_at;
        bool has_vehicle = false;
        bool has_objects = false;
        bool has_behavior = false;
    };

    void vehicleCallback(const asmc_msgs::VehicleState::ConstPtr& message);
    void objectsCallback(const planning_inputs::DetectedObjectArray::ConstPtr& message);
    void behaviorCallback(const asmc_msgs::BehaviorContext::ConstPtr& message);
    void planningTimerCallback(const ros::SteadyTimerEvent& event);

    // ---- V1 planning (frenet_planner_node.cpp) ----
    //
    // 기존(레거시) v1 planning path: /behavior/context -> /planning/feedback.
    // v2 명령이 하나도 도착하지 않았을 때만 planningTimerCallback이 호출한다
    // (runV2PlanningCycle이 false를 반환한 경우). ros::SteadyTime::now()를
    // 자체적으로 다시 얻는다 - v2와 달리 외부에서 timestamp를 주입받지
    // 않는다(R8-A2: 기존 semantics 그대로 보존).
    void runV1PlanningCycle();

    // ---- V2 planning (v2_planning.cpp) ----
    //
    // ===== v2 인터페이스 (D3-3) =====
    //
    // v1(/behavior/context -> /planning/feedback)과 병행 운영한다. v2 경로는
    // 추가될 뿐 v1의 코드 경로를 바꾸지 않는다 - 현재 MORAI에서 검증된
    // KEEP/STOP 주행을 깨지 않기 위함이다(계약 10.6절).
    //
    // D3-3 범위는 명령 수신 -> validation -> transaction -> 최소 피드백까지다.
    // 실제 궤적 생성 연결은 D4부터다(계약 12.4절).
    void behaviorCommandV2Callback(
        const ros::MessageEvent<const asmc_msgs::BehaviorCommand>& event);

    // v2 planning cycle. v1과 동일하게 20 Hz timer가 주도한다.
    //
    // 명령 callback에서 planning하지 않는 이유:
    //   - FSM이 명령을 멈추거나 늦어져도 궤적 발행이 끊기면 안 된다
    //   - valid_until 만료를 감지할 주기적 수단이 필요하다
    //   - MPC가 일정한 주기를 기대한다
    //   - v1의 input freshness / fail-safe 로직을 그대로 재사용할 수 있다
    //
    // 반환값은 "이 사이클을 v2가 처리했는지"다. false이면 호출자(timer)가
    // 기존 v1 경로를 계속 수행한다 - cutover 전까지 두 계통이 병행한다.
    bool runV2PlanningCycle(const ros::Time& now);

    // ---- Target projection (target_projection.cpp) ----
    //
    // STOP target pose(map 좌표) -> Frenet s 투영과 heading validation
    // (계약 3.4절 4항). ConvertBehaviorCommand는 RefLine을 모르므로
    // stop_position을 NaN으로 남기고, RefLine을 가진 이 계층이 완성한다.
    //
    // 실패하면 이 함수가 직접 SAFE_STOP 궤적/피드백을 발행하고 true를
    // 반환한다 - caller(runV2PlanningCycle)는 그대로 return true 한다.
    // 성공하면 planner_command.stop_position을 채우고 false를 반환한다
    // (R8-B2: STOP만 분리 - 이 블록만 이후 execution 로직과 공유 mutable
    // state 없이 early-return으로 완전히 닫혀 있었다. AVOID/LANE_CHANGE
    // 투영은 실행 로직과 얽혀 있어 v2_planning.cpp에 남는다).
    bool prepareStopTarget(const asmc_msgs::BehaviorCommand& command,
                           uint32_t feedback_command_id,
                           const CommandAcceptance& acceptance,
                           const InputSnapshot& snapshot, double ego_s,
                           bool ego_s_valid, PlannerCommand& planner_command);

    // ---- Feedback (feedback_builder.cpp) ----
    //
    // AVOID PREPARE/post-COMMIT이 채우는 피드백 필드 (D6-2c/D6-3c).
    //
    // publishFeedbackV2의 인자를 계속 늘리는 대신 한 묶음으로 넘긴다.
    // nullptr이면 기존 D4/D5 동작 그대로다 - sentinel로 채워진다.
    struct AvoidFeedbackFields {
        // 발행한 maintenance 궤적의 실제 목표속도 [m/s]. 위 주석 참고.
        double applied_speed_mps = -1.0;

        // 계약 1.3절: target_d는 음수가 유효하므로 유효성은 플래그로만
        // 판단한다. candidate_valid일 때만 true다.
        bool target_d_valid = false;
        double target_d = 0.0;

        uint32_t generated_candidate_count = 0;
        uint32_t valid_candidate_count = 0;
        double selected_cost = -1.0;
        int32_t blocking_object_id = -1;
        double maneuver_progress = -1.0;
    };

    // LANE_CHANGE PREPARE/post-COMMIT이 채우는 피드백 필드.
    //
    // AvoidFeedbackFields와 shape이 거의 같다 - FrenetFeedback.msg 자체가
    // AVOID/LANE_CHANGE 공용 필드(target_d/maneuver_progress 등)로 설계되어
    // 있기 때문이다. 그렇다고 AvoidFeedbackFields를 그대로 재사용하지 않는
    // 이유는 두 maneuver의 diagnostic 의미가 겹치지 않기 때문이다 -
    // blocking_object_id는 AVOID에서는 "회피 대상 장애물"이지만 LANE_CHANGE에는
    // 그런 개념이 없고(충돌로 막은 임의 장애물일 뿐), 타입을 공유하면 이름이
    // 두 의미 중 하나를 배신한다. 필드 목록만 같고 이름의 의미는 각자 것이다.
    struct LaneChangeFeedbackFields {
        double applied_speed_mps = -1.0;
        bool target_d_valid = false;
        double target_d = 0.0;
        uint32_t generated_candidate_count = 0;
        uint32_t valid_candidate_count = 0;
        double selected_cost = -1.0;
        int32_t blocking_object_id = -1;
        double maneuver_progress = -1.0;
    };

    // 피드백 발행. planning 결과가 있으면 관련 필드를 채운다.
    // report가 nullptr이면 궤적을 만들지 않은 사이클이다(계약 9.4절).
    // frenet_state_valid / current_s는 호출자가 ProjectEgoToS로 얻은 값을
    // 넘긴다. 별도 core API를 만들지 않고 STOP 투영에 쓰는 그 결과를
    // 재사용한다.
    //
    // ===== plan_valid와 trajectory_source는 별개 축이다 (D6-0) =====
    //
    // plan_valid        요청한 maneuver용 새 유효 후보를 만들었는가
    // trajectory_source 실제로 MPC에 무엇을 보냈는가
    //
    // D5까지는 두 값이 항상 함께 움직였다 - 후보를 만들면 그것을 그대로
    // 발행했기 때문이다. 그래서 trajectory_source == TRAJECTORY_NEW로
    // plan_valid를 유도해도 우연히 맞았다.
    //
    // D6 PREPARE부터는 어긋난다(계약 5.2절):
    //   AVOID 후보 성공 + MPC에는 KEEP 유지 궤적
    //     -> plan_valid=true,  trajectory_source=NEW
    //   AVOID 후보 실패 + MPC에는 KEEP 유지 궤적
    //     -> plan_valid=false, trajectory_source=NEW   <- 유도식이면 틀린다
    //
    // 두 번째가 위험하다. FSM은 plan_valid=true를 "EXECUTE를 보내도 된다"로
    // 읽으므로, 막힌 회피에 EXECUTE 허가가 나간다. 그래서 유도하지 않고
    // 호출자가 명시한다.
    //
    // ===== applied_speed_override (D6-2c) =====
    //
    // 기본값(nullopt)이면 planner_.last_target_speed()를 쓴다. KEEP/FOLLOW는
    // 그 값이 실제로 발행한 궤적의 목표속도와 일치한다.
    //
    // AVOID PREPARE는 다르다. maintenance가 crawl/감속/정지 중 무엇이 될지는
    // 후보 결과와 충돌 검사에 달려 있고, last_target_speed_는 그 이전에
    // 계산된 순항 목표속도(예: 10 m/s)로 남는다. 그대로 내보내면
    //   실제 MPC 궤적 목표 = 0 m/s (감속/정지 maintenance)
    //   applied_speed_mps  = 10 m/s
    // 라는 거짓 보고가 된다. FrenetFeedback.msg는 이 필드를 "실제 적용한
    // 목표속도"로 정의한다.
    //
    // 그래서 PREPARE는 발행한 궤적의 실제 종단 속도를 넘긴다.
    void publishFeedbackV2(uint32_t command_id,
                           const CommandAcceptance& acceptance,
                           const asmc_msgs::Trajectory* trajectory,
                           uint8_t trajectory_source,
                           bool plan_valid,
                           bool frenet_state_valid = false,
                           double current_s = 0.0,
                           const AvoidFeedbackFields* avoid_fields = nullptr,
                           const LaneChangeFeedbackFields* lane_change_fields =
                               nullptr);

    // input_snapshot.cpp
    InputSnapshot takeSnapshot() const;

    // ---- Output (output_publisher.cpp) ----
    void publishStop(const InputSnapshot& snapshot, uint8_t failure_reason);
    void publishPlanningResult(const asmc_planner::PlanningResult&,
                               const InputSnapshot&);
    void publishResult(const asmc_msgs::Trajectory& trajectory,
                       const InputSnapshot& snapshot,
                       bool previous_plan_still_valid);
    asmc_msgs::VehicleState stopReferenceVehicle(
        const InputSnapshot& snapshot) const;
    double trajectoryDt() const;

    // ---- Visualization (debug_marker_publisher.cpp) ----
    void publishDebugMarkers(const asmc_msgs::Trajectory& trajectory,
                             const InputSnapshot& snapshot);
    void publishDebugMarkersV2(
        const asmc_msgs::Trajectory& trajectory,
        const asmc_msgs::BehaviorCommand& command);

    // ---- State (owned by PlanningPort; see per-section comments
    // above for who reads/writes what) ----

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber vehicle_sub_;
    ros::Subscriber objects_sub_;
    ros::Subscriber behavior_sub_;
    ros::Subscriber behavior_command_v2_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher feedback_pub_;
    ros::Publisher feedback_v2_pub_;
    ros::Publisher marker_pub_;
    ros::SteadyTimer planning_timer_;

    // v2 명령 처리. adapter는 stateless이므로 멤버가 필요 없고, transaction
    // 상태만 manager가 갖는다(계약 12.8절).
    CommandExecutionManager command_manager_;

    // 마지막으로 수신한 v2 명령. timer가 소비한다. 명령 callback은 저장만
    // 하고 planning하지 않는다(runV2PlanningCycle 주석 참고).
    asmc_msgs::BehaviorCommand latest_v2_command_;
    bool has_v2_command_ = false;

    // v2 전용 궤적 캐시. command_id에 묶어 소유한다 - 다른 명령의 궤적을
    // 재사용하면 STOP 명령 중 KEEP 궤적을 따라가는 종류의 사고가 난다
    // (계약 8.4절).
    asmc_msgs::Trajectory last_v2_trajectory_;
    bool has_last_v2_trajectory_ = false;
    uint32_t last_v2_trajectory_command_id_ = 0;
    CommandFingerprint last_v2_trajectory_fingerprint_;
    AvoidFeedbackFields last_v2_avoid_feedback_;
    bool has_last_v2_avoid_feedback_ = false;
    int v2_plan_fail_streak_ = 0;

    FrenetPlanner planner_;
    bool planner_initialized_ = false;

    mutable std::mutex input_mutex_;
    asmc_msgs::VehicleState latest_vehicle_;
    planning_inputs::DetectedObjectArray latest_objects_;
    asmc_msgs::BehaviorContext latest_behavior_;
    ros::SteadyTime vehicle_received_at_;
    ros::SteadyTime objects_received_at_;
    ros::SteadyTime behavior_received_at_;
    bool has_vehicle_ = false;
    bool has_objects_ = false;
    bool has_behavior_ = false;

    asmc_msgs::VehicleState last_stop_reference_vehicle_;
    bool has_stop_reference_vehicle_ = false;
    asmc_msgs::Trajectory last_valid_trajectory_;
    bool has_last_valid_trajectory_ = false;
    int plan_fail_streak_ = 0;

    // ROS node 계층 설정(planning frequency/timeout/topic/v2 계약 검증
    // 임계값). params.yaml의 runtime: 섹션에서 로드한다 - 필드별 의미와
    // 서로 다른 maneuver끼리 값을 공유하지 않는 이유는 RuntimeConfig.hpp
    // 참고.
    RuntimeConfig runtime_{};
    bool runtime_loaded_ = false;

    static constexpr int kMaxTransientFailCycles = 5;

};

}  // namespace frenet_planner_ros
