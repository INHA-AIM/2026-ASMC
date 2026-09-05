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
        ros::SteadyTime vehicle_received_at;
        ros::SteadyTime objects_received_at;
        ros::SteadyTime behavior_received_at;
        bool has_vehicle = false;
        bool has_objects = false;
        bool has_behavior = false;
    };

    void vehicleCallback(const asmc_msgs::VehicleState::ConstPtr& message);
    void objectsCallback(const asmc_msgs::DetectedObjectArray::ConstPtr& message);
    void behaviorCallback(const asmc_msgs::BehaviorContext::ConstPtr& message);
    void planningTimerCallback(const ros::SteadyTimerEvent& event);

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
                           const AvoidFeedbackFields* avoid_fields = nullptr);

    InputSnapshot takeSnapshot() const;
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
    asmc_msgs::DetectedObjectArray latest_objects_;
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

    // v2 STOP target pose의 허용 heading 오차 [rad] (계약 3.4절 4항).
    //
    // 투영점의 reference yaw와 목표 pose의 yaw 차이가 이 값을 넘으면
    // TARGET_PROJECTION_FAILED로 거부한다. 왕복 구간이 max_lateral_offset
    // 이내로 나란히 지나갈 때 반대 방향 구간에 목표가 붙는 것을 막는다.
    //
    // 기본값 45도. 초기 개발값이며 MORAI 검증 후 조인다 - 너무 좁게 잡으면
    // 정상 STOP이 거부되고, 너무 넓으면 반대 방향을 통과시킨다.
    // 이 값은 planning 알고리즘 파라미터가 아니라 v2 계약 검증 임계값이므로
    // params.yaml이 아닌 ROS param으로 둔다(검증이 node 책임이기 때문).
    double stop_max_target_heading_error_rad_ = 0.785398;

    // ===== v2 AVOID target pose 투영 파라미터 (D6-1a) =====
    //
    // AVOID의 target pose는 선택 사항이다(원본 메시지: "AVOID: 필요하면
    // 회피 목표점"). 주어졌으면 Frenet (s, d)로 투영해 D6-2의 목표 d로 쓴다.
    //
    // STOP의 값을 그대로 쓰지 않는 이유:
    //   STOP  "정지선이 이 경로 위에 있는가"  -> 경로에 붙어 있어야 한다
    //   AVOID "회피 목표점이 이 경로 근방인가" -> 옆으로 비켜난 점이 정상이다
    // 두 질문의 허용 횡거리가 같을 이유가 없다. 그리고 STOP config를
    // AVOID가 끌어다 쓰면 STOP 튜닝이 AVOID 동작을 바꾼다(AvoidConfig에
    // dropout_grace_cycles를 따로 둔 것과 같은 판단).
    //
    // planning 알고리즘 파라미터가 아니라 v2 계약 검증 임계값이므로
    // params.yaml이 아닌 ROS param으로 둔다 - 검증이 node 책임이기 때문이다.

    // [m] 투영 탐색창. 회피 목표점은 보통 장애물 근방(수십 m 앞)이므로
    // STOP과 같은 100 m로 시작한다.
    double avoid_projection_search_window_ = 100.0;

    // [m] 목표점이 경로에서 이보다 멀면 이 경로에 대한 회피 목표가 아니라고
    // 보고 TARGET_PROJECTION_FAILED로 거부한다.
    //
    // 기본값 4.0. 이 값은 avoid_cfg_.avoid_offset(params.yaml에서 5.0)과
    // 무관하게 정한 **target pose의 spatial validity bound**다. 두 값을
    // 비교해서 정하지 않는 이유:
    //   avoid_offset  후보 생성 시 lateral_d 격자를 밀어낼 크기
    //   이 값         FSM이 준 목표점이 이 경로의 것인지 판별하는 임계값
    // 실제 회피 목표 d는 격자 크기가 아니라 차량/장애물 폭 + safety margin
    // 으로 계산되므로(legacy도 동일), avoid_offset보다 커야 할 이유가 없다.
    //
    // MORAI에서 실제 FSM target pose 분포를 보고 조정한다.
    double avoid_max_target_lateral_offset_ = 4.0;

    // [rad] 목표 pose의 heading 허용 오차. STOP과 같은 이유로 필요하다
    // (왕복 구간이 나란히 지나가면 반대 방향 구간에 목표가 붙는다).
    // 계약 3.4절 4항.
    double avoid_max_target_heading_error_rad_ = 0.785398;

    double planning_frequency_hz_ = 20.0;
    double vehicle_timeout_seconds_ = 0.20;
    double objects_timeout_seconds_ = 0.30;
    double behavior_timeout_seconds_ = 0.50;
    static constexpr int kMaxTransientFailCycles = 5;

};

}  // namespace frenet_planner_ros
