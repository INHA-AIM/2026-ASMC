/**
 * @file feedback_builder.cpp
 * @brief FrenetFeedback(v2) 메시지 생성.
 *
 * publishFeedbackV2는 command_manager_/planner_ 멤버 상태를 직접 읽는
 * FrenetPlannerNode의 멤버 함수다 - 순수 함수로 분리하려면
 * CommandExecutionManager/FrenetPlanner 참조를 그대로 파라미터로 받아야
 * 하는데, 그러면 이 함수를 호출하는 28개 지점(runV2PlanningCycle 전역에
 * 분산)이 각자 다른 지역 변수(committed/stop/reason 등)에 깊게 결합되어
 * 있어 실질적인 단순화 없이 파라미터만 늘어난다. 그래서 이번에는 멤버
 * 함수 자체는 그대로 두고, "FrenetFeedback 생성"이라는 책임이 파일
 * 이름에 드러나도록 정의만 이 파일로 옮긴다.
 */

#include "frenet_planner_node.hpp"

#include "avoid_commit_coordinator.hpp"

namespace frenet_planner_ros {

void FrenetPlannerNode::publishFeedbackV2(
    uint32_t command_id, const CommandAcceptance& acceptance,
    const asmc_msgs::Trajectory* trajectory, uint8_t trajectory_source,
    bool plan_valid, bool frenet_state_valid, double current_s,
    const AvoidFeedbackFields* avoid_fields,
    const LaneChangeFeedbackFields* lane_change_fields) {
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

    // Frenet 상태와 trajectory source는 독립 축이다. post-COMMIT AVOID/
    // LANE_CHANGE는 LOCKED source여도 이번 cycle에 실제 planning을 했으므로
    // 유효한 속도와 s/d를 보낸다. terminal latch는 caller가
    // state_valid=false로 구분한다.
    //
    // 두 fields 포인터는 같은 cycle에 동시에 넘어오지 않는다 - 호출자는
    // planner_command.mode에 따라 둘 중 하나만 채운다(AVOID 또는
    // LANE_CHANGE). 있는 쪽의 applied_speed_mps를 쓴다.
    const bool has_maneuver_fields =
        avoid_fields != nullptr || lane_change_fields != nullptr;
    const double maneuver_applied_speed_mps = avoid_fields
        ? avoid_fields->applied_speed_mps
        : (lane_change_fields ? lane_change_fields->applied_speed_mps : -1.0);
    const FeedbackKinematics kinematics = ResolveFeedbackKinematics(
        trajectory_source, has_maneuver_fields, maneuver_applied_speed_mps,
        planner_.last_target_speed(), frenet_state_valid, current_s,
        planner_.last_d());
    feedback.applied_speed_mps = kinematics.applied_speed_mps;
    feedback.frenet_state_valid = kinematics.frenet_state_valid;
    feedback.current_s = kinematics.current_s;
    feedback.current_d = kinematics.current_d;

    // D4-1은 KEEP만 다룬다. 아직 채우지 않는 필드는 메시지가 정의한 sentinel을
    // 정확히 사용한다 - 임의의 0이나 추측값을 넣지 않는다(계약 9.4절).
    feedback.maneuver_progress = avoid_fields
        ? avoid_fields->maneuver_progress
        : (lane_change_fields ? lane_change_fields->maneuver_progress : -1.0);
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
    } else if (lane_change_fields) {
        // LANE_CHANGE(D7)는 AVOID와 같은 필드를 재사용한다(계약 9.4절: "D7
        // (LANE_CHANGE): D6 필드 재사용"). target_d는 요청값이 아니라 실제
        // selected/locked된 값이다(호출자가 이미 그렇게 채워 넘긴다).
        feedback.target_d_valid = lane_change_fields->target_d_valid;
        feedback.target_d = lane_change_fields->target_d;
        feedback.generated_candidate_count =
            lane_change_fields->generated_candidate_count;
        feedback.valid_candidate_count =
            lane_change_fields->valid_candidate_count;
        feedback.selected_cost = lane_change_fields->selected_cost;
        feedback.blocking_object_id = lane_change_fields->blocking_object_id;
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
