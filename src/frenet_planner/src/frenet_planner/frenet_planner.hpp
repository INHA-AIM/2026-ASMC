#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "frenet_planner/Reference/Reference.hpp"
#include "frenet_planner/Generator/GeneratorConfig.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Evaluation/Cost.hpp"
#include "frenet_planner/frenet/avoidance_selector.hpp"
#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/frenet/leader_selector.hpp"
#include "frenet_planner/frenet/merge_selector.hpp"
#include "frenet_planner/global/data_logger.hpp"
#include "frenet_planner/global/global.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"
#include "frenet_planner/visualization/planner_debug_writer.hpp"

// 폐루프 트랙에서 current_s -> target_s의 전방 호길이(frenet_planner.cpp 정의).
// STOP stop_distance 계산에 쓰이며 Maneuver/Stop.cpp에서도 호출한다.
double ForwardArcDistance(double current_s, double target_s, double s_max);

// fresh Cartesian collision recheck(frenet_planner.cpp 정의). AVOID(저속
// crawl/PREPARE)와 MERGE(M2: locked replay/commit_pending candidate 검증)가
// 모두 쓰며 Maneuver/Merge.cpp에서도 호출한다.
bool CartesianTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObstacle>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id = nullptr);

// Frenet Frame Path Planner: 전역 경로(RefLine)를 기준으로 한 Local Planner.
// FSM(behavior_planner)이 아직 연동되지 않아서, 모드 전환 조건을 지금은
// Plan() 안에 직접 하드코딩해뒀다(내 차선을 막는 정지/저속 장애물이 있으면
// AVOID, 그 다음 대상 차선에 sa/sb(앞/뒤 차량)가 모두 있으면 MERGE, 그
// 다음 선두 차량이 있으면 FOLLOWING, 다 없으면 LANE_KEEPING). MERGE는
// 설정된 회전교차로 conflict_s 접근구간에서 시간 gap을 판단한다. behavior_planner
// 연동 시 이 조건문들을 걷어내고 FSM이 준 mode를 그대로 쓰도록 교체한다.
class FrenetPlanner {
public:
    // Plan() 실패 원인. 호출자(ROS node)가 "이전 궤적 재사용"과 "즉시 정지"를
    // 구분하려면 일시적 생성 실패와 명령 자체의 결함이 분리되어야 한다.
    // TRANSIENT_GENERATION_FAILURE만 재사용 대상이다.
    enum class PlanFailureReason {
        NONE,
        COLLISION_BLOCKED,
        // 이번 사이클에서만 유효 후보를 못 만든 경우(수치적/일시적).
        TRANSIENT_GENERATION_FAILURE,
        // 모드 자체는 지원하지만 필드 값이 잘못된 경우
        // (STOP인데 stop_position 미지정, 음수/NaN 속도 등).
        INVALID_COMMAND,
        // 외부 command로 아직 열지 않은 모드.
        UNSUPPORTED_BEHAVIOR,
        NOT_INITIALIZED,
        // FSM이 지정한 대상 객체를 유효하게 확보할 수 없다(FOLLOW leader,
        // AVOID 장애물). node가 "FOLLOW인데 실패했으니 아마 leader가
        // 사라졌겠지"라고 추측하면 계약 9.2절의 원칙을 깬다 - core가 정확한
        // 사유를 준다.
        TARGET_OBJECT_LOST,
    };

    // yaml_path: frenet_planner/config/params.yaml
    bool Init(const std::string& yaml_path);

    // ego: 현재 차량 상태(Cartesian), obstacles: 장애물 목록.
    // 반환: 성공 시 true + out_path에 결과. 유효 후보가 없으면 false.
    bool Plan(const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
              CartesianPath& out_path);

    // 외부 FSM command 기반 진입점. 현재 external whitelist는 LANE_KEEPING /
    // STOP / EMERGENCY / FOLLOWING / AVOID이며(IsSupportedExternalMode 참고),
    // MERGE/LANE_CHANGE는 core 구현이 있어도 UNSUPPORTED_BEHAVIOR로 거부한다.
    // ROS 계약이 확정되는 모드부터 whitelist를 확장한다(D7/D8). 기존 3인자
    // Plan()은 legacy UDP 회귀 경로로 유지한다.
    bool Plan(const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
              const PlannerCommand& command, CartesianPath& out_path);

    // ROS/LiDAR 경로: collision_obstacles의 authoritative 8 corners를 SAT에
    // 직접 사용하며 center/dimension 기반 geometry 재생성을 하지 않는다.
    bool Plan(const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
              const std::vector<TrackedObstacle>& collision_obstacles,
              const PlannerCommand& command, CartesianPath& out_path);

    // Cartesian 점을 reference line의 종방향 좌표 s로 투영한다.
    // 외부 STOP 명령의 (stop_x, stop_y)를 PlannerCommand.stop_position으로
    // 바꾸는 데 쓴다 - message_converter는 RefLine을 모르므로 이 변환을 할 수
    // 없다. s_hint는 FindClosestS의 탐색창 힌트로 그대로 전달된다: 트랙이
    // 자기 자신과 가까워지는 구간에서 최근접점이 다른 구간으로 튀는 것을
    // 막으려면 현재 위치 근방으로 탐색을 제한해야 한다(ref_line.hpp 주석 참고).
    // 투영점이 실제 목표점에서 stop_cfg_.max_lateral_offset보다 멀면 이 경로에
    // 대한 정지 지점이 아니라고 보고 false를 반환한다.
    bool ProjectPointToS(double x, double y, double s_hint, double& out_s) const;

    // Cartesian 점을 reference line의 (s, d)로 투영한다 (D6-1a).
    //
    // ===== 왜 ProjectPointToS를 재사용하지 않는가 =====
    //
    // 그 함수는 이름과 달리 **STOP 전용**이다:
    //   - 탐색창이 stop_cfg_.projection_search_window에 묶여 있다
    //   - 허용 횡거리가 stop_cfg_.max_lateral_offset에 묶여 있다
    //   - d를 반환하지 않는다
    //
    // AVOID target pose는 d가 핵심이다(어느 쪽으로 얼마나 비켜갈지). 그리고
    // AVOID의 허용 횡거리는 STOP의 것과 같을 이유가 없다 - STOP은 "정지선이
    // 이 경로 위에 있는가"를 묻고, AVOID는 "회피 목표점이 이 경로 근방인가"를
    // 묻는다.
    //
    // 그래서 STOP config를 AVOID에 끌어오거나 ProjectPointToS에 d 출력을
    // 덧붙이는 대신, 두 용도의 공통 primitive를 아래에 두고 ProjectPointToS는
    // 그 wrapper로 남긴다. D4에서 MORAI로 검증한 STOP 투영 동작은 바뀌지
    // 않는다.
    //
    // search_window / max_lateral_distance는 호출자가 정한다. 어느 쪽이든
    // 유한하지 않거나 0 이하면 false다.
    bool ProjectPointToFrenet(double x, double y, double s_hint,
                              double search_window, double max_lateral_distance,
                              double& out_s, double& out_d) const;

    // Ego Cartesian 위치의 s. 노드가 ProjectPointToS의 s_hint를 얻는 용도.
    bool ProjectEgoToS(const CartesianState& ego, double& out_s) const;

    // reference line의 s 지점 진행방향(yaw). 읽기 전용 조회다.
    //
    // 용도: v2 STOP의 target pose heading 검증(계약 3.4절 4항). 현재 투영
    // 방어는 s_hint + projection_search_window + max_lateral_offset뿐이라,
    // 왕복 구간이 max_lateral_offset 이내로 나란히 지나가면 **진행 방향이
    // 반대인 구간에 목표가 붙을 수 있다.**
    //
    // 이 검증을 ProjectPointToS 안에 넣지 않은 이유: 그 함수는 v1 STOP도
    // 쓰고 있어서, v2 보강을 하려다 C4에서 검증한 v1 동작까지 바뀐다.
    // core는 "s -> reference yaw" 조회만 제공하고, "이 heading 차이가 허용
    // 가능한가"라는 v2 계약 판단은 ROS 계층이 한다.
    //
    // s가 reference line 범위를 벗어나거나 결과가 유한하지 않으면 false.
    bool ReferenceHeadingAt(double s, double& out_heading) const;

    // Plan()이 반환하는 CartesianPath의 샘플 간격 [s] (params.yaml의 planner.dt).
    // MPC 쪽에서 이 값을 알아야 시간 기준으로 재샘플링해 궤적 추종(time-aligned
    // tracking)을 할 수 있다 (frenet_planner.cpp의 dt와 mpc_params.dt가 서로
    // 달라서 그냥 index를 맞춰 쓸 수 없음).
    double sample_dt() const { return path_cfg_.dt; }

    // 진단용: 매 Plan() 호출마다 갱신되는 현재 d(차선 중앙 기준 오프셋)/d_dot(횡방향
    // 속도). 횡방향 오차가 조향 포화 때문인지(모델/액추에이션 문제) 아니면 여유가
    // 있는데도 못 줄이는지(모델 불일치) 구분하려고 main.cpp 로그에서 steer_norm과
    // 같이 찍어본다.
    double last_d() const { return last_d_; }
    double last_d_dot() const { return last_d_dot_; }
    BehaviorState last_mode() const { return last_mode_; }
    double last_target_speed() const { return last_target_speed_; }

    // 외부 STOP이 HOLD 단계에 도달했는지. ROS 계층이 "정지 완료"를 판단하는
    // 데 쓴다.
    //
    // 이 getter가 필요한 이유: HOLD 진입 조건은 stop_cfg_의 position/speed
    // tolerance로 core 안에서 결정된다. node가 그 조건을 다시 구현해 "아마
    // HOLD일 것"이라고 추측하면 두 곳의 판정이 어긋난다. 이미 계산된 실행
    // 상태를 읽기만 한다 - planner 동작은 바뀌지 않는다.
    bool stop_holding() const {
        return stop_.phase == StopExecutionPhase::HOLD;
    }

    // v2 FOLLOW가 실제로 추종 중인 leader ID. FrenetFeedback.active_leader_id
    // 용이다. node가 command.primary_object_id를 그대로 되돌려 보내면
    // "지정했다"와 "실제로 추종 중이다"를 구분할 수 없다 - 확보에 실패한
    // 사이클에도 ID가 나가게 된다(계약 9.2절).
    int active_leader_id() const {
        return v2_following_.active ? v2_following_.leader_id : -1;
    }

    // v2 AVOID가 실제로 확보한 회피 대상 ID (D6-1). active_leader_id()와 같은
    // 이유로 존재한다 - 지정만 된 ID와 확보된 ID를 구분해야 한다.
    int active_avoid_target_id() const {
        return v2_avoidance_.active ? v2_avoidance_.obstacle_id : -1;
    }

    // v2 AVOID 대상이 유예(미관측) 중인 사이클 수. 진단·회귀 검증용이다.
    int avoid_missing_cycles() const { return v2_avoidance_.missing_cycles; }

    // v2 AVOID가 확보한 대상의 Frenet 위치. active_avoid_target_id()가 -1이면
    // 의미가 없다.
    double avoid_target_s() const { return v2_avoidance_.obstacle_s; }
    double avoid_target_d() const { return v2_avoidance_.obstacle_d; }

    // v2 AVOID에 대해 FSM이 요청한 방향. mode=AVOID 하나로는 구분할 수 없어
    // 별도 필드로 보존한다(계약 3.3절).
    LateralDirection avoid_requested_direction() const {
        return v2_avoidance_.requested_direction;
    }

    // FSM이 target pose로 지정한 회피 목표의 Frenet 값 (D6-1a).
    //
    // has_...가 false이면 pose가 없었거나 투영이 실패한 것이다. 그때는
    // 장애물 geometry와 요청 방향으로 목표를 계산한다(D6-2).
    //
    // d는 음수가 유효하므로 -1.0 sentinel을 쓸 수 없다 - 유효성은 플래그로만
    // 판단한다(계약 1.3절과 같은 이유).
    bool avoid_has_requested_target_d() const {
        return v2_avoidance_.has_requested_target_d;
    }
    double avoid_requested_target_d() const {
        return v2_avoidance_.requested_target_d;
    }
    double avoid_requested_target_s() const {
        return v2_avoidance_.requested_target_s;
    }

    // 이번 사이클에 평가할 회피 목표 d 목록 (D6-2a). 확보에 실패했으면
    // count()가 0이다. D6-2b가 이 목록으로 후보를 만든다.
    int avoid_target_count() const { return v2_avoid_targets_.count; }
    double avoid_target_candidate(int index) const {
        if (index < 0 || index >= v2_avoid_targets_.count) return 0.0;
        return v2_avoid_targets_.candidates[index];
    }
    bool avoid_target_from_pose() const {
        return v2_avoid_targets_.from_target_pose;
    }
    // 방향 요청과 target pose가 모순인지. 명령 결함이 아니므로 여기서
    // 거부하지 않고, 후보 생성 결과(WAITING)로 드러난다(계약 3.3.4절).
    bool avoid_direction_conflict() const {
        return v2_avoid_targets_.direction_conflict;
    }

    PlanFailureReason last_failure_reason() const { return last_failure_reason_; }

    // 이번 PlanInternal 호출이 무엇을 하려는지 (D6-2b).
    //
    // AvoidPrepareResult* 가 nullptr인지로 mode를 암묵적으로 표현하지 않는다.
    // 그러면 나중에 진단용으로 포인터만 넘겨도 동작이 바뀐다. 입력/제어축과
    // 출력축을 명시적으로 분리한다:
    //
    //   purpose               무슨 계산을 할지
    //   AvoidPrepareResult*   그 계산 결과를 내보냄
    //   out_path              실제 MPC로 보낼 궤적
    //
    // ROS의 execution_request enum을 core로 가져오지 않는다 - core는
    // handshake를 모르고, "준비만 하는 계산"이라는 것만 안다.
    enum class ExternalPlanningPurpose {
        NORMAL,
        AVOID_PREPARE,
        AVOID_COMMITTED,
    };

    // AVOID PREPARE 한 사이클의 결과 (계약 5.2절).
    //
    // ===== candidate_path와 maintenance path는 다른 것이다 =====
    //
    // candidate_path      "AVOID를 실행해도 되는가"의 근거. MPC로 보내지 않는다.
    // out_path(별도 인자)  "지금 MPC가 실제로 따라갈 것". 유지 궤적이다.
    //
    // 타입 수준에서 분리해 두는 이유는, 실수로 candidate를 발행하면 FSM이
    // EXECUTE를 보내기 전에 횡이동이 시작되기 때문이다.
    struct AvoidPrepareResult {
        // 요청한 AVOID용 새 유효 후보를 만들었는가. FrenetFeedback.plan_valid로
        // 나가는 값이다. **PrepareAvoid()의 반환값과 다른 축이다**(아래 주석).
        bool candidate_valid = false;

        // ego가 저속 구간(< kLowSpeedFallbackV)인가. **후보 성공 여부와
        // 무관한 순수 상황 정보다.**
        //
        // ===== 이름을 low_speed_waiting에서 바꾼 이유 (D6-2c) =====
        //
        // 이전 이름은 "저속 때문에 후보를 만들지 못했다"로 읽힌다. 그런데
        // 값은 candidate 평가 전에 ego.v만 보고 정해지므로,
        //   ego = 0.4 m/s + candidate_valid = true
        // 인 상황에서 "waiting인데 후보는 유효"라는 모순이 나온다.
        //
        // 현재 테스트 환경에서는 저속 후보가 실제로 막혀 드러나지 않았지만,
        // 이름이 상태 판단에 쓰이도록 유혹한다. 그래서 상황 정보로 이름을
        // 바꾸고, **execution_state는 candidate_valid만으로 정한다**:
        //   candidate_valid=true  -> READY
        //   candidate_valid=false -> WAITING
        // 이 필드는 진단·로그에만 쓴다.
        bool low_speed_context = false;

        // 요청 방향과 target pose가 모순인가. 이 경우에도 candidate_valid는
        // false여야 한다 - 충돌 검사에만 맡기면 안 된다(헤더 아래 주석).
        bool direction_conflict = false;

        // target pose가 ego 후방이라 회피 목표로 쓸 수 없는가.
        bool target_behind_ego = false;

        // 이번 사이클에 선택된 목표 d. candidate_valid일 때만 의미가 있다.
        // **여기까지가 PREPARE의 범위다** - v2_avoidance_.target_d에 lock하지
        // 않는다. NONE이면 사이클마다 좌/우가 바뀔 수 있고, FSM이 EXECUTE를
        // 주지 않았으므로 그것이 정상이다. lock은 D6-3의 COMMIT에서 한다.
        double selected_target_d = 0.0;

        // 평가한 AVOID 후보. 진단·검증용이며 MPC로 보내지 않는다.
        CartesianPath candidate_path;

        // FrenetFeedback 진단 필드용.
        std::size_t generated_candidate_count = 0;
        std::size_t valid_candidate_count = 0;
        double selected_cost = -1.0;
        int blocking_object_id = -1;

        // post-COMMIT lifecycle 결과. PREPARE에서는 sentinel/false다.
        bool maneuver_completed = false;
        double maneuver_progress = -1.0;
    };

    // AVOID PREPARE 한 사이클을 수행한다 (계약 5.2절).
    //
    // ===== 반환값의 의미 =====
    //
    // **"실제로 내보낼 maintenance_path를 유효하게 만들었는가"**다.
    // AVOID 후보의 성공 여부는 result.candidate_valid이며 별개 축이다.
    // 두 값을 다시 묶으면 D6-0에서 분리한 plan_valid / trajectory_source를
    // 되돌리는 셈이다(계약 8.3.1절).
    //
    //   maintenance 성공 + 후보 성공  -> true,  candidate_valid=true   READY
    //   maintenance 성공 + 후보 막힘  -> true,  candidate_valid=false  WAITING
    //   maintenance 실패             -> false                        안전정지
    bool PrepareAvoid(const CartesianState& ego,
                      const std::vector<ObjectInfo>& obstacles,
                      const std::vector<TrackedObstacle>& collision_obstacles,
                      const PlannerCommand& command,
                      CartesianPath& maintenance_path,
                      AvoidPrepareResult& result);

    // COMMIT 이후 잠긴 AVOID geometry를 새 collision snapshot으로 실행한다.
    // target observation/request pose는 더 이상 기동 지속 조건이 아니다.
    bool PlanCommittedAvoid(
        const CartesianState& ego,
        const std::vector<ObjectInfo>& obstacles,
        const std::vector<TrackedObstacle>& collision_obstacles,
        const PlannerCommand& command, CartesianPath& out_path,
        AvoidPrepareResult& result);

    // AVOID COMMIT의 core precondition을 mutation 없이 검사한다 (D6-3a1).
    //
    // node는 candidate 변환을 끝낸 뒤 이 검사를 통과해야 manager를
    // COMMITTED로 올릴 수 있다. single-threaded planning tick 안에서는 이
    // 검사와 ApplyAvoidCommit() 사이에 core 상태를 바꾸는 호출이 없으므로,
    // preflight 뒤의 apply는 같은 조건으로 실패하지 않는다.
    bool CanCommitAvoid(const AvoidPrepareResult& result) const;

    // CanCommitAvoid()가 성공한 결과를 적용한다. 이 단계에는 실패 반환이
    // 없다. ROS coordinator가 manager를 COMMITTED로 올린 뒤 core apply가
    // 거부되어 두 상태가 갈라지는 경로를 타입 수준에서 없앤다.
    void ApplyAvoidCommit(const AvoidPrepareResult& result);

    // 재검증을 통과한 AVOID 후보를 core 단독으로 확정하는 호환 API다.
    //
    // ===== 호출 시점이 계약이다 =====
    //
    // ROS node는 이 함수를 manager 승격 뒤 호출하지 않고 위의 preflight와
    // 실패 없는 apply를 분리해 쓴다. 전체 호출 순서는 다음이어야 한다:
    //
    //   1. PrepareAvoid()로 현재 snapshot 재평가
    //   2. result.candidate_valid 확인
    //   3. candidate_path -> ROS Trajectory 변환 성공 확인
    //   4. CanCommitAvoid() 성공
    //   5. NotifyCommitted() 성공
    //   6. ApplyAvoidCommit()
    //   7. 궤적 캐시 저장 + publish
    //
    // 5번보다 먼저 3·4번을 확인해야 하는 이유는 D6-0a와 같다. COMMIT을 먼저
    // 올리고 변환이 실패하면
    //   Feedback = COMMITTED AVOID
    //   MPC      = 직전 PREPARE의 중앙 maintenance
    // 라는 불일치가 생긴다.
    //
    // 이 시점에 target_d를 처음 lock한다. PREPARE에서는 사이클마다 좌/우가
    // 바뀔 수 있으므로 lock하지 않았다(계약 12.7.3절).
    //
    // 반환값은 실제로 lock했는지다. candidate_valid가 false인 결과를 넘기면
    // false를 돌려주고 아무것도 바꾸지 않는다 - 확보하지 않은 목표를
    // 잠그지 않는다.
    bool CommitAvoid(const AvoidPrepareResult& result);

    // v2 AVOID가 COMMIT되었는지. COMMIT 이후에는 target resolver 대신 잠긴
    // context로 기동을 완주한다(계약 8.2.1절).
    bool avoid_committed() const { return v2_avoidance_.committed; }

    // COMMIT 시점에 잠근 목표 d [m]. avoid_committed()가 false이면 의미가
    // 없다. d는 음수가 유효하므로 sentinel을 쓸 수 없다.
    double avoid_locked_target_d() const { return v2_avoidance_.target_d; }

    // COMMIT 시점의 ego d [m]. maneuver_progress를 (start_d -> target_d)
    // 구간의 진행률로 계산하는 데 쓴다(D6-3c).
    double avoid_start_d() const { return v2_avoidance_.start_d; }

    bool avoid_completed() const { return v2_avoidance_.completed; }
    double avoid_maneuver_progress() const {
        return v2_avoidance_.maneuver_progress;
    }

    // v2 AVOID의 내부 실행 단계 이름. 진단·검증용 문자열이다. 내부 enum을
    // 공개하지 않기 위해 이름으로 돌려준다.
    const char* avoid_phase_name() const;

    // 외부 AVOID 상태를 전부 비운다 (D6-3a).
    //
    // ===== 왜 별도 API가 필요한가 =====
    //
    // v2 AVOID context reset은 PlanInternal에서 "다른 mode가 들어왔을 때"만
    // 일어난다. 그런데 CANCEL이 성공하면 manager는 planner를 아예 호출하지
    // 않는다(planning_target=NONE). 그래서
    //
    //   AVOID id=20 PREPARE   -> 21 확보
    //   AVOID id=20 CANCEL    -> manager만 IDLE, planner 호출 없음
    //   AVOID id=22 target=21 -> 21이 없어도 stale context로 GRACE
    //
    // 가 된다. 첫 binding 실패인데 유예를 받는 D5-2와 같은 구조다.
    //
    // COMMITTED/EXECUTING에서 CANCEL이 거부되면(TOO_LATE_TO_CANCEL) 이
    // 함수를 **절대 호출하면 안 된다** - 진행 중인 기동의 잠긴 목표가
    // 사라진다.
    void ResetExternalAvoidance();

private:
    bool PlanInternal(const CartesianState& ego,
                      const std::vector<ObjectInfo>& obstacles,
                      const PlannerCommand* external_command,
                      const std::vector<TrackedObstacle>* collision_obstacles,
                      ExternalPlanningPurpose purpose,
                      CartesianPath& out_path,
                      AvoidPrepareResult* avoid_prepare_result);

    void UpdateCommittedAvoidLifecycle(double ego_s, double ego_d,
                                       AvoidPrepareResult& result);

    // AVOID PREPARE의 유지 궤적을 만든다 (D6-2b).
    //
    // ===== 왜 단순 KEEP이면 안 되는가 =====
    //
    // AVOID 대상은 같은 corridor에 있다. 그래서 중앙 KEEP 후보가 그 장애물
    // 때문에 충돌 검사에서 전부 탈락할 수 있다. 즉
    //
    //   AVOID candidate valid = true      (옆으로 비켜가면 안전)
    //   maintenance KEEP      = 없음      (직진하면 부딪힌다)
    //
    // 가 정상적으로 발생한다. 그렇다고 AVOID candidate를 발행하면 계약 5.2절
    // 위반이므로, 종방향으로 물러나는 대안이 필요하다.
    //
    // 우선순위:
    //   1. 충돌 검사를 통과한 KEEP/유지 궤적
    //   2. GenerateDecelToZeroPath (종방향 감속)
    //   3. 현재 위치 0속도 hold
    //
    // 저속(ego.v < kLowSpeedFallbackV)에서는 1번 대신 crawl을 쓴다. 무조건
    // 정지시키면
    //   0.3 m/s -> WAITING -> stop -> 0 m/s -> WAITING -> stop ...
    // 로 자기잠금이 되어 영원히 READY에 도달하지 못한다. crawl도 반드시
    // 충돌 검사를 통과해야 하며(PlanLowSpeedFallback 자체는 충돌 필터를
    // 타지 않는다), 막히면 2번/3번으로 내려간다.
    bool BuildAvoidMaintenancePath(
        const CartesianState& ego, const FrenetState& start,
        const PlannerCommand& base_command,
        const std::vector<TrackedObstacle>& collision_obstacles,
        CartesianPath& out_path) const;

    // 현재 위치에서 0속도로 정지 유지하는 한 점 궤적. STOP HOLD와 같은 형태다.
    void BuildZeroSpeedHoldPath(const CartesianState& ego, double s,
                                CartesianPath& out_path) const;

    // 외부(v2) AVOID target 확보 + 이번 cycle 후보 생성에 쓸 target list
    // 준비 (D6-2a, AVOID A2). PlanInternal의 external EMERGENCY 분기
    // 다음, purpose/mode 일치 guard보다 앞에서 호출된다 - 순서를 바꾸면
    // 안 된다(저속 fallback보다 반드시 위여야 한다는 계약 12.7.2절 근거는
    // 호출부 주석 참고).
    //
    // avoid_committed(purpose==AVOID_COMMITTED)와 v2_avoidance_.committed
    // (core context가 이미 COMMIT되어 target_d가 잠겼는가)는 서로 다른
    // 축이다 - 이 함수 안에서도 하나로 합치지 않는다.
    //
    // 반환값: PlanInternal이 그대로 return해야 하는 상태만 false로
    // 전달한다(TARGET_OBJECT_LOST, NORMAL purpose에서의
    // UNSUPPORTED_BEHAVIOR). 그 외엔 true.
    bool PrepareExternalAvoidTargets(
        double ego_s, const std::vector<ObjectInfo>& obstacles,
        const PlannerCommand& command, bool avoid_prepare, bool avoid_committed);

    // 외부(v2) AVOID PREPARE/COMMITTED 실행 전체 (D6-2b/D6-3, AVOID A3b).
    // PlanInternal이 FrenetState start를 만든 직후, avoid_prepare ||
    // avoid_committed일 때 그대로 호출하고 이 함수의 반환값을 그대로
    // return한다 - 이 분기의 모든 경로가 결국 true/false를 반환하므로
    // PlanInternal은 그 뒤 legacy AVOID 이하로 내려가지 않는다.
    //
    // 계약 5.2절: candidate("AVOID를 실행해도 되는가", result.candidate_valid)와
    // maintenance(out_path, "지금 MPC가 따라갈 것")는 절대 같은 것으로
    // 취급하지 않는다 - PREPARE의 반환값은 candidate_valid가 아니라
    // maintenance 성공 여부다.
    //
    // avoid_prepare(purpose==AVOID_PREPARE), avoid_committed
    // (purpose==AVOID_COMMITTED), v2_avoidance_.committed(core context
    // lock 여부)는 서로 다른 세 축이며 이 함수 안에서도 합치지 않는다.
    bool PlanExternalAvoidCycle(
        const CartesianState& ego, const FrenetState& start,
        double s, double d, double resolved_target_speed,
        const std::vector<TrackedObstacle>& predicted_collision_obstacles,
        const PlannerCommand& command, bool avoid_prepare, bool avoid_committed,
        CartesianPath& out_path, AvoidPrepareResult& result);

    // ===== D1-2: legacy 행동 선택권의 유일한 경계 =====
    //
    // Legacy(간이 FSM) 경로에서 "지금 어떤 maneuver를 할지"를 결정하는 지점을
    // 한 함수로 모았다. v2에서는 이 결정 권한이 FSM으로 넘어가며(BehaviorCommand
    // .maneuver_type), 이 함수는 호출되지 않는다 - 외부 command 경로는
    // PlanInternal에서 `cmd = *external_command`로 이 결정 전체를 건너뛴다.
    //
    // 우선순위: AVOID > MERGE > FOLLOW > KEEP (아래 if-else 순서가 곧 우선순위).
    //
    // 이 함수는 planner 상태(avoidance_/following_/merge_)를 **읽기만** 한다.
    // 상태를 갱신하는 acquisition/orchestration 코드는 호출자에 남아 있다 -
    // 그쪽은 legacy 전용 policy이므로 v2에서 재사용하지 않는다
    // (docs/d1-legacy-behavior-boundary.md 0.1절 B 분류).
    //
    // 유일한 예외: MERGE COMMIT 자동 승격(merge_.commit_pending = true)은 gap
    // 평가 코드와 물리적으로 얽혀 있어 이 함수 밖(호출자)에 남아 있다. v2에서는
    // 그 trigger가 FSM의 EXECUTE로 이동한다(계약 문서 5.4절).
    //
    // out_avoid_offset / out_leader_* 는 아래 진단 로그가 쓰는 값이라 그대로
    // 돌려준다(동작에는 영향 없음).
    void SelectLegacyBehavior(bool begin_avoidance, bool nearest_found,
                              const LeaderTarget& nearest_leader,
                              PlannerCommand& cmd, double& out_avoid_offset,
                              double& out_leader_s, double& out_leader_speed,
                              double& out_leader_accel) const;

    // TRACKING은 외부 AVOID 모드가 아니다. 정적 장애물 ID/방향만
    // 미리 잠그고 주행은 LANE_KEEPING을 계속하는 내부 상태다.
    enum class AvoidPhase { TRACKING, SHIFT, PASS };

    struct AvoidanceContext {
        bool active = false;
        int obstacle_id = -1;
        AvoidPhase phase = AvoidPhase::TRACKING;
        double obstacle_s = 0.0;
        double obstacle_d = 0.0;
        double obstacle_width = 0.0;
        double obstacle_length = 0.0;
        double target_d = 0.0;

        // ---- 아래는 v2 경로(v2_avoidance_)만 사용한다 (D6-1) ----
        // legacy 경로는 읽지도 쓰지도 않으므로 legacy 동작은 불변이다.

        // 대상의 일시적 미관측 누적. avoid_cfg_.dropout_grace_cycles를 넘으면
        // 대상 소실로 판정한다(계약 8.2절).
        int missing_cycles = 0;

        // 기동 시작 시점의 ego d. maneuver_progress를 (start_d -> target_d)
        // 구간의 진행률로 계산하기 위해 필요하다(D6-4에서 사용).
        double start_d = 0.0;

        // FSM이 요청한 방향. NONE이면 Frenet이 좌/우를 고른다.
        LateralDirection requested_direction = LateralDirection::NONE;

        // FSM이 target pose로 준 회피 목표를 Frenet으로 투영한 값 (D6-1a).
        // AVOID의 target pose는 선택 사항이므로 없을 수 있다.
        // 0.0은 유효한 d이므로 유효성은 플래그로만 판단한다.
        bool has_requested_target_d = false;
        double requested_target_d = 0.0;
        double requested_target_s = 0.0;

        // EXECUTE 재검증을 통과해 목표를 확정했는가 (D6-3a).
        //
        // true이면 target_d / start_d가 잠긴 값이며, 후보 생성은 그 목표를
        // **정확히** 겨냥한다(lateral tolerance 0). PREPARE에서는 항상
        // false다.
        bool committed = false;

        // v2-only terminal lifecycle. 완료 후에도 새 command_id가 올 때까지
        // context를 보존한다. legacy avoidance_는 이 필드를 읽거나 쓰지 않는다.
        bool completed = false;
        double maneuver_progress = 0.0;
    };

    struct FollowingContext {
        bool active = false;
        int leader_id = -1;
        double leader_s = 0.0;
        double leader_d = 0.0;
        double leader_speed = 0.0;
        double leader_accel = 0.0;
        int missing_cycles = 0;
    };

    // 외부 STOP 명령의 내부 실행 단계. 외부 BehaviorState::STOP과는 다른
    // 개념이다 - mode는 계속 STOP이고, 그 안에서 어떤 종방향 후보 생성기를
    // 쓸지가 이 phase로 갈린다.
    //   APPROACH: 정지 목표가 제동거리 밖. 속도만 낮추며 접근(velocity keeping).
    //   BRAKING : 제동거리 안. exact-stop quintic 생성.
    //   HOLD    : 목표 도달 + 저속. 0속도 유지.
    enum class StopExecutionPhase { APPROACH, BRAKING, HOLD };

    struct StopContext {
        StopExecutionPhase phase = StopExecutionPhase::APPROACH;
        // 이 context가 추종 중인 정지 목표 s. 목표가 크게 바뀌면 새 명령으로
        // 보고 phase를 리셋한다.
        double target_s = std::numeric_limits<double>::quiet_NaN();
        bool target_valid = false;
    };

    enum class MergePhase { APPROACH, WAIT, COMMIT, CROSS, CLEAR };

    struct MergeContext {
        bool active = false;
        MergeType type = MergeType::NONE;
        int highway_zone_index = -1;
        int highway_conflict_index = 0;
        MergePhase phase = MergePhase::APPROACH;
        bool gap_safe = false;
        bool gap_locked = false;
        bool commit_pending = false;  // 실제 후보 검증 전의 임시 COMMIT 요청
        int safe_cycles = 0;
        double entry_time = 0.0;
        double clear_time = 0.0;
        int preceding_id = -1;
        double preceding_time = -1.0;
        int following_id = -1;
        double following_time = -1.0;
        size_t crossing_vehicle_count = 0;
        CartesianPath locked_path;
        double locked_elapsed = 0.0;
        size_t locked_path_index = 0;  // wall-clock이 아니라 Ego의 실제 공간 진행 위치
    };

    struct ObjectTrack {
        double heading = 0.0;
        double yaw_rate = 0.0;
        int missing_cycles = 0;
    };

    // 저속(ego.v < kLowSpeedFallbackV) 전용 fallback: CartesianToFrenet/FrenetToCartesian의
    // d_prime = d_dot/s_dot 계산은 s_dot(≈속도)로 나누는 구조라 저속에서 수학적으로
    // 특이점을 가진다("고속 모드 전용" - frenet_converter.cpp 주석 참고). 그 특이점 있는
    // 경계조건으로 만든 quintic 후보가 정지 근처를 지나 실제로 움직이기 시작하는
    // 시점에 억지로 꺾인 모양이 되어 곡률필터에 걸리는 문제가 실측으로 확인됐다.
    // 저속에서는 이 경계조건 역산 자체를 안 쓰고, s/d를 직접(시간에 따라 완만하게)
    // 설계해 ComputeGeometricPath로만 렌더링한다 - 나누기가 전혀 없어 특이점이 없다.
    // 활성 AVOID가 있으면 중앙이 아니라 저장된 회피 target_d를 유지/추종한다.
    bool PlanLowSpeedFallback(const CartesianState& ego, double s, double d, double target_d,
                              CartesianPath& out_path, double duration_s = 2.0,
                              double speed_cap =
                                  std::numeric_limits<double>::infinity()) const;

    // FSM이 지정한 FOLLOW 대상의 확보 결과.
    enum class FollowTargetStatus {
        // 이번 사이클에 대상을 관측하고 검증까지 통과했다.
        VALID,
        // 관측되지 않았지만 직전에 확보한 상태가 있어 유예 중이다.
        GRACE,
        // 대상을 쓸 수 없다. 관측 자체가 없거나(첫 binding 실패), 관측되었지만
        // 차선/방향/범위를 벗어났거나, 유예를 초과했다.
        LOST,
    };

    // FSM이 지정한 대상 객체를 확보한다. **selector가 아니라 resolver다.**
    //
    //   selector: "누구를 따라갈까?"                  -> v2에서 금지
    //   resolver: "FSM이 15번을 지정했다. 유효한가?"   -> 이 함수
    //
    // FindLeader()를 호출하지 않는다. 그 함수는 같은 차선의 가장 가까운
    // 차량을 스스로 고르는 policy이며, v2에서는 대상 선택 권한이 FSM에 있다
    // (docs/d1-legacy-behavior-boundary.md 0.1절 B 분류).
    //
    // 더 가까운 차량이 나타나도 무시한다. 자동 handoff는 FSM이 지시한 대상을
    // 추종 중이라고 오인하게 만든다. 그 차량과 충돌 위험이 있으면 충돌
    // 필터가 COLLISION_BLOCKED로 막고, 대상 변경은 FSM이 새 command_id로
    // 결정한다.
    //
    // 검증 기준은 legacy와 동일하다(진행 방향, 같은 차선, 전방, 탐색 범위).
    // 유예 중에는 직전 속도로 leader_s를 예측한다.
    FollowTargetStatus ResolveExternalFollowTarget(
        double ego_s, int target_object_id,
        const std::vector<ObjectInfo>& obstacles,
        FollowingContext& context) const;

    // Legacy(내부 간이 FSM) FOLLOW의 leader 확보/유지/handoff 갱신.
    //
    // ResolveExternalFollowTarget과 달리 이쪽은 "FSM이 지정한 대상이
    // 유효한가"가 아니라 "어떤 차량을 따라갈지 스스로 고르는" legacy
    // policy다(FindLeader 사용). external(v2) 경로에서는 호출하지 않는다
    // (docs/d1-legacy-behavior-boundary.md 0.1절 B 분류).
    //
    // following_ 멤버를 직접 갱신한다: 기존 leader 유지 시도 -> dropout
    // 유예/해제 -> 인접 차량 hysteresis handoff -> 최초 획득 순서.
    //
    // out_nearest_found/out_nearest_leader: 이번 cycle에 FindLeader가 찾은
    // 최인접 차량(SelectLegacyBehavior가 KEEP->FOLLOWING 최초 전환 판단에
    // 그대로 사용한다 - following_이 아직 active=false일 수 있으므로 별도로
    // 전달해야 한다).
    void UpdateLegacyFollowContext(double ego_s, const FrenetState& start,
                                    const std::vector<ObjectInfo>& obstacles,
                                    bool& out_nearest_found,
                                    LeaderTarget& out_nearest_leader);

    // moderate-speed curvature starvation 완화(fix/yuntae-frenet-speed-continuity
    // Phase B). external KEEP에서 결합 후보가 전부 곡률 필터에 탈락하는 사례를
    // 막기 위해, 현재 오프셋을 유지하는 continuity candidate를 candidates에
    // 추가로 얹는다(대체가 아니라 추가) - 자세한 배경은 PlanInternal의 호출부
    // 주석 참고. maneuver_start.d가 차선 폭 안일 때만 호출된다(호출부에서 보장).
    void AppendExternalKeepContinuityCandidates(
        const FrenetState& maneuver_start, const PlannerCommand& cmd,
        const std::vector<TrackedObstacle>& planning_collision_obstacles,
        std::vector<FrenetPath>& candidates) const;

    // external KEEP의 no-valid-candidate 분기에서 저속 근방일 때만 호출되는
    // crawl fallback 실행부(fix/yuntae-frenet-speed-continuity Phase 5). 호출
    // 여부 판단(external_keep/속도 문턱/COLLISION_BLOCKED 제외)은 PlanInternal이
    // 그대로 유지한다 - STOP fail-safe 등 같은 분기의 다른 공용 실패 처리와
    // 뒤섞이지 않게 하기 위함이며, 이 함수는 "이미 호출하기로 결정된 뒤" 실제
    // crawl 경로 생성과 last_failure_reason_ 갱신만 담당한다.
    bool PlanExternalKeepLowSpeedFallback(const CartesianState& ego, double s,
                                          double d, double resolved_target_speed,
                                          CartesianPath& out_path);

    // 외부 EMERGENCY_STOP 실행부. 후보 파이프라인을 건너뛰고 직접 감속한다.
    // 호출 위치(v2 FOLLOW/AVOID context reset 다음, 저속 fallback/AVOID 대상
    // 확보보다 먼저)는 PlanInternal이 그대로 유지한다 - 이 함수는 gate 판단
    // 없이, 이미 EMERGENCY로 확정된 뒤의 실행(감속 경로 생성/state 갱신/
    // 5-cycle 로그)만 담당한다. 자세한 배경은 PlanInternal 호출부 주석 참고.
    bool PlanExternalEmergencyStop(const CartesianState& ego, double s, double d,
                                    double s_dot, CartesianPath& out_path);

    // 외부 STOP의 APPROACH/BRAKING/HOLD phase 전이와 그에 따른 cmd 조정
    // (target 변경 감지 -> phase reset, braking 진입거리 latch, HOLD 진입
    // 판정, APPROACH/BRAKING 각각의 target_speed/mode 조정)을 그대로
    // 옮긴 것이다. stop_.phase/target_s/target_valid를 직접 갱신한다.
    //
    // 주의: APPROACH에서 cmd.mode를 LANE_KEEPING으로 바꾸지만(candidate
    // 생성만 velocity keeping에 위임하기 위함) 이 함수 반환 이후
    // PlanInternal이 last_mode_ = STOP으로 별도 유지한다 - 그 진단용
    // ordering은 이 함수가 아니라 호출부 책임이므로 여기서 last_mode_를
    // 건드리지 않는다.
    void UpdateExternalStopExecution(double s, double s_dot, PlannerCommand& cmd,
                                      double& out_stop_distance);

    // 외부 STOP이 HOLD phase에 도달했을 때의 0속도 1-point 궤적 출력부.
    // 호출 gate(external_stop_mode && phase==HOLD)는 PlanInternal에 유지한다.
    void PlanExternalStopHold(const CartesianState& ego, double s,
                              CartesianPath& out_path) const;

    // 외부 STOP BRAKING에서 후보가 0개일 때의 fail-safe 실행부(그 자리에서
    // 감속해 정지하는 decel-to-zero 궤적). 호출 gate(phase==BRAKING &&
    // COLLISION_BLOCKED 아님)는 common no-valid 분기에 강하게 결합되어
    // 있으므로 PlanInternal에 유지한다.
    void PlanExternalStopBrakingFallback(double s, double d, double s_dot,
                                         double stop_distance,
                                         CartesianPath& out_path);

    // 외부 STOP은 절대 후진하지 않는다 - 선택된 Cartesian 궤적에서 속도가
    // 처음 음수가 되는 지점 이후를 잘라내고 마지막 점의 v/a를 0으로 만든다.
    // 호출 gate(external_stop_mode)는 PlanInternal에 유지한다.
    void TrimExternalStopReverseTrajectory(CartesianPath& selected_cartesian) const;

    // legacy MERGE의 approach/window/gap 판단과 phase/context 갱신
    // (MERGE M1). 이번 cycle에 roundabout/highway 중 어느 정책이 활성인지
    // 결정하고, gap_locked 여부에 따라 timer decrement 또는
    // FindRoundaboutGap/FindHighwayMergeGap 판단으로 merge_ phase를
    // 갱신한다. locked COMMIT trajectory replay, commit_pending candidate
    // 검증, 최종 selected path commit(MERGE M2)은 여기 포함하지 않고
    // PlanInternal에 그대로 남아 있다 - 그 코드들은 이 함수 호출 앞뒤의
    // 별도 위치에서 이미 계산된 context_highway_zone 등을 그대로 쓴다.
    //
    // 반환값: 기존 "active highway zone missing; reset" 조기 실패 경로만
    // false로 전달한다(그 외엔 true) - PlanInternal이 그대로 return한다.
    //
    // context_highway_zone은 이 함수 호출 전에 PlanInternal이 이미
    // 계산해둔 값을 그대로 받는다 - 함수 안에서 재계산하면 그 값을 쓰는
    // locked-path replay 블록(M2, 이 함수보다 앞에서 실행됨)과 미세하게
    // 다른 시점의 값을 만들 substitution correctness 위험이 있다.
    bool UpdateLegacyMergeContext(
        double s, const CartesianState& ego, const FrenetState& start,
        const std::vector<ObjectInfo>& predicted_obstacles,
        const std::vector<TrackedObstacle>& predicted_collision_obstacles,
        const std::unordered_map<int, double>& object_yaw_rates,
        const HighwayMergeZone* context_highway_zone);

    // legacy MERGE COMMIT locked trajectory의 replay/emergency-recheck
    // (MERGE M2). 결과가 "성공 경로 반환"/"실패"/"처리할 것 없음(정상
    // planning 계속)" 세 갈래이므로 bool 하나로 뭉개지 않는다 - PlanInternal이
    // SUCCESS/FAILURE면 그 자리에서 즉시 return하고, CONTINUE면 원래처럼
    // 아래 정상 pipeline으로 흘러간다. 호출 위치(merge context locals 계산
    // 직후, v2 FOLLOW/AVOID context reset보다 앞)는 PlanInternal이 그대로
    // 유지한다 - 이미 잠긴 궤적이 있으면 매 cycle gap 판단을 다시 하지 않고
    // 그 경로부터 재검증하기 위함이다.
    enum class LockedMergeResult { CONTINUE, SUCCESS, FAILURE };
    LockedMergeResult HandleLockedMergeTrajectory(
        const CartesianState& ego, double s, double d,
        double context_conflict_s, double context_stop_buffer,
        const HighwayMergeZone* context_highway_zone,
        const std::vector<TrackedObstacle>& predicted_collision_obstacles,
        CartesianPath& out_path);

    // MERGE COMMIT 대기 중(commit_pending) candidate 각각을 conflict zone
    // 이탈점까지 연장한 뒤 fresh CTRV+OBB 충돌검사로 재검증한다(MERGE M2).
    // SelectBestPath보다 반드시 먼저 호출해야 한다 - best는 이 함수가 걸러낸
    // 뒤의 candidates를 가리키므로, 순서를 바꾸면 아직 검증되지 않은 후보가
    // 선택될 수 있다. 호출 gate(cmd.mode==MERGE && commit_pending)는
    // PlanInternal에 유지한다.
    void ValidatePendingMergeCommitCandidates(
        const PlannerCommand& cmd, double active_merge_completion_distance,
        double active_merge_cross_speed,
        const std::vector<TrackedObstacle>& predicted_collision_obstacles,
        std::vector<FrenetPath>& candidates, PlannerDebugStats& stats) const;

    // no-best-candidate 분기에서 대기 중이던 MERGE COMMIT을 되돌린다(MERGE
    // M2) - gap 판단은 통과했지만 실제 ego 궤적이 충돌검사에 실패한 경우다.
    // 호출 gate(cmd.mode==MERGE && commit_pending && s < cmd.merge_stop_s)는
    // PlanInternal에 유지한다 - COLLISION_BLOCKED/TRANSIENT_GENERATION_FAILURE
    // 공용 실패 분류나 diagnostics 로그와 섞이지 않게 하기 위함이며, 이
    // 함수는 rollback 로그와 merge_ 상태 리셋만 담당한다.
    void RollbackPendingMergeCommit(double s, const PlannerDebugStats& stats);

    // 선택된 best candidate의 Cartesian 궤적을 MERGE COMMIT으로 최종
    // 확정한다(MERGE M2) - conflict 이탈점까지 연장하고 gap_locked=true로
    // 전환, locked_path/locked_elapsed/locked_path_index를 저장한다. 이미
    // ConvertToCartesianPath로 변환되고(STOP reverse-trim까지 끝난)
    // selected_cartesian을 받는다 - 여기서 다시 변환하지 않는다.
    // merge_.locked_path에는 값을 복사한다(std::move 아님) - 호출부가 이후
    // 같은 selected_cartesian을 out_path로도 move해야 하기 때문이다. 호출
    // gate(cmd.mode==MERGE && commit_pending)는 PlanInternal에 유지한다.
    void CommitSelectedMergeTrajectory(
        const PlannerCommand& cmd, const FrenetPath& best, double s,
        double active_merge_completion_distance, double active_merge_cross_speed,
        CartesianPath& selected_cartesian);

    // FSM이 지정한 AVOID 대상의 확보 결과. FollowTargetStatus와 같은 세 값을
    // 쓰지만, 두 행동의 판정 기준이 앞으로 갈릴 수 있으므로 타입을 나눈다.
    enum class AvoidTargetStatus { VALID, GRACE, LOST };

    // FSM이 지정한 회피 대상을 확보한다. **selector가 아니라 resolver다.**
    //
    // FindStaticAvoidanceTarget()을 호출하지 않는다. 그 함수는
    //   모든 객체 탐색 -> 같은 차선 -> 저속/정지 -> 가장 가까운 것 ->
    //   좌/우 방향까지 자동 결정
    // 을 하는 legacy acquisition policy이며, v2에서 그 권한은 전부 FSM에 있다
    // (docs/d1-legacy-behavior-boundary.md 0.1절 B 분류, 6.4절).
    //
    // ===== 속도 조건을 쓰지 않는 이유 =====
    //
    // legacy에는 fabs(s_dot) <= avoid_cfg_.trigger_max_speed(0.5 m/s) 조건이
    // 있다. 그 config의 설명 자체가 "이 속도 이하만 정지 장애물로 간주(그
    // 이상은 Following이 처리)"이므로, 이것은 **AVOID인가 FOLLOW인가를
    // planner가 분류하는 policy**다.
    //
    // FSM이 AVOID object=15를 지정했는데 Frenet이 "15가 0.7 m/s니까 나는
    // FOLLOW로 보겠다"고 하면 행동 선택에 다시 개입하는 셈이다. D5에서
    // min_leader_speed를 제거한 것과 정확히 같은 이유다.
    //
    // 따라서 여기서 검증하는 것은 다음뿐이다:
    //   ID 일치 / 전방 / 같은 corridor / 탐색 범위 / geometry 정상
    //
    // 탐색거리는 legacy AVOID와 같은 방식으로 확장한다. config 값을 그대로
    // 쓰면 충돌 lookahead보다 짧아져, 장애물을 회피 대상으로 등록하기 전에
    // 충돌 필터가 중앙 후보를 모두 제거해 속도가 급락한다.
    AvoidTargetStatus ResolveExternalAvoidTarget(
        double ego_s, int target_object_id,
        const std::vector<ObjectInfo>& obstacles,
        AvoidanceContext& context) const;

    // v2 AVOID 대상의 유효 탐색거리 [m]. resolver와 진단 출력이 공유한다.
    double EffectiveAvoidDetectionDistance() const;

    // 확보한 대상에 대해 평가할 회피 목표 d를 정한다 (D6-2a).
    //
    // ===== 두 갈래 =====
    //
    //   target pose 있음 (has_avoid_target_d)
    //     -> 그 d가 authoritative lateral goal이다. FSM이 목표점을 준 것을
    //        Frenet이 geometry로 덮어쓰면 계약 3.3.3절 위반이다.
    //
    //   target pose 없음
    //     -> legacy의 검증된 geometry 계산을 재사용한다:
    //          clearance = 0.5*(자차폭 + 장애물폭) + safety_margin + 0.3
    //          left  = obstacle_d + clearance
    //          right = obstacle_d - clearance
    //        legacy가 begin_avoidance에서 쓰는 식과 동일하다. 고정 ±5 m
    //        (avoid_offset)가 아니라 장애물의 실제 횡위치를 반영하므로
    //        불필요하게 멀리 이동하지 않는다.
    //
    // ===== 방향 =====
    //
    //   LEFT   왼쪽 후보만 평가
    //   RIGHT  오른쪽 후보만 평가
    //   NONE   양쪽 평가 후 valid/cost로 Frenet이 선택
    //
    // legacy처럼 항상 양쪽을 만든 뒤 방향을 고르면, LEFT를 받았는데 RIGHT로
    // 가는 계약 위반이 된다(계약 3.3.1절).
    //
    // 이 함수는 **평가할 후보 목록만** 돌려준다. 어느 쪽이 실제로 안전한지는
    // D6-2b의 후보 생성/충돌 검사가 판정한다.
    struct AvoidTargetPlan {
        // 평가할 목표 d. count가 1이면 [0]만, 2이면 [0]=left, [1]=right.
        double candidates[2] = {0.0, 0.0};
        int count = 0;
        // 목표가 FSM의 target pose에서 왔는지. 진단·피드백용이다.
        bool from_target_pose = false;
        // 방향 요청과 target pose가 모순되는지 (예: LEFT인데 pose가 장애물
        // 오른쪽).
        //
        // **충돌 검사에만 맡기면 안 된다** (D6-2b). LEFT 요청에 오른쪽 -4 m
        // pose는 물리적으로 아주 안전할 수 있어 충돌 검사가 CLEAR를 준다.
        // 그대로 READY로 올라가면 FSM은 LEFT를 명령했는데 Frenet은 RIGHT
        // 후보를 준비한 계약 위반이 된다.
        //
        // 그래서 D6-2b가 이 값을 후보 차단 사유로 쓴다. 다만 명령 결함이
        // 아니므로 INVALID_COMMAND가 아니라 WAITING이다(계약 3.3.4절).
        bool direction_conflict = false;
    };
    AvoidTargetPlan DecideAvoidTargets(const AvoidanceContext& context,
                                       LateralDirection direction) const;


    // 현재 상태에서 그 자리에 정지하는 감속 궤적을 직접 만든다.
    //
    // 후보 생성 -> 곡률 필터 -> 충돌 필터 파이프라인을 타지 않는다. 두 곳에서
    // 공유한다:
    //   1. STOP BRAKING에서 후보가 0개인 경우의 fail-safe. 정지 목표가 아주
    //      가까우면 exact-stop quintic이 가속도 한계를 넘어 전부 탈락한다.
    //      여기서 실패를 반환하면 노드가 이전 궤적을 재사용해 정지 목표를
    //      지나쳐 계속 주행한다 - STOP에서 가장 위험한 실패 방향이다.
    //   2. 외부 EMERGENCY_STOP. BehaviorCommand는 목표점 없이
    //      (has_target_pose=false) 즉시 긴급정지를 지시하므로 정지 목표
    //      계산 소유권이 Frenet에 있다(계약 8.1절). 긴급정지에서 "후보가
    //      없어서 못 멈춤"이라는 실패 방향 자체를 제거하는 것이 목적이다.
    //
    // 프로파일: v(t) = max(0, v0 - decel*t), s는 사다리꼴 적분, d는 현재
    // 오프셋 유지(정지 직전이라 횡방향 기동을 하지 않는다).
    // 모든 샘플에서 v >= 0이고 마지막 샘플은 v = 0이다.
    bool GenerateDecelToZeroPath(double s, double d, double s_dot,
                                 CartesianPath& out_path) const;

    // 일반 저속 fallback의 크롤링 가속 프로파일 상수. 회전교차로 MERGE는
    // 이 고정 2 m/s 프로파일을 쓰지 않고 정상 MERGE 후보 생성기로 처리한다.
    static constexpr double kCrawlAccel = 0.5;       // [m/s^2] 완만한 가속
    static constexpr double kMinCreepSpeed = 0.3;    // [m/s] 최소 전진속도 (0 근처에 갇히지 않도록)
    static constexpr double kCrawlSpeedCap = 2.0;    // [m/s] 이 fallback 안에서의 속도 상한

    RefLine ref_;
    PathGeneratorConfig path_cfg_{};
    KinematicLimits limits_{};
    CostWeights cost_weights_{};
    VehicleShape vehicle_shape_{};
    CollisionCheckConfig collision_cfg_{};
    CurveSpeedConfig curve_speed_cfg_{};
    FollowingConfig following_cfg_{};
    AvoidConfig avoid_cfg_{};
    StopConfig stop_cfg_{};
    MergeConfig merge_cfg_{};
    HighwayMergeConfig highway_merge_cfg_{};
    PlannerVisualizationConfig visualization_cfg_{};
    std::unique_ptr<PlannerDebugWriter> debug_writer_;
    double wheelbase_ = 3.0;
    double lane_width_ = 3.5;
    // [s] 이 값은 caller(ROS node의 planning/frequency, legacy UDP loop)의
    // 실제 replanning 주기를 나타낸다. path_cfg_.dt(trajectory 샘플 간격)와는
    // 다른 개념이다 - 절대 섞어 쓰지 않는다
    // (fix/yuntae-frenet-speed-continuity Phase 2).
    double replanning_period_s_ = 0.05;
    // [m/s^2] target speed가 이전 cycle보다 증가하는 방향에만 적용하는
    // rate limit. 감속 방향은 curve braking envelope(Phase 1)가 이미
    // 안전하게 처리하므로 여기서 추가로 막지 않는다.
    double max_target_speed_accel_ = 2.0;
    bool ref_loaded_ = false;

    // 직전 사이클의 s (FindClosestS 탐색범위 힌트용 - ref_line.hpp 주석 참고,
    // 트랙이 자기 자신과 가까운 구간에서 최근접점이 엉뚱한 s로 튀는 문제 방지).
    double last_s_ = 0.0;
    bool has_last_s_ = false;
    double last_d_ = 0.0;
    double last_d_dot_ = 0.0;
    BehaviorState last_mode_ = LANE_KEEPING;
    double last_target_speed_ = 0.0;
    // 첫 cycle에는 last_target_speed_(초기값 0.0)를 기준으로 rate limit을
    // 걸면 안 된다 - 실제 ego 속도와 무관하게 0 근처로 묶인다. false인 동안은
    // resolved_target_speed의 raw 값을 그대로 쓴다
    // (fix/yuntae-frenet-speed-continuity Phase 2).
    bool has_target_speed_history_ = false;
    PlanFailureReason last_failure_reason_ = PlanFailureReason::NONE;

    // 모드 전환(LANE_KEEPING/FOLLOWING/AVOID) 시점만 로그로 찍기 위한 이전 사이클 값.
    BehaviorState prev_mode_ = LANE_KEEPING;
    MergePhase prev_merge_phase_ = MergePhase::CLEAR;
    AvoidanceContext avoidance_{};
    StopContext stop_{};
    // legacy(간이 FSM) 전용 FOLLOW 상태.
    FollowingContext following_{};
    // 외부 command(v2) 전용 FOLLOW 상태. struct와 dropout 로직은 재사용하되
    // 상태 인스턴스는 분리한다 - v2가 following_를 바꿔놓은 뒤 legacy 경로가
    // 다시 쓰이는 상황을 구조적으로 차단한다. D10 cutover에서 하나로 정리한다.
    FollowingContext v2_following_{};
    // 외부 command(v2) 전용 AVOID 상태 (D6-1). v2_following_와 같은 이유로
    // legacy avoidance_와 분리한다 - legacy fingerprint를 유지하려면 v2가
    // legacy context를 건드리지 않는 것이 가장 안전하다.
    AvoidanceContext v2_avoidance_{};

    // 이번 사이클에 평가할 v2 AVOID 목표 d 목록 (D6-2a). D6-2b의 후보
    // 생성기가 소비한다. 확보에 실패하면 count=0으로 비워진다.
    AvoidTargetPlan v2_avoid_targets_{};
    MergeContext merge_{};
    std::unordered_map<int, ObjectTrack> object_tracks_{};
};
