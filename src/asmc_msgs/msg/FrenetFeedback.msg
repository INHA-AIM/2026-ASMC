# ============================================================
# FrenetFeedback
# Frenet Planner → Behavior Planner
#
# 값이 없는 경우:
# - 객체 ID             : -1
# - 시간·비용·진행률     : -1.0
# - 문자열              : ""
# - 활성 maneuver 없음  : MANEUVER_NONE
# ============================================================


# ------------------------------------------------------------
# 기본 정보
# ------------------------------------------------------------

std_msgs/Header header
# 피드백 생성 시각과 좌표계
# header.frame_id = "map"

uint32 command_id
# 이 피드백이 대응하는 BehaviorCommand ID
# 처리 중인 명령이 없으면 0 사용 가능
# Behavior는 현재 command_id와 일치하는 피드백만 FSM에 반영


# ------------------------------------------------------------
# 명령 수용 상태
# ------------------------------------------------------------

bool command_accepted
# 명령의 형식과 필수 입력이 정상인지 여부
#
# true  : 명령 수용
# false : 명령 거부
#
# true라고 해서 반드시 경로 생성에 성공한 것은 아님

uint8 MANEUVER_NONE=255
# 현재 활성화된 maneuver가 없음

uint8 active_maneuver
# Frenet이 현재 처리 또는 실행 중인 maneuver
# BehaviorCommand의 maneuver_type과 동일한 숫자 사용
# 활성 maneuver가 없으면 MANEUVER_NONE


# ------------------------------------------------------------
# 경로 생성 결과
# ------------------------------------------------------------

bool plan_valid
# 이번 planning cycle에서 요청 행동에 대한
# 새로운 유효 후보 궤적 생성·선택에 성공했는지 여부
#
# 이전 경로 재사용 또는 안전 정지 경로 사용 시 false 가능

uint8 TRAJECTORY_NONE=0
# MPC에 보낼 수 있는 궤적이 없음

uint8 TRAJECTORY_NEW=1
# 이번 cycle에서 새로 생성한 궤적

uint8 TRAJECTORY_REUSED=2
# 일시적 생성 실패로 이전 유효 궤적 재사용

uint8 TRAJECTORY_LOCKED=3
# COMMIT된 잠금 궤적을 계속 실행

uint8 TRAJECTORY_SAFE_STOP=4
# 충돌·입력 이상·연속 실패로 안전 정지 궤적 사용

uint8 trajectory_source
# 현재 MPC로 전달한 궤적의 출처
# 궤적이 없으면 TRAJECTORY_NONE


# ------------------------------------------------------------
# Frenet 실행 상태
# ------------------------------------------------------------

uint8 IDLE=0
# 처리 중인 명령 없음

uint8 PREPARING=1
# 목표 투영, 객체 검색, 후보 생성 준비 중

uint8 WAITING=2
# Gap 또는 안전 후보가 없어 대기 중

uint8 READY=3
# 안전 후보가 확인되어 EXECUTE 요청 가능

uint8 COMMITTED=4
# 검증된 궤적을 잠근 상태

uint8 EXECUTING=5
# 잠긴 합류·차선 변경·회피 궤적 실행 중

uint8 COMPLETED=6
# Frenet 관점의 기하학적 행동 완료

uint8 FAILED=7
# 명령 수행 불가

uint8 execution_state
# Behavior FSM이 상태 전이에 사용하는 기준값


# ------------------------------------------------------------
# 행동 진행률
# ------------------------------------------------------------

float64 maneuver_progress
# 행동의 기하학적 진행률
#
# 0.0~1.0 : 유효한 진행률
# -1.0    : 진행률을 계산하지 않는 행동
#
# MERGE/LANE_CHANGE/AVOID에서 주로 사용
# KEEP/FOLLOW에서는 -1.0


# ------------------------------------------------------------
# 선택된 객체
# ------------------------------------------------------------

int32 active_leader_id
# FOLLOW 또는 MERGE 이후 실제 추종 중인 leader ID
# leader가 없으면 -1

int32 selected_front_object_id
# Frenet이 MERGE Gap의 앞 차량으로 선택한 ID
# MERGE가 아니거나 앞 차량이 없으면 -1

int32 selected_rear_object_id
# Frenet이 MERGE Gap의 뒤 차량으로 선택한 ID
# MERGE가 아니거나 뒤 차량이 없으면 -1


# ------------------------------------------------------------
# MERGE Gap 상태
# ------------------------------------------------------------

bool gap_evaluated
# MERGE Gap 안전성 평가를 실제로 수행했는지 여부
#
# false이면 gap_safe와 estimated_entry_time_s는 유효하지 않음

bool gap_safe
# gap_evaluated=true일 때 선택된 Gap의 안전 여부
# 평가하지 않았거나 안전하지 않으면 false

float64 estimated_entry_time_s
# 현재 Gap으로 conflict point에 진입할 예상 시간 [s]
#
# 유효한 값 : 0.0 이상
# MERGE가 아니거나 Gap이 없으면 -1.0


# ------------------------------------------------------------
# 현재 Frenet 상태
# ------------------------------------------------------------

bool frenet_state_valid
# current_s와 current_d의 유효 여부
# false이면 current_s와 current_d를 사용하지 않음

float64 current_s
# Frenet reference line 기준 Ego 종방향 위치 [m]
# frenet_state_valid=false이면 사용하지 않음
# Behavior가 목표 s를 만들기 위한 값으로 사용하지 않음

float64 current_d
# Frenet reference line 기준 Ego 횡방향 위치 [m]
# d는 실제로 음수가 될 수 있으므로 -1.0을 무효값으로 사용하지 않음
# frenet_state_valid=false이면 사용하지 않음


# ------------------------------------------------------------
# Frenet 목표값
# ------------------------------------------------------------

bool target_d_valid
# target_d가 현재 행동에서 유효한지 여부
# false이면 target_d를 사용하지 않음

float64 target_d
# 선택된 궤적의 목표 횡방향 오프셋 [m]
#
# LANE_CHANGE/AVOID : 실제 목표 d
# MERGE             : global path를 따르면 일반적으로 0.0
#
# target_d는 음수가 유효할 수 있으므로 -1.0을 무효값으로 사용하지 않음

float64 applied_speed_mps
# Frenet이 실제 적용한 목표속도 [m/s]
# Behavior 요청속도에 곡률·충돌·가속도 제한을 적용한 값
# 계산할 수 없으면 -1.0


# ------------------------------------------------------------
# 후보 궤적 진단
# ------------------------------------------------------------

uint32 generated_candidate_count
# 이번 cycle에서 생성한 전체 후보 수
# 후보를 생성하지 않았으면 0

uint32 valid_candidate_count
# 모든 필터를 통과한 유효 후보 수
# 유효 후보가 없으면 0

float64 selected_cost
# 최종 선택된 Frenet 궤적의 cost
# 선택된 궤적이 없으면 -1.0
# Behavior의 행동 선택 cost와는 별개의 값


# ------------------------------------------------------------
# 처리 결과 코드
# ------------------------------------------------------------

uint8 RESULT_NONE=0
# 오류 또는 특이사항 없음

uint8 NOT_INITIALIZED=1
# Frenet reference line 또는 Planner 초기화 실패

uint8 INVALID_COMMAND=2
# 행동별 필수 필드 누락 또는 잘못된 값
# 예: FOLLOW인데 primary_object_id=-1

uint8 STALE_COMMAND=3
# BehaviorCommand.valid_until이 지나 명령 만료

uint8 TARGET_PROJECTION_FAILED=4
# target_pose_map을 Frenet s,d로 변환하지 못함

uint8 TARGET_OBJECT_LOST=5
# FOLLOW leader 또는 AVOID 장애물을 찾지 못함

uint8 TRANSIENT_GENERATION_FAILURE=6
# 일시적으로 새 후보를 생성하지 못함
# 이전 경로를 제한적으로 재사용할 수 있음

uint8 CURVATURE_BLOCKED=7
# 후보가 곡률 또는 횡가속도 제한으로 모두 제거됨

uint8 COLLISION_BLOCKED=8
# 후보가 충돌 검사에서 모두 제거됨
# 일반적으로 안전 정지 궤적 사용

uint8 TOO_LATE_TO_CANCEL=9
# 이미 COMMIT된 행동에 CANCEL이 들어와 취소를 거부
# 잠긴 궤적은 계속 실행 가능

uint8 result_code
# 현재 명령 처리 또는 계획 결과
# 정상 상태이면 RESULT_NONE


# ------------------------------------------------------------
# 충돌 및 상세 정보
# ------------------------------------------------------------

int32 blocking_object_id
# 후보를 차단한 주요 충돌 객체 ID
# 특정 차단 객체가 없으면 -1

string result_detail
# 사람이 확인하기 위한 결과 설명
#
# 예:
# "all candidates rejected by object 17"
# "target pose projection failed"
# "cancel rejected after commit"
#
# 상세 내용이 없으면 ""
# Behavior 판단 로직에는 직접 사용하지 않음
