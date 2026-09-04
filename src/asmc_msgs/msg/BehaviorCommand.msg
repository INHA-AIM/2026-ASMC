# ============================================================
# BehaviorCommand
# Behavior Planner → Frenet Planner
#
# 좌표계 : map
# 속도   : m/s
# 시간   : ROS Time
#
# 값이 없는 경우:
# - 객체 ID      : -1
# - 선택 수치     : -1.0
# - 문자열       : ""
# - 목표 Pose    : has_target_pose=false
# ============================================================


# ------------------------------------------------------------
# 기본 정보
# ------------------------------------------------------------

std_msgs/Header header
# 명령 생성 시각과 좌표계
# header.frame_id = "map"

uint32 command_id
# 한 행동이 시작해서 완료·실패·취소될 때까지 유지되는 ID
# 같은 행동을 반복 발행할 때는 동일한 ID 사용
# 행동 종류나 목표가 변경되면 새로운 ID 사용


# ------------------------------------------------------------
# 행동 종류
# ------------------------------------------------------------

uint8 KEEP=0
# 현재 차선과 경로를 유지

uint8 FOLLOW=1
# primary_object_id로 지정한 선행 차량 추종

uint8 MERGE=2
# 회전교차로 또는 고속도로 합류

uint8 LANE_CHANGE=3
# 현재 도로 내 인접 차선으로 이동

uint8 AVOID=4
# primary_object_id로 지정한 장애물 회피

uint8 STOP=5
# target_pose_map에 계획 정지

uint8 EMERGENCY_STOP=6
# 목표점 없이 즉시 긴급정지

uint8 maneuver_type
# 현재 Frenet에 요청하는 행동
# 필수 필드이므로 정상 명령에서는 빈 값이 없어야 함


# ------------------------------------------------------------
# 실행 요청
# ------------------------------------------------------------

uint8 PREPARE=0
# 목표 투영, Gap 탐색, 후보 생성 및 안전성 검증만 수행
# 궤적 COMMIT은 허용하지 않음

uint8 EXECUTE=1
# 안전한 후보가 있으면 궤적 COMMIT 허용

uint8 CANCEL=2
# COMMIT 이전의 행동 요청 취소

uint8 execution_request
# Behavior FSM이 Frenet에 전달하는 실행 요청
#
# KEEP/FOLLOW/STOP/EMERGENCY_STOP : 일반적으로 EXECUTE
# MERGE/LANE_CHANGE/AVOID         : PREPARE → EXECUTE


# ------------------------------------------------------------
# 행동 방향
# ------------------------------------------------------------

uint8 DIRECTION_NONE=0
# 방향이 필요하지 않거나 Frenet이 방향을 선택

uint8 DIRECTION_LEFT=1
# 왼쪽 합류·차선 변경·회피

uint8 DIRECTION_RIGHT=2
# 오른쪽 합류·차선 변경·회피

uint8 direction
# MERGE/LANE_CHANGE/AVOID에서 사용
# 방향이 필요하지 않으면 DIRECTION_NONE


# ------------------------------------------------------------
# 합류 종류
# ------------------------------------------------------------

uint8 MERGE_NONE=0
# MERGE 행동이 아님

uint8 MERGE_ROUNDABOUT=1
# 회전교차로 진입

uint8 MERGE_HIGHWAY=2
# 진입로에서 고속도로 본선 합류

uint8 merge_type
# maneuver_type=MERGE일 때만 사용
# MERGE가 아니면 MERGE_NONE


# ------------------------------------------------------------
# MGeo 링크 정보
# ------------------------------------------------------------

string source_link_id
# 명령 생성 시점의 Ego 현재 링크 ID
# 값이 없으면 ""

string target_link_id
# MERGE/LANE_CHANGE 이후 진입할 목표 링크 ID
# 목표 링크가 필요하지 않은 행동이면 ""


# ------------------------------------------------------------
# 목표 위치
# ------------------------------------------------------------

bool has_target_pose
# target_pose_map의 유효 여부
#
# MERGE/LANE_CHANGE/STOP : 일반적으로 true
# KEEP/FOLLOW/EMERGENCY_STOP : 일반적으로 false

geometry_msgs/Pose target_pose_map
# map 좌표계의 목표 위치와 자세
#
# MERGE/LANE_CHANGE : 목표 링크 중심선상의 전방 목표점
# STOP              : 실제 정지 위치
# AVOID             : 필요하면 회피 목표점
#
# has_target_pose=false이면 이 값은 사용하지 않음


# ------------------------------------------------------------
# 목표속도
# ------------------------------------------------------------

float64 desired_speed_mps
# Behavior가 요청하는 상위 목표속도 [m/s]
#
# -1.0 : Frenet 설정의 기본 목표속도 사용
#  0.0 : 정지
#  양수 : 요청 목표속도
#
# Frenet은 곡률·충돌·가속도 제한에 따라 더 낮게 적용 가능


# ------------------------------------------------------------
# 행동 대상 객체
# ------------------------------------------------------------

int32 primary_object_id
# FOLLOW : 추종할 선행 차량 ID
# AVOID  : 회피할 장애물 ID
# 나머지 행동 또는 대상 없음 : -1

int32 preferred_front_object_id
# MERGE하려는 Gap의 앞 차량 ID
# Behavior가 지정하지 않으면 -1
# -1이면 Frenet이 Gap selector로 자동 선택

int32 preferred_rear_object_id
# MERGE하려는 Gap의 뒤 차량 ID
# Behavior가 지정하지 않으면 -1
# -1이면 Frenet이 Gap selector로 자동 선택


# ------------------------------------------------------------
# FOLLOW 간격 설정
# ------------------------------------------------------------

bool use_custom_follow_gap
# true  : 아래 시간 간격과 최소 거리 사용
# false : Frenet 설정 파일의 기본값 사용

float64 desired_time_gap_s
# 선행 차량과 유지할 목표 시간 간격 [s]
# use_custom_follow_gap=false이면 -1.0

float64 minimum_gap_m
# 저속·정지 상황에서도 유지할 최소 거리 [m]
# use_custom_follow_gap=false이면 -1.0


# ------------------------------------------------------------
# 명령 유효 시간
# ------------------------------------------------------------

time valid_until
# 명령이 유효한 마지막 ROS 시각
# 현재 ROS 시각이 이 값을 지나면 STALE_COMMAND
# 필수 필드이므로 -1 또는 빈 값 사용 금지


# ------------------------------------------------------------
# 디버그 정보
# ------------------------------------------------------------

string reason
# Behavior가 해당 행동을 선택한 이유
#
# 예:
# "route_merge_required"
# "slow_leader"
# "red_light"
# "static_obstacle"
#
# 이유가 없으면 ""
# Frenet의 경로 생성 계산에는 사용하지 않음
