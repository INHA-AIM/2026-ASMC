// Camera lanes define the road; LiDAR tracks alone define obstacle geometry/motion.
// 이 파일에 파라미터, ROS callback, 주행 함수, main을 함께 둔다.
#include "lane_boundary_adapter/boundary_family_adapter.hpp"
#include "lane_boundary_adapter/obstacle_memory.hpp"
#include "lane_boundary_adapter/dynamic_obstacle_risk.hpp"
#include "lane_boundary_adapter/decision_guard.hpp"

#include <ros/ros.h>
#include <lane_boundary_adapter/CameraBBoxLaneResult.h>
#include <std_msgs/Int8.h>
#include <asmc_msgs/LaneBoundaryFamily.h>
#include <asmc_msgs/control_handoff.hpp>
#include <asmc_msgs/LidarObjectArray.h>
#include <asmc_msgs/LidarObservation.h>
#include <ros/master.h>
#include <limits>
#include <map>
#include <sstream>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PointStamped.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using lane_boundary_adapter::BoundaryFamilyAdapterConfig;
using lane_boundary_adapter::BoundaryFamilyStripContext;
using lane_boundary_adapter::FamilyPoint2d;
using lane_boundary_adapter::SegmentSupport;
using BboxResult = lane_boundary_adapter::CameraBBoxLaneResult;

// ============================================================================
// 1. Parameters: 기본값 / 단위 / 설명 (동일 이름의 ROS 인자로 변경 가능)
// ============================================================================
struct Parameters {
    // 실행 모드와 목표점
    bool enable_control = false;               // true: control_topic 발행. false: preview/RViz만 발행.
    double lookahead_distance = 8.0;           // [m] base_link 전방 x 좌표. 곡선 길이/직선 반경이 아님.

    // 속도와 차량 조향 모델
    double target_speed_kph = 20.0;            // [km/h] 정상 추종 속도. MORAI 속도 제어 모드 사용.
    double inferred_speed_kph = 20.0;           // [km/h] 추정 보완 구간이 포함된 경로의 속도 상한.
    double wheelbase = 3.0;                    // [m] 차량 축거. MPC의 IONIQ 5 설정과 동일.
    double steering_curvature_gain = 0.844;    // 실측 보정: kappa=gain*tan(delta)/wheelbase.
    double max_steering_deg = 40.0;            // [deg] 조향 제한. CtrlCmd에는 rad로 발행.
    double max_steering_rate = 1.0;            // [rad/s] 추종 중 조향 명령 변화율 제한.
    double max_lateral_accel = 1.0;            // [m/s^2] 목표 곡률에 따라 속도를 낮추는 상한.

    // 제어 주기와 입력 유효 시간
    double control_rate = 20.0;                // [Hz] timeout 검사와 제어/preview 발행 주기.
    double source_timeout = 0.5;              // [s] ROS 현재 시각 - 원본 카메라 stamp 허용값.
    double receive_timeout = 0.35;             // [s] 새로운 stamp를 수신하지 못한 최대 시간(steady clock).
    double future_tolerance = 0.05;            // [s] 미래 source stamp 허용 오차. zero stamp는 항상 거부.

    // 현재 차로 선택과 경계 연결
    int lane_index = -1;                       // -1: 최초 자기 차로 자동 선택 후 고정. 0: 좌-중앙, 1: 중앙-우.
    int lane_confirm_frames = 3;               // 자동 선택 시 동일 차로가 연속 확인되어야 하는 새 frame 수.
    double lane_selection_distance = 4.0;      // [m] 이 전방 x에서 y=0이 포함되는 띠를 자기 차로로 선택.
    double lane_boundary_margin = 0.15;        // [m] 선택점에서 양 경계와 확보할 여유. 경계 위면 정지.
    bool allow_inferred = true;                // STRUCTURE_INFERRED 허용. INVALID는 항상 사용 금지.
    double min_confidence = 0.25;              // adapter effective_confidence 하한. 안전 확률은 아님.
    double join_tolerance = 1e-4;              // [m] support 조각 양 경계 끝점 일치 오차. gap 보간 금지.

    // 정지 동작과 표시 / 수치 설정
    double stop_brake = 1.0;                   // [0,1] 입력 누락/invalid/비상정지/종료 시 brake.
    double shutdown_stop_duration = 0.3;       // [s] SIGINT/SIGTERM 종료 직전 정지 명령 반복 시간.
    double marker_z = -0.30;                   // [m] RViz 표시 높이. 조향 계산과 무관.
    double centerline_marker_width = 0.22;     // [m] 인식된 두 차로 중심선 두께. 추종 경로보다 굵게 표시.
    double centerpoint_marker_size = 0.65;     // [m] 각 차로 lookahead 중심점 지름. 빨간 추종점과 구분.
    double centerlabel_marker_size = 0.45;     // [m] MY LANE / ADJACENT 텍스트 높이.
    double centerlabel_z_offset = 0.8;         // [m] 중심점 위 이름 표시 높이.
    double epsilon = 1e-6;                     // 보간·곡률·시간의 수치 비교/0 나눗셈 방지 허용오차.
    double max_steering_dt = 0.1;              // [s] 루프 지연 후 조향 급변 방지용 dt 상한.

    // 기존 boundary family adapter의 기하 검증 범위
    double consumer_x_min = 0.0;               // [m] adapter 전방 범위 시작.
    double consumer_x_max = 15.0;              // [m] adapter 전방 범위 끝. lookahead보다 커야 함.
    double min_segment_length = 0.25;          // [m] adapter 최소 연속 segment 길이.
    double lane_width_min = 2.4;               // [m] 허용 차로 폭 하한.
    double lane_width_max = 4.6;               // [m] 허용 차로 폭 상한.
    double max_width_variation = 0.8;          // [m] 한 segment 안에서 허용할 폭 변화.
    double inferred_confidence_scale = 0.60;   // 추정 신뢰도 강등 계수. producer 강등과 중복하지 않음.

    // 카메라 계약과 ROS 토픽
    // 인수한 v3 설정 해시.
    std::string expected_config_sha256 =
        "2d17cf63adbf2ba049d70656233ee2f34e9a4bfb43373ee76b3371aab1e4dc2a";
    // 차선 경계 입력(base_link).
    std::string input_topic = "/perception/lane/boundary_family";
    // MORAI UDP bridge 주행 명령.
    std::string control_topic = "/ctrl_cmd/jamming";
    // 계산 결과 확인.
    std::string preview_topic = "/planning/camera_bbox_lane_change/control_preview";
    // 유효한 lookahead 목표점.
    std::string target_topic = "/planning/camera_bbox_lane_change/target";
    // 인식된 내/옆 차로 중심선과 선택 경로/목표점 RViz.
    std::string markers_topic = "/planning/camera_bbox_lane_change/markers";
    // 정지 이유/지연.
    std::string diagnostics_topic = "/planning/camera_bbox_lane_change/diagnostics";
    // Bool true: 제동.
    std::string emergency_stop_topic = "/planning/camera_bbox_lane_change/emergency_stop";

    // Deprecated camera-debug arguments accepted for startup compatibility only.
    std::string bbox_topic = "/planning/camera_bbox_lane/result"; // 앞서 만든 판정 결과.
    std::string reference_lane_topic = "/planning/camera_bbox_lane_change/reference_lane"; // 변경 중 기준 차로 유지.
    double clipped_tracking_speed_kph = 5.0; // [km/h] 이미 확인한 장애물의 잘린 BBOX를 잠시 추적할 때 속도 상한.
    double far_obstacle_speed_kph = 5.0; // [km/h] 먼 BBOX의 차로 미확정 시 접근 속도 상한. 0이면 정지.
    double bbox_source_timeout = 0.5;      // [s] BBOX 원본 영상 나이 상한.
    double bbox_receive_timeout = 0.5;     // [s] 새 BBOX 판정 미수신 상한.
    bool dynamic_obstacle_stop = true;
    double dynamic_prediction_sec = 2.0;
    double dynamic_clear_sec = 1.0;
    double dynamic_acceleration_mps2 = 4.0; // prediction bound; paired with the configured pedal limit
    double dynamic_stop_speed_mps = 0.1;
    double dynamic_rest_sec = 0.25;
    double line_return_clear_sec = 1.0;
    double line_commit_sec = 1.0;
    double dynamic_uncertain_speed_kph = 3.0;
    double max_sensor_skew = 0.4;          // [s] 현재 차선과 BBOX/판정에 사용한 차선 stamp 차이.
    int obstacle_confirm_frames = 2;      // 서로 다른 observed LiDAR scans.
    int adjacent_clear_frames = 3;        // 옆 차로 NO_DETECTION 연속 확인 수. 실제 빈 공간 보장은 아님.
    bool ignore_bbox_during_lane_change = true; // 변경 중 BBOX 검출/UNKNOWN/유실에 의한 정지·감속 제외. 완료 즉시 복원.
    bool one_lane_change_only = false;
    bool return_to_entry_lane = false;    // 통합 재밍 주행: 양 차로 연속 미검출 후 진입 차로 복귀.
    double obstacle_lateral_margin = 0.30; // [m] 도로 경계 검사에 사용할 차량 좌우 여유.
    double obstacle_longitudinal_margin = 0.5; // [m] 차량 전후 여유. 장애물 거리 여유가 아님.
    double vehicle_front = 3.845;           // [m] base_link(후륜축)에서 앞범퍼까지.
    double vehicle_rear = 0.790;            // [m] base_link에서 뒤범퍼까지.
    double vehicle_width = 1.892;           // [m] 차량 폭.
    double planning_start_x = 1.0;          // [m] 근거리 관측 기준. 이 지점 접선으로 x=0까지만 보완.
    double path_horizon = 11.0;             // [m] 변경 경로 및 차량 footprint 검사 범위.
    double path_sample_step = 0.25;         // [m] 경로 / 충돌 검사 샘플 간격.
    double max_lateral_step = 2.4;          // [m] 한 재계획 경로의 횡이동 상한. 차량 앞모서리 이탈 방지.
    double lane_change_length = 10.0;       // [m] quintic 연결 경로가 이번 횡이동 목표에 도달하는 전방 거리.
    double lane_change_speed_kph = 15.0;     // [km/h] 변경 중 속도 상한. 곡률에 따라 추가 감속.
    double completion_lateral_error = 0.30; // [m] 목표 차로 근거리 중심선의 허용 횡오차.
    double completion_heading_deg = 8.0;    // [deg] 목표 차로 중심선과 차량 방향 허용 오차.
    int completion_confirm_frames = 3;     // 완료 판정에 필요한 서로 다른 camera frame 수.
    double lane_change_timeout = 15.0;     // [s] 변경 중 전진 명령을 낸 누적 시간 한도. 정지 대기 제외.
    double lane_change_pause_timeout = 30.0; // [s] 변경 중 연속 정지 대기 한도. 초과하면 재시작 필요.
    double lane_change_cooldown = 4.0;      // [s] 완료 후 재변경 금지 시간.

    // 중앙선 걸침 장애물 회피: BBOX 하단 접점의 지면 투영(평지 가정)으로 차로 중심을 민다.
    // 덜 덮인 차로를 선택하고 그 차로 중심을 장애물 반대쪽으로 이동한다. 측정/속도 입력이 없으면 기존 동작.
    bool straddle_avoidance = true;         // false: 기존 BOTH/옆 차로 검출 정지 동작만 사용.
    std::string ego_topic = "/Ego_topic";   // 통과 거리 적분용 차량 속도(morai EgoVehicleStatus, m/s).
    double ego_speed_timeout = 0.3;         // [s] 속도 미수신 상한. 회피 중 초과하면 정지.
    double avoid_clearance = 0.5;           // [m] 차체 측면과 장애물 하단 가장자리 사이 최소 간격.
    double avoid_speed_kph = 8.0;           // [km/h] 회피 중 속도 상한.
    // 인식·판단 정지(차선 끊김, 지날 선 없음 등) 대신 마지막 조향을 유지하고 이 속도로 기어간다. 0이면 기존처럼 정지.
    // 비상정지·입력 timeout·사람 정지·차속 없음, 그리고 앞 creep_clear_distance 안의 경로 장애물은 그대로 정지한다.
    double creep_speed_kph = 0.0;
    double creep_clear_distance = 2.0;      // [m] 앞 범퍼부터 이 거리까지 지금 조향의 경로에 장애물이 없어야 기어간다. 0 이하면 검사 안 함.
    double avoid_pass_margin = 2.0;         // [m] 장애물 끝 + 차량 후방을 지난 뒤 추가 이동 거리.
    double avoid_lane_tie = 0.10;           // [m] 두 차로 offset 차이가 이 값 이하면 내 차로 유지.
    double avoid_shift_length = 5.0;        // [m] 같은 차로 안 offset 연결 경로 길이.
    double avoid_lookahead_distance = 5.0;  // [m] 같은 차로 안 회피 중 추종점 전방 x.
    // 일반 구간 Frenet AVOID와 같이 회피는 내 차로 기준 횡 offset이다. 차선은 넘을 수 있고(페인트는 hard 한계
    // 아님) 주행 가능 영역(corridor)과 장애물 간격이 hard 한계다. false면 내 차로 차선 안쪽까지만 쓴다.
    bool avoid_line_straddle = true;
    double avoid_max_offset = 3.5;          // [m] 내 차로 중심 대비 최대 횡 offset(일반 구간 lateral_d1 ±3.0 대응).
    // 주행 가능 영역: 카메라 차선의 도로 바깥 끝. LiDAR가 그 바깥 벽/구조물을 확인한 쪽만 벽-여유까지,
    // 최대 avoid_max_shoulder만큼 넓힌다. 벽을 확인하지 못한 쪽은 차선 바깥을 쓰지 않는다.
    double avoid_max_shoulder = 2.0;
    double avoid_wall_margin = 0.3;         // [m] 벽과 차체 footprint(obstacle_lateral_margin 포함) 사이 추가 여유.
    // 벽을 확인하지 못한 쪽(카메라만 사용할 때 포함)에서 회피 중 차체가 도로 바깥 차선을 넘어도 되는 거리.
    // 기본 1.0m는 차량 중심이 선1·선5(도로 끝 차선)까지 갈 수 있는 값이다(차체 절반이 차선 밖).
    // 0이면 차체가 차선 안쪽 여유까지만.
    double avoid_line_overrun = 1.0;
    // 5개 기준선(1 왼쪽 끝, 2 왼쪽 차로 중앙, 3 중앙선, 4 오른쪽 차로 중앙, 5 오른쪽 끝) 사이를 이 간격으로 나눈
    // 후보 주행선 중 장애물 간격·주행 영역을 지키며 내 차로 중앙(2 또는 4)에 가장 가까운 선을 고른다.
    double avoid_line_step = 0.25;
    // [m] 선 선택에 쓰는 장애물의 최대 전방 거리. 한 선으로 먼 장애물까지 함께 피하려 하면 엇갈린 장애물
    // 사이 통로를 출발하지 못한다. 더 먼 장애물은 가까워지면 그때 다시 선을 고른다. 0 이하면 제한 없음.
    double line_obstacle_reach = 20.0;
    // 회피 선을 고른 뒤 같은 방향으로 더 밀어 여유를 늘린다. 0이면 기존처럼
    // 통과 가능한 가장 가까운 선(최소 횡이동)을 쓴다. 단위는 선 번호.
    double avoid_line_extra = 0.0;
    // [m] 기준선 1(좌)·5(우)를 도로 바깥으로 각각 확장한다. 선 좌표 매핑과 목표선
    // 범위만 넓히며, 실제 주행 한계는 corridorAt(차로 샘플 + overrun)이 그대로 정한다.
    double line_edge_expand = 0.0;
    // 선 선택 중 카메라 차선이 끊기면(차량이 크게 틀어진 회피 중 등) 마지막 정상 차선을 IMU 회전·차속 추측
    // 항법으로 옮겨 계속 쓴다. 거리·시간 중 하나라도 넘으면 정지한다. 0이면 사용 안 함.
    double lane_hold_distance = 15.0;       // [m] 장애물 옆을 지나는 동안(가림) 차선이 끊겨도 유지.
    double lane_hold_time = 8.0;            // [s] 회피 속도 8km/h에서 약 18m.
    std::string imu_topic = "/imu";
    double imu_timeout = 0.3;               // [s] IMU 미수신 상한. 넘으면 유지하지 않는다.

    // 종방향: /Ego_topic 실제 속도로 가속·브레이크 페달을 직접 계산한다(longlCmdType=1). MORAI 내장 속도
    // 제어(longlCmdType=2)는 저속 목표에서 페달을 거의 밟지 않고, 대회 규정 검토에서도 페달 출력으로 바꾸도록 했다.
    // 속도 입력이 없으면 내장 속도 제어로 대신한다.
    bool pedal_speed_control = true;
    double speed_kp = 0.3;                  // [pedal/(m/s)]
    double speed_ki = 0.1;                  // [pedal/m]
    double speed_integral_limit = 3.0;      // [m] 적분 상한(windup 방지).
    double max_accel_pedal = 0.5;           // [0,1]
    double max_brake_pedal = 0.5;           // [0,1] 주행 중 감속용. 정지는 stop_brake를 쓴다.
    double avoid_reconsider_distance = 0.5; // [m] 이 이동 전 회피 경로가 막히면 회피를 풀고 차로 선택을 다시 한다.
    double avoid_plan_margin = 0.05;        // [m] offset 판단에만 더하는 여유. 경로 검사(avoid_clearance)와 경계가 맞닿지 않게 한다.

    // LiDAR 차량 기준 OBB(Main enable_common_geometry:=true). 현재 카메라 차선으로 계산한 차도 영역과 겹치는
    // 물체만 회피 판단·경로 검사에 쓴다(터널 벽·기둥 제외). 길이 가정 대신 실측 길이를 쓴다.
    bool use_lidar_obstacles = true;       // Metric obstacle geometry source; independent camera lane following is retained.
    std::string lidar_objects_topic = "/perception/lidar/objects_local";
    double lidar_timeout = 0.6;             // [s] 원본 시각/수신 간격 상한. 초과하면 카메라만 사용.
    double lidar_max_x = 30.0;              // [m] 이보다 먼 물체는 이번 판단에서 제외.
    double lidar_min_height = 0.15;         // [m] 노면 잔여 점 제외.
    double lidar_max_length = 20.0;         // [m] 차도와 평행한 벽 조각 제외.
    int lidar_min_points = 5;
    double lidar_road_min_overlap = 0.20;   // [m] 차도 폭 방향으로 이만큼 겹쳐야 차도 위 물체로 본다.
    double lidar_lane_min_overlap = 0.10;   // [m] 내 차로와 이만큼 겹치면 내 차로 점유로 본다.
    double lidar_structure_hold = 1.5;      // [s] 벽 등 정적 구조물(주행 영역 한계)을 이동 보정해 유지하는 시간.
} params;

constexpr double PI = 3.14159265358979323846;
volatile std::sig_atomic_t shutdown_requested = 0;

// ============================================================================
// 2. Data / State: 차로 정보와 노드 상태
// ============================================================================
struct LaneChain {
    int lane_index = -1;
    std::vector<FamilyPoint2d> left_boundary;
    std::vector<FamilyPoint2d> right_boundary;
    std::vector<FamilyPoint2d> centerline;
    bool contains_inferred = false;
};

// 같은 전방 위치에서 보간한 중심점과 양쪽 경계점.
struct LaneSample {
    FamilyPoint2d center;
    FamilyPoint2d left;
    FamilyPoint2d right;
};

struct LaneCandidate {
    LaneChain lane;
    FamilyPoint2d target;
};

struct FollowerState {
    bool received_input = false;
    bool target_ready = false;
    bool lane_markers_valid = false;      // 표시 전용: 최신 차선 기하 검증 성공 여부.
    bool emergency_stop = false;

    int locked_lane = -1;                 // 내 차로: 변경 결정 즉시 옆 차로로 전환하고 유지.
    int pending_lane = -1;
    int confirmation_count = 0;
    std::string stop_reason = "waiting_for_lane";

    ros::Time source_stamp;                // 시간 검사를 통과한 마지막 입력 stamp.
    ros::Time observed_stamp;              // 거부된 입력도 포함한 마지막 stamp (지연 진단용).
    ros::SteadyTime received_at;           // 마지막 새 입력을 받은 실제 시각.
    ros::SteadyTime last_control_time;

    LaneChain selected_lane;
    FamilyPoint2d target_point;
    double last_steering = 0.0;             // 마지막 조향 명령 [rad]. 정지 중에는 유지.
} follower_state;

// ============================================================================
// LiDAR / Lane Change State: 차로 유지 -> 점유 확인 -> 변경 -> 새 차로 유지
// ============================================================================
struct LaneChangeState {
    std::vector<LaneChain> lanes;
    std::vector<FamilyPoint2d> path;
    int occupancy[2] = {BboxResult::UNOBSERVED, BboxResult::UNOBSERVED};
    unsigned long decision_sequence = 0;
    int obstacle_hits = 0;
    int clear_hits = 0;
    int target_lane = -1;
    int entry_lane = -1;
    bool returning_to_entry = false;
    ros::SteadyTime return_clear_since;
    bool changing = false;
    bool change_failed = false;
    std::string change_failure_reason;
    ros::SteadyTime timing_updated_at;
    double drive_time_sec = 0.0;          // 실제 속도 측정값이 아닌, 전진 명령을 유지한 시간.
    double pause_time_sec = 0.0;          // 연속 정지 명령 시간. 재개하면 0으로 초기화.
    bool last_command_driving = false;
    int completion_hits = 0;
    ros::Time completion_stamp;
    ros::SteadyTime change_started, completed_at;
    bool has_completed = false;
    double maximum_path_curvature = 0.0;
    // Bounds and XY footprint come exclusively from the LiDAR OBB corners.
    struct MeasuredObstacle {
        double x, x_far, y_min, y_max;
        bool lidar = true;
        int track_id = -1;
        std::string label{};
        lane_boundary_adapter::dynamic_risk::Motion motion{};
        bool predicted = false;
        std::vector<lane_boundary_adapter::dynamic_risk::Point> footprint{};
    };
} change_state;

// ============================================================================
// Straddle Avoidance State: 선택 차로 + offset, 통과 거리 적분
// ============================================================================
struct AvoidState {
    // x는 odom_at 시점 base_link 전방 거리. 횡위치는 측정 수단이 없으므로 회피 차로 중심 기준 상대값으로
    // 저장하고 매 주기 현재 차선 관측으로 되돌린다(차선변경 중 횡이동/방향 변화에도 장애물이 차로에 고정됨).
    struct Obstacle { double x, length, rel_min, rel_max, odom_at; };
    bool active = false;
    int lane = -1;                        // offset을 적용할 기하 차로 0/1.
    double offset = 0.0;                  // [m] 차로 중심 대비 목표 횡이동(+좌).
    double target_line = 0.0;             // 선택한 주행선(5개 기준선 좌표, 예: 2.0=왼쪽 차로 중앙, 3.0=중앙선).
    std::vector<Obstacle> obstacles;
    double odom = 0.0;                    // [m] 회피 시작 후 실제 속도 적분 이동 거리.
    int pending_lane = -1;
    int hits = 0;
} avoid_state;

// LiDAR 물체(차도 필터 전). 차도 판정은 차선이 바뀌므로 매 제어 주기에 한다.
struct LidarObstacles {
    bool received = false;
    ros::SteadyTime received_at;
    ros::Time stamp;
    unsigned long sequence = 0;
    std::string source{};
    double arrival_age = 0.0;            // [s] 도착 시점의 측정 지연(처리 지연 포함).
    std::vector<LaneChangeState::MeasuredObstacle> objects;     // 장애물 후보(길이·점 수 필터).
    std::vector<LaneChangeState::MeasuredObstacle> structures;  // 주행 가능 영역 한계(벽 등, 길이 무관).
    ros::Time structures_stamp;
    ros::SteadyTime structures_received_at;
    bool structures_received = false;
} lidar_state;

// One time-aligned LiDAR snapshot; road planning filters this same snapshot.
std::vector<LaneChangeState::MeasuredObstacle> current_obstacles, planning_obstacles;
bool planning_ready = false;
std::string path_block_reason;  // 마지막 경로 거부 이유(진단용).
std::string line_decision;      // 5개 기준선 판단 요약(진단용).
struct LineCandidateCheck {
    double line=0.,length_scale=0.,lateral_scale=0.;
    unsigned attempts=0;
    std::map<std::string,unsigned> rejected;
    std::string result="rejected",last_reason;
};
std::vector<LineCandidateCheck> line_candidate_checks;
double last_target_line = 0.0;
double last_adjacent_offset = 0.0;  // 선택한 주행선의 옆 차로 중앙 기준 offset(옆 차로 변경으로 바꿀 때 사용).

// ============================================================================
// Line Selection State: 1~5 기준선 중 따라갈 선(=내 차선) 선택
// ============================================================================
using LineObstacle = lane_boundary_adapter::obstacle_memory::Obstacle;

struct LineState {
    double target = 0.0;                  // 따라가는 선(1~5 좌표). 0이면 아직 정하지 않음.
    double home = 0.0;                    // 복귀할 차로 중앙(2 또는 4).
    double lane_home = 0.0;               // 지금 선에서 가까운 차로 중앙. 횡이동 상한(avoid_max_offset)의 기준.
    // return_to_entry_lane: 진입 차로 중앙(2 또는 4). 회피로 옆 차로에 가도 지난 뒤 여기로 돌아와
    // 재밍 종료 시 일반 플래너 기준 경로(lateral 범위) 안에서 넘겨준다. 0이면 사용 안 함.
    double return_home = 0.0;
    double odom = 0.0;                    // [m] 실제 속도 적분 이동 거리.
    std::vector<LineObstacle> obstacles;  // 선 좌표로 기억한 장애물(화면 밖에서도 유지).
    lane_boundary_adapter::decision::ClearEvidence clear;
    double changed_at=-1;
    double commitment_home=0;
    // A travel milestone only; never reused as obstacle/collision geometry.
    double pass_odom=0.;
    unsigned long lidar_sequence = 0;     // 마지막으로 반영한 LiDAR 스캔.
} line_state;
bool line_mode_active = false;

double speed_integral = 0.0;  // [m] 속도 PI 적분(정지 명령 때 초기화).

struct EgoSpeed {
    bool received = false;
    double speed = 0.0;                   // [m/s]
    ros::SteadyTime received_at;
} ego_speed;

// 차로 하나에 대한 회피 판정. kUnknown은 차선 관측으로 장애물 위치를 차로에 대응시킬 수 없다는 뜻이다.
enum class AvoidFit { kFits, kNoRoom, kUnknown };

struct AvoidPlan {
    int lane = -1;
    double offset = 0.0;
    bool decided = false;  // false: 측정을 차로에 대응시킬 수 없어 판단하지 않음(기존 동작). true+lane<0: 공간 없음.
};

ros::Subscriber mode_subscriber;
ros::Subscriber gps_return_subscriber;
ros::Subscriber ego_subscriber;
ros::Subscriber lidar_subscriber;
ros::Subscriber imu_subscriber;

// 추측 항법 자세(임의 원점, 시작 방향 0). 차선 유지(lane hold)에만 쓰는 상대 자세다.
struct DeadReckoning {
    double x = 0.0, y = 0.0, yaw = 0.0;
    bool imu_received = false;
    double yaw_rate = 0.0;                // [rad/s] +좌회전
    ros::SteadyTime imu_received_at;
} dead_reckoning;

// 마지막 정상 차선과 그때의 추측 항법 자세.
struct LaneHold {
    bool available = false;
    bool active = false;
    std::vector<LaneChain> lanes;
    double x = 0.0, y = 0.0, yaw = 0.0;
    ros::SteadyTime stored_at;
    double travelled = 0.0;               // [m] 저장 이후 이동 거리.
} lane_hold;
ros::Publisher reference_lane_publisher;
bool mode_active = false;
bool mode_received = false;
bool smooth_handoff=false,prepared_ready=false;
asmc_msgs::ControlHandoff handoff_request;
ros::SteadyTime handoff_received;
ros::Subscriber handoff_sub,applied_handoff_sub;
double applied_steering=0.;ros::SteadyTime applied_handoff_received;
ros::Publisher handoff_pub;
ros::Time ego_source_stamp;
bool preparingHandoff(){return smooth_handoff && handoff_request.valid && handoff_request.target_mode==1 &&
    (ros::SteadyTime::now()-handoff_received).toSec()<=.25;}

namespace dynamic_risk = lane_boundary_adapter::dynamic_risk;
dynamic_risk::MotionHistory dynamic_motion;
struct DynamicStopState {
    lane_boundary_adapter::decision::StopHold hold;
    bool crossing_yield=false;
    double crossing_until=-1;
    double resume_until=-1;
    std::string final_check="unvalidated";
    dynamic_risk::Risk risk;
    bool risk_checked=false;
    std::string decision="clear";
    unsigned known_motion=0,unknown_motion=0;
} dynamic_state;
std::string applyDynamicObstacleStop(morai_msgs::CtrlCmd& command,double dt,double previous_integral,bool creep);
std::string crossingYieldPlan(bool& handled);
std::vector<dynamic_risk::Object> riskObjects();
bool lineClear(double line,std::vector<FamilyPoint2d>& path);
bool lineObstacleBlocksHome(const LineObstacle& obstacle);
bool pathClearOfLineObstacles(const std::vector<FamilyPoint2d>& path);

bool ignoreOccupancyDuringLaneChange();
std::string updateLaneChangePlan();
const LaneChain* getPlanningLane(int lane_index);
bool samplePlanningLane(const LaneChain& lane, double x, LaneSample& sample);
bool buildPath(const LaneChain& lane, bool changing, std::vector<FamilyPoint2d>& path, double lateral_scale = 1.0,
               bool limit_lateral_step = true, double length_scale = 1.0);
bool pathClearOfAvoidObstacles(const std::vector<FamilyPoint2d>& path);
bool buildRoadBoundedChangePath(const LaneChain& lane, std::vector<FamilyPoint2d>& path);
bool pathInsideRoad(const std::vector<FamilyPoint2d>& path);
void storePlannedTarget(const LaneChain& lane, const std::vector<FamilyPoint2d>& path);
void validateLaneChangeParameters();
void modeCallback(const std_msgs::UInt8::ConstPtr& message);
void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& message);
bool egoSpeedFresh();
void lidarCallback(const asmc_msgs::LidarObjectArray::ConstPtr& message);
void lidarObservationCallback(const asmc_msgs::LidarObservation::ConstPtr& message);
void ingestLidar(const asmc_msgs::LidarObjectArray::ConstPtr& message,const std::string& source);
void imuCallback(const sensor_msgs::Imu::ConstPtr& message);
bool imuFresh();
bool applyLaneHold();
bool sampleLaneLinear(const LaneChain& lane, double x, LaneSample& sample);
bool lidarFresh();
bool roadIntervalAt(double x, double& left, double& right);
bool corridorAt(double x, double& left, double& right);
struct LineFrame { double y[6]; };  // y[1..5]: 5개 기준선의 base_link y(+좌).
bool lineFrameAt(double x, LineFrame& frame);
double lineY(const LineFrame& frame, double line);
double lineOf(const LineFrame& frame, double y);
AvoidFit lineGridAvoidOffset(const LaneChain& own, double& offset);
void refreshPlanningObstacles();
bool lidarOccupiesLane(const LaneChain& lane);
unsigned long observationSequence();
double avoidOffsetFor(int lane_index);
AvoidFit laneAvoidOffset(const LaneChain& lane, double& offset, bool allow_straddle = false);
AvoidPlan chooseAvoidPlan(const LaneChain& current, const LaneChain* adjacent);
void startAvoidance(const AvoidPlan& plan, const LaneChain& lane);
void rememberAvoidObstacles(const LaneChain& lane);
void rememberLineObstacles(bool new_scan);
bool avoidObstacleArea(const AvoidState::Obstacle& obstacle, std::array<FamilyPoint2d, 4>& area);
void updateAvoidanceProgress(double elapsed);
bool vehicleHitsAvoidObstacle(const FamilyPoint2d& point, double yaw, double front, double rear, double half_width);
std::string startLaneChange(int adjacent_index, const LaneChain& adjacent,
                            const std::vector<FamilyPoint2d>& path, const ros::SteadyTime& now);
std::string updateLinePlan(bool new_scan);
bool lineObstacleArea(const LineObstacle& obstacle, std::array<FamilyPoint2d, 4>& area);
bool lineObstacleConfirmed(const LineObstacle& obstacle);
double lineObstacleX(const LineObstacle& obstacle);
bool lineObstacleInReach(const LineObstacle& obstacle);


struct RosInterface {
    ros::Subscriber lane_sub;
    ros::Subscriber stop_sub;
    ros::Publisher control_pub;
    ros::Publisher preview_pub;
    ros::Publisher target_pub;
    ros::Publisher marker_pub;
    ros::Publisher diagnostic_pub;
} ros_interface;

BoundaryFamilyAdapterConfig adapter_config;

// ============================================================================
// 3. Forward declarations
// ============================================================================
void loadParameters(ros::NodeHandle& private_nh);
void validateParameters();
void initializeRosInterface(ros::NodeHandle& nh);
void handleShutdownSignal(int signal_number);

void cameraLaneFollowingProcess();
std::string getStopReason(const ros::SteadyTime& now);
double calculateTargetCurvature();
double calculateSteering(double curvature, double dt);
double calculateTargetSpeed(double curvature);
morai_msgs::CtrlCmd makeStopCommand();
morai_msgs::CtrlCmd makeFollowingCommand(double dt);
morai_msgs::CtrlCmd makeCreepCommand(double dt);
bool hardStopReason(const std::string& reason);
bool creepPathBlocked();
void publishControlCommand(const morai_msgs::CtrlCmd& command);
void publishShutdownStop();

void laneBoundaryCallback(const asmc_msgs::LaneBoundaryFamily::ConstPtr& msg);
void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);
void rejectLaneInput(const std::string& reason);
bool acceptInputTimestamp(const ros::Time& stamp);
std::vector<LaneChain> buildContinuousLanes(const BoundaryFamilyStripContext& context);
std::vector<LaneCandidate> findOwnLaneCandidates(const std::vector<LaneChain>& lanes);
bool confirmOwnLane(int lane_index);
double pointDistance(const FamilyPoint2d& first, const FamilyPoint2d& second);
FamilyPoint2d interpolatePoint(const FamilyPoint2d& first, const FamilyPoint2d& second, double ratio);
bool interpolateLaneAtX(const LaneChain& lane, double target_x, LaneSample& sample);

void publishTargetPoint();
void appendRecognizedLaneMarkers(visualization_msgs::MarkerArray& markers);
void publishLaneMarkers(bool tracking);
void publishDiagnostics(bool tracking, const std::string& reason, const morai_msgs::CtrlCmd& command);
void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status,
                        const std::string& key, const std::string& value);

}  // namespace

// ============================================================================
// 4. MAIN: ROS 초기화 -> 입력 수신 -> 차로 추종 -> 종료 시 정지
// ============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_bbox_lane_change", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, handleShutdownSignal);
    std::signal(SIGTERM, handleShutdownSignal);

    try {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        loadParameters(private_nh);
        validateParameters();
        validateLaneChangeParameters();
        initializeRosInterface(nh);

        ros::WallRate control_rate(params.control_rate);
        while (ros::ok() && !shutdown_requested) {
            ros::spinOnce();
            if (mode_active || preparingHandoff()) {
                cameraLaneFollowingProcess();
            } else {
                // Sensor callbacks still refresh observations. The control loop
                // owns every actuation-dependent lane-change transition/timer.
                publishDiagnostics(false, "inactive_mode", makeStopCommand());
            }
            control_rate.sleep();
        }

        if (ros::ok() && mode_active) {
            publishShutdownStop();
        }
        ros::shutdown();
    } catch (const std::exception& error) {
        ROS_FATAL("camera_lane_follower: %s", error.what());
        return 1;
    }

    return 0;
}

namespace {

void resetActuationLifecycle() {
    // A previous calculated command may never have reached the vehicle.
    // Re-select the physically observed lane on fresh camera frames.
    follower_state.locked_lane = -1;
    follower_state.pending_lane = -1;
    follower_state.confirmation_count = 0;
    follower_state.target_ready = false;
    follower_state.stop_reason = "confirming_own_lane";
    follower_state.selected_lane = LaneChain{};
    follower_state.target_point = FamilyPoint2d{};
    follower_state.last_steering = smooth_handoff && !applied_handoff_received.isZero() &&
      (ros::SteadyTime::now()-applied_handoff_received).toSec()<=.25?applied_steering:0.;
    follower_state.last_control_time = ros::SteadyTime::now();

    // Keep sensor observations; reset actuation state on activation.
    change_state.path.clear();
    change_state.decision_sequence = observationSequence();
    change_state.obstacle_hits = 0;
    change_state.clear_hits = 0;
    change_state.target_lane = -1;
    change_state.changing = false;
    change_state.change_failed = false;
    change_state.change_failure_reason.clear();
    change_state.timing_updated_at = follower_state.last_control_time;
    change_state.drive_time_sec = 0.0;
    change_state.pause_time_sec = 0.0;
    change_state.last_command_driving = false;
    change_state.completion_hits = 0;
    change_state.completion_stamp = ros::Time();
    change_state.change_started = ros::SteadyTime();
    change_state.completed_at = ros::SteadyTime();
    change_state.has_completed = false;
    change_state.entry_lane = -1;
    change_state.returning_to_entry = false;
    change_state.return_clear_since = ros::SteadyTime();
    change_state.maximum_path_curvature = 0.0;
    avoid_state = AvoidState{};
    line_state = LineState{};
    line_mode_active = false;
    dynamic_state=DynamicStopState{};
    dynamic_motion.clear();
    change_state.occupancy[0] = change_state.occupancy[1] = BboxResult::UNOBSERVED;
    std_msgs::Int8 automatic_lane;
    automatic_lane.data = -1;
    reference_lane_publisher.publish(automatic_lane);
}

// GpsModeSource: 재밍 원인이 사라졌고 일반 주행으로 넘기기 전(복귀 대기)이면 true.
bool gps_return = false;
void gpsReturnCallback(const std_msgs::Bool::ConstPtr& message) {
    if (!message || message->data == gps_return) return;
    gps_return = message->data;
    ROS_WARN("GPS return %s: %s", gps_return ? "started" : "cleared",
             gps_return ? "steer back to the entry lane before the normal handoff" : "normal rules");
}

void modeCallback(const std_msgs::UInt8::ConstPtr& message) {
    if (!message) return;
    mode_received = true;
    const bool next_active = message->data == 1;
    if (message->data != 0 && message->data != 1) {
        ROS_ERROR_THROTTLE(1.0, "Jamming node rejected unknown planning mode %u",
                           static_cast<unsigned>(message->data));
    }
    if (next_active == mode_active) return;
    mode_active = next_active;
    if (mode_active) {
        if(!(preparingHandoff()&&prepared_ready))resetActuationLifecycle();
    } else {
        std_msgs::Int8 automatic_lane;
        automatic_lane.data = -1;
        reference_lane_publisher.publish(automatic_lane);
    }
    ROS_WARN("Jamming control lifecycle %s", mode_active ? "ACTIVE" : "INACTIVE");
}

void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& message) {
    if (!message) return;
    // Integration PR128/PR127: CompetitionInfo supplies longitudinal x only.
    // Keep the validated main contract; do not integrate absent/lateral y into odometry or pedal control.
    const double speed = std::fabs(message->velocity.x);
    if (!std::isfinite(speed)) return;
    ego_source_stamp=message->header.stamp;
    ego_speed.speed = speed;
    ego_speed.received_at = ros::SteadyTime::now();
    ego_speed.received = true;
}

bool egoSpeedFresh() {
    return ego_speed.received &&
        (ros::SteadyTime::now() - ego_speed.received_at).toSec() <= params.ego_speed_timeout;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    if (!message || !std::isfinite(message->angular_velocity.z)) return;
    dead_reckoning.yaw_rate = message->angular_velocity.z;
    dead_reckoning.imu_received_at = ros::SteadyTime::now();
    dead_reckoning.imu_received = true;
}

bool imuFresh() {
    return dead_reckoning.imu_received &&
        (ros::SteadyTime::now() - dead_reckoning.imu_received_at).toSec() <= params.imu_timeout;
}

// 선 선택 중 차선이 끊기면 마지막 정상 차선을 지금 자세로 옮겨 쓴다. 조건을 벗어나면 false(정지).
bool applyLaneHold() {
    if (params.lane_hold_distance <= 0.0 || !lane_hold.available || !line_mode_active || line_state.target <= 0.0 ||
        !egoSpeedFresh() || !imuFresh() || lane_hold.travelled > params.lane_hold_distance ||
        (ros::SteadyTime::now() - lane_hold.stored_at).toSec() > params.lane_hold_time) {
        lane_hold.active = false;
        return false;
    }
    // 저장 당시 base_link 점 p → 지금 base_link: R(-dyaw)·(p - d), d는 저장 당시 좌표계로 본 이동량.
    const double c0 = std::cos(lane_hold.yaw), s0 = std::sin(lane_hold.yaw);
    const double wx = dead_reckoning.x - lane_hold.x, wy = dead_reckoning.y - lane_hold.y;
    const double dx = c0 * wx + s0 * wy, dy = -s0 * wx + c0 * wy;
    const double dyaw = dead_reckoning.yaw - lane_hold.yaw;
    const double c = std::cos(dyaw), s = std::sin(dyaw);
    const auto move = [&](std::vector<FamilyPoint2d>& points) {
        for (auto& point : points) {
            const double px = point.x - dx, py = point.y - dy;
            point = {c * px + s * py, -s * px + c * py};
        }
    };
    std::vector<LaneChain> lanes = lane_hold.lanes;
    for (auto& lane : lanes) {
        move(lane.left_boundary); move(lane.right_boundary); move(lane.centerline);
        // 보간은 x 증가 순서를 요구한다. 회전으로 순서가 깨진 점은 버린다.
        LaneChain ordered;
        ordered.lane_index = lane.lane_index;
        ordered.contains_inferred = true;
        for (std::size_t i = 0; i < lane.centerline.size(); ++i) {
            if (!ordered.centerline.empty() && lane.centerline[i].x <= ordered.centerline.back().x + params.epsilon) continue;
            ordered.centerline.push_back(lane.centerline[i]);
            ordered.left_boundary.push_back(lane.left_boundary[i]);
            ordered.right_boundary.push_back(lane.right_boundary[i]);
        }
        lane = ordered;
    }
    change_state.lanes = lanes;
    follower_state.target_ready = true;
    lane_hold.active = true;
    return true;
}

unsigned long observationSequence() {
    return lidar_state.sequence;
}

void lidarCallback(const asmc_msgs::LidarObjectArray::ConstPtr& message) {
    ingestLidar(message,"legacy_objects_local");
}
void lidarObservationCallback(const asmc_msgs::LidarObservation::ConstPtr& message) {
    if(!message||message->producer_session_id.empty())return;
    ingestLidar(asmc_msgs::LidarObjectArray::ConstPtr(message,&message->objects),
                message->producer_session_id+"/"+std::to_string(message->generation));
}
void ingestLidar(const asmc_msgs::LidarObjectArray::ConstPtr& message,const std::string& source) {
    if (!message) return;
    // 관측 결과만 사용한다. 입력 무효/pose 불가는 빈 관측이 아니다.
    if (message->header.frame_id != "base_link" ||
        (message->status != asmc_msgs::LidarObjectArray::OBSERVED &&
         message->status != asmc_msgs::LidarObjectArray::OBSERVED_EMPTY) ||
        message->header.stamp.isZero() || (!lidar_state.stamp.isZero() && source == lidar_state.source && message->header.stamp <= lidar_state.stamp)) {
        // 장애물 판단은 끊긴 것으로 본다. 정적 구조물은 lidar_structure_hold 동안 유지한다.
        lidar_state.received = false;
        lidar_state.objects.clear();
        return;
    }
    std::vector<LaneChangeState::MeasuredObstacle> objects, structures;
    const double behind = -(params.vehicle_rear + params.avoid_pass_margin);
    for (const auto& object : message->objects) {
        // Mirror current LiDAR observations. Accept prediction-only geometry if
        // supplied, without treating its invalid velocity as measured static.
        if(object.observation_state!=asmc_msgs::LidarObject::OBSERVED &&
           object.observation_state!=asmc_msgs::LidarObject::PREDICTED)continue;
        double x0 = std::numeric_limits<double>::infinity(), x1 = -x0, y0 = x0, y1 = -x0;
        bool finite = true;
        for (const auto& corner : object.obb_corners) {
            finite = finite && std::isfinite(corner.x) && std::isfinite(corner.y);
            x0 = std::min(x0, corner.x); x1 = std::max(x1, corner.x);
            y0 = std::min(y0, corner.y); y1 = std::max(y1, corner.y);
        }
        // 회전한 OBB의 축 정렬 외접 사각형(보수적). 이미 지나간 물체와 먼 물체는 제외한다.
        if (!finite || !std::isfinite(object.height)) continue;
        if (object.height < params.lidar_min_height || x1 < behind || x0 > params.lidar_max_x) continue;
        structures.push_back({x0, x1, y0, y1, true, object.track_id});
        if (object.length > params.lidar_max_length ||
            static_cast<int>(object.source_point_count) < params.lidar_min_points) continue;
        dynamic_risk::Motion motion;
        const bool predicted=object.observation_state==asmc_msgs::LidarObject::PREDICTED||object.miss_count;
        if(!predicted){
            std::vector<dynamic_risk::Point> shape;
            for(const auto& corner:object.obb_corners)shape.push_back({corner.x,corner.y});
            const dynamic_risk::Point measured_velocity{object.absolute_velocity_mps.x,object.absolute_velocity_mps.y};
            const bool velocity_valid=object.absolute_velocity_valid&&object.dynamic_valid&&
                std::isfinite(measured_velocity.x)&&std::isfinite(measured_velocity.y);
            // Feed valid producer motion into the same temporal evidence as
            // the GPS-denied fallback. Replacing velocity only after that
            // update left OBB-shape jitter in the stationary classification.
            motion=dynamic_motion.update(source,object.track_id,message->header.stamp.toSec(),
                {(x0+x1)/2.,(y0+y1)/2.},ego_speed.speed,dead_reckoning.yaw_rate,
                egoSpeedFresh()&&imuFresh(),velocity_valid?&measured_velocity:nullptr,&shape);
        }else motion=dynamic_motion.predict(source,object.track_id,message->header.stamp.toSec(),
            dead_reckoning.yaw_rate,egoSpeedFresh()&&imuFresh());
        LaneChangeState::MeasuredObstacle measured{x0,x1,y0,y1,true,object.track_id,object.camera_class,motion};
        measured.predicted=predicted;
        for(const auto& corner:object.obb_corners)measured.footprint.push_back({corner.x,corner.y});
        measured.footprint=dynamic_risk::hull(std::move(measured.footprint));
        objects.push_back(std::move(measured));
    }
    lidar_state.source=source;
    lidar_state.objects = objects;
    lidar_state.structures = structures;
    lidar_state.structures_stamp = message->header.stamp;
    lidar_state.structures_received_at = ros::SteadyTime::now();
    lidar_state.structures_received = true;
    lidar_state.arrival_age = (ros::Time::now() - message->header.stamp).toSec();
    lidar_state.stamp = message->header.stamp;
    lidar_state.received_at = ros::SteadyTime::now();
    lidar_state.received = true;
    lidar_state.sequence++;
}

// 도착 시점 지연과 수신 간격을 각각 제한한다. 현재 시각 기준 나이로 판단하면 처리 지연(~0.4s)+주기(~0.3s)가
// 겹치는 매 주기 끝마다 LiDAR가 끊긴 것처럼 보여 주행 영역이 흔들린다.
bool lidarFresh() {
    if (!params.use_lidar_obstacles || !lidar_state.received) return false;
    return (ros::SteadyTime::now() - lidar_state.received_at).toSec() <= params.lidar_timeout &&
        lidar_state.arrival_age >= -params.future_tolerance && lidar_state.arrival_age <= params.lidar_timeout;
}

// 관측 범위 안은 보간, 밖은 끝 1m 구간의 기울기로 직선 연장한다(비스듬한 자세에서 끝값 고정은 크게 틀림).
bool sampleLaneLinear(const LaneChain& lane, double x, LaneSample& sample) {
    if (lane.centerline.size() < 2) return false;
    const double front = lane.centerline.front().x, back = lane.centerline.back().x;
    if (x >= front && x <= back) return interpolateLaneAtX(lane, x, sample);
    const bool beyond = x > back;
    const double anchor = beyond ? back : front;
    const double other = beyond ? std::max(front, back - 1.0) : std::min(back, front + 1.0);
    LaneSample a, b;
    if (std::abs(other - anchor) < params.epsilon ||
        !interpolateLaneAtX(lane, anchor, a) || !interpolateLaneAtX(lane, other, b)) return false;
    const double ratio = (x - anchor) / (other - anchor);
    sample.center = interpolatePoint(a.center, b.center, ratio);
    sample.left = interpolatePoint(a.left, b.left, ratio);
    sample.right = interpolatePoint(a.right, b.right, ratio);
    return true;
}

// 현재 관측 차선 전체(두 차로)의 좌우 끝. 관측 범위 밖 x는 끝 구간 기울기로 연장한다.
bool roadIntervalAt(double x, double& left, double& right) {
    left = -std::numeric_limits<double>::infinity();
    right = std::numeric_limits<double>::infinity();
    for (const auto& lane : change_state.lanes) {
        if (lane.centerline.size() < 2) continue;
        LaneSample sample;
        if (!samplePlanningLane(lane, x, sample) && !sampleLaneLinear(lane, x, sample)) continue;
        left = std::max(left, sample.left.y);
        right = std::min(right, sample.right.y);
    }
    return std::isfinite(left) && std::isfinite(right) && left > right;
}

// 회피용 주행 가능 영역. 카메라 도로 끝을 기본으로, LiDAR가 그 바깥 구조물(벽·기둥·갓길 물체)을 확인한
// 쪽만 구조물-여유까지(최대 avoid_max_shoulder) 넓힌다. 구조물이 차선보다 안쪽이면 그쪽으로 좁힌다.
bool corridorAt(double x, double& left, double& right) {
    if (!roadIntervalAt(x, left, right)) return false;
    // 차량 모서리 검사는 obstacle_lateral_margin을 포함하므로 그만큼 더해 차체가 차선을 overrun까지 넘게 한다.
    const double overrun = params.avoid_line_overrun > 0.0 ?
        params.avoid_line_overrun + params.obstacle_lateral_margin : 0.0;
    const double lane_left = left, lane_right = right;
    left = lane_left + overrun;
    right = lane_right - overrun;
    if (!params.use_lidar_obstacles || !lidar_state.structures_received || params.avoid_max_shoulder <= 0.0 ||
        (ros::SteadyTime::now() - lidar_state.structures_received_at).toSec() > params.lidar_structure_hold) return true;
    // 정적 구조물은 측정 이후 이동 거리만큼 뒤로 옮겨 쓴다(처리 지연·잠깐의 수신 지연에도 영역 유지).
    const double travelled = egoSpeedFresh() ?
        ego_speed.speed * std::max(0.0, (ros::Time::now() - lidar_state.structures_stamp).toSec()) : 0.0;
    const double window = params.vehicle_front;  // 기둥 사이 틈을 벽 없음으로 보지 않도록 주변 x를 함께 본다.
    double left_wall = std::numeric_limits<double>::infinity();
    double right_wall = -std::numeric_limits<double>::infinity();
    for (const auto& structure : lidar_state.structures) {
        if (structure.x_far - travelled < x - window || structure.x - travelled > x + window) continue;
        if (structure.y_min >= lane_left - params.lidar_road_min_overlap) left_wall = std::min(left_wall, structure.y_min);
        if (structure.y_max <= lane_right + params.lidar_road_min_overlap) right_wall = std::max(right_wall, structure.y_max);
    }
    // 벽을 확인한 쪽은 overrun 대신 벽 기준(최대 avoid_max_shoulder)을 쓴다.
    if (std::isfinite(left_wall)) left = std::min(lane_left + params.avoid_max_shoulder, left_wall - params.avoid_wall_margin);
    if (std::isfinite(right_wall)) right = std::max(lane_right - params.avoid_max_shoulder, right_wall + params.avoid_wall_margin);
    return left > right;
}

// Correct a LiDAR scan once, then share it between planning, dynamic risk and RViz.
void refreshPlanningObstacles() {
    current_obstacles.clear();
    planning_obstacles.clear();
    planning_ready=lidarFresh();
    change_state.occupancy[0]=change_state.occupancy[1]=BboxResult::UNOBSERVED;
    if(!planning_ready)return; // An invalid/stale scan is never an empty observation.
    double road_left=0.,road_right=0.;
    planning_ready=roadIntervalAt(params.planning_start_x,road_left,road_right);
    const double age=std::max(0.,(ros::Time::now()-lidar_state.stamp).toSec());
    for(auto object:lidar_state.objects){
        dynamic_risk::Object aligned{{object.x,object.x_far,object.y_min,object.y_max},object.motion,object.track_id};
        aligned.footprint=object.footprint;
        aligned=dynamic_risk::alignToEgo(std::move(aligned),age,
            egoSpeedFresh()?ego_speed.speed:0.,imuFresh()?dead_reckoning.yaw_rate:0.);
        if(aligned.motion.extrapolated){
            aligned.motion.age+=age;
            if(aligned.motion.age>.5){aligned.motion.valid=false;aligned.motion.stationary=false;}
        }
        object.x=aligned.box.x0;object.x_far=aligned.box.x1;
        object.y_min=aligned.box.y0;object.y_max=aligned.box.y1;
        object.motion=aligned.motion;object.footprint=std::move(aligned.footprint);
        current_obstacles.push_back(object);
        double left=0.,right=0.;
        if(!roadIntervalAt(std::max(object.x,-params.vehicle_rear),left,right))continue;
        if(std::min(object.y_max,left)-std::max(object.y_min,right)<params.lidar_road_min_overlap)continue;
        planning_obstacles.push_back(std::move(object));
    }
    if(!planning_ready)return;
    for(const auto& lane:change_state.lanes){
        if(lane.lane_index<0||lane.lane_index>1)continue;
        change_state.occupancy[lane.lane_index]=lidarOccupiesLane(lane)?BboxResult::DETECTED:BboxResult::NO_DETECTION;
    }
}

// 차량 앞쪽 차도 위 LiDAR 물체가 해당 차로 폭과 lidar_lane_min_overlap 이상 겹치는지.
bool lidarOccupiesLane(const LaneChain& lane) {
    for (const auto& object : planning_obstacles) {
        if (!object.lidar || object.x_far < 0.0) continue;
        LaneSample sample;
        if (!samplePlanningLane(lane, std::max(object.x, params.planning_start_x), sample) &&
            !interpolateLaneAtX(lane, std::clamp(object.x, lane.centerline.front().x, lane.centerline.back().x), sample)) continue;
        if (std::min(object.y_max, sample.left.y) - std::max(object.y_min, sample.right.y) >= params.lidar_lane_min_overlap) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// 5. MAIN PROCESS: 한 주기의 차로 추종 흐름
// ============================================================================
void updateLaneChangeTiming(const ros::SteadyTime& now) {
    if (!change_state.changing || change_state.change_failed) return;
    const double elapsed = std::max(0.0, (now - change_state.timing_updated_at).toSec());
    change_state.timing_updated_at = now;
    if (change_state.last_command_driving) {
        change_state.drive_time_sec += elapsed;
        change_state.pause_time_sec = 0.0;
    } else {
        change_state.pause_time_sec += elapsed;
    }
    if (change_state.drive_time_sec > params.lane_change_timeout) {
        change_state.change_failure_reason = "lane_change_timeout_restart_required";
    } else if (change_state.pause_time_sec > params.lane_change_pause_timeout) {
        change_state.change_failure_reason = "lane_change_pause_timeout_restart_required";
    }
    change_state.change_failed = !change_state.change_failure_reason.empty();
}

bool ignoreOccupancyDuringLaneChange() {
    // A voluntary return always keeps live target-obstacle/freshness checks.
    // 회피 중에는 기억한 장애물 기하로 경로를 검사하므로 차선변경과 같은 제외 규칙을 쓴다.
    return params.ignore_bbox_during_lane_change &&
        ((change_state.changing && !change_state.returning_to_entry) || avoid_state.active ||
         (line_mode_active && !line_state.obstacles.empty()));
}

void cameraLaneFollowingProcess() {
    // 1. 제어 주기 계산: 루프가 늦어져도 조향 변화량이 갑자기 커지지 않도록 제한.
    const ros::SteadyTime now = ros::SteadyTime::now();
    const double elapsed = (now - follower_state.last_control_time).toSec();
    const double dt = std::clamp(elapsed, params.epsilon, params.max_steering_dt);
    follower_state.last_control_time = now;

    updateLaneChangeTiming(now);
    updateAvoidanceProgress(elapsed);
    if (egoSpeedFresh()) {
        const double step = std::clamp(elapsed, 0.0, 0.5);
        line_state.odom += ego_speed.speed * step;
        lane_hold.travelled += ego_speed.speed * step;
        dead_reckoning.yaw += (imuFresh() ? dead_reckoning.yaw_rate : 0.0) * step;
        dead_reckoning.x += ego_speed.speed * std::cos(dead_reckoning.yaw) * step;
        dead_reckoning.y += ego_speed.speed * std::sin(dead_reckoning.yaw) * step;
    }
    refreshPlanningObstacles();  // Keep every consumer on the same LiDAR snapshot, even while stopped.
    dynamic_state.hold.observeSpeed(ros::Time::now().toSec(),ego_speed.speed,egoSpeedFresh(),params.dynamic_stop_speed_mps);
    dynamic_state.crossing_yield=false;
    dynamic_state.final_check="unvalidated";
    dynamic_state.risk={};
    dynamic_state.risk_checked=false;
    dynamic_state.known_motion=dynamic_state.unknown_motion=0;
    dynamic_state.decision=dynamic_state.hold.active?"holding":"not_evaluated";
    line_candidate_checks.clear();
    if(planning_ready){
        rememberLineObstacles(false);
        if(avoid_state.active){
            const auto* lane=getPlanningLane(avoid_state.lane);
            if(lane)rememberAvoidObstacles(*lane);
        }
    }
    bool ahead_observation=false;
    for(const auto& obstacle:line_state.obstacles)
        ahead_observation|=lineObstacleConfirmed(obstacle)&&lineObstacleBlocksHome(obstacle);
    line_state.clear.observe(ros::Time::now().toSec(),lidar_state.sequence,lidar_state.source,
        planning_ready,!ahead_observation);
    path_block_reason.clear();
    if (!avoid_state.active && !line_mode_active) line_decision.clear();  // 회피·선 선택 중에는 판단을 계속 보인다.

    // 2. 입력 상태와 비상정지 확인. 추종할 수 없으면 기본 명령은 제동.
    std::string stop_reason = getStopReason(now);
    if (stop_reason.empty() && follower_state.target_ready) {
        // 정상 차선: 추측 항법 기준 자세와 함께 저장한다.
        lane_hold.available = true;
        lane_hold.active = false;
        lane_hold.lanes = change_state.lanes;
        lane_hold.x = dead_reckoning.x; lane_hold.y = dead_reckoning.y; lane_hold.yaw = dead_reckoning.yaw;
        lane_hold.stored_at = now;
        lane_hold.travelled = 0.0;
    } else if (!stop_reason.empty() && stop_reason != "emergency_stop" && applyLaneHold()) {
        stop_reason.clear();
    } else {
        lane_hold.active = false;
    }
    if (change_state.change_failed) stop_reason = change_state.change_failure_reason;
    if (stop_reason.empty() && follower_state.target_ready) {
        stop_reason = updateLaneChangePlan();
    } else {
        change_state.obstacle_hits = 0;
        change_state.clear_hits = 0;
        change_state.return_clear_since = ros::SteadyTime();
    }
    if (!stop_reason.empty()) change_state.completion_hits = 0;
    bool tracking = stop_reason.empty() && follower_state.target_ready;
    // 인식·판단 정지면 앞 경로가 비어 있는 한 마지막 조향으로 기어간다(creep_speed_kph).
    bool creep = !tracking && !stop_reason.empty() && params.creep_speed_kph > params.epsilon &&
        mode_active && !hardStopReason(stop_reason) && !creepPathBlocked();
    if (!tracking && !creep) speed_integral = 0.0;
    const double previous_integral=speed_integral;
    morai_msgs::CtrlCmd command = makeStopCommand();

    // 3. 유효한 목표점이 있을 때만 조향 / 속도를 계산.
    if (tracking) {
        command = makeFollowingCommand(dt);
    } else if (creep) {
        ROS_WARN_THROTTLE(2.0, "Camera lane follower CREEP %.1f km/h instead of stop: %s",
                          params.creep_speed_kph, stop_reason.c_str());
        command = makeCreepCommand(dt);
        stop_reason = "creep:" + stop_reason;
    }

    if(tracking){
        if(!pathClearOfLineObstacles(follower_state.selected_lane.centerline)){
            dynamic_state.final_check="blocked";
            stop_reason="dynamic_final_path_collision";
            command=makeStopCommand();tracking=false;creep=false;speed_integral=0.;
        }else{dynamic_state.final_check="passed";path_block_reason.clear();}
    }
    if(dynamic_state.hold.active&&!tracking&&!creep)dynamic_state.hold.clear.reset("no_valid_path");
    if(stop_reason.rfind("dynamic_",0)==0&&!tracking&&!creep){
        dynamic_state.hold.engage();dynamic_state.decision="stop";
    }
    if((params.dynamic_obstacle_stop||dynamic_state.hold.active)&&(tracking||creep)){
        const auto reason=applyDynamicObstacleStop(command,dt,previous_integral,creep);
        if(!reason.empty()){
            command=makeStopCommand();tracking=false;speed_integral=0.;stop_reason=reason;
        }
    }
    if(tracking)publishTargetPoint();
    change_state.last_command_driving = change_state.changing && tracking &&
                                        command.velocity > params.epsilon;

    // 목표 선택 직후 기준 차로도 전환한다. 점유는 항상 기하 차로 0/1로 변환해 사용한다.
    if (follower_state.locked_lane >= 0) {
        std_msgs::Int8 reference;
        reference.data = follower_state.locked_lane;
        reference_lane_publisher.publish(reference);
    }

    // 4. 제어 결과와 RViz / 진단 정보 발행.
    prepared_ready=tracking && egoSpeedFresh() && imuFresh() && lidarFresh();
    publishControlCommand(command);
    publishLaneMarkers(tracking);
    publishDiagnostics(tracking, stop_reason, command);
}

// ============================================================================
// 6. Pure Pursuit: 곡률 -> 조향각 -> 목표 속도
// ============================================================================
std::string getStopReason(const ros::SteadyTime& now) {
    if (follower_state.emergency_stop) {
        return "emergency_stop";
    }

    if (follower_state.received_input) {
        const double receive_age = (now - follower_state.received_at).toSec();
        if (receive_age > params.receive_timeout) {
            return "receive_timeout";
        }

        const double source_age = (ros::Time::now() - follower_state.source_stamp).toSec();
        if (source_age < -params.future_tolerance || source_age > params.source_timeout) {
            return "source_timestamp_expired";
        }
    }

    return follower_state.stop_reason;
}

double calculateTargetCurvature() {
    const double target_x = follower_state.target_point.x;
    const double target_y = follower_state.target_point.y;

    // base_link: x 전방, y 좌측. 양의 곡률 / 조향은 좌회전.
    return 2.0 * target_y / (target_x * target_x + target_y * target_y);
}

double calculateSteering(double curvature, double dt) {
    // 차량 모델: curvature = gain * tan(steering) / wheelbase.
    const double raw_steering = std::atan(params.wheelbase * curvature / params.steering_curvature_gain);
    const double steering_limit = params.max_steering_deg * PI / 180.0;
    const double desired_steering = std::clamp(raw_steering, -steering_limit, steering_limit);

    // 한 주기에 허용할 조향 변화량 [rad].
    const double change_limit = params.max_steering_rate * dt;
    const double steering_change = desired_steering - follower_state.last_steering;
    follower_state.last_steering += std::clamp(steering_change, -change_limit, change_limit);

    return follower_state.last_steering;
}

double calculateTargetSpeed(double curvature) {
    double target_speed = params.target_speed_kph;
    if (follower_state.selected_lane.contains_inferred) {
        target_speed = std::min(target_speed, params.inferred_speed_kph);
    }

    // 횡가속도 a = v² * |curvature|. 계산 결과 m/s를 km/h로 변환.
    if (std::abs(curvature) > params.epsilon) {
        const double curve_speed = std::sqrt(params.max_lateral_accel / std::abs(curvature)) * 3.6;
        target_speed = std::min(target_speed, curve_speed);
    }

    if (change_state.changing) {
        target_speed = std::min(target_speed, params.lane_change_speed_kph);
    }
    if (lane_hold.active || avoid_state.active || (line_mode_active &&
        (!line_state.obstacles.empty() || std::abs(line_state.target - line_state.home) > params.epsilon))) {
        target_speed = std::min(target_speed, params.avoid_speed_kph);
    }
    if (change_state.maximum_path_curvature > params.epsilon) {
        const double path_speed = std::sqrt(params.max_lateral_accel /
                                           change_state.maximum_path_curvature) * 3.6;
        target_speed = std::min(target_speed, path_speed);
    }
    return target_speed;
}

morai_msgs::CtrlCmd makeStopCommand() {
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;  // 페달 제어: accel 기본값 0, brake로 제동.
    command.brake = params.stop_brake;
    command.steering = follower_state.last_steering;
    return command;
}

// 목표 속도[km/h]를 페달 PI(차속 있을 때) 또는 MORAI 내장 속도 제어로 명령에 넣는다.
void applySpeedControl(morai_msgs::CtrlCmd& command, double target_speed, double dt) {
    command.velocity = target_speed;  // [km/h] 페달 제어에서도 진단용으로 목표를 남긴다.
    if (params.pedal_speed_control && egoSpeedFresh()) {
        // 페달 PI: 목표보다 느리면 가속, 빠르면 브레이크. 둘을 동시에 밟지 않는다.
        command.longlCmdType = 1;
        const double error = target_speed / 3.6 - ego_speed.speed;
        const double proposed_integral=std::clamp(speed_integral+error*dt,
            -params.speed_integral_limit,params.speed_integral_limit);
        const double proposed_effort=params.speed_kp*error+params.speed_ki*proposed_integral;
        // Saturated pedals cannot apply more effort. Do not accumulate an
        // integral that keeps accelerating after the target speed is reached.
        if(!((proposed_effort>params.max_accel_pedal&&error>0.)||
             (proposed_effort< -params.max_brake_pedal&&error<0.)))
            speed_integral=proposed_integral;
        const double effort = params.speed_kp * error + params.speed_ki * speed_integral;
        if (effort >= 0.0) command.accel = std::min(effort, params.max_accel_pedal);
        else command.brake = std::min(-effort, params.max_brake_pedal);
    } else {
        command.longlCmdType = 2;  // 속도 입력이 없을 때만 MORAI 내장 속도 제어.
    }
}

// 기어가기를 허용하지 않는 정지: 안전 입력·의도된 정지·차속 없음.
bool hardStopReason(const std::string& reason) {
    return reason == "emergency_stop" || reason == "inactive_mode" || reason.rfind("dynamic_",0)==0 ||
        reason.find("timeout") != std::string::npos || reason.find("expired") != std::string::npos ||
        reason.find("restart_required") != std::string::npos || !egoSpeedFresh();
}

// 마지막 조향의 원호를 따라 앞 범퍼부터 creep_clear_distance까지 차체 폭 안에 장애물이 있으면 막힘.
bool creepPathBlocked() {
    if (params.creep_clear_distance <= 0.0) return false;  // 장애물로는 멈추지 않는다(사람 정지는 hardStopReason).
    const double curvature = std::tan(follower_state.last_steering) * params.steering_curvature_gain / params.wheelbase;
    const double half_width = params.vehicle_width / 2.0 + params.obstacle_lateral_margin;
    const double reach = params.vehicle_front + params.creep_clear_distance;
    const auto hits = [&](double x0, double x1, double y_min, double y_max) {
        for (double x = 0.0; x <= reach + params.epsilon; x += 0.25) {
            if (x < x0 || x > x1) continue;
            const double y = 0.5 * curvature * x * x;
            if (y_min <= y + half_width && y_max >= y - half_width) return true;
        }
        return false;
    };
    for (const auto& obstacle : planning_obstacles) {
        if (hits(obstacle.x, obstacle.x_far, obstacle.y_min, obstacle.y_max)) return true;
    }
    LineFrame frame;
    for (const auto& obstacle : line_state.obstacles) {
        const double x0 = lineObstacleX(obstacle);
        if (!lineObstacleConfirmed(obstacle) || !lineFrameAt(std::max(x0, params.planning_start_x), frame)) continue;
        const double y_left = lineY(frame, obstacle.line_left), y_right = lineY(frame, obstacle.line_right);
        if (hits(x0, x0 + obstacle.length, std::min(y_left, y_right), std::max(y_left, y_right))) return true;
    }
    return false;
}

morai_msgs::CtrlCmd makeCreepCommand(double dt) {
    morai_msgs::CtrlCmd command;
    command.steering = follower_state.last_steering;
    applySpeedControl(command, params.creep_speed_kph, dt);
    return command;
}

morai_msgs::CtrlCmd makeFollowingCommand(double dt) {
    // 목표 속도가 0이면 제동 명령을 유지하고, 조향 계산 상태만 갱신한다.
    morai_msgs::CtrlCmd command = makeStopCommand();
    const double curvature = calculateTargetCurvature();
    const double steering = calculateSteering(curvature, dt);
    const double target_speed = calculateTargetSpeed(curvature);

    if (target_speed > 0.0) {
        command = morai_msgs::CtrlCmd();
        command.steering = steering;      // [rad], 기존 MPC / bridge와 동일.
        applySpeedControl(command, target_speed, dt);
    }

    return command;
}

void publishControlCommand(const morai_msgs::CtrlCmd& command) {
    ros_interface.preview_pub.publish(command);
    if(preparingHandoff()){
        asmc_msgs::ControlHandoff out;out.header.stamp=ros::Time::now();out.header.frame_id=ros::this_node::getName();
        out.epoch=handoff_request.epoch;out.target_mode=1;out.valid=prepared_ready;
        out.state_stamp=ego_source_stamp;out.trajectory_stamp=follower_state.source_stamp;
        out.state_frame="base_link";out.reason="camera_prepared_path";
        out.longl_cmd_type=command.longlCmdType;out.steering=command.steering;out.accel=command.accel;out.brake=command.brake;
        handoff_pub.publish(out);
    }

    if (params.enable_control && mode_active) {
        ros_interface.control_pub.publish(command);
    }
}

void publishShutdownStop() {
    const ros::SteadyTime deadline = ros::SteadyTime::now() + ros::WallDuration(params.shutdown_stop_duration);

    do {
        publishControlCommand(makeStopCommand());
        ros::WallDuration(1.0 / params.control_rate).sleep();
    } while (ros::ok() && ros::SteadyTime::now() < deadline);
}

// ============================================================================
// 7. Camera callback: 시간 검사 -> 경계 검증 -> 자기 차로 선택 -> 목표점 저장
// ============================================================================
void laneBoundaryCallback(const asmc_msgs::LaneBoundaryFamily::ConstPtr& msg) {
    // 1. 처음부터 거부된 입력도 실제 지연값을 진단에 표시한다.
    follower_state.observed_stamp = msg->header.stamp;
    if (!acceptInputTimestamp(msg->header.stamp)) {
        if (msg->header.stamp != follower_state.source_stamp) follower_state.lane_markers_valid = false;
        return;
    }
    follower_state.lane_markers_valid = false;

    // 2. 인수한 카메라 설정과 기하 계약 검사.
    // IMU family adoption belongs to planning: the classifier consumes the exact image-time
    // projection context, and both consumers pin the same approved config.
    // Do not bypass this gate to restore control after a camera default update.
    // See docs/handoffs/camera-to-planning.md, section 7.1.
    if (msg->config_sha256 != params.expected_config_sha256) {
        rejectLaneInput("wrong_geometry_config_hash");
        return;
    }

    const BoundaryFamilyStripContext context =
        lane_boundary_adapter::BuildBoundaryFamilyStripContext(*msg, adapter_config);
    if (!context.usable()) {
        rejectLaneInput(context.reason);
        return;
    }

    // 3. 유효한 경계 조각을 연결하고, 현재 차로부터 lookahead까지 이어지는 후보 선택.
    const std::vector<LaneChain> lanes = buildContinuousLanes(context);
    change_state.lanes = lanes;
    follower_state.lane_markers_valid = true;
    const std::vector<LaneCandidate> candidates = findOwnLaneCandidates(lanes);
    if (candidates.empty()) {
        rejectLaneInput("no_continuous_own_lane_to_lookahead");
        return;
    }
    if (candidates.size() != 1) {
        rejectLaneInput("ambiguous_own_lane");
        return;
    }

    // 4. 같은 차로가 연속으로 확인되면 고정. 이후 유실되어도 옆 차로로 바꾸지 않는다.
    const LaneCandidate& candidate = candidates.front();
    if (!change_state.changing && !confirmOwnLane(candidate.lane.lane_index)) {
        return;
    }

    follower_state.selected_lane = candidate.lane;
    follower_state.target_point = candidate.target;
    follower_state.target_ready = true;
    follower_state.stop_reason.clear();
}

void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
    follower_state.emergency_stop = msg->data;
}

void rejectLaneInput(const std::string& reason) {
    follower_state.target_ready = false;
    follower_state.stop_reason = reason;
    follower_state.pending_lane = -1;
    follower_state.confirmation_count = 0;

    // locked_lane은 유지: 자기 차로 유실을 차로 변경으로 처리하지 않는다.
}

bool acceptInputTimestamp(const ros::Time& stamp) {
    if (stamp.isZero()) {
        rejectLaneInput("zero_source_timestamp");
        return false;
    }

    // 미래 입력을 수락한 stamp로 기록하면 정상 입력으로 복귀하지 못하므로 먼저 검사.
    const double source_age = (ros::Time::now() - stamp).toSec();
    if (source_age < -params.future_tolerance || source_age > params.source_timeout) {
        rejectLaneInput("source_timestamp_expired");
        return false;
    }

    if (follower_state.received_input) {
        if (stamp == follower_state.source_stamp) {
            return false;  // 중복 입력은 수신 유효 시간을 갱신하지 않는다.
        }
        if (stamp < follower_state.source_stamp) {
            rejectLaneInput("source_timestamp_regression");
            return false;
        }
    }

    follower_state.received_input = true;
    follower_state.source_stamp = stamp;
    follower_state.received_at = ros::SteadyTime::now();
    return true;
}

bool confirmOwnLane(int lane_index) {
    if (follower_state.locked_lane >= 0) {
        return true;
    }

    if (follower_state.pending_lane == lane_index) {
        follower_state.confirmation_count++;
    } else {
        follower_state.confirmation_count = 1;
    }
    follower_state.pending_lane = lane_index;

    if (follower_state.confirmation_count < params.lane_confirm_frames) {
        follower_state.target_ready = false;
        follower_state.stop_reason = "confirming_own_lane";
        return false;
    }

    follower_state.locked_lane = lane_index;
    ROS_INFO("Locked geometric lane %d (0=left-centre, 1=centre-right)", lane_index);
    return true;
}

// ============================================================================
// 8. 차로 연결과 목표점 보간: INVALID gap을 건너뛰지 않는다.
// ============================================================================
std::vector<LaneChain> buildContinuousLanes(const BoundaryFamilyStripContext& context) {
    auto segments = context.segments;
    std::sort(segments.begin(), segments.end(), [](const auto& first, const auto& second) {
        if (first.lateral_index != second.lateral_index) {
            return first.lateral_index < second.lateral_index;
        }
        return first.centerline.front().x < second.centerline.front().x;
    });

    std::vector<LaneChain> lanes;
    bool may_join_previous = false;

    for (const auto& segment : segments) {
        const bool inferred = segment.support == SegmentSupport::kStructureInferred;
        if (segment.effective_confidence < params.min_confidence || (inferred && !params.allow_inferred)) {
            may_join_previous = false;
            continue;
        }

        // x가 순서대로 증가하는 구간만 x=lookahead에서 보간할 수 있다.
        bool increasing_x = true;
        for (std::size_t i = 1; i < segment.centerline.size(); ++i) {
            if (segment.centerline[i].x <= segment.centerline[i - 1].x + params.epsilon) {
                increasing_x = false;
                break;
            }
        }
        if (!increasing_x) {
            may_join_previous = false;
            continue;
        }

        // 같은 차로이며 양쪽 경계 끝점이 모두 맞닿은 경우에만 이어 붙인다.
        bool touching_previous = false;
        if (may_join_previous && !lanes.empty()) {
            const LaneChain& previous = lanes.back();
            const bool same_lane = previous.lane_index == static_cast<int>(segment.lateral_index);
            const double left_gap = pointDistance(previous.left_boundary.back(), segment.left_boundary.front());
            const double right_gap = pointDistance(previous.right_boundary.back(), segment.right_boundary.front());
            touching_previous = same_lane && left_gap <= params.join_tolerance && right_gap <= params.join_tolerance;
        }

        if (!touching_previous) {
            LaneChain lane;
            lane.lane_index = static_cast<int>(segment.lateral_index);
            lanes.push_back(lane);
        }

        LaneChain& lane = lanes.back();
        const std::size_t start_index = touching_previous ? 1 : 0;  // 겹친 끝점은 한 번만 저장.
        lane.left_boundary.insert(lane.left_boundary.end(),
                                  segment.left_boundary.begin() + start_index, segment.left_boundary.end());
        lane.right_boundary.insert(lane.right_boundary.end(),
                                   segment.right_boundary.begin() + start_index, segment.right_boundary.end());
        lane.centerline.insert(lane.centerline.end(),
                               segment.centerline.begin() + start_index, segment.centerline.end());
        lane.contains_inferred = lane.contains_inferred || inferred;
        may_join_previous = true;
    }

    return lanes;
}

std::vector<LaneCandidate> findOwnLaneCandidates(const std::vector<LaneChain>& lanes) {
    std::vector<LaneCandidate> candidates;

    for (const LaneChain& lane : lanes) {
        // 변경 결정 때 갱신한 내 차로의 중심선을 추종한다. 출발 차로를 재선택하지 않는다.
        const int tracking_lane = follower_state.locked_lane;
        if (tracking_lane >= 0 && lane.lane_index != tracking_lane) {
            continue;
        }

        LaneSample near_sample;
        LaneSample target_sample;
        if (!interpolateLaneAtX(lane, params.lane_selection_distance, near_sample)) {
            continue;
        }
        if (!interpolateLaneAtX(lane, params.lookahead_distance, target_sample)) {
            continue;
        }

        // 전방 선택 위치에서 차량 y=0이 양쪽 경계 안에 있어야 한다.
        // 최초 자기 차로만 y=0으로 선택. 이후에는 변경 결정으로 갱신한 내 차로를 유지.
        if (follower_state.locked_lane < 0 &&
            (near_sample.left.y < params.lane_boundary_margin ||
             near_sample.right.y > -params.lane_boundary_margin)) {
            continue;
        }

        candidates.push_back({lane, target_sample.center});
    }

    return candidates;
}

double pointDistance(const FamilyPoint2d& first, const FamilyPoint2d& second) {
    return std::hypot(first.x - second.x, first.y - second.y);
}

FamilyPoint2d interpolatePoint(const FamilyPoint2d& first, const FamilyPoint2d& second, double ratio) {
    FamilyPoint2d point;
    point.x = first.x + ratio * (second.x - first.x);
    point.y = first.y + ratio * (second.y - first.y);
    return point;
}

bool interpolateLaneAtX(const LaneChain& lane, double target_x, LaneSample& sample) {
    // 관측한 중심선 밖으로 외삽하지 않는다.
    if (lane.centerline.size() < 2) {
        return false;
    }
    if (target_x < lane.centerline.front().x - params.epsilon ||
        target_x > lane.centerline.back().x + params.epsilon) {
        return false;
    }

    for (std::size_t i = 1; i < lane.centerline.size(); ++i) {
        if (target_x > lane.centerline[i].x + params.epsilon) {
            continue;
        }

        const double segment_dx = lane.centerline[i].x - lane.centerline[i - 1].x;
        if (segment_dx <= params.epsilon) {
            return false;
        }

        const double ratio = std::clamp((target_x - lane.centerline[i - 1].x) / segment_dx, 0.0, 1.0);
        sample.center = interpolatePoint(lane.centerline[i - 1], lane.centerline[i], ratio);
        sample.left = interpolatePoint(lane.left_boundary[i - 1], lane.left_boundary[i], ratio);
        sample.right = interpolatePoint(lane.right_boundary[i - 1], lane.right_boundary[i], ratio);
        return true;
    }

    return false;
}

// ============================================================================
// 9. RViz / Diagnostics: 중심선, 목표점, 정지 이유 표시
// ============================================================================
void publishTargetPoint() {
    geometry_msgs::PointStamped target;
    target.header.frame_id = adapter_config.expected_frame_id;
    target.header.stamp = follower_state.source_stamp;
    target.point.x = follower_state.target_point.x;
    target.point.y = follower_state.target_point.y;
    ros_interface.target_pub.publish(target);
}

// 제어 가능 여부와 별개로 최신 관측 중심선을 표시한다. 끊긴 조각은 따로 그린다.
void appendRecognizedLaneMarkers(visualization_msgs::MarkerArray& markers) {
    if (!follower_state.lane_markers_valid || !follower_state.received_input) return;
    const double source_age = (ros::Time::now() - follower_state.source_stamp).toSec();
    const double receive_age = (ros::SteadyTime::now() - follower_state.received_at).toSec();
    if (source_age < -params.future_tolerance || source_age > params.source_timeout ||
        receive_age > params.receive_timeout) return;

    int segment_id = 0;
    for (const LaneChain& lane : change_state.lanes) {
        if (lane.centerline.size() < 2) continue;
        const int active_lane = follower_state.locked_lane;
        const bool assigned = active_lane >= 0;
        const bool own = assigned && lane.lane_index == active_lane;
        const std::string name = !assigned ? "unassigned_lane" : (own ? "own_lane" : "adjacent_lane");

        visualization_msgs::Marker line;
        line.header.frame_id = adapter_config.expected_frame_id;
        line.header.stamp = follower_state.source_stamp;
        line.ns = name + "_centerline";
        line.id = segment_id++;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = params.centerline_marker_width;
        line.lifetime = ros::Duration(params.receive_timeout);
        line.color.r = !assigned ? 0.65 : (own ? 0.1 : 1.0);
        line.color.g = !assigned ? 0.65 : (own ? 1.0 : 0.15);
        line.color.b = !assigned ? 0.65 : (own ? 0.15 : 1.0);
        line.color.a = 1.0;
        for (const auto& center : lane.centerline) {
            geometry_msgs::Point point;
            point.x = center.x;
            point.y = center.y;
            point.z = params.marker_z;
            line.points.push_back(point);
        }
        markers.markers.push_back(line);

        LaneSample center;
        const bool has_lookahead = interpolateLaneAtX(lane, params.lookahead_distance, center);
        visualization_msgs::Marker label = line;
        label.ns = name + "_label";
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.points.clear();
        label.pose.position = line.points[line.points.size() / 2];
        if (has_lookahead) {
            label.pose.position.x = center.center.x;
            label.pose.position.y = center.center.y;
        }
        label.pose.position.z += params.centerlabel_z_offset;
        label.scale.x = label.scale.y = 0.0;
        label.scale.z = params.centerlabel_marker_size;
        label.text = !assigned ? "UNASSIGNED" : (own ? (change_state.changing ? "MY LANE (CHANGING)" : "MY LANE") : "ADJACENT");
        label.text += " (" + std::to_string(lane.lane_index) + ")";
        markers.markers.push_back(label);

        // LD가 유효한 관측 조각 안에 있을 때만 점을 표시한다. gap을 보간하지 않는다.
        if (!has_lookahead) continue;
        visualization_msgs::Marker point = line;
        point.ns = name + "_centerpoint";
        point.type = visualization_msgs::Marker::SPHERE;
        point.points.clear();
        point.pose.position.x = center.center.x;
        point.pose.position.y = center.center.y;
        point.pose.position.z = params.marker_z;
        point.scale.x = point.scale.y = point.scale.z = params.centerpoint_marker_size;
        markers.markers.push_back(point);
    }
}

void publishLaneMarkers(bool tracking) {
    visualization_msgs::MarkerArray markers;

    // 이전 마커를 지우고 최신 관측 중심선과 유효한 추종 경로만 다시 표시한다.
    visualization_msgs::Marker clear;
    clear.header.frame_id = adapter_config.expected_frame_id;
    clear.header.stamp = ros::Time::now();
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    appendRecognizedLaneMarkers(markers);

    if (tracking) {
        visualization_msgs::Marker line;
        line.header.frame_id = adapter_config.expected_frame_id;
        line.header.stamp = follower_state.source_stamp;
        line.ns = "selected_centerline";
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.12;
        line.lifetime = ros::Duration(params.receive_timeout);
        line.color.g = 1.0;
        line.color.b = 1.0;
        line.color.a = 1.0;

        for (const FamilyPoint2d& center : follower_state.selected_lane.centerline) {
            geometry_msgs::Point point;
            point.x = center.x;
            point.y = center.y;
            point.z = params.marker_z;
            line.points.push_back(point);
        }
        markers.markers.push_back(line);

        // 실제 조향 목표: 내 차로 중심선의 x=lookahead_distance 지점 (원점 거리도 LD 이상).
        visualization_msgs::Marker waypoint = line;
        waypoint.ns = "lookahead_target";
        waypoint.id = 1;
        waypoint.type = visualization_msgs::Marker::SPHERE;
        waypoint.points.clear();
        waypoint.pose.position.x = follower_state.target_point.x;
        waypoint.pose.position.y = follower_state.target_point.y;
        waypoint.pose.position.z = params.marker_z;
        waypoint.scale.x = 0.5;
        waypoint.scale.y = 0.5;
        waypoint.scale.z = 0.5;
        waypoint.color.r = 1.0;
        waypoint.color.g = 0.0;
        waypoint.color.b = 0.0;  // RGB(1, 0, 0): 빨간 목표점.
        markers.markers.push_back(waypoint);
    }

    // 회피 경로 검사에 쓰는 장애물 가정 영역(하단 접점 + 길이 가정, 간격 포함 전 영역).
    int obstacle_id = 0;
    for (const auto& obstacle : avoid_state.obstacles) {
        std::array<FamilyPoint2d, 4> area;
        if (!avoidObstacleArea(obstacle, area)) continue;
        visualization_msgs::Marker box;
        box.header.frame_id = adapter_config.expected_frame_id;
        box.header.stamp = ros::Time::now();
        box.ns = "avoid_obstacle";
        box.id = obstacle_id++;
        box.type = visualization_msgs::Marker::LINE_STRIP;
        box.action = visualization_msgs::Marker::ADD;
        box.pose.orientation.w = 1.0;
        box.scale.x = 0.15;
        for (std::size_t corner = 0; corner <= area.size(); ++corner) {
            geometry_msgs::Point point;
            point.x = area[corner % area.size()].x;
            point.y = area[corner % area.size()].y;
            point.z = params.marker_z;
            box.points.push_back(point);
        }
        box.lifetime = ros::Duration(params.receive_timeout);
        box.color.r = 1.0;
        box.color.g = 0.45;
        box.color.a = 0.6;
        markers.markers.push_back(box);
    }

    // 5개 기준선(흰색, 번호 표시)과 회피 중 선택한 주행선(주황).
    std::vector<std::pair<double, LineFrame>> frames;
    for (double x = 0.0; x <= params.path_horizon + params.epsilon; x += 0.5) {
        LineFrame frame;
        if (lineFrameAt(x, frame)) frames.emplace_back(x, frame);
    }
    if (frames.size() >= 2) {
        std::vector<double> lines = {1.0, 2.0, 3.0, 4.0, 5.0};
        if (line_mode_active) lines.push_back(line_state.target);
        else if (avoid_state.active) lines.push_back(avoid_state.target_line);
        for (std::size_t index = 0; index < lines.size(); ++index) {
            const bool target = index == 5;
            visualization_msgs::Marker line;
            line.header.frame_id = adapter_config.expected_frame_id;
            line.header.stamp = ros::Time::now();
            line.ns = target ? "avoid_target_line" : "five_lines";
            line.id = static_cast<int>(index);
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = target ? 0.2 : 0.06;
            line.lifetime = ros::Duration(params.receive_timeout);
            line.color.r = 1.0;
            line.color.g = target ? 0.5 : 1.0;
            line.color.b = target ? 0.0 : 1.0;
            line.color.a = target ? 1.0 : 0.7;
            for (const auto& [x, frame] : frames) {
                geometry_msgs::Point point;
                point.x = x;
                point.y = lineY(frame, lines[index]);
                point.z = params.marker_z + 0.05;
                line.points.push_back(point);
            }
            markers.markers.push_back(line);
            visualization_msgs::Marker label = line;
            label.ns = target ? "avoid_target_line_label" : "five_line_labels";
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.points.clear();
            label.pose.position = line.points.back();
            label.pose.position.z += 0.5;
            label.scale.x = label.scale.y = 0.0;
            label.scale.z = 0.5;
            char text[32];
            std::snprintf(text, sizeof(text), target ? "TARGET %.2f" : "%.0f", lines[index]);
            label.text = text;
            markers.markers.push_back(label);
        }
    }

    // 선 선택에서 기억한 장애물 영역(빨강: 확인됨, 회색: 확인 중).
    for (const auto& obstacle : line_state.obstacles) {
        std::array<FamilyPoint2d, 4> area;
        if (!lineObstacleArea(obstacle, area)) continue;
        visualization_msgs::Marker outline;
        outline.header.frame_id = adapter_config.expected_frame_id;
        outline.header.stamp = ros::Time::now();
        outline.ns = "line_obstacle";
        outline.id = obstacle.track_id;
        outline.type = visualization_msgs::Marker::LINE_STRIP;
        outline.action = visualization_msgs::Marker::ADD;
        outline.pose.orientation.w = 1.0;
        outline.scale.x = 0.15;
        outline.lifetime = ros::Duration(params.receive_timeout);
        const bool confirmed = lineObstacleConfirmed(obstacle);
        outline.color.r = confirmed ? 1.0 : 0.6;
        outline.color.g = confirmed ? 0.1 : 0.6;
        outline.color.b = confirmed ? 0.1 : 0.6;
        outline.color.a = 0.9;
        for (std::size_t corner = 0; corner <= area.size(); ++corner) {
            geometry_msgs::Point point;
            point.x = area[corner % area.size()].x;
            point.y = area[corner % area.size()].y;
            point.z = params.marker_z;
            outline.points.push_back(point);
        }
        markers.markers.push_back(outline);
    }

    // The current physical LiDAR OBB footprint (blue); prediction-only tracks are grey.
    for (const auto& obstacle : planning_obstacles) {
        visualization_msgs::Marker box;
        box.header.frame_id = adapter_config.expected_frame_id;
        box.header.stamp = ros::Time::now();
        box.ns = "road_obstacle";
        box.id = obstacle.track_id;
        box.type = visualization_msgs::Marker::LINE_STRIP;
        box.action = visualization_msgs::Marker::ADD;
        box.pose.orientation.w = 1.0;
        box.scale.x = .10;
        for(std::size_t i=0;i<=obstacle.footprint.size()&&!obstacle.footprint.empty();++i){
            geometry_msgs::Point point;point.x=obstacle.footprint[i%obstacle.footprint.size()].x;
            point.y=obstacle.footprint[i%obstacle.footprint.size()].y;point.z=params.marker_z+.2;
            box.points.push_back(point);
        }
        box.lifetime = ros::Duration(params.receive_timeout);
        box.color.r = obstacle.predicted?.6:.1;
        box.color.g = obstacle.predicted?.6:.5;
        box.color.b = obstacle.predicted?.6:1.;
        box.color.a = .8;
        markers.markers.push_back(box);
    }

    ros_interface.marker_pub.publish(markers);
}

void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status,
                        const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
}

void publishDiagnostics(bool tracking, const std::string& reason, const morai_msgs::CtrlCmd& command) {
    double source_age = -1.0;
    if (!follower_state.observed_stamp.isZero()) {
        source_age = (ros::Time::now() - follower_state.observed_stamp).toSec();
    }

    diagnostic_msgs::DiagnosticStatus status;
    status.name = "camera_bbox_lane_change";
    status.hardware_id = "lidar_obb";
    status.level = tracking ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = tracking ? (change_state.changing ? "changing" : (avoid_state.active ? "avoiding" : "following")) : reason;
    addDiagnosticValue(status,"occupancy_stop_suppressed",ignoreOccupancyDuringLaneChange()?"true":"false");
    addDiagnosticValue(status,"obstacle_geometry_source","lidar_obb");
    addDiagnosticValue(status,"object_camera_geometry_used","false");
    addDiagnosticValue(status,"lidar_source",lidar_state.source);
    addDiagnosticValue(status,"lidar_snapshot_sequence",std::to_string(lidar_state.sequence));
    addDiagnosticValue(status,"dynamic_state",dynamic_state.decision);
    addDiagnosticValue(status,"dynamic_stop_hold",dynamic_state.hold.active?"true":"false");
    addDiagnosticValue(status,"dynamic_at_rest",dynamic_state.hold.resting(ros::Time::now().toSec(),params.dynamic_rest_sec)?"true":"false");
    addDiagnosticValue(status,"dynamic_safe_frames",std::to_string(dynamic_state.hold.clear.frames));
    addDiagnosticValue(status,"dynamic_clear_resets",std::to_string(dynamic_state.hold.clear.resets));
    addDiagnosticValue(status,"dynamic_clear_reset_reason",dynamic_state.hold.clear.reset_reason);
    addDiagnosticValue(status,"dynamic_risk_checked",dynamic_state.risk_checked?"true":"false");
    addDiagnosticValue(status,"resume_target_held",dynamic_state.hold.active&&line_mode_active?"true":"false");
    addDiagnosticValue(status,"crossing_yield",dynamic_state.crossing_yield?"true":"false");
    addDiagnosticValue(status,"final_path_check",dynamic_state.final_check);
    addDiagnosticValue(status,"line_clear_frames",std::to_string(line_state.clear.frames));
    addDiagnosticValue(status,"dynamic_blocking_id",std::to_string(dynamic_state.risk.id));
    addDiagnosticValue(status,"dynamic_ttc_sec",std::to_string(dynamic_state.risk.ttc));
    addDiagnosticValue(status,"dynamic_known_motion",std::to_string(dynamic_state.known_motion));
    addDiagnosticValue(status,"dynamic_unknown_motion",std::to_string(dynamic_state.unknown_motion));
    addDiagnosticValue(status,"line_return_remaining_m",std::to_string(std::max(0.,line_state.pass_odom-line_state.odom)));
    for(const auto& object:current_obstacles){
        std::ostringstream detail;
        detail<<"motion_valid="<<object.motion.valid<<" vx="<<object.motion.velocity.x
              <<" vy="<<object.motion.velocity.y<<" stationary="<<object.motion.stationary
              <<" extrapolated="<<object.motion.extrapolated<<" motion_age="<<object.motion.age
              <<" x="<<object.x<<" x_far="<<object.x_far
              <<" y_min="<<object.y_min<<" y_max="<<object.y_max;
        addDiagnosticValue(status,"risk_input/"+std::to_string(object.track_id),detail.str());
    }
    addDiagnosticValue(status,"line_candidate_count",std::to_string(line_candidate_checks.size()));
    for(std::size_t i=0;i<line_candidate_checks.size();++i){
        const auto& check=line_candidate_checks[i];
        std::ostringstream detail;
        detail<<"line="<<check.line<<" result="<<check.result<<" attempts="<<check.attempts
              <<" length_scale="<<check.length_scale<<" lateral_scale="<<check.lateral_scale;
        for(const auto& rejected:check.rejected)detail<<" "<<rejected.first<<"="<<rejected.second;
        if(!check.last_reason.empty())detail<<" last_reject="<<check.last_reason;
        addDiagnosticValue(status,"line_candidate/"+std::to_string(i),detail.str());
    }

    addDiagnosticValue(status, "enable_control", params.enable_control ? "true" : "false");
    addDiagnosticValue(status, "mode_received", mode_received ? "true" : "false");
    addDiagnosticValue(status, "mode_active", mode_active ? "true" : "false");
    addDiagnosticValue(status, "active_lane", std::to_string(follower_state.locked_lane));
    addDiagnosticValue(status, "locked_lane", std::to_string(follower_state.locked_lane));
    addDiagnosticValue(status, "entry_lane", std::to_string(change_state.entry_lane));
    addDiagnosticValue(status, "returning_to_entry", change_state.returning_to_entry ? "true" : "false");
    addDiagnosticValue(status, "gps_return", gps_return ? "true" : "false");
    addDiagnosticValue(status, "line_return_home", std::to_string(line_state.return_home));
    addDiagnosticValue(status, "source_age_sec", std::to_string(source_age));
    addDiagnosticValue(status, "lookahead_distance", std::to_string(params.lookahead_distance));
    addDiagnosticValue(status, "target_y_m", tracking ? std::to_string(follower_state.target_point.y) : "unavailable");
    addDiagnosticValue(status, "velocity_kph", std::to_string(command.velocity));
    addDiagnosticValue(status, "steering_rad", std::to_string(command.steering));
    addDiagnosticValue(status, "inferred", tracking && follower_state.selected_lane.contains_inferred ? "true" : "false");

    addDiagnosticValue(status, "change_state", change_state.changing ? "changing" : "following_lane");
    addDiagnosticValue(status, "target_lane", std::to_string(change_state.target_lane));
    addDiagnosticValue(status, "obstacle_input_valid", lidarFresh() ? "true" : "false");
    addDiagnosticValue(status, "obstacle_age_sec", lidar_state.received ?
        std::to_string((ros::Time::now() - lidar_state.stamp).toSec()) : "unavailable");
    addDiagnosticValue(status, "lane0_state", std::to_string(change_state.occupancy[0]));
    addDiagnosticValue(status, "lane1_state", std::to_string(change_state.occupancy[1]));
    addDiagnosticValue(status, "obstacle_confirmations", std::to_string(change_state.obstacle_hits));
    addDiagnosticValue(status, "adjacent_clear_confirmations", std::to_string(change_state.clear_hits));
    addDiagnosticValue(status, "change_completed", change_state.has_completed ? "true" : "false");
    addDiagnosticValue(status, "change_drive_time_sec", std::to_string(change_state.drive_time_sec));
    addDiagnosticValue(status, "change_pause_time_sec", std::to_string(change_state.pause_time_sec));
    addDiagnosticValue(status, "uses_obstacle_distance", "lidar_obb");
    addDiagnosticValue(status, "avoid_active", avoid_state.active ? "true" : "false");
    addDiagnosticValue(status, "avoid_lane", std::to_string(avoid_state.lane));
    addDiagnosticValue(status, "avoid_offset_m", std::to_string(avoid_state.offset));
    addDiagnosticValue(status, "avoid_travelled_m", std::to_string(avoid_state.odom));
    addDiagnosticValue(status, "avoid_obstacles", std::to_string(avoid_state.obstacles.size()));
    addDiagnosticValue(status, "avoid_confirmations", std::to_string(avoid_state.hits));
    addDiagnosticValue(status, "measured_obstacles", planning_ready ?
        std::to_string(planning_obstacles.size()) : "incomplete");
    addDiagnosticValue(status, "ego_speed_fresh", egoSpeedFresh() ? "true" : "false");
    addDiagnosticValue(status, "lidar_fresh", lidarFresh() ? "true" : "false");
    addDiagnosticValue(status, "path_block_reason", path_block_reason);
    addDiagnosticValue(status, "line_decision", line_decision);
    addDiagnosticValue(status, "line_mode", line_mode_active ? "true" : "false");
    addDiagnosticValue(status, "line_target", line_mode_active ? std::to_string(line_state.target) : "none");
    addDiagnosticValue(status, "line_obstacles", std::to_string(line_state.obstacles.size()));
    addDiagnosticValue(status, "lane_hold_active", lane_hold.active ? "true" : "false");
    addDiagnosticValue(status, "lane_hold_travelled_m", std::to_string(lane_hold.travelled));
    addDiagnosticValue(status, "imu_fresh", imuFresh() ? "true" : "false");
    addDiagnosticValue(status, "avoid_target_line", avoid_state.active ? std::to_string(avoid_state.target_line) : "none");
    double corridor_left = 0.0, corridor_right = 0.0;
    const bool corridor_ok = corridorAt(params.lookahead_distance, corridor_left, corridor_right);
    addDiagnosticValue(status, "corridor_at_lookahead_m", corridor_ok ?
        "[" + std::to_string(corridor_right) + "," + std::to_string(corridor_left) + "]" : "unavailable");
    addDiagnosticValue(status, "lidar_objects", std::to_string(current_obstacles.size()));
    addDiagnosticValue(status,"lidar_predicted_objects",std::to_string(std::count_if(current_obstacles.begin(),
        current_obstacles.end(),[](const auto& object){return object.predicted;})));
    addDiagnosticValue(status, "road_obstacles", std::to_string(std::count_if(planning_obstacles.begin(),
        planning_obstacles.end(), [](const auto& obstacle) { return obstacle.lidar; })));

    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header.stamp = ros::Time::now();
    diagnostics.status.push_back(status);
    ros_interface.diagnostic_pub.publish(diagnostics);

    if (!tracking && reason.rfind("creep:", 0) != 0) {
        ROS_WARN_THROTTLE(2.0, "Camera lane follower STOP: %s (source age=%.3fs)", reason.c_str(), source_age);
    }
}

// ============================================================================
// 10. ROS 초기화와 파라미터 로드 / 검증
// ============================================================================
void initializeRosInterface(ros::NodeHandle& nh) {
    adapter_config.consumer_x_min_m = params.consumer_x_min;
    adapter_config.consumer_x_max_m = params.consumer_x_max;
    adapter_config.min_segment_length_m = params.min_segment_length;
    adapter_config.lane_width_min_m = params.lane_width_min;
    adapter_config.lane_width_max_m = params.lane_width_max;
    adapter_config.max_width_variation_m = params.max_width_variation;
    adapter_config.inferred_confidence_scale = params.inferred_confidence_scale;

    std::string reason;
    if (!lane_boundary_adapter::ValidateBoundaryFamilyAdapterConfig(adapter_config, &reason)) {
        throw std::runtime_error(reason);
    }
    if (nh.resolveName(params.control_topic) == nh.resolveName(params.preview_topic)) {
        throw std::runtime_error("control_topic and preview_topic must differ");
    }

    reference_lane_publisher = nh.advertise<std_msgs::Int8>(params.reference_lane_topic, 1);
    follower_state.locked_lane = params.lane_index;
    ros_interface.preview_pub = nh.advertise<morai_msgs::CtrlCmd>(params.preview_topic, 1);
    if (params.enable_control) {
        ros_interface.control_pub = nh.advertise<morai_msgs::CtrlCmd>(params.control_topic, 1);
    }
    ros_interface.target_pub = nh.advertise<geometry_msgs::PointStamped>(params.target_topic, 1);
    ros_interface.marker_pub = nh.advertise<visualization_msgs::MarkerArray>(params.markers_topic, 1);
    ros_interface.diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(params.diagnostics_topic, 1);
    ros_interface.lane_sub = nh.subscribe(params.input_topic, 1, laneBoundaryCallback);
    ros_interface.stop_sub = nh.subscribe(params.emergency_stop_topic, 1, emergencyStopCallback);
    ros::NodeHandle("~").param("smooth_handoff",smooth_handoff,false);
    if(smooth_handoff){
        applied_handoff_sub=nh.subscribe<morai_msgs::CtrlCmd>("/ctrl_cmd",1,
          [&](const morai_msgs::CtrlCmd::ConstPtr& m){if(m->longlCmdType==1&&std::isfinite(m->steering)){
            applied_steering=m->steering;applied_handoff_received=ros::SteadyTime::now();}});
        handoff_pub=nh.advertise<asmc_msgs::ControlHandoff>("/planning/jamming/handoff_candidate",4);
        handoff_sub=nh.subscribe<asmc_msgs::ControlHandoff>("/planning/jamming/handoff_request",1,
            [&](const asmc_msgs::ControlHandoff::ConstPtr& m){
                if(m->epoch<handoff_request.epoch)return;
                if(m->epoch!=handoff_request.epoch){prepared_ready=false;if(!mode_active&&m->valid&&m->target_mode==1)resetActuationLifecycle();}
                handoff_request=*m;handoff_received=ros::SteadyTime::now();
            });
    }
    mode_subscriber = nh.subscribe("/planning/mode", 1, modeCallback);
    gps_return_subscriber = nh.subscribe("/planning/mode/gps_return", 1, gpsReturnCallback);
    if (params.straddle_avoidance || params.dynamic_obstacle_stop) ego_subscriber = nh.subscribe(params.ego_topic, 1, egoCallback);
    if ((params.straddle_avoidance && params.lane_hold_distance > 0.0) || params.dynamic_obstacle_stop) {
        imu_subscriber = nh.subscribe(params.imu_topic, 5, imuCallback);
    }
    if (params.use_lidar_obstacles) {
        // Prefer the existing atomic envelope (session/generation + track IDs).
        // Keep the array-only path for isolated fixtures and legacy producers.
        std::string envelope=params.lidar_objects_topic;
        const auto suffix=envelope.rfind("objects_local");
        bool available=false;
        if(suffix!=std::string::npos&&suffix+13==envelope.size()){
            envelope.replace(suffix,13,"observations_local");
            ros::master::V_TopicInfo topics;
            if(ros::master::getTopics(topics))for(const auto& topic:topics)
                if(topic.name==envelope&&topic.datatype=="asmc_msgs/LidarObservation")available=true;
        }
        if(available)lidar_subscriber=nh.subscribe(envelope,1,lidarObservationCallback);
        else lidar_subscriber=nh.subscribe(params.lidar_objects_topic,1,lidarCallback);
        ROS_INFO("Jamming obstacle memory input: %s",(available?envelope:params.lidar_objects_topic).c_str());
    }
    follower_state.last_control_time = ros::SteadyTime::now();

    ROS_WARN("Camera lane follower: control=%s lookahead_x=%.2fm speed=%.1fkm/h. "
             "Camera lane following with LiDAR obstacle geometry; camera fusion labels retained.",
             params.enable_control ? "ENABLED" : "preview only",
             params.lookahead_distance, params.target_speed_kph);
}

void handleShutdownSignal(int) {
    shutdown_requested = 1;
}

void loadParameters(ros::NodeHandle& private_nh) {
    private_nh.param("bbox_topic", params.bbox_topic, params.bbox_topic);
    private_nh.param("reference_lane_topic", params.reference_lane_topic, params.reference_lane_topic);
    private_nh.param("bbox_source_timeout", params.bbox_source_timeout, params.bbox_source_timeout);
    private_nh.param("bbox_receive_timeout", params.bbox_receive_timeout, params.bbox_receive_timeout);
    private_nh.param("dynamic_obstacle_stop",params.dynamic_obstacle_stop,params.dynamic_obstacle_stop);
    private_nh.param("dynamic_prediction_sec",params.dynamic_prediction_sec,params.dynamic_prediction_sec);
    private_nh.param("dynamic_clear_sec",params.dynamic_clear_sec,params.dynamic_clear_sec);
    private_nh.param("dynamic_acceleration_mps2",params.dynamic_acceleration_mps2,params.dynamic_acceleration_mps2);
    private_nh.param("dynamic_stop_speed_mps",params.dynamic_stop_speed_mps,params.dynamic_stop_speed_mps);
    private_nh.param("dynamic_rest_sec",params.dynamic_rest_sec,params.dynamic_rest_sec);
    private_nh.param("line_return_clear_sec",params.line_return_clear_sec,params.line_return_clear_sec);
    private_nh.param("line_commit_sec",params.line_commit_sec,params.line_commit_sec);
    private_nh.param("dynamic_uncertain_speed_kph",params.dynamic_uncertain_speed_kph,params.dynamic_uncertain_speed_kph);
    private_nh.param("ignore_bbox_during_lane_change", params.ignore_bbox_during_lane_change, params.ignore_bbox_during_lane_change);
    private_nh.param("one_lane_change_only", params.one_lane_change_only, params.one_lane_change_only);
    private_nh.param("return_to_entry_lane", params.return_to_entry_lane, params.return_to_entry_lane);
    private_nh.param("max_lateral_step", params.max_lateral_step, params.max_lateral_step);
    private_nh.param("max_sensor_skew", params.max_sensor_skew, params.max_sensor_skew);
    private_nh.param("obstacle_confirm_frames", params.obstacle_confirm_frames, params.obstacle_confirm_frames);
    private_nh.param("adjacent_clear_frames", params.adjacent_clear_frames, params.adjacent_clear_frames);
    private_nh.param("obstacle_lateral_margin", params.obstacle_lateral_margin, params.obstacle_lateral_margin);
    private_nh.param("obstacle_longitudinal_margin", params.obstacle_longitudinal_margin, params.obstacle_longitudinal_margin);
    private_nh.param("vehicle_front", params.vehicle_front, params.vehicle_front);
    private_nh.param("vehicle_rear", params.vehicle_rear, params.vehicle_rear);
    private_nh.param("vehicle_width", params.vehicle_width, params.vehicle_width);
    private_nh.param("planning_start_x", params.planning_start_x, params.planning_start_x);
    private_nh.param("path_horizon", params.path_horizon, params.path_horizon);
    private_nh.param("path_sample_step", params.path_sample_step, params.path_sample_step);
    private_nh.param("lane_change_length", params.lane_change_length, params.lane_change_length);
    private_nh.param("lane_change_speed_kph", params.lane_change_speed_kph, params.lane_change_speed_kph);
    private_nh.param("clipped_tracking_speed_kph", params.clipped_tracking_speed_kph, params.clipped_tracking_speed_kph);
    private_nh.param("far_obstacle_speed_kph", params.far_obstacle_speed_kph, params.far_obstacle_speed_kph);
    private_nh.param("completion_lateral_error", params.completion_lateral_error, params.completion_lateral_error);
    private_nh.param("completion_heading_deg", params.completion_heading_deg, params.completion_heading_deg);
    private_nh.param("completion_confirm_frames", params.completion_confirm_frames, params.completion_confirm_frames);
    private_nh.param("lane_change_timeout", params.lane_change_timeout, params.lane_change_timeout);
    private_nh.param("lane_change_pause_timeout", params.lane_change_pause_timeout, params.lane_change_pause_timeout);
    private_nh.param("lane_change_cooldown", params.lane_change_cooldown, params.lane_change_cooldown);
    private_nh.param("straddle_avoidance", params.straddle_avoidance, params.straddle_avoidance);
    private_nh.param("ego_topic", params.ego_topic, params.ego_topic);
    private_nh.param("ego_speed_timeout", params.ego_speed_timeout, params.ego_speed_timeout);
    private_nh.param("avoid_clearance", params.avoid_clearance, params.avoid_clearance);
    private_nh.param("avoid_speed_kph", params.avoid_speed_kph, params.avoid_speed_kph);
    private_nh.param("creep_speed_kph", params.creep_speed_kph, params.creep_speed_kph);
    private_nh.param("creep_clear_distance", params.creep_clear_distance, params.creep_clear_distance);
    private_nh.param("avoid_pass_margin", params.avoid_pass_margin, params.avoid_pass_margin);
    private_nh.param("avoid_lane_tie", params.avoid_lane_tie, params.avoid_lane_tie);
    private_nh.param("avoid_shift_length", params.avoid_shift_length, params.avoid_shift_length);
    private_nh.param("avoid_lookahead_distance", params.avoid_lookahead_distance, params.avoid_lookahead_distance);
    private_nh.param("avoid_line_straddle", params.avoid_line_straddle, params.avoid_line_straddle);
    private_nh.param("avoid_max_offset", params.avoid_max_offset, params.avoid_max_offset);
    private_nh.param("avoid_max_shoulder", params.avoid_max_shoulder, params.avoid_max_shoulder);
    private_nh.param("avoid_wall_margin", params.avoid_wall_margin, params.avoid_wall_margin);
    private_nh.param("avoid_line_overrun", params.avoid_line_overrun, params.avoid_line_overrun);
    private_nh.param("avoid_line_step", params.avoid_line_step, params.avoid_line_step);
    private_nh.param("line_obstacle_reach", params.line_obstacle_reach, params.line_obstacle_reach);
    private_nh.param("avoid_line_extra", params.avoid_line_extra, params.avoid_line_extra);
    private_nh.param("line_edge_expand", params.line_edge_expand, params.line_edge_expand);
    private_nh.param("lane_hold_distance", params.lane_hold_distance, params.lane_hold_distance);
    private_nh.param("lane_hold_time", params.lane_hold_time, params.lane_hold_time);
    private_nh.param("imu_topic", params.imu_topic, params.imu_topic);
    private_nh.param("imu_timeout", params.imu_timeout, params.imu_timeout);
    private_nh.param("pedal_speed_control", params.pedal_speed_control, params.pedal_speed_control);
    private_nh.param("speed_kp", params.speed_kp, params.speed_kp);
    private_nh.param("speed_ki", params.speed_ki, params.speed_ki);
    private_nh.param("speed_integral_limit", params.speed_integral_limit, params.speed_integral_limit);
    private_nh.param("max_accel_pedal", params.max_accel_pedal, params.max_accel_pedal);
    private_nh.param("max_brake_pedal", params.max_brake_pedal, params.max_brake_pedal);
    private_nh.param("avoid_reconsider_distance", params.avoid_reconsider_distance, params.avoid_reconsider_distance);
    private_nh.param("avoid_plan_margin", params.avoid_plan_margin, params.avoid_plan_margin);
    private_nh.param("use_lidar_obstacles", params.use_lidar_obstacles, params.use_lidar_obstacles);
    private_nh.param("lidar_objects_topic", params.lidar_objects_topic, params.lidar_objects_topic);
    private_nh.param("lidar_timeout", params.lidar_timeout, params.lidar_timeout);
    private_nh.param("lidar_max_x", params.lidar_max_x, params.lidar_max_x);
    private_nh.param("lidar_min_height", params.lidar_min_height, params.lidar_min_height);
    private_nh.param("lidar_max_length", params.lidar_max_length, params.lidar_max_length);
    private_nh.param("lidar_min_points", params.lidar_min_points, params.lidar_min_points);
    private_nh.param("lidar_road_min_overlap", params.lidar_road_min_overlap, params.lidar_road_min_overlap);
    private_nh.param("lidar_lane_min_overlap", params.lidar_lane_min_overlap, params.lidar_lane_min_overlap);
    private_nh.param("lidar_structure_hold", params.lidar_structure_hold, params.lidar_structure_hold);
    private_nh.param("enable_control", params.enable_control, params.enable_control);
    private_nh.param("lookahead_distance", params.lookahead_distance, params.lookahead_distance);
    private_nh.param("target_speed_kph", params.target_speed_kph, params.target_speed_kph);
    private_nh.param("inferred_speed_kph", params.inferred_speed_kph, params.inferred_speed_kph);
    private_nh.param("wheelbase", params.wheelbase, params.wheelbase);
    private_nh.param("steering_curvature_gain", params.steering_curvature_gain, params.steering_curvature_gain);
    private_nh.param("max_steering_deg", params.max_steering_deg, params.max_steering_deg);
    private_nh.param("max_steering_rate", params.max_steering_rate, params.max_steering_rate);
    private_nh.param("max_lateral_accel", params.max_lateral_accel, params.max_lateral_accel);
    private_nh.param("control_rate", params.control_rate, params.control_rate);
    private_nh.param("source_timeout", params.source_timeout, params.source_timeout);
    private_nh.param("receive_timeout", params.receive_timeout, params.receive_timeout);
    private_nh.param("future_tolerance", params.future_tolerance, params.future_tolerance);
    private_nh.param("lane_index", params.lane_index, params.lane_index);
    private_nh.param("lane_confirm_frames", params.lane_confirm_frames, params.lane_confirm_frames);
    private_nh.param("lane_selection_distance", params.lane_selection_distance, params.lane_selection_distance);
    private_nh.param("lane_boundary_margin", params.lane_boundary_margin, params.lane_boundary_margin);
    private_nh.param("allow_inferred", params.allow_inferred, params.allow_inferred);
    private_nh.param("min_confidence", params.min_confidence, params.min_confidence);
    private_nh.param("join_tolerance", params.join_tolerance, params.join_tolerance);
    private_nh.param("stop_brake", params.stop_brake, params.stop_brake);
    private_nh.param("shutdown_stop_duration", params.shutdown_stop_duration, params.shutdown_stop_duration);
    private_nh.param("marker_z", params.marker_z, params.marker_z);
    private_nh.param("epsilon", params.epsilon, params.epsilon);
    private_nh.param("max_steering_dt", params.max_steering_dt, params.max_steering_dt);
    private_nh.param("consumer_x_min", params.consumer_x_min, params.consumer_x_min);
    private_nh.param("consumer_x_max", params.consumer_x_max, params.consumer_x_max);
    private_nh.param("min_segment_length", params.min_segment_length, params.min_segment_length);
    private_nh.param("lane_width_min", params.lane_width_min, params.lane_width_min);
    private_nh.param("lane_width_max", params.lane_width_max, params.lane_width_max);
    private_nh.param("max_width_variation", params.max_width_variation, params.max_width_variation);
    private_nh.param("inferred_confidence_scale", params.inferred_confidence_scale, params.inferred_confidence_scale);
    private_nh.param("expected_config_sha256", params.expected_config_sha256, params.expected_config_sha256);
    private_nh.param("input_topic", params.input_topic, params.input_topic);
    private_nh.param("control_topic", params.control_topic, params.control_topic);
    private_nh.param("preview_topic", params.preview_topic, params.preview_topic);
    private_nh.param("target_topic", params.target_topic, params.target_topic);
    private_nh.param("markers_topic", params.markers_topic, params.markers_topic);
    private_nh.param("centerline_marker_width", params.centerline_marker_width, params.centerline_marker_width);
    private_nh.param("centerpoint_marker_size", params.centerpoint_marker_size, params.centerpoint_marker_size);
    private_nh.param("centerlabel_marker_size", params.centerlabel_marker_size, params.centerlabel_marker_size);
    private_nh.param("centerlabel_z_offset", params.centerlabel_z_offset, params.centerlabel_z_offset);

    private_nh.param("diagnostics_topic", params.diagnostics_topic, params.diagnostics_topic);
    private_nh.param("emergency_stop_topic", params.emergency_stop_topic, params.emergency_stop_topic);
}

void validateParameters() {
    for(double value:{params.dynamic_prediction_sec,params.dynamic_clear_sec,params.dynamic_acceleration_mps2,params.dynamic_uncertain_speed_kph,
                      params.dynamic_stop_speed_mps,params.dynamic_rest_sec,params.line_return_clear_sec,params.line_commit_sec})
        if(!std::isfinite(value)||value<=0)throw std::runtime_error("invalid dynamic obstacle risk parameters");

    // 0보다 커야 하는 값.
    const std::vector<double> positive_values = {
        params.lookahead_distance, params.wheelbase, params.steering_curvature_gain,
        params.max_steering_deg, params.max_steering_rate, params.max_lateral_accel,
        params.control_rate, params.source_timeout, params.receive_timeout,
        params.lane_selection_distance, params.join_tolerance, params.shutdown_stop_duration,
        params.epsilon, params.max_steering_dt, params.consumer_x_max,
        params.min_segment_length, params.lane_width_min, params.lane_width_max,
        params.max_width_variation, params.inferred_confidence_scale,
        params.centerline_marker_width, params.centerpoint_marker_size,
        params.centerlabel_marker_size, params.centerlabel_z_offset
    };
    for (double value : positive_values) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::runtime_error("parameters must be finite and positive");
        }
    }

    // 0을 허용하는 값.
    const std::vector<double> nonnegative_values = {
        params.target_speed_kph, params.inferred_speed_kph, params.future_tolerance,
        params.lane_boundary_margin, params.min_confidence, params.consumer_x_min
    };
    for (double value : nonnegative_values) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::runtime_error("parameters must be finite and nonnegative");
        }
    }

    // 차로 선택 / lookahead / adapter 범위의 관계.
    const bool invalid_lane_selection =
        params.lane_index < -1 || params.lane_index > 1 || params.lane_confirm_frames < 1;
    const bool invalid_lookahead_range =
        params.consumer_x_min > params.lane_selection_distance ||
        params.lane_selection_distance >= params.lookahead_distance ||
        params.lookahead_distance >= params.consumer_x_max;
    const bool invalid_lane_geometry =
        params.lane_width_min >= params.lane_width_max ||
        params.lane_boundary_margin * 2.0 >= params.lane_width_min ||
        params.min_confidence > 1.0 || params.inferred_confidence_scale > 1.0;
    const bool invalid_control_settings =
        !std::isfinite(params.stop_brake) || params.stop_brake <= 0.0 || params.stop_brake > 1.0 ||
        params.max_steering_deg >= 90.0;
    const bool invalid_display_or_contract =
        !std::isfinite(params.marker_z) || params.expected_config_sha256.empty();

    if (invalid_lane_selection || invalid_lookahead_range || invalid_lane_geometry ||
        invalid_control_settings || invalid_display_or_contract) {
        throw std::runtime_error("invalid lane follower parameter ranges");
    }

    const std::vector<std::string> topics = {
        params.input_topic, params.control_topic, params.preview_topic,
        params.target_topic, params.markers_topic, params.diagnostics_topic,
        params.emergency_stop_topic
    };
    for (const std::string& topic : topics) {
        if (topic.empty()) {
            throw std::runtime_error("topic names must not be empty");
        }
    }
}


// ============================================================================
// 11. LiDAR-only dynamic risk after path selection.
// ============================================================================
// Applied after line/path selection, so an adjacent object does not stop a
// vehicle whose actual selected path remains clear. No class-based trigger.
std::vector<dynamic_risk::Object> riskObjects(){
    std::vector<dynamic_risk::Object> objects;
    const double age=std::max(0.,(ros::Time::now()-lidar_state.stamp).toSec());
    for(const auto& measured:current_obstacles){
        dynamic_risk::Object object{{measured.x,measured.x_far,measured.y_min,measured.y_max},measured.motion,measured.track_id};
        object.footprint=measured.footprint;object.uncertainty=object.motion.valid?
            (object.motion.extrapolated?.3*object.motion.age+1.5*object.motion.age*object.motion.age:0.):.5*age;
        objects.push_back(std::move(object));
    }
    return objects;
}

std::string crossingYieldPlan(bool& handled){
    handled=false;
    if(!egoSpeedFresh()||!follower_state.target_ready)return "";
    const double now=ros::Time::now().toSec();
    const LaneChain* physical=nullptr;double closest=1e100;
    for(const auto& lane:change_state.lanes){
        LaneSample sample;
        if(samplePlanningLane(lane,params.planning_start_x,sample)&&std::abs(sample.center.y)<closest){
            physical=&lane;closest=std::abs(sample.center.y);
        }
    }
    if(!physical)return "";
    std::vector<FamilyPoint2d> reference;
    if(!buildPath(*physical,true,reference))return "";
    std::vector<dynamic_risk::Point> road;
    for(auto p:reference)road.push_back({p.x,p.y});
    std::vector<dynamic_risk::Object> crossing;
    for(const auto& object:riskObjects())if(dynamic_risk::lateralMover(object,road))crossing.push_back(object);
    dynamic_risk::Config config;config.front=params.vehicle_front;config.rear=params.vehicle_rear;
    config.width=params.vehicle_width;config.margin=std::max(.3,params.obstacle_lateral_margin);
    config.stationary_margin=params.obstacle_lateral_margin;
    config.acceleration=params.dynamic_acceleration_mps2;
    config.horizon=std::max(4.,params.dynamic_prediction_sec);
    // A crossing risk in the old physical lane must not replace an already
    // verified avoidance curve with that occupied lane's centreline.
    if(!crossing.empty()&&line_mode_active&&line_state.target>0.&&
       std::abs(line_state.target-line_state.home)>params.epsilon){
        std::vector<FamilyPoint2d> chosen;
        if(lineClear(line_state.target,chosen)){
            std::vector<dynamic_risk::Point> actual;
            for(auto point:chosen)actual.push_back({point.x,point.y});
            if(dynamic_risk::assess(actual,crossing,ego_speed.speed,params.target_speed_kph/3.6,config).clear){
                dynamic_state.crossing_until=-1.;return "";
            }
        }
    }
    if(!crossing.empty()){
        const auto risk=dynamic_risk::assess(road,crossing,ego_speed.speed,params.target_speed_kph/3.6,config);
        if(!risk.clear)dynamic_state.crossing_until=now+.5;
    }
    if(now>dynamic_state.crossing_until)return "";
    handled=true;dynamic_state.crossing_yield=true;
    // Give the actor time to pass before searching for an opposite escape line.
    // A reference path already occupied now requires a full stop, not creep.
    if(!pathInsideRoad(reference)||!pathClearOfLineObstacles(reference))return "dynamic_crossing_wait";
    storePlannedTarget(*physical,reference);
    change_state.target_lane=physical->lane_index;
    return "";
}

std::string applyDynamicObstacleStop(morai_msgs::CtrlCmd& command,double dt,double previous_integral,bool creep) {
    const double now=ros::Time::now().toSec();
    dynamic_state.known_motion=dynamic_state.unknown_motion=0;
    const auto stop=[&](const std::string& reason){
        dynamic_state.hold.engage();dynamic_state.decision="stop";return reason;
    };
    // refreshPlanningObstacles clears invalid inputs. An empty vector caused
    // by sensor loss must never certify free space while the vehicle is moving.
    if(!lidarFresh()||!egoSpeedFresh()||!imuFresh())
        return stop("dynamic_inputs_unavailable");
    if(dynamic_state.hold.active&&creep){
        dynamic_state.hold.clear.reset("creep_without_confirmed_path");dynamic_state.decision="waiting_for_safe_path";
        return "dynamic_wait_for_safe_path";
    }
    const auto objects=riskObjects();
    if(!objects.empty()&&!egoSpeedFresh())return stop("dynamic_ego_unavailable");
    bool near_unknown=false;
    for(const auto& object:objects){
        if(object.motion.valid)++dynamic_state.known_motion;
        else{
            ++dynamic_state.unknown_motion;
            near_unknown|=object.box.x0<params.lookahead_distance+params.vehicle_front&&
                object.box.x1> -params.vehicle_rear&&object.box.y0<params.vehicle_width/2.+2.&&
                object.box.y1> -params.vehicle_width/2.-2.;
        }
    }
    dynamic_risk::Config config;config.front=params.vehicle_front;config.rear=params.vehicle_rear;
    config.width=params.vehicle_width;config.margin=std::max(.3,params.obstacle_lateral_margin);
    config.stationary_margin=params.obstacle_lateral_margin;
    config.acceleration=params.dynamic_acceleration_mps2;
    config.horizon=params.dynamic_prediction_sec;
    std::vector<dynamic_risk::Point> path{{0,0}};
    if(creep){
        // Creep uses the last steering command. Checking a different old lane
        // path would certify a trajectory the vehicle is not actually following.
        const double curvature=std::tan(command.steering)*params.steering_curvature_gain/params.wheelbase;
        const double horizon=std::max(config.horizon,ego_speed.speed/config.deceleration+config.reaction);
        const double distance=std::max(params.path_horizon,ego_speed.speed*horizon+params.vehicle_front);
        for(double arc=.25;arc<=distance;arc+=.25)
            path.push_back(std::abs(curvature)<1e-6?dynamic_risk::Point{arc,0}:
                dynamic_risk::Point{std::sin(curvature*arc)/curvature,(1-std::cos(curvature*arc))/curvature});
    }else for(auto p:follower_state.selected_lane.centerline)
        if(p.x>path.back().x&&std::hypot(p.x-path.back().x,p.y-path.back().y)>.01)path.push_back({p.x,p.y});
    const double original=command.velocity;
    double target=original;
    if(near_unknown||dynamic_state.crossing_yield||dynamic_state.hold.active||now<dynamic_state.resume_until)
        target=std::min(target,params.dynamic_uncertain_speed_kph);
    const auto assess=[&](double kph){return dynamic_risk::assess(path,objects,std::max(0.,ego_speed.speed),kph/3.6,config);};
    dynamic_state.risk=assess(target);
    dynamic_state.risk_checked=true;
    if(!dynamic_state.risk.clear){
        for(double fraction:{.5,.25}){
            const double candidate=target*fraction;
            const auto risk=assess(candidate);
            if(risk.clear){target=candidate;dynamic_state.risk=risk;break;}
        }
    }
    if(!dynamic_state.risk.clear){
        ROS_WARN_THROTTLE(2.0,"Dynamic obstacle stop: id=%d ttc=%.2fs known=%u unknown=%u",
            dynamic_state.risk.id,dynamic_state.risk.ttc,dynamic_state.known_motion,dynamic_state.unknown_motion);
        return stop("dynamic_path_conflict");
    }
    if(dynamic_state.hold.active){
        std::vector<std::pair<double,double>> shape;
        for(const auto& point:path)shape.emplace_back(point.x,point.y);
        dynamic_state.hold.observePath(shape);
        const std::string plan=std::to_string(follower_state.locked_lane)+"/"+
            std::to_string(static_cast<int>(std::lround(line_state.target*4)));
        if(!dynamic_state.hold.release(now,lidarFresh()&&egoSpeedFresh()&&imuFresh(),true,plan,
                lidar_state.sequence,lidar_state.source,params.dynamic_rest_sec,params.dynamic_clear_sec)){
            dynamic_state.decision=dynamic_state.hold.resting(now,params.dynamic_rest_sec)?"confirming_clear":"waiting_for_rest";
            return dynamic_state.decision=="waiting_for_rest"?"dynamic_wait_for_rest":"dynamic_confirming_clear";
        }
        dynamic_state.resume_until=now+1.;
    }
    dynamic_state.decision=target<original||dynamic_state.crossing_yield?"yield":"clear";
    if(target<original){speed_integral=previous_integral;command.accel=command.brake=0.;applySpeedControl(command,target,dt);}
    return "";
}

// ============================================================================
// 12. 경로 생성 / 차량 모서리의 도로 경계 검사 (장애물 충돌 거리 검사는 없음)
// ============================================================================
const LaneChain* getPlanningLane(int lane_index) {
    const LaneChain* found = nullptr;
    for (const auto& lane : change_state.lanes) {
        if (lane.lane_index != lane_index) continue;
        LaneSample first, last;
        if (!interpolateLaneAtX(lane, params.planning_start_x, first) ||
            !interpolateLaneAtX(lane, params.path_horizon, last)) continue;
        if (found != nullptr) return nullptr;  // 겹친 경계가 모호하면 변경하지 않는다.
        found = &lane;
    }
    return found;
}

bool samplePlanningLane(const LaneChain& lane, double x, LaneSample& sample) {
    if (x >= params.planning_start_x && interpolateLaneAtX(lane, x, sample)) return true;
    // 후방 차로 점유 확인은 관측되지 않는 경계를 근거리 접선으로 연장한 보수적 근사다.
    // 전방도 관측 끝 밖에서는 마지막 1m 접선으로 검사하되 주행 경로 자체에는 외삽하지 않는다.
    double first_x = params.planning_start_x;
    if (x >= params.planning_start_x) {
        // 관측 중간의 INVALID gap을 다른 조각의 접선으로 메우지 않는다.
        // 경로 끝까지 연속 관측된 조각만 앞범퍼 검사에 필요한 끝단 외삽을 허용한다.
        if (x <= lane.centerline.back().x || lane.centerline.back().x < params.path_horizon) return false;
        first_x = lane.centerline.back().x - 1.0;
    }
    LaneSample first, second;
    if (!interpolateLaneAtX(lane, first_x, first) || !interpolateLaneAtX(lane, first_x + 1.0, second)) return false;
    const double ratio = x - first_x;
    sample.center = interpolatePoint(first.center, second.center, ratio);
    sample.left = interpolatePoint(first.left, second.left, ratio);
    sample.right = interpolatePoint(first.right, second.right, ratio);
    return true;
}

bool buildPath(const LaneChain& lane, bool changing, std::vector<FamilyPoint2d>& path, double lateral_scale,
               bool limit_lateral_step, double length_scale) {
    path.clear();
    LaneSample origin, next;
    if (!samplePlanningLane(lane, 0.0, origin) || !samplePlanningLane(lane, 1.0, next)) return false;
    const double origin_heading = next.center.y - origin.center.y;
    // 회피 offset은 선택 차로 중심선 전체를 평행 이동한다(+좌).
    const double offset = avoidOffsetFor(lane.lane_index);
    const double origin_y = origin.center.y + offset;
    // 같은 차로 안 회피는 장애물 앞에서 이동을 마치도록 짧은 연결 길이를 쓴다.
    // 큰 offset(차선 걸침)은 quintic 최대 곡률 5.77·A/L²이 조향 한계의 80% 안에 들도록 길이를 늘린다.
    const double curvature_limit = params.steering_curvature_gain *
        std::tan(params.max_steering_deg * PI / 180.0) / params.wheelbase;
    // 실제 필요한 횡이동(현재 위치→목표)으로 길이를 정한다. 일반 구간처럼 여러 배수(length_scale)를 후보로 쓴다.
    const double avoid_length = std::clamp(
        std::sqrt(5.77 * std::abs(origin_y) / (0.8 * curvature_limit)) * length_scale,
        params.avoid_shift_length, params.path_horizon);
    const double connect_length = avoid_state.active && !change_state.changing &&
        lane.lane_index == avoid_state.lane ? avoid_length : params.lane_change_length;
    const int steps = static_cast<int>(std::ceil(params.path_horizon / params.path_sample_step));
    for (int index = 0; index <= steps; ++index) {
        const double x = std::min(index * params.path_sample_step, params.path_horizon);
        LaneSample sample;
        if (!samplePlanningLane(lane, x, sample)) return false;
        double y = sample.center.y + offset;
        if (changing) {
            // 재계획할 때마다 현재 차량 위치/방향(y=0, dy/dx=0)에서 옆 중심선에 부드럽게 연결.
            const double ratio = std::clamp(x / connect_length, 0.0, 1.0);
            const double blend = ratio * ratio * ratio * (10.0 + ratio * (-15.0 + 6.0 * ratio));
            // 짧은 관측 지평에서 차량 앞부분이 바깥 경계를 넘지 않도록 횡이동량을 제한.
            const double lateral_gain = (limit_lateral_step ? std::min(1.0, params.max_lateral_step /
                std::max(std::abs(origin_y), params.epsilon)) : 1.0) * lateral_scale;
            // 횡이동량만 줄인다. 도로 방향 정렬까지 줄이면 변경 중 차체가 비스듬한
            // 상태에서 모든 후보의 앞 모서리가 바깥 경계를 넘어 영구 정지할 수 있다.
            y -= origin_y * (1.0 - blend * lateral_gain);
            y -= origin_heading * x * (1.0 - blend);
        }
        path.push_back({x, y});
    }
    return true;
}

bool buildRoadBoundedChangePath(const LaneChain& lane, std::vector<FamilyPoint2d>& path) {
    // 회피 중에는 매 주기 재계획으로 결국 따라가게 될 횡이동 제한 없는 연결 경로가 장애물을 피해야 한다.
    // The final reduced path is checked again, not just its converged candidate.
    if (avoid_state.active && lane.lane_index == avoid_state.lane) {
        // 일반 구간 저속 AVOID처럼 전환거리 1.0~1.6배 후보를 평가해 장애물 충돌과 주행 영역(차체 모서리,
        // 곡률)을 모두 통과하는 가장 짧은 것을 쓴다. 긴 전환은 차체가 덜 비스듬해 모서리가 덜 튀어나간다.
        for (double length_scale : {1.0, 1.2, 1.4, 1.6}) {
            std::vector<FamilyPoint2d> converged;
            if (!buildPath(lane, true, converged, 1.0, false, length_scale) ||
                !pathClearOfAvoidObstacles(converged)) continue;
            for (double scale : {1.0, 0.85, 0.70, 0.55, 0.40}) {
                if (buildPath(lane, true, path, scale, true, length_scale) && pathInsideRoad(path) &&
                    pathClearOfAvoidObstacles(path)){path_block_reason.clear();return true;}
            }
        }
        if (path_block_reason.empty()) path_block_reason = "obstacle_collision";
        return false;
    }
    // 방향을 바꾸지 않고, 큰 횡이동 후보부터 차량 footprint가 들어가는 후보를 선택한다.
    for (double scale : {1.0, 0.85, 0.70, 0.55, 0.40}) {
        if (buildPath(lane, true, path, scale) && pathInsideRoad(path)) return true;
    }
    return false;
}

bool pathInsideRoad(const std::vector<FamilyPoint2d>& path) {
    change_state.maximum_path_curvature = 0.0;
    for (std::size_t index = 0; index < path.size(); ++index) {
        const auto& point = path[index];
        const std::size_t previous = index == 0 ? 0 : index - 1;
        const std::size_t next = std::min(index + 1, path.size() - 1);
        const double yaw = std::atan2(path[next].y - path[previous].y, path[next].x - path[previous].x);
        // Road containment uses the actual body length. The collision-only
        // longitudinal/sample margin remains in the obstacle swept-body check;
        // adding it here can put a fictitious rear corner outside an unseen
        // extrapolated boundary and reject every possible recovery at rest.
        const double front = params.vehicle_front;
        const double rear = params.vehicle_rear;
        const double half_width = params.vehicle_width / 2.0 + params.obstacle_lateral_margin;
        for (double dx : {-rear, front}) {
            for (double dy : {-half_width, half_width}) {
                const double x = point.x + dx * std::cos(yaw) - dy * std::sin(yaw);
                const double y = point.y + dx * std::sin(yaw) + dy * std::cos(yaw);
                // 두 차로의 바깥 경계(회피 중에는 주행 가능 영역) 밖으로 차량 모서리가 나가는 후보는 거부.
                double road_left = -std::numeric_limits<double>::infinity();
                double road_right = std::numeric_limits<double>::infinity();
                if (avoid_state.active || line_mode_active) {
                    if (!corridorAt(x, road_left, road_right)) {
                        path_block_reason = "corridor_unavailable_x=" + std::to_string(x);
                        return false;
                    }
                } else {
                    for (const auto& lane : change_state.lanes) {
                        LaneSample sample;
                        if (!samplePlanningLane(lane, x, sample)) continue;
                        road_left = std::max(road_left, sample.left.y);
                        road_right = std::min(road_right, sample.right.y);
                    }
                }
                if (y > road_left || y < road_right) {
                    char detail[160];
                    std::snprintf(detail, sizeof(detail), "outside_road x=%.1f y=%.2f bounds=[%.2f,%.2f]",
                                  x, y, road_right, road_left);
                    path_block_reason = detail;
                    return false;
                }
            }
        }
        if (index > 0 && index + 1 < path.size()) {
            const auto& before = path[index - 1];
            const auto& after = path[index + 1];
            const double denominator = pointDistance(before, point) * pointDistance(point, after) * pointDistance(before, after);
            if (denominator > params.epsilon) {
                const double cross = (point.x - before.x) * (after.y - before.y) -
                                     (point.y - before.y) * (after.x - before.x);
                const double curvature = 2.0 * std::abs(cross) / denominator;
                change_state.maximum_path_curvature = std::max(change_state.maximum_path_curvature, curvature);
            }
        }
    }
    const double steering_curvature_limit = params.steering_curvature_gain *
        std::tan(params.max_steering_deg * PI / 180.0) / params.wheelbase;
    if (change_state.maximum_path_curvature > steering_curvature_limit) {
        path_block_reason = "curvature " + std::to_string(change_state.maximum_path_curvature);
        return false;
    }
    return true;
}

// 회피 중 기억한 장애물 영역과 경로 각 점의 차량 footprint(방향 포함)가 겹치지 않는지 검사한다.
bool pathClearOfAvoidObstacles(const std::vector<FamilyPoint2d>& path) {
    const double front = params.vehicle_front + params.obstacle_longitudinal_margin + params.path_sample_step;
    const double rear = params.vehicle_rear + params.obstacle_longitudinal_margin + params.path_sample_step;
    const double half_width = params.vehicle_width / 2.0 + params.obstacle_lateral_margin;
    for (std::size_t index = 0; index < path.size(); ++index) {
        const std::size_t previous = index == 0 ? 0 : index - 1;
        const std::size_t next = std::min(index + 1, path.size() - 1);
        const double yaw = std::atan2(path[next].y - path[previous].y, path[next].x - path[previous].x);
        if (vehicleHitsAvoidObstacle(path[index], yaw, front, rear, half_width)) {
            char detail[96];
            std::snprintf(detail, sizeof(detail), "obstacle_collision path_x=%.1f y=%.2f", path[index].x, path[index].y);
            path_block_reason = detail;
            return false;
        }
    }
    return true;
}

void storePlannedTarget(const LaneChain& lane, const std::vector<FamilyPoint2d>& path) {
    follower_state.selected_lane = lane;
    follower_state.selected_lane.centerline = path;
    change_state.path = path;
    // 같은 차로 안 회피는 짧은 offset 연결을 따라가도록 추종점을 당긴다.
    const double lookahead = (avoid_state.active || (line_mode_active&&
        std::abs(line_state.target-line_state.home)>params.epsilon)) && !change_state.changing ?
        params.avoid_lookahead_distance : params.lookahead_distance;
    for (std::size_t index = 1; index < path.size(); ++index) {
        if (path[index].x < lookahead) continue;
        const double ratio = (lookahead - path[index - 1].x) / (path[index].x - path[index - 1].x);
        follower_state.target_point = interpolatePoint(path[index - 1], path[index], ratio);
        return;
    }
}

// ============================================================================
// 13. 중앙선 걸침 회피: 차로별 허용 offset 계산, 차로 선택, 통과 거리
// ============================================================================
double avoidOffsetFor(int lane_index) {
    return avoid_state.active && lane_index == avoid_state.lane ? avoid_state.offset : 0.0;
}

// ============================================================================
// 5개 기준선: 1 왼쪽 도로 끝, 2 왼쪽 차로 중앙, 3 중앙선, 4 오른쪽 차로 중앙, 5 오른쪽 도로 끝.
// 장애물이 덮는 범위를 이 선 번호로 표현하고, 그 사이 후보 주행선 중 하나를 고른다.
// ============================================================================
bool sampleLaneIndexAt(int lane_index, double x, LaneSample& sample) {
    for (const auto& lane : change_state.lanes) {
        if (lane.lane_index != lane_index || lane.centerline.size() < 2) continue;
        if (samplePlanningLane(lane, x, sample) || sampleLaneLinear(lane, x, sample)) return true;
    }
    return false;
}

bool lineFrameAt(double x, LineFrame& frame) {
    LaneSample left, right;
    if (!sampleLaneIndexAt(0, x, left) || !sampleLaneIndexAt(1, x, right)) return false;
    frame.y[0] = 0.0;
    frame.y[1] = left.left.y + params.line_edge_expand;
    frame.y[2] = left.center.y;
    frame.y[3] = 0.5 * (left.right.y + right.left.y);
    frame.y[4] = right.center.y;
    frame.y[5] = right.right.y - params.line_edge_expand;
    for (int line = 1; line < 5; ++line) {
        if (!(frame.y[line] > frame.y[line + 1])) return false;
    }
    return true;
}

// 선 번호(실수) → y. 1~5 밖은 가장 가까운 구간의 기울기로 연장한다.
double lineY(const LineFrame& frame, double line) {
    const int segment = std::clamp(static_cast<int>(std::floor(line)), 1, 4);
    return frame.y[segment] + (line - segment) * (frame.y[segment + 1] - frame.y[segment]);
}

double lineOf(const LineFrame& frame, double y) {
    int segment = 4;
    for (int candidate = 1; candidate <= 4; ++candidate) {
        if (y >= frame.y[candidate + 1]) { segment = candidate; break; }
    }
    return segment + (y - frame.y[segment]) / (frame.y[segment + 1] - frame.y[segment]);
}

// 내 차로 중앙(2 또는 4)에서 가까운 후보 주행선부터 검사한다. 같은 거리면 일반 구간 설정처럼 오른쪽 우선.
// 가능 조건: 모든 장애물에서 차체(반폭+avoid_clearance+avoid_plan_margin)가 장애물 횡범위와 겹치지 않고,
// 차체 모서리(반폭+obstacle_lateral_margin)가 주행 영역(corridorAt) 안에 있으며, offset이 avoid_max_offset 이하.
AvoidFit lineGridAvoidOffset(const LaneChain& own, double& offset) {
    const double own_line = own.lane_index == 0 ? 2.0 : 4.0;
    const double edge = params.vehicle_width / 2.0 + params.obstacle_lateral_margin;
    const double clear = params.vehicle_width / 2.0 + params.avoid_clearance + params.avoid_plan_margin;
    struct Placed { LaneChangeState::MeasuredObstacle obstacle; LineFrame frame; double x, left, right; };
    std::vector<Placed> placed;
    double cover_min = std::numeric_limits<double>::infinity(), cover_max = -cover_min;
    for (const auto& obstacle : planning_obstacles) {
        if (obstacle.x_far < -(params.vehicle_rear + params.avoid_pass_margin)) continue;  // 이미 지나감.
        Placed item{obstacle, {}, std::max(obstacle.x, params.planning_start_x), 0.0, 0.0};
        if (!lineFrameAt(item.x, item.frame) || !corridorAt(item.x, item.left, item.right)) {
            line_decision = "lines_unavailable";
            return AvoidFit::kUnknown;
        }
        cover_min = std::min(cover_min, lineOf(item.frame, obstacle.y_max));
        cover_max = std::max(cover_max, lineOf(item.frame, obstacle.y_min));
        placed.push_back(item);
    }
    char summary[160];
    if (placed.empty()) {
        offset = 0.0;
        last_target_line = own_line;
        line_decision = "clear";
        return AvoidFit::kFits;
    }
    const auto feasible = [&](double line) {
        for (const auto& item : placed) {
            const double y = lineY(item.frame, line);
            const double shift = y - lineY(item.frame, own_line);
            if (std::abs(shift) > params.avoid_max_offset + params.epsilon) return false;
            if (!(y + clear <= item.obstacle.y_min || y - clear >= item.obstacle.y_max)) return false;
            if (y + edge > item.left || y - edge < item.right) return false;
        }
        return true;
    };
    const int steps = static_cast<int>(std::ceil(4.0 / params.avoid_line_step));
    for (int k = 0; k <= steps; ++k) {
        // 선 번호가 클수록 오른쪽이다. k=0은 내 차로 중앙 한 번만 검사한다.
        for (int side : {1, -1}) {
            if (k == 0 && side < 0) continue;
            const double line = own_line + side * k * params.avoid_line_step;
            if (!feasible(line)) continue;
            const auto& nearest = *std::min_element(placed.begin(), placed.end(),
                [](const auto& a, const auto& b) { return a.x < b.x; });
            offset = lineY(nearest.frame, line) - lineY(nearest.frame, own_line);
            last_target_line = line;
            last_adjacent_offset = lineY(nearest.frame, line) - lineY(nearest.frame, own_line == 2.0 ? 4.0 : 2.0);
            std::snprintf(summary, sizeof(summary), "cover %.2f-%.2f own %.0f -> line %.2f (%.2fm)",
                          cover_min, cover_max, own_line, line, offset);
            line_decision = summary;
            return AvoidFit::kFits;
        }
    }
    std::snprintf(summary, sizeof(summary), "cover %.2f-%.2f own %.0f -> no line", cover_min, cover_max, own_line);
    line_decision = summary;
    return AvoidFit::kNoRoom;
}

// 최신 측정 장애물을 피하면서 lane 안에 차체가 들어가는 중심 offset 중 0에 가장 가까운 값.
// 장애물 하단 가장자리 ± (차체 반폭 + avoid_clearance)는 차량 중심이 들어갈 수 없는 구간이다.
// 경로 범위 밖의 먼 장애물도 포함한다. 건너뛰면 막힌 차로를 offset 0으로 통과 가능하다고 오판한다.
AvoidFit laneAvoidOffset(const LaneChain& lane, double& offset, bool allow_straddle) {
    if (allow_straddle) return lineGridAvoidOffset(lane, offset);
    const double edge = params.vehicle_width / 2.0 + params.obstacle_lateral_margin;
    const double clear = params.vehicle_width / 2.0 + params.avoid_clearance + params.avoid_plan_margin;
    double low = -std::numeric_limits<double>::infinity();
    double high = std::numeric_limits<double>::infinity();
    std::vector<std::pair<double, double>> blocked;
    for (const auto& obstacle : planning_obstacles) {
        if (obstacle.x_far < -(params.vehicle_rear + params.avoid_pass_margin)) continue;  // 이미 지나감.
        LaneSample sample;
        if (!samplePlanningLane(lane, std::max(obstacle.x, params.planning_start_x), sample)) return AvoidFit::kUnknown;
        if (allow_straddle) {
            // 일반 구간 AVOID처럼 차선은 넘을 수 있다. 차체는 주행 가능 영역 안, offset은 avoid_max_offset 이하.
            double road_left = 0.0, road_right = 0.0;
            if (!corridorAt(std::max(obstacle.x, params.planning_start_x), road_left, road_right)) return AvoidFit::kUnknown;
            low = std::max({low, road_right + edge - sample.center.y, -params.avoid_max_offset});
            high = std::min({high, road_left - edge - sample.center.y, params.avoid_max_offset});
        } else {
            low = std::max(low, sample.right.y - sample.center.y + edge);
            high = std::min(high, sample.left.y - sample.center.y - edge);
        }
        blocked.emplace_back(obstacle.y_min - sample.center.y - clear, obstacle.y_max - sample.center.y + clear);
    }
    if (blocked.empty()) {
        offset = 0.0;
        return AvoidFit::kFits;
    }
    if (low > high) return AvoidFit::kNoRoom;
    std::vector<double> candidates = {std::clamp(0.0, low, high)};
    for (const auto& interval : blocked) {
        for (double value : {interval.first, interval.second}) {
            if (value >= low - params.epsilon && value <= high + params.epsilon) {
                candidates.push_back(std::clamp(value, low, high));
            }
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](double first, double second) {
        return std::abs(first) < std::abs(second);
    });
    for (double candidate : candidates) {
        const bool inside = std::any_of(blocked.begin(), blocked.end(), [&](const auto& interval) {
            return candidate > interval.first + params.epsilon && candidate < interval.second - params.epsilon;
        });
        if (!inside) {
            offset = candidate;
            return AvoidFit::kFits;
        }
    }
    return AvoidFit::kNoRoom;
}

// 횡이동이 작은 쪽을 고른다(덜 덮인 차로). 차이가 avoid_lane_tie 이하면 내 차로를 유지한다.
// 어느 한 차로라도 위치를 대응시킬 수 없으면 판단하지 않고(lane=-1) 기존 동작을 따른다.
AvoidPlan chooseAvoidPlan(const LaneChain& current, const LaneChain* adjacent) {
    AvoidPlan own_plan, adjacent_plan;
    double offset = 0.0;
    const AvoidFit own_fit = laneAvoidOffset(current, offset, params.avoid_line_straddle);
    if (own_fit == AvoidFit::kUnknown) return {};
    if (own_fit == AvoidFit::kFits) own_plan = {current.lane_index, offset, true};
    if (adjacent != nullptr) {
        const AvoidFit adjacent_fit = laneAvoidOffset(*adjacent, offset);
        if (adjacent_fit == AvoidFit::kUnknown) return {};
        if (adjacent_fit == AvoidFit::kFits) adjacent_plan = {adjacent->lane_index, offset, true};
    }
    adjacent_plan.decided = true;  // 두 차로 모두 판정됨. lane<0이면 측정 기준으로 지나갈 공간이 없다.
    // 실제 횡이동량으로 비교한다. 옆 차로는 두 차로 중심 간격 + 그 차로 offset만큼 움직인다.
    double adjacent_shift = std::numeric_limits<double>::infinity();
    LaneSample own_origin, adjacent_origin;
    if (adjacent_plan.lane >= 0 && samplePlanningLane(current, params.planning_start_x, own_origin) &&
        samplePlanningLane(*adjacent, params.planning_start_x, adjacent_origin)) {
        adjacent_shift = std::abs(adjacent_origin.center.y + adjacent_plan.offset - own_origin.center.y);
    }
    if (own_plan.lane >= 0 && std::abs(own_plan.offset) <= adjacent_shift + params.avoid_lane_tie) return own_plan;
    return adjacent_plan;
}

void rememberAvoidObstacles(const LaneChain& lane) {
    if(!planning_ready)return;
    std::vector<AvoidState::Obstacle> current;
    for(const auto& obstacle:planning_obstacles){
        LaneSample sample;
        if(!samplePlanningLane(lane,std::max(obstacle.x,params.planning_start_x),sample))continue;
        current.push_back({obstacle.x,obstacle.x_far-obstacle.x,obstacle.y_min-sample.center.y,
            obstacle.y_max-sample.center.y,avoid_state.odom});
    }
    avoid_state.obstacles=std::move(current);
}

// 장애물 가정 영역의 현재 base_link 좌표(사각형 4점). x는 속도 적분, 횡위치는 회피 차로의 현재 관측
// 중심선 기준이다. 시작·끝 x에서 각각 중심선을 샘플해 도로 방향으로 정렬한다. 차량이 도로에 대해 비스듬할 때
// 차량 x축 방향 직사각형으로 두면 장애물 끝이 옆 차로로 넘어간 것처럼 계산된다(20°에서 6m 길이당 약 2m).
bool avoidObstacleArea(const AvoidState::Obstacle& obstacle, std::array<FamilyPoint2d, 4>& area) {
    const LaneChain* lane = getPlanningLane(avoid_state.lane);
    const double x0 = obstacle.x - (avoid_state.odom - obstacle.odom_at);
    const double x1 = x0 + obstacle.length;
    LaneSample start, end;
    if (lane == nullptr || !samplePlanningLane(*lane, std::max(x0, params.planning_start_x), start)) return false;
    if (!samplePlanningLane(*lane, std::max(x1, params.planning_start_x), end) &&
        !interpolateLaneAtX(*lane, std::clamp(x1, lane->centerline.front().x, lane->centerline.back().x), end)) end = start;
    // 차량 footprint에 이미 obstacle_lateral_margin이 포함되어 있으므로 남은 간격만 더한다.
    const double extra = std::max(0.0, params.avoid_clearance - params.obstacle_lateral_margin);
    area = {{{x0, start.center.y + obstacle.rel_min - extra}, {x1, end.center.y + obstacle.rel_min - extra},
             {x1, end.center.y + obstacle.rel_max + extra}, {x0, start.center.y + obstacle.rel_max + extra}}};
    return true;
}

void startAvoidance(const AvoidPlan& plan, const LaneChain& lane) {
    avoid_state = AvoidState{};
    avoid_state.active = true;
    avoid_state.lane = plan.lane;
    avoid_state.offset = plan.offset;
    avoid_state.target_line = last_target_line;
    rememberAvoidObstacles(lane);
    ROS_INFO("Straddle avoidance started: lane %d offset %.2fm, %s (%zu obstacles)",
             plan.lane, plan.offset, line_decision.c_str(), avoid_state.obstacles.size());
}

void updateAvoidanceProgress(double elapsed) {
    if (!avoid_state.active) return;
    if (egoSpeedFresh()) avoid_state.odom += ego_speed.speed * std::clamp(elapsed, 0.0, 0.5);
    // 장애물 끝(길이 가정)과 차량 후방, 여유 거리를 모두 지난 장애물은 검사에서 제외한다.
    const double passed = params.vehicle_rear + params.avoid_pass_margin;
    auto& obstacles = avoid_state.obstacles;
    obstacles.erase(std::remove_if(obstacles.begin(), obstacles.end(), [&](const auto& obstacle) {
        return obstacle.x - (avoid_state.odom - obstacle.odom_at) + obstacle.length + passed < 0.0;
    }), obstacles.end());
    if (obstacles.empty() && !change_state.changing) {
        ROS_INFO("Straddle avoidance finished after %.1fm: following lane %d centre",
                 avoid_state.odom, follower_state.locked_lane);
        avoid_state = AvoidState{};
    }
}

// 방향을 가진 차량 사각형과 장애물 가정 사각형(x: 접점~길이, y: 하단 폭+간격)의 분리축 검사.
bool vehicleHitsAvoidObstacle(const FamilyPoint2d& point, double yaw, double front, double rear, double half_width) {
    const double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
    std::array<FamilyPoint2d, 4> vehicle;
    std::size_t corner = 0;
    for (double dx : {-rear, front}) {
        for (double dy : {-half_width, half_width}) {
            vehicle[corner++] = {point.x + dx * cos_yaw - dy * sin_yaw, point.y + dx * sin_yaw + dy * cos_yaw};
        }
    }
    for (const auto& obstacle : avoid_state.obstacles) {
        std::array<FamilyPoint2d, 4> area;
        if (!avoidObstacleArea(obstacle, area)) return true;  // 위치를 다시 잡을 수 없으면 보수적으로 거부.
        // 분리축: 차량 두 축 + 장애물 영역(평행사변형)의 두 변 법선.
        const double lx = area[1].x - area[0].x, ly = area[1].y - area[0].y;
        const double norm = std::max(std::hypot(lx, ly), params.epsilon);
        const std::array<std::array<double, 2>, 4> axes = {{{cos_yaw, sin_yaw}, {-sin_yaw, cos_yaw},
                                                           {1.0, 0.0}, {-ly / norm, lx / norm}}};
        bool separated = false;
        for (const auto& axis : axes) {
            double vehicle_min = std::numeric_limits<double>::infinity(), vehicle_max = -vehicle_min;
            double area_min = vehicle_min, area_max = vehicle_max;
            for (const auto& p : vehicle) {
                const double value = p.x * axis[0] + p.y * axis[1];
                vehicle_min = std::min(vehicle_min, value); vehicle_max = std::max(vehicle_max, value);
            }
            for (const auto& p : area) {
                const double value = p.x * axis[0] + p.y * axis[1];
                area_min = std::min(area_min, value); area_max = std::max(area_max, value);
            }
            if (vehicle_max <= area_min + params.epsilon || area_max <= vehicle_min + params.epsilon) {
                separated = true;
                break;
            }
        }
        if (!separated) return true;
    }
    return false;
}

// ============================================================================
// 14. 선 선택: 1~5 기준선 중 장애물을 피하는 선을 고르고 그 선이 속한 차로를 내 차선으로 한다.
// ============================================================================
// 차량 footprint(방향 포함)와 볼록 사각형 영역의 분리축 검사.


double lineObstacleX(const LineObstacle& obstacle) {
    return obstacle.x - (line_state.odom - obstacle.odom_at);
}

// 기억한 장애물의 현재 영역. 시작·끝 x의 기준선으로 도로 방향에 맞추고 간격(차체 여유 제외분)을 더한다.
bool lineObstacleArea(const LineObstacle& obstacle, std::array<FamilyPoint2d, 4>& area) {
    const double x0 = lineObstacleX(obstacle), x1 = x0 + obstacle.length;
    const double near_x = std::max(x0, params.planning_start_x);
    const double far_x = std::max(x1, near_x);
    LineFrame start, end;
    if (!lineFrameAt(near_x, start)) return false;
    // 먼 끝이 관측 범위를 넘어도 장애물 전체를 포기하지 않는다. 관측되는 가장 먼
    // 기준선을 쓰고, 없으면 가까운 끝의 기준선을 쓴다(차선 유지의 끝 연장과 같은 취급).
    // 한 장애물의 먼 끝을 못 잡았다고 모든 후보 선을 막으면, 차로를 일부만 덮은
    // 장애물에서도 회피 경로를 못 그리고 no_clear_line으로 정지한다.
    if (!lineFrameAt(far_x, end)) {
        end = start;
        for (double x = far_x - params.path_sample_step; x > near_x; x -= params.path_sample_step) {
            LineFrame candidate;
            if (lineFrameAt(x, candidate)) { end = candidate; break; }
        }
    }
    const double extra = std::max(0.0, params.avoid_clearance - params.obstacle_lateral_margin);
    area = {{{x0, lineY(start, obstacle.line_left) + extra}, {x1, lineY(end, obstacle.line_left) + extra},
             {x1, lineY(end, obstacle.line_right) - extra}, {x0, lineY(start, obstacle.line_right) - extra}}};
    return true;
}

bool lineObstacleConfirmed(const LineObstacle& obstacle) {
    return obstacle.hits >= params.obstacle_confirm_frames;
}

// line_obstacle_reach보다 먼 장애물은 지금 선 선택(선 간격·틈 중앙)에서 뺀다.
bool lineObstacleInReach(const LineObstacle& obstacle) {
    return params.line_obstacle_reach <= 0.0 || lineObstacleX(obstacle) <= params.line_obstacle_reach;
}

bool lineObstacleBlocksHome(const LineObstacle& obstacle) {
    if(!lineObstacleInReach(obstacle)||lineObstacleX(obstacle)+obstacle.length+
       params.vehicle_rear+params.avoid_pass_margin<=0.)return false;
    const double home=line_state.return_home>0.?line_state.return_home:
        (follower_state.locked_lane==0?2.:4.);
    LineFrame frame;
    if(!lineFrameAt(std::max(lineObstacleX(obstacle),params.planning_start_x),frame))return true;
    const double y=lineY(frame,home);
    const double margin=params.vehicle_width/2.+params.avoid_clearance+params.avoid_plan_margin;
    return y+margin>lineY(frame,obstacle.line_right)&&y-margin<lineY(frame,obstacle.line_left);
}

// Mirror the producer's complete valid track snapshot, including predicted tracks.
void rememberLineObstacles(bool /*new_scan*/) {
    if(!planning_ready||lidar_state.sequence==line_state.lidar_sequence)return;
    std::vector<LineObstacle> current;
    for(const auto& obstacle:planning_obstacles){
        LineFrame frame;
        if(!lineFrameAt(std::max(obstacle.x,params.planning_start_x),frame))continue;
        LineObstacle measured{obstacle.x,obstacle.x_far-obstacle.x,lineOf(frame,obstacle.y_max),
            lineOf(frame,obstacle.y_min),line_state.odom,1};
        measured.track_id=obstacle.track_id;measured.label=obstacle.label;
        measured.last_seen=lidar_state.stamp.toSec();measured.predicted=obstacle.predicted;
        current.push_back(std::move(measured));
    }
    lane_boundary_adapter::obstacle_memory::updateLidar(line_state.obstacles,std::move(current),
        lidar_state.source,params.obstacle_confirm_frames);
    line_state.lidar_sequence=lidar_state.sequence;
    const double passed=params.vehicle_rear+params.avoid_pass_margin;
    auto& obstacles=line_state.obstacles;
    obstacles.erase(std::remove_if(obstacles.begin(),obstacles.end(),[&](const auto& obstacle){
        return lineObstacleX(obstacle)+obstacle.length+passed<0.;
    }),obstacles.end());
}

// 선 u를 따라가는 경로. 현재 위치/방향에서 선으로 부드럽게 연결한다(buildPath와 같은 quintic).
bool buildLinePath(double line, std::vector<FamilyPoint2d>& path, double length_scale,
                   double lateral_scale, bool limit_lateral_step) {
    path.clear();
    LineFrame origin;
    if (!lineFrameAt(0.0, origin)) return false;
    const double origin_y = lineY(origin, line);
    const double curvature_limit = params.steering_curvature_gain *
        std::tan(params.max_steering_deg * PI / 180.0) / params.wheelbase;
    const double connect_length = std::clamp(
        std::max(params.avoid_shift_length,
            std::sqrt(5.77 * std::abs(origin_y) / (0.8 * curvature_limit))) * length_scale,
        params.avoid_shift_length, params.path_horizon);
    const double lateral_gain = (limit_lateral_step ? std::min(1.0, params.max_lateral_step /
        std::max(std::abs(origin_y), params.epsilon)) : 1.0) * lateral_scale;
    const int steps = static_cast<int>(std::ceil(params.path_horizon / params.path_sample_step));
    for (int index = 0; index <= steps; ++index) {
        const double x = std::min(index * params.path_sample_step, params.path_horizon);
        LineFrame frame;
        if (!lineFrameAt(x, frame)) return false;
        const double ratio = std::clamp(x / connect_length, 0.0, 1.0);
        const double blend = ratio * ratio * ratio * (10.0 + ratio * (-15.0 + 6.0 * ratio));
        // Blend the complete observed curve from the ego pose. Subtracting only
        // its near-field tangent left camera/extrapolation curvature jumps in
        // the first metre, even when the desired lane was almost straight.
        // The resulting curve still has to pass road, steering and OBB checks.
        const double y = blend * (lineY(frame, line) - origin_y * (1.0 - lateral_gain));
        path.push_back({x, y});
    }
    return true;
}

bool pathClearOfLineObstacles(const std::vector<FamilyPoint2d>& path) {
    std::vector<dynamic_risk::Object> objects;
    for(const auto& obstacle:line_state.obstacles){
        if(!lineObstacleConfirmed(obstacle))continue;
        std::array<FamilyPoint2d,4> area;
        if(!lineObstacleArea(obstacle,area)){path_block_reason="obstacle_frame_unavailable";return false;}
        dynamic_risk::Object object;object.id=obstacle.track_id;
        for(auto p:area)object.footprint.push_back({p.x,p.y});
        objects.push_back(std::move(object));
    }
    std::vector<dynamic_risk::Point> actual;
    for(auto p:path)actual.push_back({p.x,p.y});
    dynamic_risk::Config config;
    config.front=params.vehicle_front+params.obstacle_longitudinal_margin+params.path_sample_step;
    config.rear=params.vehicle_rear+params.obstacle_longitudinal_margin+params.path_sample_step;
    config.width=params.vehicle_width;config.margin=params.obstacle_lateral_margin;
    const auto hit=dynamic_risk::checkStaticPath(actual,objects,config);
    if(!hit.clear){
        path_block_reason="final_obstacle_collision id="+std::to_string(hit.id)+" x="+std::to_string(hit.point.x);
        return false;
    }
    return true;
}

// 선 단위 사전 검사: 내 차로 중앙 기준 횡이동 상한, 각 장애물과 판단 여유, 주행 영역.
// 횡이동 상한은 지금 차로 중앙 기준이다. 진입 차로 복귀 중에는 진입 차로 중앙 기준도 허용한다.
// (진입 차로를 기준으로만 재면 진입 차로를 막은 장애물을 옆 차로로 피하는 선이 모두 거부된다.)
bool lineAllowed(double line) {
    LineFrame near;
    if (!lineFrameAt(params.planning_start_x, near)) {path_block_reason="line_frame_unavailable";return false;}
    const auto within = [&](double home) {
        return home > 0.0 && std::abs(lineY(near, line) - lineY(near, home)) <= params.avoid_max_offset + params.epsilon;
    };
    if (!within(line_state.lane_home) && !within(line_state.return_home)) {path_block_reason="line_offset_limit";return false;}
    const double edge = params.vehicle_width / 2.0 + params.obstacle_lateral_margin;
    const double clear = params.vehicle_width / 2.0 + params.avoid_clearance + params.avoid_plan_margin;
    for (const auto& obstacle : line_state.obstacles) {
        if (!lineObstacleConfirmed(obstacle) || !lineObstacleInReach(obstacle)) continue;
        // 차량 후방까지 완전히 지난 장애물은 선 간격 판단에서 뺀다(경로 footprint 검사는 그대로).
        if (lineObstacleX(obstacle) + obstacle.length + params.vehicle_rear +
            params.obstacle_longitudinal_margin < 0.0) continue;
        const double x = std::max(lineObstacleX(obstacle), params.planning_start_x);
        LineFrame frame;
        double left = 0.0, right = 0.0;
        if (!lineFrameAt(x, frame) || !corridorAt(x, left, right)) {path_block_reason="line_corridor_unavailable";return false;}
        const double y = lineY(frame, line);
        if (!(y + clear <= lineY(frame, obstacle.line_right) || y - clear >= lineY(frame, obstacle.line_left))) {
            path_block_reason="line_clearance id="+std::to_string(obstacle.track_id)+" required="+std::to_string(clear);
            return false;
        }
        if (y + edge > left || y - edge < right) {
            path_block_reason="line_boundary y="+std::to_string(y)+" bounds=["+std::to_string(right)+","+std::to_string(left)+"]";
            return false;
        }
    }
    return true;
}

// 모든 연결 후보에 차체 충돌·주행 영역·조향 곡률 검사를 적용한다.
bool predictedPathRiskClear(const std::vector<FamilyPoint2d>& path){
    if(!params.dynamic_obstacle_stop||!dynamic_state.hold.active)return true;
    std::vector<dynamic_risk::Point> actual;for(auto p:path)actual.push_back({p.x,p.y});
    const auto objects=riskObjects();
    dynamic_risk::Config config;config.front=params.vehicle_front;config.rear=params.vehicle_rear;
    config.width=params.vehicle_width;config.margin=std::max(.3,params.obstacle_lateral_margin);
    config.stationary_margin=params.obstacle_lateral_margin;
    config.acceleration=params.dynamic_acceleration_mps2;config.horizon=params.dynamic_prediction_sec;
    const double target=std::min(params.target_speed_kph,params.avoid_speed_kph)/3.6;
    for(double fraction:{1.,.5,.25}){
        const auto risk=dynamic_risk::assess(actual,objects,std::max(0.,ego_speed.speed),target*fraction,config);
        if(risk.clear)return true;
        path_block_reason="predicted_collision id="+std::to_string(risk.id)+" t="+std::to_string(risk.ttc);
    }
    return false;
}

bool lineClear(double line, std::vector<FamilyPoint2d>& path) {
    LineCandidateCheck check;check.line=line;
    const auto reject=[&](const std::string& stage){
        ++check.attempts;++check.rejected[stage];check.last_reason=path_block_reason;
    };
    const auto finish=[&](bool passed){
        check.result=passed?"passed":"rejected";
        // At most two checks per grid entry, plus home/current/gap refinements.
        if(line_candidate_checks.size()<128)line_candidate_checks.push_back(check);
        return passed;
    };
    if (!lineAllowed(line)) {reject("line_gate");return finish(false);}
    for (double length_scale : {1.0, 1.2, 1.4, 1.6}) {
        check.length_scale=length_scale;check.lateral_scale=1.;
        std::vector<FamilyPoint2d> converged;
        if (!buildLinePath(line, converged, length_scale, 1.0, false)) {
            path_block_reason="unlimited_path_unavailable";reject("unlimited_build");continue;
        }
        if (!pathClearOfLineObstacles(converged)) {reject("unlimited_collision");continue;}
        for (double lateral_scale : {1.0, 0.85, 0.70, 0.55, 0.40}) {
            check.lateral_scale=lateral_scale;
            if(!buildLinePath(line,path,length_scale,lateral_scale,true)){
                path_block_reason="limited_path_unavailable";reject("limited_build");continue;
            }
            if(!pathInsideRoad(path)){reject("limited_road_or_curvature");continue;}
            if(!pathClearOfLineObstacles(path)){reject("limited_collision");continue;}
            if(!predictedPathRiskClear(path)){reject("predicted_collision");continue;}
            ++check.attempts;path_block_reason.clear();return finish(true);
        }
        // The lateral-step cap can pull an otherwise safe lane-change curve
        // back into the obstacle. Use the complete curve only after checking
        // its actual swept body, road boundary and steering curvature too.
        check.lateral_scale=1.;
        if(pathInsideRoad(converged)&&pathClearOfLineObstacles(converged)&&predictedPathRiskClear(converged)){
            ++check.attempts;path=std::move(converged);
            check.result="full_path_passed";
            if(line_candidate_checks.size()<128)line_candidate_checks.push_back(check);
            path_block_reason.clear();return true;
        }
        reject("full_road_or_collision");
    }
    return finish(false);
}

// 선 line의 왼쪽과 오른쪽에 모두 아직 지나지 않은 장애물이 있으면 그 사이 틈의 중앙 선.
// 한쪽에만 있거나 line이 장애물 안이면 false. 좁은 틈은 한쪽에 붙지 않고 가운데로 지난다.
bool lineGapCentre(double line, double& centre) {
    double left = -std::numeric_limits<double>::infinity();
    double right = std::numeric_limits<double>::infinity();
    for (const auto& obstacle : line_state.obstacles) {
        if (!lineObstacleConfirmed(obstacle) || !lineObstacleInReach(obstacle) ||
            lineObstacleX(obstacle) + obstacle.length + params.vehicle_rear + params.avoid_pass_margin <= 0.0) continue;
        if (obstacle.line_right <= line) left = std::max(left, obstacle.line_right);
        else if (obstacle.line_left >= line) right = std::min(right, obstacle.line_left);
        else return false;
    }
    if (!std::isfinite(left) || !std::isfinite(right)) return false;
    centre = 0.5 * (left + right);
    return true;
}

std::string updateLinePlan(bool new_scan) {
    LineFrame near;
    if (!lineFrameAt(params.planning_start_x, near)) return "five_lines_unavailable";
    if (planning_ready) rememberLineObstacles(new_scan);
    const double now=ros::Time::now().toSec();
    const double previous_target=line_state.target;
    const bool stable_clear=line_state.clear.ready(now,params.line_return_clear_sec);
    // At rest, retain the currently safe target while confirming release.
    // A blocked target may still be replaced by the safe-path search below.
    const bool resume_hold=dynamic_state.hold.active;
    const bool passed_avoidance=line_state.odom>=line_state.pass_odom;
    const double own_home = follower_state.locked_lane == 0 ? 2.0 : 4.0;
    if (line_state.target <= 0.0) line_state.target = own_home;
    if(line_state.commitment_home<=0)line_state.commitment_home=own_home;
    if (params.return_to_entry_lane && line_state.return_home <= 0.0 && change_state.entry_lane >= 0)
        line_state.return_home = change_state.entry_lane == 0 ? 2.0 : 4.0;
    // 회피 중이 아니면(기억한 장애물 없음, 차로 중앙 주행) 차량이 실제로 있는 차로를 내 차선으로 다시 잡는다.
    // 시뮬레이터에서 차량을 다른 차로로 옮기거나 노드를 재시작해도 장애물 없이 옆 차로로 가지 않는다.
    // 진입 차로 복귀가 남아 있으면 그 차로가 아닌 곳으로 다시 잡지 않는다.
    const double vehicle_line = lineOf(near, 0.0);
    if (!resume_hold && passed_avoidance && stable_clear && line_state.obstacles.empty() && std::abs(line_state.target - 3.0) > 0.5) {
        const double vehicle_home = vehicle_line < 3.0 ? 2.0 : 4.0;
        if (vehicle_home != line_state.target && std::abs(vehicle_line - vehicle_home) < 0.5 &&
            (line_state.return_home <= 0.0 || vehicle_home == line_state.return_home)) {
            ROS_INFO("Line selection: vehicle is in lane centre %.0f (line %.2f); own lane follows it",
                     vehicle_home, vehicle_line);
            line_state.target = vehicle_home;
        }
    }
    // 복귀할 차로 중앙: 지금 선에서 가까운 2 또는 4(중앙선 위면 현재 내 차선). 진입 차로 복귀면 진입 차로.
    line_state.lane_home = std::abs(line_state.target - 3.0) < params.epsilon ? own_home :
        (line_state.target < 3.0 ? 2.0 : 4.0);
    line_state.home = line_state.return_home > 0.0 ? line_state.return_home : line_state.lane_home;
    double cover_left = std::numeric_limits<double>::infinity(), cover_right = -cover_left;
    bool ahead = false;
    for (const auto& obstacle : line_state.obstacles) {
        if (!lineObstacleConfirmed(obstacle)) continue;
        cover_left = std::min(cover_left, obstacle.line_left);
        cover_right = std::max(cover_right, obstacle.line_right);
        // 장애물 끝 + 차량 후방 + 여유를 완전히 지나기 전에는 복귀하지 않는다. 옆을 지나는 동안 복귀하면
        // 다시 측정된 장애물에 막혀 선택할 선이 없어진다.
        ahead = ahead || lineObstacleBlocksHome(obstacle);
    }
    char summary[160];
    const auto describe = [&](const char* action) {
        if (std::isfinite(cover_left)) {
            std::snprintf(summary, sizeof(summary), "cover %.2f-%.2f %s line %.2f", cover_left, cover_right,
                          action, line_state.target);
        } else {
            std::snprintf(summary, sizeof(summary), "clear %s line %.2f", action, line_state.target);
        }
        line_decision = summary;
    };
    std::vector<FamilyPoint2d> path;
    double centre = 0.0;
    std::vector<FamilyPoint2d> centre_path;
    // GPS가 돌아오면 통과 여유(avoid_pass_margin)를 기다리지 않고, 경로가 비는 즉시 진입 차로로 복귀한다.
    // 일반 플래너가 이어받을 수 있도록 복귀 대기(최대 recovery_timeout_s) 안에 기준 경로로 돌아가기 위함이다.
    if (!resume_hold && stable_clear && ((!ahead&&passed_avoidance) || gps_return) && std::abs(line_state.target - line_state.home) > params.epsilon &&
        lineClear(line_state.home, path)) {
        // 일반 구간 AVOID의 RETURN과 같다: 장애물 중심을 모두 지났고 차로 중앙 경로가 비어 있으면 복귀.
        ROS_INFO("Line selection: return %.2f -> %.2f%s", line_state.target, line_state.home,
                 gps_return ? " (GPS return)" : "");
        line_state.target = line_state.home;
        describe("return");
    } else if (lineClear(line_state.target, path)) {
        // 양쪽 장애물 사이를 지나는 중이면 틈 중앙으로 맞춘다. 한 칸 이상 벗어났을 때만 옮겨 떨림을 막는다.
        if (!resume_hold && (line_state.changed_at<0||now-line_state.changed_at>=params.line_commit_sec) &&
            lineGapCentre(line_state.target, centre) &&
            std::abs(centre - line_state.target) >= params.avoid_line_step - params.epsilon &&
            lineClear(centre, centre_path)) {
            ROS_INFO("Line selection: %.2f -> gap centre %.2f", line_state.target, centre);
            line_state.target = centre;
            path = centre_path;
            describe("centre to");
        } else {
            describe("keep");
        }
    } else {
        // Do not reverse direction while physically braking. Once at rest a
        // new safe path may be chosen, then must pass the resume confirmation.
        if(dynamic_state.hold.active&&!dynamic_state.hold.resting(now,params.dynamic_rest_sec))
            return "dynamic_wait_for_stop";
        std::vector<double> candidates;
        for(double line=0.;line<=6.+params.epsilon;line+=params.avoid_line_step){
            candidates.push_back(line);
            double gap=0.;if(lineGapCentre(line,gap))candidates.push_back(gap);
        }
        std::sort(candidates.begin(),candidates.end());
        candidates.erase(std::unique(candidates.begin(),candidates.end(),[](double a,double b){return std::abs(a-b)<1e-5;}),candidates.end());
        std::stable_sort(candidates.begin(),candidates.end(),[&](double a,double b){
            return lane_boundary_adapter::decision::lineCost(a,line_state.target,line_state.commitment_home)<
                   lane_boundary_adapter::decision::lineCost(b,line_state.target,line_state.commitment_home);
        });
        bool found=false,waiting_side=false;
        for(double candidate:candidates){
            if(candidate<0||candidate>6)continue;
            if(!lane_boundary_adapter::decision::canSwitchSide(line_state.target,candidate,line_state.commitment_home,
                    now,line_state.changed_at,params.line_commit_sec)){
                if(!waiting_side){
                    std::vector<FamilyPoint2d> deferred;
                    waiting_side=lineClear(candidate,deferred);
                }
                continue;
            }
            if(!lineClear(candidate,path))continue;
            double chosen=candidate,gap=0.;std::vector<FamilyPoint2d> gap_path;
            if(lineGapCentre(candidate,gap)&&
               lane_boundary_adapter::decision::canSwitchSide(line_state.target,gap,line_state.commitment_home,now,line_state.changed_at,params.line_commit_sec)&&
               lineClear(gap,gap_path)){
                chosen=gap;path=std::move(gap_path);
            }else{
                const double side=candidate>=line_state.target?1.:-1.;
                for(double extra=params.avoid_line_step;extra<=params.avoid_line_extra+params.epsilon;extra+=params.avoid_line_step){
                    const double pushed=candidate+side*extra;std::vector<FamilyPoint2d> extra_path;
                    if(pushed<0||pushed>6||
                       !lane_boundary_adapter::decision::canSwitchSide(line_state.target,pushed,line_state.commitment_home,now,line_state.changed_at,params.line_commit_sec)||
                       !lineClear(pushed,extra_path))break;
                    chosen=pushed;path=std::move(extra_path);
                }
            }
            ROS_INFO("Line selection: %.2f blocked -> %.2f (change cost)",line_state.target,chosen);
            line_state.target=chosen;found=true;break;
        }
        if(!found){
            describe("no clear line from");
            if(waiting_side)return "dynamic_wait_for_committed_side";
            return "no_clear_line";
        }
        describe("switch to");
    }
    if(std::abs(line_state.target-previous_target)>params.epsilon)line_state.changed_at=now;
    if(!pathInsideRoad(path)||!pathClearOfLineObstacles(path))return "dynamic_final_path_collision";
    if(std::abs(line_state.target-line_state.home)>params.epsilon){
        for(const auto& obstacle:line_state.obstacles)
            if(lineObstacleConfirmed(obstacle)&&lineObstacleBlocksHome(obstacle))
                line_state.pass_odom=std::max(line_state.pass_odom,line_state.odom+lineObstacleX(obstacle)+
                    obstacle.length+params.vehicle_rear+params.avoid_pass_margin);
    }
    path_block_reason.clear();
    // 고른 선이 속한 차로가 내 차선(초록)이다. 중앙선 위(3)면 이전 차선을 유지한다.
    if (line_state.target < 3.0 - params.epsilon) follower_state.locked_lane = 0;
    else if (line_state.target > 3.0 + params.epsilon) follower_state.locked_lane = 1;
    const LaneChain* own = getPlanningLane(follower_state.locked_lane);
    if (own != nullptr) storePlannedTarget(*own, path);
    else {
        change_state.path = path;
        follower_state.selected_lane.centerline = path;
    }
    // Match the avoidance lookahead even when storePlannedTarget is unavailable.
    const double lookahead=std::abs(line_state.target-line_state.home)>params.epsilon?
        params.avoid_lookahead_distance:params.lookahead_distance;
    for (std::size_t index = 1; index < path.size(); ++index) {
        if (path[index].x < lookahead) continue;
        const double ratio = (lookahead - path[index - 1].x) / (path[index].x - path[index - 1].x);
        follower_state.target_point = interpolatePoint(path[index - 1], path[index], ratio);
        break;
    }
    change_state.target_lane = follower_state.locked_lane;
    return "";
}

// ============================================================================
// 15. 상태 전이: 새 BBOX로 점유 확인, 차선 frame으로 변경 완료 확인
// ============================================================================
std::string startLaneChange(int adjacent_index, const LaneChain& adjacent,
                            const std::vector<FamilyPoint2d>& path, const ros::SteadyTime& now) {
    // 모든 변경 조건이 충족되면 옆 차로를 즉시 내 차로로 지정한다.
    // 변경 중 정지/입력 유실이 생겨도 이 선택을 되돌리지 않는다.
    const int previous_lane = follower_state.locked_lane;
    follower_state.locked_lane = adjacent_index;
    change_state.target_lane = follower_state.locked_lane;
    change_state.changing = true;
    change_state.returning_to_entry = false;
    change_state.return_clear_since = ros::SteadyTime();
    change_state.has_completed = false; // 이번 변경의 완료 여부를 새로 기록한다.
    change_state.change_started = now;
    change_state.timing_updated_at = now;
    change_state.drive_time_sec = change_state.pause_time_sec = 0.0;
    change_state.last_command_driving = false;
    change_state.completion_hits = 0;
    change_state.completion_stamp = ros::Time();
    storePlannedTarget(adjacent, path);
    ROS_INFO("BBOX lane change started: %d -> %d (new own lane, offset %.2fm)",
             previous_lane, follower_state.locked_lane, avoidOffsetFor(adjacent_index));
    return "";
}

std::string updateLaneChangePlan() {
    if (change_state.change_failed) return change_state.change_failure_reason;
    if (change_state.entry_lane < 0) change_state.entry_lane = follower_state.locked_lane;
    const auto now = ros::SteadyTime::now();
    const bool new_scan = change_state.decision_sequence != observationSequence();
    change_state.decision_sequence = observationSequence();
    // 기본: 1~5 기준선 중 따라갈 선(=내 차선)을 고른다. 속도 입력이 없으면(통과 판단 불가) 기존 동작.
    // 기존 방식으로 진행 중인 차선변경/회피가 있으면 그것을 먼저 끝낸다.
    line_mode_active = params.straddle_avoidance && params.avoid_line_straddle && egoSpeedFresh() &&
        !change_state.changing && !avoid_state.active;
    if(params.dynamic_obstacle_stop){
        bool handled=false;
        const auto crossing=crossingYieldPlan(handled);
        if(handled)return crossing;
    }
    if (line_mode_active) return updateLinePlan(new_scan);
    // 속도를 적분할 수 없으면 장애물 옆을 지났는지 알 수 없으므로 회피 중에는 정지한다.
    if (avoid_state.active && !egoSpeedFresh()) return "avoid_ego_speed_timeout";
    if (change_state.changing) {
        const LaneChain* target = getPlanningLane(change_state.target_lane);
        if (target == nullptr) return "target_lane_horizon_missing";
        // 회피 차선변경은 목표 차로에 걸친 장애물을 offset과 경로 검사로 이미 반영했다.
        const bool target_obstacle_planned = avoid_state.active && avoid_state.lane == change_state.target_lane;
        if (!ignoreOccupancyDuringLaneChange()) {
            if (!target_obstacle_planned) {
                if (change_state.occupancy[change_state.target_lane] != BboxResult::NO_DETECTION) {
                    change_state.clear_hits = 0;
                    return "target_lane_detected";
                }
                if (new_scan) change_state.clear_hits = std::min(change_state.clear_hits + 1, params.adjacent_clear_frames);
                // Return initiation already required a full clear interval. After
                // a brief input gap, stop until a *new*, valid target-clear frame
                // arrives, then continue that same checked maneuver. Repeating the
                // initiation dwell after every transient frame gap caused stop/go
                // starvation at the exit. A detected/unknown target still stops.
                const int resume_frames = change_state.returning_to_entry ? 1 : params.adjacent_clear_frames;
                if (change_state.clear_hits < resume_frames) return "confirming_target_lane_after_stop";
            }
        }
        std::vector<FamilyPoint2d> path;
        if (!buildRoadBoundedChangePath(*target, path)) return "lane_change_path_blocked";
        storePlannedTarget(*target, path);

        LaneSample near, ahead;
        samplePlanningLane(*target, params.planning_start_x, near);
        samplePlanningLane(*target, params.planning_start_x + 1.0, ahead);
        const double heading = std::atan2(ahead.center.y - near.center.y, 1.0);
        // 회피 offset이 있으면 밀린 중심에 도달했는지로 완료를 판단한다.
        const double lateral_error = near.center.y + avoidOffsetFor(change_state.target_lane);
        if (follower_state.source_stamp != change_state.completion_stamp) {
            change_state.completion_stamp = follower_state.source_stamp;
            if (std::abs(lateral_error) <= params.completion_lateral_error &&
                std::abs(heading) <= params.completion_heading_deg * PI / 180.0) {
                change_state.completion_hits++;
            } else {
                change_state.completion_hits = 0;
            }
        }
        if (change_state.completion_hits >= params.completion_confirm_frames) {
            // 내 차로는 시작 때 이미 전환했다. 여기서는 차량의 이동 완료만 기록한다.
            change_state.changing = false;
            change_state.has_completed = true;
            change_state.returning_to_entry = false;
            change_state.return_clear_since = ros::SteadyTime();
            change_state.completed_at = now;
            change_state.obstacle_hits = 0;
            change_state.clear_hits = 0;
            ROS_INFO("Lane change complete: following lane %d", follower_state.locked_lane);
            return "";
        }
        return "";
    }

    if (avoid_state.active) {
        // 같은 차로 안 회피 또는 회피 차선변경 완료 후: 통과할 때까지 offset을 유지한다.
        const LaneChain* current = getPlanningLane(follower_state.locked_lane);
        if (current == nullptr) return "current_lane_horizon_missing";
        if (new_scan && planning_ready) {
            double offset = 0.0;
            const AvoidFit fit = laneAvoidOffset(*current, offset, params.avoid_line_straddle);
            if (fit == AvoidFit::kNoRoom) return "avoid_lane_space_insufficient";
            // 가까워질수록 접점 측정이 정확해진다. 같은 방향으로 더 필요한 offset만 반영한다.
            if (fit == AvoidFit::kFits && std::abs(offset) > std::abs(avoid_state.offset) &&
                offset * avoid_state.offset >= 0.0) {
                avoid_state.offset = offset;
                avoid_state.target_line = last_target_line;
            }
            rememberAvoidObstacles(*current);
        }
        // 일반 구간 AVOID의 RETURN: 기억한 모든 장애물의 중심을 지난 뒤 d=0 복귀 경로가 충돌·도로 검사를
        // 통과하면 즉시 복귀한다. 통과 거리 적분(updateAvoidanceProgress)은 그 밖의 해제 경로다.
        const bool passed_centres = std::all_of(avoid_state.obstacles.begin(), avoid_state.obstacles.end(),
            [](const auto& obstacle) {
                return obstacle.x - (avoid_state.odom - obstacle.odom_at) + obstacle.length / 2.0 <= 0.0;
            });
        if (passed_centres && !avoid_state.obstacles.empty()) {
            const double locked_offset = avoid_state.offset;
            avoid_state.offset = 0.0;
            std::vector<FamilyPoint2d> back;
            if (buildRoadBoundedChangePath(*current, back)) {
                ROS_INFO("Straddle avoidance returning to lane %d centre after %.1fm", avoid_state.lane, avoid_state.odom);
                avoid_state = AvoidState{};
                storePlannedTarget(*current, back);
                change_state.target_lane = follower_state.locked_lane;
                return "";
            }
            avoid_state.offset = locked_offset;
        }
        std::vector<FamilyPoint2d> path;
        if (!buildRoadBoundedChangePath(*current, path)) {
            // 시작 직후(거의 이동 전)라면 판단 자체가 맞지 않은 것이므로 풀고 차로 선택을 다시 한다.
            // 이미 진행했다면 장애물이 화면에서 사라졌을 수 있으므로 기억을 유지한 채 정지한다.
            if (avoid_state.odom < params.avoid_reconsider_distance) {
                ROS_WARN("Straddle avoidance in lane %d blocked before moving (%s); re-evaluating lane choice",
                         avoid_state.lane, path_block_reason.c_str());
                avoid_state = AvoidState{};
            }
            return "avoid_path_blocked";
        }
        storePlannedTarget(*current, path);
        change_state.target_lane = follower_state.locked_lane;
        return "";
    }

    const LaneChain* current = getPlanningLane(follower_state.locked_lane);
    if (current == nullptr) return "current_lane_horizon_missing";
    const int adjacent_index = 1 - follower_state.locked_lane;
    // 장애물이 나타나기 전부터 옆 차로의 연속 미검출을 확인해 변경 시작 지연을 줄인다.
    if (new_scan) {
        change_state.clear_hits = change_state.occupancy[adjacent_index] == BboxResult::NO_DETECTION ?
            std::min(change_state.clear_hits + 1, params.adjacent_clear_frames) : 0;
    }
    // LiDAR가 차도 위에서 내 차로를 점유하면 카메라 박스가 없어도(신뢰도 미달·잘림) 점유로 본다.
    const bool lidar_fresh = lidarFresh();
    const bool detected = change_state.occupancy[follower_state.locked_lane] == BboxResult::DETECTED ||
        (params.straddle_avoidance && lidar_fresh && lidarOccupiesLane(*current));
    const bool both_clear = !detected &&
        change_state.occupancy[0] == BboxResult::NO_DETECTION &&
        change_state.occupancy[1] == BboxResult::NO_DETECTION;
    if (!both_clear) change_state.return_clear_since = ros::SteadyTime();
    else if (new_scan && change_state.return_clear_since.isZero()) change_state.return_clear_since = now;
    const bool return_ready = params.return_to_entry_lane && !params.one_lane_change_only &&
        change_state.has_completed && change_state.entry_lane == adjacent_index && both_clear &&
        !change_state.return_clear_since.isZero() &&
        (now - change_state.return_clear_since).toSec() >= 1.0 &&
        (now - change_state.completed_at).toSec() >= params.lane_change_cooldown &&
        change_state.clear_hits >= params.adjacent_clear_frames;
    const bool measurement_ready = params.straddle_avoidance && egoSpeedFresh() && planning_ready &&
        !planning_obstacles.empty();
    // 진입 차로 복귀가 막히면 회피 판단 없이 현재 차로 중심선을 따른다.
    const auto follow_current = [&]() -> std::string {
        std::vector<FamilyPoint2d> path;
        if (!buildPath(*current, false, path) || !pathInsideRoad(path)) return "following_path_outside_road";
        storePlannedTarget(*current, path);
        change_state.target_lane = follower_state.locked_lane;
        return "";
    };
    if (!detected && !return_ready) {
        change_state.obstacle_hits = 0;
        // 내 차로 겹침이 점유 기준(min_contact_overlap_ratio) 미만인 옆 차로 장애물도 넘친 만큼 중심을 민다.
        // 이미 차로 안에 간격 여유가 있으면 offset은 0이고 기존 중심선 추종을 유지한다.
        double offset = 0.0;
        const AvoidFit fit = measurement_ready ? laneAvoidOffset(*current, offset, params.avoid_line_straddle) :
            AvoidFit::kUnknown;
        if (fit == AvoidFit::kNoRoom) {
            avoid_state.hits = 0;
            avoid_state.pending_lane = -1;
            return "avoid_lane_space_insufficient";
        }
        if (fit == AvoidFit::kFits && std::abs(offset) > params.epsilon) {
            if (new_scan) {
                avoid_state.hits = avoid_state.pending_lane == follower_state.locked_lane ? avoid_state.hits + 1 : 1;
                avoid_state.pending_lane = follower_state.locked_lane;
            }
            // 확인 전에는 정지하지 않고 기존 중심선을 따른다. 확인되면 같은 차로 안 회피로 전환한다.
            if (avoid_state.hits >= params.adjacent_clear_frames) {
                startAvoidance({follower_state.locked_lane, offset}, *current);
                std::vector<FamilyPoint2d> path;
                if (!buildRoadBoundedChangePath(*current, path)) {
                    avoid_state = AvoidState{};
                    return "avoid_path_blocked";
                }
                storePlannedTarget(*current, path);
                change_state.target_lane = follower_state.locked_lane;
                return "";
            }
        } else {
            avoid_state.hits = 0;
            avoid_state.pending_lane = -1;
        }
        std::vector<FamilyPoint2d> path;
        if (!buildPath(*current, false, path) || !pathInsideRoad(path)) return "following_path_outside_road";
        storePlannedTarget(*current, path);
        change_state.target_lane = follower_state.locked_lane;
        return "";
    }
    const bool lane_change_allowed = !(change_state.has_completed && params.one_lane_change_only) &&
        !(change_state.has_completed && (now-change_state.completed_at).toSec() < params.lane_change_cooldown);
    // 먼 미확정 박스가 옆 차로에 있을 수 있으므로 그동안은 옆 차로 변경 없이 내 차로 안 회피만 판단한다.
    // 단 5개 기준선 판단은 측정된 장애물로 주행선을 고르므로 그 결과가 옆 차로이면 변경을 허용한다.
    const LaneChain* adjacent_lane = getPlanningLane(adjacent_index);
    const LaneChain* adjacent = adjacent_lane;

    // 측정된 접점으로 덜 덮인 차로와 offset을 고른다. offset 없는 단순 변경은 기존 확인 절차를 따른다.
    // 복귀는 장애물 없는 단순 변경이므로 회피 계획을 거치지 않는다.
    if (measurement_ready && detected) {
        AvoidPlan plan = chooseAvoidPlan(*current, adjacent);
        // 5개 기준선에서 고른 주행선이 옆 차로 안쪽(옆 차로 중앙 ±0.5선)이면 내 차로 offset 회피가 아니라
        // 옆 차로로 변경한다. 차로 소유권·판정 기준 차로를 넘기고 그 차로 안에서 offset을 적용한다.
        const double adjacent_line = follower_state.locked_lane == 0 ? 4.0 : 2.0;
        if (params.avoid_line_straddle && plan.lane == follower_state.locked_lane && adjacent_lane != nullptr &&
            lane_change_allowed && std::abs(last_target_line - adjacent_line) <= 0.5 + params.epsilon) {
            plan.lane = adjacent_index;
            plan.offset = last_adjacent_offset;
        }
        const bool plain_change = plan.lane == adjacent_index && std::abs(plan.offset) <= params.epsilon &&
            change_state.occupancy[adjacent_index] == BboxResult::NO_DETECTION;
        if (plan.lane == follower_state.locked_lane ||
            (plan.lane == adjacent_index && !plain_change && lane_change_allowed)) {
            if (new_scan) {
                avoid_state.hits = plan.lane == avoid_state.pending_lane ? avoid_state.hits + 1 : 1;
                avoid_state.pending_lane = plan.lane;
                change_state.obstacle_hits++;
            }
            if (change_state.obstacle_hits < params.obstacle_confirm_frames ||
                avoid_state.hits < params.adjacent_clear_frames) return "confirming_straddle_avoidance";
            startAvoidance(plan, plan.lane == follower_state.locked_lane ? *current : *adjacent_lane);
            std::vector<FamilyPoint2d> path;
            if (plan.lane == follower_state.locked_lane) {
                if (!buildRoadBoundedChangePath(*current, path)) {
                    avoid_state = AvoidState{};
                    return "avoid_path_blocked";
                }
                storePlannedTarget(*current, path);
                change_state.target_lane = follower_state.locked_lane;
                return "";
            }
            if (!buildRoadBoundedChangePath(*adjacent_lane, path)) {
                avoid_state = AvoidState{};
                return "lane_change_path_blocked";
            }
            return startLaneChange(adjacent_index, *adjacent_lane, path, now);
        }
        // 측정(카메라 접점·LiDAR)으로 두 차로 모두 막혔다고 판정했다. LiDAR 물체를 모르는 기존 절차로
        // 옆 차로에 들어가지 않도록 정지한다.
        if (plan.decided && plan.lane < 0) {
            avoid_state.hits = 0;
            avoid_state.pending_lane = -1;
            return "avoid_no_room_in_road";
        }
    }
    avoid_state.hits = 0;
    avoid_state.pending_lane = -1;

    if (change_state.has_completed && params.one_lane_change_only) return "lane_change_already_completed";
    if (!lane_change_allowed) return "lane_change_cooldown";
    if (new_scan) change_state.obstacle_hits++;

    if (adjacent == nullptr) {
        change_state.clear_hits = 0;
        if (return_ready) return follow_current();
        return "adjacent_lane_horizon_missing";
    }
    std::vector<FamilyPoint2d> path;
    if (change_state.occupancy[adjacent_index] != BboxResult::NO_DETECTION || !buildRoadBoundedChangePath(*adjacent, path)) {
        change_state.clear_hits = 0;
        if (return_ready) return follow_current();
        return "adjacent_lane_detected_or_path_invalid";
    }
    if ((!return_ready && change_state.obstacle_hits < params.obstacle_confirm_frames) ||
        change_state.clear_hits < params.adjacent_clear_frames) return "confirming_lidar_lane_change";
    const std::string result = startLaneChange(adjacent_index, *adjacent, path, now);
    change_state.returning_to_entry = return_ready;
    return result;
}

void validateLaneChangeParameters() {
    if (!std::isfinite(params.clipped_tracking_speed_kph) || params.clipped_tracking_speed_kph <= 0 ||
        params.clipped_tracking_speed_kph > 5) throw std::runtime_error("clipped_tracking_speed_kph must be in (0,5]");
    if (!std::isfinite(params.far_obstacle_speed_kph) || params.far_obstacle_speed_kph < 0 ||
        params.far_obstacle_speed_kph > 5.0) throw std::runtime_error("far_obstacle_speed_kph must be in [0, 5]");
    for (double value : {params.bbox_source_timeout, params.bbox_receive_timeout, params.max_sensor_skew,
         params.obstacle_lateral_margin, params.obstacle_longitudinal_margin, params.vehicle_front,
         params.vehicle_rear, params.vehicle_width, params.planning_start_x, params.path_horizon,
         params.path_sample_step, params.lane_change_length, params.max_lateral_step,
         params.lane_change_speed_kph, params.completion_lateral_error, params.completion_heading_deg,
         params.lane_change_timeout, params.lane_change_pause_timeout, params.lane_change_cooldown}) {
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("lane change parameters must be finite and positive");
    }
    if (params.bbox_topic.empty() || params.reference_lane_topic.empty() || params.obstacle_confirm_frames < 1 ||
        params.adjacent_clear_frames < 1 || params.completion_confirm_frames < 1 ||
        params.lookahead_distance >= params.path_horizon || params.path_horizon >= params.consumer_x_max ||
        params.lane_change_length > params.path_horizon || params.lane_change_length < params.lookahead_distance ||
        params.planning_start_x + 1 >= params.lookahead_distance || params.path_sample_step > 0.5 ||
        params.vehicle_width + 2*params.obstacle_lateral_margin >= params.lane_width_max) {
        throw std::runtime_error("invalid lane change parameter relationships");
    }
    for (double value : {params.ego_speed_timeout, params.avoid_clearance, params.avoid_speed_kph,
         params.avoid_pass_margin, params.avoid_shift_length,
         params.avoid_lookahead_distance}) {
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("avoidance parameters must be finite and positive");
    }
    for (double value : {params.lidar_timeout, params.lidar_max_x, params.lidar_max_length, params.lidar_structure_hold}) {
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("lidar obstacle parameters must be finite and positive");
    }
    if (params.lidar_objects_topic.empty() || params.lidar_min_points < 1 ||
        !std::isfinite(params.lidar_min_height) || params.lidar_min_height < 0 ||
        !std::isfinite(params.lidar_road_min_overlap) || params.lidar_road_min_overlap < 0 ||
        !std::isfinite(params.lidar_lane_min_overlap) || params.lidar_lane_min_overlap < 0) {
        throw std::runtime_error("invalid lidar obstacle parameters");
    }
    if (!std::isfinite(params.avoid_plan_margin) || params.avoid_plan_margin < 0 ||
        !std::isfinite(params.avoid_reconsider_distance) || params.avoid_reconsider_distance < 0 ||
        !std::isfinite(params.avoid_max_offset) || params.avoid_max_offset <= 0 ||
        !std::isfinite(params.avoid_max_shoulder) || params.avoid_max_shoulder < 0 ||
        !std::isfinite(params.avoid_wall_margin) || params.avoid_wall_margin < 0 ||
        !std::isfinite(params.avoid_line_overrun) || params.avoid_line_overrun < 0 ||
        !std::isfinite(params.avoid_line_extra) || params.avoid_line_extra < 0.0 || params.avoid_line_extra > 2.0 ||
        !std::isfinite(params.line_edge_expand) || params.line_edge_expand < 0.0 || params.line_edge_expand > 0.5 ||
        !std::isfinite(params.avoid_line_step) || params.avoid_line_step < 0.05 || params.avoid_line_step > 1.0 ||
        !std::isfinite(params.lane_hold_distance) || params.lane_hold_distance < 0 ||
        !std::isfinite(params.lane_hold_time) || params.lane_hold_time < 0 ||
        !std::isfinite(params.imu_timeout) || params.imu_timeout <= 0 || params.imu_topic.empty() ||
        !std::isfinite(params.speed_kp) || params.speed_kp < 0 || !std::isfinite(params.speed_ki) || params.speed_ki < 0 ||
        !std::isfinite(params.speed_integral_limit) || params.speed_integral_limit < 0 ||
        !std::isfinite(params.max_accel_pedal) || params.max_accel_pedal <= 0 || params.max_accel_pedal > 1 ||
        !std::isfinite(params.max_brake_pedal) || params.max_brake_pedal <= 0 || params.max_brake_pedal > 1 ||
        !std::isfinite(params.avoid_lane_tie) || params.avoid_lane_tie < 0 || params.ego_topic.empty() ||
        params.avoid_speed_kph > params.lane_change_speed_kph || params.avoid_shift_length > params.path_horizon ||
        params.avoid_lookahead_distance <= params.planning_start_x + 1 ||
        params.avoid_lookahead_distance >= params.path_horizon) {
        throw std::runtime_error("invalid avoidance parameter relationships");
    }
}

}  // namespace
