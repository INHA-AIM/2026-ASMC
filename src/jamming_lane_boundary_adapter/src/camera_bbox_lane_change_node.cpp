// 운영 기본은 기존 BBOX 차로 변경. 선택한 실험에서는 같은 제어기에 local OBB·접점 검사를 연결한다.
// 이 파일에 파라미터, ROS callback, 주행 함수, main을 함께 둔다.
#include "lane_boundary_adapter/boundary_family_adapter.hpp"
#include "lane_boundary_adapter/contact_clearance.hpp"
#include "lane_boundary_adapter/drivable_clearance.hpp"
#include <sensor_msgs/Image.h>
#include <memory>

#include <ros/ros.h>
#include <lane_boundary_adapter/CameraBBoxLaneResult.h>
#include <std_msgs/Int8.h>
#include <asmc_msgs/LaneBoundaryFamily.h>
#include <limits>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PointStamped.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <stdexcept>
#include <string>
#include <vector>
#include <deque>

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
    // Internal trial profile. Legacy lane-occupancy policy is the recovery path.
    bool contact_clearance_enabled = false;
    bool contact_local_required = false;
    std::string bev_mode="off"; // off / audit / guard; internal experiment only
    double contact_speed_kph = 5.0;
    double contact_max_skew = 0.15; // no ego-motion alignment is claimed

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

    // BBOX 차로 판정: 실제 거리나 장애물 크기를 만들지 않는다.
    std::string bbox_topic = "/planning/camera_bbox_lane/result"; // 앞서 만든 판정 결과.
    std::string reference_lane_topic = "/planning/camera_bbox_lane_change/reference_lane"; // 변경 중 기준 차로 유지.
    double clipped_tracking_speed_kph = 5.0; // [km/h] 이미 확인한 장애물의 잘린 BBOX를 잠시 추적할 때 속도 상한.
    double far_obstacle_speed_kph = 5.0; // [km/h] 먼 BBOX의 차로 미확정 시 접근 속도 상한. 0이면 정지.
    double bbox_source_timeout = 0.5;      // [s] BBOX 원본 영상 나이 상한.
    double bbox_receive_timeout = 0.5;     // [s] 새 BBOX 판정 미수신 상한.
    double max_sensor_skew = 0.4;          // [s] 현재 차선과 BBOX/판정에 사용한 차선 stamp 차이.
    int obstacle_confirm_frames = 2;      // 내 차로 검출을 연속 확인할 서로 다른 BBOX frame 수.
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
    bool monotonic_x = false;  // validated once after joining; legacy fallback otherwise.
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
// BBOX / Lane Change State: 차로 유지 -> 점유 확인 -> 변경 -> 새 차로 유지
// ============================================================================
struct LaneChangeState {
    std::vector<LaneChain> lanes;
    std::vector<FamilyPoint2d> path;
    ros::Time bbox_stamp, bbox_lane_stamp;
    ros::SteadyTime bbox_received_at;
    bool bbox_received = false;
    bool bbox_valid = false;
    bool deferred_far = false;
    bool clipped_association = false;
    std::string bbox_error = "waiting_for_bbox";
    int occupancy[2] = {BboxResult::UNOBSERVED, BboxResult::UNOBSERVED};
    unsigned long bbox_sequence = 0;
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
} change_state;

BboxResult::ConstPtr contact_result;
struct ContactLaneFrame { ros::Time stamp; std::vector<LaneChain> lanes; };
std::deque<ContactLaneFrame> contact_lane_history; // bounded, only accepted geometry

std::string contact_decision = "disabled";
unsigned contact_candidates = 0, contact_road_rejected=0, contact_collision_rejected=0;
bool contact_geometry_ready = false;
double contact_plan_wall_ms = 0.;
bool contact_plan_evaluated=false;
double contact_pursuit_x=4.0;
double contact_maneuver_length=0.;
// Competition CAN speed remains valid in GPS shadow; never use its zeroed map
// pose. The geometric trial is limited to low ACTUAL speed, not just a command.
ros::Time contact_speed_stamp;
ros::SteadyTime contact_speed_receipt;
bool contact_speed_ready=false;
double contact_actual_speed=std::numeric_limits<double>::quiet_NaN();
void contactSpeedCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& m) {
    contact_actual_speed=std::isfinite(m->velocity.x)?std::abs(m->velocity.x):std::numeric_limits<double>::quiet_NaN();
    contact_speed_stamp=m->header.stamp;contact_speed_receipt=ros::SteadyTime::now();
}

struct BevEntry {
    ros::Time stamp;ros::SteadyTime received;
    std::shared_ptr<lane_boundary_adapter::clearance::DrivableClearance> grid;
};
std::deque<sensor_msgs::Image::ConstPtr> bev_probability,bev_coverage;
std::deque<BevEntry> bev_frames;
lane_boundary_adapter::clearance::GridSpec bev_spec;
ros::Time bev_last_stamp;
std::string bev_status="off";
unsigned bev_rejected=0,bev_invalid=0;
double bev_pair_skew=0.;
void clearBev(){bev_probability.clear();bev_coverage.clear();bev_frames.clear();bev_last_stamp=ros::Time();}
void bevInput(const sensor_msgs::Image::ConstPtr& m,bool probability) {
    const auto now=ros::Time::now();const double age=(now-m->header.stamp).toSec();
    if(m->header.stamp.isZero() || age<-.05 || age>.35 || m->header.frame_id!="base_link" || m->encoding!="mono8" ||
       m->width!=unsigned(bev_spec.cols)||m->height!=unsigned(bev_spec.rows)||m->step<m->width||m->data.size()!=size_t(m->height)*m->step){++bev_invalid;return;}
    if(!bev_last_stamp.isZero() && m->header.stamp<bev_last_stamp){clearBev();++bev_invalid;return;}
    if(m->header.stamp==bev_last_stamp)return;
    auto& queue=probability?bev_probability:bev_coverage;queue.push_back(m);while(queue.size()>4)queue.pop_front();
    for(const auto& p:bev_probability)for(const auto& c:bev_coverage)if(p->header.stamp==c->header.stamp){
        if(p->header.stamp<=bev_last_stamp)continue;
        std::vector<uint8_t> a,b;a.reserve(p->width*p->height);b.reserve(a.capacity());
        for(unsigned r=0;r<p->height;++r){a.insert(a.end(),p->data.begin()+r*p->step,p->data.begin()+r*p->step+p->width);
            b.insert(b.end(),c->data.begin()+r*c->step,c->data.begin()+r*c->step+c->width);}
        auto grid=std::make_shared<lane_boundary_adapter::clearance::DrivableClearance>();
        const lane_boundary_adapter::clearance::Vehicle vehicle{params.vehicle_front,params.vehicle_rear,params.vehicle_width/2,
            params.obstacle_lateral_margin,params.obstacle_longitudinal_margin};
        if(!grid->prepare(bev_spec,a,b,vehicle)){++bev_invalid;return;}
        bev_frames.push_back({p->header.stamp,ros::SteadyTime::now(),grid});while(bev_frames.size()>8)bev_frames.pop_front();
        bev_last_stamp=p->header.stamp;
    }
}
std::shared_ptr<lane_boundary_adapter::clearance::DrivableClearance> selectBev(ros::Time image,ros::Time lane) {
    std::shared_ptr<lane_boundary_adapter::clearance::DrivableClearance> best;double distance=.050000001;
    for(const auto& e:bev_frames){double age=(ros::Time::now()-e.stamp).toSec();
        if(age<-.05||age>.35||(ros::SteadyTime::now()-e.received).toSec()>.35||std::abs((lane-e.stamp).toSec())>.050000001)continue;
        double d=std::abs((image-e.stamp).toSec());if(d<distance){distance=d;best=e.grid;}}
    bev_pair_skew=best?distance:-1.;return best;
}

ros::Subscriber bbox_subscriber;
ros::Subscriber mode_subscriber;
ros::Publisher reference_lane_publisher;
bool mode_active = false;
bool mode_received = false;
bool await_activation_bbox = false;

void bboxCallback(const BboxResult::ConstPtr& message);
std::string bboxStopReason();
bool ignoreBboxDuringLaneChange();
std::string updateLaneChangePlan();
std::string updateContactPathPlan();
const LaneChain* getPlanningLane(int lane_index, const std::vector<LaneChain>* geometry = nullptr, double required_x = -1.0);
bool samplePlanningLane(const LaneChain& lane, double x, LaneSample& sample);
bool buildPath(const LaneChain& lane, bool changing, std::vector<FamilyPoint2d>& path, double lateral_scale = 1.0, double change_length = 0.0, bool full_shift = false, double supported_end = -1.0);
bool buildRoadBoundedChangePath(const LaneChain& lane, std::vector<FamilyPoint2d>& path);
// Optional native probe evidence. The live decision and arithmetic are identical
// with a null report; offline analysis need not reproduce the road check.
struct RoadCheckFailure {
    std::string reason;
    double x=0., y=0., left=0., right=0., curvature_limit=0.;
};
bool pathInsideRoad(const std::vector<FamilyPoint2d>& path, const std::vector<LaneChain>* geometry = nullptr,
                    RoadCheckFailure* failure = nullptr);
void storePlannedTarget(const LaneChain& lane, const std::vector<FamilyPoint2d>& path);
void validateLaneChangeParameters();
void modeCallback(const std_msgs::UInt8::ConstPtr& message);


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
            if (mode_active) {
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
    follower_state.last_steering = 0.0;
    follower_state.last_control_time = ros::SteadyTime::now();

    contact_lane_history.clear();
    contact_speed_ready=false;
    // Keep latest camera/BBOX observations, but never credit frames received
    // before activation toward a new lane-change decision.
    change_state.path.clear();
    change_state.decision_sequence = change_state.bbox_sequence;
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
    // The classifier may still hold a recently published reference lane from
    // the previous driving episode. Discard labels made against that reference.
    change_state.bbox_valid = false;
    change_state.bbox_error = "waiting_for_bbox_after_activation";
    await_activation_bbox = true;
    std_msgs::Int8 automatic_lane;
    automatic_lane.data = -1;
    reference_lane_publisher.publish(automatic_lane);
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
    clearBev();
    if (mode_active) {
        resetActuationLifecycle();
    } else {
        std_msgs::Int8 automatic_lane;
        automatic_lane.data = -1;
        reference_lane_publisher.publish(automatic_lane);
    }
    ROS_WARN("Jamming control lifecycle %s", mode_active ? "ACTIVE" : "INACTIVE");
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

bool ignoreBboxDuringLaneChange() {
    // A voluntary return always keeps live target-obstacle/freshness checks.
    return !params.contact_clearance_enabled && params.ignore_bbox_during_lane_change && change_state.changing &&
           !change_state.returning_to_entry;
}

void cameraLaneFollowingProcess() {
    // 1. 제어 주기 계산: 루프가 늦어져도 조향 변화량이 갑자기 커지지 않도록 제한.
    const ros::SteadyTime now = ros::SteadyTime::now();
    const double elapsed = (now - follower_state.last_control_time).toSec();
    const double dt = std::clamp(elapsed, params.epsilon, params.max_steering_dt);
    follower_state.last_control_time = now;

    contact_plan_evaluated=false;
    updateLaneChangeTiming(now);

    // 2. 입력 상태와 비상정지 확인. 추종할 수 없으면 기본 명령은 제동.
    std::string stop_reason = getStopReason(now);
    if (change_state.change_failed) stop_reason = change_state.change_failure_reason;
    if (stop_reason.empty()) {
        stop_reason = bboxStopReason();
    }
    if (stop_reason.empty() && follower_state.target_ready) {
        const auto plan_started=ros::SteadyTime::now();
        stop_reason = params.contact_clearance_enabled ? updateContactPathPlan() : updateLaneChangePlan();
        contact_plan_wall_ms=(ros::SteadyTime::now()-plan_started).toSec()*1000.;
        contact_plan_evaluated=params.contact_clearance_enabled;
    } else {
        change_state.obstacle_hits = 0;
        change_state.clear_hits = 0;
        change_state.return_clear_since = ros::SteadyTime();
    }
    if (!stop_reason.empty()) change_state.completion_hits = 0;
    const bool tracking = stop_reason.empty() && follower_state.target_ready;
    morai_msgs::CtrlCmd command = makeStopCommand();

    // 3. 유효한 목표점이 있을 때만 조향 / 속도를 계산.
    if (tracking) {
        command = makeFollowingCommand(dt);
        publishTargetPoint();
    }

    change_state.last_command_driving = change_state.changing && tracking &&
                                        command.velocity > params.epsilon;

    // 목표 선택 직후 기준 차로도 전환한다. 점유는 항상 기하 차로 0/1로 변환해 사용한다.
    if (follower_state.locked_lane >= 0) {
        std_msgs::Int8 reference;
        reference.data = follower_state.locked_lane;
        reference_lane_publisher.publish(reference);
    }

    // 4. 제어 결과와 RViz / 진단 정보 발행.
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
    if(params.contact_clearance_enabled) target_speed=std::min(target_speed,params.contact_speed_kph);
    // 변경 중에는 BBOX 때문에 속도 0이나 감속 명령이 생기지 않도록 한다.
    if (!ignoreBboxDuringLaneChange()) {
        if (change_state.clipped_association) target_speed = std::min(target_speed, params.clipped_tracking_speed_kph);
        if (change_state.deferred_far) target_speed = std::min(target_speed, params.far_obstacle_speed_kph);
    }

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

morai_msgs::CtrlCmd makeFollowingCommand(double dt) {
    // 목표 속도가 0이면 제동 명령을 유지하고, 조향 계산 상태만 갱신한다.
    morai_msgs::CtrlCmd command = makeStopCommand();
    const double curvature = calculateTargetCurvature();
    const double steering = calculateSteering(curvature, dt);
    const double target_speed = calculateTargetSpeed(curvature);

    if (target_speed > 0.0) {
        command = morai_msgs::CtrlCmd();
        command.longlCmdType = 2;  // MORAI 내장 속도 제어.
        command.velocity = target_speed;  // [km/h]
        command.steering = steering;      // [rad], 기존 MPC / bridge와 동일.
    }

    return command;
}

void publishControlCommand(const morai_msgs::CtrlCmd& command) {
    ros_interface.preview_pub.publish(command);

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
    if(params.contact_clearance_enabled) {
        contact_lane_history.push_back({msg->header.stamp,lanes});
        while(contact_lane_history.size()>8)contact_lane_history.pop_front();
    }
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
    contact_lane_history.clear();
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

    for(auto& lane:lanes) lane.monotonic_x=std::is_sorted(lane.centerline.begin(),lane.centerline.end(),
        [](const FamilyPoint2d&a,const FamilyPoint2d&b){return a.x<b.x;});
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

    // Preserve the previous epsilon comparison and arithmetic exactly. A
    // joined, non-monotonic legacy chain keeps its original linear selection.
    std::size_t begin=1;
    if(lane.monotonic_x){
        auto it=std::lower_bound(lane.centerline.begin()+1,lane.centerline.end(),target_x,
            [](const FamilyPoint2d&p,double x){return x>p.x+params.epsilon;});
        begin=std::distance(lane.centerline.begin(),it);
    }
    for (std::size_t i = begin; i < lane.centerline.size(); ++i) {
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
    status.hardware_id = "camera_bbox";
    status.level = tracking && !change_state.deferred_far && !ignoreBboxDuringLaneChange() ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = tracking ? (ignoreBboxDuringLaneChange() ? "changing_without_bbox_stop" :
        (change_state.deferred_far ? "approaching_unresolved_far_obstacle" : "following")) : reason;
    addDiagnosticValue(status, "bbox_stop_suppressed", ignoreBboxDuringLaneChange() ? "true" : "false");
    addDiagnosticValue(status, "bbox_input_error", change_state.bbox_error);

    addDiagnosticValue(status, "enable_control", params.enable_control ? "true" : "false");
    addDiagnosticValue(status, "mode_received", mode_received ? "true" : "false");
    addDiagnosticValue(status, "mode_active", mode_active ? "true" : "false");
    addDiagnosticValue(status, "active_lane", std::to_string(follower_state.locked_lane));
    addDiagnosticValue(status, "clipped_obstacle_tracking", change_state.clipped_association ? "true" : "false");
    addDiagnosticValue(status, "locked_lane", std::to_string(follower_state.locked_lane));
    addDiagnosticValue(status, "entry_lane", std::to_string(change_state.entry_lane));
    addDiagnosticValue(status, "returning_to_entry", change_state.returning_to_entry ? "true" : "false");
    addDiagnosticValue(status, "source_age_sec", std::to_string(source_age));
    addDiagnosticValue(status, "lookahead_distance", std::to_string(params.lookahead_distance));
    addDiagnosticValue(status, "target_y_m", tracking ? std::to_string(follower_state.target_point.y) : "unavailable");
    addDiagnosticValue(status, "velocity_kph", std::to_string(command.velocity));
    addDiagnosticValue(status, "steering_rad", std::to_string(command.steering));
    addDiagnosticValue(status, "inferred", tracking && follower_state.selected_lane.contains_inferred ? "true" : "false");

    addDiagnosticValue(status, "change_state", change_state.changing ? "changing" : "following_lane");
    addDiagnosticValue(status, "target_lane", std::to_string(change_state.target_lane));
    addDiagnosticValue(status, "bbox_valid", change_state.bbox_valid ? "true" : "false");
    addDiagnosticValue(status, "far_obstacle_unresolved", change_state.deferred_far ? "true" : "false");
    addDiagnosticValue(status, "bbox_age_sec", change_state.bbox_received ?
        std::to_string((ros::Time::now() - change_state.bbox_stamp).toSec()) : "unavailable");
    addDiagnosticValue(status, "lane0_state", std::to_string(change_state.occupancy[0]));
    addDiagnosticValue(status, "lane1_state", std::to_string(change_state.occupancy[1]));
    addDiagnosticValue(status, "obstacle_confirmations", std::to_string(change_state.obstacle_hits));
    addDiagnosticValue(status, "adjacent_clear_confirmations", std::to_string(change_state.clear_hits));
    addDiagnosticValue(status, "change_completed", change_state.has_completed ? "true" : "false");
    addDiagnosticValue(status, "change_drive_time_sec", std::to_string(change_state.drive_time_sec));
    addDiagnosticValue(status, "change_pause_time_sec", std::to_string(change_state.pause_time_sec));
    addDiagnosticValue(status, "uses_obstacle_distance", params.contact_clearance_enabled?"image_contact_and_observed_local_footprint":"false");
    addDiagnosticValue(status, "contact_clearance_enabled", params.contact_clearance_enabled?"true":"false");
    addDiagnosticValue(status, "contact_decision", contact_decision);
    addDiagnosticValue(status,"bev_mode",params.bev_mode);
    addDiagnosticValue(status,"bev_status",bev_status);
    addDiagnosticValue(status,"bev_rejected",std::to_string(bev_rejected));
    addDiagnosticValue(status,"bev_invalid",std::to_string(bev_invalid));
    addDiagnosticValue(status,"bev_pair_skew_s",std::to_string(bev_pair_skew));
    addDiagnosticValue(status,"contact_plan_evaluated",contact_plan_evaluated?"true":"false");
    addDiagnosticValue(status, "contact_plan_wall_ms",std::to_string(contact_plan_wall_ms));
    addDiagnosticValue(status, "contact_actual_speed_mps",std::isfinite(contact_actual_speed)?std::to_string(contact_actual_speed):"unavailable");
    addDiagnosticValue(status,"contact_maneuver_length_m",std::to_string(contact_maneuver_length));
    addDiagnosticValue(status, "contact_candidates", std::to_string(contact_candidates));
    addDiagnosticValue(status, "contact_road_rejected", std::to_string(contact_road_rejected));
    addDiagnosticValue(status, "contact_collision_rejected", std::to_string(contact_collision_rejected));
    addDiagnosticValue(status, "effective_pursuit_x_m", std::to_string(params.contact_clearance_enabled?contact_pursuit_x:params.lookahead_distance));
    addDiagnosticValue(status, "contact_lane_history_size",std::to_string(contact_lane_history.size()));
    addDiagnosticValue(status, "contact_geometry_stamp_ns",contact_result?std::to_string(contact_result->lane_stamp.toNSec()):"unavailable");
    addDiagnosticValue(status, "contact_geometry_ready", contact_geometry_ready?"true":"false");

    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header.stamp = ros::Time::now();
    diagnostics.status.push_back(status);
    ros_interface.diagnostic_pub.publish(diagnostics);

    if (!tracking) {
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

    ros::NodeHandle("~").param<std::string>("bev_mode",params.bev_mode,"off");
    if(params.bev_mode!="off" && params.bev_mode!="audit" && params.bev_mode!="guard")throw std::runtime_error("invalid bev_mode");
    if(params.bev_mode!="off") {
        if(!params.contact_clearance_enabled || !params.contact_local_required)throw std::runtime_error("BEV trial requires measured local clearance");
        double xmin,ymin;std::string frame;
        if(!nh.getParam("/camera_bev/x_max_m",bev_spec.x_max)||!nh.getParam("/camera_bev/y_max_m",bev_spec.y_max)||
           !nh.getParam("/camera_bev/x_min_m",xmin)||!nh.getParam("/camera_bev/y_min_m",ymin)||
           !nh.getParam("/camera_bev/resolution_m",bev_spec.resolution)||!nh.getParam("/camera_bev/frame_id",frame)||
           frame!="base_link"||!std::isfinite(bev_spec.resolution)||bev_spec.resolution<=0.||
           !std::isfinite(xmin)||!std::isfinite(ymin)||!std::isfinite(bev_spec.x_max)||!std::isfinite(bev_spec.y_max))
            throw std::runtime_error("canonical camera BEV grid unavailable");
        bev_spec.rows=std::lround((bev_spec.x_max-xmin)/bev_spec.resolution);
        bev_spec.cols=std::lround((bev_spec.y_max-ymin)/bev_spec.resolution);
        if(bev_spec.rows<1||bev_spec.rows>1000||bev_spec.cols<1||bev_spec.cols>1000)throw std::runtime_error("invalid camera BEV grid dimensions");
        static ros::Subscriber probability,coverage;
        probability=nh.subscribe<sensor_msgs::Image>("/perception/bev/camera/drivable_probability",1,
            +[](const sensor_msgs::Image::ConstPtr& m){bevInput(m,true);});
        coverage=nh.subscribe<sensor_msgs::Image>("/perception/bev/camera/coverage",1,
            +[](const sensor_msgs::Image::ConstPtr& m){bevInput(m,false);});
    }
    bbox_subscriber = nh.subscribe(params.bbox_topic, 1, bboxCallback);
    reference_lane_publisher = nh.advertise<std_msgs::Int8>(params.reference_lane_topic, 1);
    follower_state.locked_lane = params.lane_index;
    ros_interface.preview_pub = nh.advertise<morai_msgs::CtrlCmd>(params.preview_topic, 1);
    if (params.enable_control) {
        ros_interface.control_pub = nh.advertise<morai_msgs::CtrlCmd>(params.control_topic, 1);
    }
    ros_interface.target_pub = nh.advertise<geometry_msgs::PointStamped>(params.target_topic, 1);
    ros_interface.marker_pub = nh.advertise<visualization_msgs::MarkerArray>(params.markers_topic, 1);
    ros_interface.diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(params.diagnostics_topic, 1);
    static ros::Subscriber contact_speed_sub;
    if(params.contact_clearance_enabled) contact_speed_sub=nh.subscribe("/Ego_topic",1,contactSpeedCallback);
    ros_interface.lane_sub = nh.subscribe(params.input_topic, params.contact_clearance_enabled?8:1, laneBoundaryCallback);
    ros_interface.stop_sub = nh.subscribe(params.emergency_stop_topic, 1, emergencyStopCallback);
    mode_subscriber = nh.subscribe("/planning/mode", 1, modeCallback);
    follower_state.last_control_time = ros::SteadyTime::now();

    ROS_WARN("Camera lane follower: control=%s lookahead_x=%.2fm speed=%.1fkm/h. "
             "BBOX lane change enabled (no distance/clearance estimate); stop other /ctrl_cmd controllers.",
             params.enable_control ? "ENABLED" : "preview only",
             params.lookahead_distance, params.target_speed_kph);
}

void handleShutdownSignal(int) {
    shutdown_requested = 1;
}

void loadParameters(ros::NodeHandle& private_nh) {
    private_nh.param("contact_clearance_enabled",params.contact_clearance_enabled,false);
    private_nh.param("contact_local_required",params.contact_local_required,false);
    private_nh.param("contact_speed_kph",params.contact_speed_kph,5.0);
    if(!std::isfinite(params.contact_speed_kph)||params.contact_speed_kph<=0||params.contact_speed_kph>5.0)
        throw std::runtime_error("contact_speed_kph must be in (0,5]");
    private_nh.param("bbox_topic", params.bbox_topic, params.bbox_topic);
    private_nh.param("reference_lane_topic", params.reference_lane_topic, params.reference_lane_topic);
    private_nh.param("bbox_source_timeout", params.bbox_source_timeout, params.bbox_source_timeout);
    private_nh.param("bbox_receive_timeout", params.bbox_receive_timeout, params.bbox_receive_timeout);
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
// 11. BBOX 입력: 상대 라벨을 고정된 기하 차로 index(0/1)의 점유 상태로 변환.
// ============================================================================
void bboxCallback(const BboxResult::ConstPtr& message) {
    const auto reject = [](const std::string& reason) {
        change_state.bbox_valid = false;
        change_state.bbox_error = reason;
        change_state.obstacle_hits = change_state.clear_hits = 0;
        change_state.return_clear_since = ros::SteadyTime();
    };
    const auto stamp = message->header.stamp;
    const double age = (ros::Time::now() - stamp).toSec();
    if (stamp.isZero() || age < -params.future_tolerance || age > params.bbox_source_timeout) {
        reject("bbox_timestamp_expired"); return;
    }
    // The experimental geometric path may resolve a tiny lane-overlap ratio,
    // but cannot reinterpret stale/projection/mask/clipping failures as clear.
    bool contact_resolvable = params.contact_clearance_enabled && message->contact_projection_valid &&
        message->ground_contacts.size()==message->boxes.size() &&
        message->ground_contact_valid.size()==message->boxes.size() &&
        message->assignments.size()==message->boxes.size() && message->box_reasons.size()==message->boxes.size();
    for(std::size_t i=0;contact_resolvable && i<message->boxes.size();++i)
        contact_resolvable=message->ground_contact_valid[i] &&
            (message->assignments[i]!=BboxResult::UNKNOWN ||
             message->box_reasons[i].find("contact_overlap_too_small:")==0 ||
             (message->box_reasons[i].find("local_lidar_supported:")==0 && message->local_obstacles_valid &&
              message->ground_contact_sources.size()==message->boxes.size() &&
              message->ground_contact_sources[i]=="lidar_observed" &&
              std::abs((message->header.stamp-message->local_obstacles_stamp).toSec())<=.050000001));
    // classifier timeout/UNKNOWN은 같은 stamp여도 즉시 거부한다.
    if (!message->valid && !contact_resolvable) {
        // 원인은 보존한다. 변경 중 제동 제외 여부는 제어 단계에서 결정한다.
        std::string detail = message->reason;
        for (std::size_t i = 0; i < message->assignments.size() && i < message->box_reasons.size(); ++i) {
            if (message->assignments[i] == BboxResult::UNKNOWN) {
                detail += ":" + message->box_reasons[i];
                break;
            }
        }
        reject("bbox_classification_invalid:" + detail);
        return;
    }
    if (change_state.bbox_received && stamp == change_state.bbox_stamp) return;
    if (change_state.bbox_received && stamp < change_state.bbox_stamp) { reject("bbox_timestamp_regression"); return; }
    const double lane_age = (ros::Time::now() - message->lane_stamp).toSec();
    if (message->own_lane < 0 || message->own_lane > 1 || message->lane_stamp.isZero() ||
        lane_age < -params.future_tolerance || lane_age > params.source_timeout ||
        std::abs((stamp-message->lane_stamp).toSec()) > params.max_sensor_skew ||
        message->assignments.size() != message->boxes.size() || message->box_reasons.size() != message->boxes.size() || message->clipped_association.size() != message->boxes.size()) {
        reject("invalid_bbox_lane_contract"); return;
    }
    if (await_activation_bbox &&
        (follower_state.locked_lane < 0 ||
         message->own_lane != follower_state.locked_lane)) {
        reject("waiting_for_bbox_matching_observed_lane"); return;
    }
    int own = BboxResult::NO_DETECTION, adjacent = BboxResult::NO_DETECTION;
    bool deferred_far = false;
    bool clipped_association = false;
    for (std::size_t i=0; i<message->assignments.size(); ++i) {
        const auto label = message->assignments[i];
        if (message->clipped_association[i] && label != BboxResult::OWN && label != BboxResult::ADJACENT) {
            reject("invalid_clipped_bbox_contract"); return;
        }
        clipped_association |= message->clipped_association[i];
        if ((label == BboxResult::UNKNOWN && !contact_resolvable) || label > BboxResult::DEFERRED_FAR) { reject("bbox_lane_unknown"); return; }
        deferred_far |= label == BboxResult::DEFERRED_FAR;
        if (label == BboxResult::OWN || label == BboxResult::BOTH) own = BboxResult::DETECTED;
        if (label == BboxResult::ADJACENT || label == BboxResult::BOTH) adjacent = BboxResult::DETECTED;
    }
    if (deferred_far) {
        if (own != BboxResult::DETECTED) own = BboxResult::UNOBSERVED;
        if (adjacent != BboxResult::DETECTED) adjacent = BboxResult::UNOBSERVED;
    }
    if (!contact_resolvable && (message->own_state != own || message->adjacent_state != adjacent)) { reject("bbox_occupancy_mismatch"); return; }
    contact_result=message;
    change_state.deferred_far = deferred_far;
    change_state.clipped_association = clipped_association;
    change_state.occupancy[message->own_lane] = own;
    change_state.occupancy[1-message->own_lane] = adjacent;
    change_state.bbox_stamp = stamp;
    change_state.bbox_lane_stamp = message->lane_stamp;
    change_state.bbox_received_at = ros::SteadyTime::now();
    change_state.bbox_received = change_state.bbox_valid = true;
    change_state.bbox_error.clear();
    change_state.bbox_sequence++;
    await_activation_bbox = false;
}

std::string bboxStopReason() {
    // 차선 입력·경로·비상정지·변경 시간 제한은 별도 검사에서 계속 적용한다.
    if (ignoreBboxDuringLaneChange()) return "";
    if(params.contact_clearance_enabled) {
        const double speed_age=(ros::Time::now()-contact_speed_stamp).toSec();
        if(contact_speed_stamp.isZero() || !std::isfinite(contact_actual_speed) || speed_age<-.05 || speed_age>.35 ||
           (ros::SteadyTime::now()-contact_speed_receipt).toSec()>.35){contact_speed_ready=false;return "contact_actual_speed_unavailable";}
        // Entry <=5.76 km/h; retain up to7.2 for the speed servo's transient.
        // The command remains capped at5. GPS shadow never licenses25 km/h
        // entry into this low-speed experiment. No stale velocity is reused.
        if(contact_actual_speed>(contact_speed_ready?2.0:1.6)){contact_speed_ready=false;return "contact_actual_speed_above_trial_limit";}
        contact_speed_ready=true;
        if(params.contact_local_required && (!contact_result || !contact_result->local_obstacles_valid))
            return "contact_local_obstacles_unavailable";
    }
    if(params.contact_clearance_enabled && (!contact_result || !contact_result->contact_projection_valid))
        return "contact_projection_unavailable";
    if (!change_state.bbox_valid) return change_state.bbox_error;
    if ((ros::SteadyTime::now()-change_state.bbox_received_at).toSec() > params.bbox_receive_timeout) return "bbox_receive_timeout";
    const double age = (ros::Time::now()-change_state.bbox_stamp).toSec();
    if (age < -params.future_tolerance || age > params.bbox_source_timeout) return "bbox_timestamp_expired";
    if (std::abs((follower_state.source_stamp-change_state.bbox_stamp).toSec()) > params.max_sensor_skew ||
        std::abs((follower_state.source_stamp-change_state.bbox_lane_stamp).toSec()) > params.max_sensor_skew) return "camera_bbox_timestamp_skew";
    return "";
}

// ============================================================================
// 12. 공통 경로 생성·도로 경계 검사 (실험 경로의 장애물 검사는 별도 clearance 코어)
// ============================================================================
const LaneChain* getPlanningLane(int lane_index, const std::vector<LaneChain>* geometry, double required_x) {
    const double end_x=required_x>=0.?required_x:params.path_horizon;
    const LaneChain* found = nullptr;
    for (const auto& lane : geometry ? *geometry : change_state.lanes) {
        if (lane.lane_index != lane_index || lane.centerline.empty()) continue;
        LaneSample first, last;
        const double near_x=params.contact_clearance_enabled ? lane_boundary_adapter::clearance::nearReferenceX(lane.centerline.front().x,params.planning_start_x):params.planning_start_x;
        if (!std::isfinite(near_x) || !interpolateLaneAtX(lane, near_x, first) ||
            !interpolateLaneAtX(lane, end_x, last)) continue;
        if (found != nullptr) return nullptr;  // 겹친 경계가 모호하면 변경하지 않는다.
        found = &lane;
    }
    return found;
}

bool samplePlanningLane(const LaneChain& lane, double x, LaneSample& sample) {
    if(lane.centerline.empty())return false;
    const double near_x=params.contact_clearance_enabled ? lane_boundary_adapter::clearance::nearReferenceX(lane.centerline.front().x,params.planning_start_x):params.planning_start_x;
    if(!std::isfinite(near_x))return false;
    if (x >= near_x && interpolateLaneAtX(lane, x, sample)) return true;
    // 후방 차로 점유 확인은 관측되지 않는 경계를 근거리 접선으로 연장한 보수적 근사다.
    // 전방도 관측 끝 밖에서는 마지막 1m 접선으로 검사하되 주행 경로 자체에는 외삽하지 않는다.
    double first_x = near_x;
    if (x >= near_x) {
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

bool buildPath(const LaneChain& lane, bool changing, std::vector<FamilyPoint2d>& path, double lateral_scale, double change_length, bool full_shift, double supported_end) {
    path.clear();
    LaneSample origin, next;
    if (!samplePlanningLane(lane, 0.0, origin) || !samplePlanningLane(lane, 1.0, next)) return false;
    const double origin_heading = next.center.y - origin.center.y;
    const double end_x=supported_end>=0.?std::min(supported_end,params.path_horizon):params.path_horizon;
    if(!std::isfinite(end_x)||end_x<=0.)return false;
    const int steps = static_cast<int>(std::ceil(end_x / params.path_sample_step));
    for (int index = 0; index <= steps; ++index) {
        const double x = std::min(index * params.path_sample_step, end_x);
        LaneSample sample;
        if (!samplePlanningLane(lane, x, sample)) return false;
        double y = sample.center.y;
        if (changing) {
            // 재계획할 때마다 현재 차량 위치/방향(y=0, dy/dx=0)에서 옆 중심선에 부드럽게 연결.
            const double ratio = std::clamp(x / (change_length>0?change_length:params.lane_change_length), 0.0, 1.0);
            const double blend = ratio * ratio * ratio * (10.0 + ratio * (-15.0 + 6.0 * ratio));
            // 짧은 관측 지평에서 차량 앞부분이 바깥 경계를 넘지 않도록 횡이동량을 제한.
            const double lateral_gain = (full_shift ? 1.0 : std::min(1.0, params.max_lateral_step /
                std::max(std::abs(origin.center.y), params.epsilon))) * lateral_scale;
            // 횡이동량만 줄인다. 도로 방향 정렬까지 줄이면 변경 중 차체가 비스듬한
            // 상태에서 모든 후보의 앞 모서리가 바깥 경계를 넘어 영구 정지할 수 있다.
            y -= origin.center.y * (1.0 - blend * lateral_gain);
            // Experimental complete shifts preserve endpoint heading with the
            // quintic Hermite slope basis. Legacy partial-shift output stays
            // unchanged for recovery and numerical comparison.
            y -= origin_heading * (full_shift ?
                (change_length>0?change_length:params.lane_change_length)*
                    lane_boundary_adapter::clearance::startSlopeBasis(ratio) : x*(1.0-blend));
        }
        path.push_back({x, y});
    }
    return true;
}

bool buildRoadBoundedChangePath(const LaneChain& lane, std::vector<FamilyPoint2d>& path) {
    // 방향을 바꾸지 않고, 큰 횡이동 후보부터 차량 footprint가 들어가는 후보를 선택한다.
    for (double scale : {1.0, 0.85, 0.70, 0.55, 0.40}) {
        if (buildPath(lane, true, path, scale) && pathInsideRoad(path)) return true;
    }
    return false;
}

bool pathInsideRoad(const std::vector<FamilyPoint2d>& path, const std::vector<LaneChain>* geometry,
                    RoadCheckFailure* failure) {
    if(failure)*failure=RoadCheckFailure{};
    change_state.maximum_path_curvature = 0.0;
    for (std::size_t index = 0; index < path.size(); ++index) {
        const auto& point = path[index];
        const std::size_t previous = index == 0 ? 0 : index - 1;
        const std::size_t next = std::min(index + 1, path.size() - 1);
        const double yaw = std::atan2(path[next].y - path[previous].y, path[next].x - path[previous].x);
        const double front = params.vehicle_front + params.obstacle_longitudinal_margin + params.path_sample_step;
        const double rear = params.vehicle_rear + params.obstacle_longitudinal_margin + params.path_sample_step;
        const double half_width = params.vehicle_width / 2.0 + params.obstacle_lateral_margin;
        for (double dx : {-rear, front}) {
            for (double dy : {-half_width, half_width}) {
                const double x = point.x + dx * std::cos(yaw) - dy * std::sin(yaw);
                const double y = point.y + dx * std::sin(yaw) + dy * std::cos(yaw);
                // 두 차로의 바깥 경계 밖으로 차량 모서리가 나가는 후보는 거부.
                double road_left = -std::numeric_limits<double>::infinity();
                double road_right = std::numeric_limits<double>::infinity();
                for (const auto& lane : geometry ? *geometry : change_state.lanes) {
                    LaneSample sample;
                    if (!samplePlanningLane(lane, x, sample)) continue;
                    road_left = std::max(road_left, sample.left.y);
                    road_right = std::min(road_right, sample.right.y);
                }
                if (y > road_left || y < road_right) {
                    if(failure){failure->reason="inflated_corner_outside_road";failure->x=x;failure->y=y;
                        failure->left=road_left;failure->right=road_right;}
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
    if(failure && change_state.maximum_path_curvature>steering_curvature_limit){
        failure->reason="steering_curvature_limit";failure->curvature_limit=steering_curvature_limit;
    }
    return change_state.maximum_path_curvature <= steering_curvature_limit;
}

void storePlannedTarget(const LaneChain& lane, const std::vector<FamilyPoint2d>& path) {
    follower_state.selected_lane = lane;
    follower_state.selected_lane.centerline = path;
    change_state.path = path;
    const double pursuit_x=params.contact_clearance_enabled?contact_pursuit_x:params.lookahead_distance;
    for (std::size_t index = 1; index < path.size(); ++index) {
        if (path[index].x < pursuit_x) continue;
        const double ratio = (pursuit_x - path[index - 1].x) / (path[index].x - path[index - 1].x);
        follower_state.target_point = interpolatePoint(path[index - 1], path[index], ratio);
        return;
    }
}

// Experimental clearance policy uses the SAME path generator, road/curvature
// checks, controller and selector. It never uses a semantic class to skip an
// obstacle. Lane overlap and vehicle-corridor collision are separate facts.
std::string updateContactPathPlan() {
    contact_candidates=contact_road_rejected=contact_collision_rejected=bev_rejected=0; contact_geometry_ready=false;
    auto fail=[](const std::string& reason){contact_decision=reason;return reason;};
    if(!contact_result || !contact_result->contact_projection_valid) return fail("contact_projection_unavailable");
    if(std::abs((contact_result->lane_stamp-contact_result->header.stamp).toSec())>params.contact_max_skew)
        return fail("contact_lane_timestamp_skew");
    const std::vector<LaneChain>* geometry=nullptr;
    for(const auto& frame:contact_lane_history)
        if(frame.stamp==contact_result->lane_stamp){geometry=&frame.lanes;break;}
    if(!geometry)return fail("contact_exact_lane_context_missing");
    auto bev=params.bev_mode=="off"?nullptr:selectBev(contact_result->header.stamp,contact_result->lane_stamp);
    bev_status=params.bev_mode=="off"?"off":(bev?"paired":"unavailable_or_time_skew");
    if(params.bev_mode=="guard" && !bev)return fail("bev_unavailable_or_time_skew");
    if(contact_result->ground_contacts.size()!=contact_result->boxes.size() ||
       contact_result->ground_contact_valid.size()!=contact_result->boxes.size()) return fail("contact_contract_invalid");
    std::vector<lane_boundary_adapter::clearance::Polygon> contacts;
    std::vector<lane_boundary_adapter::clearance::Polygon> local_occupied;
    auto polygonFromMessage=[](const geometry_msgs::Polygon& input){
        lane_boundary_adapter::clearance::Polygon polygon;
        for(const auto& p:input.points)polygon.push_back({p.x,p.y});
        return polygon;
    };
    for(std::size_t i=0;i<contact_result->boxes.size();++i){
        if(!contact_result->ground_contact_valid[i])return fail("contact_ground_unavailable");
        auto polygon=polygonFromMessage(contact_result->ground_contacts[i]);
        if(!lane_boundary_adapter::clearance::convexValid(polygon))return fail("contact_geometry_invalid");
        const bool measured=contact_result->ground_contact_sources.size()==contact_result->boxes.size() &&
            contact_result->ground_contact_sources[i]=="lidar_observed";
        if(measured)local_occupied.push_back(std::move(polygon));
        else {
            if(polygon.size()!=4)return fail("contact_ground_unavailable");
            contacts.push_back(std::move(polygon));
        }
    }
    // Every measured local footprint participates, independently of camera
    // matching. Preserve its ordered convex projection, including tilted OBBs;
    // do not re-expand the measured geometry into an axis-aligned rectangle.
    if(contact_result->local_obstacles_valid) {
        if(std::abs((contact_result->header.stamp-contact_result->local_obstacles_stamp).toSec())>.050000001)
            return fail("local_obstacle_timestamp_skew");
        for(const auto& poly:contact_result->local_obstacle_footprints){
            auto polygon=polygonFromMessage(poly);
            if(!lane_boundary_adapter::clearance::convexValid(polygon))return fail("local_obstacle_geometry_invalid");
            local_occupied.push_back(std::move(polygon));
        }
    }
    contact_geometry_ready=true;
    // Classifier can retain 18 m while this consumer intentionally clips at
    // 15 m. Full vehicle coverage must use their intersection, not the larger
    // upstream support (which made otherwise feasible curved paths fail).
    double horizon=contact_result->contact_support_x_max_m;
    for(const auto& lane:*geometry) {
        if(lane.left_boundary.empty() || lane.right_boundary.empty())return fail("contact_road_support_missing");
        horizon=std::min({horizon,lane.left_boundary.back().x,lane.right_boundary.back().x});
    }
    if(!std::isfinite(horizon) || horizon<params.lookahead_distance+params.vehicle_front+params.obstacle_longitudinal_margin+2*params.path_sample_step)
        return fail("contact_swept_horizon_missing");
    std::vector<lane_boundary_adapter::clearance::Point> road_reference;
    const auto* reference_lane=getPlanningLane(1,geometry,horizon);
    if(!reference_lane)return fail("contact_road_reference_missing");
    for(const auto& p:reference_lane->left_boundary)road_reference.push_back({p.x,p.y});
    std::vector<lane_boundary_adapter::clearance::Polygon> occupied;
    // Guard uses measured semantic NEW swept space instead of inventing the
    // unseen length of image contacts. Visible polygons and all local OBBs
    // remain hard obstacles. Audit leaves every existing decision unchanged.
    if(params.bev_mode=="guard")occupied=contacts;
    else if(!lane_boundary_adapter::clearance::projectedContactOccupancy(contacts,road_reference,horizon,occupied))
        return fail("contact_occlusion_geometry_invalid");
    occupied.insert(occupied.end(),local_occupied.begin(),local_occupied.end());
    const bool fresh=change_state.decision_sequence!=change_state.bbox_sequence;
    change_state.decision_sequence=change_state.bbox_sequence;
    if(change_state.entry_lane<0)change_state.entry_lane=follower_state.locked_lane;
    struct Candidate { LaneChain lane; std::vector<FamilyPoint2d> path; double offset=0,length=0; };
    auto find=[&](int index,Candidate& found){
        const auto* original=getPlanningLane(index,geometry,horizon);if(!original)return false;
        const lane_boundary_adapter::clearance::Vehicle shape{params.vehicle_front,params.vehicle_rear,
            params.vehicle_width/2,params.obstacle_lateral_margin,params.obstacle_longitudinal_margin};
        // Recorded r24 has a feasible -0.35m/3m remaining corridor missed by
        // the old0.1m/6m grid. Search147 bounded candidates per lane through
        // unchanged curvature, inflated footprint and observed-road guards.
        // Prefer a short feasible completion without weakening vehicle limits.
        // r45: a15m completion satisfies the SAME road and footprint checks
        // when all shorter completions overshoot the outer boundary mid-turn.
        // Try it only after the whole old grid fails, preserving every earlier
        // accepted choice. Actual output remains clipped to observed support;
        // completing a mathematical maneuver is never claimed beyond that.
        for(int pass=0;pass<2;++pass)
        for(double offset:{0.,-.05,.05,-.1,.1,-.15,.15,-.2,.2,-.25,.25,-.3,.3,-.35,.35,-.4,.4,-.45,.45,-.5,.5})
        for(double length:{3.,4.,5.,6.,8.,10.,12.,15.}) {
            if((length==15.)!=(pass==1))continue;
            LaneChain lane=*original;
            // Offset only the target center, never the observed road boundaries.
            for(auto& p:lane.centerline)p.y+=offset;
            std::vector<FamilyPoint2d> path;
            ++contact_candidates;
            // Contact trial evaluates the complete shift. The old 2.4 m cap
            // otherwise makes distinct adjacent-lane offsets identical; every
            // full candidate still passes the unchanged road/curvature guard.
            // r34: corrected observations ended at14.76m, so requiring a
            // fixed15m sample rejected the whole corridor. Generate only to
            // the existing grid point whose ENTIRE vehicle is supported.
            // Never invent the missing endpoint or bridge an INVALID gap.
            const double supported_axle_x=horizon-std::hypot(params.vehicle_front+params.obstacle_longitudinal_margin+params.path_sample_step,
                params.vehicle_width/2+params.obstacle_lateral_margin)-.1;
            const double path_end=std::floor(std::min(params.path_horizon,supported_axle_x)/params.path_sample_step)*params.path_sample_step;
            if(!buildPath(lane,true,path,1.0,length,true,path_end))continue;
            if(path.empty() || path.back().x<params.lookahead_distance || !pathInsideRoad(path,geometry)){++contact_road_rejected;continue;}
            std::vector<lane_boundary_adapter::clearance::Point> points;points.reserve(path.size());
            for(const auto& p:path)points.push_back({p.x,p.y});
            if(!lane_boundary_adapter::clearance::pathClearOfPolygons(points,occupied,shape,horizon,params.path_sample_step)){++contact_collision_rejected;continue;}
            if(params.bev_mode=="guard"){
                auto report=bev->check(points,shape,params.path_sample_step);bev_status=report.reason;
                if(!report.clear){++bev_rejected;continue;}
            }
            found={std::move(lane),std::move(path),offset,length};return true;
        }
        return false;
    };
    Candidate candidate;
    int target=change_state.changing?change_state.target_lane:follower_state.locked_lane;
    bool clear=find(target,candidate);
    Candidate adjacent;
    // Keep a verified collision-free corridor in the current lane. A box
    // touching the lane divider does not require a full lane change. Forcing
    // that change because a0.1m offset was needed stranded the low-speed drum
    // trial mid-turn; earlier high-speed trials had confounded this policy.
    const bool adjacent_clear=!change_state.changing && !clear &&
        find(1-follower_state.locked_lane,adjacent);
    if(!change_state.changing && (!clear || adjacent_clear)){
        if(change_state.has_completed && params.one_lane_change_only)return fail("lane_change_already_completed");
        if(change_state.has_completed && (ros::SteadyTime::now()-change_state.completed_at).toSec()<params.lane_change_cooldown)
            return fail("lane_change_cooldown");
        target=1-follower_state.locked_lane;
        clear=adjacent_clear;
        if(clear)candidate=std::move(adjacent);
        if(fresh)change_state.clear_hits=clear?change_state.clear_hits+1:0;
        if(!clear)return fail("contact_all_paths_blocked");
        if(change_state.clear_hits<params.adjacent_clear_frames)return fail("contact_confirming_clear_path");
        change_state.target_lane=target;follower_state.locked_lane=target;
        change_state.changing=true;change_state.returning_to_entry=false;
        change_state.change_started=change_state.timing_updated_at=ros::SteadyTime::now();
        change_state.drive_time_sec=change_state.pause_time_sec=0.;change_state.completion_hits=0;
        ROS_INFO("Contact-checked lane change started: target=%d",target);
    }
    if(!clear)return fail("contact_target_path_blocked");
    if(!change_state.changing){
        if(contacts.empty() && params.return_to_entry_lane && !params.one_lane_change_only && change_state.has_completed &&
           target!=change_state.entry_lane && (ros::SteadyTime::now()-change_state.completed_at).toSec()>params.lane_change_cooldown){
            if(fresh)++change_state.clear_hits;
            Candidate returning;
            if(change_state.clear_hits>=params.adjacent_clear_frames && find(change_state.entry_lane,returning)){
                candidate=std::move(returning);target=change_state.entry_lane;
                change_state.target_lane=follower_state.locked_lane=target;
                change_state.changing=change_state.returning_to_entry=true;
                change_state.change_started=change_state.timing_updated_at=ros::SteadyTime::now();
                change_state.drive_time_sec=change_state.pause_time_sec=0.;change_state.completion_hits=0;
            }
        }else change_state.clear_hits=0;
    }
    // The legacy8m target lies beyond a3m residual maneuver and cuts its
    // curvature. This only selects a target ON the checked path; it does not
    // relax the vehicle/road/obstacle guards or add another controller.
    if(params.bev_mode=="audit" && bev){
        std::vector<lane_boundary_adapter::clearance::Point> points;for(const auto& p:candidate.path)points.push_back({p.x,p.y});
        const lane_boundary_adapter::clearance::Vehicle shape{params.vehicle_front,params.vehicle_rear,params.vehicle_width/2,params.obstacle_lateral_margin,params.obstacle_longitudinal_margin};
        bev_status=bev->check(points,shape,params.path_sample_step).reason;
    }
    contact_maneuver_length=candidate.length;
    contact_pursuit_x=std::clamp(candidate.length*.5,1.5,4.0);
    storePlannedTarget(candidate.lane,candidate.path);
    change_state.target_lane=target;
    if(change_state.changing){
        const auto* original=getPlanningLane(target,geometry,horizon);LaneSample near,ahead;
        if(original && samplePlanningLane(*original,params.planning_start_x,near) &&
           samplePlanningLane(*original,params.planning_start_x+1.,ahead) &&
           contact_result->lane_stamp!=change_state.completion_stamp){
            change_state.completion_stamp=contact_result->lane_stamp;
            const double heading=std::atan2(ahead.center.y-near.center.y,1.);
            if(std::abs(near.center.y)<=params.completion_lateral_error &&
               std::abs(heading)<=params.completion_heading_deg*PI/180.)++change_state.completion_hits;
            else change_state.completion_hits=0;
            if(change_state.completion_hits>=params.completion_confirm_frames){
                change_state.changing=false;change_state.has_completed=true;change_state.returning_to_entry=false;
                change_state.completed_at=ros::SteadyTime::now();change_state.clear_hits=0;
            }
        }
    }
    contact_decision="contact_checked:lane="+std::to_string(target)+":offset="+std::to_string(candidate.offset);
    return "";
}

// ============================================================================
// 13. 상태 전이: 새 BBOX로 점유 확인, 차선 frame으로 변경 완료 확인
// ============================================================================
std::string updateLaneChangePlan() {
    if (change_state.change_failed) return change_state.change_failure_reason;
    if (change_state.entry_lane < 0) change_state.entry_lane = follower_state.locked_lane;
    const auto now = ros::SteadyTime::now();
    const bool new_scan = change_state.decision_sequence != change_state.bbox_sequence;
    change_state.decision_sequence = change_state.bbox_sequence;
    if (change_state.changing) {
        const LaneChain* target = getPlanningLane(change_state.target_lane);
        if (target == nullptr) return "target_lane_horizon_missing";
        if (!ignoreBboxDuringLaneChange()) {
            if (change_state.deferred_far) {
                change_state.clear_hits = 0;
                return "far_obstacle_lane_unresolved_during_change";
            }
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
        std::vector<FamilyPoint2d> path;
        if (!buildRoadBoundedChangePath(*target, path)) return "lane_change_path_blocked";
        storePlannedTarget(*target, path);

        LaneSample near, ahead;
        samplePlanningLane(*target, params.planning_start_x, near);
        samplePlanningLane(*target, params.planning_start_x + 1.0, ahead);
        const double heading = std::atan2(ahead.center.y - near.center.y, 1.0);
        if (follower_state.source_stamp != change_state.completion_stamp) {
            change_state.completion_stamp = follower_state.source_stamp;
            if (std::abs(near.center.y) <= params.completion_lateral_error &&
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
            return bboxStopReason(); // 완료한 주기부터 BBOX 유효성 검사를 다시 적용.
        }
        return "";
    }

    // 잘린 접점의 연속성만으로 새로운 차선변경을 시작하지 않는다.
    if (change_state.clipped_association && (!change_state.has_completed ||
        change_state.occupancy[follower_state.locked_lane] == BboxResult::DETECTED))
        return "clipped_obstacle_cannot_start_change";
    const LaneChain* current = getPlanningLane(follower_state.locked_lane);
    if (current == nullptr) return "current_lane_horizon_missing";
    const int adjacent_index = 1 - follower_state.locked_lane;
    // 장애물이 나타나기 전부터 옆 차로의 연속 미검출을 확인해 변경 시작 지연을 줄인다.
    if (new_scan) {
        change_state.clear_hits = change_state.occupancy[adjacent_index] == BboxResult::NO_DETECTION ?
            std::min(change_state.clear_hits + 1, params.adjacent_clear_frames) : 0;
    }
    const bool detected = change_state.occupancy[follower_state.locked_lane] == BboxResult::DETECTED;
    const bool both_clear = !change_state.deferred_far && !change_state.clipped_association &&
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
    if (change_state.deferred_far && detected) return "far_obstacle_lane_unresolved";
    if (change_state.deferred_far && params.far_obstacle_speed_kph <= params.epsilon) return "far_obstacle_approach_disabled";
    const auto follow_current = [&]() -> std::string {
        change_state.obstacle_hits = 0;
        std::vector<FamilyPoint2d> path;
        if (!buildPath(*current, false, path) || !pathInsideRoad(path)) return "following_path_outside_road";
        storePlannedTarget(*current, path);
        change_state.target_lane = follower_state.locked_lane;
        return "";
    };
    if (!detected && !return_ready) return follow_current();
    if (change_state.has_completed && params.one_lane_change_only) return "lane_change_already_completed";
    if (change_state.has_completed && (now-change_state.completed_at).toSec() < params.lane_change_cooldown) return "lane_change_cooldown";
    if (new_scan) change_state.obstacle_hits++;

    const LaneChain* adjacent = getPlanningLane(adjacent_index);
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
        change_state.clear_hits < params.adjacent_clear_frames) return "confirming_bbox_lane_change";

    // 모든 변경 조건이 충족되면 옆 차로를 즉시 내 차로로 지정한다.
    // 변경 중 정지/입력 유실이 생겨도 이 선택을 되돌리지 않는다.
    const int previous_lane = follower_state.locked_lane;
    follower_state.locked_lane = adjacent_index;
    change_state.target_lane = follower_state.locked_lane;
    change_state.changing = true;
    change_state.returning_to_entry = return_ready;
    change_state.return_clear_since = ros::SteadyTime();
    change_state.has_completed = false; // 이번 변경의 완료 여부를 새로 기록한다.
    change_state.change_started = now;
    change_state.timing_updated_at = now;
    change_state.drive_time_sec = change_state.pause_time_sec = 0.0;
    change_state.last_command_driving = false;
    change_state.completion_hits = 0;
    change_state.completion_stamp = ros::Time();
    storePlannedTarget(*adjacent, path);
    ROS_INFO("BBOX lane change started: %d -> %d (new own lane)", previous_lane, follower_state.locked_lane);
    return "";
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
}

}  // namespace
