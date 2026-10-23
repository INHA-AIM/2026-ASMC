#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <ros/ros.h>
#include <asmc_lidar/frame_time.hpp>
#include <Lidar/ImuRansac/GroundEvidence.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <cstdlib>
#include <Eigen/Dense>
#include <unordered_set>
#include <utility> 
#include <vector>
#include <deque>
#include <limits>
#include <time.h>
#include <float.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <pthread.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>


// ============ Message Type ============ // 
#include <yaml-cpp/yaml.h>

#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/MoraiEventCmdSrv.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/GPSMessage.h>
#include <GeographicLib/UTMUPS.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <Msg/Yolo_bbox_arr.h>


using namespace std;

// ========================= Type ========================= //
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef uint64_t uint64;
typedef short int16;
typedef int int32;
typedef float float32;
typedef double float64;
typedef char char8;

const double PI = 3.141592653589793;
const float64 c_LLA2ENU_A = 6378137.0;
const float64 c_LLA2ENU_FLAT_RATIO = 1 / 298.257223563;
const float64 c_LLA2ENU_E2 = 2 * c_LLA2ENU_FLAT_RATIO - pow(c_LLA2ENU_FLAT_RATIO, 2);
// ========================= Size ========================= //
const int32 c_PATHSIZE = 26513;
const int32 c_MAPSIZE = 2604;
const int32 c_MISSIONSIZE = 12;

// ========================= Path ========================= //
const int32 c_GLOBALPATH = 0;
const int32 c_LOCALPATH = 1;

// ========================= ControlType ========================= //
const int32 c_PUREPURSUIT = 0;
const int32 c_STANLEY = 1;
const int32 c_LKAS = 2;
const int32 c_ADVANCED_STANLEY = 3;
const int32 ControlMethod[4] = { c_PUREPURSUIT, c_STANLEY, c_LKAS, c_ADVANCED_STANLEY };
// ========================= DWA ========================= //
const int32 c_SAMPLING = 19;
// ========================= HZ ========================= //
const int32 c_LIDAR_HZ = 15;
//========================index======================//
const vector<pair<int32, int32>> traffic_idx = {
    {494, 564},
    {3147, 3175}
};

// ========================= BufferSize ========================= //
const int32 c_CAMERA_DATA_SIZE = 64979;
const int32 c_CAMERA_PACKET_SIZE = 64997;
const int32 c_CAMERA_JPEG_SIZE = 200000;

const int32 c_GPS_BUFFER_SIZE = 100;
const int32 c_IMU_BUFFER_SIZE = 100000;
const int32 c_LIDAR_BUFFER_SIZE = 2000000;
const int32 c_CAN_BUFFER_SIZE = 300;

struct Point2D
{
	float32 f32_X;
	float32 f32_Y;
};

struct BBox {
    Point2D st_Max_pt; // 좌하단
    Point2D st_Min_pt; // 우상단
    string str_class;
    // YOLO가 내놓은 원본 라벨. str_class는 융합 정책이 car/person/unknown으로
    // 접어 넣은 값이라 truck/bus 구분이 사라지므로 원본을 따로 들고 간다.
    string str_raw_class;
    int c_idx;
};

struct RAW_GPS_DATA
{
	uint64 u64_Timestamp;
	int32 s32_Num;
	char arc_Buffer[c_GPS_BUFFER_SIZE];
};

struct RAW_IMU_DATA
{
	uint64 u64_Timestamp;
	int32 s32_Num;
	char arc_Buffer[c_IMU_BUFFER_SIZE];
};

struct RawImageFrame
{
	uint64 u64_Timestamp;
	int32 s32_Num;
	char arc_Buffer[c_CAMERA_JPEG_SIZE];
};

struct RAW_YOLO_DATA 
{
	vector<BBox> vec_YOLO;
    bool b_Yolo_ready = false;
    // Timestamp of the camera image used for this inference result.
    uint64 u64_ImageTimestampNs = 0;
    // Local monotonically increasing sequence distinguishes a new inference
    // result from repeated reads in the faster Main processing loop.
    uint64 u64_FrameSeq = 0;
};

struct RAW_LIDAR_DATA
{
	uint64 u64_Timestamp;
	uint64 source_stamp_ns = 0;
	uint64 receipt_ros_ns = 0;
	uint64 receipt_steady_ns = 0;
	int32 s32_Num;
	char arc_Buffer[c_LIDAR_BUFFER_SIZE];
};

struct RAW_CAN_DATA
{
	uint64 u64_Timestamp;
	int32 s32_Num;
	char arc_Buffer[c_CAN_BUFFER_SIZE];
};

struct SENSOR_DATA
{
	int32 s32_TimeStamp;

	RawImageFrame st_RawCamera;
	RawImageFrame st_RawCamera2;
	RAW_YOLO_DATA st_RawYOLO;
	RAW_LIDAR_DATA st_RawLiDAR;
	RAW_GPS_DATA st_RawGPS;
	RAW_IMU_DATA st_RawIMU;
	RAW_CAN_DATA st_RawCAN;

	pthread_mutex_t st_MutexLiDAR;
	pthread_mutex_t st_MutexCamera;
	pthread_mutex_t st_MutexYOLO;
	pthread_mutex_t st_MutexGPS;
	pthread_mutex_t st_MutexIMU;
	pthread_mutex_t st_MutexCAN;
};

struct GPS_REF
{
	float64 f64_RefPhi;
	float64 f64_RefQ;
	float64 f64_RefX;
	float64 f64_RefY;
	float64 f64_RefZ;
	float64 f64_RefLat;
	float64 f64_RefLon;
	float64 f64_RefH;
};

struct GPS_DATA
{
    float64 f64_Latitude;
    float64 f64_Longitude;
    float64 f64_Altitude;
    float64 f64_E;
    float64 f64_N;
    float64 f64_U;
    bool b_isJamming;
};

struct IMU_DATA
{
	float32 f32_AccelX;
	float32 f32_AccelY;
	float32 f32_AccelZ;

	float32 f32_QuatX;
	float32 f32_QuatY;
	float32 f32_QuatZ;
	float32 f32_QuatW;

	float32 f32_AngularVelX;
	float32 f32_AngularVelY;
	float32 f32_AngularVelZ;

	float32 f32_Roll_rad;
	float32 f32_Pitch_rad;
	float32 f32_Yaw_rad;

	float32 f32_Roll_deg;
	float32 f32_Pitch_deg;
	float32 f32_Yaw_deg;
	bool b_ImuReady=false;
};

struct CAN_DATA
{
	float32 f32_CurrentVel_KPH;
	float32 f32_CurrentVel_MPS;
	float32 f32_Steer_deg;
	float32 f32_Steer_rad;
};

struct GPS_IMU
{
	GPS_DATA st_GPS;
	IMU_DATA st_IMU;
	CAN_DATA st_CAN;
	bool b_Jamming;
};

struct Point3D
{
	float32 f32_X;
	float32 f32_Y;
	float32 f32_Z;
};

struct Path
{
	float32 arf32_E[c_PATHSIZE];
	float32 arf32_N[c_PATHSIZE];
	float32 arf32_Tanv[c_PATHSIZE];
    float32 arf32_Norv[c_PATHSIZE];
    float32 arf32_Angle[c_PATHSIZE];
	int32 s32_NearIdx;
	int32 s32_Size;
	float32 arf32_TargetVelTraffic[c_PATHSIZE];
	float32 s_ref[c_PATHSIZE];
};

struct Map
{
	float32 arf32_E[c_MAPSIZE];
	float32 arf32_N[c_MAPSIZE];
	int32 s32_Size;
};

struct IdxSet
{
	std::pair<int32, int32> arp_Idx[c_MISSIONSIZE];
	int32 s32_SelectNumber;
	int32 s32_Num;
};

struct VehicleSpec
{
	float32 f32_Width;
	float32 f32_Length;
	float32 f32_TurningRadius;
	float32 f32_WheelBase;
	float32 f32_FrontOverhang;
	float32 f32_RearOverhang;
	Point3D st_LiDARPos;
	Point3D st_LiDARRotationDeg;
	Point3D st_GPSPos;
};

struct DWAParam
{
	// 속도, 가속, 감속 한계
	float32 f32_VMin, f32_VMax; // m/s
	float32 f32_WMin, f32_WMax; // rad/s
	float32 f32_AccMax, f32_DecMax; // m/s² 
	float32 f32_AlphaMax, f32_AlphaDecMax; // rad/s²

	// 속도, 각속도 샘플링
	float32 arf32_V[c_SAMPLING];
	float32 arf32_W[c_SAMPLING];

	// 가/감속 마진
	float32 f32_AMargin, f32_AlphaMargin;

	// 제어 주기
	float32 f32_Dt;
	float32 f32_SamplingTime;
	float32 f32_AccTime;
	float32 f32_DecTime;
	float32 f32_AlphaAccTime;
	float32 f32_AlphaDecTime;
	
	// 경로 개수, 격자 가로, 세로 샘플 개수
	int32 s32_PathCount;
	int32 s32_SampleWidth, s32_SampleLenght; 
	int32 s32_BestPathIdx;
	int32 s32_SearchLen;

	// 가중치 파라미터
	float32 f32_Alpha, f32_Beta, f32_Gamma, f32_Delta, f32_Kappa;
};

struct Planning
{
	Path st_GlobalPath;
	Path st_LocalPath;

	// DWA
	DWAParam st_DWA;
	Path st_DWAPath[c_SAMPLING];

	// HighWayPath st_HighWayPath[3];

	Map st_InBoundary1;
	Map st_InBoundary2;
	Map st_InBoundary3;
	Map st_InBoundary4;
	Map st_OutBoundary;

	// Avoid st_AvoidSmall;
	// Avoid st_AvoidCircle;
	// Avoid st_AvoidTunnel;

	IdxSet st_Mission;
	IdxSet st_Traffic;
	IdxSet st_Corner;
	IdxSet st_Straight;

	// int32 ars32_TrafficStopIdx[5] = {1220, 3495, 6480, 8215, 9990};
	// int32 s32_PathType = c_LOCALPATH;
	int32 s32_PathType = c_GLOBALPATH;
	int32 s32_ControlType;
	int32 s32_WayIdx;
	//frenet s 적용한 LookAhead Point
	Point2D st_LAP;

	float32 f32_TargetVelocity;
	float32 f32_SafetyVel;
	float32 f32_HighWayMaxVel = 30;
	
	bool b_StopSig = false;
	bool b_MissionSig = false;

	bool Local_Mode = true;

	int32 s32_SlowSig = 0;
	int32 s32_AEBMode = 0;
	float32 f32_SafeMargin;

	// float32 f32_Vertex[4][2];

	float32 Ego_MaxX, Ego_MinX;   // +X(앞), -X(뒤)  반길이
    float32 Ego_MaxY, Ego_MinY;   // +Y(좌), -Y(우)  반폭

	// float32 lateral_ratio = 1.0f;   // 자차 Y폭 스케일(보수적이면 >1)
    // bool    dynamic_inflate = true; // 동적 객체 팽창 여부

	int32 s32_endTimer = 0;
	int32 s32_StopTimer = 0;
	int32 s32_SafeTimer = 0;
	//Local 경로가 d=0 이면서 아직 Global 복귀중일때.
	bool b_Comeback2Global = false;
	bool b_avoiding = false;

	bool b_ACC;

	vector<Eigen::Vector2d> path_points;
	bool b_Warning;

};

struct Lateral_Control
{
	float32 f32_LD;
	float32 f32_Alpha;
};

struct Longitudinal_Control
{
	float32 f32_KP;
	float32 f32_KI;
	float32 f32_KD;

	float32 f32_P_Error;
	float32 f32_I_Error;
	float32 f32_D_Error;

	float32 f32_Input_P;
	float32 f32_Input_I;
	float32 f32_Input_D;
};

struct Control
{
	Lateral_Control st_Lateral_Control;
	Longitudinal_Control st_Longitudinal_Control;

	float32 f32_DefaultVel_KPH;
	float32 f32_TargetVel_KPH;
	float32 f32_MissionVel_KPH[c_MISSIONSIZE];

	float32 f32_accelCmd;
	float32 f32_brakeCmd;

	float32 f32_TargetSteer_Rad;
};

// ==================================== LiDAR ==================================== //
/*struct Object
{
    pcl::PointIndices* objPoints;
    float32 f32_core_x;
    float32 f32_core_y;
    float32 f32_core_z;
    float32 f32_Dist = std::hypot(f32_core_x, f32_core_y); 
    float32 f32_xMin;
    float32 f32_yMin;
    float32 f32_zMin;
    float32 f32_xMax;
    float32 f32_yMax;
    float32 f32_zMax;

    float32 f32_velocity;

    float32 f32_dir_x;
    float32 f32_dir_y;

    int32 s32_id = -1;  
    int32 s32_age= 0;
    int32 s32_miss_count = 0;  // 매칭 실패 횟수
    int32 s32_kf_init_count =0;

    
    Eigen::Vector4f Eigen_kf_state = Eigen::Vector4f::Zero(4);  // [x, y, vx, vy, ax, ay]
    Eigen::Matrix4f Eigen_kf_P = Eigen::Matrix4f::Identity(4, 4);
    bool b_kf_initialized = false;
};*/

//======================= 라이다 ============================

struct P3 { float x, y, z; };

struct LiDARFilterRange 
{
    float32 f32_Min;
    float32 f32_Max;
};

struct LiDARROI 
{
    float32 f32_MinX;
    float32 f32_MaxX;
    float32 f32_MinY;
    float32 f32_MaxY;
    float32 f32_MinZ;
    float32 f32_MaxZ;
};

struct LiDARClustering 
{
    bool b_Enabled;
    float32 f32_Eps;
    int32 s32_MinPoints;
    // VLP-16은 거리가 멀수록 인접 점 간격이 커진다. 고정 반경 하나로
    // 전 구간을 묶으면 근거리 과병합 또는 원거리 과분할 중 하나가 생기므로
    // 점의 수평 거리에 따라 탐색 반경을 선형 보간한다.
    bool b_Adaptive;
    float32 f32_AdaptiveNearRange;
    float32 f32_AdaptiveFarRange;
    float32 f32_AdaptiveNearEps;
    float32 f32_AdaptiveFarEps;
};

struct LiDARUDP 
{
    string str_Ip;
    int32 s32_Port;
};

struct LiDARObjectInfo
{
    bool motion_valid = false;
    // Exact preprocessing basis for the internal 2D relative CA tracker.
    // Public 3D velocity remains invalid because vertical/angular derivatives
    // have not been measured by this tracker.
    bool tracking_frame_valid = false;
    float tracking_height_input = 0;
    asmc::lidar::Rot tracking_yaw_from_input = asmc::lidar::Rot::Identity();
    asmc::lidar::Vec tracking_origin_in_yaw = asmc::lidar::Vec::Zero();
    // Experiment bookkeeping (behaviour changes only with tracking_options, default off):
    // source stamp and CAN speed of the scan described by tracking_*, and the time elapsed since
    // this track's last real measurement (missed scans accumulate their dt).
    std::int64_t tracking_stamp_ns = 0;
    float tracking_ego_speed_mps = 0;
    double kf_elapsed_since_meas_s = 0;
    // Fresh detection covariance in the SAME tracking axes as its position measurement.
    Eigen::Matrix2f observation_R = Eigen::Matrix2f::Identity() * 0.01f;
    bool observation_R_valid = false;
	string str_ObjectClass = "unknown";      // selected camera consumer group or unknown

    // str_ObjectClass를 확정시킨 YOLO 원본 라벨(16종/7종; 소비 그룹과 구분).
    // 클래스가 unknown으로 돌아가면 같이 unknown이 된다.
    std::string str_YoloClass = "unknown";

    std::string last_seen_class = "unknown";
    int continuity_counter = 0;    
    int class_miss_counter = 0;
    uint64 camera_observation_ns = 0;
    uint64 camera_observation_epoch = 0;
    bool is_class_locked = false; 

    pcl::PointIndices* objPoints;
    pcl::PointCloud<pcl::PointXYZI> pcl_ClusterPoints;
    ObjectGroundEvidence ground_evidence;
	// float64 f64_TsPrev;
    uint16 u16_Idx;
    float32 f32_Yaw;
    float32 f32_CoreX;
    float32 f32_CoreY;
    float32 f32_CoreZ;
	float32	f32_PredCoreX;
	float32	f32_PredCoreY;
    float32 f32_PrevCoreX;
    float32 f32_PrevCoreY;
	float32 f32_CoreE;//글로벌 좌표계 (x)
    float32 f32_CoreN;//글로벌 좌표계 (y)	
	float32 f32_PrevE;
    float32 f32_PrevN;
	float32 f32_PrevYaw;
    float32 f32_Dist; 
    float32 f32_MinX;
    float32 f32_MinY;
    float32 f32_MinZ;
    float32 f32_MaxX;
    float32 f32_MaxY;
    float32 f32_MaxZ;
    float32 f32_MinE_X;
    float32 f32_MinE_Y;
    float32 f32_MinE_Z;
    float32 f32_MaxE_X;
    float32 f32_MaxE_Y;
    float32 f32_MaxE_Z;
	Point2D pt_MinX;
	Point2D pt_MaxX;
	Point2D pt_MinY;
	Point2D pt_MaxY;
	//여기서부터 지워
    float f32_OBB_CenterX{0}, f32_OBB_CenterY{0}, f32_OBB_CenterZ{0};
    float f32_OBB_Width{0},  f32_OBB_Height{0};
    float f32_OBB_ZMin{0},   f32_OBB_ZMax{0};
    float f32_OBB_YawRad{0};               // XY 평면에서의 회전각 (라디안, +x 기준 CCW)

    // 꼭짓점 8개: 아래 4개(zMin) + 위 4개(zMax), 시계방향
    // struct P3 { float x, y, z; };
    std::array<P3, 8> OBB_Corners_Body;    // body(=LiDAR) 좌표계 기준
	//

    float32 f32_Velocity;

    float32 f32_AbsVx;
    float32 f32_AbsVy;

    // MotionClassifier 전용 결과. f32_AbsVx/Vy/Velocity는 항상 KF가
    // 계산한 현재 프레임 원본이며 아래 판정값으로 덮어쓰지 않는다.
    float32 f32_DecisionVxKPH = 0.0f;
    float32 f32_DecisionVyKPH = 0.0f;
    float32 f32_DecisionSpeedKPH = 0.0f;
    // Planner-facing velocity after motion-state gating. STATIC is zero;
    // PENDING blends toward consistent candidate samples instead of either
    // dropping motion completely or exposing a one-frame raw spike.
    float32 f32_PlannerVxKPH = 0.0f;
    float32 f32_PlannerVyKPH = 0.0f;
    float32 f32_MapDisplacementSpeedKPH = 0.0f;
    float32 f32_CorrectedDisplacementSpeedKPH = 0.0f;
    float32 f32_MotionVelocityJumpKPH = 0.0f;
    int32 s32_MotionState = 0;
    int32 s32_MotionValidObservations = 0;
    int32 s32_MotionAcceptedSamples = 0;
    int32 s32_MotionCandidateSamples = 0;

    float32 f32_DirX;
    float32 f32_DirY;

    int32 s32_Id;
    int32 s32_Age;
    int32 s32_MissCount;  // 매칭 실패 횟수
    int32 s32_KfInitCount;
	int32 s32_DynamicCount;
	int32 s32_StaticCount;

	Eigen::Vector2f vec_Mu;         // 모델 가중치
	Eigen::Matrix2f mat_Pi;         // 전이확률 행렬
	bool b_IMMInit = false;  // IMM 초기화 상태 변수 추가

    // Eigen::Vector4f Eigen_KfState = Eigen::Vector4f::Zero(4);  // [x, y, vx, vy]
    // Eigen::Matrix4f Eigen_KfP = Eigen::Matrix4f::Identity(4, 4);
    Eigen::Matrix<float, 6, 1> Eigen_KfState_CV = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 6, 6> Eigen_KfP_CV     = Eigen::Matrix<float, 6, 6>::Identity();
	
	Eigen::Matrix<float, 6, 1> Eigen_KfState_CA = Eigen::Matrix<float, 6, 1>::Zero();
	Eigen::Matrix<float, 6, 6> Eigen_KfP_CA     = Eigen::Matrix<float, 6, 6>::Identity();
    bool b_KfInitialized;
 
	bool b_IsDynamic = false;

	float32 f32_Vertex[4][2];
};

using Object = LiDARObjectInfo;

struct LiDARTracking
{
	int32 s32_NewTrackId ; //트래킹하면서 부여할 아이디 
	float32 f32_COST_W_DIST; // 거리 비용 가중치
	float32 f32_COST_W_IOU; // IoU 비용 가중치
	float32 f32_COST_W_DIR; // Dir 비용 가중치
	float32 f32_COST_THRESHOLD; // 게이팅 임계값: 이 값보다 cost가 크면 매칭 후보에서 제외
	float32 f32_COST_INF; 	// 매칭 불가 표시용 무한대 값
	// 연관은 자차 기준 좌표 대신 map 중심을 사용한다. 아래 값은 OBB가 같은
	// XY 점유를 중복 검출했을 때 현재 프레임에서 합치는 보수적 조건이다.
	bool b_UseMapAssociation;
	float32 f32_ContinuityBonus;
	float32 f32_DuplicateCenterDistance;
	float32 f32_DuplicateIouThreshold;
	float32 f32_MergedTrackRetireDistance;
};

struct LiDAR
{
    asmc::lidar::Context frame_context;
    // ===== 설정값 =====
    int32 s32_LiDARMode;
    LiDARUDP st_UDP;
    LiDARFilterRange st_Range;
    LiDARROI st_ROI;
    LiDARClustering st_Clustering;
	LiDARObjectInfo st_ObjectInfo;
	LiDARTracking st_Tracking;

    // UDP로 수신한 원시 패킷
    vector<uint8_t> u8_RawPacket;

	// ===== 전 프레임에 있는 객체 정보 =====
	deque<vector<LiDARObjectInfo>> Prev_Objects_Queue;
    // ===== 실시간 객체 정보 =====
    vector<LiDARObjectInfo> vec_Objects;
	vector<LiDARObjectInfo> Dynamic_Cars;
	vector<LiDARObjectInfo> Static_Cars;
	vector<LiDARObjectInfo> Dynamic_People;
	vector<LiDARObjectInfo> Static_People;
	vector<LiDARObjectInfo> Truck_Bus;
	vector<LiDARObjectInfo> Noise;
	vector<LiDARObjectInfo> Unknown;

	vector<LiDARObjectInfo> LocalObstacle;

	LiDARObjectInfo ACC_TargetCar;
    float32 f32_Threshold;

    // ===== 포인트 클라우드 =====
    pcl::PointCloud<pcl::PointXYZI> pcl_RawCloud;

    vector<vector<int32>> vec_FusionIdx;
};


extern ros::Time ts_PrevTime;
extern ros::Time ts_CurrTime;
extern LiDARTracking st_Tracking; //전역인스턴스 공유 

struct CarInfo
{
	float32 s;
	float32 d;
	float32 v; //m/s
};

struct PersonInfo
{
	float32 dist;
	float32 vel;
	float32 s;
	float32 d;
};

// ==================================== CAMERA ==================================== //


struct ImageChunk
{
    int32 s32_Sec;
    int32 s32_NSec;
    int32 s32_Index;
    int32 s32_Size;
    uint8 u8_JpegData[c_CAMERA_DATA_SIZE];
    char8 c8_Tail[2];
};

struct ImagePacket
{
    char8 c8_Header[3];
    uint8 u8_Data[c_CAMERA_PACKET_SIZE];
};

struct UdpConfig
{
	string str_Ip;
	int32 s32_Port;
};

struct HsvThresholds
{
	vector<int32> lower_red1;
	vector<int32> upper_red1;

	vector<int32> lower_red2;
	vector<int32> upper_red2;

	vector<int32> lower_green;
	vector<int32> upper_green;

	vector<int32> lower_yellow;
	vector<int32> upper_yellow;

	vector<int32> lower_bright_yellow;
	vector<int32> upper_bright_yellow;
	vector<int32> lower_dark_yellow;
	vector<int32> upper_dark_yellow;

	vector<int32> lower_white;
	vector<int32> upper_white;
};

struct TrafficLightState {
	int32 s32_Threshold;

	string str_Color;
	float64 f64_ElapsedTime;
	string str_PrevColor;

	ros::Time time_Stamp;
	bool b_stop_flag, b_slow_flag, b_unknown_flag;

};

struct FUSION_Flag
{
    bool b_Img_ready = false;
    bool b_Yolo_ready = false;
    bool b_Lidar_ready = false;
};


struct FusionData
{
    vector<BBox> vec_Lidar_BBoxes;
    vector<BBox> vec_Camera_BBoxes;
};

struct CameraParameters
{
	Eigen::Matrix<double, 3, 4> intrinsic_matrix_70_fov;
	Eigen::Matrix<double, 3, 4> intrinsic_matrix_90_fov;
    Eigen::Matrix4d sensor_fusion_transform;
    Eigen::Matrix4d bird_eye_view_transform;
    Eigen::Matrix<double, 3, 4> sensor_fusion_projection_matrix;
    Eigen::Matrix<double, 3, 4> bird_eye_view_projection_matrix;

	float32 initial_roll_rad;
	float32 initial_pitch_rad;

    vector<float32> world_points_vector;
	vector<Point3D> world_points_3d;
	deque<vector<float32>> z_offset_queue;
	int32 moving_average_filter_size;
    vector<Point3D> projected_points;
    cv::Mat perspective_transform_matrix;
    cv::Mat inverse_perspective_transform_matrix;

    vector<cv::Point2f> destination_points;
    bool is_initial_transform_calculated = false;
};

struct YoloDetectionResult
{
    vector<BBox> detected_objects;
};

struct CAMERA
{
	int32 s32_Display;
	//Mode
	int32 s32_CameraMode;

	//UDP
	UdpConfig front_camera_udp;
	UdpConfig traffic_light_camera_udp;
	vector<uint8_t> raw_image_buffer;

	//Data
	cv::Mat bgr_image;
	ros::Time front_image_stamp;
	vector<uint8_t> front_compressed_image;

	cv::Mat traffic_light_image;
	bool is_traffic_light_camera_available = false;
	HsvThresholds hsv_thresholds;
	TrafficLightState traffic_light_state;

	//Fusion
	FUSION_Flag fusion_flags;
	FusionData fusion_data;
	YoloDetectionResult yolo_results;

	//Matrix
	CameraParameters params;

	// Stop Line
	bool is_stop_line_detected = false;
	float32 stop_line_distance;

	//LaneDetection
	float32 lane_angle;
	bool endSig = 0;

};

// ==================================== RAD<->DEG ==================================== //
float64 Deg2Rad(const float64 &f64_Deg);
float32 Deg2Rad(const float32 &f32_Deg);

float64 Rad2Deg(const float64 &f64_Rad);
float32 Rad2Deg(const float32 &f32_Rad);
// ==================================== RAD<->DEG ==================================== //

// ==================================== Scale ==================================== //
float32 Scale2D(float32 &f32_X, float32 &f32_Y);
float32 Scale2D(float32 *arf_X, float32 *arf_Y, int32 s32_Idx);
float32 Scale2D(Point2D &st_Point2D);

float32 Scale3D(float32 &f32_X, float32 &f32_Y, float32 &f32_Z);
float32 Scale3D(float32 *arf_X, float32 *arf_Y, float32 *arf_Z, int32 s32_Idx);
float32 Scale3D(Point3D &st_Point3D);
// ==================================== Scale ==================================== //

// ==================================== Normalize ==================================== //
float32 Normalize2D(float32 f32_X, float32 f32_Y);
float32 Normalize2D(float32 f32_X1, float32 f32_Y1, float32 f32_X2, float32 f32_Y2);
float32 Normalize2D(float32 *arf_X, float32 *arf_Y, int32 s32_Idx1, int32 s32_Idx2);
float32 Normalize2D(float32 *arf_X, float32 *arf_Y, int32 s32_Idx1, float32 &f32_X, float32 &f32_Y);
float32 Normalize2D(float32 *arf_X, float32 *arf_Y, int32 s32_Idx1, Point2D &st_P2);

float32 Normalize2D(Point2D &st_P1, Point2D &st_P2);
float32 Normalize2D(std::vector<float32> &vf_P1, Point2D &st_P2);
float32 Normalize2D(std::vector<float32> &vf_P1, float32 &f32_X, float32 &f32_Y);

float32 Normalize3D(float32 &f32_X1, float32 &f32_Y1, float32 &f32_Z1, float32 &f32_X2, float32 &f32_Y2, float32 &f32_Z2);
float32 Normalize3D(float32 *arf_X, float32 *arf_Y, float32 *arf_Z, int32 &s32_Idx1, int32 &s32_Idx2);
float32 Normalize3D(Point3D &st_P1, Point3D &st_P2);

float32 NormalizeAngle(float32 angle);
float32 NormalizeDWAFactor(float32* arr, int32 size, int32 idx);
// ==================================== Normalize ==================================== //

// ==================================== Velocity ==================================== //
float32 KPH2MPS(float32 f32_KPH);
float32 MPS2KPH(float32 f32_MPS);
// ==================================== Velocity ==================================== //

// ==================================== Vector ==================================== //
Point2D UnitVector(Point2D &st_Point2DStart, Point2D &st_Point2DEnd);
Point2D UnitVector(float32 f32_XStart, float32 f32_YStart, float32 f32_XEnd, float32 f32_YEnd);
float32 CrossProduct(Point2D &st_RefPoint, Point2D &st_Point);
// ==================================== Vector ==================================== //

// ==================================== LoadData ==================================== //
void LoadData(std::ifstream &st_LoadFile, Path &st_Path);
// void LoadData(std::ifstream &st_LoadFile, HighWayPath &st_Path);
void LoadData(std::ifstream &st_LoadFile, Map &st_Map);
void LoadIdx(std::ifstream &st_LoadFile, IdxSet &st_IdxSet);
// ==================================== LoadData ==================================== //

// ==================================== Sigmoid ==================================== //
float32 Sigmoid(float32 f32_Value);
// ==================================== Sigmoid ==================================== //

// ==================================== RayCasting ==================================== //
bool InOrOut(Planning &st_Planning, float32 f32_PointX, float32 f32_PointY);
bool InOrOut_lane(Planning &st_Planning, float32 f32_PointX, float32 f32_PointY);
// ==================================== RayCasting ==================================== //

// ==================================== FindNear ==================================== //
int32 FindNear(Planning &st_Planning, float32 &f32_X, float32 &f32_Y, int32 s32_FindNearLen, int32 s32_PathType);
int32 FindNear(Planning &st_Planning, Point2D &st_Point2D, int32 s32_StartIdx, int32 s32_FindNearLen, int32 s32_PathType);
int32 FindClostIdx(Planning &st_Planning, float32 f32_E, float32 f32_N);
int32 FindClostIdx(Planning &st_Planning, float32 f32_E, float32 f32_N, int32 PathType);
int32 FindClostIdx(Path &st_GlobalPath, Point2D& st_Point, int32 s32_SearchLen);

void FindClostIdx(GPS_IMU& st_GPS, Planning &st_Planning);
// ==================================== FindNear ==================================== //

void GetTan(Path &path);
void Get_S_Ref(Path &path);
Point2D ENU2Frenet(Planning &st_Planning, float32 f32_E, float32 f32_N, string PathType = "");
Point2D Frenet2ENU(Planning &st_Planning, float32 f32_s, float32 f32_d);
void Body2ENU(VehicleSpec& st_VehicleSpec, const GPS_IMU& st_GPSIMU, Object& st_BodyObst);
Point2D Body2ENU_Point(const VehicleSpec& st_VehicleSpec, const GPS_IMU& st_GPSIMU, float32 x, float32 y);
void MinMax2ENU(VehicleSpec& st_VehicleSpec, const GPS_IMU& st_GPSIMU, Object& st_BodyObst);

uint64_t GetMillisecond();

Point2D CalcTangentVector(Point2D p1, Point2D p2);
Point2D CalcNormalVector(Point2D edge);
float32 CalcDotProduct(Point2D v1, Point2D v2);
void PolygonProjection(Point2D axis, Point2D vertices[], int numVertices, float32 *min, float32 *max);
bool OverlappingTest(float32 minA, float32 maxA, float32 minB, float32 maxB);
void CalcVertex(float32 f32_X_global, float32 f32_Y_global, float32 f32_MaxX, float32 f32_MaxY, float32 f32_MinX, float32 f32_MinY, float32 f32_heading, float32 (&f32_Vertex)[4][2], int32 s32_Mode);
bool CalcSAT(float32 vertices1[4][2], float32 vertices2[4][2]);
static float32 accumulateS(const Path& path, int i);

#endif
