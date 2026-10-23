#include "SensorFusion/FusionClassPolicy.hpp"
#include <Global/Global.hpp>
#include <Parser/Parser.hpp>
#include <Camera/Camera.hpp>
#include <Lidar/LidarProcess/LidarProcess.hpp>
#include <Lidar/LidarProcess/PipelineProfile.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <SensorFusion/SensorFusion.hpp>
#include <SensorFusion/FusionTrace.hpp>
#include <SensorFusion/FusionStateTrace.hpp>
#include <PlannerInterface/PlannerObjectSender.hpp>
#include <PlannerInterface/PlannerObjectRosPublisher.hpp>
#include <ros/package.h>
#include <Geometry/FrameBoundaryPublisher.hpp>
#include <Geometry/OutputGate.hpp>
#include <Lidar/Tracking/TrackTrace.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>


// 실행 디렉터리와 무관하게 패키지 기준으로 설정 파일을 찾는다.
const string YamlURL = ros::package::getPath("Main") + "/setting.yaml";

// Publisher & Subscriber
ros::Publisher VizPub, CtrlPub;
ros::ServiceClient GearPub; 
ros::Subscriber LiDAR_Sub, Camera_Sub;
ros::Publisher  VelPlotPub;
ros::Timer      VelPlotTimer;

// Main Struct
VehicleSpec st_VehicleSpec;
GPS_IMU st_GPSIMU, st_GPSIMUTemporal;
Planning st_Planning, st_PlanningTemporal;
Control st_Control, st_ControlTemporal;
LiDAR st_LiDAR, st_LiDARTemporal;
CAMERA st_Camera, st_CameraTemporal;
GPS_REF st_GPSRef;
std::unique_ptr<PlannerObjectSender> st_PlannerObjectSender;
std::unique_ptr<PlannerObjectRosPublisher> st_PlannerObjectRosPublisher;
std::unique_ptr<frame_boundary::Publisher> st_FrameBoundaryPublisher;
std::unique_ptr<MotionClassifier> st_MotionClassifier;

int32 GPSHZ, VisionHZ, LiDARHZ, IMUHZ, CANHZ;
int32 VIEWERHZ; // InitViewer에서, 기준이 되는 센서의 HZ 할당해주기.

SENSOR_DATA st_SensorData;
SENSOR_DATA st_CurrentData;

// ==================== 전역 Mutex ==================== //
pthread_mutex_t st_CopyMutex;
pthread_mutex_t st_DataTemporal;
pthread_cond_t st_DataTemporalCond;
std::uint64_t u64_LidarFrameGeneration = 0;

bool b_isRunning = true;
bool b_EnableSensorFusion = false;
bool b_BenchmarkLidarPipeline = false;
int32 s32_BenchmarkWarmupFrames = 30;
int32 s32_BenchmarkSampleFrames = 200;

namespace {

struct LidarPipelineBenchmark {
    int32 seen_frames = 0;
    std::vector<double> elapsed_ms;
    double raw_points_sum = 0.0;
    double tracks_sum = 0.0;

    // 단계별 누적치. ROI 폭이 늘어날 때 어느 구간이 비싸지는지 분리한다.
    LidarStageProfile stage_sum;

    void Accumulate(const LidarStageProfile& p) {
        stage_sum.make_cloud_ms += p.make_cloud_ms;
        stage_sum.roi_ms += p.roi_ms;
        stage_sum.roi_pub_ms += p.roi_pub_ms;
        stage_sum.voxel_ms += p.voxel_ms;
        stage_sum.imu_correct_ms += p.imu_correct_ms;
        stage_sum.ransac_ms += p.ransac_ms;
        stage_sum.ransac_pub_ms += p.ransac_pub_ms;
        stage_sum.cluster_ms += p.cluster_ms;
        stage_sum.cluster_pub_ms += p.cluster_pub_ms;
        stage_sum.map_cluster_pub_ms += p.map_cluster_pub_ms;
        stage_sum.objectinfo_ms += p.objectinfo_ms;
        stage_sum.framefilter_ms += p.framefilter_ms;
        stage_sum.motion_ms += p.motion_ms;
        stage_sum.marker_ms += p.marker_ms;
        stage_sum.roi_points += p.roi_points;
        stage_sum.voxel_points += p.voxel_points;
        stage_sum.ransac_points += p.ransac_points;
        stage_sum.cluster_points += p.cluster_points;
        stage_sum.clusters += p.clusters;
    }

    void Add(double milliseconds, int32 raw_points, std::size_t tracks) {
        ++seen_frames;
        if (seen_frames <= s32_BenchmarkWarmupFrames) return;
        elapsed_ms.push_back(milliseconds);
        raw_points_sum += raw_points;
        tracks_sum += tracks;
        Accumulate(g_LidarStageProfile);
        if (static_cast<int32>(elapsed_ms.size()) <
            s32_BenchmarkSampleFrames) {
            return;
        }

        std::vector<double> sorted = elapsed_ms;
        std::sort(sorted.begin(), sorted.end());
        const double sum = std::accumulate(
            elapsed_ms.begin(), elapsed_ms.end(), 0.0);
        const double mean = sum / elapsed_ms.size();
        double squared_error_sum = 0.0;
        for (const double value : elapsed_ms) {
            const double error = value - mean;
            squared_error_sum += error * error;
        }
        const double stddev = std::sqrt(
            squared_error_sum / elapsed_ms.size());
        const auto percentile = [&sorted](double fraction) {
            const std::size_t index = static_cast<std::size_t>(std::ceil(
                fraction * static_cast<double>(sorted.size()))) - 1;
            return sorted[std::min(index, sorted.size() - 1)];
        };
        const double n = static_cast<double>(elapsed_ms.size());
        ROS_INFO(
            "[LIDAR_BENCH] roi_y=[%.1f,%.1f] frames=%zu mean=%.3fms median=%.3fms p95=%.3fms p99=%.3fms min=%.3fms max=%.3fms stddev=%.3fms raw_points_mean=%.1f tracks_mean=%.2f",
            st_LiDAR.st_ROI.f32_MinY, st_LiDAR.st_ROI.f32_MaxY,
            elapsed_ms.size(), mean, percentile(0.50), percentile(0.95),
            percentile(0.99), sorted.front(), sorted.back(), stddev,
            raw_points_sum / n,
            tracks_sum / n);
        ROS_INFO(
            "[LIDAR_STAGE] roi_y=[%.1f,%.1f] make=%.3f roi=%.3f voxel=%.3f imu=%.3f ransac=%.3f cluster=%.3f objinfo=%.3f framefilter=%.3f motion=%.3f | pub_roi=%.3f pub_ransac=%.3f pub_cluster=%.3f pub_mapcluster=%.3f pub_marker=%.3f",
            st_LiDAR.st_ROI.f32_MinY, st_LiDAR.st_ROI.f32_MaxY,
            stage_sum.make_cloud_ms / n, stage_sum.roi_ms / n,
            stage_sum.voxel_ms / n, stage_sum.imu_correct_ms / n,
            stage_sum.ransac_ms / n, stage_sum.cluster_ms / n,
            stage_sum.objectinfo_ms / n, stage_sum.framefilter_ms / n,
            stage_sum.motion_ms / n, stage_sum.roi_pub_ms / n,
            stage_sum.ransac_pub_ms / n, stage_sum.cluster_pub_ms / n,
            stage_sum.map_cluster_pub_ms / n, stage_sum.marker_ms / n);
        ROS_INFO(
            "[LIDAR_POINTS] roi_y=[%.1f,%.1f] raw=%.1f roi=%.1f voxel=%.1f ransac=%.1f cluster=%.1f clusters=%.2f",
            st_LiDAR.st_ROI.f32_MinY, st_LiDAR.st_ROI.f32_MaxY,
            raw_points_sum / n, stage_sum.roi_points / n,
            stage_sum.voxel_points / n, stage_sum.ransac_points / n,
            stage_sum.cluster_points / n, stage_sum.clusters / n);
        b_BenchmarkLidarPipeline = false;
        // 측정이 끝나면 스스로 내려가야 로그가 flush 되고 다음 조건이
        // 이전 프로세스와 CPU를 나눠 쓰지 않는다.
        b_isRunning = false;
        ros::shutdown();
    }
};

LidarPipelineBenchmark st_LidarPipelineBenchmark;

}  // namespace

void InitMutex()
{
    pthread_mutex_init(&st_CopyMutex, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexLiDAR, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexCamera, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexGPS, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexIMU, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexCAN, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexYOLO, NULL);
    pthread_mutex_init(&st_DataTemporal, NULL);
    pthread_cond_init(&st_DataTemporalCond, NULL);
}

void InitVehicleSpec()
{
    YAML::Node config = YAML::LoadFile(YamlURL);
    st_VehicleSpec.f32_Length = config["Vehicle"]["Length"].as<float32>();
    st_VehicleSpec.f32_Width = config["Vehicle"]["Width"].as<float32>();
    st_VehicleSpec.f32_WheelBase = config["Vehicle"]["WheelBase"].as<float32>();
    st_VehicleSpec.f32_FrontOverhang = config["Vehicle"]["FrontOverhang"].as<float32>();
    st_VehicleSpec.f32_RearOverhang = config["Vehicle"]["RearOverhang"].as<float32>();
    st_VehicleSpec.f32_TurningRadius = config["Vehicle"]["TurningRadius"].as<float32>();
    st_VehicleSpec.st_LiDARPos.f32_X = config["Vehicle"]["LiDAR"]["x"].as<float32>();
    st_VehicleSpec.st_LiDARPos.f32_Y = config["Vehicle"]["LiDAR"]["y"].as<float32>();
    st_VehicleSpec.st_LiDARPos.f32_Z = config["Vehicle"]["LiDAR"]["z"].as<float32>();
    st_VehicleSpec.st_LiDARRotationDeg.f32_X = config["Vehicle"]["LiDAR"]["roll"].as<float32>();
    st_VehicleSpec.st_LiDARRotationDeg.f32_Y = config["Vehicle"]["LiDAR"]["pitch"].as<float32>();
    st_VehicleSpec.st_LiDARRotationDeg.f32_Z = config["Vehicle"]["LiDAR"]["yaw"].as<float32>();
    st_VehicleSpec.st_GPSPos.f32_X = config["Vehicle"]["GPS"]["x"].as<float32>();
    st_VehicleSpec.st_GPSPos.f32_Y = config["Vehicle"]["GPS"]["y"].as<float32>();
    st_VehicleSpec.st_GPSPos.f32_Z = config["Vehicle"]["GPS"]["z"].as<float32>();
    GPSHZ = config["Vehicle"]["GPS"]["Hz"].as<int32>();
    IMUHZ = config["Vehicle"]["IMU"]["Hz"].as<int32>();
    VisionHZ = config["Vehicle"]["Camera"]["Hz"].as<int32>();
    LiDARHZ = config["Vehicle"]["LiDAR"]["Hz"].as<int32>();
    CANHZ = config["Vehicle"]["CAN"]["Hz"].as<int32>();
}

void InitPlannerInterface(bool planner_udp_enabled)
{
    // ~planner_udp_enabled=false: the planner UDP sender (and its socket) is never created, so the UDP Send below never runs.
    if (!planner_udp_enabled) {
        ROS_INFO("Planner object interface disabled (planner_udp_enabled=false)");
        return;
    }
    YAML::Node config = YAML::LoadFile(YamlURL);
    st_PlannerObjectSender =
        MakePlannerObjectSender(planner_udp_enabled, config["PlannerInterface"]);
    if (!st_PlannerObjectSender) ROS_INFO("Planner object interface disabled");
}

void InitGlobal(ros::NodeHandle &st_NodeHandle, pthread_t *pth_GPS)
{
    // 인지 전용: 전역 경로/맵 로드는 Planning 전용이라 제거.
    // GPS 변환은 GPSMessage의 east/northOffset 기반(UTM)이라 Ref 파일이 필요 없다.
    pthread_create(pth_GPS, NULL, ParseGPS, (void*)&st_NodeHandle);
}

void InitIMU(ros::NodeHandle &st_NodeHandle, pthread_t *pth_IMU)
{
    pthread_create(pth_IMU, NULL, ParseIMU, (void*)&st_NodeHandle);
}

/* ==================== 인지 전용 빌드에서 제외 (Planning/Control 소스 부재) ====================
void InitPlanning()
{
    YAML::Node config = YAML::LoadFile(YamlURL);

    // Control Type -> 0 : Pure Pursuit, 1 : Stanley, 2 : LKAS, 3 : Advanced Stanley
    st_Planning.s32_ControlType = ControlMethod[config["Planning"]["Control_Type"].as<int32>()];
    st_Planning.f32_SafeMargin = config["Planning"]["SafeMargin"].as<float32>();
    st_Planning.f32_SafetyVel = config["Planning"]["SafeVel"].as<float32>();

    // ======================================= DWA Param ======================================= //  
    // 최대/최소 속도, 각속도 지정 -> 1. Speed Boundary Limits
    st_Planning.st_DWA.f32_VMax = KPH2MPS(config["Planning"]["DWAParam"]["vMax"].as<float32>());
    st_Planning.st_DWA.f32_VMin = KPH2MPS(config["Planning"]["DWAParam"]["vMin"].as<float32>());

    // 가속도, 각가속도 Margin 지정
    st_Planning.st_DWA.f32_AMargin = config["Planning"]["DWAParam"]["aMargin"].as<float32>();
    st_Planning.st_DWA.f32_AlphaMargin = config["Planning"]["DWAParam"]["alphaMargin"].as<float32>();

    // 최대/최소 가속도, 각가속도InitCamera
    st_Planning.st_DWA.f32_AccTime = config["Planning"]["DWAParam"]["accTime"].as<float32>();
    st_Planning.st_DWA.f32_DecTime = config["Planning"]["DWAParam"]["decTime"].as<float32>();
    st_Planning.st_DWA.f32_AlphaAccTime = config["Planning"]["DWAParam"]["alphaAccTime"].as<float32>();
    st_Planning.st_DWA.f32_AlphaDecTime = config["Planning"]["DWAParam"]["alphaDecTime"].as<float32>();
    
    // 제어 주기
    st_Planning.st_DWA.f32_Dt = 1 / config["Planning"]["DWAParam"]["ControlHz"].as<float32>();
    st_Planning.st_DWA.f32_SamplingTime = config["Planning"]["DWAParam"]["SamplingTime"].as<float32>();

    // 경로 샘플 수
    st_Planning.st_DWA.s32_PathCount = config["Planning"]["DWAParam"]["PathCount"].as<int32>();
    st_Planning.st_DWA.s32_SampleWidth = config["Planning"]["DWAParam"]["Width"].as<int32>();
    st_Planning.st_DWA.s32_SampleLenght = config["Planning"]["DWAParam"]["Length"].as<int32>();
    st_Planning.st_DWA.s32_SearchLen = config["Planning"]["DWAParam"]["SearchLen"].as<int32>();
    st_Planning.st_DWA.f32_Alpha = config["Planning"]["DWAParam"]["alpha"].as<float32>();
    st_Planning.st_DWA.f32_Beta = config["Planning"]["DWAParam"]["beta"].as<float32>();
    st_Planning.st_DWA.f32_Gamma = config["Planning"]["DWAParam"]["gamma"].as<float32>();
    st_Planning.st_DWA.f32_Delta = config["Planning"]["DWAParam"]["delta"].as<float32>();
    // ======================================= DWA Param ======================================= //  
}

void InitControl()
{
    YAML::Node config = YAML::LoadFile(YamlURL);

    st_Control.f32_DefaultVel_KPH = config["Control"]["Default_KPH"].as<float32>();
    st_Control.st_Lateral_Control.f32_LD = config["Control"]["LD"].as<float32>();
    st_Control.st_Longitudinal_Control.f32_KP = config["Control"]["Pid"]["Kp"].as<float64>();
    st_Control.st_Longitudinal_Control.f32_KI = config["Control"]["Pid"]["Ki"].as<float64>();
    st_Control.st_Longitudinal_Control.f32_KD = config["Control"]["Pid"]["Kd"].as<float64>();
};
==================== 인지 전용 빌드에서 제외 ==================== */

void InitializeCamera(ros::NodeHandle &st_NodeHandle, pthread_t *pth_Camera)
{
    YAML::Node config = YAML::LoadFile(YamlURL);


    st_Camera.s32_CameraMode =  config["Camera"]["CameraMode"].as<int32>();
    st_Camera.front_camera_udp.str_Ip = config["Camera"]["UDP1"]["ip"].as<string>();
    st_Camera.front_camera_udp.s32_Port = config["Camera"]["UDP1"]["port"].as<int32>();
    st_Camera.traffic_light_camera_udp.str_Ip = config["Camera"]["UDP2"]["ip"].as<string>();
    st_Camera.traffic_light_camera_udp.s32_Port = config["Camera"]["UDP2"]["port"].as<int32>();
    st_Camera.s32_Display = config["Camera"]["Display"].as<int32>();
    st_Camera.hsv_thresholds.lower_red1 = config["Camera"]["HSV"]["lower_red1"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_red1 = config["Camera"]["HSV"]["upper_red1"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_red2 = config["Camera"]["HSV"]["lower_red2"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_red2 = config["Camera"]["HSV"]["upper_red2"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_green = config["Camera"]["HSV"]["lower_green"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_green = config["Camera"]["HSV"]["upper_green"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_yellow = config["Camera"]["HSV"]["lower_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_yellow = config["Camera"]["HSV"]["upper_yellow"].as<vector<int32>>();
    st_Camera.params.world_points_vector = config["Camera"]["BEV"]["WorldPoints"].as<vector<float32>>();

    st_Camera.hsv_thresholds.lower_bright_yellow = config["Camera"]["HSV"]["lower_bright_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_bright_yellow = config["Camera"]["HSV"]["upper_bright_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_dark_yellow = config["Camera"]["HSV"]["lower_dark_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_dark_yellow = config["Camera"]["HSV"]["upper_dark_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_white = config["Camera"]["HSV"]["lower_white"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_white = config["Camera"]["HSV"]["upper_white"].as<vector<int32>>();


    st_Camera.traffic_light_state.s32_Threshold = config["Camera"]["TrafficLight"]["Threshold"].as<int32>();
    st_Camera.params.moving_average_filter_size =  config["Camera"]["BEV"]["MAF"].as<int32>();

    InitProjectionMatrix(st_Camera, config);
    pthread_create(pth_Camera, NULL, ParseCamera, (void*)&st_NodeHandle);

};

void InitLidarSettings()
{   
    YAML::Node config = YAML::LoadFile(YamlURL);

    st_LiDAR.s32_LiDARMode = config["LiDAR"]["LiDARMode"].as<int32>();
    st_LiDAR.st_UDP.str_Ip = config["LiDAR"]["UDP"]["ip"].as<string>();
    st_LiDAR.st_UDP.s32_Port = config["LiDAR"]["UDP"]["port"].as<int32>();

    st_LiDAR.st_Range.f32_Min = config["LiDAR"]["Filter"]["Range"]["Min"].as<float32>();
    st_LiDAR.st_Range.f32_Max = config["LiDAR"]["Filter"]["Range"]["Max"].as<float32>();

    st_LiDAR.st_ROI.f32_MinX = config["LiDAR"]["Filter"]["ROI"]["MinX"].as<float32>();
    st_LiDAR.st_ROI.f32_MaxX = config["LiDAR"]["Filter"]["ROI"]["MaxX"].as<float32>();
    st_LiDAR.st_ROI.f32_MinY = config["LiDAR"]["Filter"]["ROI"]["MinY"].as<float32>();
    st_LiDAR.st_ROI.f32_MaxY = config["LiDAR"]["Filter"]["ROI"]["MaxY"].as<float32>();
    st_LiDAR.st_ROI.f32_MinZ = config["LiDAR"]["Filter"]["ROI"]["MinZ"].as<float32>();
    st_LiDAR.st_ROI.f32_MaxZ = config["LiDAR"]["Filter"]["ROI"]["MaxZ"].as<float32>();

    const YAML::Node clustering = config["LiDAR"]["Clustering"];
    st_LiDAR.st_Clustering.b_Enabled = clustering["Enabled"].as<bool>();
    st_LiDAR.st_Clustering.f32_Eps = clustering["Eps"].as<float32>();
    st_LiDAR.st_Clustering.s32_MinPoints = clustering["MinPoints"].as<int32>();
    const YAML::Node adaptive = clustering["Adaptive"];
    st_LiDAR.st_Clustering.b_Adaptive =
        adaptive["Enabled"].as<bool>(true);
    st_LiDAR.st_Clustering.f32_AdaptiveNearRange =
        adaptive["NearRange"].as<float32>(8.0F);
    st_LiDAR.st_Clustering.f32_AdaptiveFarRange =
        adaptive["FarRange"].as<float32>(50.0F);
    st_LiDAR.st_Clustering.f32_AdaptiveNearEps =
        adaptive["NearEps"].as<float32>(0.55F);
    st_LiDAR.st_Clustering.f32_AdaptiveFarEps =
        adaptive["FarEps"].as<float32>(0.85F);

    const YAML::Node tracking = config["LiDAR"]["Tracking"];
    st_Tracking.b_UseMapAssociation =
        tracking["UseMapAssociation"].as<bool>(true);
    st_Tracking.f32_COST_W_DIST =
        tracking["DistanceWeight"].as<float32>(1.0F);
    st_Tracking.f32_COST_W_IOU =
        tracking["ShapeWeight"].as<float32>(1.0F);
    st_Tracking.f32_COST_THRESHOLD =
        tracking["CostThreshold"].as<float32>(3.0F);
    st_Tracking.f32_ContinuityBonus =
        tracking["ContinuityBonus"].as<float32>(0.20F);
    st_Tracking.f32_DuplicateCenterDistance =
        tracking["DuplicateCenterDistance"].as<float32>(0.60F);
    st_Tracking.f32_DuplicateIouThreshold =
        tracking["DuplicateIouThreshold"].as<float32>(0.35F);
    st_Tracking.f32_MergedTrackRetireDistance =
        tracking["MergedTrackRetireDistance"].as<float32>(0.45F);
    st_LiDAR.st_Tracking = st_Tracking;
    ros::NodeHandle private_node("~");
    private_node.param(
        "roi_min_x", st_LiDAR.st_ROI.f32_MinX,
        st_LiDAR.st_ROI.f32_MinX);
    private_node.param(
        "roi_max_x", st_LiDAR.st_ROI.f32_MaxX,
        st_LiDAR.st_ROI.f32_MaxX);
    private_node.param(
        "roi_min_y", st_LiDAR.st_ROI.f32_MinY,
        st_LiDAR.st_ROI.f32_MinY);
    private_node.param(
        "roi_max_y", st_LiDAR.st_ROI.f32_MaxY,
        st_LiDAR.st_ROI.f32_MaxY);
    ROS_INFO("LiDAR ROI: x=[%.1f,%.1f] y=[%.1f,%.1f] z=[%.1f,%.1f]",
             st_LiDAR.st_ROI.f32_MinX, st_LiDAR.st_ROI.f32_MaxX,
             st_LiDAR.st_ROI.f32_MinY, st_LiDAR.st_ROI.f32_MaxY,
             st_LiDAR.st_ROI.f32_MinZ, st_LiDAR.st_ROI.f32_MaxZ);
    ROS_INFO(
        "LiDAR clustering: adaptive=%s range=[%.1f,%.1f]m eps=[%.2f,%.2f]m min_points=%d",
        st_LiDAR.st_Clustering.b_Adaptive ? "true" : "false",
        st_LiDAR.st_Clustering.f32_AdaptiveNearRange,
        st_LiDAR.st_Clustering.f32_AdaptiveFarRange,
        st_LiDAR.st_Clustering.f32_AdaptiveNearEps,
        st_LiDAR.st_Clustering.f32_AdaptiveFarEps,
        st_LiDAR.st_Clustering.s32_MinPoints);
    ROS_INFO(
        "LiDAR tracking: map_association=%s gate=%.2f continuity=%.2f duplicate=(%.2fm, iou %.2f)",
        st_Tracking.b_UseMapAssociation ? "true" : "false",
        st_Tracking.f32_COST_THRESHOLD,
        st_Tracking.f32_ContinuityBonus,
        st_Tracking.f32_DuplicateCenterDistance,
        st_Tracking.f32_DuplicateIouThreshold);
}

void InitLidar(ros::NodeHandle &st_NodeHandle, pthread_t *pth_Lidar)
{
    InitLidarSettings();
    pthread_create(pth_Lidar, NULL, ParseLiDAR, (void*)&st_NodeHandle);
}


void CopyData(SENSOR_DATA *pst_Data)
{
    pthread_mutex_lock(&st_SensorData.st_MutexLiDAR);
    memcpy(&(pst_Data->st_RawLiDAR), &st_SensorData.st_RawLiDAR, sizeof(RAW_LIDAR_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR);

    pthread_mutex_lock(&st_SensorData.st_MutexGPS);
    memcpy(&(pst_Data->st_RawGPS), &st_SensorData.st_RawGPS, sizeof(RAW_GPS_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexGPS);

    pthread_mutex_lock(&st_SensorData.st_MutexIMU);
    memcpy(&(pst_Data->st_RawIMU), &st_SensorData.st_RawIMU, sizeof(RAW_IMU_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexIMU);

    pthread_mutex_lock(&st_SensorData.st_MutexCAN);
    memcpy(&(pst_Data->st_RawCAN), &st_SensorData.st_RawCAN, sizeof(RAW_CAN_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexCAN);

    pthread_mutex_lock(&st_SensorData.st_MutexCamera);
    memcpy(&(pst_Data->st_RawCamera), &st_SensorData.st_RawCamera, sizeof(RawImageFrame));
    pthread_mutex_unlock(&st_SensorData.st_MutexCamera);

    pthread_mutex_lock(&st_SensorData.st_MutexCamera);
    memcpy(&(pst_Data->st_RawCamera2), &st_SensorData.st_RawCamera2, sizeof(RawImageFrame));
    pthread_mutex_unlock(&st_SensorData.st_MutexCamera);

    pthread_mutex_lock(&st_SensorData.st_MutexYOLO);
    pst_Data->st_RawYOLO = st_SensorData.st_RawYOLO;     
    pthread_mutex_unlock(&st_SensorData.st_MutexYOLO);

}

void InitCAN(ros::NodeHandle &st_NodeHandle, pthread_t *pth_CAN)
{
    pthread_create(pth_CAN, NULL, ParseCAN, (void*)&st_NodeHandle);
}

void *TemporalThread(void *p_Arg)
{
    std::uint64_t processed_lidar_generation = 0;
    while (true)
    {
        pthread_mutex_lock(&st_DataTemporal);
        while (b_isRunning && ros::ok() &&
               processed_lidar_generation == u64_LidarFrameGeneration) {
            pthread_cond_wait(&st_DataTemporalCond, &st_DataTemporal);
        }
        if (!b_isRunning || !ros::ok())
        {
            pthread_mutex_unlock(&st_DataTemporal);
            break;
        }
        // Latest-frame-wins under overload: CopyData and the generation
        // snapshot are atomic with respect to the LiDAR callback.  If another
        // scan arrives while processing, the predicate remains different and
        // the next loop runs immediately without relying on another signal.
        const auto previous_generation = processed_lidar_generation;
        CopyData(&st_CurrentData);
        processed_lidar_generation = u64_LidarFrameGeneration;
        pthread_mutex_unlock(&st_DataTemporal);

        fusion_trace::Event("lidar_process_start",
            frame_boundary::SourceStamp(st_CurrentData.st_RawLiDAR).toNSec(),
            [&](std::ostream& out) {
                out << ",\"generation\":" << processed_lidar_generation
                    << ",\"skipped_generations\":"
                    << processed_lidar_generation - previous_generation - 1;
            });

        DeadReckoningProcess(st_CurrentData.st_RawGPS, st_CurrentData.st_RawIMU, st_CurrentData.st_RawCAN, st_GPSIMU);
        ProcessCameraStream(st_CurrentData.st_RawCamera, st_CurrentData.st_RawCamera2 ,st_Camera, st_GPSIMU);

        const ros::Time previous_lidar_stamp = ts_CurrTime;
        const auto lidar_pipeline_start = std::chrono::steady_clock::now();
        GPS_IMU lidar_pose = st_GPSIMU;
        std::string pose_reason;
        const auto scan_stamp=frame_boundary::SourceStamp(st_CurrentData.st_RawLiDAR);
        auto scan_time=frame_boundary::ScanTime(st_CurrentData.st_RawLiDAR);
        LocalInputSelection selected_inputs;
        const bool pose_ready = frame_boundary::Enabled()
            ? LookupLocalStateForLidarNs(scan_stamp.toNSec(),&lidar_pose,&selected_inputs,&pose_reason)
            : LookupSensorPoseForLidar(st_CurrentData.st_RawLiDAR.u64_Timestamp,&lidar_pose,&pose_reason);
        if(frame_boundary::Enabled() && pose_ready) {
            scan_time=frame_boundary::ScanTime(st_CurrentData.st_RawLiDAR);
            const auto map=LookupMapPoseForLidar(scan_time,lidar_pose);
            frame_boundary::PrepareScan(scan_time,lidar_pose,st_VehicleSpec,map,processed_lidar_generation);
            fusion_trace::Event("scan_input_selection",scan_stamp.toNSec(),[&](std::ostream& out) {
                out << ",\"generation\":" << processed_lidar_generation
                    << ",\"receipt_steady_ns\":" << scan_time.receipt_ns
                    << ",\"receipt_ros_ns\":" << st_CurrentData.st_RawLiDAR.receipt_ros_ns
                    << ",\"processing_steady_ns\":" << scan_time.processing_ns
                    << ",\"imu_stamp_ns\":" << selected_inputs.imu_stamp_ns
                    << ",\"imu_receipt_steady_ns\":" << selected_inputs.imu_receipt_steady_ns
                    << ",\"can_stamp_ns\":" << selected_inputs.can_stamp_ns
                    << ",\"can_receipt_steady_ns\":" << selected_inputs.can_receipt_steady_ns
                    << ",\"map_reason\":" << fusion_trace::Quote(map.reason);
            });
            if(map.valid) {
                const Eigen::Vector3d gps=map.map_from_base.translation()+map.map_from_base.rotation()*
                    Eigen::Vector3d(st_VehicleSpec.st_GPSPos.f32_X,st_VehicleSpec.st_GPSPos.f32_Y,st_VehicleSpec.st_GPSPos.f32_Z);
                lidar_pose.st_GPS.f64_E=gps.x(); lidar_pose.st_GPS.f64_N=gps.y(); lidar_pose.st_GPS.f64_U=gps.z();
                lidar_pose.st_GPS.b_isJamming=false;
            } else {
                lidar_pose.st_GPS.f64_E=lidar_pose.st_GPS.f64_N=lidar_pose.st_GPS.f64_U=NAN;
                lidar_pose.st_GPS.b_isJamming=true;
            }
        }
        if (!pose_ready) {
            ROS_WARN_THROTTLE(
                1.0, "Skipping LiDAR frame without synchronized pose: %s",
                pose_reason.c_str());
        }
        track_trace::Sink().SetScan(static_cast<std::int64_t>(processed_lidar_generation), scan_stamp.toNSec());
        const bool lidar_processed = pose_ready && LidarProcess(
            st_CurrentData.st_RawLiDAR, lidar_pose, st_LiDAR,
            *st_MotionClassifier);
        if (lidar_processed) {
            fusion_state_trace::Snapshot("class_lidar_ready", ts_CurrTime, ros::Time(),
                st_LiDAR.vec_Objects, processed_lidar_generation);
        }
        if (lidar_processed && b_EnableSensorFusion) {
            FusionProcess(st_CurrentData.st_RawYOLO, st_LiDAR, st_Camera,
                          ts_CurrTime);
        }
        if (lidar_processed) {
            fusion_state_trace::Snapshot("class_publish_input", ts_CurrTime, ros::Time(),
                st_LiDAR.vec_Objects, processed_lidar_generation);
        }
        if(frame_boundary::Enabled() && st_FrameBoundaryPublisher) {
            if(lidar_processed) st_FrameBoundaryPublisher->Publish(st_LiDAR);
            else st_FrameBoundaryPublisher->Invalid(scan_stamp,pose_ready?"empty_or_invalid_lidar_input":pose_reason);
        }
        // B0 output gate (Geometry/OutputGate.hpp). Common geometry OFF keeps the previous rule:
        // legacy publish iff lidar_processed, no V2, no SuppressMap, no SetLegacyReason.
        // Common geometry ON: at most one of /lidar/local_object and /lidar/map_object per scan, legacy only
        // with map_v2 and the frame-wide LegacyMotionReady gate, DELETEALL whenever withheld.
        const output_gate::Inputs gate=frame_boundary::GateInputs(
            pose_ready,pose_reason,lidar_processed,st_LiDAR,processed_lidar_generation);
        output_gate::Decision decision=output_gate::Decide(gate);
        if(gate.common_geometry && st_FrameBoundaryPublisher)
            decision=st_FrameBoundaryPublisher->PublishV2(st_LiDAR,scan_stamp,decision);
        const bool legacy_motion_ready=!gate.common_geometry ||
            (lidar_processed && frame_boundary::LegacyMotionReady(st_LiDAR));
        const output_gate::LegacyPlan legacy=output_gate::PlanLegacy(decision,gate,legacy_motion_ready);
        if(legacy.report_reason && st_FrameBoundaryPublisher)
            st_FrameBoundaryPublisher->SetLegacyReason(legacy.reason);
        if(legacy.suppress_markers && st_PlannerObjectRosPublisher)
            st_PlannerObjectRosPublisher->SuppressMap(scan_stamp);
        if (legacy.publish && st_PlannerObjectRosPublisher) {
            st_PlannerObjectRosPublisher->Publish(
                st_LiDAR, lidar_pose, st_VehicleSpec, ts_CurrTime);
        }
        if (b_BenchmarkLidarPipeline && !ts_CurrTime.isZero() &&
            ts_CurrTime != previous_lidar_stamp) {
            const auto lidar_pipeline_end = std::chrono::steady_clock::now();
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(
                    lidar_pipeline_end - lidar_pipeline_start).count();
            st_LidarPipelineBenchmark.Add(
                elapsed_ms, st_CurrentData.st_RawLiDAR.s32_Num,
                st_LiDAR.vec_Objects.size());
        }
        // UDP and the ROS legacy topics share the same boolean.
        if (legacy.publish && st_PlannerObjectSender) {
            st_PlannerObjectSender->Send(
                st_LiDAR, lidar_pose, st_VehicleSpec);
        }
        fusion_trace::Event("lidar_process_end",
            frame_boundary::SourceStamp(st_CurrentData.st_RawLiDAR).toNSec(),
            [&](std::ostream& out) {
                out << ",\"pose_ready\":" << (pose_ready ? "true" : "false")
                    << ",\"processed\":" << (lidar_processed ? "true" : "false")
                    << ",\"duration_ms\":" << std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - lidar_pipeline_start).count()
                    << ",\"objects\":" << st_LiDAR.vec_Objects.size();
                if (gate.common_geometry) {
                    out << ",\"v2_mode\":\"" << output_gate::ModeName(decision.v2) << "\""
                        << ",\"v2_reason\":" << fusion_trace::Quote(decision.reason)
                        << ",\"legacy_publish\":" << (legacy.publish ? "true" : "false");
                }
            });

        // 인지 전용: PlanningProcess/ControlProcess 호출 제거 (소스 부재)
        pthread_mutex_lock(&st_CopyMutex);
        // These types own STL/OpenCV/PCL containers. Byte-copying them
        // corrupts ownership metadata and causes an invalid free at shutdown.
        st_CameraTemporal = st_Camera;
        st_LiDARTemporal = st_LiDAR;
        st_GPSIMUTemporal = st_GPSIMU;
        pthread_mutex_unlock(&st_CopyMutex);
    }
    return nullptr;
}

// Experiment switches and the opt-in tracking trace (all default off = unchanged behaviour).
// Shared by main() and the offline replay harness.
void ConfigureTrackingExperiment(ros::NodeHandle& private_node)
{
    auto& options = tracking_options::Get();
    private_node.param("lidar_yaw_ema_enabled", options.yaw_ema_enabled, true);
    private_node.param("lidar_obb_long_axis_state", options.obb_long_axis_state, false);
    private_node.param("lidar_association_innovation_gate", options.association_innovation_gate, false);
    private_node.param("lidar_point_support_ownership", options.point_support_ownership, false);
    private_node.param("lidar_kf_reacquire_elapsed_init", options.reacquire_elapsed_init, false);
    private_node.param("lidar_kf_ego_translation_comp", options.ego_translation_comp, false);
    private_node.param("lidar_observation_quality_r", options.observation_quality_r, false);
    private_node.param("lidar_observation_obb_center", options.observation_obb_center, false);
    private_node.param("lidar_kf_jerk_q", options.jerk_q, false);
    private_node.param("lidar_kf_jerk_spectral_density", options.jerk_spectral_density, 1.0);
    private_node.param("lidar_ego_gap_reset", options.ego_gap_reset, false);
    private_node.param("lidar_kf_two_point_init_cov", options.two_point_init_cov, false);
    std::string path, ids, run_id;
    double max_mb = 512.0;
    private_node.param("lidar_track_trace_path", path, std::string());
    private_node.param("lidar_track_trace_ids", ids, std::string());
    private_node.param("lidar_track_trace_run_id", run_id, std::string("runtime"));
    private_node.param("lidar_track_trace_max_mb", max_mb, 512.0);
    track_trace::Sink().Configure(path, run_id, ids, max_mb);
    ROS_INFO("tracking experiment: reacquire_elapsed_init=%d ego_translation_comp=%d track_trace=%s",
             options.reacquire_elapsed_init ? 1 : 0, options.ego_translation_comp ? 1 : 0,
             path.empty() ? "off" : path.c_str());
}

void InitTemporal(ros::NodeHandle &st_NodeHandle, pthread_t *pth_Temporal)
{
    pthread_create(pth_Temporal, NULL, TemporalThread, NULL);
}

// void InitViewer(ros::NodeHandle &st_NodeHandle, pthread_t *pth_Viewer)
// {
//     VIEWERHZ = IMUHZ;
//     pthread_create(pth_Viewer, NULL, ParseViewer, (void*)&st_NodeHandle);
// }

int main(int argc, char **argv)
{
    pthread_t th_GPS, th_CAN, th_Camera, th_IMU, th_Lidar, th_Temporal;

    ros::init(argc, argv, "Main");
    ros::NodeHandle st_NodeHandle;
    ros::NodeHandle st_PrivateNodeHandle("~");
    std::string camera_class_profile;
    st_PrivateNodeHandle.param("camera_class_profile", camera_class_profile, std::string("mission16_v1"));
    if (camera_class_profile != "legacy" && camera_class_profile != "mission16_v1") {
        ROS_FATAL("Unsupported camera_class_profile"); return 1;
    }
    fusion_class::MissionProfile() = camera_class_profile == "mission16_v1";
    std::string fusion_trace_path;
    st_PrivateNodeHandle.param("fusion_trace_path", fusion_trace_path, std::string());
    try {
        fusion_trace::Sink().Configure(fusion_trace_path);
    } catch (const std::exception& error) {
        ROS_FATAL("%s", error.what());
        return 1;
    }
    st_PrivateNodeHandle.param(
        "enable_sensor_fusion", b_EnableSensorFusion, true);
    st_PrivateNodeHandle.param(
        "benchmark_lidar_pipeline", b_BenchmarkLidarPipeline, false);
    st_PrivateNodeHandle.param(
        "benchmark_warmup_frames", s32_BenchmarkWarmupFrames, 30);
    st_PrivateNodeHandle.param(
        "benchmark_sample_frames", s32_BenchmarkSampleFrames, 200);
    s32_BenchmarkWarmupFrames = std::max(0, s32_BenchmarkWarmupFrames);
    s32_BenchmarkSampleFrames = std::max(10, s32_BenchmarkSampleFrames);
    bool planner_udp_enabled = kPlannerUdpEnabledDefault;
    st_PrivateNodeHandle.param(
        "planner_udp_enabled", planner_udp_enabled, kPlannerUdpEnabledDefault);

    InitMutex();
    InitVehicleSpec();
    try {
        InitProjectionMatrix(st_Camera,YAML::LoadFile(YamlURL));
        frame_boundary::Configure(st_PrivateNodeHandle,st_VehicleSpec,st_Camera);
    } catch(const std::exception& error) {
        ROS_FATAL("Common geometry initialization failed: %s",error.what()); return 1;
    }
    st_FrameBoundaryPublisher=std::make_unique<frame_boundary::Publisher>(st_NodeHandle);
    ConfigureSensorPoseHistory(st_PrivateNodeHandle);
    try {
        ConfigureTrackingExperiment(st_PrivateNodeHandle);
    } catch (const std::exception& error) {
        ROS_FATAL("%s", error.what());
        return 1;
    }
    st_MotionClassifier =
        std::make_unique<MotionClassifier>(st_PrivateNodeHandle);
    st_PlannerObjectRosPublisher =
        std::make_unique<PlannerObjectRosPublisher>(st_NodeHandle);
    InitPlannerInterface(planner_udp_enabled);
    InitGlobal(st_NodeHandle, &th_GPS);
    InitIMU(st_NodeHandle, &th_IMU);
    InitializeCamera(st_NodeHandle, &th_Camera);

    InitLidarViz(st_NodeHandle);
    if (b_EnableSensorFusion) {
        InitFusionLidarViz(st_NodeHandle);
    } else {
        ROS_INFO("Camera-LiDAR sensor fusion disabled");
    }
    InitLidar(st_NodeHandle, &th_Lidar);

    InitCAN(st_NodeHandle, &th_CAN);
    InitTemporal(st_NodeHandle, &th_Temporal);
    // InitViewer(st_NodeHandle, &th_Viewer);
    
    // Worker threads own the ROS callbacks.  The former 100-us polling loop
    // woke this thread roughly ten thousand times per second for no work,
    // competing with LiDAR and pose synchronization during heavy frames.
    ros::waitForShutdown();

    // Wake the temporal worker and wait for every worker to finish before
    // global STL/OpenCV/PCL objects and ROS synchronization objects destruct.
    pthread_mutex_lock(&st_DataTemporal);
    b_isRunning = false;
    pthread_cond_broadcast(&st_DataTemporalCond);
    pthread_mutex_unlock(&st_DataTemporal);

    pthread_join(th_GPS, nullptr);
    pthread_join(th_CAN, nullptr);
    pthread_join(th_Camera, nullptr);
    pthread_join(th_IMU, nullptr);
    pthread_join(th_Lidar, nullptr);
    pthread_join(th_Temporal, nullptr);

    ReportFusionTotals();
    if(st_FrameBoundaryPublisher) st_FrameBoundaryPublisher->ReportTotals();
    fusion_trace::Sink().Flush();
    track_trace::Sink().Flush();
    // B-15: frame_boundary::Publisher owns a ros::WallTimer. Left to static destruction it is destroyed after
    // roscpp's function-local TimerManager<WallTime>::global() (created later, destroyed earlier) and
    // ~WallTimer -> TimerManager::remove() locks a destroyed mutex -> boost::lock_error -> std::terminate.
    st_FrameBoundaryPublisher.reset();

    return 0;
}
