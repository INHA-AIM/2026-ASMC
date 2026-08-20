#include <Global/Global.hpp>
#include <Parser/Parser.hpp>
#include <Camera/Camera.hpp>
#include <Lidar/LidarProcess/LidarProcess.hpp>
#include <SensorFusion/SensorFusion.hpp>
#include <PlannerInterface/PlannerObjectSender.hpp>
#include <ros/package.h>
#include <memory>


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

int32 GPSHZ, VisionHZ, LiDARHZ, IMUHZ, CANHZ;
int32 VIEWERHZ; // InitViewer에서, 기준이 되는 센서의 HZ 할당해주기.

SENSOR_DATA st_SensorData;
SENSOR_DATA st_CurrentData;

// ==================== 전역 Mutex ==================== //
pthread_mutex_t st_CopyMutex;
pthread_mutex_t st_DataTemporal;
pthread_cond_t st_DataTemporalCond;

bool b_isRunning = true;

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

void InitPlannerInterface()
{
    YAML::Node config = YAML::LoadFile(YamlURL);
    const YAML::Node planner = config["PlannerInterface"];
    if (!planner || !planner["Enabled"].as<bool>(false)) {
        ROS_INFO("Planner object interface disabled");
        return;
    }

    st_PlannerObjectSender = std::make_unique<PlannerObjectSender>(
        planner["IP"].as<string>(),
        planner["Port"].as<int>(),
        planner["MaxObjects"].as<std::size_t>(20));
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

void InitLidar(ros::NodeHandle &st_NodeHandle, pthread_t *pth_Lidar)
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

    st_LiDAR.st_Clustering.b_Enabled = config["LiDAR"]["Clustering"]["Enabled"].as<bool>();
    st_LiDAR.st_Clustering.f32_Eps = config["LiDAR"]["Clustering"]["Eps"].as<float32>();
    st_LiDAR.st_Clustering.s32_MinPoints = config["LiDAR"]["Clustering"]["MinPoints"].as<int32>();
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
    // uint64_t u64_StartTime_ms = 0;
    while (true)
    {
        pthread_mutex_lock(&st_DataTemporal);
        pthread_cond_wait(&st_DataTemporalCond, &st_DataTemporal);
        if (!b_isRunning || !ros::ok())
        {
            pthread_mutex_unlock(&st_DataTemporal);
            break;
        }
        CopyData(&st_CurrentData);
        pthread_mutex_unlock(&st_DataTemporal);

        DeadReckoningProcess(st_CurrentData.st_RawGPS, st_CurrentData.st_RawIMU, st_CurrentData.st_RawCAN, st_GPSIMU);
        ProcessCameraStream(st_CurrentData.st_RawCamera, st_CurrentData.st_RawCamera2 ,st_Camera, st_GPSIMU);

        LidarProcess(st_CurrentData.st_RawLiDAR, st_GPSIMU, st_LiDAR);
        FusionProcess(st_CurrentData.st_RawYOLO, st_LiDAR, st_Camera);
        if (st_PlannerObjectSender) {
            st_PlannerObjectSender->Send(st_LiDAR, st_GPSIMU, st_VehicleSpec);
        }

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

    InitMutex();
    InitVehicleSpec();
    InitPlannerInterface();
    InitGlobal(st_NodeHandle, &th_GPS);
    InitIMU(st_NodeHandle, &th_IMU);
    InitializeCamera(st_NodeHandle, &th_Camera);

    InitLidarViz(st_NodeHandle);
    InitFusionLidarViz(st_NodeHandle);
    InitLidar(st_NodeHandle, &th_Lidar);

    InitCAN(st_NodeHandle, &th_CAN);
    InitTemporal(st_NodeHandle, &th_Temporal);
    // InitViewer(st_NodeHandle, &th_Viewer);
    
    while(ros::ok())
    {
        usleep(100);
    }

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

    return 0;
}
