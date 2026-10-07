#include "Parser.hpp"
#include <Parser/GpsSanityFilter.hpp>
#include <Parser/SensorPoseHistory.hpp>
#include <Geometry/FrameBoundary.hpp>
#include <SensorFusion/SensorFusion.hpp>
#include <SensorFusion/FusionTrace.hpp>

#include <algorithm>
#include <memory>
#include <mutex>

namespace
{
ros::Publisher g_CameraImagePublisher;
std::unique_ptr<GpsSanityFilter> g_GpsSanityFilter;
SensorPoseHistory g_SensorPoseHistory;
std::mutex g_GpsSanityMutex;
double g_LidarPoseWaitTimeoutS = 0.05;
}

void ConfigureSensorPoseHistory(ros::NodeHandle& private_node)
{
    SensorPoseHistoryConfig config;
    private_node.param("lidar_pose_gps_max_age_s", config.gps_max_age_s,
                       config.gps_max_age_s);
    private_node.param("lidar_pose_imu_max_age_s", config.imu_max_age_s,
                       config.imu_max_age_s);
    private_node.param("lidar_pose_can_max_age_s", config.can_max_age_s,
                       config.can_max_age_s);
    private_node.param("lidar_pose_maximum_speed_mps",
                       config.maximum_extrapolation_speed_mps,
                       config.maximum_extrapolation_speed_mps);
    private_node.param("lidar_pose_history_duration_s",
                       config.history_duration_s,
                       config.history_duration_s);
    private_node.param("lidar_pose_wait_timeout_s",
                       g_LidarPoseWaitTimeoutS,
                       g_LidarPoseWaitTimeoutS);
    g_LidarPoseWaitTimeoutS = std::max(0.0, g_LidarPoseWaitTimeoutS);
    g_SensorPoseHistory.Configure(config);
    ROS_INFO(
        "LiDAR pose synchronization: GPS age<=%.2fs IMU/CAN age<=%.2f/%.2fs extrapolation<=%.1fm/s callback_wait<=%.3fs",
        config.gps_max_age_s, config.imu_max_age_s, config.can_max_age_s,
        config.maximum_extrapolation_speed_mps,
        g_LidarPoseWaitTimeoutS);
}

bool LookupSensorPoseForLidar(std::uint64_t lidar_stamp_ms,
                              GPS_IMU* pose,
                              std::string* reason)
{
    return g_SensorPoseHistory.WaitForLookup(
        lidar_stamp_ms, g_LidarPoseWaitTimeoutS, pose, reason);
}

bool LookupLocalStateForLidar(std::uint64_t stamp_ms,GPS_IMU* local,std::string* reason) {
    return g_SensorPoseHistory.WaitForLocalLookup(stamp_ms,g_LidarPoseWaitTimeoutS,local,reason);
}
asmc::lidar::PoseSelection LookupMapPoseForLidar(std::uint64_t stamp_ms,const GPS_IMU& local) {
    return g_SensorPoseHistory.MapAt(stamp_ms,local);
}

// Parser.cpp 파일 역할 
// GPS/IMU -> 축 변환 후 SensorData에 담기
// LiDAR -> 축 변환, PCL 변환
// Camera -> Raw 데이터 이미지 형태로 변환 
// 변환 용도로만 작성, ~Processing 함수는 각 센서별 처리 절차 코드 담는 곳으로 사용

//==========================카메라========================== //  
void *ParseCamera(void *p_NodeHandle)
{
    ros::NodeHandle *pst_NodeHandle = (ros::NodeHandle *)p_NodeHandle;
    if (st_Camera.s32_CameraMode == 0)
    {
        ParseCameraUDP(*pst_NodeHandle);
    }
    else if (st_Camera.s32_CameraMode == 1)
    {
        ParseCameraROS(*pst_NodeHandle);
    }
    cv::destroyAllWindows();
    return nullptr;
}


void ParseCameraUDP(ros::NodeHandle &st_NodeHandle)
{
    UDPReceiver cls_Receiver_c1(st_Camera.front_camera_udp);
    cls_Receiver_c1.start();

    UDPReceiver cls_Receiver_c2(st_Camera.traffic_light_camera_udp);
    cls_Receiver_c2.start();

    ros::Publisher st_PublishImg = st_NodeHandle.advertise<sensor_msgs::CompressedImage>("/camera/image/compressed", 1);
    ros::Subscriber st_SubscribeYOLO = st_NodeHandle.subscribe("/YOLO/BBox", 1, YoloCallback);
    ros::Rate rate(VisionHZ);
    while (ros::ok())
    {
        size_t front_nbytes = 0;

        // Camera 1 (Sensor Fusion, )
        pthread_mutex_lock(&st_SensorData.st_MutexCamera);
        if (auto image_data_opt = cls_Receiver_c1.get_image_data())
        {
            vector<uint8>& image_data = *image_data_opt;
            if (!image_data.empty())
            {
                if (image_data.size() <= static_cast<size_t>(c_CAMERA_JPEG_SIZE))
                {
                    front_nbytes = image_data.size();
                    memcpy(st_SensorData.st_RawCamera.arc_Buffer,
                                image_data.data(),
                                front_nbytes);
                    st_SensorData.st_RawCamera.s32_Num = static_cast<int32_t>(front_nbytes);
                    st_SensorData.st_RawCamera.u64_Timestamp =
                        static_cast<uint64_t>(ros::Time::now().toNSec());
                }
                else
                {
                    ROS_WARN_THROTTLE(2.0, "Dropping oversized front-camera JPEG: %zu bytes", image_data.size());
                }
            }
        }
        sensor_msgs::CompressedImage msg;
        if (front_nbytes > 0)
        {
            msg.header.stamp.fromNSec(
                st_SensorData.st_RawCamera.u64_Timestamp);
            msg.format = "jpeg";
            msg.data.resize(front_nbytes);
            std::memcpy(msg.data.data(), st_SensorData.st_RawCamera.arc_Buffer, front_nbytes);
            // 필요시 frame_id도 설정
            msg.header.frame_id = "camera";
        }
        pthread_mutex_unlock(&st_SensorData.st_MutexCamera);


        size_t traffic_light_nbytes = 0;
        pthread_mutex_lock(&st_SensorData.st_MutexCamera);
        if (auto image_data_opt = cls_Receiver_c2.get_image_data())
        {
            vector<uint8>& image_data = *image_data_opt;
            if (!image_data.empty())
            {
                if (image_data.size() <= static_cast<size_t>(c_CAMERA_JPEG_SIZE))
                {
                    traffic_light_nbytes = image_data.size();
                    memcpy(st_SensorData.st_RawCamera2.arc_Buffer,
                                image_data.data(),
                                traffic_light_nbytes);
                    st_SensorData.st_RawCamera2.s32_Num = static_cast<int32_t>(traffic_light_nbytes);
                    st_SensorData.st_RawCamera2.u64_Timestamp = static_cast<uint64_t>(ros::Time::now().toNSec());
                }
            }
        }
        pthread_mutex_unlock(&st_SensorData.st_MutexCamera);

        if (front_nbytes > 0)
            st_PublishImg.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
    cls_Receiver_c1.stop();
    cls_Receiver_c2.stop();
    cv::destroyAllWindows();
}


void ParseCameraROS(ros::NodeHandle &st_NodeHandle)
{
    // Keep the original Main/YOLO interface intact while the outer MORAI
    // bridge owns the UDP receiver and publishes /image_jpeg/compressed.
    g_CameraImagePublisher =
        st_NodeHandle.advertise<sensor_msgs::CompressedImage>("/camera/image/compressed", 1);
    ros::Subscriber st_SubscribeCamera = st_NodeHandle.subscribe("/image_jpeg/compressed", 1, CameraRosCallback);
    ros::Subscriber st_SubscribeYOLO = st_NodeHandle.subscribe("/YOLO/BBox", 1, YoloCallback);
    ros::Rate rate(VisionHZ);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void CameraRosCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    // 수신 크기만큼만 복사 (기존: sizeof(RawImageFrame) 고정 복사 → 범위 밖 읽기)
    const size_t nbytes = std::min(msg->data.size(), static_cast<size_t>(c_CAMERA_JPEG_SIZE));
    pthread_mutex_lock(&st_SensorData.st_MutexCamera);
    memcpy(st_SensorData.st_RawCamera.arc_Buffer, msg->data.data(), nbytes);
    st_SensorData.st_RawCamera.s32_Num = static_cast<int32>(nbytes);
    const ros::Time source_stamp = msg->header.stamp.isZero()
        ? ros::Time::now() : msg->header.stamp;
    st_SensorData.st_RawCamera.u64_Timestamp =
        static_cast<uint64>(source_stamp.toNSec());
    pthread_mutex_unlock(&st_SensorData.st_MutexCamera);

    BufferFusionCameraFrame(source_stamp, msg->data);

    g_CameraImagePublisher.publish(msg);
}

void YoloCallback(const Msg::Yolo_bbox_arr::ConstPtr& data)
{
    YoloDetectionResult st_YOLO;

	for (int32 s32_Index = 0; s32_Index < data->yolo_bbox_arr.size(); s32_Index++)
	{
        const auto& detection = data->yolo_bbox_arr[s32_Index];
		BBox st_BBox;
		st_BBox.st_Max_pt.f32_X = detection.xMax;
        st_BBox.st_Max_pt.f32_Y = detection.yMax;
        st_BBox.st_Min_pt.f32_X = detection.xMin;
        st_BBox.st_Min_pt.f32_Y = detection.yMin;
        st_BBox.str_class = detection.classes;
        st_BBox.str_raw_class = detection.classes;
        st_YOLO.detected_objects.push_back(st_BBox);
    }
    const ros::Time image_stamp = data->header.stamp.isZero()
        ? ros::Time(0) : data->header.stamp;
    uint64 frame_seq = 0;
    pthread_mutex_lock(&st_SensorData.st_MutexYOLO);
    st_SensorData.st_RawYOLO.vec_YOLO = st_YOLO.detected_objects;
    st_SensorData.st_RawYOLO.b_Yolo_ready = true;
    st_SensorData.st_RawYOLO.u64_ImageTimestampNs =
        image_stamp.isZero() ? 0ULL : image_stamp.toNSec();
    frame_seq = ++st_SensorData.st_RawYOLO.u64_FrameSeq;
    pthread_mutex_unlock(&st_SensorData.st_MutexYOLO);

    BufferFusionYoloFrame(frame_seq, image_stamp,
                          st_YOLO.detected_objects);
}
//==========================카메라========================== //  





//==========================라이다========================== // 
void* ParseLiDAR(void* p_NodeHandle)
{
    ros::NodeHandle* pst_NodeHandle = static_cast<ros::NodeHandle*>(p_NodeHandle);
    LiDARParser_Driver_ROS(*pst_NodeHandle);
    return nullptr;
}

void LiDARParser_Driver_ROS(ros::NodeHandle &st_NodeHandle)
{   

    ros::Subscriber st_SubscribeLiDAR = st_NodeHandle.subscribe("/velodyne_points", 1, LiDARParser_Callback);
    ros::Rate rate(LiDARHZ);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void LiDARParser_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   
    if(frame_boundary::Enabled() && msg->header.stamp.isZero()) {
        ROS_WARN_THROTTLE(1.,"Rejected LiDAR scan: zero source stamp");
        return;
    }
    pcl::PointCloud<pcl::PointXYZI> pcl_Cloud;
    pcl::fromROSMsg(*msg, pcl_Cloud);

    if(frame_boundary::Enabled() && pcl_Cloud.size()>
        sizeof(st_SensorData.st_RawLiDAR.arc_Buffer)/sizeof(pcl::PointXYZI)) {
        ROS_WARN_THROTTLE(1.,"Rejected LiDAR scan: raw buffer capacity exceeded");
        return;
    }


    int32 s32_Count = pcl_Cloud.size();
    int32 s32_bytes = s32_Count * sizeof(pcl::PointXYZI);


    // Debugging logs
    // cout << "pcl_Cloud size: " << count << endl;
    // cout << "bytes: " << bytes << endl;


    // Match TemporalThread's lock order: the source buffer and generation must
    // be one atomic observation in the common path, including under overload.
    const bool atomic_observation=frame_boundary::Enabled();
    if(atomic_observation) pthread_mutex_lock(&st_DataTemporal);
    pthread_mutex_lock(&st_SensorData.st_MutexLiDAR);
    // cout << "Before setting st_RawLiDAR.s32_Num: " << st_SensorData.st_RawLiDAR.s32_Num << endl;

    // /velodyne_points 의 stamp 는 브릿지가 ros::Time::now() 로 찍은 ROS epoch 다.
    // GetMillisecond() 는 steady_clock(부팅 기준)이라 여기에 쓰면 이 노드가 발행하는
    // stamp 가 실제 시각과 약 17억 초(20698일) 어긋난다 — bag 분석에서 다른 토픽과
    // 맞출 수 없고 TF lookup 도 실패한다. 들어온 stamp 를 그대로 승계한다.
    // 단위는 ms 를 유지해야 한다 (LidarProcess 가 *1e6 으로 ns 로 되돌린다).
    const ros::Time ts_LiDARIn = msg->header.stamp.isZero() ? ros::Time::now()
                                                            : msg->header.stamp;
    const auto source_ms=ts_LiDARIn.toNSec()/1000000ULL;
    if(frame_boundary::Enabled() && (!source_ms ||
        source_ms<=st_SensorData.st_RawLiDAR.u64_Timestamp)) {
        pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR);
        if(atomic_observation) pthread_mutex_unlock(&st_DataTemporal);
        ROS_WARN_THROTTLE(1.,"Rejected LiDAR scan: duplicate or regressing source stamp");
        return;
    }
    st_SensorData.st_RawLiDAR.u64_Timestamp =
        static_cast<uint64>(source_ms);

    st_SensorData.st_RawLiDAR.s32_Num  = s32_Count;  // Set the number of points (not bytes)

        // cout << "arc_Buffer size: " << sizeof(st_SensorData.st_RawLiDAR.arc_Buffer) << endl;

    // Ensure arc_Buffer is large enough to hold all points
    memcpy(st_SensorData.st_RawLiDAR.arc_Buffer, pcl_Cloud.points.data(), s32_bytes);
    pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR);

    // LiDAR, not CAN, owns the perception pipeline cadence.  Signalling only
    // after the complete raw frame has been published also gives the worker a
    // real predicate to wait on instead of processing the same scan whenever
    // a vehicle-status callback happens to run.
    if(!atomic_observation) pthread_mutex_lock(&st_DataTemporal);
    ++u64_LidarFrameGeneration;
    const auto generation = u64_LidarFrameGeneration;
    pthread_cond_signal(&st_DataTemporalCond);
    pthread_mutex_unlock(&st_DataTemporal);
    fusion_trace::Event("lidar_input", ts_LiDARIn.toNSec(), [&](std::ostream& out) {
        out << ",\"generation\":" << generation << ",\"points\":" << s32_Count
            << ",\"buffer_stamp_ns\":" << (ts_LiDARIn.toNSec() / 1000000ULL) * 1000000ULL;
    });
        // cout << "After setting st_RawLiDAR.s32_Num: " << st_SensorData.st_RawLiDAR.s32_Num << endl;

}

//==========================라이다========================== // 



//==========================GPS========================== // 
void *ParseGPS(void *pst_NodeHandle)
{
    ros::NodeHandle *st_NodeHandle = (ros::NodeHandle *)pst_NodeHandle;
    ros::NodeHandle private_node("~");
    GpsSanityConfig sanity_config;
    private_node.param("gps_sanity_min_x", sanity_config.min_x,
                       sanity_config.min_x);
    private_node.param("gps_sanity_min_y", sanity_config.min_y,
                       sanity_config.min_y);
    private_node.param("gps_sanity_max_x", sanity_config.max_x,
                       sanity_config.max_x);
    private_node.param("gps_sanity_max_y", sanity_config.max_y,
                       sanity_config.max_y);
    private_node.param("gps_sanity_base_jump_allowance_m",
                       sanity_config.base_jump_allowance_m,
                       sanity_config.base_jump_allowance_m);
    private_node.param("gps_sanity_maximum_speed_mps",
                       sanity_config.maximum_speed_mps,
                       sanity_config.maximum_speed_mps);
    private_node.param("gps_sanity_reanchor_after_s",
                       sanity_config.reanchor_after_s,
                       sanity_config.reanchor_after_s);
    {
        std::lock_guard<std::mutex> lock(g_GpsSanityMutex);
        g_GpsSanityFilter.reset(new GpsSanityFilter(sanity_config));
    }
    ROS_INFO(
        "GPS sanity gate: bounds=[%.1f,%.1f]x[%.1f,%.1f] max_speed=%.1fm/s reanchor=%.1fs",
        sanity_config.min_x, sanity_config.max_x,
        sanity_config.min_y, sanity_config.max_y,
        sanity_config.maximum_speed_mps, sanity_config.reanchor_after_s);
    ros::Subscriber st_SubscribeGPS = st_NodeHandle->subscribe<morai_msgs::GPSMessage>("/gps", 1, GPSParser_Callback);
    ros::Rate rate(GPSHZ);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return nullptr;
}

void GPSParser_Callback(const morai_msgs::GPSMessage::ConstPtr &st_Msg)
{
    GPS_DATA st_GPSData;
    float64 f64_X, f64_Y, f64_Z, f64_Dx, f64_Dy, f64_Dz, f64_Latitude, f64_Longitude, f64_Altitude;
    float64 f64_Phi, f64_Lambda, f64_N;

    st_GPSData.f64_Latitude = st_Msg->latitude;
    st_GPSData.f64_Longitude = st_Msg->longitude;
    st_GPSData.f64_Altitude = st_Msg->altitude;
    st_GPSData.b_isJamming = (st_Msg->status == 0);
    const ros::Time received=ros::Time::now();
    const ros::Time original_stamp=st_Msg->header.stamp;
    auto invalidate=[&] {
        const auto stamp=original_stamp.isZero()?received:original_stamp;
        g_SensorPoseHistory.AddGpsInvalid(stamp.toNSec()/1000000ULL,received.toNSec()/1000000ULL);
    };
    if (st_Msg->status == 0 || (frame_boundary::Enabled() &&
        (original_stamp.isZero() || !std::isfinite(st_Msg->latitude) ||
         !std::isfinite(st_Msg->longitude) || !std::isfinite(st_Msg->altitude) ||
         !std::isfinite(st_Msg->eastOffset) || !std::isfinite(st_Msg->northOffset) ||
         !frame_boundary::MapOffsetsValid(st_Msg->eastOffset,st_Msg->northOffset) ||
         (st_Msg->latitude==0. && st_Msg->longitude==0.)))) {
        invalidate();
        ROS_WARN_THROTTLE(1.0, "Rejected GPS sample: invalid status");
        return;
    }

    int32 s32_Zone = 52;
    bool b_North = true;
    float64 f64_EastOffset = st_Msg->eastOffset;
    float64 f64_NorthOffset = st_Msg->northOffset;

    float64 f64_East, f64_North;
    GeographicLib::UTMUPS::Forward(
        st_GPSData.f64_Latitude,
        st_GPSData.f64_Longitude,
        s32_Zone, b_North, f64_East, f64_North
    );
    st_GPSData.f64_E = f64_East - f64_EastOffset;
    st_GPSData.f64_N = f64_North - f64_NorthOffset;
    st_GPSData.f64_U = frame_boundary::Enabled()
        ? st_Msg->altitude-frame_boundary::MapAltitudeOrigin() : 0;

    const ros::Time source_stamp = st_Msg->header.stamp.isZero()
        ? ros::Time::now() : st_Msg->header.stamp;
    GpsSanityDecision sanity;
    {
        std::lock_guard<std::mutex> lock(g_GpsSanityMutex);
        if (!g_GpsSanityFilter) {
            g_GpsSanityFilter.reset(new GpsSanityFilter());
        }
        sanity = g_GpsSanityFilter->Evaluate(
            st_GPSData.f64_E, st_GPSData.f64_N, source_stamp.toSec());
    }
    if (!sanity.accepted) {
        invalidate();
        ROS_WARN_THROTTLE(
            1.0,
            "Rejected GPS sample: status=%d map=(%.1f,%.1f) reason=%s jump=%.1fm allowed=%.1fm",
            st_Msg->status, st_GPSData.f64_E, st_GPSData.f64_N,
            sanity.reason.c_str(),
            sanity.jump_m, sanity.allowed_jump_m);
        return;
    }

    const std::uint64_t source_stamp_ms =
        static_cast<std::uint64_t>(source_stamp.toNSec() / 1000000ULL);
    g_SensorPoseHistory.AddGps(source_stamp_ms, st_GPSData);

    // cout << st_GPSData.f64_E << " " << st_GPSData.f64_N << endl;

    // f64_Phi = Deg2Rad(st_GPSData.f64_Latitude);
    // f64_Lambda = Deg2Rad(st_GPSData.f64_Longitude);
    // f64_N = c_LLA2ENU_A / sqrt(1 - c_LLA2ENU_E2 * sin(f64_Phi) * sin(f64_Phi));

    // f64_X = (f64_N + st_GPSData.f64_Altitude) * cos(f64_Phi) * cos(f64_Lambda);
    // f64_Y = (f64_N + st_GPSData.f64_Altitude) * cos(f64_Phi) * sin(f64_Lambda);
    // f64_Z = ((1 - c_LLA2ENU_E2) * f64_N + st_GPSData.f64_Altitude) * sin(f64_Phi);

    // f64_Dx = f64_X - st_GPSRef.f64_RefX;
    // f64_Dy = f64_Y - st_GPSRef.f64_RefY;
    // f64_Dz = f64_Z - st_GPSRef.f64_RefZ;

    // st_GPSData.f64_E = -sin(st_GPSRef.f64_RefLon) * f64_Dx + cos(st_GPSRef.f64_RefLon) * f64_Dy;
    // st_GPSData.f64_N = -sin(st_GPSRef.f64_RefLat) * cos(st_GPSRef.f64_RefLon) * f64_Dx - sin(st_GPSRef.f64_RefLat) * sin(st_GPSRef.f64_RefLon) * f64_Dy + cos(st_GPSRef.f64_RefLat) * f64_Dz;

    pthread_mutex_lock(&st_SensorData.st_MutexGPS);
    st_SensorData.st_RawGPS.u64_Timestamp = source_stamp_ms;
    st_SensorData.st_RawGPS.s32_Num = sizeof(GPS_DATA);
    memcpy(st_SensorData.st_RawGPS.arc_Buffer, &st_GPSData, sizeof(GPS_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexGPS);
}
//==========================GPS========================== // 


//==========================CAN========================== //
void *ParseCAN(void *pst_NodeHandle)
{
    ros::NodeHandle *st_NodeHandle = (ros::NodeHandle *)pst_NodeHandle;
    ros::Subscriber st_SubscribeCAN = st_NodeHandle->subscribe<morai_msgs::EgoVehicleStatus>("/Competition_topic", 1, CANParser_Callback);
    ros::Publisher st_PublishCMD = st_NodeHandle->advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);
    ros::Rate rate(CANHZ);
    while (ros::ok())
    {
        ros::spinOnce();
        morai_msgs::CtrlCmd ctrl_msg;
        ctrl_msg.longlCmdType = 1;

        ctrl_msg.accel = st_Control.f32_accelCmd;
        ctrl_msg.brake = st_Control.f32_brakeCmd;
        ctrl_msg.steering = st_Control.f32_TargetSteer_Rad;

        st_PublishCMD.publish(ctrl_msg);
        rate.sleep();
    }
    return nullptr;
}

void CANParser_Callback(const morai_msgs::EgoVehicleStatus::ConstPtr& st_Msg)
{
    CAN_DATA st_CANData;
    st_CANData.f32_CurrentVel_KPH = 3.6 * sqrt(pow(st_Msg->velocity.x, 2) + pow(st_Msg->velocity.y, 2) + pow(st_Msg->velocity.z, 2));
    st_CANData.f32_CurrentVel_MPS = sqrt(pow(st_Msg->velocity.x, 2) + pow(st_Msg->velocity.y, 2) + pow(st_Msg->velocity.z, 2));
    if(frame_boundary::Enabled() && st_Msg->header.stamp.isZero()) {
        st_CANData.f32_CurrentVel_MPS=NAN; st_CANData.f32_CurrentVel_KPH=NAN;
    }
    st_CANData.f32_Steer_deg = st_Msg->wheel_angle;
    st_CANData.f32_Steer_rad = Deg2Rad(st_Msg->wheel_angle);

    const ros::Time source_stamp = st_Msg->header.stamp.isZero()
        ? ros::Time::now() : st_Msg->header.stamp;
    const std::uint64_t source_stamp_ms =
        static_cast<std::uint64_t>(source_stamp.toNSec() / 1000000ULL);
    std::string sanity_reason;
    if (!g_SensorPoseHistory.AddCan(
            source_stamp_ms, st_CANData, &sanity_reason)) {
        ROS_WARN_THROTTLE(
            1.0, "Rejected vehicle-speed sample: %s speed=%.2fm/s",
            sanity_reason.c_str(), st_CANData.f32_CurrentVel_MPS);
        return;
    }

    pthread_mutex_lock(&st_SensorData.st_MutexCAN);
    st_SensorData.st_RawCAN.u64_Timestamp = source_stamp_ms;
    st_SensorData.st_RawCAN.s32_Num = sizeof(CAN_DATA);
    memcpy(st_SensorData.st_RawCAN.arc_Buffer, &st_CANData, sizeof(CAN_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexCAN);
}

//==========================IMU========================== // 
void *ParseIMU(void *pst_NodeHandle)
{
    ros::NodeHandle *st_NodeHandle = (ros::NodeHandle *)pst_NodeHandle;
    ros::Subscriber st_SubscribeIMU = st_NodeHandle->subscribe<sensor_msgs::Imu>("/imu", 1, IMUParser_Callback);
    ros::Rate rate(IMUHZ);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return nullptr;
}

void IMUParser_Callback(const sensor_msgs::Imu::ConstPtr& st_Msg)
{
    IMU_DATA st_IMUData;

    st_IMUData.f32_AccelX = st_Msg->linear_acceleration.x;
    st_IMUData.f32_AccelY = st_Msg->linear_acceleration.y;
    st_IMUData.f32_AccelZ = st_Msg->linear_acceleration.z;

    st_IMUData.f32_AngularVelX = st_Msg->angular_velocity.x;
    st_IMUData.f32_AngularVelY = st_Msg->angular_velocity.y;
    st_IMUData.f32_AngularVelZ = st_Msg->angular_velocity.z;

    st_IMUData.f32_QuatX = st_Msg->orientation.x;
    st_IMUData.f32_QuatY = st_Msg->orientation.y;
    st_IMUData.f32_QuatZ = st_Msg->orientation.z;
    st_IMUData.f32_QuatW = st_Msg->orientation.w;

    st_IMUData.f32_Roll_rad = atan2(2.0 * (st_IMUData.f32_QuatW * st_IMUData.f32_QuatX + st_IMUData.f32_QuatY * st_IMUData.f32_QuatZ),
                                    1.0 - 2.0 * (st_IMUData.f32_QuatX * st_IMUData.f32_QuatX + st_IMUData.f32_QuatY * st_IMUData.f32_QuatY));
    st_IMUData.f32_Pitch_rad = asin(2.0 * (st_IMUData.f32_QuatW * st_IMUData.f32_QuatY - st_IMUData.f32_QuatZ * st_IMUData.f32_QuatX));
    st_IMUData.f32_Yaw_rad = atan2(2.0 * (st_IMUData.f32_QuatW * st_IMUData.f32_QuatZ + st_IMUData.f32_QuatX * st_IMUData.f32_QuatY), 
                                    1.0 - 2.0 * (st_IMUData.f32_QuatY * st_IMUData.f32_QuatY + st_IMUData.f32_QuatZ * st_IMUData.f32_QuatZ));
    
    st_IMUData.f32_Roll_deg = Rad2Deg(st_IMUData.f32_Roll_rad);
    st_IMUData.f32_Pitch_deg = Rad2Deg(st_IMUData.f32_Pitch_rad);
    st_IMUData.f32_Yaw_deg = Rad2Deg(st_IMUData.f32_Yaw_rad);
    st_IMUData.b_ImuReady=true; //라이다에서 Kalman.cpp에서 사용중
    if(frame_boundary::Enabled()) {
        const auto& q=st_Msg->orientation;
        const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        st_IMUData.b_ImuReady=!st_Msg->header.stamp.isZero() && std::isfinite(norm) && std::abs(norm-1.)<=.001;
    }

    const ros::Time source_stamp = st_Msg->header.stamp.isZero()
        ? ros::Time::now() : st_Msg->header.stamp;
    const std::uint64_t source_stamp_ms =
        static_cast<std::uint64_t>(source_stamp.toNSec() / 1000000ULL);
    std::string sanity_reason;
    if (!g_SensorPoseHistory.AddImu(
            source_stamp_ms, st_IMUData, &sanity_reason)) {
        ROS_WARN_THROTTLE(
            1.0, "Rejected IMU sample: %s yaw=%.1fdeg",
            sanity_reason.c_str(), st_IMUData.f32_Yaw_deg);
        return;
    }

    pthread_mutex_lock(&st_SensorData.st_MutexIMU);
    st_SensorData.st_RawIMU.u64_Timestamp = source_stamp_ms;
    st_SensorData.st_RawIMU.s32_Num = sizeof(IMU_DATA);
    memcpy(st_SensorData.st_RawIMU.arc_Buffer, &st_IMUData, sizeof(IMU_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexIMU);

}
//==========================IMU========================== // 


//==========================Viewer========================== // 
// void *ParseViewer(void *pst_NodeHandle)
// {
//     ros::NodeHandle *st_NodeHandle = (ros::NodeHandle *)pst_NodeHandle;
//     ros::Publisher st_PublishViewer = st_NodeHandle->advertise<visualization_msgs::MarkerArray>("/showAll", 1);
//     ros::Rate rate(VIEWERHZ);

//     while (ros::ok())
//     {
//         ShowAll(st_PublishViewer, st_VehicleSpec, st_GPSIMUTemporal, st_ControlTemporal, st_PlanningTemporal, st_LiDARTemporal);
//         rate.sleep();
//     }
// }
//==========================Viewer========================== // 
