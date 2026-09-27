#include <Lidar/LidarProcess/LidarProcess.hpp>
#include <Lidar/LidarProcess/PipelineProfile.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <Lidar/TunnelFilter/TunnelFilterRos.hpp>
#include <PlannerInterface/PlannerObjectAdapter.hpp>


ros::Publisher marker_pub;
LidarStageProfile g_LidarStageProfile;
//deque<vector<LiDARObjectInfo>> prev_objects_queue;// 전프레임에 저장되어있는 벡터
extern SENSOR_DATA st_SensorData;
extern VehicleSpec st_VehicleSpec;
extern Planning st_Planning;

void InitLidarViz(ros::NodeHandle& nh)
{
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "/lidar_tracking_markers", 1);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MakePointCloudFromRaw(const RAW_LIDAR_DATA& st_RawLiDAR)
{

    //1포인트 크기 (x, y, z, intensity = 4 * float32 = 16바이트)
    uint32 u32_Bytes  = st_RawLiDAR.s32_Num * sizeof(pcl::PointXYZI);  // bytes 계산


    if (u32_Bytes < sizeof(pcl::PointXYZI)) {
        cout << "IN - Not enough data" << endl;
        return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    }


    uint32 u32_PointsCount = u32_Bytes / sizeof(pcl::PointXYZI);
    // cout << "Calculated points_count (number of points): " << u32_PointsCount << endl;  //포인트 개 확인

    //PCL 포인트클라우드 생성
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_RawCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl_RawCloud->points.resize(u32_PointsCount);

    //RAW 버퍼에서 PCL points 복사
    memcpy(pcl_RawCloud->points.data(), st_RawLiDAR.arc_Buffer, u32_PointsCount * sizeof(pcl::PointXYZI));

    pcl_RawCloud->width    = u32_PointsCount;
    pcl_RawCloud->height   = 1;
    pcl_RawCloud->is_dense = false;

    return pcl_RawCloud;
}


static ros::Publisher& RawPub()
{
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar/debug_raw", 1);
    return pub;
}

static ros::Publisher& RansacPub()
{
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar/debug_ransac", 1);
    return pub;
}

static ros::Publisher& RoiPub()
{
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar/debug_roi", 1);
    return pub;
}


static ros::Publisher& ClusterPub()
{
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar/debug_clustered", 1);
    return pub;
}

static ros::Publisher& MapClusterPub()
{
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(
        "/lidar/debug_clustered_map", 1);
    return pub;
}

static pcl::PointCloud<pcl::PointXYZI>::Ptr TransformCloudToMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const GPS_IMU& gps_imu,
    const VehicleSpec& vehicle)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(
        new pcl::PointCloud<pcl::PointXYZI>);
    if (!cloud) return transformed;
    transformed->points.reserve(cloud->points.size());
    for (const auto& point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        const PlannerPoint3D map_point = TransformLidarPointToMap(
            {point.x, point.y, point.z}, gps_imu, vehicle);
        pcl::PointXYZI output;
        output.x = static_cast<float>(map_point.x);
        output.y = static_cast<float>(map_point.y);
        output.z = static_cast<float>(map_point.z);
        output.intensity = point.intensity;
        transformed->points.push_back(output);
    }
    transformed->width = static_cast<std::uint32_t>(transformed->points.size());
    transformed->height = 1;
    transformed->is_dense = false;
    return transformed;
}

static inline void PublishCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                const RAW_LIDAR_DATA& raw, const char* frame)
{
    sensor_msgs::PointCloud2 msg;
    if (cloud) {
        pcl::toROSMsg(*cloud, msg);         // 비어 있어도 OK (width=0, height=0)
    } else {
        pcl::PointCloud<pcl::PointXYZI> empty;
        pcl::toROSMsg(empty, msg);
    }
    msg.header.stamp    = ros::Time().fromNSec((uint64)raw.u64_Timestamp * 1000000ULL);
    msg.header.frame_id = frame;
    RawPub().publish(msg);
}

static inline void PublishCloudWithPub(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                       const RAW_LIDAR_DATA& raw, const char* frame,
                                       ros::Publisher& pub)
{
    sensor_msgs::PointCloud2 msg;
    if (cloud) {
        pcl::toROSMsg(*cloud, msg);
    } else {
        pcl::PointCloud<pcl::PointXYZI> empty;
        pcl::toROSMsg(empty, msg);
    }
    msg.header.stamp    = ros::Time().fromNSec((uint64)raw.u64_Timestamp * 1000000ULL);
    msg.header.frame_id = frame;
    pub.publish(msg);
}

void LidarProcess(const RAW_LIDAR_DATA& st_RawLiDAR,
                  const GPS_IMU& st_GPSIMU,
                  LiDAR& st_LiDAR,
                  MotionClassifier& motion_classifier)

{
    auto start = std::chrono::steady_clock::now();  

    pthread_mutex_lock(&st_SensorData.st_MutexLiDAR); //LiDAR 파이프라인 잠금 시작

    static uint64_t last_raw = 0;
    const uint64_t raw = st_RawLiDAR.u64_Timestamp;
    if (raw == last_raw) {
        pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR); // 반드시 잠금 해제
        return; // 중복 프레임은 처리/갱신/속도계산 전부 스킵
    }
    last_raw = raw;

    // An empty cloud has no plane or clusters to estimate. Returning here
    // prevents repeated PCL RANSAC errors while invalid UDP is rejected.
    if (st_RawLiDAR.s32_Num <= 0) {
        pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR);
        return;
    }

    // [MOVE/SET] ROS 시간은 여기서, raw(ms) -> ns 변환
    ts_CurrTime = ros::Time().fromNSec((uint64_t)raw * 1000000ULL); // ms → ns

    g_LidarStageProfile.Reset();
    g_LidarStageProfile.raw_points = st_RawLiDAR.s32_Num;

    deque<vector<LiDARObjectInfo>> prev_objects_queue = st_LiDAR.Prev_Objects_Queue;
    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxeled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr imu_compensation_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inout_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustering_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    
    vector<LiDARObjectInfo> object; // 현재프레임 객체 
    vector<pcl::PointIndices> all_cluster_indices; // 바운딩 박스 만들기 위한 클러스터 인덱스 정보
    std_msgs::Header header; //바운딩박스 만들때 사용하기 위한 거 

    //const int MAX_FRAMES = 5;// prev_objects_queue 최대 프레임 수

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_RawCloud;
    LIDAR_STAGE(make_cloud_ms,
                pcl_RawCloud = MakePointCloudFromRaw(st_RawLiDAR));
    // PublishCloud(pcl_RawCloud, st_RawLiDAR, "map");

    LIDAR_STAGE(roi_ms, Roi(pcl_RawCloud, roi_cloud, st_LiDAR, st_VehicleSpec));

    // GPS 음영에서만 터널 전용 ROI를 적용한다. 전역 st_LiDAR 설정을 바꾸지
    // 않으므로 GPS가 복구된 다음 프레임부터 일반 ROI가 자동으로 사용된다.
    if (tunnel_filter::TunnelExperimentEnabled()) {
        const tunnel_filter::Params& tunnel_params =
            tunnel_filter::TunnelExperimentParams();
        const bool tunnel_roi_active = tunnel_params.dynamic_roi_enable &&
            tunnel_filter::IsGpsDenied(st_GPSIMU.st_GPS.b_isJamming,
                                       st_GPSIMU.st_GPS.b_isTunnel);
        if (tunnel_roi_active) {
            const auto first_removed = std::remove_if(
                roi_cloud->points.begin(), roi_cloud->points.end(),
                [&tunnel_params](const pcl::PointXYZI& point) {
                    return point.x < tunnel_params.roi_min_x_m ||
                           point.x > tunnel_params.roi_max_x_m ||
                           point.y < tunnel_params.roi_min_y_m ||
                           point.y > tunnel_params.roi_max_y_m ||
                           point.z < tunnel_params.roi_min_z_m ||
                           point.z > tunnel_params.roi_max_z_m;
                });
            roi_cloud->points.erase(first_removed, roi_cloud->points.end());
            roi_cloud->width = static_cast<std::uint32_t>(roi_cloud->points.size());
            roi_cloud->height = 1;
        }

        static bool previous_tunnel_roi_active = false;
        if (tunnel_roi_active != previous_tunnel_roi_active) {
            if (tunnel_roi_active) {
                ROS_INFO("Tunnel ROI active: x=[%.1f,%.1f] y=[%.1f,%.1f] z=[%.1f,%.1f]",
                         tunnel_params.roi_min_x_m, tunnel_params.roi_max_x_m,
                         tunnel_params.roi_min_y_m, tunnel_params.roi_max_y_m,
                         tunnel_params.roi_min_z_m, tunnel_params.roi_max_z_m);
            } else {
                ROS_INFO("Tunnel ROI inactive: restored normal LiDAR ROI");
            }
            previous_tunnel_roi_active = tunnel_roi_active;
        }
    }
    g_LidarStageProfile.roi_points = static_cast<int>(roi_cloud->points.size());
    LIDAR_STAGE(roi_pub_ms,
                PublishCloudWithPub(roi_cloud, st_RawLiDAR, "velodyne", RoiPub()));
    LIDAR_STAGE(voxel_ms, Voxel(roi_cloud, voxeled_cloud));
    g_LidarStageProfile.voxel_points =
        static_cast<int>(voxeled_cloud->points.size());
    // GPS 음영에서는 터널 필터를 먼저 끝내고, 그 생존 클러스터를 아래의 기존
    // FrameFilter/MotionClassifier에 넣는다. 진단 토픽과 /lidar_nogps가 같은
    // 프레임의 같은 필터 결과를 사용하도록 여기서 동기 처리한다.
    bool tunnel_output_ready = false;
    if (tunnel_filter::TunnelExperimentEnabled()) {
        tunnel_filter::TunnelExperimentCapture(voxeled_cloud, ts_CurrTime,
                                               st_GPSIMU);
        tunnel_filter::TunnelExperimentRun();
        tunnel_output_ready =
            tunnel_filter::TunnelExperimentOutputReady(ts_CurrTime);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr object_source_cloud = ransac_cloud;
    std::vector<std::string> tunnel_yolo_classes;
    if (tunnel_output_ready) {
        const auto& frame = tunnel_filter::TunnelExperimentLastFrame();
        const auto& result = *frame.result;
        object_source_cloud = result.leveled;
        imu_compensation_cloud = result.leveled;

        // /lidar/debug_ransac은 터널 모드에서는 지면·벽·천장을 모두 제거한 최종
        // 점군을 보여준다. 객체 생성용 cluster index는 원본 leveled cloud 기준을
        // 유지하므로 object_source_cloud와 분리한다.
        ransac_cloud->points.reserve(result.kept_indices.size());
        for (const int index : result.kept_indices) {
            ransac_cloud->points.push_back(
                result.leveled->points[static_cast<std::size_t>(index)]);
        }
        ransac_cloud->width =
            static_cast<std::uint32_t>(ransac_cloud->points.size());
        ransac_cloud->height = 1;
        ransac_cloud->is_dense = false;

        // OBB 안전 필터에서 제거되지 않은 cluster만 기존 tracker에 전달한다.
        // centroid 순서와 cluster 순서는 Objectinfo_for_km의 계약상 같아야 한다.
        for (std::size_t i = 0;
             i < result.clusters.size() && i < result.objects.size(); ++i) {
            const auto& tunnel_object = result.objects[i];
            if (tunnel_object.flags.removed) continue;
            all_cluster_indices.push_back(result.clusters[i]);
            pcl::PointXYZI centroid;
            centroid.x = centroid.y = centroid.z = 0.0f;
            centroid.intensity = 0.0f;
            for (const int index : result.clusters[i].indices) {
                const auto& point =
                    result.leveled->points[static_cast<std::size_t>(index)];
                centroid.x += point.x;
                centroid.y += point.y;
                centroid.z += point.z;
            }
            const float point_count = static_cast<float>(
                std::max<std::size_t>(1, result.clusters[i].indices.size()));
            centroid.x /= point_count;
            centroid.y /= point_count;
            centroid.z /= point_count;
            clustering_cloud->points.push_back(centroid);
            tunnel_yolo_classes.push_back(
                tunnel_object.is_protected
                    ? tunnel_filter::ProtectClassName(
                          tunnel_object.protect_class)
                    : "unknown");
        }
        clustering_cloud->width =
            static_cast<std::uint32_t>(clustering_cloud->points.size());
        clustering_cloud->height = 1;
        clustering_cloud->is_dense = false;
        g_LidarStageProfile.ransac_points =
            static_cast<int>(ransac_cloud->points.size());
        g_LidarStageProfile.cluster_points =
            static_cast<int>(clustering_cloud->points.size());
        g_LidarStageProfile.clusters =
            static_cast<int>(all_cluster_indices.size());
    } else {
        LIDAR_STAGE(imu_correct_ms,
                    CorrectingPoints(voxeled_cloud, imu_compensation_cloud,
                                     st_GPSIMU));

        // 터널 지붕은 GPS가 무효가 되기 전에 먼저 보인다. 터널 기능을 켠
        // 실행에서는 GPS 정상 경로도 동일한 z 상한을 RANSAC 전에 적용해
        // /last_lidar가 입구 천장을 장애물로 만들지 않게 한다. 벽/지면 전용
        // 필터는 여전히 GPS 음영일 때만 실행한다.
        if (tunnel_filter::TunnelExperimentEnabled()) {
            const auto& tunnel_params =
                tunnel_filter::TunnelExperimentParams();
            if (tunnel_params.pre_gps_ceiling_filter_enable) {
                std::vector<std::uint8_t> labels(
                    imu_compensation_cloud->points.size(),
                    tunnel_filter::kKept);
                const int ceiling_removed = tunnel_filter::MarkCeiling(
                    *imu_compensation_cloud,
                    tunnel_params.ceiling_max_z_lidar_m, &labels);
                if (ceiling_removed > 0) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(
                        new pcl::PointCloud<pcl::PointXYZI>());
                    ceiling_filtered->points.reserve(
                        imu_compensation_cloud->points.size() -
                        static_cast<std::size_t>(ceiling_removed));
                    for (std::size_t i = 0;
                         i < imu_compensation_cloud->points.size(); ++i) {
                        if (labels[i] == tunnel_filter::kCeiling) continue;
                        ceiling_filtered->points.push_back(
                            imu_compensation_cloud->points[i]);
                    }
                    ceiling_filtered->width = static_cast<std::uint32_t>(
                        ceiling_filtered->points.size());
                    ceiling_filtered->height = 1;
                    ceiling_filtered->is_dense = false;
                    imu_compensation_cloud = ceiling_filtered;
                    ROS_INFO_THROTTLE(
                        2.0,
                        "Pre-GPS ceiling filter removed %d points above z=%.2fm",
                        ceiling_removed,
                        tunnel_params.ceiling_max_z_lidar_m);
                }
            }
        }
        LIDAR_STAGE(ransac_ms,
                    NormalVectorFiltering(imu_compensation_cloud, ransac_cloud,
                                          st_GPSIMU));
        g_LidarStageProfile.ransac_points =
            static_cast<int>(ransac_cloud->points.size());
        LIDAR_STAGE(cluster_ms,
                    Clustering(ransac_cloud, clustering_cloud,
                               all_cluster_indices, st_GPSIMU));
        g_LidarStageProfile.cluster_points =
            static_cast<int>(clustering_cloud->points.size());
        g_LidarStageProfile.clusters =
            static_cast<int>(all_cluster_indices.size());
    }
    LIDAR_STAGE(ransac_pub_ms,
                PublishCloudWithPub(ransac_cloud, st_RawLiDAR, "velodyne",
                                    RansacPub()));


    // for (const auto& pt : ransac_cloud->points)
    // {
    //     // (1) 유한한 값이 아니면 스킵
    //     if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
    //         continue;
    //     Object ob;
    //     ob.f32_CoreX=pt.x;
    //     ob.f32_CoreY=pt.y;
    //     Body2ENU(st_VehicleSpec,st_GPSIMU,ob);
    //     // (2) Planning 영역 밖이면 스킵

    //     if (!InOrOut(st_Planning, ob.f32_CoreE, ob.f32_CoreN))
    //         continue;

    //     // 조건 통과한 포인트만 추가
    //     inout_cloud->points.push_back(pt);
    // }

    // //width/height 메타데이터 갱신
    // inout_cloud->width  = static_cast<uint32_t>(inout_cloud->points.size());
    // inout_cloud->height = 1;   // unorganized
    // inout_cloud->is_dense = false;
    LIDAR_STAGE(cluster_pub_ms,
                PublishCloudWithPub(clustering_cloud, st_RawLiDAR, "velodyne",
                                    ClusterPub()));
    LIDAR_STAGE(map_cluster_pub_ms,
                PublishCloudWithPub(
                    TransformCloudToMap(clustering_cloud, st_GPSIMU, st_VehicleSpec),
                    st_RawLiDAR, "map", MapClusterPub()));

    LIDAR_STAGE(objectinfo_ms,
                Objectinfo_for_km(object, object_source_cloud, clustering_cloud,
                                  all_cluster_indices, st_GPSIMU, st_VehicleSpec));

    // 새 터널 필터의 YOLO 보호 결과를 추적 객체 초기 분류에 보존한다. 이후의
    // 기존 sensor fusion은 정상적으로 같은 객체를 갱신할 수 있다.
    if (tunnel_output_ready) {
        for (std::size_t i = 0;
             i < object.size() && i < tunnel_yolo_classes.size(); ++i) {
            if (tunnel_yolo_classes[i] == "unknown") continue;
            object[i].str_ObjectClass = tunnel_yolo_classes[i];
            object[i].str_YoloClass = tunnel_yolo_classes[i];
        }
    }

    // GPS가 끊기기 직전에는 z 상한으로 천장 윗부분을 잘라도, 남은 천장과
    // 양쪽 벽이 도로 전체를 가로지르는 하나의 거대 OBB로 합쳐질 수 있다.
    // 터널 기능을 명시적으로 켠 GPS 정상 경로에서만, 세 축이 모두 구조물
    // 크기인 OBB를 tracker와 /last_lidar에 들어가기 전에 제거한다.
    if (!tunnel_output_ready && tunnel_filter::TunnelExperimentEnabled()) {
        const auto& tunnel_params = tunnel_filter::TunnelExperimentParams();
        const auto before = object.size();
        object.erase(
            std::remove_if(
                object.begin(), object.end(),
                [&](const LiDARObjectInfo& value) {
                    return tunnel_filter::IsPreGpsStructuralObb(
                        value.f32_OBB_Width, value.f32_OBB_Height,
                        value.f32_OBB_ZMax - value.f32_OBB_ZMin,
                        tunnel_params);
                }),
            object.end());
        const auto removed = before - object.size();
        if (removed > 0) {
            ROS_WARN_THROTTLE(
                1.0,
                "Pre-GPS structural OBB guard removed %zu tunnel-size objects",
                removed);
        }
    }

    // 일반/터널 검출기가 바뀌는 경계에서는 서로 다른 cluster geometry를 같은
    // track으로 오인하지 않도록 이전 queue만 비운다. 새 ID는 전역 증가값을 써서
    // 재사용되지 않고, 터널 내부에서는 이후 프레임부터 안정적으로 유지된다.
    static bool previous_tunnel_output_ready = false;
    if (tunnel_output_ready != previous_tunnel_output_ready) {
        prev_objects_queue.clear();
        st_LiDAR.Prev_Objects_Queue.clear();
        previous_tunnel_output_ready = tunnel_output_ready;
    }
    LIDAR_STAGE(framefilter_ms,
                FrameFilter(prev_objects_queue, object, st_GPSIMU, st_VehicleSpec));
    
    // PublishCloud(clustering_cloud, st_RawLiDAR, "map");
    header.stamp = ts_CurrTime;      // 바운딩 박스 stamp 전달 
    header.frame_id = "velodyne";

    // cout<<"시간텀 :"<<ts_CurrTime-ts_PrevTime<<endl;
    ts_PrevTime = ts_CurrTime;
    // cout<<"After:"<<ts_PrevTime<<endl;

    // 모든 트랙에서 판정 이력을 먼저 갱신한다. 주행가능영역 필터는 이후
    // Publisher 경계에서 적용하므로 도로 진입 전 이력을 잃지 않는다.
    LIDAR_STAGE(motion_ms, motion_classifier.Update(object, ts_CurrTime));

    // object를 f32_Dist 기준으로 정렬한 후 st_LiDAR.vec_Objects에 저장
    sort(object.begin(), object.end(), [](const LiDARObjectInfo& a, const LiDARObjectInfo& b) {return a.f32_Dist < b.f32_Dist;});// 작은 값 우선

    // // === 경계 밖 객체 제거 ===
    // object.erase(
    // std::remove_if(object.begin(), object.end(),
    //     [&](const LiDARObjectInfo& o){
    //         if (o.f32_CoreX == 0.f && o.f32_CoreY == 0.f && o.f32_CoreZ == 0.f) return true;
    //         if (!std::isfinite(o.f32_CoreE) || !std::isfinite(o.f32_CoreN))     return true;
    //         return !InOrOut(st_Planning, o.f32_CoreE, o.f32_CoreN);
    //     }),
    // object.end());

    // 정렬된 결과를 st_LiDAR.vec_Objects에 복사
    st_LiDAR.vec_Objects = object;
    prev_objects_queue.push_back(object);
    st_LiDAR.Prev_Objects_Queue = prev_objects_queue;
    const int MAX_FRAMES = 3; // 과거 3 프레임까지의 기록을 유지
    if (st_LiDAR.Prev_Objects_Queue.size() > MAX_FRAMES) {
        st_LiDAR.Prev_Objects_Queue.pop_front();
    }

    // prev_objects_queue.push_back(object);
    // if ((int)prev_objects_queue.size() > MAX_FAMES) prev_objects_queue.pop_front();

    pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR);

    visualization_msgs::MarkerArray marker_array;
    // Marker ID가 배열 index이고 수명이 무한이므로, 이전 프레임에서
    // 관측되다 사라진 OBB가 RViz에 계속 남을 수 있다. 매 프레임
    // 초기화하고 현재 관측(miss_count == 0)된 OBB만 다시 그린다.
    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    for (int i = 0; i < object.size(); ++i) {
        if(object[i].f32_CoreX==0&&object[i].f32_CoreY==0&&object[i].f32_CoreZ==0){
            break;
        }

        if (object[i].s32_MissCount != 0) {
            continue;
        }

        marker_array.markers.push_back(createTextMarker(object[i], i, header));
        marker_array.markers.push_back(createArrowMarker(object[i], i, header));
        // marker_array.markers.push_back(createBoxMarker(object[i], i, header));
        marker_array.markers.push_back(createOBBMarker(object[i], i, header));
    }

    LIDAR_STAGE(marker_ms, marker_pub.publish(marker_array));
    g_LidarStageProfile.valid = true;
}   
