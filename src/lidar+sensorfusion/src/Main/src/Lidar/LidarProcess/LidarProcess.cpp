#include <Lidar/LidarProcess/LidarProcess.hpp>
#include <Lidar/LidarProcess/PipelineProfile.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <Geometry/FrameBoundary.hpp>
#include "PipelineTrace.hpp"


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
    msg.header.stamp    = frame_boundary::SourceStamp(raw);
    msg.header.frame_id = frame;
    RawPub().publish(msg);
}

static inline void PublishCloudWithPub(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                       const RAW_LIDAR_DATA& raw, const char* frame,
                                       ros::Publisher& pub)
{
    if(frame_boundary::Enabled() && !pub.getNumSubscribers()) return;
    sensor_msgs::PointCloud2 msg;
    if (cloud) {
        pcl::toROSMsg(*cloud, msg);
    } else {
        pcl::PointCloud<pcl::PointXYZI> empty;
        pcl::toROSMsg(empty, msg);
    }
    msg.header.stamp    = frame_boundary::SourceStamp(raw);
    msg.header.frame_id = frame;
    pub.publish(msg);
}

bool LidarProcess(const RAW_LIDAR_DATA& st_RawLiDAR,
                  const GPS_IMU& st_GPSIMU,
                  LiDAR& st_LiDAR,
                  MotionClassifier& motion_classifier)

{
    auto start = std::chrono::steady_clock::now();  

    pthread_mutex_lock(&st_SensorData.st_MutexLiDAR); //LiDAR 파이프라인 잠금 시작

    static uint64_t last_raw = 0;
    const uint64_t raw = frame_boundary::Enabled()?frame_boundary::SourceStamp(st_RawLiDAR).toNSec():st_RawLiDAR.u64_Timestamp;
    if (raw == last_raw || (frame_boundary::Enabled() && raw<last_raw)) {
        pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR); // 반드시 잠금 해제
        return false; // 중복 프레임은 처리/갱신/속도계산 전부 스킵
    }
    last_raw = raw;

    // An empty cloud has no plane or clusters to estimate. Returning here
    // prevents repeated PCL RANSAC errors while invalid UDP is rejected.
    if (st_RawLiDAR.s32_Num <= 0) {
        pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR);
        return false;
    }

    // [MOVE/SET] ROS 시간은 여기서, raw(ms) -> ns 변환
    ts_CurrTime = frame_boundary::SourceStamp(st_RawLiDAR);

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
    g_LidarStageProfile.roi_points = static_cast<int>(roi_cloud->points.size());
    LIDAR_STAGE(roi_pub_ms,
                PublishCloudWithPub(roi_cloud, st_RawLiDAR, "velodyne", RoiPub()));
    LIDAR_STAGE(voxel_ms, Voxel(roi_cloud, voxeled_cloud));
    g_LidarStageProfile.voxel_points =
        static_cast<int>(voxeled_cloud->points.size());
    LIDAR_STAGE(imu_correct_ms,
                CorrectingPoints(voxeled_cloud, imu_compensation_cloud, st_GPSIMU));
    LIDAR_STAGE(ransac_ms,
                NormalVectorFiltering(imu_compensation_cloud, ransac_cloud, st_GPSIMU));
    pipeline_trace::Ground(ts_CurrTime,*voxeled_cloud,*imu_compensation_cloud,
        *ransac_cloud,CurrentGroundPlaneSnapshot());
    g_LidarStageProfile.ransac_points =
        static_cast<int>(ransac_cloud->points.size());
    LIDAR_STAGE(ransac_pub_ms,
                PublishCloudWithPub(ransac_cloud, st_RawLiDAR, frame_boundary::Enabled()?"lidar_leveled":"velodyne", RansacPub()));

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
    LIDAR_STAGE(cluster_ms,
                Clustering(ransac_cloud, clustering_cloud, all_cluster_indices,
                           st_GPSIMU, st_LiDAR.st_Clustering));
    g_LidarStageProfile.cluster_points =
        static_cast<int>(clustering_cloud->points.size());
    g_LidarStageProfile.clusters = static_cast<int>(all_cluster_indices.size());
    LIDAR_STAGE(cluster_pub_ms,
                PublishCloudWithPub(clustering_cloud, st_RawLiDAR, frame_boundary::Enabled()?"lidar_leveled":"velodyne",
                                    ClusterPub()));
    if (!frame_boundary::Enabled() || (frame_boundary::Current().map_pose.valid && MapClusterPub().getNumSubscribers()))
    LIDAR_STAGE(map_cluster_pub_ms,
                PublishCloudWithPub(
                    TransformCloudToMap(clustering_cloud, st_GPSIMU, st_VehicleSpec),
                    st_RawLiDAR, "map", MapClusterPub()));

    LIDAR_STAGE(objectinfo_ms,
                Objectinfo_for_km(object, ransac_cloud, clustering_cloud,
                                  all_cluster_indices, st_GPSIMU, st_VehicleSpec));
    LIDAR_STAGE(framefilter_ms,
                FrameFilter(prev_objects_queue, object, st_GPSIMU, st_VehicleSpec));
    
    pipeline_trace::Tracking(ts_CurrTime,ts_PrevTime,object);
    // PublishCloud(clustering_cloud, st_RawLiDAR, "map");
    header.stamp = ts_CurrTime;      // 바운딩 박스 stamp 전달 
    header.frame_id = frame_boundary::Enabled()?"lidar_leveled":"velodyne";

    // cout<<"시간텀 :"<<ts_CurrTime-ts_PrevTime<<endl;
    ts_PrevTime = ts_CurrTime;
    // cout<<"After:"<<ts_PrevTime<<endl;

    if(frame_boundary::Enabled()) st_LiDAR.frame_context=frame_boundary::Current();

    // 모든 트랙에서 판정 이력을 먼저 갱신한다. 주행가능영역 필터는 이후
    // Publisher 경계에서 적용하므로 도로 진입 전 이력을 잃지 않는다.
    if(frame_boundary::Enabled() && !frame_boundary::Current().map_pose.valid) {
        motion_classifier.InvalidateMapHistory(object);
    } else {
        LIDAR_STAGE(motion_ms,
                    motion_classifier.Update(object, ts_CurrTime, st_GPSIMU, st_VehicleSpec));
    }

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
    const auto ground_plane = CurrentGroundPlaneSnapshot();
    for (auto& tracked_object : object) {
        if(frame_boundary::Enabled() && tracked_object.s32_MissCount==0)
            frame_boundary::RecordTrackingFrame(tracked_object);
        tracked_object.ground_evidence = ObjectGroundEvidence{};
        if (tracked_object.s32_MissCount == 0) {
            tracked_object.ground_evidence = MeasureObjectGroundEvidence(
                tracked_object.pcl_ClusterPoints.points, ground_plane);
        }
    }
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
    return true;
}   
