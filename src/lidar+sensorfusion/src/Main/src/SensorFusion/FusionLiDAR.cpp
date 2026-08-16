// FusionLiDAR.cpp
#include "SensorFusion.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <limits>
#include <algorithm>
#include <string>
#include <iostream>

// 통합 퍼블리셔
static ros::Publisher pub_ObjectBoxes;   // 모든 객체 박스
static ros::Publisher pub_ObjectPcl;     // 모든 클러스터 포인트

void InitFusionLidarViz(ros::NodeHandle& nh)
{
    pub_ObjectBoxes = nh.advertise<visualization_msgs::MarkerArray>("/lidar/object_boxes", 1, false);
    pub_ObjectPcl   = nh.advertise<sensor_msgs::PointCloud2>        ("/lidar/object",       1, false);

    // 시각화 기본 색상 (시안) 및 텍스트 표기 on
    g_box_color.r = 0.0f; g_box_color.g = 1.0f; g_box_color.b = 1.0f; g_box_color.a = 0.35f;
    g_show_text   = true;
}

// z-floor 기준으로 ground/noise 라벨링:
// 대부분이 바닥(z<eff_floor)이고(flat/sparse) "낮은" 클러스터만 noise
// 필요시 프레임별 동적 바닥 추정(dynamic_floor) 지원
void SetLiDARClassWithGroundNoise(LiDAR& st_LiDAR, const std::string& class_name, float z_floor)
{
    double z_margin = 0.08;  //동적 바닥에 더하는 여유(m)
    double noise_height_th  = 0.25;  //낮다고 볼 최대 높이(m)
    double vehicle_height_safeguard = 0.90;  //이 높이 이상이면 절대 noise 아님
    double noise_ratio_th = 0.70;  //지면점 비율 임계
    int noise_min_pts = 30;    //너무 희박하면 noise로 보기도

    bool dynamic_floor = false; //프레임별 바닥 추정 on/off

    // ---- 프레임별 동적 바닥 임계 계산 ----
    float32 eff_floor = z_floor;
    if (dynamic_floor) {
        float32 frame_min = std::numeric_limits<float32>::max();
        for (const auto& obj : st_LiDAR.vec_Objects) {
            for (const auto& pt : obj.pcl_ClusterPoints.points) {
                if (pt.z < frame_min) frame_min = pt.z;
            }
        }
        if (frame_min < std::numeric_limits<float32>::max()) {
            eff_floor = frame_min + static_cast<float32>(z_margin);
        }
    }

    // ---- 각 클러스터 라벨링 ----
    int n_noise = 0, n_other = 0;
    for (auto& obj : st_LiDAR.vec_Objects)
    {
        const auto& cloud = obj.pcl_ClusterPoints;
        if (cloud.empty()) { obj.str_ObjectClass = "unknown"; continue; }

        const int N = static_cast<int>(cloud.size());
        float32 minZ =  std::numeric_limits<float32>::max();
        float32 maxZ = -std::numeric_limits<float32>::max();
        int below = 0;

        for (const auto& pt : cloud.points) {
            if (pt.z < minZ) minZ = pt.z;
            if (pt.z > maxZ) maxZ = pt.z;
            if (pt.z < eff_floor) ++below;
        }

        const float32 height = maxZ - minZ;
        const float   ratio  = (N > 0) ? static_cast<float>(below) / static_cast<float>(N) : 1.0f;

        const bool mostly_ground = (ratio > noise_ratio_th);
        const bool flat          = (height < noise_height_th);
        const bool sparse        = (N < noise_min_pts);
        const bool tall_object   = (height >= vehicle_height_safeguard);

        if ((mostly_ground && (flat || sparse)) && !tall_object) {
            obj.str_ObjectClass = "noise";
            ++n_noise;
        } else {
            obj.str_ObjectClass = class_name; 
            ++n_other;
        }
    }
}


// 클러스터 포인트로 AABB 계산 (축정렬)
static void UpdateAABB(LiDAR& st_LiDAR)
{
    for (auto& obj : st_LiDAR.vec_Objects)
    {
        if (obj.pcl_ClusterPoints.empty()) {
            // 비어있으면 기존 중심 유지
            obj.f32_MinX = obj.f32_MaxX = obj.f32_CoreX;
            obj.f32_MinY = obj.f32_MaxY = obj.f32_CoreY;
            obj.f32_MinZ = obj.f32_MaxZ = obj.f32_CoreZ;
            obj.f32_Yaw  = 0.0f;
            continue;
        }

        float32 minX =  std::numeric_limits<float32>::max();
        float32 minY =  std::numeric_limits<float32>::max();
        float32 minZ =  std::numeric_limits<float32>::max();
        float32 maxX = -std::numeric_limits<float32>::max();
        float32 maxY = -std::numeric_limits<float32>::max();
        float32 maxZ = -std::numeric_limits<float32>::max();

        for (const auto& pt : obj.pcl_ClusterPoints.points) {
            if (pt.x < minX) minX = pt.x;
            if (pt.y < minY) minY = pt.y;
            if (pt.z < minZ) minZ = pt.z;
            if (pt.x > maxX) maxX = pt.x;
            if (pt.y > maxY) maxY = pt.y;
            if (pt.z > maxZ) maxZ = pt.z;
        }

        obj.f32_MinX = minX; obj.f32_MaxX = maxX;
        obj.f32_MinY = minY; obj.f32_MaxY = maxY;
        obj.f32_MinZ = minZ; obj.f32_MaxZ = maxZ;

        obj.f32_CoreX = 0.5f*(minX+maxX);
        obj.f32_CoreY = 0.5f*(minY+maxY);
        obj.f32_CoreZ = 0.5f*(minZ+maxZ);
        obj.f32_Yaw   = 0.0f; // 축정렬
    }
}

// 마커 & 포인트클라우드 퍼블리시
static void PublishBoxesAndPcl(LiDAR& st_LiDAR)
{
    // ---- MarkerArray (바운딩 박스 + 텍스트) ----
    if (pub_ObjectBoxes) {
        visualization_msgs::MarkerArray arr;

        // 이전 마커 삭제
        {
            visualization_msgs::Marker clear;
            clear.header.frame_id = "velodyne";
            clear.header.stamp    = ros::Time::now();
            clear.action          = visualization_msgs::Marker::DELETEALL;
            arr.markers.push_back(clear);
        }

        const ros::Time stamp = ros::Time::now();

        for (size_t idx=0; idx<st_LiDAR.vec_Objects.size(); ++idx) {
            const auto& obj = st_LiDAR.vec_Objects[idx];
            if (obj.pcl_ClusterPoints.empty()) continue;

            // 박스
            visualization_msgs::Marker box;
            box.header.frame_id = "velodyne";
            box.header.stamp    = stamp;
            box.ns              = "object_bbox";
            box.id              = static_cast<int>(idx);
            box.type            = visualization_msgs::Marker::CUBE;
            box.action          = visualization_msgs::Marker::ADD;

            box.pose.position.x = obj.f32_CoreX;
            box.pose.position.y = obj.f32_CoreY;
            box.pose.position.z = obj.f32_CoreZ;
            box.pose.orientation.w = 1.0;

            box.scale.x = std::max(0.01f, obj.f32_MaxX - obj.f32_MinX);
            box.scale.y = std::max(0.01f, obj.f32_MaxY - obj.f32_MinY);
            box.scale.z = std::max(0.01f, obj.f32_MaxZ - obj.f32_MinZ);

            // noise는 회색/반투명, 그 외는 기본색
            if (obj.str_ObjectClass == "noise") {
                box.color.r = 0.5f; box.color.g = 0.5f; box.color.b = 0.5f; box.color.a = 0.3f;
            } else {
                box.color = g_box_color; box.color.a = 0.5f;
            }

            arr.markers.push_back(box);

            // 텍스트
            if (g_show_text && !obj.str_ObjectClass.empty()) {
                visualization_msgs::Marker text;
                text.header.frame_id = "velodyne";
                text.header.stamp    = stamp;
                text.ns              = "object_label";
                text.id              = static_cast<int>(100000 + idx);
                text.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text.action          = visualization_msgs::Marker::ADD;
                text.pose.position.x = obj.f32_CoreX;
                text.pose.position.y = obj.f32_CoreY;
                text.pose.position.z = obj.f32_MaxZ + 0.3f;
                text.scale.z         = (obj.str_ObjectClass == "noise") ? 0.4 : 0.6;
                if (obj.str_ObjectClass == "noise") {
                    text.color.r=0.7; text.color.g=0.7; text.color.b=0.7; text.color.a=0.8;
                } else {
                    text.color.r=1.0; text.color.g=1.0; text.color.b=1.0; text.color.a=0.9;
                }
                text.text = obj.str_ObjectClass;
                arr.markers.push_back(text);
            }
        }

        pub_ObjectBoxes.publish(arr);
    }

    // ---- PointCloud2 (전체 클러스터) ----
    if (pub_ObjectPcl) {
        pcl::PointCloud<pcl::PointXYZI> pcl_All;
        for (const auto& obj : st_LiDAR.vec_Objects) {
            pcl_All += obj.pcl_ClusterPoints;
        }
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(pcl_All, msg);
        msg.header.frame_id = "velodyne";
        msg.header.stamp    = ros::Time::now();
        pub_ObjectPcl.publish(msg);
    }
}

void FusionLiDAR(LiDAR &st_LiDAR)
{
    // 고정 크기 로직 제거 → 클러스터 기반 AABB & 시각화만 수행
    UpdateAABB(st_LiDAR);
    PublishBoxesAndPcl(st_LiDAR);
}
