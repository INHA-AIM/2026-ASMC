// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/common/common.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <cmath>
// #include <vector>
// #include <Eigen/Dense>
// #include <Msg/LidarPointArray.h>
// #include <tf/transform_datatypes.h>
// #include <morai_msgs/EgoVehicleStatus.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include "tracking_utils.h"
// #include "globals.h"
// #include <morai_msgs/EgoVehicleStatus.h>
#include "Visualization.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


visualization_msgs::Marker createBoxMarker(
    const LiDARObjectInfo &o, int32 s32_Id, const std_msgs::Header& header)
{
    visualization_msgs::Marker marker;
    marker.header = header;
    float32 f32_SizeX = o.f32_MaxX - o.f32_MinX;
    float32 f32_SizeY = o.f32_MaxY - o.f32_MinY;
    float32 f32_SizeZ = o.f32_MaxZ - o.f32_MinZ;

    // ── 2) 크기 필터링: 원하시는 범위(예: X:[0,15], Y:[0,3.5], Z:[0,4]) 이외는 제거
    if (f32_SizeX < 0.0f   || f32_SizeX > 15.0f ||
        f32_SizeY < 0.0f   || f32_SizeY >  3.5f ||
        f32_SizeZ < 0.0f   || f32_SizeZ >  4.0f)
    {
        marker.action = visualization_msgs::Marker::DELETE;
        return marker;
    }

    
    marker.ns     = "box";
    marker.id     = s32_Id;
    marker.type   = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = o.f32_CoreX;
    marker.pose.position.y = o.f32_CoreY;
    marker.pose.position.z = o.f32_CoreZ;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = f32_SizeX;
    marker.scale.y = f32_SizeY;
    marker.scale.z = f32_SizeZ;

    // miss_count 에 따라 색상 변경
    if (o.s32_MissCount > 0) {
        // 파란색
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    }
    // else {
    //     // 녹색
    //     marker.color.r = 0.0f;
    //     marker.color.g = 1.0f;
    //     marker.color.b = 0.0f;
    // }
    marker.color.a = 0.4f;
    marker.lifetime = ros::Duration(0.0);
    return marker;
}

visualization_msgs::Marker createOBBMarker(
    const LiDARObjectInfo &o, int32 s32_Id, const std_msgs::Header& header)
{
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns     = "obb";
    marker.id     = s32_Id;
    marker.type   = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // 선 두께
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // 12개 모서리 연결
    static const std::array<std::pair<int,int>,12> edges = {{
        {0,1},{1,2},{2,3},{3,0},   // 하층
        {4,5},{5,6},{6,7},{7,4},   // 상층
        {0,4},{1,5},{2,6},{3,7}    // 수직
    }};
    for (auto e : edges) {
        geometry_msgs::Point p1, p2;
        p1.x = o.OBB_Corners_Body[e.first].x;
        p1.y = o.OBB_Corners_Body[e.first].y;
        p1.z = o.OBB_Corners_Body[e.first].z;
        p2.x = o.OBB_Corners_Body[e.second].x;
        p2.y = o.OBB_Corners_Body[e.second].y;
        p2.z = o.OBB_Corners_Body[e.second].z;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
    marker.lifetime = ros::Duration(0.0);
    return marker;
}


visualization_msgs::Marker createTextMarker(
    const LiDARObjectInfo &o, int32 s32_Id, const std_msgs::Header& header)
{
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns     = "id";
    marker.id     = s32_Id + 1000;
    marker.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = o.f32_CoreX;
    marker.pose.position.y = o.f32_CoreY;
    marker.pose.position.z = o.f32_CoreZ + 1.5;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.z = 0.7;  // 글자 크기

    // miss_count 에 따라 색상 변경
    if (o.s32_MissCount > 0) {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    } else {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }
    marker.color.a = 1.0f;

    char buf[128];
    // int len = snprintf(buf, sizeof(buf),
    //     "ID:%d\n|v|:%.2f km/h\n",
    //     o.s32_Id, o.f32_Velocity * 3.6f);
    const char* object_class =
        o.str_ObjectClass.empty() ? "unclassified" : o.str_ObjectClass.c_str();
    int len = snprintf(buf, sizeof(buf),
        "ID:%d MOVE:%d CLS:%s\n|v|:%.2f km/h\n",
        o.s32_Id, o.b_IsDynamic, object_class, o.f32_Velocity * 3.6f);


    const int safe_len = std::max(0, std::min(len, static_cast<int>(sizeof(buf) - 1)));
    marker.text = string(buf, safe_len);

    marker.lifetime = ros::Duration(0.0);
    return marker;
    return marker;
}

visualization_msgs::Marker createArrowMarker(
    const LiDARObjectInfo &o, int s32_Id, const std_msgs::Header& header)
{
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns     = "velocity";
    marker.id     = s32_Id + 2000;
    marker.type   = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // (1) 포즈 위치는 무시해도 되지만, orientation은 반드시 초기화
    marker.pose.position.x    = 0.0;
    marker.pose.position.y    = 0.0;
    marker.pose.position.z    = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // (2) points 로 화살표 방향 지정
    geometry_msgs::Point start, end;
    float scale_factor = 0.5f;
    start.x = o.f32_CoreX;
    start.y = o.f32_CoreY;
    start.z = o.f32_CoreZ;
    end.x   = o.f32_CoreX + o.f32_DirX * scale_factor;
    end.y   = o.f32_CoreY + o.f32_DirY * scale_factor;
    end.z   = o.f32_CoreZ;
    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.05;  // shaft diameter
    marker.scale.y = 0.1;   // head diameter
    marker.scale.z = 0.1;

    // 색상
    if (o.s32_MissCount > 0) {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    } else {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration(0.0);

    return marker;
}

// void InitLidarViz(ros::NodeHandle& nh, ros::Publisher& marker_pub) {
//     marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/lidar_tracking_markers", 1);
// }
