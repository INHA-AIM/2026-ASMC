#include <gtest/gtest.h>

#include <Lidar/Tracking/tracking_utils.hpp>

#include <deque>
#include <vector>

namespace {

LiDARObjectInfo MakeDetection(float center_x, float center_y,
                              float width, float height,
                              float z_offset) {
    LiDARObjectInfo object{};
    object.f32_CoreX = center_x;
    object.f32_CoreY = center_y;
    object.f32_CoreZ = z_offset;
    object.f32_MinX = center_x - width * 0.5F;
    object.f32_MaxX = center_x + width * 0.5F;
    object.f32_MinY = center_y - height * 0.5F;
    object.f32_MaxY = center_y + height * 0.5F;
    object.f32_MinZ = z_offset;
    object.f32_MaxZ = z_offset + 0.2F;
    object.f32_OBB_Width = width;
    object.f32_OBB_Height = height;
    for (int x_index = 0; x_index < 4; ++x_index) {
        for (int y_index = 0; y_index < 3; ++y_index) {
            pcl::PointXYZI point;
            point.x = object.f32_MinX +
                width * static_cast<float>(x_index) / 3.0F;
            point.y = object.f32_MinY +
                height * static_cast<float>(y_index) / 2.0F;
            point.z = z_offset;
            object.pcl_ClusterPoints.push_back(point);
        }
    }
    return object;
}

TEST(TrackingAssociation, ConsolidatesVerticallySplitSameOccupancy) {
    st_Tracking.f32_DuplicateCenterDistance = 0.60F;
    st_Tracking.f32_DuplicateIouThreshold = 0.35F;
    std::vector<LiDARObjectInfo> detections;
    detections.push_back(MakeDetection(10.0F, 1.0F, 1.0F, 0.8F, -0.5F));
    detections.push_back(MakeDetection(10.02F, 1.01F, 1.0F, 0.8F, 0.2F));

    ConsolidateCurrentDetections(detections);

    ASSERT_EQ(detections.size(), 1U);
    EXPECT_EQ(detections.front().pcl_ClusterPoints.size(), 24U);
}

TEST(TrackingAssociation, KeepsNearbyNonOverlappingObjectsSeparate) {
    st_Tracking.f32_DuplicateCenterDistance = 0.60F;
    st_Tracking.f32_DuplicateIouThreshold = 0.35F;
    std::vector<LiDARObjectInfo> detections;
    detections.push_back(MakeDetection(10.0F, 1.0F, 0.2F, 0.2F, 0.0F));
    detections.push_back(MakeDetection(10.4F, 1.0F, 0.2F, 0.2F, 0.0F));

    ConsolidateCurrentDetections(detections);

    EXPECT_EQ(detections.size(), 2U);
}

TEST(TrackingAssociation, PreservesIdAcrossLargeBodyFrameShiftUsingMapCenter) {
    st_Tracking.s32_NewTrackId = 100;
    st_Tracking.b_UseMapAssociation = true;
    st_Tracking.f32_COST_W_DIST = 1.0F;
    st_Tracking.f32_COST_W_IOU = 1.0F;
    st_Tracking.f32_COST_THRESHOLD = 3.0F;
    st_Tracking.f32_COST_INF = 9999.0F;
    st_Tracking.f32_ContinuityBonus = 0.20F;

    auto previous = MakeDetection(10.0F, 0.0F, 1.0F, 0.8F, 0.0F);
    previous.s32_Id = 42;
    previous.s32_Age = 20;
    previous.b_KfInitialized = true;
    previous.s32_KfInitCount = 2;
    previous.Eigen_KfState_CA << 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F;
    previous.f32_CoreE = 100.0F;
    previous.f32_CoreN = 200.0F;

    auto current = MakeDetection(5.0F, 0.0F, 1.0F, 0.8F, 0.0F);
    current.s32_Id = -1;
    current.f32_CoreE = 100.2F;
    current.f32_CoreN = 200.0F;

    std::deque<std::vector<LiDARObjectInfo>> history;
    history.push_back({previous});
    std::vector<LiDARObjectInfo> detections{current};
    GPS_IMU pose{};

    CompareCentroids(history, detections, 0.1, pose);

    ASSERT_FALSE(detections.empty());
    EXPECT_EQ(detections.front().s32_Id, 42);
    EXPECT_EQ(detections.front().s32_Age, 21);
}

TEST(TrackingAssociation, RecoversFromLegacyGroundNoiseOnCurrentObservation) {
    st_Tracking.s32_NewTrackId = 100;
    st_Tracking.b_UseMapAssociation = true;
    st_Tracking.f32_COST_W_DIST = 1.0F;
    st_Tracking.f32_COST_W_IOU = 1.0F;
    st_Tracking.f32_COST_THRESHOLD = 3.0F;
    st_Tracking.f32_COST_INF = 9999.0F;
    st_Tracking.f32_ContinuityBonus = 0.20F;

    auto previous = MakeDetection(10.0F, 0.0F, 1.0F, 0.8F, -1.4F);
    previous.s32_Id = 42;
    previous.s32_Age = 10;
    previous.b_KfInitialized = true;
    previous.s32_KfInitCount = 2;
    previous.Eigen_KfState_CA << 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F;
    previous.f32_CoreE = 100.0F;
    previous.f32_CoreN = 200.0F;
    previous.str_ObjectClass = "지면노이즈";

    auto current = MakeDetection(9.9F, 0.0F, 1.0F, 0.8F, -1.1F);
    current.s32_Id = -1;
    current.f32_CoreE = 100.1F;
    current.f32_CoreN = 200.0F;
    current.str_ObjectClass = "unknown";

    std::deque<std::vector<LiDARObjectInfo>> history;
    history.push_back({previous});
    std::vector<LiDARObjectInfo> detections{current};
    GPS_IMU pose{};

    CompareCentroids(history, detections, 0.1, pose);

    ASSERT_FALSE(detections.empty());
    EXPECT_EQ(detections.front().s32_Id, 42);
    EXPECT_EQ(detections.front().str_ObjectClass, "unknown");
}

TEST(TrackingAssociation, OneNoiseMeasurementDoesNotDowngradeValidTrack) {
    st_Tracking.s32_NewTrackId = 100;
    st_Tracking.b_UseMapAssociation = true;
    st_Tracking.f32_COST_W_DIST = 1.0F;
    st_Tracking.f32_COST_W_IOU = 1.0F;
    st_Tracking.f32_COST_THRESHOLD = 3.0F;
    st_Tracking.f32_COST_INF = 9999.0F;
    st_Tracking.f32_ContinuityBonus = 0.20F;

    auto previous = MakeDetection(10.0F, 0.0F, 1.0F, 0.8F, -1.1F);
    previous.s32_Id = 42;
    previous.s32_Age = 10;
    previous.b_KfInitialized = true;
    previous.s32_KfInitCount = 2;
    previous.Eigen_KfState_CA << 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F;
    previous.f32_CoreE = 100.0F;
    previous.f32_CoreN = 200.0F;
    previous.str_ObjectClass = "unknown";

    auto current = MakeDetection(9.9F, 0.0F, 1.0F, 0.8F, -1.4F);
    current.s32_Id = -1;
    current.f32_CoreE = 100.1F;
    current.f32_CoreN = 200.0F;
    current.str_ObjectClass = "지면노이즈";

    std::deque<std::vector<LiDARObjectInfo>> history;
    history.push_back({previous});
    std::vector<LiDARObjectInfo> detections{current};
    GPS_IMU pose{};

    CompareCentroids(history, detections, 0.1, pose);

    ASSERT_FALSE(detections.empty());
    EXPECT_EQ(detections.front().s32_Id, 42);
    EXPECT_EQ(detections.front().str_ObjectClass, "unknown");
}

TEST(TrackingAssociation, SparseLowObstacleKeepsNonDegenerateUnknownObb) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr centroids(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices indices;
    for (int index = 0; index < 6; ++index) {
        pcl::PointXYZI point;
        point.x = 20.0F;
        point.y = -0.5F + 0.2F * static_cast<float>(index);
        point.z = -1.42F;
        indices.indices.push_back(index);
        non_ground->push_back(point);
    }
    pcl::PointXYZI centroid;
    centroid.x = 20.0F;
    centroid.y = 0.0F;
    centroid.z = -1.42F;
    centroids->push_back(centroid);

    std::vector<LiDARObjectInfo> detections;
    std::vector<pcl::PointIndices> clusters{indices};
    GPS_IMU pose{};
    VehicleSpec vehicle{};

    Objectinfo_for_km(
        detections, non_ground, centroids, clusters, pose, vehicle);

    ASSERT_EQ(detections.size(), 1U);
    EXPECT_EQ(detections.front().str_ObjectClass, "unknown");
    EXPECT_GE(detections.front().f32_OBB_Width, 0.05F);
    EXPECT_GE(detections.front().f32_OBB_Height, 0.05F);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
