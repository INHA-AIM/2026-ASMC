#ifndef LIDAR_PIPELINE_PROFILE_HPP
#define LIDAR_PIPELINE_PROFILE_HPP

#include <chrono>

// ROI 크기에 따른 전처리 비용을 단계별로 분리해 보기 위한 계측 구조체다.
// 벤치마크 모드에서만 읽히며, 평시에는 단순 double 대입 비용만 남는다.
struct LidarStageProfile {
    double make_cloud_ms = 0.0;
    double roi_ms = 0.0;
    double roi_pub_ms = 0.0;
    double voxel_ms = 0.0;
    double imu_correct_ms = 0.0;
    double ransac_ms = 0.0;
    double ransac_pub_ms = 0.0;
    double cluster_ms = 0.0;
    double cluster_pub_ms = 0.0;
    double map_cluster_pub_ms = 0.0;
    double objectinfo_ms = 0.0;
    double framefilter_ms = 0.0;
    double motion_ms = 0.0;
    double marker_ms = 0.0;

    int raw_points = 0;
    int roi_points = 0;
    int voxel_points = 0;
    int ransac_points = 0;
    int cluster_points = 0;
    int clusters = 0;

    bool valid = false;

    void Reset() { *this = LidarStageProfile(); }
};

extern LidarStageProfile g_LidarStageProfile;

#define LIDAR_STAGE(field, ...)                                            \
    do {                                                                   \
        const auto lidar_stage_begin = std::chrono::steady_clock::now();   \
        __VA_ARGS__;                                                       \
        g_LidarStageProfile.field +=                                       \
            std::chrono::duration<double, std::milli>(                     \
                std::chrono::steady_clock::now() - lidar_stage_begin)      \
                .count();                                                  \
    } while (0)

#endif  // LIDAR_PIPELINE_PROFILE_HPP
