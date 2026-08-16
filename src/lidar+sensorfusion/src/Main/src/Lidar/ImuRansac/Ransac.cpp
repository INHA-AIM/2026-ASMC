// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/common/common.h>

// #include <cmath>
// #include <vector>

#include "ImuRansac.hpp"
extern VehicleSpec st_VehicleSpec;
extern Planning st_Planning;


// 법선 벡터 내적 필터링ac_cloud,
void NormalVectorFiltering(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud3, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud4,const GPS_IMU& st_GPSIMU) {
    if (!pcl_Cloud3 || pcl_Cloud3->points.size() < 3) {
        pcl_Cloud4->clear();
        return;
    }

    pcl::ModelCoefficients::Ptr pcl_Coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr pcl_Inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr pcl_Wall_inliers(new pcl::PointIndices());  // 벽면 점을 추적하는 변수
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud3_4(new pcl::PointCloud<pcl::PointXYZI>(*pcl_Cloud3));
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud4_4(new pcl::PointCloud<pcl::PointXYZI>);
    int32 s32_Max_iterations = 3; // 벽이 먼저 잡혀도 지면을 다시 찾을 수 있게 반복
    int32 s32_Iteration = 0;
    bool ground_removed = false;
    

    // RANSAC 세그멘테이션 반복//이거
    while (s32_Iteration++ < s32_Max_iterations) {
        // RANSAC 세그멘테이션 초기화
        pcl::SACSegmentation<pcl::PointXYZI> pcl_Seg;
        pcl_Seg.setOptimizeCoefficients(true);
        pcl_Seg.setModelType(pcl::SACMODEL_PLANE);
        pcl_Seg.setMethodType(pcl::SAC_RANSAC);
        pcl_Seg.setMaxIterations(20);
        pcl_Seg.setDistanceThreshold(0.3);  // 허용 거리 20cm
        pcl_Seg.setInputCloud(pcl_Cloud3_4);
        pcl_Seg.segment(*pcl_Inliers, *pcl_Coefficients);

        if (pcl_Inliers->indices.empty()) {
            break;
        }

        // 법선 벡터를 기준으로 지면인지 벽면인지 판별
        if (!pcl_Inliers->indices.empty()) { // Z축에 가까운 평면(지면)
            pcl::PointXYZI pcl_Normal_vector;
            pcl_Normal_vector.x = pcl_Coefficients->values[0];
            pcl_Normal_vector.y = pcl_Coefficients->values[1];
            pcl_Normal_vector.z = pcl_Coefficients->values[2];
            pcl_Normal_vector.intensity = 0.0f;


            // RANSAC 계수를 명시적으로 정규화한 뒤 Z축과 비교한다.
            // 기존의 abs(dot) > 1 조건은 정규화된 법선에서는 절대로
            // 성립하지 않아 정상 도로 평면까지 벽으로 처리하고 있었다.
            const float32 normal_norm = std::sqrt(
                pcl_Normal_vector.x * pcl_Normal_vector.x +
                pcl_Normal_vector.y * pcl_Normal_vector.y +
                pcl_Normal_vector.z * pcl_Normal_vector.z);
            const float32 f32_Dot_product = normal_norm > 1e-6f
                ? pcl_Normal_vector.z / normal_norm
                : 0.0f;

            // Z축에서 약 35도 이내인 평면을 지면으로 인정한다.
            if (std::fabs(f32_Dot_product) >= 0.82f) {

                // 지면을 제거: pcl_Cloud3에서 해당 pcl_Inliers 점들을 제거
                pcl::ExtractIndices<pcl::PointXYZI> pcl_Extract;
                pcl_Extract.setInputCloud(pcl_Cloud3_4);
                pcl_Extract.setIndices(pcl_Inliers);
                pcl_Extract.setNegative(true); // true로 설정하면 pcl_Inliers 외 점들만 남김
                pcl_Extract.filter(*pcl_Cloud3_4);
                for (const auto& pt : pcl_Cloud3_4->points) {
                    if (std::isfinite(pt.x) && std::isfinite(pt.y) &&
                        std::isfinite(pt.z)) {
                        pcl_Cloud4_4->points.push_back(pt);
                    }
                }

                //width/height 메타데이터 갱신
                pcl_Cloud4_4->width  = static_cast<uint32_t>(pcl_Cloud4_4->points.size());
                pcl_Cloud4_4->height = 1;   // unorganized
                pcl_Cloud4_4->is_dense = false;
                // pcl_cloud4에 남은 점들을 저장
                *pcl_Cloud4 = *pcl_Cloud4_4;
                ground_removed = true;
                break; // 지면을 찾았으므로 반복 종료
            }

            else { // 벽면을 찾았을 때

                // 벽면을 제외한 점들을 wall_pcl_Inliers에 저장
                *pcl_Wall_inliers = *pcl_Inliers;

                // 벽면 점들은 다시 RANSAC에서 제외
                pcl::ExtractIndices<pcl::PointXYZI> pcl_ExtractWall;
                pcl_ExtractWall.setInputCloud(pcl_Cloud3_4);
                pcl_ExtractWall.setIndices(pcl_Wall_inliers);
                pcl_ExtractWall.setNegative(true); // 벽면을 제외
                pcl_ExtractWall.filter(*pcl_Cloud3_4);

                continue; // 벽면을 제외하고 계속 진행
            }
        }
    }

    // 지면 모델을 찾지 못했다고 센서 전체를 빈 값으로 만들면 후단
    // 클러스터링이 완전히 멈춘다. 이 경우 남아 있는 유효점을 전달한다.
    if (!ground_removed) {
        *pcl_Cloud4 = *pcl_Cloud3_4;
        ROS_WARN_THROTTLE(2.0,
            "Ground plane not found; forwarding non-ground-filtered LiDAR points");
    }

    pcl_Cloud4->width = static_cast<uint32_t>(pcl_Cloud4->points.size());
    pcl_Cloud4->height = 1;
    pcl_Cloud4->is_dense = false;
}
