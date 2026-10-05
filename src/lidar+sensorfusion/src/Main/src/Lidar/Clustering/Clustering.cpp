
#include "Clustering.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

extern Planning st_Planning;
extern VehicleSpec st_VehicleSpec;

namespace {

constexpr float kDefaultClusterTolerance = 0.5F;
constexpr int kMaximumClusterPoints = 10000;

float PositiveOr(float value, float fallback) {
    return std::isfinite(value) && value > 0.0F ? value : fallback;
}

std::vector<pcl::PointIndices> ExtractAdaptiveClusters(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const pcl::search::KdTree<pcl::PointXYZI>::Ptr& tree,
    const LiDARClustering& config,
    int min_points) {
    std::vector<pcl::PointIndices> clusters;
    if (!cloud || cloud->empty()) return clusters;

    std::vector<std::uint8_t> processed(cloud->size(), 0U);
    std::vector<int> queue;
    std::vector<int> neighbors;
    std::vector<float> squared_distances;
    queue.reserve(cloud->size());

    for (std::size_t seed = 0; seed < cloud->size(); ++seed) {
        if (processed[seed] != 0U) continue;
        const auto& seed_point = cloud->points[seed];
        if (!std::isfinite(seed_point.x) || !std::isfinite(seed_point.y) ||
            !std::isfinite(seed_point.z)) {
            processed[seed] = 1U;
            continue;
        }

        queue.clear();
        queue.push_back(static_cast<int>(seed));
        processed[seed] = 1U;

        for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
            const int point_index = queue[cursor];
            const auto& point = cloud->points[point_index];
            const float point_range = std::hypot(point.x, point.y);
            const float point_tolerance =
                AdaptiveClusterTolerance(point_range, config);

            neighbors.clear();
            squared_distances.clear();
            if (tree->radiusSearch(
                    point_index, point_tolerance,
                    neighbors, squared_distances) <= 0) {
                continue;
            }
            for (std::size_t n = 0; n < neighbors.size(); ++n) {
                const int neighbor_index = neighbors[n];
                if (neighbor_index < 0 ||
                    static_cast<std::size_t>(neighbor_index) >= cloud->size() ||
                    processed[neighbor_index] != 0U) {
                    continue;
                }
                const auto& neighbor = cloud->points[neighbor_index];
                const float neighbor_tolerance = AdaptiveClusterTolerance(
                    std::hypot(neighbor.x, neighbor.y), config);
                const float symmetric_tolerance =
                    std::min(point_tolerance, neighbor_tolerance);
                if (squared_distances[n] >
                    symmetric_tolerance * symmetric_tolerance) {
                    continue;
                }
                processed[neighbor_index] = 1U;
                queue.push_back(neighbor_index);
            }
        }

        if (static_cast<int>(queue.size()) < min_points ||
            static_cast<int>(queue.size()) > kMaximumClusterPoints) {
            continue;
        }
        std::sort(queue.begin(), queue.end());
        pcl::PointIndices indices;
        indices.indices = queue;
        clusters.push_back(std::move(indices));
    }
    return clusters;
}

}  // namespace

float AdaptiveClusterTolerance(
    float horizontal_range_m,
    const LiDARClustering& config) {
    const float fixed = PositiveOr(config.f32_Eps, kDefaultClusterTolerance);
    if (!config.b_Adaptive) return fixed;

    const float near_range = std::max(
        0.0F, PositiveOr(config.f32_AdaptiveNearRange, 8.0F));
    const float far_range = std::max(
        near_range + 1.0F,
        PositiveOr(config.f32_AdaptiveFarRange, 50.0F));
    const float near_tolerance = PositiveOr(
        config.f32_AdaptiveNearEps, fixed);
    const float far_tolerance = std::max(
        near_tolerance,
        PositiveOr(config.f32_AdaptiveFarEps, near_tolerance));
    const float range = std::isfinite(horizontal_range_m)
        ? std::max(0.0F, horizontal_range_m) : near_range;
    const float alpha = std::clamp(
        (range - near_range) / (far_range - near_range), 0.0F, 1.0F);
    return near_tolerance + alpha * (far_tolerance - near_tolerance);
}

//클러스터링
void Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud4, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud5,
vector<pcl::PointIndices> &vp_AllClusterIndices,const GPS_IMU& st_GPSIMU,
const LiDARClustering& config){
    if (pcl_Cloud4->empty()) {
      
      // 빈 결과 하나를 보장
      pcl_Cloud4->points.clear();
      pcl::PointXYZI pcl_EmptyPoint;
      pcl_EmptyPoint.x = 0.0; pcl_EmptyPoint.y = 0.0; pcl_EmptyPoint.z = 0.0;
      pcl_EmptyPoint.intensity = 0.0;
      pcl_Cloud4->points.push_back(pcl_EmptyPoint);
      vp_AllClusterIndices.clear();
      return;
    }
    pcl::search::KdTree<pcl::PointXYZI>::Ptr pcl_Tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl_Tree->setInputCloud (pcl_Cloud4);  //KdTree 생성
    vector<pcl::PointIndices> vp_ClusterIndices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장
    // setting.yaml에서 읽은 값을 실제 군집화에도 사용한다. 원거리의 성긴
    // 점군에는 조금 큰 반경을, 근거리에는 작은 반경을 적용한다. 하나의
    // 고정 반경을 크게 키워 가드레일 전체를 합치는 방식은 사용하지 않는다.
    const float tolerance =
        std::isfinite(config.f32_Eps) && config.f32_Eps > 0.0F
            ? config.f32_Eps : kDefaultClusterTolerance;
    const int min_points = std::max(1, config.s32_MinPoints);
    if (config.b_Adaptive) {
        vp_ClusterIndices = ExtractAdaptiveClusters(
            pcl_Cloud4, pcl_Tree, config, min_points);
    } else {
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> pcl_Ec;
        pcl_Ec.setInputCloud (pcl_Cloud4);
        pcl_Ec.setClusterTolerance (tolerance);
        pcl_Ec.setMinClusterSize (min_points);
        pcl_Ec.setMaxClusterSize (kMaximumClusterPoints);
        pcl_Ec.setSearchMethod (pcl_Tree);
        pcl_Ec.extract (vp_ClusterIndices);
    }

    // [LOG C1] 클러스터 요약 (프레임당 1줄)
    int min_pts = INT_MAX, max_pts = 0;
    for (const auto& ci : vp_ClusterIndices) {
        int n = (int)ci.indices.size();
        min_pts = std::min(min_pts, n);
        max_pts = std::max(max_pts, n);
    }
    // std::cout << "[CLUSTER] num=" << vp_ClusterIndices.size()
    //           << " pts_min=" << (vp_ClusterIndices.empty()?0:min_pts)
    //           << " pts_max=" << (vp_ClusterIndices.empty()?0:max_pts) << "\n";

    vp_AllClusterIndices=vp_ClusterIndices;
    if (vp_ClusterIndices.size()==0) {
            //예시: 기본적인 빈 포인트 추가
            pcl::PointXYZI pcl_EmptyPoint;
            pcl_EmptyPoint.x = 0.0;
            pcl_EmptyPoint.y = 0.0;
            pcl_EmptyPoint.z = 0.0;
            pcl_EmptyPoint.intensity = 0.0;
            pcl_Cloud4->points.push_back(pcl_EmptyPoint);
            return;
    }

    vector<pcl::PointIndices> vp_FilteredClusterIndices;

    for (vector<pcl::PointIndices>::iterator vp_ClusterIt = vp_ClusterIndices.begin(); vp_ClusterIt != vp_ClusterIndices.end(); ++vp_ClusterIt) {
        pcl::PointXYZI pcl_Centroid;
        pcl_Centroid.x = 0.0;
        pcl_Centroid.y = 0.0;
        pcl_Centroid.z = 0.0;
        pcl_Centroid.intensity = 0.0f;

        // 클러스터 내 모든 포인트의 평균 계산
        for (vector<int>::iterator point_it = vp_ClusterIt->indices.begin(); point_it != vp_ClusterIt->indices.end(); ++point_it) {
            pcl_Centroid.x += pcl_Cloud4->points[*point_it].x;
            pcl_Centroid.y += pcl_Cloud4->points[*point_it].y;
            pcl_Centroid.z += pcl_Cloud4->points[*point_it].z;
            
        }
        pcl_Centroid.x /= vp_ClusterIt->indices.size();
        pcl_Centroid.y /= vp_ClusterIt->indices.size();
        pcl_Centroid.z /= vp_ClusterIt->indices.size();
        // if(pcl_Centroid.z<0.0){
        //     cout<<"z값 내놔 미친"<<pcl_Centroid.z<<endl;
        //     // continue;
        // }// 풀 같은 지면제거 후에 남은 노이즈 제거하기 위한 조건문
        // 코어 포인트를 최종 클러스터 점군에 추가 // 현재 병합과정으로 인해 삭제함
        // Object ob;
        // ob.f32_CoreX=pcl_Centroid.x;
        // ob.f32_CoreY=pcl_Centroid.y;
        // Body2ENU(st_VehicleSpec,st_GPSIMU,ob);
        // if(InOrOut(st_Planning, ob.f32_CoreE, ob.f32_CoreN)==false){
        //     continue;
        // }
        pcl_Cloud5->points.push_back(pcl_Centroid);
        vp_FilteredClusterIndices.push_back(*vp_ClusterIt);
    }

    vp_AllClusterIndices = vp_FilteredClusterIndices;

    // if (pcl_Cloud4->points.size()==0) {
    //         //예시: 기본적인 빈 포인트 추가
    //         pcl::PointXYZI pcl_EmptyPoint;
    //         pcl_EmptyPoint.x = 0.0;
    //         pcl_EmptyPoint.y = 0.0;
    //         pcl_EmptyPoint.z = 0.0;
    //         pcl_EmptyPoint.intensity = 0.0;
    //         pcl_Cloud4->points.push_back(pcl_EmptyPoint);
    //         return;
    // }

    // 결과를 정리cmka
    pcl_Cloud5->width = pcl_Cloud5->points.size();
    pcl_Cloud5->height = 1;
    pcl_Cloud5->is_dense = true; 
}
/*
//군집끼리 병합시키는 함수 지금은 안쓰는중,,,만약 쓰게 되면 렌삭 파일에서 일정 z값이하 continue하는 부분 제거해야함
void merge(vector<objif> &ob , pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud4,int d){
    const float DISTANCE = 4;
    bool merged;
        while(merged){
            merged=false;
            for(vector<objif>::iterator it=ob.begin();it != ob.end() ; ++it){
                for(vector<objif>::iterator it2=it+1;it2 != ob.end();){
                    bool shouldMerge=false;
                    float dx=(*it).core_x - (*it2).core_x;
                    float dy=(*it).core_y - (*it2).core_y;
                    float dist=sqrt(dx*dx+dy*dy);
                    if(dist<= DISTANCE){
                        const vector<int> &indices1 =(*it).objPoints->indices;
                        const vector<int> &indices2 =(*it2).objPoints->indices;
                        const vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> &points = pcl_Cloud4->points;
                        for(int i : indices1){
                            const pcl::PointXYZI &PP=points[i];
                            for(int i2 : indices2){
                                const pcl::PointXYZI &PP2=points[i2];
                                float pdx = PP.x - PP2.x;
                                float pdy = PP.y - PP2.y;
                                if ((pdx * pdx + pdy * pdy) <= (d * d)) {
                                    shouldMerge = true;
                                    break;
                                }
                            }
                            if(shouldMerge) break;
                        }
                    }
                    if(shouldMerge)
                    {
                        it->xMin = min(it->xMin, it2->xMin);
                        it->yMin = min(it->yMin, it2->yMin);
                        it->zMin = min(it->zMin, it2->zMin);
                        it->xMax = max(it->xMax, it2->xMax);
                        it->yMax = max(it->yMax, it2->yMax);
                        it->zMax = max(it->zMax, it2->zMax);
                        int totalPoints = it->objPoints->indices.size() +it2->objPoints->indices.size();
                        float sumX = 0, sumY = 0, sumZ = 0;
                        for (int i : it->objPoints->indices) {
                            sumX += pcl_Cloud4->points[i].x;
                            sumY += pcl_Cloud4->points[i].y;
                            sumZ += pcl_Cloud4->points[i].z;
                        }
                        for (int i : it2->objPoints->indices) {
                            sumX += pcl_Cloud4->points[i].x;
                            sumY += pcl_Cloud4->points[i].y;
                            sumZ += pcl_Cloud4->points[i].z;
                        }
                        it->core_x = sumX / totalPoints;
                        it->core_y = sumY / totalPoints;
                        it->core_z = sumZ / totalPoints;

                        // 포인트 인덱스 병합
                        (*it).objPoints->indices.insert((*it).objPoints->indices.end(), 
                                                     (*it2).objPoints->indices.begin(), 
                                                     (*it2).objPoints->indices.end());

                        it2 = ob.erase(it2); // 벡터에서 삭제 후 다음 클러스터로
                        merged = true; // 병합이 일어났음을 표시
                        break;
                    } else{
                        ++it2;
                    }
                }
                if (merged) break;
            }
            
        }
        //cout<<"completed"<<endl;
    
    
}*/
