
#include "Clustering.hpp"

extern Planning st_Planning;
extern VehicleSpec st_VehicleSpec;

//클러스터링
void Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud4, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud5,
vector<pcl::PointIndices> &vp_AllClusterIndices,const GPS_IMU& st_GPSIMU){
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
    // 군집화 오브젝트 생성
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> pcl_Ec;
    pcl_Ec.setInputCloud (pcl_Cloud4);       // 입력
    pcl_Ec.setClusterTolerance (1.5);  // 100cm
    pcl_Ec.setMinClusterSize (10);     // 최소 포인트 수 / 풀숲에 영향을 줌
    pcl_Ec.setMaxClusterSize (10000);   // 최대 포인트 수
    pcl_Ec.setSearchMethod (pcl_Tree);      // 위에서 정의한 탐색 방법 지정
    pcl_Ec.extract (vp_ClusterIndices);   // 군집화 적용

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


