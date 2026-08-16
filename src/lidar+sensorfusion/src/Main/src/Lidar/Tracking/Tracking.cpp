#include <Lidar/Tracking/tracking_utils.hpp>
#include <Lidar/Kalman/kalman_utils.hpp>
#include <iomanip>
const double DISP_SPIKE = 0.60;  // 프레임 간 위치 점프(m) 스파이크 기준
const double V_SPIKE    = 6.00;  // 속도 스파이크(m/s) 기준(≈ 21.6 km/h)
#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <cstdlib> // std::rand(), std::srand()
#include <ctime> 
//노렌삭

float32 calculateEmaSpeed(float32 previousEmaSpeed, float32 rawSpeed, float32 alpha) {
    // EMA 공식: (1 - alpha) * 이전 EMA 속도 + alpha * 현재 raw 속도
    return (1.0 - alpha) * previousEmaSpeed + alpha * rawSpeed;
} // 급회전할때 적용할 것 


namespace obb_util {

struct P2 { double x, y; };
struct P3 { double x, y, z; };

inline double cross(const P2& o, const P2& a, const P2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}


// Monotone chain convex hull (CCW로 반환, 닫히지 않은 형태)
inline std::vector<P2> convexHull(std::vector<P2> pts) {
    std::sort(pts.begin(), pts.end(), [](const P2& a, const P2& b){
        return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
    });
    pts.erase(std::unique(pts.begin(), pts.end(), [](const P2& a, const P2& b){
        return a.x == b.x && a.y == b.y;
    }), pts.end());

    if (pts.size() <= 1) return pts;

    std::vector<P2> lower, upper;
    for (const auto& p : pts) {
        while (lower.size() >= 2 && cross(lower[lower.size()-2], lower.back(), p) <= 0) lower.pop_back();
        lower.push_back(p);
    }
    for (int i = (int)pts.size()-1; i >= 0; --i) {
        const auto& p = pts[i];
        while (upper.size() >= 2 && cross(upper[upper.size()-2], upper.back(), p) <= 0) upper.pop_back();
        upper.push_back(p);
    }
    lower.pop_back(); upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower; // hull (CCW)
}

// -θ 회전
inline P2 rotateMinus(const P2& p, double theta) {
    const double c = std::cos(theta), s = std::sin(theta);
    return { p.x * c + p.y * s, -p.x * s + p.y * c };
}

// +θ 회전
inline P2 rotatePlus(const P2& p, double theta) {
    const double c = std::cos(theta), s = std::sin(theta);
    return { p.x * c - p.y * s, p.x * s + p.y * c };
}

inline double distSq(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2) {
    return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
}

struct OBB2D {
    double cx{0}, cy{0};     // 중심 (XY)
    double w{0}, h{0};       // 가로/세로 (XY)
    double yaw{0};           // 라디안, +x 기준 CCW
    std::array<P2,4> corners; // z 미포함, 시계/반시계 일관 유지
};


inline OBB2D minAreaRectFromHull(const std::vector<P2>& hull) {
    OBB2D out;
    if (hull.empty()) return out;

    if (hull.size() == 1) {
        out.cx = hull[0].x;
        out.cy = hull[0].y;
        out.w = 0;
        out.h = 0;
        out.yaw = 0;
        out.corners = {hull[0], hull[0], hull[0], hull[0]};
        return out;
    }
    if (hull.size() == 2) {
        const auto& a = hull[0];
        const auto& b = hull[1];
        out.cx = 0.5 * (a.x + b.x);
        out.cy = 0.5 * (a.y + b.y);
        out.w = std::hypot(b.x - a.x, b.y - a.y);
        out.h = 0.0;
        out.yaw = std::atan2(b.y - a.y, b.x - a.x);
        out.corners = {a, b, b, a};
        return out;
    }

    double best_area = std::numeric_limits<double>::infinity();
    double best_angle = 0.0;
    std::array<P2, 4> best_rect_corners{};
    double best_w = 0.0, best_h = 0.0;
    P2 best_center{0, 0};

    const int N = (int)hull.size();
    for (int i = 0; i < N; ++i) {
        const P2& p1 = hull[i];
        const P2& p2 = hull[(i + 1) % N];
        const double theta = std::atan2(p2.y - p1.y, p2.x - p1.x);

        double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18;
        for (const auto& p : hull) {
            P2 r = rotateMinus(p, theta);
            minx = std::min(minx, r.x);
            maxx = std::max(maxx, r.x);
            miny = std::min(miny, r.y);
            maxy = std::max(maxy, r.y);
        }

        const double w = maxx - minx;
        const double h = maxy - miny;
        const double area = w * h;

        if (area < best_area) {
            best_area = area;
            best_angle = theta;
            best_w = w;
            best_h = h;
            best_center = P2{(minx + maxx) / 2.0, (miny + maxy) / 2.0};

            std::array<P2, 4> rect_r = {
                P2{minx, miny}, P2{maxx, miny}, P2{maxx, maxy}, P2{minx, maxy}
            };
            for (int k = 0; k < 4; ++k) {
                best_rect_corners[k] = rotatePlus(rect_r[k], theta);
            }
        }
    }

    // 최종 OBB 구조체에 값 할당
    out.cx = best_center.x;
    out.cy = best_center.y;
    out.w = best_w;
    out.h = best_h;
    out.yaw = best_angle;
    out.corners = best_rect_corners;

    return out;
}

inline double getYawFromRANSAC(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (cloud.size() < 3) {
        return 0.0; // 점이 부족하면 0.0을 반환
    }

    // 1. 2D 점군으로 변환
    std::vector<P2> pts2d;
    pts2d.reserve(cloud.size());
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
        pts2d.push_back({p.x, p.y});
    }

    if (pts2d.empty()) {
        return 0.0;
    }

    // 2. RANSAC 직선 피팅
    const int max_iterations = 500;
    const double threshold = 0.03; // 미터 단위 오차 허용
    int best_inlier_count = -1;
    double best_yaw = 0.0;

    // std::srand(std::time(0)); // 프로그램 시작 시 한 번만 호출
    for (int i = 0; i < max_iterations; ++i) {
        int idx1 = std::rand() % pts2d.size();
        int idx2 = std::rand() % pts2d.size();
        while (idx1 == idx2) {
            idx2 = std::rand() % pts2d.size();
        }

        P2 p1 = pts2d[idx1];
        P2 p2 = pts2d[idx2];
        
        // 두 점을 지나는 직선의 방정식 A*x + B*y + C = 0
        double A = p2.y - p1.y;
        double B = p1.x - p2.x;
        double C = -A * p1.x - B * p1.y;
        double inv_norm = 1.0 / std::sqrt(A*A + B*B);

        // Inlier 수 계산
        int current_inlier_count = 0;
        for (const auto& p : pts2d) {
            double distance = std::abs(A * p.x + B * p.y + C) * inv_norm;
            if (distance < threshold) {
                current_inlier_count++;
            }
        }
        
        if (current_inlier_count > best_inlier_count) {
            best_inlier_count = current_inlier_count;
            best_yaw = std::atan2(A, -B);
        }
    }
    
    return best_yaw;
}

inline void computeOBB3D_PCA(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    OBB2D& obb2d,
    double& zMin, double& zMax,
    std::array<P3,8>& corners3d
){
    zMin = std::numeric_limits<double>::infinity();
    zMax = -std::numeric_limits<double>::infinity();

    std::vector<P2> pts2d;
    pts2d.reserve(cloud.size());

    double mean_x = 0, mean_y = 0;
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        pts2d.push_back({p.x, p.y});
        mean_x += p.x;
        mean_y += p.y;
        zMin = std::min(zMin, (double)p.z);
        zMax = std::max(zMax, (double)p.z);
    }

    if (pts2d.empty()) {
        obb2d = {}; zMin = 0; zMax = 0; corners3d.fill({0,0,0});
        return;
    }

    mean_x /= pts2d.size();
    mean_y /= pts2d.size();

    // PCA - 주성분 벡터 구하기
    Eigen::MatrixXd data(pts2d.size(), 2);
    for (size_t i = 0; i < pts2d.size(); ++i) {
        data(i, 0) = pts2d[i].x - mean_x;
        data(i, 1) = pts2d[i].y - mean_y;
    }

    Eigen::MatrixXd cov = (data.transpose() * data) / pts2d.size();
    Eigen::EigenSolver<Eigen::MatrixXd> solver(cov);
    Eigen::VectorXd eigenvalues = solver.eigenvalues().real();
    Eigen::MatrixXd eigenvectors = solver.eigenvectors().real();

    // 주성분 벡터 (x, y)
    Eigen::Vector2d axis1(eigenvectors(0,0), eigenvectors(1,0));
    Eigen::Vector2d axis2(eigenvectors(0,1), eigenvectors(1,1));

    // 회전 각도 (yaw)
    // double yaw = atan2(axis1.y(), axis1.x());
    double yaw = getYawFromRANSAC(cloud);

    // 회전
    double c = cos(-yaw), s = sin(-yaw);
    std::vector<P2> rotated;
    rotated.reserve(pts2d.size());

    for (auto& p : pts2d) {
        rotated.push_back({p.x * c - p.y * s, p.x * s + p.y * c});
    }

    // Convex Hull을 사용하여 회전된 점들의 경계를 구합니다.
    auto hull = convexHull(rotated);

    double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
    for (auto& p : hull) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
    }

    // OBB 계산
    obb2d.cx = (minx + maxx) / 2.0;
    obb2d.cy = (miny + maxy) / 2.0;
    obb2d.w = maxx - minx;
    obb2d.h = maxy - miny;
    obb2d.yaw = yaw;

    // 꼭짓점 계산 (회전된 좌표계에서 원래 좌표계로 변환)
    std::array<P2,4> rect = {
        P2{minx, miny}, P2{maxx, miny}, P2{maxx, maxy}, P2{minx, maxy}
    };

    for (int k = 0; k < 4; ++k) {
        double X = rect[k].x * cos(yaw) - rect[k].y * sin(yaw);
        double Y = rect[k].x * sin(yaw) + rect[k].y * cos(yaw);
        obb2d.corners[k] = {X, Y};
    }

    // 3D 코너 8개
    for (int k = 0; k < 4; ++k) {
        corners3d[k] = {obb2d.corners[k].x, obb2d.corners[k].y, zMin};
        corners3d[k+4] = {obb2d.corners[k].x, obb2d.corners[k].y, zMax};
    }
}



} // namespace obb_util




//렌삭기법 


void Objectinfo_for_km(vector<LiDARObjectInfo> &vec_NowObject,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_Cloud4,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_Cloud5,
                        vector<pcl::PointIndices> vec_AllClusterIndices, const GPS_IMU& st_GPSIMU, VehicleSpec& st_VehicleSpec)
{
    
    vec_NowObject.clear();
    int32 s32_Centroid_index = 0;
    
    for (int32 i = 0; i < vec_AllClusterIndices.size(); i++) {
        pcl::PointIndices cluster = vec_AllClusterIndices[i];

        float32 f32_MinX = std::numeric_limits<float32>::max();
        float32 f32_MinY = std::numeric_limits<float32>::max();
        float32 f32_MinZ = std::numeric_limits<float32>::max();
        float32 f32_MaxX = -std::numeric_limits<float32>::max();
        float32 f32_MaxY = -std::numeric_limits<float32>::max();
        float32 f32_MaxZ = -std::numeric_limits<float32>::max();


        LiDARObjectInfo st_CurrentObj;

        st_CurrentObj.u16_Idx = i;
        st_CurrentObj.pcl_ClusterPoints.points.reserve(cluster.indices.size());   //재할당 제거 복사 비용 감소용

        // for (int32 j = 0; j < cluster.indices.size(); j++) {
        //     int32 idx = cluster.indices[j];
        //     const pcl::PointXYZI &pt = pcl_Cloud4->points[idx];
        //     st_CurrentObj.pcl_ClusterPoints.points.push_back(pt);
        //     f32_MinX = min(f32_MinX, pt.x);
        //     f32_MinY = min(f32_MinY, pt.y);
        //     f32_MinZ = min(f32_MinZ, pt.z);
        //     f32_MaxX = max(f32_MaxX, pt.x);
        //     f32_MaxY = max(f32_MaxY, pt.y);
        //     f32_MaxZ = max(f32_MaxZ, pt.z);
        // }
        

        for (int32 j = 0; j < cluster.indices.size(); j++) {
            int32 idx = cluster.indices[j];
            const pcl::PointXYZI &pt = pcl_Cloud4->points[idx];
            st_CurrentObj.pcl_ClusterPoints.points.push_back(pt);

            // Min X
            if (pt.x < f32_MinX) {
                f32_MinX = pt.x;
            }
            // Max X
            if (pt.x > f32_MaxX) {
                f32_MaxX = pt.x;
            }
            // Min Y
            if (pt.y < f32_MinY) {
                f32_MinY = pt.y;
            }
            // Max Y
            if (pt.y > f32_MaxY) {
                f32_MaxY = pt.y;
            }
            // Z는 단순 값만 필요하다면 그대로 유지
            f32_MinZ = min(f32_MinZ, pt.z);
            f32_MaxZ = max(f32_MaxZ, pt.z);
        }
        // cout<<pcl_Cloud5->points.size()<<endl;


        
        
        if(pcl_Cloud5->points.size()==0){
            break;
        }
        pcl::PointXYZI pcl_Centroid = pcl_Cloud5->points[s32_Centroid_index];
        s32_Centroid_index++;
        st_CurrentObj.objPoints = &vec_AllClusterIndices[i];
        st_CurrentObj.f32_CoreX = pcl_Centroid.x;
        st_CurrentObj.f32_CoreY = pcl_Centroid.y;
        st_CurrentObj.f32_CoreZ = pcl_Centroid.z;
        if(st_CurrentObj.f32_CoreZ<=-1.3){ //기존 라이다 일 경우 0.8 최근에 단 천장 라이다이면 1.3
            st_CurrentObj.str_ObjectClass = "지면노이즈";
            // cout<<"나왔다:"<<st_CurrentObj.str_ObjectClass<<endl;
        }
        //obb
        // obb_util::OBB2D obb2d;
        // double zMinD=0, zMaxD=0;
        // std::array<obb_util::P3,8> corners3d{};

        // // computeOBB3D_fromClusterXY 오버로드(참조 버전)를 쓰는 걸 추천
        // // (복사 없이 const 참조로 동작)
        // obb_util::computeOBB3D_PCA(
        //     st_CurrentObj.pcl_ClusterPoints,
        //     obb2d, zMinD, zMaxD, corners3d
        // );

        // obb_util::computeOBB3D_RotatingCalipersPercentile(
        //     st_CurrentObj.pcl_ClusterPoints,
        //     obb2d, zMinD, zMaxD, corners3d,
        //     0.01, 0.99   // 필요시 0.1, 0.9 로 조정 가능
        // );
        obb_util::OBB2D obb2d;
        double zMinD, zMaxD;

        std::array<obb_util::P3,8> corners3d;
        // cout<<"되고 있슈"<<endl;
        obb_util::computeOBB3D_PCA(st_CurrentObj.pcl_ClusterPoints, obb2d, zMinD, zMaxD, corners3d);

        // cout<<"되고 있슈"<<endl;
        // 결과 저장
        st_CurrentObj.f32_OBB_CenterX = static_cast<float>(obb2d.cx);
        st_CurrentObj.f32_OBB_CenterY = static_cast<float>(obb2d.cy);
        st_CurrentObj.f32_OBB_CenterZ = static_cast<float>(0.5*(zMinD + zMaxD));
        st_CurrentObj.f32_OBB_Width   = static_cast<float>(obb2d.w);
        st_CurrentObj.f32_OBB_Height  = static_cast<float>(obb2d.h);
        st_CurrentObj.f32_OBB_ZMin    = static_cast<float>(zMinD);
        st_CurrentObj.f32_OBB_ZMax    = static_cast<float>(zMaxD);
        st_CurrentObj.f32_OBB_YawRad  = static_cast<float>(obb2d.yaw);

        for (int k=0;k<8;++k) {
            st_CurrentObj.OBB_Corners_Body[k].x = static_cast<float>(corners3d[k].x);
            st_CurrentObj.OBB_Corners_Body[k].y = static_cast<float>(corners3d[k].y);
            st_CurrentObj.OBB_Corners_Body[k].z = static_cast<float>(corners3d[k].z);
        }
        st_CurrentObj.f32_PrevE = numeric_limits<float32>::quiet_NaN();
        st_CurrentObj.f32_PrevN = numeric_limits<float32>::quiet_NaN();
        st_CurrentObj.f32_Dist= std::hypot(st_CurrentObj.f32_CoreX, st_CurrentObj.f32_CoreY);
        st_CurrentObj.f32_MinX = f32_MinX;
        st_CurrentObj.f32_MinY = f32_MinY;
        st_CurrentObj.f32_MinZ = f32_MinZ;
        st_CurrentObj.f32_MaxX = f32_MaxX;
        st_CurrentObj.f32_MaxY = f32_MaxY;
        st_CurrentObj.f32_MaxZ = f32_MaxZ;
        st_CurrentObj.f32_Velocity = 0.0;


        st_CurrentObj.f32_DirX = 0.0;
        st_CurrentObj.f32_DirY = 0.0;
        st_CurrentObj.s32_Id = -1;
        st_CurrentObj.s32_Age= 0;
        st_CurrentObj.s32_MissCount = 0;
        st_CurrentObj.s32_DynamicCount =0;
        st_CurrentObj.s32_StaticCount =0;
        st_CurrentObj.s32_KfInitCount =0;
        // st_CurrentObj.Eigen_KfState= Eigen::VectorXf::Zero(4);
        // st_CurrentObj.Eigen_KfP = Eigen::MatrixXf::Identity(4, 4);
        st_CurrentObj.Eigen_KfState_CA = Eigen::Matrix<float, 6, 1>::Zero();
        st_CurrentObj.Eigen_KfP_CA     = Eigen::Matrix<float, 6, 6>::Identity();
        st_CurrentObj.b_KfInitialized = false;

        vec_NowObject.push_back(st_CurrentObj);
    }
}





// 전/현재 프레임 군집 중심점 비교 후 속도 가속도 방향 구하는 함수
void CompareCentroids(const deque<vector<LiDARObjectInfo>> &PastObject,
                      vector<LiDARObjectInfo> &vec_NowObject, float64 f64_Time,const GPS_IMU& st_GPSIMU) {
    // // === [LOG T0] 프레임 요약 (1줄) ===
    // const size_t N_prev = PastObject.empty() ? 0 : PastObject.back().size();
    // const size_t M_now  = vec_NowObject.size();
    // std::cout << std::fixed << std::setprecision(3)
    //           << "[CMP] N_prev=" << N_prev
    //           << " M_now="      << M_now
    //           << " dt="         << f64_Time << "s\n";

    // prev 접근은 empty 체크 뒤에
    if (PastObject.empty()) return;
    const auto& vec_Compare = PastObject.back();

    int32 N = vec_Compare.size();
    int32 M = vec_NowObject.size();

    // 예측 위치 기반 비용 행렬 생성
    vector<pair<float,float>> vec_PredictedPositions(N);
    for (int32 i = 0; i < N; ++i) {
        pair<float,float> pp = predictKalmanPosition(vec_Compare[i], f64_Time ,st_GPSIMU.st_IMU.f32_Yaw_deg);
        vec_PredictedPositions[i].first  = pp.first;   // pred_x
        vec_PredictedPositions[i].second = pp.second;  // pred_y
    }

    // 2) 비용 행렬 생성 (이전에 하던 대로)
    vector<vector<double>> vec_CostMatrix(N, vector<double>(M, st_Tracking.f32_COST_INF));
    for (int32 i = 0; i < N; ++i) {
        // 예측 위치
        float32 f32_Pred_x = vec_PredictedPositions[i].first;
        float32 f32_Pred_y = vec_PredictedPositions[i].second;

        for (int32 j = 0; j < M; ++j) {
            // 검출된 객체 위치
            float32 f32_Det_x = vec_NowObject[j].f32_CoreX;
            float32 f32_Det_y = vec_NowObject[j].f32_CoreY;

            // 1) Euclidean distance
            float32 f32_Dx = f32_Det_x - f32_Pred_x;
            float32 f32_Dy = f32_Det_y - f32_Pred_y;
            float32 f32_Cost_dist = sqrt(f32_Dx * f32_Dx + f32_Dy * f32_Dy);

            // 2) 2D IoU 비용
            float32 f32_Xi1 = max(vec_Compare[i].f32_MinX, vec_NowObject[j].f32_MinX);
            float32 f32_Yi1 = max(vec_Compare[i].f32_MinY, vec_NowObject[j].f32_MinY);
            float32 f32_Xi2 = min(vec_Compare[i].f32_MaxX, vec_NowObject[j].f32_MaxX);
            float32 f32_Yi2 = min(vec_Compare[i].f32_MaxY, vec_NowObject[j].f32_MaxY);
            float32 f32_Inter_w = max(0.0f, f32_Xi2 - f32_Xi1);
            float32 f32_Inter_h = max(0.0f, f32_Yi2 - f32_Yi1);
            float32 f32_Inter = f32_Inter_w * f32_Inter_h;
            float32 f32_Area_trk = (vec_Compare[i].f32_MaxX - vec_Compare[i].f32_MinX) * (vec_Compare[i].f32_MaxY - vec_Compare[i].f32_MinY);
            float32 f32_Area_det = (vec_NowObject[j].f32_MaxX - vec_NowObject[j].f32_MinX) * (vec_NowObject[j].f32_MaxY - vec_NowObject[j].f32_MinY);
            float32 f32_UnionA = f32_Area_trk + f32_Area_det - f32_Inter + 1e-6f;
            float32 f32_Cost_iou = 1.0f - (f32_Inter / f32_UnionA);

            // // 3) heading도 고려 (보류)
            // // 이전 객체의 속도 벡터 → 방향 정규화
            // float32 f32_Prev_vx = vec_Compare[i].Eigen_KfState(2);
            // float32 f32_Prev_vy = vec_Compare[i].Eigen_KfState(3);
            // float32 f32_Prev_mag = sqrt(f32_Prev_vx*f32_Prev_vx + f32_Prev_vy*f32_Prev_vy);
            // float32 f32_Prev_dir_x = f32_Prev_vx / (f32_Prev_mag + 1e-6);
            // float32 f32_Prev_dir_y = f32_Prev_vy / (f32_Prev_mag + 1e-6);

            // // 현재 감지된 객체의 방향 (또는 속도 벡터) → 정규화
            // float32 f32_Curr_vx = vec_NowObject[j].Eigen_KfState(2);
            // float32 f32_Curr_vy = vec_NowObject[j].Eigen_KfState(3);
            // float32 f32_Curr_mag = sqrt(f32_Curr_vx*f32_Curr_vx + f32_Curr_vy*f32_Curr_vy);
            // float32 f32_Curr_dir_x = f32_Curr_vx / (f32_Curr_mag + 1e-6);
            // float32 f32_Curr_dir_y = f32_Curr_vy / (f32_Curr_mag + 1e-6);

            // 방향 벡터의 코사인 유사도 (dot product)
            //float32 f32_dot = prev_dir_x * curr_dir_x + prev_dir_y * curr_dir_y;

            // 방향이 비슷하면 dot ≈ 1, 다르면 dot ≈ 0~ -1
            //float32 f32_angle_cost = 1.0f - dot;

            // 최종 비용 계산 및 게이팅
            float32 f32_Cost=st_Tracking.f32_COST_W_DIST * f32_Cost_dist
           + st_Tracking.f32_COST_W_IOU  * f32_Cost_iou;
            
            if (f32_Cost < st_Tracking.f32_COST_THRESHOLD) {
                vec_CostMatrix[i][j] = f32_Cost;
            } else {
                vec_CostMatrix[i][j] = st_Tracking.f32_COST_INF;
            }
            
        }
    }

    AssignmentProblemSolver solver;
    vector<int32> assignment;
    solver.Solve(vec_CostMatrix, assignment, AssignmentProblemSolver::optimal);

    vector<bool> matched_now(M, false);
    vector<bool> matched_prev(N, false);

    
    int matched_count = 0;
    for (int i = 0; i < N; ++i) {
        int j = (i < static_cast<int>(assignment.size())) ? assignment[i] : -1;
        // j 가 유효하고, cost 가 임계값 이내인 경우에만 매칭으로 인정
        if (j >= 0 && j < M && vec_CostMatrix[i][j] < st_Tracking.f32_COST_THRESHOLD) {
            // (선택) vec_CostMatrix[i][j] 를 로그로 찍어보면
            // 내가 설정한 COST_THRESHOLD 값이 적절한지 디버깅할 때 도움이 된다고함
            // cout << "Cost("<<i<<","<<j<<")="<<vec_CostMatrix[i][j]<<"\n";
            vec_NowObject[j].f32_PrevCoreX = vec_Compare[i].f32_CoreX;
            vec_NowObject[j].f32_PrevCoreY = vec_Compare[i].f32_CoreY;
            float32 f32_Prev_core_x = vec_Compare[i].f32_CoreX;
            float32 f32_Prev_core_y = vec_Compare[i].f32_CoreY;
            // 이제 기존 코드처럼 트랙 업데이트
            vec_NowObject[j].f32_PredCoreX = vec_PredictedPositions[i].first;
            vec_NowObject[j].f32_PredCoreY = vec_PredictedPositions[i].second;
            float32 f32_Pred_x = vec_PredictedPositions[i].first;
            float32 f32_Pred_y = vec_PredictedPositions[i].second;
            float32 f32_Det_x  = vec_NowObject[j].f32_CoreX;
            float32 f32_Det_y  = vec_NowObject[j].f32_CoreY;
            float32 f32_Dx     = f32_Det_x - vec_PredictedPositions[i].first;
            float32 f32_Dy     = f32_Det_y - vec_PredictedPositions[i].second;
            float32 f32_Distance = sqrt(f32_Dx*f32_Dx + f32_Dy*f32_Dy); // 이거는 그냥 트래킹 파라미터 조정을 어떻게 하면 좋을지 고민해야 할떄 쓸듯

            // velocity, kf_state, id 등 복사
            //vec_NowObject[j].velocity       = distance / f64_Time;
            // vec_NowObject[j].Eigen_KfState       = vec_Compare[i].Eigen_KfState;
            // vec_NowObject[j].Eigen_KfP           = vec_Compare[i].Eigen_KfP;
            vec_NowObject[j].Eigen_KfState_CA       = vec_Compare[i].Eigen_KfState_CA;
            vec_NowObject[j].Eigen_KfP_CA          = vec_Compare[i].Eigen_KfP_CA;
            vec_NowObject[j].b_KfInitialized = vec_Compare[i].b_KfInitialized;
            vec_NowObject[j].s32_Id             = vec_Compare[i].s32_Id;
            vec_NowObject[j].s32_Age           = vec_Compare[i].s32_Age+ 1;
            vec_NowObject[j].s32_MissCount     = 0;
            vec_NowObject[j].s32_StaticCount   =vec_Compare[i].s32_StaticCount;
            vec_NowObject[j].s32_DynamicCount  =vec_Compare[i].s32_DynamicCount;
            vec_NowObject[j].str_ObjectClass = vec_Compare[i].str_ObjectClass;
            vec_NowObject[j].f32_PrevE  = vec_Compare[i].f32_PrevE;
            vec_NowObject[j].f32_PrevN  = vec_Compare[i].f32_PrevN;
            vec_NowObject[j].f32_PrevYaw = vec_Compare[i].f32_PrevYaw ;

            vec_NowObject[j].str_ObjectClass      = vec_Compare[i].str_ObjectClass;
            vec_NowObject[j].last_seen_class      = vec_Compare[i].last_seen_class;
            vec_NowObject[j].continuity_counter   = vec_Compare[i].continuity_counter;
            vec_NowObject[j].is_class_locked      = vec_Compare[i].is_class_locked;

            float32 f32_PrevYYY=vec_NowObject[j].f32_PrevYaw;
            // 칼만 업데이트
            kalman_filter_update(vec_NowObject[j], f32_Det_x, f32_Det_y,vec_NowObject[j].f32_PredCoreX,
            vec_NowObject[j].f32_PredCoreY ,f64_Time,st_GPSIMU.st_IMU.f32_Yaw_deg);
            // kalman_filter_update(vec_NowObject[j], f32_Det_x, f32_Det_y,f32_Pred_x,f32_Pred_y ,f64_Time,st_GPSIMU.st_IMU.f32_Yaw_deg);
            
            // if (!vec_NowObject[j].b_IMMInit) {
            //     // 초기 IMM 설정
            //     float vx_init = (f32_Det_x - vec_NowObject[j].f32_PrevCoreX) / f64_Time;
            //     float vy_init = (f32_Det_y - vec_NowObject[j].f32_PrevCoreY) / f64_Time;
            //     IMM_Init_Lite(vec_NowObject[j],f32_Det_x, f32_Det_y,vx_init, vy_init);
            // }

            // // 관측 있을 때 → IMM 업데이트
            // if (vec_NowObject[j].s32_MissCount == 0 && vec_NowObject[j].b_IMMInit==true) {
            //     Eigen::Matrix<float,2,6> H;
            //     H << 1,0,0,0,0,0,
            //     0,1,0,0,0,0;
            //     Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.01f;

            //     IMM_Step_Lite(vec_NowObject[j],
            //       Eigen::Vector2f(f32_Det_x, f32_Det_y),static_cast<float>(f64_Time), H, R, 9.21f);  // chi2_gate는 99% 기본값
            // }

            // 디버깅 출력
            float32 f32_Rel_vx = vec_NowObject[j].Eigen_KfState_CA(2); 
            float32 f32_Rel_vy = vec_NowObject[j].Eigen_KfState_CA(3); 
            // float32 f32_Rel_vx = vec_NowObject[j].Eigen_KfState(2); 
            // float32 f32_Rel_vy = vec_NowObject[j].Eigen_KfState(3); 
            // float32 f32_Rel_vx= (vec_NowObject[j].f32_CoreX - f32_Prev_core_x)/f64_Time ;
            // float32 f32_Rel_vy= (vec_NowObject[j].f32_CoreY - f32_Prev_core_y)/f64_Time ;
            //--->
            float32 v_mps = st_GPSIMU.st_CAN.f32_CurrentVel_KPH / 3.6f;
            // cout<<"자차속도 :"<<v_mps*3.6<<endl;

            //요값 넣어서 자차랑 상대객체 속도 enu 기준으로 만든 부분
            //float32 yaw_rad = st_GPSIMU.st_IMU.f32_Yaw_rad;

            //요값 넣어서 자차랑 상대객체 속도 enu 기준으로 만든 부분
            // float32 ego_vx = v_mps * cos(yaw_rad); // East 성분
            // float32 ego_vy = v_mps * sin(yaw_rad); // North 성분

            // float vx_enu_rel = cos(yaw_rad) * f32_Rel_vx - sin(yaw_rad) * f32_Rel_vy;
            // float vy_enu_rel = sin(yaw_rad) * f32_Rel_vx + cos(yaw_rad) * f32_Rel_vy;

            
            // float32 f32_Abs_vx = f32_Rel_vx + v_mps*cos(vec_NowObject[j].f32_PrevYaw-f32_PrevYYY);
            // // cout<<"절대 x:"<<f32_Abs_vx<<endl;
            // float32 f32_Abs_vy = f32_Rel_vy + v_mps*sin(vec_NowObject[j].f32_PrevYaw-f32_PrevYYY);
            float32 f32_Abs_vx = f32_Rel_vx + v_mps;
            float32 f32_Abs_vy = f32_Rel_vy ;
            float32 f32_PreviousEmaSpeed = vec_NowObject[j].f32_Velocity;
            float32 f32_NowVelocity = sqrt(f32_Abs_vx*f32_Abs_vx + f32_Abs_vy*f32_Abs_vy);
            // cout<<"요값변화량"<<st_GPSIMU.st_IMU.f32_Yaw_deg* M_PI / 180.0f - f32_PrevYYY<<endl;
            float32 f32_SpeedPub=0;

            // 임계값으로 어떻게든 해보자
            // cout<<"차속도:"<<v_mps<<endl;
            if(v_mps*3.6<3){
                f32_Rel_vx= (vec_NowObject[j].f32_CoreX - f32_Prev_core_x)/f64_Time ;
                f32_Rel_vy= (vec_NowObject[j].f32_CoreY - f32_Prev_core_y)/f64_Time ;  
                f32_Abs_vx = f32_Rel_vx + v_mps;
                f32_Abs_vy = f32_Rel_vy ;       
                f32_NowVelocity = sqrt(f32_Abs_vy*f32_Abs_vy + f32_Abs_vx*f32_Abs_vx );
                // if(abs(st_GPSIMU.st_IMU.f32_Yaw_deg-f32_PrevYYY)>0.05)
                //     vec_NowObject[j].f32_Velocity = calculateEmaSpeed(f32_PreviousEmaSpeed,f32_NowVelocity,0.3);
                // else{
                //     vec_NowObject[j].f32_Velocity = f32_NowVelocity;
                // }
                vec_NowObject[j].f32_Velocity = f32_NowVelocity;
                if (vec_NowObject[j].f32_Velocity > 1e-3f) {
                    vec_NowObject[j].f32_DirX = f32_Abs_vx / vec_NowObject[j].f32_Velocity;
                    vec_NowObject[j].f32_DirY = f32_Abs_vy / vec_NowObject[j].f32_Velocity;
                } else {
                    vec_NowObject[j].f32_DirX = 0.0f;
                    vec_NowObject[j].f32_DirY = 0.0f;
                }
                if(vec_NowObject[j].f32_Velocity*3.6 >=4){
                    vec_NowObject[j].b_IsDynamic=true;
                    vec_NowObject[j].s32_DynamicCount++;
                    // cout<<"동적 얼마나: "<<vec_NowObject[j].s32_DynamicCount<<endl;;
                }
                else{
                    vec_NowObject[j].b_IsDynamic=false;
                    vec_NowObject[j].s32_StaticCount++;
                }

            }
            else{
                f32_NowVelocity = sqrt(f32_Abs_vy*f32_Abs_vy + f32_Abs_vx*f32_Abs_vx ) ;
                // if(abs(st_GPSIMU.st_IMU.f32_Yaw_deg-f32_PrevYYY)>0.05)
                //     vec_NowObject[j].f32_Velocity = calculateEmaSpeed(f32_PreviousEmaSpeed,f32_NowVelocity,0.3);
                // else{
                //     vec_NowObject[j].f32_Velocity = f32_NowVelocity;
                // }
                vec_NowObject[j].f32_Velocity = f32_NowVelocity;
                if (vec_NowObject[j].f32_Velocity > 1e-3f) {
                    vec_NowObject[j].f32_DirX = f32_Abs_vx / vec_NowObject[j].f32_Velocity;
                    vec_NowObject[j].f32_DirY = f32_Abs_vy / vec_NowObject[j].f32_Velocity;
                } else {
                    vec_NowObject[j].f32_DirX = 0.0f;
                    vec_NowObject[j].f32_DirY = 0.0f;
                }
                if(vec_NowObject[j].f32_Velocity*3.6 >=6){
                    vec_NowObject[j].b_IsDynamic=true;
                    vec_NowObject[j].s32_DynamicCount++;
                    if(vec_NowObject[j].str_ObjectClass=="person"){
                        cout<<"동적사람속도: "<<vec_NowObject[j].f32_Velocity<<endl;
                    }
                
                        //자동차,트럭,버스 정적상태에 대해서 동적으로 잠시 튀는 현상 막기 위한 코드 
                        // if(vec_NowObject[j].str_ObjectClass=="car" || vec_NowObject[j].str_ObjectClass=="truck" || vec_NowObject[j].str_ObjectClass=="bus")
                        //     cout<<"동적 얼마나: "<<vec_NowObject[j].s32_DynamicCount<<endl;
                        //     if(vec_NowObject[j].s32_DynamicCount >= vec_NowObject[j].s32_StaticCount){
                        //         vec_NowObject[j].b_IsDynamic=true;
                        //         vec_NowObject[j].s32_DynamicCount++;
                        //     }
                        //     else{ 
                        //         if(vec_NowObject[j].s32_DynamicCount <=12){
                        //             vec_NowObject[j].b_IsDynamic=false;
                        //             vec_NowObject[j].s32_StaticCount++;
                        //         }
                        //     }
                }
                else{
                    // if(vec_NowObject[j].str_ObjectClass=="person"){
                    //     if(vec_NowObject[j].f32_Velocity*3.6 >=3){
                    //         vec_NowObject[j].b_IsDynamic=true;
                    //         vec_NowObject[j].s32_DynamicCount++;
                    //     }
                    // }
                    vec_NowObject[j].b_IsDynamic=false;
                    vec_NowObject[j].s32_StaticCount++;
                    if(vec_NowObject[j].str_ObjectClass=="person"){
                        cout<<"정적사람속도: "<<vec_NowObject[j].f32_Velocity<<endl;
                    }
                }
            }
            

            


            //17프레임부터 동정적 판단 가능 ==
            // if(f32_SpeedPub*3.6<6){
            //     cout<<"나이 : "<<vec_NowObject[j].s32_Age<<endl;
            //     cout<<"거리 : "<<sqrt(vec_NowObject[j].f32_CoreX*vec_NowObject[j].f32_CoreX+vec_NowObject[j].f32_CoreY*vec_NowObject[j].f32_CoreY)<<endl;
            //     cout<<"뭘까요? : "<<"정적"<<endl;
            // }
            // else{
            //     cout<<"뭘까요? : "<<"동적!!!"<<endl;
            //     cout<<"절대속도 : "<<f32_SpeedPub *3.6 <<endl;
            // }

            // vec_NowObject[j].f32_Velocity = f32_Speed_pub;
            // cout<<"동적 정적 판단용:"<<speed_pub*3.6<<endl;
            // float32 f32_abs_vx = (vec_NowObject[j].f32_CoreX - prev_core_x)/ f64_Time + current_speed_x;
            // float32 f32_abs_vy = (vec_NowObject[j].f32_CoreY - prev_core_y)/ f64_Time + current_speed_y;
            // float32 f32_speed_pub_2 = sqrt(abs_vx*abs_vx + abs_vy*abs_vy) ;
            // //vec_NowObject[j].velocity = speed_pub_2;//칼만 위치로 얻은 절대속도

            // float32 f32_Abs_vx = (vec_NowObject[j].Eigen_KfState(2)+ current_speed_x);
            // float32 f32_Abs_vy = (vec_NowObject[j].Eigen_KfState(3)) + current_speed_y;
            // float32 f32_speed_pub_3 = sqrt(Abs_vx*Abs_vx + Abs_vy*Abs_vy) ;
            // vec_NowObject[j].velocity = speed_pub_3; //칼만 속도로 얻은 절대 속도

            // const float32 f32_EPS = 1e-6f;
            // if (f32_Speed_pub > f32_EPS) {
            //     vec_NowObject[j].f32_dir_x = f32_Di_x / (f32_Speed_pub * f64_Time);
            //     vec_NowObject[j].f32_dir_y = f32_Di_y / (f32_Speed_pub * f64_Time);
            // } else {
            //     vec_NowObject[j].f32_dir_x = vec_NowObject[j].f32_dir_y = 0.0f;
            // }


            matched_now[j]  = true;
            matched_prev[i] = true;
            ++matched_count;
        }
    }

    
    //현재프레임에서 매칭되지 않은 애들 처리
    for (int j = 0; j < M; ++j) {
        if (!matched_now[j]&& vec_NowObject[j].s32_Id < 0) {
            vec_NowObject[j].b_KfInitialized = false;
            vec_NowObject[j].s32_Id = st_Tracking.s32_NewTrackId++;
            vec_NowObject[j].s32_Age= 1;
            vec_NowObject[j].s32_MissCount = 0;
            vec_NowObject[j].f32_PrevE = std::numeric_limits<float>::quiet_NaN();
            vec_NowObject[j].f32_PrevN = std::numeric_limits<float>::quiet_NaN();
            kalman_filter_update(vec_NowObject[j],vec_NowObject[j].f32_CoreX,vec_NowObject[j].f32_CoreY,0,0,f64_Time,st_GPSIMU.st_IMU.f32_Yaw_deg);
            // if (!vec_NowObject[j].b_IMMInit) {
            //     // 초기 IMM 설정
            //     float vx_init = (f32_Det_x - f32_Pred_x) / f64_Time;
            //     float vy_init = (f32_Det_y - f32_Pred_y) / f64_Time;
            //     IMM_Init_Lite(vec_NowObject[j],f32_Det_x, f32_Det_y,vx_init, vy_init);
            // }

            // // 관측 있을 때 → IMM 업데이트
            // if (vec_NowObject[j].s32_MissCount == 0 && vec_NowObject[j].b_IMMInit==true) {
            //     Eigen::Matrix<float,2,6> H;
            //     H << 1,0,0,0,0,0,
            //     0,1,0,0,0,0;
            //     Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.01f;

            //     IMM_Step_Lite(vec_NowObject[j],
            //       Eigen::Vector2f(f32_Det_x, f32_Det_y),static_cast<float>(f64_Time), H, R, 9.21f);  // chi2_gate는 99% 기본값
            // }
        }
    }

    //매칭되지 않은 전프레임 처리
    for (int i = 0; i < N; ++i) {
        if (!matched_prev[i]&& vec_Compare[i].b_KfInitialized) {
            LiDARObjectInfo missed = vec_Compare[i];
            missed.s32_Age= vec_Compare[i].s32_Age+ 1;
            missed.s32_MissCount = vec_Compare[i].s32_MissCount + 1;

            if (missed.s32_MissCount < 25) {
                // 1) 칼만 predict-only 단계 수행 (kf_state, kf_P 갱신)
                kalman_filter_predict(missed, f64_Time,st_GPSIMU.st_IMU.f32_Yaw_deg);

                // if (!vec_NowObject[j].b_IMMInit) {
                //     // 초기 IMM 설정
                //     float vx_init = (f32_Det_x - f32_Pred_x) / f64_Time;
                //     float vy_init = (f32_Det_y - f32_Pred_y) / f64_Time;
                //     IMM_Init_Lite(vec_NowObject[j],f32_Det_x, f32_Det_y,vx_init, vy_init);
                // }

                // // 관측 없을 때 → IMM 예측
                // if (vec_NowObject[j].s32_MissCount != 0 && vec_NowObject[j].b_IMMInit==true) {
                //     IMM_PredictOnly_Lite(vec_NowObject[j],
                //                         static_cast<float>(f64_Time),
                //                         st_GPSIMU.st_IMU.f32_Yaw_deg);
                // }

                // 2) 예측된 상태에서 위치(update core_x/core_y)
                // missed.f32_CoreX = missed.Eigen_KfState(0);
                // missed.f32_CoreY = missed.Eigen_KfState(1);
                missed.f32_CoreX = missed.Eigen_KfState_CA(0);
                missed.f32_CoreY = missed.Eigen_KfState_CA(1);

                // // 3) 속도·가속도(velocity, acceleration)도
                // float32 f32_Rel_vx = missed.Eigen_KfState(2);
                // float32 f32_Rel_vy = missed.Eigen_KfState(3);
                // // ego 속도
                // //float32 f32_ego_vx = current_speed_x;
                // //float32 f32_ego_vy = current_speed_y;

                // // 절대 속도로 변환
                // float32 f32_Abs_vx = f32_Rel_vx ;//+ ego_vx;
                // float32 f32_Abs_vy = f32_Rel_vy ;//+ ego_vy;
                // missed.f32_Velocity = sqrt(f32_Abs_vx*f32_Abs_vx + f32_Abs_vy*f32_Abs_vy);
                // missed.f32_DirX = f32_Abs_vx / (missed.f32_Velocity + 1e-6f);
                // missed.f32_DirY = f32_Abs_vy / (missed.f32_Velocity + 1e-6f);

                // 가속도도 동일하게
                // std::cout << "[MISS] prev_id=" << vec_Compare[i].s32_Id
                //     << " miss=" << (vec_Compare[i].s32_MissCount + 1) << "\n";
                vec_NowObject.push_back(missed);
            }
        }
    }
}

