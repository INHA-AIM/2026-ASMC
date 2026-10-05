#include <Lidar/Tracking/tracking_utils.hpp>
#include <Lidar/Kalman/kalman_utils.hpp>
#include <iomanip>
#include <cstdint>
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

float32 calculateEmaSpeed(float32 previousEmaSpeed, float32 rawSpeed, float32 alpha) {
    // EMA 공식: (1 - alpha) * 이전 EMA 속도 + alpha * 현재 raw 속도
    return (1.0 - alpha) * previousEmaSpeed + alpha * rawSpeed;
} // 급회전할때 적용할 것 


namespace obb_util {

struct P2 { double x, y; };
struct P3 { double x, y, z; };

struct OBB2D {
    double cx{0}, cy{0};     // 중심 (XY)
    double w{0}, h{0};       // w: yaw 축 길이, h: 수직 축 길이
    double yaw{0};           // 라디안, +x 기준 CCW, pi 주기의 축 방향
    std::array<P2,4> corners; // z 미포함, 시계/반시계 일관 유지
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 0.5 * kPi;
constexpr double kLShapeAngleStep = kPi / 180.0;  // 1 deg
constexpr double kClosenessMinDistance = 0.05;    // voxel leaf(0.1 m)의 절반
constexpr double kMinimumObbSide = 0.05;          // planner OBB 유효성 하한
constexpr std::size_t kMinLShapePoints = 10;
constexpr double kYawFilterAlpha = 0.25;
constexpr double kMinReliableAspectRatio = 1.20;

inline double wrapToPi(double angle) {
    return std::remainder(angle, 2.0 * kPi);
}

// 직사각형의 축 방향은 yaw와 yaw+pi가 동일하다.
inline double normalizeAxisYaw(double yaw) {
    while (yaw >= kHalfPi) yaw -= kPi;
    while (yaw < -kHalfPi) yaw += kPi;
    return yaw;
}

inline std::vector<P2> finiteXY(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    std::vector<P2> points;
    points.reserve(cloud.size());
    for (const auto& p : cloud.points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            points.push_back({p.x, p.y});
        }
    }
    return points;
}

// 주어진 yaw에서 모든 점을 포함하는 OBB를 다시 계산한다.
// yaw smoothing 후 기존 박스를 단순 회전시키지 않고 재투영하므로 점유 영역을 과소평가하지 않는다.
inline bool computeOBBAtYaw(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    double yaw,
    OBB2D& obb2d,
    double& zMin, double& zMax,
    std::array<P3,8>& corners3d
) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    double uMin = std::numeric_limits<double>::infinity();
    double uMax = -std::numeric_limits<double>::infinity();
    double vMin = std::numeric_limits<double>::infinity();
    double vMax = -std::numeric_limits<double>::infinity();
    zMin = std::numeric_limits<double>::infinity();
    zMax = -std::numeric_limits<double>::infinity();
    std::size_t validCount = 0;

    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const double u =  p.x * c + p.y * s;
        const double v = -p.x * s + p.y * c;
        uMin = std::min(uMin, u);
        uMax = std::max(uMax, u);
        vMin = std::min(vMin, v);
        vMax = std::max(vMax, v);
        zMin = std::min(zMin, static_cast<double>(p.z));
        zMax = std::max(zMax, static_cast<double>(p.z));
        ++validCount;
    }

    if (validCount == 0) {
        obb2d = {};
        zMin = 0.0;
        zMax = 0.0;
        corners3d.fill({0.0, 0.0, 0.0});
        return false;
    }

    const double centerU = 0.5 * (uMin + uMax);
    const double centerV = 0.5 * (vMin + vMax);
    obb2d.cx = centerU * c - centerV * s;
    obb2d.cy = centerU * s + centerV * c;
    // A distant object can be sampled on only one scan line.  Keep the
    // measured center, but make the footprint non-degenerate at the same
    // minimum accepted by PlannerObjectAdapter instead of dropping it.
    obb2d.w = std::max(uMax - uMin, kMinimumObbSide);
    obb2d.h = std::max(vMax - vMin, kMinimumObbSide);
    obb2d.yaw = normalizeAxisYaw(yaw);

    const double halfW = 0.5 * obb2d.w;
    const double halfH = 0.5 * obb2d.h;
    const P2 axisU{c, s};
    const P2 axisV{-s, c};
    const std::array<std::pair<double,double>,4> signs{{
        {-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}
    }};

    for (int i = 0; i < 4; ++i) {
        const double su = signs[i].first;
        const double sv = signs[i].second;
        obb2d.corners[i] = {
            obb2d.cx + su * halfW * axisU.x + sv * halfH * axisV.x,
            obb2d.cy + su * halfW * axisU.y + sv * halfH * axisV.y
        };
        corners3d[i] = {obb2d.corners[i].x, obb2d.corners[i].y, zMin};
        corners3d[i + 4] = {obb2d.corners[i].x, obb2d.corners[i].y, zMax};
    }
    return true;
}

// Zhang et al.의 search-based L-Shape fitting 중 closeness criterion.
// 점이 10개 미만이면 방향 신뢰도가 낮으므로 결정론적인 AABB(yaw=0)를 사용한다.
inline void computeOBB3D_LShape(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    OBB2D& obb2d,
    double& zMin, double& zMax,
    std::array<P3,8>& corners3d
) {
    const std::vector<P2> points = finiteXY(cloud);
    if (points.size() < kMinLShapePoints) {
        computeOBBAtYaw(cloud, 0.0, obb2d, zMin, zMax, corners3d);
        if (obb2d.w < obb2d.h) {
            computeOBBAtYaw(cloud, kHalfPi, obb2d, zMin, zMax, corners3d);
        }
        return;
    }

    double bestScore = -std::numeric_limits<double>::infinity();
    double bestYaw = 0.0;

    for (double theta = 0.0; theta < kHalfPi; theta += kLShapeAngleStep) {
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        double uMin = std::numeric_limits<double>::infinity();
        double uMax = -std::numeric_limits<double>::infinity();
        double vMin = std::numeric_limits<double>::infinity();
        double vMax = -std::numeric_limits<double>::infinity();

        for (const auto& p : points) {
            const double u =  p.x * c + p.y * s;
            const double v = -p.x * s + p.y * c;
            uMin = std::min(uMin, u);
            uMax = std::max(uMax, u);
            vMin = std::min(vMin, v);
            vMax = std::max(vMax, v);
        }

        double score = 0.0;
        for (const auto& p : points) {
            const double u =  p.x * c + p.y * s;
            const double v = -p.x * s + p.y * c;
            const double distanceToU = std::min(u - uMin, uMax - u);
            const double distanceToV = std::min(v - vMin, vMax - v);
            const double distanceToEdge = std::max(
                std::min(distanceToU, distanceToV), kClosenessMinDistance);
            score += 1.0 / distanceToEdge;
        }

        if (score > bestScore) {
            bestScore = score;
            bestYaw = theta;
        }
    }

    computeOBBAtYaw(cloud, bestYaw, obb2d, zMin, zMax, corners3d);

    // 긴 축이 항상 yaw 축이 되도록 정규화하여 90도 축 전환을 제거한다.
    if (obb2d.w < obb2d.h) {
        bestYaw += kHalfPi;
        computeOBBAtYaw(cloud, bestYaw, obb2d, zMin, zMax, corners3d);
    }
}

inline void writeOBBToObject(
    LiDARObjectInfo& object,
    const OBB2D& obb2d,
    double zMin, double zMax,
    const std::array<P3,8>& corners3d
) {
    object.f32_OBB_CenterX = static_cast<float>(obb2d.cx);
    object.f32_OBB_CenterY = static_cast<float>(obb2d.cy);
    object.f32_OBB_CenterZ = static_cast<float>(0.5 * (zMin + zMax));
    object.f32_OBB_Width = static_cast<float>(obb2d.w);
    object.f32_OBB_Height = static_cast<float>(obb2d.h);
    object.f32_OBB_ZMin = static_cast<float>(zMin);
    object.f32_OBB_ZMax = static_cast<float>(zMax);
    object.f32_OBB_YawRad = static_cast<float>(obb2d.yaw);

    for (int i = 0; i < 8; ++i) {
        object.OBB_Corners_Body[i].x = static_cast<float>(corners3d[i].x);
        object.OBB_Corners_Body[i].y = static_cast<float>(corners3d[i].y);
        object.OBB_Corners_Body[i].z = static_cast<float>(corners3d[i].z);
    }
}

// 같은 트랙으로 매칭된 객체에만 적용한다. 이전 OBB를 현재 body 좌표계로 회전 보정한 뒤
// pi 주기의 축 각도 오차를 EMA하고, 필터 yaw에서 OBB 전체를 다시 계산한다.
inline void stabilizeMatchedOBB(
    const LiDARObjectInfo& previous,
    LiDARObjectInfo& current,
    double egoYawNowDeg
) {
    // miss 동안에는 Kalman의 ego yaw 기준만 갱신되고 OBB 자체는 갱신되지 않으므로,
    // 재검출 첫 프레임은 raw L-Shape를 채택하고 다음 연속 프레임부터 smoothing한다.
    if (current.pcl_ClusterPoints.empty() || previous.s32_MissCount > 0) return;

    const double egoYawNow = egoYawNowDeg * kPi / 180.0;
    const double egoYawDelta = wrapToPi(egoYawNow - previous.f32_PrevYaw);
    const double predictedYaw = normalizeAxisYaw(
        static_cast<double>(previous.f32_OBB_YawRad) - egoYawDelta);
    const double rawYaw = current.f32_OBB_YawRad;
    const double aspectRatio = current.f32_OBB_Height > 1e-6f
        ? current.f32_OBB_Width / current.f32_OBB_Height
        : std::numeric_limits<double>::infinity();

    double filteredYaw = predictedYaw;
    if (aspectRatio >= kMinReliableAspectRatio) {
        const double yawError = std::remainder(rawYaw - predictedYaw, kPi);
        filteredYaw = predictedYaw + kYawFilterAlpha * yawError;
    }
    filteredYaw = normalizeAxisYaw(filteredYaw);

    OBB2D filteredObb;
    double zMin = 0.0;
    double zMax = 0.0;
    std::array<P3,8> corners3d{};
    if (computeOBBAtYaw(
            current.pcl_ClusterPoints, filteredYaw,
            filteredObb, zMin, zMax, corners3d)) {
        writeOBBToObject(current, filteredObb, zMin, zMax, corners3d);
    }
}



} // namespace obb_util

namespace tracking_association {

bool IsGroundNoiseClass(const std::string& object_class) {
    return object_class == "noise" || object_class == "지면노이즈";
}

bool ShouldCarryPreviousClass(const LiDARObjectInfo& previous,
                              const LiDARObjectInfo& current) {
    // A low, sparse first return used to label an otherwise valid obstacle as
    // ground noise for the entire lifetime of the track.  A current
    // non-noise observation must be allowed to recover immediately.  In the
    // opposite direction, do not let one noisy measurement downgrade an
    // already valid UNKNOWN/vehicle/pedestrian track.
    return !IsGroundNoiseClass(previous.str_ObjectClass) ||
           IsGroundNoiseClass(current.str_ObjectClass);
}

float AabbIou(const LiDARObjectInfo& left, const LiDARObjectInfo& right) {
    const float intersection_width = std::max(
        0.0F, std::min(left.f32_MaxX, right.f32_MaxX) -
                  std::max(left.f32_MinX, right.f32_MinX));
    const float intersection_height = std::max(
        0.0F, std::min(left.f32_MaxY, right.f32_MaxY) -
                  std::max(left.f32_MinY, right.f32_MinY));
    const float intersection = intersection_width * intersection_height;
    const float left_area = std::max(
        0.0F, left.f32_MaxX - left.f32_MinX) *
        std::max(0.0F, left.f32_MaxY - left.f32_MinY);
    const float right_area = std::max(
        0.0F, right.f32_MaxX - right.f32_MinX) *
        std::max(0.0F, right.f32_MaxY - right.f32_MinY);
    const float union_area = left_area + right_area - intersection;
    return union_area > 1e-6F ? intersection / union_area : 0.0F;
}

float ShapeAgreement(const LiDARObjectInfo& left,
                     const LiDARObjectInfo& right) {
    const auto side_agreement = [](float lhs, float rhs) {
        lhs = std::max(lhs, 0.05F);
        rhs = std::max(rhs, 0.05F);
        return std::min(lhs, rhs) / std::max(lhs, rhs);
    };
    // 직사각형 OBB의 yaw 축은 90도 바뀌어도 같은 점유를 나타낼 수 있다.
    // 긴 변/짧은 변끼리 비교해 축 교환만으로 연관이 끊기지 않게 한다.
    const float left_long = std::max(
        left.f32_OBB_Width, left.f32_OBB_Height);
    const float left_short = std::min(
        left.f32_OBB_Width, left.f32_OBB_Height);
    const float right_long = std::max(
        right.f32_OBB_Width, right.f32_OBB_Height);
    const float right_short = std::min(
        right.f32_OBB_Width, right.f32_OBB_Height);
    return side_agreement(left_long, right_long) *
           side_agreement(left_short, right_short);
}

bool FiniteMapCenter(const LiDARObjectInfo& object) {
    return std::isfinite(object.f32_CoreE) &&
           std::isfinite(object.f32_CoreN);
}

float MapCenterDistance(const LiDARObjectInfo& left,
                        const LiDARObjectInfo& right) {
    if (!FiniteMapCenter(left) || !FiniteMapCenter(right)) {
        return std::numeric_limits<float>::infinity();
    }
    return std::hypot(
        left.f32_CoreE - right.f32_CoreE,
        left.f32_CoreN - right.f32_CoreN);
}

void RecomputeGeometry(LiDARObjectInfo& object) {
    if (object.pcl_ClusterPoints.empty()) return;

    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    std::size_t valid_points = 0;
    for (const auto& point : object.pcl_ClusterPoints.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        min_z = std::min(min_z, point.z);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
        max_z = std::max(max_z, point.z);
        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;
        ++valid_points;
    }
    if (valid_points == 0) return;

    object.f32_CoreX = static_cast<float>(sum_x / valid_points);
    object.f32_CoreY = static_cast<float>(sum_y / valid_points);
    object.f32_CoreZ = static_cast<float>(sum_z / valid_points);
    object.f32_MinX = min_x;
    object.f32_MinY = min_y;
    object.f32_MinZ = min_z;
    object.f32_MaxX = max_x;
    object.f32_MaxY = max_y;
    object.f32_MaxZ = max_z;
    object.f32_Dist = std::hypot(object.f32_CoreX, object.f32_CoreY);
    object.pcl_ClusterPoints.width = static_cast<std::uint32_t>(
        object.pcl_ClusterPoints.points.size());
    object.pcl_ClusterPoints.height = 1;
    object.pcl_ClusterPoints.is_dense = false;

    obb_util::OBB2D obb;
    double z_min = 0.0;
    double z_max = 0.0;
    std::array<obb_util::P3, 8> corners{};
    obb_util::computeOBB3D_LShape(
        object.pcl_ClusterPoints, obb, z_min, z_max, corners);
    obb_util::writeOBBToObject(object, obb, z_min, z_max, corners);
}

}  // namespace tracking_association

void ConsolidateCurrentDetections(vector<LiDARObjectInfo>& now_object) {
    if (now_object.size() < 2) return;

    bool merged = true;
    while (merged) {
        merged = false;
        for (std::size_t left_index = 0;
             left_index < now_object.size() && !merged; ++left_index) {
            for (std::size_t right_index = left_index + 1;
                 right_index < now_object.size(); ++right_index) {
                LiDARObjectInfo& left = now_object[left_index];
                LiDARObjectInfo& right = now_object[right_index];
                // 클러스터 평균은 관측되는 면이 달라지면 움직이지만, OBB 중심은
                // 동일 점유를 비교하기에 더 직접적이다.
                const float center_distance = std::hypot(
                    left.f32_OBB_CenterX - right.f32_OBB_CenterX,
                    left.f32_OBB_CenterY - right.f32_OBB_CenterY);
                if (center_distance >
                    st_Tracking.f32_DuplicateCenterDistance) {
                    continue;
                }
                if (tracking_association::AabbIou(left, right) <
                    st_Tracking.f32_DuplicateIouThreshold) {
                    continue;
                }

                std::size_t owner_index = left_index;
                std::size_t fragment_index = right_index;
                if (right.pcl_ClusterPoints.size() >
                    left.pcl_ClusterPoints.size()) {
                    owner_index = right_index;
                    fragment_index = left_index;
                }
                LiDARObjectInfo& owner = now_object[owner_index];
                const LiDARObjectInfo& fragment = now_object[fragment_index];
                owner.pcl_ClusterPoints.points.insert(
                    owner.pcl_ClusterPoints.points.end(),
                    fragment.pcl_ClusterPoints.points.begin(),
                    fragment.pcl_ClusterPoints.points.end());
                tracking_association::RecomputeGeometry(owner);
                now_object.erase(now_object.begin() + fragment_index);
                merged = true;
                break;
            }
        }
    }
}




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
        // Ground points have already been removed against the fitted plane.
        // Do not classify the remaining cluster from absolute LiDAR-frame
        // centroid Z.  At range, a real low obstacle may expose only its
        // bottom scan line and have the same centroid Z as the road.  The old
        // <= -1.3 m rule therefore hid valid obstacles and the tracker then
        // kept that stale label even after their full geometry became visible.
        // L-Shape fitting 기반 OBB
        obb_util::OBB2D obb2d;
        double zMinD, zMaxD;

        std::array<obb_util::P3,8> corners3d;
        // cout<<"되고 있슈"<<endl;
        obb_util::computeOBB3D_LShape(st_CurrentObj.pcl_ClusterPoints, obb2d, zMinD, zMaxD, corners3d);

        // cout<<"되고 있슈"<<endl;
        obb_util::writeOBBToObject(st_CurrentObj, obb2d, zMinD, zMaxD, corners3d);
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
        st_CurrentObj.f32_AbsVx = 0.0;
        st_CurrentObj.f32_AbsVy = 0.0;


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

    // 비용 행렬 생성. body 좌표의 같은 정적 물체는 자차가 움직인 거리만큼
    // 프레임마다 이동해 보이므로, 동기화된 pose로 만든 map 중심을 우선 쓴다.
    // OBB의 body-frame AABB IoU 역시 회전/자차 이동에 민감하므로 연관 비용에는
    // 물체의 두 평면 치수가 얼마나 비슷한지만 사용한다.
    vector<vector<double>> vec_CostMatrix(N, vector<double>(M, st_Tracking.f32_COST_INF));
    for (int32 i = 0; i < N; ++i) {
        // 예측 위치
        float32 f32_Pred_x = vec_PredictedPositions[i].first;
        float32 f32_Pred_y = vec_PredictedPositions[i].second;

        for (int32 j = 0; j < M; ++j) {
            // 검출된 객체 위치
            float32 f32_Det_x = vec_NowObject[j].f32_CoreX;
            float32 f32_Det_y = vec_NowObject[j].f32_CoreY;

            // 1) Euclidean distance. map 좌표가 유효하지 않은 프레임에만
            // 기존 Kalman body-frame 예측으로 되돌아간다.
            float32 f32_Dx = f32_Det_x - f32_Pred_x;
            float32 f32_Dy = f32_Det_y - f32_Pred_y;
            float32 f32_Cost_dist = sqrt(f32_Dx * f32_Dx + f32_Dy * f32_Dy);
            if (st_Tracking.b_UseMapAssociation &&
                tracking_association::FiniteMapCenter(vec_Compare[i]) &&
                tracking_association::FiniteMapCenter(vec_NowObject[j])) {
                f32_Cost_dist = tracking_association::MapCenterDistance(
                    vec_Compare[i], vec_NowObject[j]);
            }

            // 2) 관측점 수에 따라 OBB 크기가 달라져도 0~1 범위 안에서만
            // 비용을 더한다. 서로 다른 크기의 인접 물체를 무조건 연결하지
            // 않으면서, 자차 이동만으로 IoU가 0이 되는 문제를 피한다.
            const float32 f32_Cost_shape = 1.0F -
                tracking_association::ShapeAgreement(
                    vec_Compare[i], vec_NowObject[j]);

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
            float32 f32_Cost =
                st_Tracking.f32_COST_W_DIST * f32_Cost_dist +
                st_Tracking.f32_COST_W_IOU * f32_Cost_shape;
            // 동일 후보가 여럿이면 오래 유지된 실측 트랙을 우선한다. 보너스는
            // 게이트를 우회할 정도로 커지지 않도록 설정값 이내로 제한한다.
            const float32 age_ratio = std::clamp(
                static_cast<float32>(vec_Compare[i].s32_Age) / 30.0F,
                0.0F, 1.0F);
            f32_Cost = std::max(
                0.0F,
                f32_Cost - st_Tracking.f32_ContinuityBonus * age_ratio);
            
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
            vec_NowObject[j].s32_KfInitCount = vec_Compare[i].s32_KfInitCount;
            vec_NowObject[j].s32_Id             = vec_Compare[i].s32_Id;
            vec_NowObject[j].s32_Age           = vec_Compare[i].s32_Age+ 1;
            vec_NowObject[j].s32_MissCount     = 0;
            vec_NowObject[j].s32_StaticCount   =vec_Compare[i].s32_StaticCount;
            vec_NowObject[j].s32_DynamicCount  =vec_Compare[i].s32_DynamicCount;
            vec_NowObject[j].f32_PrevE  = vec_Compare[i].f32_PrevE;
            vec_NowObject[j].f32_PrevN  = vec_Compare[i].f32_PrevN;
            vec_NowObject[j].f32_PrevYaw = vec_Compare[i].f32_PrevYaw ;

            if (tracking_association::ShouldCarryPreviousClass(
                    vec_Compare[i], vec_NowObject[j])) {
                vec_NowObject[j].str_ObjectClass =
                    vec_Compare[i].str_ObjectClass;
                vec_NowObject[j].str_YoloClass =
                    vec_Compare[i].str_YoloClass;
                vec_NowObject[j].last_seen_class =
                    vec_Compare[i].last_seen_class;
                vec_NowObject[j].continuity_counter =
                    vec_Compare[i].continuity_counter;
                vec_NowObject[j].class_miss_counter =
                    vec_Compare[i].class_miss_counter;
                vec_NowObject[j].is_class_locked =
                    vec_Compare[i].is_class_locked;
            }

            obb_util::stabilizeMatchedOBB(
                vec_Compare[i], vec_NowObject[j],
                st_GPSIMU.st_IMU.f32_Yaw_deg);
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

            const float32 f32_Rel_vx = vec_NowObject[j].Eigen_KfState_CA(2);
            const float32 f32_Rel_vy = vec_NowObject[j].Eigen_KfState_CA(3);
            const float32 v_mps =
                st_GPSIMU.st_CAN.f32_CurrentVel_KPH / 3.6f;
            const float32 f32_Abs_vx = f32_Rel_vx + v_mps;
            const float32 f32_Abs_vy = f32_Rel_vy;

            vec_NowObject[j].f32_AbsVx = f32_Abs_vx;
            vec_NowObject[j].f32_AbsVy = f32_Abs_vy;
            vec_NowObject[j].f32_Velocity =
                std::hypot(f32_Abs_vx, f32_Abs_vy);
            if (vec_NowObject[j].f32_Velocity > 1e-3f) {
                vec_NowObject[j].f32_DirX =
                    f32_Abs_vx / vec_NowObject[j].f32_Velocity;
                vec_NowObject[j].f32_DirY =
                    f32_Abs_vy / vec_NowObject[j].f32_Velocity;
            } else {
                vec_NowObject[j].f32_DirX = 0.0f;
                vec_NowObject[j].f32_DirY = 0.0f;
            }

            // 동정적 판정은 트래킹의 실시간 속도를 변경하지 않는 별도
            // MotionClassifier가 이 프레임의 모든 트랙에 대해 수행한다.
            

            


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

    // 한 물체가 직전 프레임에 두 트랙으로 분할됐다가 현재 프레임에 다시
    // 합쳐지면 Hungarian의 1:1 제약 때문에 나머지 트랙은 miss ghost가 된다.
    // 그 ghost가 다음 프레임에서 ID를 되찾아 가는 현상을 막기 위해, 실제로
    // 매칭된 현재 관측과 사실상 같은 위치/크기인 이전 중복 트랙은 종료한다.
    const float32 retire_distance = std::max(
        0.0F, st_Tracking.f32_MergedTrackRetireDistance);
    const float32 retire_shape_agreement = std::clamp(
        st_Tracking.f32_DuplicateIouThreshold * 0.8F, 0.0F, 1.0F);
    for (int i = 0; i < N; ++i) {
        if (matched_prev[i]) continue;
        for (int j = 0; j < M; ++j) {
            if (!matched_now[j]) continue;
            if (tracking_association::MapCenterDistance(
                    vec_Compare[i], vec_NowObject[j]) > retire_distance) {
                continue;
            }
            if (tracking_association::ShapeAgreement(
                    vec_Compare[i], vec_NowObject[j]) <
                retire_shape_agreement) {
                continue;
            }
            matched_prev[i] = true;
            break;
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
