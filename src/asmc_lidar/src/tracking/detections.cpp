// Detection records and same-scan consolidation.
// Ported from Main src/Lidar/Tracking/Tracking.cpp (namespace tracking_association,
// ConsolidateCurrentDetections, Objectinfo_for_km) and
// src/SensorFusion/FusionClassState.hpp (fusion_state::Copy) at 8f93c22e.
// Not ported (legacy-only map association): FiniteMapCenter/MapCenterDistance
// (Tracking.cpp:323-336). TrackedObject has no f32_PrevE/N, so Main's NaN
// initialisation of them (:539-540) has no counterpart.
#include <asmc_lidar/pipeline/obb.hpp>
#include <asmc_lidar/pipeline/tracker.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace tracking_association {

// Ported from Main Tracking.cpp:269-271 (8f93c22e)
bool IsGroundNoiseClass(const std::string& object_class) {
  return object_class == "noise" || object_class == "지면노이즈";
}

// Ported from Main Tracking.cpp:273-282 (8f93c22e)
bool ShouldCarryPreviousClass(const TrackedObject& previous, const TrackedObject& current) {
  // A current non-noise observation recovers a track labelled ground noise
  // immediately; one noisy measurement does not downgrade a valid track.
  return !IsGroundNoiseClass(previous.str_ObjectClass) || IsGroundNoiseClass(current.str_ObjectClass);
}

// Ported from Main Tracking.cpp:284-300 (8f93c22e)
float AabbIou(const TrackedObject& left, const TrackedObject& right) {
  const float intersection_width =
      std::max(0.0F, std::min(left.f32_MaxX, right.f32_MaxX) - std::max(left.f32_MinX, right.f32_MinX));
  const float intersection_height =
      std::max(0.0F, std::min(left.f32_MaxY, right.f32_MaxY) - std::max(left.f32_MinY, right.f32_MinY));
  const float intersection = intersection_width * intersection_height;
  const float left_area =
      std::max(0.0F, left.f32_MaxX - left.f32_MinX) * std::max(0.0F, left.f32_MaxY - left.f32_MinY);
  const float right_area =
      std::max(0.0F, right.f32_MaxX - right.f32_MinX) * std::max(0.0F, right.f32_MaxY - right.f32_MinY);
  const float union_area = left_area + right_area - intersection;
  return union_area > 1e-6F ? intersection / union_area : 0.0F;
}

// Ported from Main Tracking.cpp:302-321 (8f93c22e)
float ShapeAgreement(const TrackedObject& left, const TrackedObject& right) {
  const auto side_agreement = [](float lhs, float rhs) {
    lhs = std::max(lhs, 0.05F);
    rhs = std::max(rhs, 0.05F);
    return std::min(lhs, rhs) / std::max(lhs, rhs);
  };
  // An OBB axis may swap by 90 deg for the same occupancy: compare long with
  // long and short with short sides.
  const float left_long = std::max(left.f32_OBB_Width, left.f32_OBB_Height);
  const float left_short = std::min(left.f32_OBB_Width, left.f32_OBB_Height);
  const float right_long = std::max(right.f32_OBB_Width, right.f32_OBB_Height);
  const float right_short = std::min(right.f32_OBB_Width, right.f32_OBB_Height);
  return side_agreement(left_long, right_long) * side_agreement(left_short, right_short);
}

// Ported from Main Tracking.cpp:338-391 (8f93c22e)
void RecomputeGeometry(TrackedObject& object) {
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
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
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
  object.pcl_ClusterPoints.width = static_cast<std::uint32_t>(object.pcl_ClusterPoints.points.size());
  object.pcl_ClusterPoints.height = 1;
  object.pcl_ClusterPoints.is_dense = false;

  obb_util::OBB2D obb;
  double z_min = 0.0;
  double z_max = 0.0;
  std::array<obb_util::P3, 8> corners{};
  obb_util::computeOBB3D_LShape(object.pcl_ClusterPoints, obb, z_min, z_max, corners);
  obb_util::writeOBBToObject(object, obb, z_min, z_max, corners);
}

// Ported from Main FusionClassState.hpp:11-21 (8f93c22e), unlock=false.
void CopyClassState(const TrackedObject& source, TrackedObject& target) {
  target.str_ObjectClass = source.str_ObjectClass;
  target.str_YoloClass = source.str_YoloClass;
  target.last_seen_class = source.last_seen_class;
  target.continuity_counter = source.continuity_counter;
  target.class_miss_counter = source.class_miss_counter;
  target.camera_observation_ns = source.camera_observation_ns;
  target.camera_observation_epoch = source.camera_observation_epoch;
  target.is_class_locked = source.is_class_locked;
}

}  // namespace tracking_association

// Ported from Main Tracking.cpp:395-441 (8f93c22e)
void ConsolidateCurrentDetections(std::vector<TrackedObject>& now_object, const TrackingConfig& st_Tracking) {
  if (now_object.size() < 2) return;

  bool merged = true;
  while (merged) {
    merged = false;
    for (std::size_t left_index = 0; left_index < now_object.size() && !merged; ++left_index) {
      for (std::size_t right_index = left_index + 1; right_index < now_object.size(); ++right_index) {
        TrackedObject& left = now_object[left_index];
        TrackedObject& right = now_object[right_index];
        // OBB centres compare the same occupancy more directly than cluster
        // means, which move with the visible face.
        const float center_distance =
            std::hypot(left.f32_OBB_CenterX - right.f32_OBB_CenterX, left.f32_OBB_CenterY - right.f32_OBB_CenterY);
        if (center_distance > st_Tracking.f32_DuplicateCenterDistance) {
          continue;
        }
        if (tracking_association::AabbIou(left, right) < st_Tracking.f32_DuplicateIouThreshold) {
          continue;
        }

        std::size_t owner_index = left_index;
        std::size_t fragment_index = right_index;
        if (right.pcl_ClusterPoints.size() > left.pcl_ClusterPoints.size()) {
          owner_index = right_index;
          fragment_index = left_index;
        }
        TrackedObject& owner = now_object[owner_index];
        const TrackedObject& fragment = now_object[fragment_index];
        owner.pcl_ClusterPoints.points.insert(owner.pcl_ClusterPoints.points.end(),
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

// Ported from Main Tracking.cpp:446-569 (8f93c22e) (Objectinfo_for_km). The
// unused st_GPSIMU/st_VehicleSpec parameters and the dangling objPoints pointer
// (:519, into a by-value copy) are not ported.
void BuildDetections(std::vector<TrackedObject>& vec_NowObject, const CloudPtr& pcl_Cloud4,
                     const CloudPtr& pcl_Cloud5, const std::vector<pcl::PointIndices>& vec_AllClusterIndices) {
  vec_NowObject.clear();
  int32 s32_Centroid_index = 0;

  for (int32 i = 0; i < static_cast<int32>(vec_AllClusterIndices.size()); i++) {
    const pcl::PointIndices& cluster = vec_AllClusterIndices[i];

    float32 f32_MinX = std::numeric_limits<float32>::max();
    float32 f32_MinY = std::numeric_limits<float32>::max();
    float32 f32_MinZ = std::numeric_limits<float32>::max();
    float32 f32_MaxX = -std::numeric_limits<float32>::max();
    float32 f32_MaxY = -std::numeric_limits<float32>::max();
    float32 f32_MaxZ = -std::numeric_limits<float32>::max();

    TrackedObject st_CurrentObj;

    st_CurrentObj.u16_Idx = static_cast<uint16>(i);
    st_CurrentObj.pcl_ClusterPoints.points.reserve(cluster.indices.size());

    for (int32 j = 0; j < static_cast<int32>(cluster.indices.size()); j++) {
      int32 idx = cluster.indices[j];
      const pcl::PointXYZI& pt = pcl_Cloud4->points[idx];
      st_CurrentObj.pcl_ClusterPoints.points.push_back(pt);

      // Preserved: no NaN guard; x/y by comparison, z by std::min/std::max.
      if (pt.x < f32_MinX) {
        f32_MinX = pt.x;
      }
      if (pt.x > f32_MaxX) {
        f32_MaxX = pt.x;
      }
      if (pt.y < f32_MinY) {
        f32_MinY = pt.y;
      }
      if (pt.y > f32_MaxY) {
        f32_MaxY = pt.y;
      }
      f32_MinZ = std::min(f32_MinZ, pt.z);
      f32_MaxZ = std::max(f32_MaxZ, pt.z);
    }

    if (pcl_Cloud5->points.size() == 0) {
      break;
    }
    // Deviation: Main indexes past the end (undefined) when there are fewer
    // centroids than clusters; Clustering always emits one per cluster.
    if (static_cast<std::size_t>(s32_Centroid_index) >= pcl_Cloud5->points.size()) {
      break;
    }
    pcl::PointXYZI pcl_Centroid = pcl_Cloud5->points[s32_Centroid_index];
    s32_Centroid_index++;
    st_CurrentObj.f32_CoreX = pcl_Centroid.x;
    st_CurrentObj.f32_CoreY = pcl_Centroid.y;
    st_CurrentObj.f32_CoreZ = pcl_Centroid.z;
    // No class from absolute centroid z: the old z <= -1.3 m noise rule hid
    // real low obstacles (Main :523-528).
    obb_util::OBB2D obb2d;
    double zMinD, zMaxD;

    std::array<obb_util::P3, 8> corners3d;
    obb_util::computeOBB3D_LShape(st_CurrentObj.pcl_ClusterPoints, obb2d, zMinD, zMaxD, corners3d);

    obb_util::writeOBBToObject(st_CurrentObj, obb2d, zMinD, zMaxD, corners3d);
    st_CurrentObj.f32_Dist = std::hypot(st_CurrentObj.f32_CoreX, st_CurrentObj.f32_CoreY);
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
    st_CurrentObj.s32_Age = 0;
    st_CurrentObj.s32_MissCount = 0;
    st_CurrentObj.s32_DynamicCount = 0;
    st_CurrentObj.s32_StaticCount = 0;
    st_CurrentObj.s32_KfInitCount = 0;
    st_CurrentObj.Eigen_KfState_CA = Eigen::Matrix<float, 6, 1>::Zero();
    st_CurrentObj.Eigen_KfP_CA = Eigen::Matrix<float, 6, 6>::Identity();
    st_CurrentObj.b_KfInitialized = false;

    vec_NowObject.push_back(st_CurrentObj);
  }
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
