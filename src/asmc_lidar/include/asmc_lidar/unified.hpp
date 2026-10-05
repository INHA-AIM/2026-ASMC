#pragma once

#include <asmc_lidar/ground_live.hpp>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace asmc_lidar {
// All public geometry is rear-axle base_link. A detection_index is only valid
// inside one observation; it is deliberately not a persistent track ID.
struct FrameDetection {
  std::size_t detection_index=0, voxel_points=0, raw_points=0;
  Eigen::Vector3d center_base=Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d,8> corners_base{};
  Eigen::Vector3d forward_base=Eigen::Vector3d::UnitX();
  double yaw_base_rad=0;
  std::vector<std::size_t> source_indices;
  std::string object_class="unknown", velocity_state="unknown";
  std::string relevance="unknown";
};
struct StructureEvidence {
  std::size_t support_voxels=0, support_raw_points=0;
  Eigen::Vector3d min_base=Eigen::Vector3d::Zero(), max_base=Eigen::Vector3d::Zero();
  // Rejected RANSAC surfaces are only hypotheses, not a wall/guardrail label.
  std::string quality="unconfirmed_plane_support";
};
struct OptionalContext {
  std::string source, frame_id;
  std::int64_t observation_ns=0;
  bool valid=false;
  double quality=0;
  // A missing/invalid/stale context is unknown, never a deletion instruction.
};
struct UnifiedResult {
  std::int64_t observation_ns=0;
  std::string frame_id="base_link", status="valid";
  std::string map_context_status="unknown", camera_context_status="unknown";
  std::vector<FrameDetection> detections;
  std::vector<StructureEvidence> structure_evidence;
  std::vector<std::size_t> retained_raw_indices, structure_candidate_raw_indices;
  std::size_t ground_raw_points=0, unknown_raw_points=0;
  double clustering_ms=0, obb_ms=0;
};
bool ContextUsable(const OptionalContext&,std::int64_t observation_ns,
                   std::int64_t max_age_ns=100000000);
UnifiedResult BuildUnified(const Cloud& raw,const GroundLiveResult& ground,
                           const Calibration& calibration,std::int64_t observation_ns,
                           const OptionalContext* map_context=nullptr,
                           const OptionalContext* camera_context=nullptr);
}  // namespace asmc_lidar
