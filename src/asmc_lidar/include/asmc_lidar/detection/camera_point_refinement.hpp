#pragma once
#include <pcl/PointIndices.h>
#include <asmc_lidar/core/types.hpp>
#include <asmc_lidar/core/frame_time.hpp>
#include <asmc_lidar/detection/observation.hpp>
#include <asmc_lidar/experimental/tunnel_filter_stage.hpp>

namespace asmc::lidar::pipeline {
struct TunnelCameraInput;
struct CameraPointRefinementConfig {
  bool enabled=false;
  // Independent opt-in: join complete sparse steel parents, never all points
  // in a detector rectangle. Defaults preserve existing configurations.
  bool steel_fragment_merge=false;
  float steel_minimum_confidence=.5f, steel_maximum_gap_m=2.8f;
  float steel_minimum_ownership=.8f;
  float minimum_confidence=.7f;
  float max_age_s=.15f;
  float connection_m=.35f, angular_connection=.01f, maximum_connection_m=.65f;
  float ambiguity_depth_m=.75f;
  float fragment_connection_m=1.f;
  float wall_band_m=.15f;
  // Vehicle-only sparse scan ribbons are grouped BEFORE surface validation.
  // This is a bounded angular envelope, not a global clustering tolerance.
  bool vehicle_scanline_grouping=true, independent_wall_cleanup=true;
  bool detached_wall_cleanup=false;
  float scanline_gap_slope=.035f, scanline_max_gap_m=1.25f;
  int minimum_points=5, maximum_box_points=8192, maximum_boxes=32;
};
void ValidateCameraPointRefinement(const CameraPointRefinementConfig&);
struct CameraPointDecision {
  int box=-1, input_points=0, object_points=0, wall_points=0, components=0;
  int scanline_groups=0;
  float depth_m=0;
  std::string source_class,reason;
};
struct CameraPointRefinementReport {
  std::int64_t image_stamp_ns=0;
  std::array<double,12> projection{};  // row-major; CURRENT leveled point -> image homogeneous
  bool enabled=false, applied=false;
  std::string reason;
  int projected_points=0, extracted_objects=0, removed_wall_points=0;
  double ms=0;
  std::vector<CameraPointDecision> decisions;
};
struct CameraPointRefinementResult {
  std::vector<pcl::PointIndices> clusters;
  // Original cluster index if unambiguous, -1 for new/multiple parents, -2 for
  // residuals. support.box_index identifies extracted camera-supported objects.
  std::vector<int> parent_clusters;
  std::vector<CameraPointSupport> support;
  std::vector<int> removed_wall_indices;
  CameraPointRefinementReport report;
};
// Produces a plan, never edits source points or past tracking state. Residual
// non-wall support remains as independent LiDAR obstacles. All point indices
// refer to the supplied CURRENT scan in context.frame.
CameraPointRefinementResult RefineCameraPoints(
    const CloudPtr&,const std::vector<pcl::PointIndices>&,
    const asmc::lidar::Context&,const TunnelCameraInput*,
    const std::vector<TunnelWallEvidence>&,const CameraPointRefinementConfig&);
struct SteelFragmentHistory {
  CloudPtr points;  // Previous measured support, motion-aligned to THIS scan.
  CameraPointSupport support;  // Original image time is never refreshed.
};
CameraPointRefinementResult ContinueMeasuredSteelFragments(
    const CloudPtr&,const std::vector<pcl::PointIndices>&,
    const std::vector<SteelFragmentHistory>&,const asmc::lidar::Context&,
    const TunnelCameraInput*,const std::vector<TunnelWallEvidence>&,
    const CameraPointRefinementConfig&,const std::vector<CameraPointSupport>* current_support=nullptr);
}
