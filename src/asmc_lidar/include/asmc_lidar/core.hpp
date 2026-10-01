#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <frame_time/core.hpp> // L1-B1 frame/time contract, shared implementation.
#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>

namespace asmc_lidar {
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

enum class Mode { Reference, GroundReview };
enum class ReferenceProfile { Tunnel, General };
enum class PointClass : std::uint8_t { Unknown = 0, Ground = 1, Nonground = 2 };

struct Calibration {
  Eigen::Isometry3d base_from_sensor = Eigen::Isometry3d::Identity();
  std::string source;
};

struct FrameInput {
  std::int64_t stamp_ns = 0;
  std::int64_t receipt_ns = 0;
  std::string clock = "ros";
  std::string stamp_provenance = "bridge_receive";
  Cloud::ConstPtr cloud;
  double roll_rad = 0.0, pitch_rad = 0.0;
  bool imu_valid = false;
  ReferenceProfile profile = ReferenceProfile::Tunnel;
  bool allow_repeated_snapshot = false; // offline stationary warmup only
  bool already_preprocessed = false; // only for frozen offline voxel snapshots
  bool camera_available = false;     // missing is distinct from valid empty
  std::vector<std::uint8_t> protected_classes;
  std::vector<std::uint8_t> ambiguous;
  tunnel_filter::EgoMotion motion;
};

struct GroundResult {
  std::vector<PointClass> by_original;
  std::vector<int> roi_to_original;
  Cloud::Ptr ground{new Cloud}, nonground{new Cloud}, uncertain{new Cloud};
  double input_copy_ms = 0.0, segmentation_ms = 0.0, restore_ms = 0.0;
  std::string reason;
};

struct FrameOutput {
  std::string status;
  std::int64_t stamp_ns = 0;
  std::string camera_state;
  Cloud::Ptr input_base{new Cloud}, ground_base{new Cloud}, nonground_base{new Cloud}, uncertain_base{new Cloud};
  std::vector<int> raw_to_voxel;
  std::string voxel_fnv1a64;
  std::vector<std::uint8_t> reference_labels;
  std::vector<tunnel_filter::TunnelObject> reference_objects_base;
  GroundResult patchwork;
  std::size_t raw_points = 0, roi_points = 0, voxel_points = 0;
  double preprocessing_ms = 0.0, reference_ms = 0.0;
};

class Processor {
 public:
  Processor(Mode mode, Calibration calibration, tunnel_filter::Params reference_params,
            double sensor_height_m = 1.80);
  ~Processor();
  FrameOutput Process(const FrameInput& input);
  void Reset();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Exact frame-by-frame L-shape search parameters from Son 6435af1 Tracking.cpp.
tunnel_filter::ObbFunction PinnedLShapeObb();
tunnel_filter::Params LoadReferenceParams(const std::string& yaml_path);
Calibration LoadCalibration(const std::string& yaml_path);
}  // namespace asmc_lidar
