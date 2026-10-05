#pragma once

#include <asmc_lidar/unified.hpp>
#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

namespace asmc_lidar {
struct CameraBox {
  double xmin=0,ymin=0,xmax=0,ymax=0;
  std::string label;
};
struct CameraObservation {
  std::int64_t stamp_ns=0;
  std::string frame_id;
  bool valid=false;
  unsigned width=0,height=0;
  std::vector<CameraBox> boxes;
};
struct EgoTwist {
  bool valid=false;
  Eigen::Vector3d linear_base=Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_base=Eigen::Vector3d::Zero();
};
struct TrackedDetection {
  FrameDetection geometry;
  int id=-1, age=0, miss_count=0;
  bool observed=false, relative_velocity_valid=false, absolute_velocity_valid=false;
  bool dynamic_valid=false, is_dynamic=false;
  bool yaw_rate_valid=false;
  double yaw_rate_rad_s=0.0;
  Eigen::Vector3d relative_velocity_base=Eigen::Vector3d::Zero();
  Eigen::Vector3d absolute_velocity_base=Eigen::Vector3d::Zero();
  std::string classification="unknown",camera_class="unknown";
};
// One state for normal road, tunnel and recovery. Only a scan clock reset
// resets tracking; GPS and map-pose availability do not enter this core.
class TrackingEngine {
 public:
  std::vector<TrackedDetection> Update(const UnifiedResult& scan,double ego_yaw_rad,
      const EgoTwist& twist=EgoTwist{},const CameraObservation* camera=nullptr);
  void Reset();
 private:
  struct Track {
    TrackedDetection output;
    Eigen::Vector4d state=Eigen::Vector4d::Zero();
    Eigen::Matrix4d covariance=Eigen::Matrix4d::Identity();
    std::string pending_class="unknown";
    int class_confirmations=0,class_misses=0;
    std::int64_t last_camera_stamp_ns=0;
  };
  std::vector<Track> tracks_;
  std::int64_t last_stamp_ns_=0;
  double last_yaw_rad_=0;
  int next_id_=1;
};
}  // namespace asmc_lidar
