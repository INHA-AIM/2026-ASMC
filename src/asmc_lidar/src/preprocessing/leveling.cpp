// Roll/pitch leveling of the voxel cloud.
// Ported from Main src/Lidar/ImuRansac/ImuFusion.cpp at 8f93c22e, common-geometry branch.

#include <asmc_lidar/preprocessing/preprocessing.hpp>
#include <asmc_lidar/geometry/projection.hpp>

#include <Eigen/Core>

namespace asmc {
namespace lidar {
namespace pipeline {

// Ported from Main src/Lidar/ImuRansac/ImuFusion.cpp:43-77 (8f93c22e)
// Main takes roll/pitch from st_GPSIMU.st_IMU, the mount from SensorMount(st_VehicleSpec)
// and use_roll from frame_boundary::UseRoll(); Main's StoreLeveling(exact) is the
// caller's job (the returned matrix).
asmc::lidar::Rot LevelPoints(const CloudPtr& input, const CloudPtr& output, float32 roll_rad,
                             float32 pitch_rad, const asmc::lidar::Rot& mount_rotation,
                             bool use_roll) {
  // Legacy float Ry(pitch) matrix (enable_common_geometry=false) is not ported.
  // Throws std::invalid_argument before the output is touched, as in Main.
  const asmc::lidar::Rot exact =
      asmc::lidar::leveling_rotation(roll_rad, pitch_rad, mount_rotation, use_roll, true);
  // Main clears only the point vector: width/height/is_dense are never set here.
  output->points.clear();

  for (const auto& point : input->points) {
    Eigen::Vector3f pt(point.x, point.y, point.z);
    Eigen::Vector3f corrected = (exact * pt.cast<double>()).cast<float>().eval();

    pcl::PointXYZI pcl_NewPoint;
    pcl_NewPoint.x = corrected.x();
    pcl_NewPoint.y = corrected.y();
    pcl_NewPoint.z = corrected.z();
    pcl_NewPoint.intensity = point.intensity;

    output->points.push_back(pcl_NewPoint);
  }
  return exact;
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
