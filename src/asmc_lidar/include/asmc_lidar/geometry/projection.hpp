#pragma once
#include <asmc_lidar/core/frame_time.hpp>

namespace asmc::lidar {
using Projection = Eigen::Matrix<double, 3, 4>;
// Remove vehicle roll/pitch in sensor axes. Mount attitude is not IMU attitude.
Rot leveling_rotation(double roll, double pitch, const Rot& base_from_sensor,
                      bool use_roll = true, bool use_pitch = true);
// Input coordinates and the exact preprocessing rotation belong to one scan.
std::optional<Projection> camera_projection(const Context& input,
    const Pose& base_from_camera, const Eigen::Matrix3d& intrinsic);
}  // namespace asmc::lidar
