#include <asmc_lidar/geometry/projection.hpp>
#include <stdexcept>

namespace asmc::lidar {
Rot leveling_rotation(double roll, double pitch, const Rot& mount,
                      bool use_roll, bool use_pitch) {
  if (!std::isfinite(roll) || !std::isfinite(pitch) || !rotation_valid(mount))
    throw std::invalid_argument("invalid leveling attitude or mount");
  return mount.transpose() * rpy(use_roll ? roll : 0., use_pitch ? pitch : 0., 0.) * mount;
}
std::optional<Projection> camera_projection(const Context& input,
    const Pose& camera, const Eigen::Matrix3d& k) {
  const auto base = base_from_input(input);
  if (!base || !pose_valid(camera) || !k.allFinite() || k(0,0) <= 0. ||
      k(1,1) <= 0. || std::abs(k.determinant()) < 1e-12) return {};
  Rot optical;
  optical << 0,-1,0, 0,0,-1, 1,0,0;
  return k * optical * (camera.inverse() * *base).matrix().topRows<3>();
}
}  // namespace asmc::lidar
