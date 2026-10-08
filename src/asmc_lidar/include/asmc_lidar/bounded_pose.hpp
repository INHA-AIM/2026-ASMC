#pragma once
#include <asmc_lidar/frame_time.hpp>

namespace asmc::lidar {
// Explicit normal-GPS prediction policy. PoseHistory remains interpolation-only.
class BoundedPoseHistory {
 public:
  explicit BoundedPoseHistory(Ns max_age_ns = 300000000, unsigned recovery_samples = 3);
  std::string append(const PoseSample& sample);
  PoseSelection at(const Observation& observation, const Rot& map_from_base_rotation,
                   const Vec& ego_linear_base) const;
  void clear();
 private:
  Ns max_age_;
  unsigned recovery_samples_;
  std::vector<PoseSample> samples_;
};
}  // namespace asmc::lidar
