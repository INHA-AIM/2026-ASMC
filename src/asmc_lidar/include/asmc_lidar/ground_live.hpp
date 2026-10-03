#pragma once

#include <asmc_lidar/core.hpp>
#include <map>

namespace asmc_lidar {
// 0 outside ROI, 1 ground, 2 nonground, 3 unclassified. Structural removal
// is a separate reason, never silently counted as ground classification.
struct GroundPrepared {
  Cloud::Ptr voxel{new Cloud}, leveled{new Cloud};
  std::vector<int> raw_to_voxel;
  std::array<float,9> leveling{};
  double prepare_ms = 0;
};
GroundPrepared PrepareGeneralGround(const Cloud&, const Calibration&, float pitch);
struct GroundMethodResult {
  std::vector<std::uint8_t> labels, structural_removed;
  double copy_ms=0, segment_ms=0, guard_ms=0, restore_ms=0;
};
struct GroundLiveResult {
  GroundPrepared prepared;
  std::map<std::string,GroundMethodResult> methods;
};
class GroundComparison {
 public:
  GroundComparison(std::string method="all", std::string patchwork_variant="guard25");
  ~GroundComparison();
  void Reset();
  GroundLiveResult Process(const Cloud&, const Calibration&, float pitch);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace asmc_lidar
