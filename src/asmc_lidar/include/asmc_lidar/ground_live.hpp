#pragma once

#include <asmc_lidar/core.hpp>
#include <asmc_lidar/general.hpp>
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
GroundPrepared PrepareGeneralGround(const Cloud&, const Calibration&, float roll, float pitch,
                                    bool level_roll);
struct GroundMethodResult {
  std::vector<std::uint8_t> labels, structural_removed;
  std::vector<std::uint8_t> pre_guard_labels;
  std::vector<unsigned> reasons;
  std::map<std::string, unsigned> events;
  std::vector<GroundPlaneAttempt> planes;
  double copy_ms=0, segment_ms=0, guard_ms=0, restore_ms=0;
};
// Review profiles only. Empty path preserves V1 defaults, including tail behavior.
struct GroundComparisonConfig {
  std::string id="v1", sha256;
  std::string leveling_mode="pitch_only";
  std::string jaeho_ground_policy="legacy", patchwork_guard_policy="horizontal";
  bool linefit_cover_all=false;
  int linefit_threads=4, patchwork_min_points=10;
  double patchwork_distance=.125, patchwork_seeds=.125;
  double linefit_distance=.05, linefit_search=.1, linefit_long=2.;
};
GroundComparisonConfig LoadGroundComparisonConfig(const std::string& path="");
struct GroundLiveResult {
  GroundPrepared prepared;
  std::map<std::string,GroundMethodResult> methods;
};
class GroundComparison {
 public:
  GroundComparison(std::string method="all", std::string patchwork_variant="guard25",
                   GroundComparisonConfig config=LoadGroundComparisonConfig());
  ~GroundComparison();
  void Reset();
  GroundLiveResult Process(const Cloud&, const Calibration&, float pitch);
  GroundLiveResult Process(const Cloud&, const Calibration&, float roll, float pitch);
  GroundLiveResult ProcessPrepared(const GroundPrepared&, std::size_t raw_size);
  const GroundComparisonConfig& config() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace asmc_lidar
