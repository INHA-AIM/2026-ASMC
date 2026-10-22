// Experimental TunnelFilterCore connection stage (default OFF).
//
// New connection code, not a port: at 8f93c22e Main builds TunnelFilterCore only
// as the EXCLUDE_FROM_ALL test library tunnel_filter_core (Main CMakeLists.txt:100-103)
// and no Main source calls it. The core below is the byte-identical PR #102 copy.
// Not used on purpose: tunnel_filter::IsGpsDenied/ShouldRunExperiment (GPS gate),
// the core's dynamic_roi_* fields (GPS-only ROI reduction, never read by the core),
// camera protection masks and ego motion.

#include <asmc_lidar/pipeline/tunnel_filter_stage.hpp>

#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace asmc {
namespace lidar {
namespace pipeline {

// The core keeps state across scans (ground reuse window, 3-frame wall
// persistence), so one instance lives as long as the stage.
struct TunnelFilterStage::Impl {
  // Core defaults unchanged: no new thresholds or tuning.
  tunnel_filter::TunnelPipeline core{tunnel_filter::Params()};
};

TunnelFilterStage::TunnelFilterStage(const TunnelFilterStageConfig& config) : config_(config) {
  // OFF: the core is not constructed and Apply() returns before touching anything.
  if (config_.enabled) impl_ = std::make_unique<Impl>();
}

TunnelFilterStage::~TunnelFilterStage() = default;

TunnelFilterStageReport TunnelFilterStage::Apply(const CloudPtr& voxel, float32 roll_rad,
                                                 float32 pitch_rad, const CloudPtr& ground_output,
                                                 std::vector<int>& ground_kept_indices) {
  TunnelFilterStageReport report;
  if (!impl_) return report;  // OFF bypass: ran=false, inputs untouched.

  // Reject inconsistent inputs before the core state advances.
  if (!voxel || !ground_output) throw std::invalid_argument("tunnel stage: null cloud");
  if (ground_kept_indices.size() != ground_output->points.size())
    throw std::invalid_argument("tunnel stage: ground indices do not match the ground output");
  const std::size_t n = voxel->points.size();
  for (const int index : ground_kept_indices)
    if (index < 0 || static_cast<std::size_t>(index) >= n)
      throw std::invalid_argument("tunnel stage: ground index outside the voxel cloud");

  // Sensor-frame input (metres), scan roll/pitch; camera protection unavailable
  // (no protect/ambiguous masks: fail-open), no ego motion.
  const tunnel_filter::FrameResult& result =
      impl_->core.Process(*voxel, roll_rad, pitch_rad, nullptr, nullptr, false, nullptr);
  report.ran = true;
  report.input_points = result.input_points;
  report.core_ms = result.timings.total_ms;
  // labels[i] belongs to voxel->points[i] (the core keeps input order).
  for (const std::uint8_t label : result.labels) {
    if (label == tunnel_filter::kCeiling) ++report.ceiling_labeled;
    else if (label == tunnel_filter::kWall) ++report.wall_labeled;
    else if (label == tunnel_filter::kGround) ++report.ground_labeled;
  }

  // Remove by voxel index; kGround/kKept/kInvalid stay (common RANSAC is
  // authoritative for ground). Order of the survivors is preserved.
  std::size_t out = 0;
  for (std::size_t k = 0; k < ground_kept_indices.size(); ++k) {
    const std::uint8_t label = result.labels[static_cast<std::size_t>(ground_kept_indices[k])];
    if ((config_.suppress_ceiling && label == tunnel_filter::kCeiling) ||
        (config_.suppress_wall && label == tunnel_filter::kWall))
      continue;
    if (out != k) {
      ground_output->points[out] = ground_output->points[k];
      ground_kept_indices[out] = ground_kept_indices[k];
    }
    ++out;
  }
  report.removed_points = static_cast<int>(ground_kept_indices.size() - out);
  if (report.removed_points > 0) {
    ground_output->points.resize(out);
    ground_kept_indices.resize(out);
    ground_output->width = static_cast<std::uint32_t>(out);
    ground_output->height = 1;
  }
  return report;
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
