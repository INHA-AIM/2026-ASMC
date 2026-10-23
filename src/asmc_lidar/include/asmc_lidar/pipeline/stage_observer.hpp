#pragma once
// Read-only observation hook of Pipeline::Process (monitoring only).
//
// The stage clouds (ROI, voxel, leveled, non-ground) exist only inside
// Pipeline::Process. A StageObserver set in PipelineConfig::observer is asked
// once per processed scan (Begin) whether it wants that scan; only then the
// pipeline keeps the pre-tracking detections and the pre-tunnel ground indices
// and calls OnScan with const views of every stage result after the scan was
// fully processed. The observer must copy what it needs during OnScan: every
// reference is invalid afterwards.
//
// The hook never changes an algorithm input or output: with observer == nullptr
// or Begin() == false the pipeline runs exactly as before (verified by
// test/pipeline/test_stage_observer.cpp). OnScan runs on the pipeline thread
// after the result is complete, so a slow observer delays the next scan but
// not this scan's result; observers are expected to copy a bounded sample and
// hand the work to another thread.

#include <asmc_lidar/pipeline/types.hpp>

#include <pcl/PointIndices.h>

#include <cstdint>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

struct ScanInput;
struct ScanResult;

struct StageView {
  const ScanInput& scan;        // raw sensor-frame cloud (scan.points)
  const LocalState& local;      // IMU/CAN values selected for this scan
  const ScanResult& result;     // counts, timings, plane, context, tracks
  const Cloud& roi;             // sensor frame
  const Cloud& voxel;           // sensor frame; leveled[i] = leveling * voxel[i]
  const Cloud& leveled;         // leveled frame, same order as voxel
  // Clustering input after RANSAC (and the tunnel stage when ON). Only the
  // first result.counts.cluster_input_points entries are real points:
  // Clustering appends one (0,0,0) dummy when nothing was clustered.
  const Cloud& cluster_input;
  // leveled/voxel index of cluster_input[i] (after the tunnel stage).
  const std::vector<int>& cluster_input_indices;
  // leveled/voxel index of every RANSAC non-ground point before the tunnel
  // stage (equals cluster_input_indices when the stage is OFF).
  const std::vector<int>& ground_kept_indices;
  const std::vector<pcl::PointIndices>& clusters;  // indices into cluster_input
  const std::vector<TrackedObject>& detections;    // BuildDetections output, leveled frame
};

class StageObserver {
 public:
  virtual ~StageObserver() = default;
  // Called before the stage copies of a processed scan; false = not observed.
  virtual bool Begin(std::int64_t stamp_ns) = 0;
  virtual void OnScan(const StageView& view) = 0;
};

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
