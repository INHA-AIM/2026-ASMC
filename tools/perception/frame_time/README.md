# C0 frame/time audit

This directory contains offline probes only. It is not a catkin package and is
not installed by an operating launch. `audit_frame_time.py` orchestrates it.

The frozen input/config inventory is
`config/baselines/lidar_frame_time_c0_v1.json`. Inputs are resolved beneath an
explicit storage root; no workstation path is an operating default.

## Boundaries

- `lidar_replay` calls the current `LidarRuntime` in recorded arrival order.
  No wait and no retroactive IMU/GPS consumption: this differs from the live
  worker's bounded wait/latest-frame scheduling.
- The bridge and planning estimators have identically named C++ symbols but
  different definitions. Build/run them in **separate executables**, never link
  them into one process. Both compile the unmodified production sources.
- A planning state whose source stamp differs from the scan is an asynchronous
  policy comparison. It is excluded from same-time pose error statistics.
- The camera timeline uses the current source-clock/session gates and SLERP.
  Its separate capture cannot establish LiDAR/image acquisition synchronization.
- Ground distance 0.36m and GPS forward lever arm 0.35m are different quantities.
- Original calibration sizes are front 1280×720, left/right 640×480.
- Motion-classifier track velocity and qualified planning KF velocity are
  separate contracts; neither zero placeholders nor UNKNOWN imply stationary.

Synthetic projection checks use 1e-8; float scan transforms retain the existing
1e-5m/rad tests. These are numerical consistency limits, not sensor accuracy.
All alternative-pose projections/map relations are sensitivity probes, not a
proposal to replace the operating pose or change filter thresholds.

Record large output only in a newly registered experiment run. Never overwrite
the source bags or completed runs. `/tmp` is for unit/build scratch only.
