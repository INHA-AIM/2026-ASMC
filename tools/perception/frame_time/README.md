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

## Build and small regressions

Run from the repository root in the supported ROS Noetic environment after the
normal workspace build (including the required LiDAR pipeline):

```bash
source devel/setup.bash
cmake -S tools/perception/frame_time -B /tmp/asmc-c0-probes \
  -DASMC_C0_RUNTIME_LIB_DIR="$PWD/devel/lib" -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/asmc-c0-probes -j2
C0_PROBES=/tmp/asmc-c0-probes python3 tools/perception/test_frame_time_audit.py
python3 tools/perception/frame_time/test_state_probes.py --probes /tmp/asmc-c0-probes
```

The last fixture intentionally **reproduces the current bridge GPS-zero defect**:
planning zero-GPS valid count0, bridge count9. It is evidence, not an endorsement
of that behavior. Update the assertion when the separate production fix lands.
CI runs these small regressions and compiles the support probe; it does not fetch
private recordings or run MORAI. Eigen, YAML, OpenCV, OpenSSL, Zlib, PCL and
GeographicLib are the existing ROS/LiDAR build dependencies.

## Registered full replay

Resolve and register a fresh run using the parent workspace storage tools first.
Pass that exact run ID and its resolved output; the directory must not exist yet.
In the container, use its mounted storage paths, not the host `.storage/env.sh`.

```bash
python3 tools/perception/audit_frame_time.py \
  --manifest config/baselines/lidar_frame_time_c0_v1.json \
  --storage-root '/mnt/d/morai dataset/shahn03' \
  --run-id '<registered-new-run-id>' --output '/artifacts/<registered-new-run-id>' \
  --probes /tmp/asmc-c0-probes --replay "$PWD/devel/lib/asmc_lidar/lidar_replay"
```

This is read-only with respect to inputs and operating ROS. It needs no master.
It rejects changed source/config/input hashes; do not replace pinned hashes to
conceal a different baseline. A new baseline requires a new manifest/run.

- `summary.json`: provenance, native binary SHA, parity, pose/time distributions,
  support sensitivity and qualified velocity identity/validity.
- `normal/`, `tunnel/`: arrival-ordered native replay and both state probes.
- `support/`: full observed voxel-cluster support through current `HdMapAudit`
  and `ApplyDeliveryPreview`, without altering the runtime's output arrays.
- `gallery.html`: RGB corner projection, local/map comparison and map boundary.
  Delivered-object gallery and all-observed support analysis have different populations.
- `index.html`: compact summary. Private large artifacts stay in registered storage.

`summary.passed` means audit consistency, not full autonomy or sensor accuracy
acceptance. Asynchronous pose differences, unknown clocks and unresolved failures
remain visible even if consistency passes. Close the registered run when writers
have finished; preserve failed runs as well.

## Optional live metadata analysis

Live driving is opt-in and requires a separate registered run and user authorization.
C0's live trial is retained as an exact script/artifact in that run; this package
adds no default driving launcher. `summarize_live.py` only reads a metadata bag:

```bash
python3 tools/perception/frame_time/summarize_live.py \
  --bag '/artifacts/<live-run-id>/timing.bag' \
  --output '/artifacts/<live-run-id>/analysis.json'
```

Supply statistics are subscriber arrival times, not pure algorithm latency.
Different device and camera clocks are never subtracted without a proven mapping.
See [2026-10-26 results](../../../docs/records/lidar/2026-10-26-frame-time-baseline.md)
for the actual inputs, live-test limitations and the next production fix.
