# Frame-boundary implementation evidence

This is CPU/ROS implementation and 78-trial frozen-input bag evidence, not completed
GPU, object-preservation or operational acceptance. Both runtime candidates remain opt-in.

- `build-and-test-attempts.tar.gz`: all 20 temporary build/test logs and 69 final
  XML result files. Earlier compiler, startup and test failures remain in the logs;
  XML files describe the last execution of each test.
- `manifest.json`: SHA256 of each file in that archive.
- `binary-sha.json`: the tested Main executable, Main shared library and outer
  `asmc_lidar_frame_time` library at the initial implementation revision. Builds
  use temporary directories, not shared devel.
- `parser-order-final.tar.gz`, `parser-order-manifest.json`, and
  `parser-order-binary-sha.json`: two subsequent successful runs and final Main
  XML/binaries after rejecting invalid scan stamps and atomically storing source
  buffer/generation. The earlier archive and binary hashes remain unchanged.
- `host-audit-attempts`: validators and pure measurement tests. `06.log` records a
  host import-path error; `07-corrected-library-path.log` records the same tests
  passing with the camera library explicitly on PYTHONPATH. It is not a code fix.
- `frame-boundary-height-audit.json`: read-only primary-bag GPS/IMU/Ego height
  comparison. It does not establish a universal altitude datum.
- `fixture-manifest.json`: unchanged B1a sensor/GPS variants and B0 frozen YOLO
  inputs, with their SHA256 and availability policy.
- `frame-boundary-host-state.json`: successful Windows process query that found
  Simulator running. No frame-boundary playback/workload trial was started.

The corrected final core run passes 3 CTest targets. The final new ROS suite passes
10 cases; Main's aggregate catkin result is 288 tests with no failures (the initial
revision had 9 new ROS cases and 286 aggregate tests). The outer
aggregate is 533 tests with the known facade failure. The unrelated unchanged
`behavior_planner/fsm_command_binding_test` link error is separately retained in
`boundary-outer-tests-build.log`; a complete all-package test-target build has not
passed.

The user stopped Simulator before all fixed trials. Synthetic continuous Kalman
GPS contrast is narrower than deterministic raw-LiDAR pipeline replay.

- `fixed-matrix-{state,cost}-*.tar.gz`: all 78 trial folders, original observations,
  logs, reports, launch arguments, parameters and 200ms resource records. State
  archives are split by bag/repetition; cost archives include all three repetitions.
  Execution completion does not waive strict comparison failures.
- `fixed-matrix-{index,manifest,protocol}.json`: complete sequence, source/config/
  binary/fixture hashes and frozen-YOLO availability. Every recorded runtime source
  SHA was rechecked after all 78 trials and remained unchanged.
- `fixed-matrix-analysis.json`: all reports and all changed/missing source stamps;
  24 GPS contrast pairs fail strict local payload equality. Map observer gate has
  no reported violations, but observer/Main receipt boundaries still need review.
- `fixed-matrix-summary.json`: repeat-level resource/latency and cumulative final
  frame totals. Timer counters are partial and are not used as final totals.
- `fixed-matrix-first-repeat-identity.json`: first primary/pitch common-center
  review at 1mm tolerance, retaining unmatched observations and normal-repeat class
  differences. It is not independent object GT or an acceptance waiver.
- `fixed-matrix-frame-boundary-host-state.{json,jsonl}`: successful final Simulator
  check and full measurement-time query history. The older initial running-state
  file remains unchanged; only the measurement-specific monitor was stopped.
- `fixed-matrix-evidence-sha.json`: SHA256 for every fixed-matrix evidence file.
- `fusion-coordinate-audit.json`: six pinned reference repositories and 54 actual
  camera-Python/LiDAR-C++ synthetic projection cases; physical/time calibration
  acceptance is not implied.
- `frame-pose-audit-{primary,shadow}.json`: read-only local/map corner rigidity and
  causal Ego pose-convention audit, with explicit assumptions and unavailable data.

Remaining blockers include original-ns/actual-callback-receipt capture, signed ego
motion semantics, full raw-pipeline reproducibility, independent object-preservation
and normal map tracking review, actual GPU fusion, and direct user launch review.

See [baseline](../../perception_lidar_camera_frame_boundaries_v1.yaml) and
[runtime handoff](../../../../docs/handoffs/perception/lidar-camera-frame-boundaries.md).
