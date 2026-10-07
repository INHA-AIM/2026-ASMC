# Frame-boundary implementation evidence

This is CPU/ROS implementation evidence, not completed bag, GPU, object-preservation
or performance acceptance. Both runtime candidates remain opt-in.

- `build-and-test-attempts.tar.gz`: all 20 temporary build/test logs and 69 final
  XML result files. Earlier compiler, startup and test failures remain in the logs;
  XML files describe the last execution of each test.
- `manifest.json`: SHA256 of each file in that archive.
- `binary-sha.json`: the tested Main executable, Main shared library and outer
  `asmc_lidar_frame_time` library. Builds use temporary directories, not shared devel.
- `host-audit-attempts`: validators and pure measurement tests. `06.log` records a
  host import-path error; `07-corrected-library-path.log` records the same tests
  passing with the camera library explicitly on PYTHONPATH. It is not a code fix.
- `frame-boundary-height-audit.json`: read-only primary-bag GPS/IMU/Ego height
  comparison. It does not establish a universal altitude datum.
- `fixture-manifest.json`: unchanged B1a sensor/GPS variants and B0 frozen YOLO
  inputs, with their SHA256 and availability policy.
- `frame-boundary-host-state.json`: successful Windows process query that found
  Simulator running. No frame-boundary playback/workload trial was started.

The corrected final core run passes 3 CTest targets. The new ROS suite passes 9
cases; Main's aggregate catkin result is 286 tests with no failures. The outer
aggregate is 533 tests with the known facade failure. The unrelated unchanged
`behavior_planner/fsm_command_binding_test` link error is separately retained in
`boundary-outer-tests-build.log`; a complete all-package test-target build has not
passed.

New measurement tools have parser/import and pure protocol checks, but their real
playback paths remain unvalidated until Simulator is stopped. Synthetic continuous
Kalman GPS contrast is narrower than deterministic raw-LiDAR pipeline replay.

See [baseline](../../perception_lidar_camera_frame_boundaries_v1.yaml) and
[runtime handoff](../../../../docs/handoffs/perception/lidar-camera-frame-boundaries.md).
