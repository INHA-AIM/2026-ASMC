# Live camera environment check — 2026-10-07

Tested implementation: `602dc2f`, shared container `asmc-ros-noetic`, current
`asmc-camera-objects` worktree at `/root/ws`, RTX 4060 Ti, host networking,
ROS master `http://127.0.0.1:11311`, `ROS_HOSTNAME=localhost`, `use_sim_time=false`.
The user explicitly approved starting the bridge for this diagnosis. No vehicle
control, teleport, scenario load, simulation resume/pause or GUI window was issued.

Initially master and bridge were stopped. Container workspace/model/GPU/X11
mounts were already correct; a CUDA kernel and Qt connection both succeeded.
Local ROS hostname was added to the running container's interactive shell and
compose environment. The old GPS warning's sensor values were not recorded, so
this check does not establish its original cause or change GPS validity rules.

The bridge was started using `asmc_bridge bridge.launch` with local transport and
competition streams. Raw inputs were observed for 12 seconds: valid GPS status 1
at 5.08 Hz, IMU at 47.12 Hz with quaternion norm approximately 1, Ego at 49.04 Hz,
valid `/vehicle/state` at 50.04 Hz, RGB front/left/right each at 19.98 Hz, camera4
at 9.99 Hz, LiDAR at 13.32 Hz. Topic publishers were unique.

`perception.launch profile:=debug` and `Main main.launch start_yolo:=false
rviz:=false` then ran together. During the recorded 20-second observation:

| Output | Count | Observed Hz | Decodable image |
|---|---:|---:|---|
| Camera objects / independent YOLO overlay | 398 each | 19.88 | yes, 1280×720 |
| Traffic-light overlay | 199 | 9.94 | yes, 640×480 |
| Three-view mosaic | 131 | 6.54 | yes, 1920×1440 |
| BEV comparison | 100 | 5.00 | yes, 1344×1050 |
| Fusion debug | 208 | 10.39 | raw image, 1280×720 |

All three models were loaded, warmed, ACTIVE and prepared with zero inference
errors. TwinLite batch rate was 9.99 Hz, ResNet 9.96 Hz, YOLO 19.54 Hz at the last
status sample. Debug image publication rates are distinct from inference rates.
`/YOLO/BBox` had only `/camera_objects_yolo_bbox_adapter` as publisher; `/ctrl_cmd`
had no publisher. Three recorded fusion windows had matched=56/55/55, dropped=0,
offset p95=26.7/29.4/29.3 ms. No GPS/vehicle-state/pose-synchronization error occurred
during this window. One startup LiDAR scan lacked speed before the initial
callback; this did not continue after initialization.

Lane inference succeeded, but `LaneBoundaryFamily.valid=false`,
`status_reason=too_few_observed_roles`, no boundaries, `deployment_allowed=false`.
Consequently TwinLite `pipeline_ready=false` is retained. This is not evidence
of a valid lane handoff or permission to drive. Drivable-boundary filtering and
ego-hood filtering warnings were preserved; their policies were not changed.

This is a live input/output smoke check, not the repeated performance acceptance
or shadow-driving trial. Previous strict acceptance failures remain in force.
After diagnosis, only the two owned camera/fusion launches were stopped normally.
Bridge/master were left running for the user's direct launch/GUI commands.

| Raw artifact | SHA-256 |
|---|---|
| `inputs.json` | `3e76511a1472a9f4de7db6ce0ac021a467b6b2e22aadb7783216ffb890e3acae` |
| `outputs.json` | `4b94e0464ebff510da41b9e29d53153876365ada89472c288eed469d61d16275` |
| `lane.json` | `7088d5fe5875d8c09ee4f62a1364971674eafe67323fb1b1117989fa8e4c0751` |
