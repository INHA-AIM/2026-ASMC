# V4 ground candidate experiment

Research-only; preserve V1/V3 defaults and the nested operating Main package.
[Decision and limits](../../../docs/packages/asmc_lidar/l2a2-v4-ground-selection-20261005.md).
Configurations are in src/asmc_lidar/config/ground_v4. All use roll_pitch.

Inside the Noetic container, with a built /root/ws and /data mount:

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash

# New output directory; never overwrite frozen artifacts.
python3 tools/lidar/ground_v4/run.py \
  /data/lidar_assessment/20261005_ground_v3_drive/tunnel_gt \
  /data/lidar_assessment/ground_v4_rerun/tunnel \
  --profiles jaeho_height jaeho_support patchwork_support

python3 tools/lidar/ground_v4/score.py \
  /data/lidar_assessment/20261005_ground_v3_drive/tunnel_gt \
  /data/lidar_assessment/20261005_ground_v3_drive/tunnel_v3/roll_pitch_all.jsonl \
  /data/lidar_assessment/ground_v4_rerun/tunnel \
  /data/lidar_assessment/ground_v4_rerun/tunnel_score \
  --jaeho-profile jaeho_support

python3 tools/lidar/ground_v4/mask_diagnostics.py \
  /data/lidar_assessment/20261005_ground_v3_drive/tunnel_gt \
  /data/lidar_assessment/20261005_ground_v3_drive/tunnel_v3/roll_pitch_all.jsonl \
  /data/lidar_assessment/ground_v4_rerun/tunnel \
  /data/lidar_assessment/ground_v4_rerun/tunnel_changes.json

cd build/asmc_lidar
ctest --output-on-failure
```

Repeat with the same input stream and use ground_live_review/verify_repeat.py.
The scorer keeps the historical key `jaeho_height`; top-level `jaeho_profile`
identifies whether it actually evaluated height_gate or height_support. Do not
merge those two hypotheses. New drives have no actor GT: actor gates use raw/sync.

plot.py INPUT BASELINE CANDIDATE INPUT_ID OUTPUT.png produces same-scan sensor
XYZ BEV/side panels. ROS publications remain rear-axle base_link.

runtime.py BAG NEW_OUTPUT creates a separate master on port11431, plays only
/imu and /velodyne_points, never control commands. It evaluates single methods
for31s at rate1.4, start110s of the frozen tunnel bag. It stops only its own
processes and does not start/stop the user's bridge.

closeout.py DATA_ROOT NEW_OUTPUT recomputes gates from the frozen
lidar_assessment/20261005_ground_v4 results, fingerprints every cloud/label file,
and records final source/binary hashes. Keep its original artifacts unchanged;
a new algorithm experiment needs a new dated result tree and gate review.

No scan has access to GT in the candidate implementations. Graph metrics are
independent of object clustering. This experiment changes neither tracker nor
planner topics.
